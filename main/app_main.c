/*
 *
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <hap_fw_upgrade.h>

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include "relays.h"

#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"

/* Comment out the below line to disable Firmware Upgrades */
#define CONFIG_FIRMWARE_SERVICE

static const char *TAG = "HAP lightbulb";
static const char *TAGSENSOR = "HAP lightsensor";

#define GARAGEDOOR_TASK_PRIORITY  1
#define GARAGEDOOR_TASK_STACKSIZE 4 * 1024
#define GARAGEDOOR_TASK_NAME      "hap_garagedooropener"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        2

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0
#define LED GPIO_NUM_2

#define INPUT_MD_OPEN GPIO_NUM_12       // Motor nyitóirányú feszültség érzékelés, optocsatolós bemenet
#define INPUT_MD_CLOSE GPIO_NUM_13      // Motor záróirányú feszültség érzékelés, optocsatolós bemenet
#define INPUT_LS_OPEN GPIO_NUM_27       // Végállás kapcsoló nyitóirányú feszültség érzékelés, optocsatolós bemenet
#define INPUT_LS_CLOSE GPIO_NUM_14      // Végállás kapcsoló záróirányú feszültség érzékelés, optocsatolós bemenet

//#define ADC_MD_OPEN                 ADC1_CHANNEL_6      // Motor nyitás áram érzékelés, analog bemenet ( GPIO34 )
//#define ADC_MD_CLOSE                ADC1_CHANNEL_4      // Motor zárás áram  érzékelés, analog bemenet ( GPIO32 )
//#define ADC_LS_OPEN                 ADC1_CHANNEL_0      // Végállás kapcsoló nyitva állás érzékelés, analog bemenet ( GPIO36 )
//#define ADC_LS_CLOSE                ADC1_CHANNEL_3      // Végállás kapcsoló zárva állás érzékelés, analog bemenet ( GPIO39 )

//ADC Attenuation
//#define ADC_ATTEN           ADC_ATTEN_DB_11

#define MOTOR_DIRECTION_UPCOUNTER_LIMIT     3
#define MOTOR_DIRECTION_DOWNCOUNTER_LIMIT   3

#define LIMIT_SWITCH_UPCOUNTER_LIMIT     3
#define LIMIT_SWITCH_DOWNCOUNTER_LIMIT   3

hap_char_t *currentValueHC;
hap_char_t *targetValueHC;
hap_char_t *obstructionValueHC;
int timerCount = 0;
int ledOutput = 0;
int notifyTimerCount = 0;
int relayTimerCount = 0;

static bool targetStateChanged = false;
static bool timerChanged = false;
static bool obstructionDetected = false;

typedef enum {
    CS_FULLY_OPEN = 0,
    CS_FULLY_CLOSED = 1,
    CS_ACTIVELY_OPENING = 2,
    CS_ACTIVELY_CLOSING = 3,
    CS_STOPPED = 4,
} cState;
static cState currentState = CS_FULLY_CLOSED;

typedef enum {
    RS_VARAKOZAS_STEP1 = 0,                // Ha a motor menetben van, akkor a megállítása előtt várunk 0.5 sec-et
    RS_MEGALLIT_IMPULZUS_STEP2,            // Ha a motor menetben van, akkor ezzel az impulzussal állítjuk le
    RS_VARAKOZAS_STEP3,                    // Leállítás után várunk 1 sec-et
    RS_INDIT_IMPULZUS_STEP4,               // Indítjuk a motort egy 1 sec-es impulzussal
    RS_VARAKOZAS_STEP5,                    // Motor indító impulzus vége, várakozás 1 sec, irányellenőrzés
    RS_MEGALLIT_IMPULZUS_STEP6,            // Ha a motor rossz irányba megy, akkor ezzel az impulzussal állítjuk le
    RS_VARAKOZAS_STEP7,                    // Leállítás után várunk 1 sec-et
    RS_INDIT_IMPULZUS_STEP8,               // Indítjuk a motort egy 1 sec-es impulzussal
    RS_VARAKOZAS_STEP9,                    // Indítás után várunk 1 sec-et, irányellenőrzés
    RS_TESZT_STEP10,
    RS_IDLE,
} rStatus;
static rStatus relayStatus = RS_IDLE;

typedef enum {
    MD_WORK = 0,
    MD_IDLE = 1,
} mDirection;
static mDirection motorDirection = MD_IDLE;
static int motorDirectionOpenUpCounter = 0;
static int motorDirectionOpenDownCounter = 0;
static int motorDirectionCloseUpCounter = 0;
static int motorDirectionCloseDownCounter = 0;
static bool motorDirectionOpen = false;
static bool motorDirectionClose = false;

typedef enum {
    GD_OPEN = 0,
    GD_OPENING,
    GD_CLOSED,
    GD_CLOSING,
    GD_IDLE_FROM_OPENING,
    GD_IDLE_FROM_CLOSING,
    GD_UNKNOWN,
} gDirection;
static gDirection gateDirection = GD_UNKNOWN;

typedef enum {
    TS_OPEN = 0,
    TS_CLOSE = 1,
    TS_IDLE = 2,
} tState;
static tState targetState = TS_IDLE;

typedef enum {
    LS_OPEN = 0,
    LS_CLOSED = 1,
    LS_IDLE = 2,
    LS_ERROR = 3
} lSwitch;
static lSwitch limitSwitch = LS_CLOSED;
static lSwitch limitSwitchTemp = LS_CLOSED;
static int limitSwitchOpenUpCounter = 0;
static int limitSwitchOpenDownCounter = 0;
static int limitSwitchCloseUpCounter = 0;
static int limitSwitchCloseDownCounter = 0;
static bool limitSwitchOpen = false;
static bool limitSwitchClose = false;




/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int accessory_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void hap_event_handler(void* arg, esp_event_base_t event_base, int32_t event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED :
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED :
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED :
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                     (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED :
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                     (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED :
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED :
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING : {
            char *reason = (char *)data;
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",  reason ? reason : "null");
            break;
            case HAP_EVENT_PAIRING_MODE_TIMED_OUT :
                ESP_LOGI(TAG, "Pairing Mode timed out. Please reboot the device.");
        }
        default:
            /* Silently ignore unknown events */
            break;
    }
}

/* Callback for handling writes on the Light Bulb Service
 */
static int garage_door_opener_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;

    for (i = 0; i < count; i++) {
        write = &write_data[i];
        ESP_LOGI(TAG, "UUID értéke %s", hap_char_get_type_uuid(write->hc));

        /* Setting a default error value */
        *(write->status) = HAP_STATUS_VAL_INVALID;
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
            ESP_LOGI(TAG, "Target door state %d", write->val.i);

            switch (currentState) {
                case CS_FULLY_CLOSED:{
                     /*
                      * Itt az OPEN parancs megengedett
                      */
                     if ( write->val.i == TS_OPEN){
                         /*
                          * Beállításra kerülnek a nyitáshoz szükséges paraméterek
                          */
                         targetState = TS_OPEN;
                         targetStateChanged = true;
                     }
                    break;
                }
                case CS_FULLY_OPEN:{
                    /*
                     * Itt a CLOSED parancs megengedett
                     */
                    if ( write->val.i == TS_CLOSE){
                        /*
                         * Beállításra kerülnek a záráshoz szükséges paraméterek
                         */
                        targetState = TS_CLOSE;
                        targetStateChanged = true;
                    }
                    break;
                }
                case CS_ACTIVELY_CLOSING:{
                    /*
                     * Itt az OPEN parancs megengedett
                     */
                    if ( write->val.i == TS_OPEN){
                        /*
                         * Beállításra kerülnek a nyitáshoz szükséges paraméterek
                         */
                        targetState = TS_OPEN;
                        targetStateChanged = true;
                    }
                    break;
                }
                case CS_ACTIVELY_OPENING:{
                    /*
                     * Itt a CLOSED parancs megengedett
                     */
                    if ( write->val.i == TS_CLOSE){
                        /*
                         * Beállításra kerülnek a záráshoz szükséges paraméterek
                         */
                        targetState = TS_CLOSE;
                        targetStateChanged = true;
                    }
                    break;
                }
                case CS_STOPPED:{
                    /*
                     * Itt a CLOSED és az OPEN parancs is megengedett
                     */
                    if ( write->val.i == TS_CLOSE){
                        /*
                         * Beállításra kerülnek a záráshoz szükséges paraméterek
                         */
                        targetState = TS_CLOSE;
                        targetStateChanged = true;
                    } else if ( write->val.i == TS_OPEN){
                        /*
                         * Beállításra kerülnek a nyitáshoz szükséges paraméterek
                         */
                        targetState = TS_OPEN;
                        targetStateChanged = true;
                    }
                    break;
                }
                default:{
                    break;
                }
            }

            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
        /* If the characteristic write was successful, update it in hap core
         */
        if (*(write->status) == HAP_STATUS_SUCCESS) {
            hap_char_update_val(write->hc, &(write->val));
        } else {
            /* Else, set the return value appropriately to report error */
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static int garage_door_opener_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    }

    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_DOOR_STATE)) {

        ESP_LOGI(TAG, "Pillanatnyi állapot lekérdezése: %d", currentState);
        hap_val_t new_val;
        new_val.i = currentState;

        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;

        //const hap_val_t *val = hap_char_get_val(hc);
        currentValueHC = hc;

    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {

        ESP_LOGI(TAG, "Cél állapot lekérdezése: %d", targetState);
        hap_val_t new_val;
        new_val.i = targetState;

        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;

        //const hap_val_t *val = hap_char_get_val(hc);
        targetValueHC = hc;

    }  else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_OBSTRUCTION_DETECTED)) {

        ESP_LOGI(TAG, "Akadály detektálás lekérdezése: %d", false);
        hap_val_t new_val;
        new_val.b = obstructionDetected;

        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;

        //const hap_val_t *val = hap_char_get_val(hc);
        obstructionValueHC = hc;

    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_NAME)) {
        ESP_LOGI(TAGSENSOR, "Név lekérdezése");
        /*hap_val_t new_val;
        new_val.s = "Név";
        hap_char_update_val(hc, &new_val);*/
        *status_code = HAP_STATUS_SUCCESS;
    } else {
        ESP_LOGI(TAGSENSOR, "Ismeretlen lekérdezés %s", hap_char_get_type_uuid(hc));
        *status_code = HAP_STATUS_SUCCESS;
    }
    return HAP_SUCCESS;
}

/*The main thread for handling the Light Bulb Accessory */
static void thread_entry(void *arg)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Garage Door Opener",
        .manufacturer = "DigitInvent kft.",
        .model = "EspDoor01",
        .serial_num = "10000001",
        .fw_rev = "0.9.0",
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .identify_routine = accessory_identify,
        .cid = HAP_CID_GARAGE_DOOR_OPENER,
    };

    /* Create accessory object */
    accessory = hap_acc_create(&cfg);
    if (!accessory) {
        ESP_LOGE(TAG, "Failed to create accessory");
        goto light_err;
    }

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* Create the Light Bulb Service. Include the "name" since this is a user visible service  */
    service = hap_serv_garage_door_opener_create(1, 1, false);
    if (!service) {
        ESP_LOGE(TAG, "Failed to create garage door opener service");
        goto light_err;
    }

    /* Add the optional characteristic to the Light Bulb Service */
    int ret = hap_serv_add_char(service, hap_char_name_create("Garage door opener"));
    
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to garage door opener");
        goto light_err;
    }
    /* Set the write callback for the service */
    hap_serv_set_write_cb(service, garage_door_opener_write);

    /* Set the read callback for the service */
    hap_serv_set_read_cb(service, garage_door_opener_read);
    
    /* Add the Light Bulb Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

#ifdef CONFIG_FIRMWARE_SERVICE
    /*  Required for server verification during OTA, PEM format as string  */
    static char server_cert[] = {};
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    /* Create and add the Firmware Upgrade Service, if enabled.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    service = hap_serv_fw_upgrade_create(&ota_config);
    if (!service) {
        ESP_LOGE(TAG, "Failed to create Firmware Upgrade Service");
        goto light_err;
    }
    hap_acc_add_serv(accessory, service);
#endif

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Initialize the relays Hardware */
    relays_init();

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* TODO: Do the actual hardware initialization here */

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &hap_event_handler, NULL);

    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);

light_err:
    hap_acc_delete(accessory);
    vTaskDelete(NULL);
}

typedef struct {
    uint64_t event_count;
} example_queue_element_t;

static bool IRAM_ATTR example_timer_on_alarm_cb_v1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    timerChanged = true;
    timerCount++;
    if ( timerCount == 10){
        timerCount = 0;
        if (ledOutput == 0){
            gpio_set_level(LED, 1);
            ledOutput = 1;
        } else {
            gpio_set_level(LED, 0);
            ledOutput = 0;
        }
    }

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

static void setObstructionDetected(bool value){
    if ( obstructionDetected != value){
        ESP_LOGI(TAGSENSOR, "Az obstructionDetected megváltozott! Új érték: %d", obstructionDetected);
        obstructionDetected = value;
        hap_val_t new_val;
        new_val.b = obstructionDetected;
        hap_char_update_val(obstructionValueHC, &new_val);
    }
}

static void setCurrentState(cState value){
    if ( currentState != value){
        if ( value == CS_STOPPED){
            setObstructionDetected(true);
        } else {
            setObstructionDetected(false);
        }
        currentState = value;
        ESP_LOGI(TAGSENSOR, "A currentState megváltozott! Új érték: %d", currentState);
        hap_val_t new_val;
        new_val.i = currentState;
        hap_char_update_val(currentValueHC, &new_val);
    }
}

static void setTargetState(tState value){
    if ( targetState != value){
        targetState = value;
        ESP_LOGI(TAGSENSOR, "A targetState megváltozott! Új érték: %d", targetState);
        hap_val_t new_val;
        new_val.i = targetState;
        hap_char_update_val(targetValueHC, &new_val);
    }
}

void app_main()
{
    gptimer_handle_t gptimer = NULL;
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    xTaskCreate(thread_entry, GARAGEDOOR_TASK_NAME, GARAGEDOOR_TASK_STACKSIZE,
                NULL, GARAGEDOOR_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Create timer1 handle");

    gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
            .on_alarm = example_timer_on_alarm_cb_v1,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    gptimer_alarm_config_t alarm_config1 = {
            .reload_count = 0,
            .alarm_count = 100000, // period = 0.1s
            .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));

    if ( gptimer != NULL){
        ESP_ERROR_CHECK(gptimer_start(gptimer));
        ESP_LOGI(TAG, "Timer elindítva");
    }

    gpio_set_direction(INPUT_MD_OPEN, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_MD_CLOSE, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_LS_OPEN, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_LS_CLOSE, GPIO_MODE_INPUT);

    /*
     * Analog csatornák inicializálása
     */
    //ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    //ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_LS_OPEN, ADC_ATTEN));
    //ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_LS_CLOSE, ADC_ATTEN));

    while (1){
        if ( timerChanged){
            timerChanged = false;
            /*
             * Motor forgási irányának megállapítása. Mindkét motorirány egy-egy optocatolós bemenetre
             * csatlakozik. Ha az optocsatoló bemenetére feszültség keról, akkor az kinyit, így a processzor
             * bemenete magasra kerül.
             *
             * Nyitó irányú motor mozgás
             */
            int optoMotorDirectionOpen = gpio_get_level(INPUT_MD_OPEN);
            if (optoMotorDirectionOpen == 1){
                motorDirectionOpenDownCounter = 0;
                if ( motorDirectionOpenUpCounter >= MOTOR_DIRECTION_UPCOUNTER_LIMIT){
                    motorDirectionOpen = true;
                } else {
                    motorDirectionOpenUpCounter++;
                }
            } else {
                motorDirectionOpenUpCounter = 0;
                if ( motorDirectionOpenDownCounter >= MOTOR_DIRECTION_DOWNCOUNTER_LIMIT){
                    motorDirectionOpen = false;
                } else {
                    motorDirectionOpenDownCounter++;
                }
            }

            /*
             * Záró irányú motor mozgás
             */
            int optoMotorDirectionClose = gpio_get_level(INPUT_MD_CLOSE);
            if (optoMotorDirectionClose == 1){
                motorDirectionCloseDownCounter = 0;
                if ( motorDirectionCloseUpCounter >= MOTOR_DIRECTION_UPCOUNTER_LIMIT){
                    motorDirectionClose = true;
                } else {
                    motorDirectionCloseUpCounter++;
                }
            } else {
                motorDirectionCloseUpCounter = 0;
                if ( motorDirectionCloseDownCounter >= MOTOR_DIRECTION_DOWNCOUNTER_LIMIT){
                    motorDirectionClose = false;
                } else {
                    motorDirectionCloseDownCounter++;
                }
            }

            /*
             * A motormozgás optobemenetek állapotából kalkulálásra kerül a végleges státusz.
             * - MD_WORK, nyitó, vagy záró irányú mozgás van
             * - MD_IDLE, motorok állnak
             */
            if ( motorDirectionOpen == true || motorDirectionClose == true){
                if ( motorDirection != MD_WORK){
                    motorDirection = MD_WORK;
                    ESP_LOGI(TAGSENSOR, "A motorDirection megváltozott! Új érték:MD_WORK");
                }
            } else {
                if ( motorDirection != MD_IDLE){
                    motorDirection = MD_IDLE;
                    ESP_LOGI(TAGSENSOR, "A motorDirection megváltozott! Új érték:MD_IDLE");
                }
            }

            /*
             * Opto bemenetes végállás kapcsolók státuszának megállapítása
             * A kapcsolón akkor van feszültség, ha az nyitva van. Tehát nyitott kapcsoló állásban az opto
             * bemenetére feszültség kerül, az opto ettől kinyit és a processzor bemenetére feszültség kerül,
             * azaz logikai magasba megy.
             *
             * Nyitás irányú végálláskapcsoló vizsgálata
             * Ha a limitSwitch = true, akkor a végálláskapcsoló aktív, azaz zárt állapotban van!
             */
            int optoLimitSwitchOpen = gpio_get_level(INPUT_LS_OPEN);
            if (optoLimitSwitchOpen == 1){
                limitSwitchOpenDownCounter = 0;
                if ( limitSwitchOpenUpCounter >= LIMIT_SWITCH_UPCOUNTER_LIMIT){
                    if ( limitSwitchOpen != false){
                        limitSwitchOpen = false;
                        ESP_LOGI(TAGSENSOR, "A limitSwitchOpen kapcsoló nyitott helyzetben!");
                    }
                } else {
                    limitSwitchOpenUpCounter++;
                }
            } else {
                limitSwitchOpenUpCounter = 0;
                if ( limitSwitchOpenDownCounter >= LIMIT_SWITCH_DOWNCOUNTER_LIMIT){
                    if ( limitSwitchOpen != true){
                        limitSwitchOpen = true;
                        ESP_LOGI(TAGSENSOR, "A limitSwitchOpen kapcsoló zárt helyzetben!");
                    }
                } else {
                    limitSwitchOpenDownCounter++;
                }
            }

            /*
             * Zárás irányú végálláskapcsoló vizsgálata
             * Ha a limitSwitch = true, akkor a végálláskapcsoló aktív, azaz zárt állapotban van!
             */
            int optoLimitSwitchClose = gpio_get_level(INPUT_LS_CLOSE);
            if (optoLimitSwitchClose == 1){
                limitSwitchCloseDownCounter = 0;
                if ( limitSwitchCloseUpCounter >= LIMIT_SWITCH_UPCOUNTER_LIMIT){
                    if ( limitSwitchClose != false){
                        limitSwitchClose = false;
                        ESP_LOGI(TAGSENSOR, "A limitSwitchClose kapcsoló nyitott helyzetben!");
                    }
                } else {
                    limitSwitchCloseUpCounter++;
                }
            } else {
                limitSwitchCloseUpCounter = 0;
                if ( limitSwitchCloseDownCounter >= LIMIT_SWITCH_DOWNCOUNTER_LIMIT){
                    if ( limitSwitchClose != true){
                        limitSwitchClose = true;
                        ESP_LOGI(TAGSENSOR, "A limitSwitchClose kapcsoló zárt helyzetben!");
                    }
                } else {
                    limitSwitchCloseDownCounter++;
                }
            }

            limitSwitchTemp = limitSwitch;
            if ( limitSwitchOpen == true && limitSwitchClose == true){
                /*
                 * Mindkét kapcsoló zárt állásban van, ez elvileg nem lehetséges...
                 */
                limitSwitch = LS_ERROR;
            } else if ( limitSwitchOpen == true && limitSwitchClose == false){
                /*
                 * A kapu nyitott helyzetű végállásban van.
                 */
                limitSwitch = LS_OPEN;
            } else if ( limitSwitchOpen == false && limitSwitchClose == true){
                /*
                 * A kapu zárt helyzetű végállásban van
                 */
                limitSwitch = LS_CLOSED;
            } else {
                /*
                 * A kapu a végállások között van
                 */
                limitSwitch = LS_IDLE;
            }


            /*
             * Ha megváltozott a limit switch, akkor vizsgáljuk...
             */
            if ( limitSwitch != limitSwitchTemp){
                /*
                 * A limitSwitch értékéből kalkuláljuk a gateDirection-t.
                 */
                switch (limitSwitch) {
                    case LS_ERROR:{
                        ESP_LOGI(TAGSENSOR, "A limitSwitch megváltozott! Új érték: LS_ERROR");
                        gateDirection = GD_UNKNOWN;
                        ESP_LOGI(TAGSENSOR, "A gateDirection megváltozott! Új érték: GD_UNKNOWN");
                        break;
                    }
                    case LS_CLOSED:{
                        /*
                         * A kapu zárt helyzetbe került
                         */
                        ESP_LOGI(TAGSENSOR, "A limitSwitch megváltozott! Új érték: LS_CLOSED");
                        gateDirection = GD_CLOSED;
                        ESP_LOGI(TAGSENSOR, "A gateDirection megváltozott! Új érték: GD_CLOSED");
                        break;
                    }
                    case LS_OPEN:{
                        /*
                         * A kapu nyitott helyzetbe került
                         */
                        ESP_LOGI(TAGSENSOR, "A limitSwitch megváltozott! Új érték: LS_OPEN");
                        gateDirection = GD_OPEN;
                        ESP_LOGI(TAGSENSOR, "A gateDirection megváltozott! Új érték: GD_OPEN");
                        break;
                    }
                    case LS_IDLE:{
                        /*
                         * A kapu köztes helyzetbe került
                         */
                        ESP_LOGI(TAGSENSOR, "A limitSwitch megváltozott! Új érték: LS_IDLE");
                        switch (gateDirection) {
                            // Ha nyitás, vagy zárás közben történt..
                            case GD_IDLE_FROM_CLOSING:{
                                break;
                            }
                            case GD_IDLE_FROM_OPENING:{
                                break;
                            }
                            case GD_UNKNOWN:{
                                break;
                            }
                            case GD_OPENING:{
                                break;
                            }
                            case GD_CLOSING:{
                                break;
                            }
                            case GD_OPEN:{
                                /*
                                 * A kapu eddig nyitva volt és most került köztes helyzetbe, tehát, ha a motor működik,
                                 * akkor zárás van folyamatban, ellenkező esetben középen megállt a kapu.
                                 */
                                if ( motorDirection == MD_WORK){
                                    gateDirection = GD_CLOSING;
                                    ESP_LOGI(TAGSENSOR, "A gateDirection megváltozott! Új érték: GD_CLOSING");
                                } else {
                                    gateDirection = GD_IDLE_FROM_OPENING;
                                    ESP_LOGI(TAGSENSOR, "A gateDirection megváltozott! Új érték: GD_IDLE_FROM_OPENING");
                                }
                                break;
                            }
                            case GD_CLOSED:{
                                /*
                                 * A kapu eddig zárva volt és most került köztes helyzetbe, tehát, ha a motor működik,
                                 * akkor nyitás van folyamatban, ellenkező esetben középen megállt a kapu.
                                 */
                                if ( motorDirection == MD_WORK){
                                    gateDirection = GD_OPENING;
                                    ESP_LOGI(TAGSENSOR, "A gateDirection megváltozott! Új érték: GD_OPENING");
                                } else {
                                    gateDirection = GD_IDLE_FROM_CLOSING;
                                    ESP_LOGI(TAGSENSOR, "A gateDirection megváltozott! Új érték: GD_IDLE_FROM_CLOSING");
                                }

                                break;
                            }
                        }
                        break;
                    }
                }

                /*
                 * Változott a limitSwitch értéke, ezért a targetState-et is változtatjuk, ha kell.
                 */
                if ( limitSwitch == LS_IDLE){
                    if ( limitSwitchTemp == LS_CLOSED){
                        /*
                         * A kapu zárt állapotból köztes állapotba került, tehát nyitás van folyamatban
                         */
                        setTargetState(TS_OPEN);
                        setCurrentState(CS_ACTIVELY_OPENING);
                    } else if (limitSwitchTemp == LS_OPEN){
                        /*
                         * A kapu nyitott állapotból köztes állapotba került, tehát zárás van folyamatban
                         */
                        setTargetState(TS_CLOSE);
                        setCurrentState(CS_ACTIVELY_CLOSING);
                    }
                } else if ( limitSwitch == LS_CLOSED){
                    /*
                     * A kapu nyitott állapotból, vagy köztes állapotból zártba került, tehát zárás van folyamatban
                     */
                    setTargetState(TS_CLOSE);
                    setCurrentState(CS_FULLY_CLOSED);
                } else if ( limitSwitch == LS_OPEN){
                    /*
                     * A kapu zárt állapotból, vagy köztes állapotból nyitottba került, tehát nyitás van folyamatban
                     */
                    setTargetState(TS_OPEN);
                    setCurrentState(CS_FULLY_OPEN);
                }
            }

            if ( limitSwitch == LS_IDLE){
                /*
                 * A kapu köztes állapotban van, de a gateDirection irányból tudjuk a pillanatnyi irányt
                 */
                if ( gateDirection == GD_IDLE){
                    /*
                     * A kapu a végpontok között áll!
                     */
                    setCurrentState(CS_STOPPED);
                } else if ( gateDirection == GD_OPENING){
                    /*
                     * A gateDirection szerint nyitás irány van, ellenőrizzük a targetState-et és currentState-et
                     */
                    setTargetState(TS_OPEN);
                    setCurrentState(CS_ACTIVELY_OPENING);

                } else if ( gateDirection == GD_CLOSING){
                    /*
                     * A gateDirection szerint zárás irány van, ellenőrizzük a targetState-et és currentState-et
                     */
                    setTargetState(TS_CLOSE);
                    setCurrentState(CS_ACTIVELY_CLOSING);
                }
            }

            /*
             * Ha megvaltozott a targetState, akkor elkezdjük annak végrehajtását.
             */
            if (targetStateChanged){
                targetStateChanged = false;

                // Ha a relé megvan húzva éppen, akkor azt el kell engedni!
                relay1_set_on(false);

                /*
                 * Ha a kapu mozgásban van, akkor ellenőrizni kell, hogy a cél irányba halad-e
                 */
                switch (targetState) {
                    case TS_OPEN:{
                        // NYITAS parancs van érvényben
                        if ( gateDirection == GD_OPEN){
                            // A Kapu nyitva van, nem kell tenni semmit
                        } else if ( gateDirection == GD_OPENING ){
                            // A kapu mozgásban van nyitás irányban, nem kell tenni semmit!
                        } else if ( gateDirection == GD_CLOSING){
                            // A kapu zárás irányba mozog, le kell állítani.
                            relayStatus = RS_VARAKOZAS_STEP1;
                        } else {
                            // A kapu nincs mozgásban és nincs a megfelelő végállásban sem, el kell indítani
                            relayStatus = RS_VARAKOZAS_STEP3;
                        };
                        break;
                    }
                    case TS_CLOSE:{
                        // ZÁRÁS parancs van érvényben
                        if ( gateDirection == GD_CLOSED){
                            // A Kapu zárva van, nem kell tenni semmit
                        } else if ( gateDirection == GD_CLOSING ){
                            // A kapu mozgásban van zárás irányban, nem kell tenni semmit!
                        } else if ( gateDirection == GD_OPENING){
                            // A kapu nyitás irányba mozog, le kell állítani.
                            relayStatus = RS_VARAKOZAS_STEP1;
                        } else {
                            // A kapu nincs mozgásban és nincs a megfelelő végállásban sem, el kell indítani
                            relayStatus = RS_VARAKOZAS_STEP3;
                        };
                        break;
                    }
                    case TS_IDLE:{
                        break;
                    }
                }
                relayTimerCount = 0;
            } else {
                relayTimerCount++;

                switch (relayStatus) {
                    case RS_IDLE:{
                        relayTimerCount = 0;
                        break;
                    }

                    case RS_VARAKOZAS_STEP1:{
                        if ( relayTimerCount >= 5 ){
                            /*
                             * Eltelt 0.5 másodperc.
                             */
                            relayTimerCount = 0;
                            relayStatus = RS_MEGALLIT_IMPULZUS_STEP2;
                        }
                        break;
                    }
                    case RS_MEGALLIT_IMPULZUS_STEP2:{
                        relay1_set_on(true);
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc, relét lekapcsol.
                             */
                            relay1_set_on(false);
                            relayTimerCount = 0;
                            relayStatus = RS_VARAKOZAS_STEP3;
                        }
                        break;
                    }
                    case RS_VARAKOZAS_STEP3:{
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc
                             */
                            relayTimerCount = 0;
                            relayStatus = RS_INDIT_IMPULZUS_STEP4;
                        }
                        break;
                    }
                    case RS_INDIT_IMPULZUS_STEP4:{
                        relay1_set_on(true);
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc, relét lekapcsol.
                             */
                            relay1_set_on(false);
                            relayTimerCount = 0;
                            relayStatus = RS_VARAKOZAS_STEP5;
                        }
                        break;
                    }
                    case RS_VARAKOZAS_STEP5:{
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc, meg kell vizsgálni, hogy a kapu elérte-e a cél állapotot, vagy a
                             * motor megfelelő irányban mozgaja-e a kaput.
                             */
                            relayTimerCount = 0;

                            switch (targetState) {
                                case TS_OPEN:{
                                    // NYITAS parancs van érvényben
                                    if ( gateDirection == GD_OPEN){
                                        // A Kapu nyitva van, nem kell tenni semmit
                                    } else if ( gateDirection == GD_OPENING ){
                                        // A kapu mozgásban van nyitás irányban, nem kell tenni semmit!
                                    } else if ( gateDirection == GD_CLOSING){
                                        // A kapu zárás irányban mozog, le kell állítani és meg kell fordítani az irányt
                                        relayStatus = RS_MEGALLIT_IMPULZUS_STEP6;
                                    } else {
                                        // A kapu nincs mozgásban és nincs a megfelelő végállásban sem, ez hiba!
                                        // TODO
                                    };
                                    break;
                                }
                                case TS_CLOSE:{
                                    // ZÁRÁS parancs van érvényben
                                    if ( gateDirection == GD_CLOSED){
                                        // A Kapu zárva van, nem kell tenni semmit
                                    } else if ( gateDirection == GD_CLOSING ){
                                        // A kapu mozgásban van zárás irányban, nem kell tenni semmit!
                                    } else if ( gateDirection == GD_OPENING){
                                        // A kapu nyitás irányban mozog, le kell állítani és meg kell fordítani az irányt
                                        relayStatus = RS_MEGALLIT_IMPULZUS_STEP6;
                                    } else {
                                        // A kapu nincs mozgásban és nincs a megfelelő végállásban sem, ez hiba!
                                        // TODO
                                    };
                                    break;
                                }
                                case TS_IDLE:{
                                    break;
                                }
                            }
                        }
                        break;
                    }
                    case RS_MEGALLIT_IMPULZUS_STEP6:{
                        relay1_set_on(true);
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc, relét lekapcsol.
                             */
                            relay1_set_on(false);
                            relayTimerCount = 0;
                            relayStatus = RS_VARAKOZAS_STEP7;
                        }
                        break;
                    }
                    case RS_VARAKOZAS_STEP7:{
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc
                             */
                            relayTimerCount = 0;
                            relayStatus = RS_INDIT_IMPULZUS_STEP8;
                        }
                        break;
                    }
                    case RS_INDIT_IMPULZUS_STEP8:{
                        relay1_set_on(true);
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc, relét lekapcsol.
                             */
                            relay1_set_on(false);
                            relayTimerCount = 0;
                            relayStatus = RS_VARAKOZAS_STEP9;
                        }
                        break;
                    }
                    case RS_VARAKOZAS_STEP9:{
                        if ( relayTimerCount >= 10 ){
                            /*
                             * Eltelt 1 másodperc
                             */
                            relayTimerCount = 0;
                            relayStatus = RS_TESZT_STEP10;
                        }
                        break;
                    }
                    case RS_TESZT_STEP10:{
                        switch (targetState) {
                            case TS_OPEN:{
                                // NYITAS parancs van érvényben
                                if ( gateDirection == GD_OPEN){
                                    // A Kapu nyitva van, nem kell tenni semmit
                                } else if ( gateDirection == GD_OPENING ){
                                    // A kapu mozgásban van nyitás irányban, nem kell tenni semmit!
                                } else if ( gateDirection == GD_CLOSING){
                                    // A kapu zárás irányban mozog, ez hiba!
                                    // TODO
                                } else {
                                    // A kapu nincs mozgásban és nincs a megfelelő végállásban sem, ez hiba!
                                    // TODO
                                };
                                break;
                            }
                            case TS_CLOSE:{
                                // ZÁRÁS parancs van érvényben
                                if ( gateDirection == GD_CLOSED){
                                    // A Kapu zárva van, nem kell tenni semmit
                                } else if ( gateDirection == GD_CLOSING ){
                                    // A kapu mozgásban van zárás irányban, nem kell tenni semmit!
                                } else if ( gateDirection == GD_OPENING){
                                    // A kapu nyitás irányban mozog, ez hiba!
                                    // TODO
                                } else {
                                    // A kapu nincs mozgásban és nincs a megfelelő végállásban sem, ez hiba!
                                    // TODO
                                };
                                break;
                            }
                            case TS_IDLE:{
                                break;
                            }
                        }
                        break;
                    }
                }
            }
        }
    }
}
