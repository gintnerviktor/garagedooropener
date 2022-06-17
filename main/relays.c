/*
 *
 */
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include <driver/gpio.h>

static const char *TAG = "gardenlight-lightbulb";

#define LED2 GPIO_NUM_16
#define LED1 GPIO_NUM_17
#define RELE1 GPIO_NUM_33
#define RELE2 GPIO_NUM_25

/**
 * @brief initialize the lightbulb lowlevel module
 */
void relays_init(void)
{
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);

    gpio_set_direction(RELE1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELE2, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "Initilalization done");
}

/**
 * @brief turn on/off the lowlevel relay1
 */
int relay1_set_on(bool value)
{
    ESP_LOGI(TAG, "relay1_set_on : %s", value == true ? "true" : "false");

    if ( value){
        gpio_set_level(LED1, 1);
        gpio_set_level(RELE1, 1);
    } else {
        gpio_set_level(LED1, 0);
        gpio_set_level(RELE1, 0);
    }
    return 0;
}

/**
 * @brief turn on/off the lowlevel relay2
 */
int relay2_set_on(bool value)
{
    ESP_LOGI(TAG, "relay2_set_on : %s", value == true ? "true" : "false");

    if ( value){
        gpio_set_level(LED2, 1);
        gpio_set_level(RELE2, 1);
    } else {
        gpio_set_level(LED2, 0);
        gpio_set_level(RELE2, 0);
    }
    return 0;
}