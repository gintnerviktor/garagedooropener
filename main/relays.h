/*
 *
 */

#ifndef _RELAYS_H_
#define _RELAYS_H_

/**
 * @brief initialize the relays lowlevel module
 *
 * @param none
 *
 * @return none
 */
void relays_init(void);

/**
 * @brief deinitialize the relays's lowlevel module
 *
 * @param none
 *
 * @return none
 */
void relays_deinit(void);

/**
 * @brief turn on/off the lowlevel relay1
 *
 * @param value The "On" value
 *
 * @return none
 */
int relay1_set_on(bool value);

/**
 * @brief turn on/off the lowlevel relay2
 *
 * @param value The "On" value
 *
 * @return none
 */
int relay2_set_on(bool value);

#endif /* _RELAYS_H_ */