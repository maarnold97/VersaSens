/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : versa_api.h                                                   **
** version  : 1                                                            **
** date     : DD/MM/21                                                     **
**                                                                         **
*****************************************************************************
**                                                                         **
** Copyright (c) EPFL                                                      **
** All rights reserved.                                                    **
**                                                                         **
*****************************************************************************
	 
VERSION HISTORY:
----------------
Version     : 1
Date        : DD/MM/YY
Revised by  : Benjamin Duc
Description : Original version.


*/

/***************************************************************************/
/***************************************************************************/

/**
* @file   versa_api.h
* @date   DD/MM/YY
* @brief  This is the main header of versa_api.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _VERSA_API_H
#define _VERSA_API_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

#include <zephyr/types.h>

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/*! LED pins */
#define GREEN_LED_PIN  39
#define RED_LED_PIN    27
#define YELLOW_LED_PIN 12

#define RST_N_PIN       23
#define START_PIN       45

/*! Mode pins */
#define MODE_IDLE_PIN   26
#define MODE_STORE_PIN  24
#define MODE_STREAM_PIN 41

/*! Modes */
#define MODE_IDLE   0x01
#define MODE_STORE  0x02
#define MODE_STREAM 0x04
#define MODE_IDLE_SELECT 0x08
#define MODE_STORE_SELECT 0x10
#define MODE_STREAM_SELECT 0x20

typedef enum {
    LED_CMD_SET_MODE_INDICATOR,
    LED_CMD_TOGGLE_MODE,
    LED_CMD_EXIT_MODE_SELECT,
    LED_CMD_SYSTEM_STATUS, // for non-mode LED behavior
} led_cmd_type_t;

typedef struct {
    led_cmd_type_t cmd;
    uint32_t mode;      // 0 = idle, 1 = storing, 2 = streaming
} led_cmd_t;


/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _VERSA_API_C_SRC



#endif  /* _VERSA_API_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Start the LED thread that indicates the status of the device
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int versa_start_led_thread(void);

/**
 * @brief Start the mode thread that manage the mode depending on the switch of the main board
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int versa_start_mode_thread(void);

/**
 * @brief Initialize storage memory, PMU, BLE, and sensors marked as active.
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int versa_init(void);

/**
 * @brief Configure the sensors marked as active.
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int versa_config(void);

/**
 * @brief Start the continuous reading of the sensors marked as active.
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int versa_sensor_start(void);

/**
 * @brief Stop the continuous reading of the sensors marked as active.
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int versa_sensor_stop(void);

/**
 * @brief Start the thread that automaticly initialize new modules when they are connected
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int enable_auto_connect(void);

/**
 * @brief Stop the thread that automaticly initialize new modules when they are connected
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int disable_auto_connect(void);


/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _VERSA_API_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/