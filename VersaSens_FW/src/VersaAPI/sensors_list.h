/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : sensors_list.h                                                   **
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
* @file   sensors_list.h
* @date   DD/MM/YY
* @brief  This is the main header of sensors_list
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _SENSORS_LIST_H
#define _SENSORS_LIST_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

#include <zephyr/types.h>
#include <zephyr/fs/fs.h>
#include "ADS1298.h"
#include "BNO086.h"
#include "MAX30001.h"
#include "MAX77658.h"
#include "MAX86178.h"
#include "MLX90632.h"
#include "T5838.h"


/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/   

// Sensor IDs
#define ADS1298_ID     0
#define BNO086_ID      1
#define MAX30001_ID    2
#define MAX86178_ID    3
#define MLX90632_ID    4
#define T5838_ID       5

// Module IDs
#define MAIN_MODULE_ID       0
#define HEART_MODULE_ID      1
#define EXG_MODULE_ID        2
#define EDA_MODULE_ID        3

// Use this define to exclude a sensor from the build
// #define ADS1298_DISABLE
// #define BNO086_DISABLE
// #define MAX30001_DISABLE
// #define MAX86178_DISABLE
// #define MLX90632_DISABLE
// #define T5838_DISABLE

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

// Sensor structure
typedef struct {
    int module_id;
    int (*init)(void);
    int (*config)(void);
    int (*start_continuous)(void);
    int (*stop_continuous)(void);
} Sensor;

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _SENSORS_LIST_C_SRC
Sensor sensors_list[] = {
    #ifndef ADS1298_DISABLE
    [ADS1298_ID] = {EXG_MODULE_ID, ADS1298_init, ADS1298_config, ADS1298_start_thread, ADS1298_stop_thread},
    #endif
    #ifndef BNO086_DISABLE
    [BNO086_ID] = {MAIN_MODULE_ID, bno086_init, NULL, bno086_start_stream, bno086_stop_stream},
    #endif
    #ifndef MAX30001_DISABLE
    [MAX30001_ID] = {EDA_MODULE_ID, MAX30001_init, MAX30001_config, MAX30001_start_thread, MAX30001_stop_thread},
    #endif
    #ifndef MAX86178_DISABLE
    [MAX86178_ID] = {HEART_MODULE_ID, max86178_init, max86178_config, max86178_start_thread, max86178_stop_thread},
    #endif
    #ifndef MLX90632_DISABLE
    [MLX90632_ID] = {HEART_MODULE_ID, mlx90632_init, mlx90632_config, mlx90632_start_continuous_read, mlx90632_stop_continuous_read},
    #endif
    #ifndef T5838_DISABLE
    [T5838_ID] = {HEART_MODULE_ID, t5838_init, NULL, t5838_start, t5838_stop}
    #endif
};


#endif  /* _SENSORS_LIST_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/


/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/


#endif /* _SENSORS_LIST_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/