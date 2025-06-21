/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : versa_tools.c                                                **
** version  : 1                                                            **
** date     : DD/MM/YY                                                     **
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
* @file   versa_tools.c
* @date   DD/MM/YY
* @brief  This is the main header of versa_tools.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "versa_tools.h"
#include <zephyr/logging/log.h>

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(VERSA_TOOLS, LOG_LEVEL_INF);

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/


sensorPacketMetadata_t init_sensor_packet_metadata(uint8_t sensor_id) {
    sensorPacketMetadata_t metadata = {0};
    metadata.arr[0] = ((sensor_id & 0x0F)<<4);
    return metadata;
}

sensorPacketMetadata_t init_sensor_packet_metadata_with_length(uint8_t sensor_id, uint8_t length) {
    sensorPacketMetadata_t metadata = {0};
    metadata.arr[0] = ((sensor_id & 0x0F)<<4);
    metadata.arr[4] = length;
    return metadata;
}



void update_sensor_packet_metadata(sensorPacketMetadata_t* metadata, uint16_t time_s, uint16_t time_ms, uint8_t index, uint8_t length) {
    if(index >= 4 || time_ms >=1000) {
        LOG_ERR("invalid metadata!\n");
        return;
    }

    uint8_t sensor_id = metadata->arr[0];
    metadata->arr[0] = sensor_id | ((index & 0x03)<<2) || (uint8_t)((time_ms & 0x0300)>>8);
    metadata->arr[1] = (uint8_t) (time_ms & 0x00FF);
    metadata->arr[2] = (uint8_t) ((time_s & 0xFF00)>>8);
    metadata->arr[3] = (uint8_t) (time_s & 0x00FF);
    metadata->arr[4] = length;
}

void update_sensor_packet_metadata_without_length(sensorPacketMetadata_t* metadata, uint16_t time_s, uint16_t time_ms, uint8_t index) {
    if(index >= 4 || time_ms >=1000) {
        LOG_ERR("invalid metadata! index= %d and time_ns = %d\n", index, time_ms);
        return;
    }

    uint8_t sensor_id = metadata->arr[0];
    metadata->arr[0] = sensor_id | ((index & 0x03)<<2) || (uint8_t)((time_ms & 0x0300)>>8);
    metadata->arr[1] = (uint8_t) (time_ms & 0x00FF);
    metadata->arr[2] = (uint8_t) ((time_s & 0xFF00)>>8);
    metadata->arr[3] = (uint8_t) (time_s & 0x00FF);
}



/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/