/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : versa_tools.h                                                   **
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
* @file   versa_tools.h
* @date   DD/MM/YY
* @brief  This is the main header of versa_tools.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _VERSA_TOOLS_H
#define _VERSA_TOOLS_H

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

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

typedef struct {
    uint8_t arr[5];
} sensorPacketMetadata_t;


/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/



/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

sensorPacketMetadata_t init_sensor_packet_metadata(uint8_t sensor_id);
sensorPacketMetadata_t init_sensor_packet_metadata_with_length(uint8_t sensor_id, uint8_t length);
void update_sensor_packet_metadata(sensorPacketMetadata_t* metadata, uint16_t time_s, uint16_t time_ms, uint8_t index, uint8_t length);
void update_sensor_packet_metadata_without_length(sensorPacketMetadata_t* metadata, uint16_t time_s, uint16_t time_ms, uint8_t index);





/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _VERSA_TOOLS_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/