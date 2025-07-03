/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : SPI_Heepocrates.h                                            **
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
* @file   SPI_Heepocrates.h
* @date   DD/MM/YY
* @brief  This is the main header of SPI_Heepocrates.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _SPI_HEEPOCRATES_H
#define _SPI_HEEPOCRATES_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include "pin_assignments.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/* Use timer for to manage the data aquisition of the SPI Heepocrates. Comment if not used */
// #define HEEPO_USE_TIMER

#define SPIS_INST_IDX 0

/* Maximum size of the data from the sensor */
#define MAX_DATA_SIZE_HEEPO 65535

#define MAX_BYTES_PER_MEASUREMENT 4
#define MAX_CHUNK_SIZE 2000
#define MAX_FIFO_SIZE 3*MAX_CHUNK_SIZE

#if MAX_CHUNK_SIZE*MAX_BYTES_PER_MEASUREMENT > 65531
    #error "Tiling_Manager: MAX_CHUNK_SIZE TOO BIG! has to be smaller thank 2^16 - 4, because that is the biggest SPI transfer possible both on the nRF5340 and HEEPOCRATES"
#endif

#define MIN_CHUNK_SIZE 100
#if MAX_CHUNK_SIZE < MIN_CHUNK_SIZE
    #error "Tiling_Manager: MAX_CHUNK_SIZE NEEDS TO BE SMALLER THAN MIN CHUNK SIZE!"
#endif
#define CHUNK_STEP_SIZE 1 // If you set MAX_CHUNK_SIZE equal to MIN_CHUNK_SIZE this has no effect
#define MAX_LATENCY_MS 300 // If you set MAX_CHUNK_SIZE equal to MIN_CHUNK_SIZE this has no effect

#define SEND_DATA_CMD 1
#define RESULT_CMD    2

#define NBR_OF_DISCRADED_LATENCY_MEASUREMENTS 2


/* SPI Heepocrates ready signal pin */


#define MAX_RESULT_SIZE 4

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/


typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    int8_t len;
    uint8_t index;
    uint8_t data[MAX_RESULT_SIZE];
	uint8_t size;
} __attribute__((packed)) HEEPO_result_t;

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

/**
 * @brief This function initializes the SPI Heepocrates
 * 
 * @retval None
 */
void SPI_Heepocrates_init(void);



/*****************************************************************************
*****************************************************************************/

/**
 * @brief This function adds the data to the FIFO
 * 
 * @param data : pointer to the data
 * @param size : size of the data
 * 
 * @retval None
 */
void SPI_Heep_add_fifo(uint8_t *data, size_t size);

bool get_calibration_done(void);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _SPI_HEEPOCRATES_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/
