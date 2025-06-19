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
#define MAX_DATA_SIZE_HEEPO 65536

/* FIFO max number of elements */
#define SPI_HEEPOCRATES_FIFO_SIZE 10000

/* SPI Heepocrates ready signal pin */


#define MAX_RESULT_SIZE 4

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

struct sensor_data_heepo {
	void *fifo_reserved;  // reserved for use by k_fifo
	uint8_t data[MAX_DATA_SIZE_HEEPO];  // sensor data
	size_t size;  // size of the data
};

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
 * @brief This function starts a SPI transaction
 * 
 * @param p_tx_buffer : pointer to the buffer to transmit
 * @param length_tx : length of the buffer to transmit
 * @param p_rx_buffer : pointer to the buffer to receive
 * @param length_rx : length of the buffer to receive
 * 
 * @retval None
 */
void SPI_Heepocrates_start(uint8_t * p_tx_buffer, uint16_t length_tx, uint8_t * p_rx_buffer, uint16_t length_rx);

/*****************************************************************************
*****************************************************************************/

/**
 * @brief This function gets the data from the FIFO
 * 
 * @retval None
 */
void SPI_Heep_get_fifo();

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