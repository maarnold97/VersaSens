/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : pin_assignemnts.h                                            **
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

#ifndef _PIN_ASSIGNMENTS_H
#define _PIN_ASSIGNMENTS_H

#include <nrfx.h>

// ====================== SPI INTERFACE ========================
#define SPI_SCK  NRF_GPIO_PIN_MAP(0,31)
#define SPI_MOSI NRF_GPIO_PIN_MAP(1,14)
#define SPI_MISO NRF_GPIO_PIN_MAP(1,13)

// ====================== I2C INTERFACE ========================
#define I2C_SDA  NRF_GPIO_PIN_MAP(0,28)
#define I2C_SCL  NRF_GPIO_PIN_MAP(1,10)

// ====================== PDM INTERFACE ========================
#define PDM_CLK  NRF_GPIO_PIN_MAP(1,2)
#define PDM_DATA NRF_GPIO_PIN_MAP(1,3)

// ====================== PDM INTERFACE ========================
#define PDM_CLK  NRF_GPIO_PIN_MAP(1,2)
#define PDM_DATA NRF_GPIO_PIN_MAP(1,3)

// ==================== SD CARD INTERFACE ======================
#define SD_CS_N  NRF_GPIO_PIN_MAP(0,11)
#define SD_MOSI  NRF_GPIO_PIN_MAP(0,9)
#define SD_MISO  NRF_GPIO_PIN_MAP(0,10)
#define SD_SCK   NRF_GPIO_PIN_MAP(0,8)

// =================== NOR FLASH INTERFACE =====================
#define FLASH_CS_N    NRF_GPIO_PIN_MAP(0,18)
#define FLASH_IO0     NRF_GPIO_PIN_MAP(0,13)
#define FLASH_IO1     NRF_GPIO_PIN_MAP(0,14)
#define FLASH_IO2     NRF_GPIO_PIN_MAP(0,15)
#define FLASH_IO3     NRF_GPIO_PIN_MAP(0,16)
#define FLASH_SCK     NRF_GPIO_PIN_MAP(0,17)
#define FLASH_RESET_N NRF_GPIO_PIN_MAP(1,9)

// ====================== PMU INTERFACE ========================
#define PMU_AMUX      NRF_GPIO_PIN_MAP(0,4)

// ====================== IMU INTERFACE ========================
#define IMU_ENABLE_N  NRF_GPIO_PIN_MAP(0,2)
#define IMU_INT       NRF_GPIO_PIN_MAP(1,8)
#define IMU_RX        NRF_GPIO_PIN_MAP(0,21)
#define IMU_TX        NRF_GPIO_PIN_MAP(1,6)

// ====================== LED INTERFACE ========================
#define RED_LED       NRF_GPIO_PIN_MAP(1,4)
#define GREEN_LED     NRF_GPIO_PIN_MAP(0,19)
#define YELLOW_LED    NRF_GPIO_PIN_MAP(0,12)

// ===================== OTHER INTERFACE =======================
#define MODE_BTN_N    NRF_GPIO_PIN_MAP(0,24)
#define RESET_OUT_N   NRF_GPIO_PIN_MAP(0,23)



#endif // _PIN_ASSIGNMENTS_H