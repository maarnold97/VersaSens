/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : SPI_Heepocrates.c                                                   **
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
* @file   SPI_Heepocrates.c
* @date   DD/MM/YY
* @brief  This is the main header of SPI_Heepocrates.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _SPI_HEEPOCRATES_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include "SPI_Heepocrates.h"
#include <nrfx_spis.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <nrfx_gpiote.h>
#include "thread_config.h"


/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

/** @brief Symbol specifying message to be sent via SPIS data transfer. */
#define MSG_TO_SEND_SLAVE "test message"

#define MAX_FIFO_SIZE 100
#define MAX_BYTES_PER_MEASUREMENT 4
#define MAX_CHUNK_SIZE 10
#define MIN_CHUNK_SIZE 2
#define CHUNK_STEP_SIZE 1
#define MAX_LATENCY_MS 500

#define SEND_DATA_CMD 1
#define RESULT_CMD    2

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(SPI_Heepocrates, LOG_LEVEL_INF);

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Function for handling SPIS driver events.
 *
 * @param[in] p_event   Pointer to the SPIS driver event.
 * @param[in] p_context Pointer to the context passed from the driver.
 */
static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context);

// /**
//  * @brief Function for handling the SPI Heepocrates thread.
//  *
//  * @param[in] arg1 Pointer to the first argument.
//  * @param[in] arg2 Pointer to the second argument.
//  * @param[in] arg3 Pointer to the third argument.
//  */
// void HEEPO_thread_func(void *arg1, void *arg2, void *arg3);

void set_spi_buffers_thread_func(void *arg1, void *arg2, void *arg3);
void calculate_latency_thread_func(void *arg1, void *arg2, void *arg3);
void fill_data_buffers_thread_func(void *arg1, void *arg2, void *arg3);
void handle_spi_buffers_set_thread_func(void *arg1, void *arg2, void *arg3);

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

/** @brief Transmit buffer initialized with the specified message ( @ref MSG_TO_SEND_SLAVE ). */
static uint8_t m_tx_buffer_slave[300] = MSG_TO_SEND_SLAVE;

/** @brief Receive buffer defined with the size to store specified message ( @ref MSG_TO_SEND_MASTER ). */
static uint8_t m_rx_buffer_slave[65536] = MSG_TO_SEND_SLAVE;

nrfx_spis_t spis_inst = NRFX_SPIS_INSTANCE(SPIS_INST_IDX);

K_FIFO_DEFINE(heepo_fifo);
K_FIFO_DEFINE(startTimeFIFO);
K_FIFO_DEFINE(stopTimeFIFO);

typedef struct {
    void *fifo_reserved;
    int64_t time;
} time_item_t;

/** @brief Buffer for the next measurement to be sent */
static uint8_t heepo_next_meas[300]; //useless
uint8_t heepo_next_meas_size; // useless

static uint8_t buffer0[MAX_BYTES_PER_MEASUREMENT*MAX_CHUNK_SIZE];
static uint8_t buffer1[MAX_BYTES_PER_MEASUREMENT*MAX_CHUNK_SIZE];
static uint8_t *spiDataBuffer = NULL;

static uint32_t chunkSize = MIN_CHUNK_SIZE;
static uint32_t maxChunkSize = MAX_CHUNK_SIZE;

static volatile bool bufferReady = false;
static volatile bool intAsserted = false;

static bool calibrationDone = false;

// void (*chunkSizeCalculator)(uint32_t*);

static atomic_t heepo_fifo_counter = ATOMIC_INIT(0);

// bool length_sent = false;

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(setSpiBuffersThreadStack, 1024);
K_THREAD_STACK_DEFINE(fillDataBuffersThreadStack, 1024);
K_THREAD_STACK_DEFINE(handleSpiBuffersSetThreadStack, 1024);
K_THREAD_STACK_DEFINE(calculateLatencyThreadStack, 1024);

struct k_thread setSpiBuffersThread;
struct k_thread calculateLatencyThread;
struct k_thread fillDataBuffersThread;
struct k_thread handleSpiBuffersSetThread;

/*! Flag to stop the thread */
volatile bool HEEPO_stop_thread_flag = false;

// semaphore for the thread
K_SEM_DEFINE(HEEPO_XFER_DONE, 0, 1);
K_SEM_DEFINE(SPI_BUFFERS_SET, 0, 1);

K_SEM_DEFINE(HEEPO_BUSY, 2,2);

K_SEM_DEFINE(SPI_BUFFERS_AVAILABLE, 2,2);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

void SPI_Heepocrates_init(void)
{
    /* Assign pin 0 and 1 to the MCUAPP */
    nrf_gpio_pin_control_select(0, NRF_GPIO_PIN_SEL_APP);
    nrf_gpio_pin_control_select(1, NRF_GPIO_PIN_SEL_APP);

    nrfx_err_t status;
    (void)status;

    nrfx_spis_config_t spis_config = NRFX_SPIS_DEFAULT_CONFIG(SCK_PIN_SLAVE,
                                                              MOSI_PIN_SLAVE,
                                                              MISO_PIN_SLAVE,
                                                              CSN_PIN_SLAVE);
    spis_config.mode = NRF_SPIS_MODE_0;
    spis_config.miso_drive = NRF_GPIO_PIN_S0S1;

    // Initialize the SPIS instance
    nrfx_spis_uninit(&spis_inst);
    void * p_context = "Some context";
    status = nrfx_spis_init(&spis_inst, &spis_config, spis_handler, p_context);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIS_INST_GET(SPIS_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_SPIS_INST_HANDLER_GET(SPIS_INST_IDX), 0);

    // Set the ready pin
    nrf_gpio_cfg_output(PIN_HEEPO_RDY);
    nrf_gpio_pin_clear(PIN_HEEPO_RDY);

    // Start the SPI Heepocrates thread
    SPI_Heepocrates_start(m_tx_buffer_slave, sizeof(m_tx_buffer_slave), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));

    return;
}

/*****************************************************************************
*****************************************************************************/

void SPI_Heepocrates_start(uint8_t * p_tx_buffer, uint16_t length_tx, uint8_t * p_rx_buffer, uint16_t length_rx)
{
    nrfx_err_t status;
    (void)status;

    // Assign TX and RX buffers
    if (p_tx_buffer == NULL)
    {
        p_tx_buffer = m_tx_buffer_slave;
        length_tx = 0;
    }

    if (p_rx_buffer == NULL)
    {
        p_rx_buffer = m_rx_buffer_slave;
        length_rx = 0;
    }

    // status = nrfx_spis_buffers_set(&spis_inst, p_tx_buffer, length_tx, p_rx_buffer, length_rx);
    // NRFX_ASSERT(status == NRFX_SUCCESS);

 
    k_thread_create(&setSpiBuffersThread, setSpiBuffersThreadStack, K_THREAD_STACK_SIZEOF(setSpiBuffersThreadStack),
                    set_spi_buffers_thread_func, NULL, NULL, NULL, 10, 0, K_NO_WAIT);
    k_thread_name_set(&setSpiBuffersThread, "setSpiBuffersThread");

    k_thread_create(&calculateLatencyThread, calculateLatencyThreadStack, K_THREAD_STACK_SIZEOF(calculateLatencyThreadStack),
                    calculate_latency_thread_func, NULL, NULL, NULL, 8, 0, K_NO_WAIT);
    k_thread_name_set(&calculateLatencyThread, "calculateLatencyThread");

    k_thread_create(&fillDataBuffersThread, fillDataBuffersThreadStack, K_THREAD_STACK_SIZEOF(fillDataBuffersThreadStack),
                    fill_data_buffers_thread_func, NULL, NULL, NULL, 8, 0, K_NO_WAIT);
    k_thread_name_set(&fillDataBuffersThread, "fillDataBuffersThread");

    k_thread_create(&handleSpiBuffersSetThread, handleSpiBuffersSetThreadStack, K_THREAD_STACK_SIZEOF(handleSpiBuffersSetThreadStack),
                    handle_spi_buffers_set_thread_func, NULL, NULL, NULL, 9, 0, K_NO_WAIT);
    k_thread_name_set(&handleSpiBuffersSetThread, "handleSpiBuffersSetThread");
    return;
}

/*****************************************************************************
*****************************************************************************/

void SPI_Heep_add_fifo(uint8_t *data, size_t size)
{
    // Check if the FIFO is full
    if(heepo_fifo_counter >= MAX_FIFO_SIZE)
    {
        LOG_ERR("HEEPO FIFO FULL");
        return;
    }

    if(!calibrationDone) {
        return;
    }

    // Allocate a new sensor data
    struct sensor_data_heepo *p_data = k_malloc(sizeof(*p_data));

    if (p_data == NULL)
    {
        LOG_ERR("Failed to allocate memory for k_malloc\n");
        return;
    }

    // Add the data to the FIFO
    p_data->size = size;
    memcpy(p_data->data, data, size);
    k_fifo_put(&heepo_fifo, p_data);
    atomic_inc(&heepo_fifo_counter);

    return;
}

/*****************************************************************************
*****************************************************************************/

// void SPI_Heep_get_fifo()
// {
//     // Get the data from the FIFO
//     struct sensor_data_heepo *p_data = k_fifo_get(&heepo_fifo, K_NO_WAIT);

//     // Check if the FIFO is empty
//     if (p_data != NULL)
//     {
//         // Copy the data to the buffer
//         memcpy(heepo_next_meas, p_data->data, p_data->size);
//         heepo_next_meas_size = p_data->size;
//         // Free the memory
//         k_free(p_data);
//         heepo_fifo_counter--;
//     }
//     else
//     {
//         heepo_next_meas_size = 0;
//     }

//     return;
// }

/*****************************************************************************
*****************************************************************************/

// void SPI_Heep_get_fifo_wait()
// {
//     // Get the data from the FIFO
//     struct sensor_data_heepo *p_data = k_fifo_get(&heepo_fifo, K_FOREVER);
//     if (p_data != NULL)
//     {
//         // Copy the data to the buffer
//         memcpy(heepo_next_meas, p_data->data, p_data->size);
//         heepo_next_meas_size = p_data->size;
//         // Free the memory
//         k_free(p_data);
//         heepo_fifo_counter--;
//     }
//     else
//     {
//         heepo_next_meas_size = 0;
//     }

//     return;
// }

/*****************************************************************************
*****************************************************************************/

void fill_data_buffers_thread_func(void *arg1, void *arg2, void *arg3) 
{
    uint32_t i = 0;
    uint16_t bufferIndex = 0;
    uint16_t bufferSize = 0;
    struct sensor_data_heepo *p_data = k_fifo_get(&heepo_fifo, K_FOREVER);
    uint32_t measurementSize = p_data->size;
    uint8_t *targetWriteBuffer = buffer0;
    uint32_t counter = 0;
    atomic_dec(&heepo_fifo_counter);
    for(i=0;i<MAX_CHUNK_SIZE;i++) { //fill entire buffer0
        memcpy(buffer0+bufferIndex+sizeof(bufferIndex), p_data->data, measurementSize); 
        bufferIndex += measurementSize;
    }
    k_free(p_data);
    // and also buffer1
    memcpy(buffer1, buffer0, bufferIndex+sizeof(bufferIndex));

    while(!calibrationDone) {
        bufferSize = chunkSize*measurementSize;
        memcpy(targetWriteBuffer, &bufferSize, sizeof(bufferSize));
        bufferReady = true;
        targetWriteBuffer = (targetWriteBuffer == buffer0) ? buffer1 : buffer0;
        k_sem_take(&HEEPO_BUSY, K_FOREVER);

        // first time we manually set the buffers, then after it is handled by the NRFX_SPIS_XFER_DONE interrupt
        static bool firstTimeBuffersSet = true;
        if(firstTimeBuffersSet) {
            nrfx_spis_buffers_set(&spis_inst, spiDataBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
            firstTimeBuffersSet = false;
        }
        if(counter>1) {
            chunkSize = (chunkSize + CHUNK_STEP_SIZE < maxChunkSize) ? chunkSize + CHUNK_STEP_SIZE : maxChunkSize;
        }
        counter++;
    }
    atomic_val_t fifo_counter;

    bufferSize = 0;
    targetWriteBuffer = buffer0;
    while(!HEEPO_stop_thread_flag) {
        struct sensor_data_heepo *p_data = k_fifo_get(&heepo_fifo, K_FOREVER);
        atomic_dec(&heepo_fifo_counter);
        bufferSize += p_data->size;
        memcpy(targetWriteBuffer+bufferSize+sizeof(bufferSize), p_data->data, p_data->size); 
        k_free(p_data); 
        i++;

        if(i == chunkSize) {
            memcpy(targetWriteBuffer, &bufferSize, sizeof(bufferSize));
            if(k_sem_take(&HEEPO_BUSY, K_NO_WAIT) == -EBUSY) {
                k_sem_take(&HEEPO_BUSY, K_FOREVER);
                fifo_counter = atomic_get(&heepo_fifo_counter);
                if(((float)fifo_counter/(float)MIN_CHUNK_SIZE) > 1 ) {
                    chunkSize = (chunkSize/2 >= MIN_CHUNK_SIZE) ? chunkSize/2 : MIN_CHUNK_SIZE;
                } else {
                    chunkSize = (chunkSize - CHUNK_STEP_SIZE > MIN_CHUNK_SIZE) ? chunkSize - CHUNK_STEP_SIZE : MIN_CHUNK_SIZE;
                }
            } else {
                chunkSize = (chunkSize + CHUNK_STEP_SIZE < maxChunkSize) ? chunkSize + CHUNK_STEP_SIZE : maxChunkSize;

            }
            bufferReady = true;
            bufferSize = 0;
            i = 0;
            
            targetWriteBuffer = (targetWriteBuffer == buffer0) ? buffer1 : buffer0;
            
        }
    }
    k_thread_abort(k_current_get());
}

void calculate_latency_thread_func(void *arg1, void *arg2, void *arg3) {
    time_item_t *startTime;
    time_item_t *stopTime;
    int64_t latency = 0;
    uint32_t bestChunkSize = MIN_CHUNK_SIZE;
    uint32_t i;
    for(i=0;i<2;i++) { // we don't care about the first two measurements
        stopTime = k_fifo_get(&stopTimeFIFO, K_FOREVER);
        k_free(stopTime);
        startTime = k_fifo_get(&startTimeFIFO, K_FOREVER);
        k_free(startTime);
    }
    while(!calibrationDone) {
        stopTime = k_fifo_get(&stopTimeFIFO, K_FOREVER);
        startTime = k_fifo_get(&startTimeFIFO, K_FOREVER);
        latency = stopTime->time - startTime->time;
        k_free(stopTime);
        k_free(startTime);
        if(latency > MAX_LATENCY_MS) {
            maxChunkSize = bestChunkSize - CHUNK_STEP_SIZE;
            calibrationDone = true;
        } else {
            bestChunkSize += CHUNK_STEP_SIZE;
            if(bestChunkSize > MAX_CHUNK_SIZE) {
                calibrationDone = true;
            }
        }
    }
    k_thread_abort(k_current_get());
}


void handle_spi_buffers_set_thread_func(void *arg1, void *arg2, void *arg3) { // very high priority
    while(!calibrationDone) {
        k_sem_take(&SPI_BUFFERS_SET, K_FOREVER);
        if(bufferReady && !intAsserted) {
            nrf_gpio_pin_set(PIN_HEEPO_RDY);
            time_item_t *startTime = k_malloc(sizeof(time_item_t));
            if(startTime == NULL) {
                LOG_ERR("unable to allocate memory");
            }
            startTime->time = k_uptime_get();
            k_fifo_put(&startTimeFIFO, startTime);
            intAsserted = true;
            bufferReady = false;
        }
    }

    while(true) {
        k_sem_take(&SPI_BUFFERS_SET, K_FOREVER);
        if(bufferReady && !intAsserted) {
            nrf_gpio_pin_set(PIN_HEEPO_RDY);
            intAsserted = true;
            bufferReady = false;
        }
    }
}
void set_spi_buffers_thread_func(void *arg1, void *arg2, void *arg3) // very high priority
{

    nrfx_err_t status;
    spiDataBuffer = buffer0;
    uint16_t bufferSize;
    while(!calibrationDone) {
        k_sem_take(&HEEPO_XFER_DONE, K_FOREVER);
        switch(m_rx_buffer_slave[0]) {
            case SEND_DATA_CMD:
                nrf_gpio_pin_clear(PIN_HEEPO_RDY); 
                intAsserted = false; 
                spiDataBuffer = (spiDataBuffer == buffer0) ? buffer1 : buffer0; 
                break;
            case RESULT_CMD:
                // do something with result
                time_item_t *stopTime = k_malloc(sizeof(time_item_t));
                if(stopTime == NULL) {
                    LOG_ERR("unable to allocate memory");
                }
                stopTime->time = k_uptime_get();

                k_sem_give(&HEEPO_BUSY);                    
                break;
            default:
                LOG_ERR("UNKNOWN COMMAND FROM HEEPO");
        }
        memcpy(&bufferSize, spiDataBuffer, sizeof(bufferSize));
        nrfx_spis_buffers_set(&spis_inst, spiDataBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
    }

    while(true) {
        k_sem_take(&HEEPO_XFER_DONE, K_FOREVER);
        switch(m_rx_buffer_slave[0]) {
            case SEND_DATA_CMD:
                nrf_gpio_pin_clear(PIN_HEEPO_RDY); 
                intAsserted = false; 
                spiDataBuffer = (spiDataBuffer == buffer0) ? buffer1 : buffer0; 
                break;
            case RESULT_CMD:
                LOG_INF("Received Result.");
                uint32_t i;
                for(i=0; i<MAX_RESULT_SIZE;i++) {
                    LOG_INF("SPIS rx buffer[%d]: 0x%02x",i, m_rx_buffer_slave[i]);
                }
                k_sem_give(&HEEPO_BUSY);                    
                break;
            default:
                LOG_ERR("UNKNOWN COMMAND FROM HEEPO");
        }
        memcpy(&bufferSize, spiDataBuffer, sizeof(bufferSize));
        nrfx_spis_buffers_set(&spis_inst, spiDataBuffer, (size_t) (bufferSize+sizeof(bufferSize)), m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
    }
    k_thread_abort(k_current_get());
}


void SPI_Heep_stop_thread(void)
{
    HEEPO_stop_thread_flag = true;
    return;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context)
{
    // Handle the SPIS event
    
    if (p_event->evt_type == NRFX_SPIS_XFER_DONE) {
        
        k_sem_give(&HEEPO_XFER_DONE);

        
    }
    else if (p_event->evt_type == NRFX_SPIS_BUFFERS_SET_DONE)
    {
        k_sem_give(&SPI_BUFFERS_AVAILABLE);
        // // Set the ready pin
        // if(bufferReady && !intAsserted) {
        //     nrf_gpio_pin_set(PIN_HEEPO_RDY);
        //     int64_t startTime = k_uptime_get();
        //     intAsserted = true;
        //     bufferReady = false;
        // }
    }
    
}

/*****************************************************************************
*****************************************************************************/

// void HEEPO_thread_func(void *arg1, void *arg2, void *arg3)
// {
//     while (HEEPO_stop_thread_flag == false)
//     {
//         // Wait for the transfer to be done
//         k_sem_take(&HEEPO_XFER_DONE, K_FOREVER);

//         // check if the last transfer was a length transfer
//         if (length_sent == false)
//         {
//             // Get the data from the FIFO
//             SPI_Heep_get_fifo_wait();

//             // Put the length in the buffer
//             m_tx_buffer_slave[0] = heepo_next_meas_size;
//             if (heepo_next_meas_size != 0)
//             {
//                 length_sent = true;
//             }
//         }
//         else
//         {
//             // Put the data in the buffer
//             memcpy(m_tx_buffer_slave, heepo_next_meas, heepo_next_meas_size);
//             length_sent = false;
//         }

//         // Set the buffers
//         nrfx_spis_buffers_set(&spis_inst,
//                               m_tx_buffer_slave, sizeof(m_tx_buffer_slave),
//                               m_rx_buffer_slave, sizeof(m_rx_buffer_slave));
//     }

//     k_thread_abort(k_current_get());
// }

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/