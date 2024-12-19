/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : storage.c                                                     **
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
* @file   storage.c
* @date   DD/MM/YY
* @brief  This is the main header of storage.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _STORAGE_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "storage.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/devicetree.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <nrfx_gpiote.h>

LOG_MODULE_REGISTER(storage, LOG_LEVEL_INF);

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/"DISK_DRIVE_NAME":"

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

// FATFS file system object
static FATFS fat_fs;

/*! mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

// Double buffer for used to store data before writing to the storage
struct{
    int8_t fifo_buffer1[FIFO_BUFFER_SIZE];
    int8_t fifo_buffer2[FIFO_BUFFER_SIZE];
}fifo_buffers;

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/**
 * @brief This function is the thread function that writes the fifo buffer to the storage
 *  
 * @param arg1 : argument 1, not used
 * @param arg2 : argument 2, not used
 * @param arg3 : argument 3, not used
 * 
 * @retval None
 */
void write_fifo_to_storage(void *arg1, void *arg2, void *arg3);

/**
 * @brief This function is the handler of the timer work
 *  
 * @param work : the work structure
 * 
 * @retval None
 */
void timer_work_handler(struct k_work *work);

/**
 * @brief This function is the handler of the timer
 *  
 * @param dummy : the timer structure
 * 
 * @retval None
 */
void my_timer_handler(struct k_timer *dummy);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

// Current FIFO buffer
int8_t *fifo_buffer = fifo_buffers.fifo_buffer1;
int fifo_buffer_index = 0;

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

// Path to the disk mount point
static const char *disk_mount_pt = DISK_MOUNT_PT;

// Thread stack and instance
K_THREAD_STACK_DEFINE(fifo_thread_stack, 1024);
struct k_thread fifo_thread;

// File system objects
struct fs_file_t save_file;

// Flags
bool writing_in_progress = false;
bool fifo_busy = false;
bool write_failed = false;
bool sync_flag = false;

// Work object
K_WORK_DEFINE(my_work, timer_work_handler);
// Timer object
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int storage_init(void)
{
    int res;

    write_failed = false;

    /*! Register the disk I/O device */
    res = disk_access_init(DISK_DRIVE_NAME);
    if (res) {
        LOG_ERR("Storage init error: %d", res);
        return res;
    }

    BYTE work[1000];
    f_mkfs("", 0, work, sizeof(work));

    /*! Mount the disk */
    mp.mnt_point = disk_mount_pt;
    res = fs_mount(&mp);
    if (res) {
        LOG_ERR("Error mounting the filesystem: %d", res);
        return res;
    }

    k_timer_start(&my_timer, K_SECONDS(PERIOD_SYNC), K_SECONDS(PERIOD_SYNC));

    return 0;
}

/***************************************************************************/
/***************************************************************************/

int storage_read(const char *file_name, const char *path, uint8_t *data, size_t size)
{
    int res;
    struct fs_file_t file;
    ssize_t bytes_read;
    int base = strlen(disk_mount_pt);

    fs_file_t_init(&file);

    /*! Prepare full path for the file */
    char full_path[PATH_MAX];
    strncpy(full_path, disk_mount_pt, sizeof(full_path));
    full_path[base++] = '/';
    full_path[base] = 0;

    if(strlen(path) > 0)
    {
        strcat(&full_path[base], path);
        base = base + strlen(path);
        full_path[base++] = '/';
    }

    strcat(&full_path[base], file_name);

    /*! Open the file */
    res = fs_open(&file, full_path, FS_O_READ);
    if (res) {
        LOG_ERR("Error opening file %s: %d", full_path, res);
        return res;
    }

    /*! Read the file */
    res = fs_read(&file, data, size);
    if (res < 0) {
        LOG_ERR("Error reading file %s: %d", full_path, res);
        return res;
    }

    return 0;
}

/***************************************************************************/
/***************************************************************************/

int storage_add_to_fifo(uint8_t *data, size_t size)
{
    // Check if the FIFO is busy
    while(writing_in_progress)
    {
        k_sleep(K_USEC(1));
    }
    writing_in_progress = true;

    size_t bytes_written = 0; // Number of bytes written so far

    // Write the data to the FIFO buffer
    while (bytes_written < size)
    {
        size_t bytes_to_write = MIN(FIFO_BUFFER_SIZE - fifo_buffer_index, size - bytes_written);
        memcpy(fifo_buffer + fifo_buffer_index, data + bytes_written, bytes_to_write);
        fifo_buffer_index += bytes_to_write;
        bytes_written += bytes_to_write;

        // If the FIFO buffer is full, switch to the other one
        if (fifo_buffer_index == FIFO_BUFFER_SIZE)
        {
            if(fifo_buffer == fifo_buffers.fifo_buffer1)
                fifo_buffer = fifo_buffers.fifo_buffer2;
            else
                fifo_buffer = fifo_buffers.fifo_buffer1;

            fifo_buffer_index = 0;

            // Start the thread to write the FIFO buffer to the storage
            k_thread_create(&fifo_thread, fifo_thread_stack, K_THREAD_STACK_SIZEOF(fifo_thread_stack),
                            write_fifo_to_storage, NULL, NULL, NULL, STORAGE_PRIO, 0, K_NO_WAIT);
            k_thread_name_set(&fifo_thread, "Storage Thread");
        }

    }

    writing_in_progress = false;

    return 0;
}

/***************************************************************************/
/***************************************************************************/

void storage_open_file(int conf)
{
    write_failed = false;

    int res;
    ssize_t bytes_written;
    int base = strlen(disk_mount_pt);
    int i = 0;

    fs_file_t_init(&save_file);

    /*! Prepare full path for the file */
    char full_path[PATH_MAX];
    strncpy(full_path, disk_mount_pt, sizeof(full_path));
    full_path[base++] = '/';
    static struct fs_dirent entry;

    do {
        full_path[base] = 0; // Reset the end of the base path
        snprintf(&full_path[base], sizeof(full_path) - base, "frame%d.txt", i);
        res = fs_stat(full_path, &entry);
        i++;
    } while (res == 0); // Continue if the file exists

    /*! Open the file */
    res = fs_open(&save_file, full_path, conf);
    if (res) {
        LOG_ERR("Error opening file %s: %d", full_path, res);
        return;
    }
    LOG_INF("File %s created", full_path);

    /*! Initialize the FIFO buffer */
    fifo_buffer_index = 0;
    fifo_buffer = fifo_buffers.fifo_buffer1;
    
    return;
}
/***************************************************************************/
/***************************************************************************/

void storage_close_file(void)
{
    writing_in_progress = true;
    // write the remaining data to the storage
     if (fs_write(&save_file, fifo_buffer, fifo_buffer_index) != fifo_buffer_index) {
        LOG_ERR("Failed to write to file");
    }
    writing_in_progress = false;

    // Close the file
    fs_close(&save_file);
    return;
}

/***************************************************************************/
/***************************************************************************/

void erase_file(void)
{
    int res;
    ssize_t bytes_written;
    int base = strlen(disk_mount_pt);

    fs_file_t_init(&save_file);

    /*! Prepare full path for the file */
    char full_path[PATH_MAX];
    strncpy(full_path, disk_mount_pt, sizeof(full_path));
    full_path[base++] = '/';
    full_path[base] = 0;

    strcat(&full_path[base], "frame.txt");

    /*! Unlink the file */
    res = fs_unlink(full_path);

    return;
}

/***************************************************************************/
/***************************************************************************/

int storage_create_dir(const char *dir_name)
{
    int res;
    char full_path[PATH_MAX];
    int base = strlen(disk_mount_pt);

    if(strlen(dir_name) == 0)
    {
        return 0;
    }

    /*! Prepare full path for the directory */
    strncpy(full_path, disk_mount_pt, sizeof(full_path));
    full_path[base++] = '/';
    full_path[base] = 0;

    if(strlen(dir_name) > 0)
    {
        strcat(&full_path[base], dir_name);
    }

    /*! Create the directory */
    res = fs_mkdir(full_path);
    if (res) {
        LOG_ERR("Error creating directory %s: %d", full_path, res);
        return res;
    }

    return 0;
}

/***************************************************************************/
/***************************************************************************/

int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

    char full_path[PATH_MAX];
    int base = strlen(disk_mount_pt);

    strncpy(full_path, disk_mount_pt, sizeof(full_path));

    if(strlen(path) > 0)
    {
        full_path[base++] = '/';
        full_path[base] = 0;
        strcat(&full_path[base], path);
    }

	fs_dir_t_init(&dirp);

	// Open the directory
	res = fs_opendir(&dirp, full_path);
	if (res) {
		printk("Error opening dir %s [%d]\n", full_path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", full_path);
	for (;;) {
		// Read an entry from the directory
		res = fs_readdir(&dirp, &entry);

		/*! entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

        // Print the entry name
		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
		count++;
	}

	// Close the directory
	fs_closedir(&dirp);
	if (res == 0) {
		res = count;
	}

	return res;
}

/***************************************************************************/
/***************************************************************************/

bool get_write_failed(void)
{
    return write_failed;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void write_fifo_to_storage(void *arg1, void *arg2, void *arg3){
    // Select the buffer to write
    uint8_t *fifo_buffer_write = NULL;
    if(fifo_buffer == fifo_buffers.fifo_buffer1)
        fifo_buffer_write = fifo_buffers.fifo_buffer2;
    else
        fifo_buffer_write = fifo_buffers.fifo_buffer1;
    
    /*! Write the file */
    if (fs_write(&save_file, fifo_buffer_write, FIFO_BUFFER_SIZE) != FIFO_BUFFER_SIZE) {
        LOG_ERR("Failed to write to file");
        write_failed = true;
    }

    // Sync the file system
    if (sync_flag)
    {
        fs_sync(&save_file);
        sync_flag = false;
    }

    // k_thread_abort(k_current_get());
}

/***************************************************************************/
/***************************************************************************/

void timer_work_handler(struct k_work *work)
{
    // Set the sync flag
    sync_flag = true;
}

/***************************************************************************/
/***************************************************************************/

void my_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&my_work);
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/