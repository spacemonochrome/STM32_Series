/*
 * File_Handling_RTOS.h
 *
 *  Updated: 27.08.2025
 *  Author: teslaninsovalyesi.blogspot.com
 */

#ifndef FILE_HANDLING_H_
#define FILE_HANDLING_H_

#include "fatfs.h"
#include <string.h>
#include <stdio.h>

/* ===================== API Functions ===================== */

/* Mounts the USB drive */
void Mount_USB(void);

/* Unmounts the USB drive */
void Unmount_USB(void);

/* Recursively scans the USB starting from given path */
FRESULT Scan_USB(const char *path);

/* Deletes all files in the root directory (simple format) */
FRESULT Format_USB(void);

/* Creates a new file (overwrites if exists) */
FRESULT Create_File(const char *name);

/* Writes data to an existing file (overwrite mode) */
FRESULT Write_File(const char *name, const char *data);

/* Appends data to an existing file */
FRESULT Update_File(const char *name, const char *data);

/* Reads file content and sends it via UART */
FRESULT Read_File(const char *name);

/* Removes a file */
FRESULT Remove_File(const char *name);

/* Creates a new directory */
FRESULT Create_Dir(const char *name);

/* Checks USB free space and total capacity */
void Check_USB_Details(void);

/* UART formatted printing helper */
void UART_Printf(const char *format, ...);

#endif /* FILE_HANDLING_H_ */
