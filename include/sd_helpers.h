#pragma once

/**
 * @file sd_helpers.h
 * @brief Simple helpers for mounting the filesystem and opening/closing files.
 */

#include "ff.h"      // FIL, f_open, etc.
#include "f_util.h"  // FRESULT_str

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mount the filesystem and open a file for append/write.
 *
 * On success, the file handle is valid and the filesystem remains mounted
 * until sd_close_and_unmount is called.
 *
 * @param fil       Output: file handle to open.
 * @param filename  Path to the file to open/create.
 * @return true on success; false on error (an error is printed).
 */
bool sd_mount_and_open(FIL* fil, const char* filename);

/**
 * @brief Close an open file and unmount the filesystem.
 *
 * Errors are printed but otherwise ignored.
 *
 * @param fil  File handle previously opened by sd_mount_and_open.
 */
void sd_close_and_unmount(FIL* fil);

#ifdef __cplusplus
}
#endif
