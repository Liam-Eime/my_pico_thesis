#pragma once

/** sd_helpers: mount/unmount and open/close helpers for FatFs. */

#include "ff.h"      // FIL, f_open, etc.
#include "f_util.h"  // FRESULT_str

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mount the filesystem and open a file for append/write.
 * Keeps the FS mounted until sd_close_and_unmount is called.
 * Returns true on success; errors are printed.
 */
bool sd_mount_and_open(FIL* fil, const char* filename);

/**
 * Close an open file and unmount the filesystem. Errors are printed.
 */
void sd_close_and_unmount(FIL* fil);

#ifdef __cplusplus
}
#endif
