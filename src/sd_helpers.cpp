#include "sd_helpers.h"
#include <stdio.h>

// Module-scoped FATFS instance; mounted while files are open
static FATFS g_fs;

bool sd_mount_and_open(FIL* fil, const char* filename) {
    FRESULT fr = f_mount(&g_fs, "", 1);
    if (fr != FR_OK) {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    fr = f_open(fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK && fr != FR_EXIST) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        f_unmount("");
        return false;
    }
    return true;
}

void sd_close_and_unmount(FIL* fil) {
    if (fil) {
        FRESULT fr = f_close(fil);
        if (fr != FR_OK) {
            printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
        }
    }
    f_unmount("");
}
