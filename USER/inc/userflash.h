#ifndef __USER_FLASH_H
#define __USER_FLASH_H
#include "stdint.h"
#include "stdbool.h"
#include "global.h"

#define BOOTLOADER_FLASH_START_ADDRESS 0x08000000
#define BOOTLOADER_FLASH_SIZE			0x5000			//20k
#define APPLICATION_FLASH_START_ADDRESS  0x08005000
#define APPLICATION_FLASH_SIZE  0x1A800				//106K

#define USER_FLASH_START_ADDRESS 0x0801F800
#define USER_FLASH_SIZE   0x800

#define FLASH_PAGE_SIZE  0x800
#define USER_FLASH_PAGE_NUMBER  (USER_FLASH_SIZE / FLASH_PAGE_SIZE)


/*
first page used for user info
*/
#define FLASH_USER_MAGIC 0x22334479

#define USER_NVRAM_DATA_ADDRESS USER_FLASH_START_ADDRESS

typedef __packed struct _UserNvData {
	uint32_t magic;
	uint32_t FlashFlag;
	uint32_t MASTERADDR;
	uint32_t bakeupEnergy[MAX_CHARGER_PATH_NUM];
} UserNvdata;

UserNvdata *get_user_flash(void);
void save_user_flash(UserNvdata *pNv);
#endif
