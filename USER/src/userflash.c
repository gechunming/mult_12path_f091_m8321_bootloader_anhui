#include "stm32f0xx.h"
#include "userflash.h"

UserNvdata *get_user_flash(void)
{
	UserNvdata *pfnv = (UserNvdata *)USER_NVRAM_DATA_ADDRESS;
	UserNvdata tmpNv;
	
	if (pfnv->magic != FLASH_USER_MAGIC) {
		//init flash
		tmpNv.magic = FLASH_USER_MAGIC;
		tmpNv.ADDR_REF_P_PLUSEWIDTH_TIME = 2566;
		tmpNv.ADDR_REF_I_PLUSEWIDTH_TIME = 416;
		tmpNv.ADDR_AC_BACKUP_E = 0;
		tmpNv.ADDR_REF_001_E = 7786;
		FLASH_Unlock();
		FLASH_ErasePage(USER_NVRAM_DATA_ADDRESS);
		FLASH_ProgramBuf(USER_NVRAM_DATA_ADDRESS, (uint8_t*)&tmpNv,  sizeof(tmpNv));
		FLASH_Lock();
	}
	return pfnv;
}

void save_user_flash(UserNvdata *pNv)
{
		FLASH_Unlock();
		FLASH_ErasePage(USER_NVRAM_DATA_ADDRESS);
		FLASH_ProgramBuf(USER_NVRAM_DATA_ADDRESS, (uint8_t*)pNv,  sizeof(UserNvdata));
		FLASH_Lock();
}

