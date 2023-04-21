/*
 * MESCflash.c
 *
 *  Created on: 16 May 2021
 *      Author: cod3b453
 *      Author: Oleksandr Ismailov
 */

#include <MESCflash.h>
#include <HAL/MESC_HAL.h>

static ProfileStatus readFlash(void* const buffer, uint32_t const address, uint32_t const length)
{
  uint32_t const* src = (uint32_t const*)(getFlashBaseAddress() + address);
  uint32_t* dst = (uint32_t*)buffer;

  for (uint32_t i = 0, j = 0; i < length; i = i + 4, j = j + 1)
    {
      dst[j] = src[j];
    }
  return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus writeBegin(void)
{
  if (!flash_Unlock())
    {
      return PROFILE_STATUS_ERROR_STORAGE_WRITE;
    }
#ifdef USE_TTERM
  vTaskDelay(100);
#endif
  return eraseFlash(getFlashBaseAddress(), PROFILE_MAX_SIZE);
}

static ProfileStatus writeFlash(void const* const buffer, uint32_t const address, uint32_t const length)
{
  uint32_t addr = getFlashBaseAddress() + address;
  uint32_t const* src = (uint32_t const*)buffer;

  for (uint32_t i = 0, j = 0; i < length; i = i + 4, j = j + 1)
    {
      bool sts = flash_Unlock();
      if (!sts)
        {
          return PROFILE_STATUS_ERROR_STORAGE_WRITE;
        }
      // TODO: Details about word doublewrd should be hidden in hal
      sts = flash_Program((addr + i), src[j]);
#if 0                    // OI
#ifndef STM32L4xx_HAL_H  //ToDo FIX THIS HACK... L4 series cannot use FLASH_TYPEPROGRAM_WORD... only DOUBLEWORD
		  sts = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (addr + i), src[j] );
#else
		  sts = HAL_ERROR;
#endif
#endif
      if (!sts)
        {
          return PROFILE_STATUS_ERROR_STORAGE_WRITE;
        }
    }
  return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus writeEnd(void)
{
#ifdef USE_TTERM
  vTaskDelay(100);
#endif
  return flash_Lock() ? PROFILE_STATUS_SUCCESS : PROFILE_STATUS_ERROR_STORAGE_WRITE;
}

void flash_register_profile_io(void)
{
  profile_configure_storage_io(readFlash, writeFlash, writeBegin, writeEnd);
}
