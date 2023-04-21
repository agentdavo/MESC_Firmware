/*
 * MESCflash.h
 *
 *  Created on: 16 May 2021
 *      Author: cod3b453
 */

#pragma once

#include "MESCprofile.h"

uint32_t getFlashBaseAddress();
uint32_t getFlashBaseSize();
ProfileStatus eraseFlash(uint32_t address, uint32_t length);

void flash_register_profile_io();
