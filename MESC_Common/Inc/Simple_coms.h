/*
 * Simple_coms.h
 *
 *  Created on: Dec 14, 2022
 *      Author: D Molony
 */

#pragma once

#include <stdint.h>

typedef struct
{
	uint16_t some_thing;
	int16_t other_thing;
	char data[100];
	uint32_t len;
	uint32_t time;
	uint32_t period;
} COMS_data_t;

#ifdef __cplus_plus
extern "C" {
#endif

void SimpleComsInit(COMS_data_t *coms_instance);
void SimpleComsProcess(COMS_data_t *coms_instance);

#ifdef __cplus_plus
}
#endif