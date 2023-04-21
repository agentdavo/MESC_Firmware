
/*
* Copyright 2021-2022 David Molony
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include "MESCfoc.h"

extern uint32_t MESC_errors;
struct MESC_log_vars
{
  float current_A;
  float current_B;
  float current_C;
  float voltage;
  float motor_flux;
  float flux_a;
  float flux_b;
  int count;
};

enum
{
  //Define anticipated errors possible, not all will be implemented
  ERROR_OVERCURRENT_PHA,
  ERROR_OVERCURRENT_PHB,
  ERROR_OVERCURRENT_PHC,
  ERROR_OVERVOLTAGE,
  ERROR_UNDERVOLTAGE,
  ERROR_BRK,
  ERROR_OVERTEMPU,
  ERROR_OVERTEMPV,
  ERROR_OVERTEMPW,
  ERROR_OVERTEMP_MOTOR,
  ERROR_HARDFAULT,
  ERROR_BUSFAULT,
  ERROR_NMI,
  ERROR_MEMFAULT,
  ERROR_USAGE,
  ERROR_ADC_OUT_OF_RANGE_IA,
  ERROR_ADC_OUT_OF_RANGE_IB,
  ERROR_ADC_OUT_OF_RANGE_IC,
  ERROR_ADC_OUT_OF_RANGE_VBUS,
  ERROR_WDG,
  ERROR_UNBALANCED_CURRENT,
  ERROR_MEASUREMENT_FAIL,
  ERROR_DETECTION_FAIL,
  ERROR_HALL0,
  ERROR_HALL7,
  ERROR_MATH,
  ERROR_INPUT_OOR
};

void handleError(MESC_motor_typedef* _motor, uint32_t error_code);
