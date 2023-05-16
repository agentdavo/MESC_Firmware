#pragma once

#include <AFormat/r_a_as_rzt2_if.h>
#include <AFormat/r_a_as_rzt2_private.h>

void R_AFORMAT_Open(const uint8_t a_as_ch);
uint32_t R_AFORMAT_Getversion(const r_a_as_type_t type);
int32_t R_AFORMAT_Control(const r_a_as_cmd_t cmd, a_as_control_t *const pa_as_control, void *const pbuf);
void R_AFORMAT_Clear_Param(const uint8_t a_as_ch);
void R_AFORMAT_INT_END_Isr(const uint8_t a_as_ch, const uint32_t ss_reg);
void R_AFORMAT_FSS_UPD_Isr(const uint8_t a_as_ch, const uint32_t fss_reg);
void R_AFORMAT_Copy_Param(const uint8_t a_as_ch);

