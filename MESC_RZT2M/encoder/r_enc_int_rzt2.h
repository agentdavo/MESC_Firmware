#pragma once

#include <stdint.h>

// Declaration of the callback function
typedef void (*r_enc_int_isr_t)(void);
typedef void (*r_enc_err_isr_t)(void);

// Request information to be sent to the encoder
typedef struct
{
    r_enc_int_isr_t   r_enc_int_isr_func;
    r_enc_err_isr_t   r_enc_err_isr_func;
} r_enc_isr_t;

void r_enc_int_isr_open(uint8_t ch, const r_enc_isr_t* p_func);

void r_enc_ch0_elc_start(void);
void r_enc_ch0_elc_stop(void);
void r_enc_ch1_elc_start(void);
void r_enc_ch1_elc_stop(void);

void r_pinmux_encif04_create(short encoder_type);
void r_pinmux_encif04_release(void);
void r_pinmux_encif59_create(short encoder_type);
void r_pinmux_encif59_release(void);

void copy_to_rodata_encif(unsigned char *p, const void *p_org, unsigned int sz);

