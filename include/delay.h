#ifndef INC_DELAY_H
#define INC_DELAY_H

#include "sdr_type.h"
#include "sdr_base.h"

#define DELAY 16

void delay(fx_pt out[DELAY_16_MAX_OUT_SIZE], unsigned num_inputs, fx_pt data[DELAY_16_MAX_IN_SIZE]);
void delay320(fx_pt out[DELAY_320_MAX_OUT_SIZE], fx_pt data[DELAY_320_MAX_IN_SIZE]);

#endif
