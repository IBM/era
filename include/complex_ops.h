#ifndef INC_COMPLEX
#define INC_COMPLEX

#include "sdr_type.h"
#include "sdr_base.h"

void complex_conjugate(fx_pt out[CMP_CONJ_MAX_SIZE], unsigned num_ins, fx_pt data[CMP_CONJ_MAX_SIZE]);
void complex_to_mag_squared(fx_pt1 out[CMP2MAGSQ_MAX_SIZE], unsigned num_ins, fx_pt data[CMP2MAGSQ_MAX_SIZE]); // signal_power
void complex_to_magnitude(fx_pt1 out[CMP2MAG_MAX_SIZE], unsigned num_ins, fx_pt data[CMP2MAG_MAX_SIZE]);
void complex_multiply(fx_pt out[CMP_MULT_MAX_SIZE], unsigned num_ins, fx_pt data1[CMP_CONJ_MAX_SIZE], fx_pt data2[CMP_MULT_MAX_SIZE]);
fx_pt1 abs_c(fx_pt data);
void division(fx_pt1 output[DIVIDE_MAX_SIZE], unsigned num_inputs, fx_pt1 dividend[DIVIDE_MAX_SIZE], fx_pt1 divisor[DIVIDE_MAX_SIZE]);

#endif
