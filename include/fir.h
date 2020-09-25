#ifndef INC_FIR
#define INC_FIR
#include "sdr_type.h"
#include "sdr_base.h"

void fir(fx_pt1 output[FIR_MAVG64_MAX_SIZE],fx_pt1 input_sample[FIR_MAVG64_MAX_SIZE], const fx_pt1 coefficient[COEFF_LENGTH]);
void firc(fx_pt output[FIRC_MAVG48_MAX_SIZE],fx_pt input_sample[FIRC_MAVG48_MAX_SIZE]);

void convlv(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH], const fx_pt_ext coefficient[COEFF_LENGTH]);
void firG_using_my_volk(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH], const float taps_coeff[2*COEFF_LENGTH]);
void firG_using_cmp_volk(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH], const fx_pt_ext coefficient[COEFF_LENGTH]);
void firG(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH+COEFF_LENGTH], const fx_pt_ext coefficient[COEFF_LENGTH]);


#endif
