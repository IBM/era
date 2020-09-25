#ifndef INC_SYNC_SHORT
#define INC_SYNC_SHORT
#include <complex.h>

#include "sdr_type.h"
#include "sdr_base.h"

void sync_short( unsigned num_inputs, fx_pt input_sample[SYNC_S_MAX_IN_SIZE], fx_pt input_abs[SYNC_S_MAX_ABS_SIZE], fx_pt1 correlation[SYNC_S_MAX_COR_SIZE], float* freq_offset_out, unsigned* num_outputs, fx_pt* output); //hls::stream<fx_pt> &output )

#endif
