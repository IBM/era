#ifndef INC_SYNC_LONG
#define INC_SYNC_LONG
#include <complex.h>

#include "sdr_type.h"
#include "sdr_base.h"

void sync_long( unsigned num_inputs, fx_pt* input, fx_pt* input_delayed, float* freq_offset_out, unsigned* num_outputs, fx_pt* output );


#endif

