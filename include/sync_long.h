#ifndef INC_SYNC_LONG
#define INC_SYNC_LONG
#include <complex.h>

#include "sdr_type.h"
#include "sdr_base.h"

#ifdef INT_TIME
/* This is RECV-Synch-Long internal Timing information (gathering resources) */
extern uint64_t sylg_total_sec;
extern uint64_t sylg_total_usec;

extern uint64_t sylg_firG_sec;
extern uint64_t sylg_firG_usec;

extern uint64_t sylg_search_sec;
extern uint64_t sylg_search_usec;

extern uint64_t sylg_outgen_sec;
extern uint64_t sylg_outgen_usec;
#endif

void sync_long( unsigned num_inputs, fx_pt* input, fx_pt* input_delayed, float* freq_offset_out, unsigned* num_outputs, fx_pt* output );


#endif

