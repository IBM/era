#ifndef INC_GR_EQUALIZER
#define INC_GR_EQUALIZER

#include <complex.h>
#include <math.h>
#include "sdr_type.h"
#include "sdr_base.h"

#ifdef INT_TIME
/* This is RECV-Equalize internal Timing information (gathering resources) */
extern uint64_t reql_total_sec;
extern uint64_t reql_total_usec;

extern uint64_t reql_symset_sec;
extern uint64_t reql_symset_usec;

extern uint64_t reql_lseq_call_sec;
extern uint64_t reql_lseq_call_usec;

extern uint64_t reql_outsym_sec;
extern uint64_t reql_outsym_usec;

extern uint64_t reql_decSF_sec;
extern uint64_t reql_decSF_usec;
#endif

void gr_equalize( float wifi_start, unsigned num_inputs, fx_pt inputs[FRAME_EQ_IN_MAX_SIZE],
		  unsigned* msg_psdu,
		  unsigned* num_out_bits, uint8_t outputs[FRAME_EQ_OUT_MAX_SIZE],
		  unsigned* num_out_sym, fx_pt out_symbols[FRAME_EQ_OUT_MAX_SIZE] );

#endif
