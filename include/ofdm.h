#ifndef INC_OFDM
#define INC_OFDM

#include "sdr_type.h"
#include "sdr_base.h"

//void decode_signal( fx_pt constellation[CHUNK], hls::stream< ap_uint<1> > &output_data  );
void decode_signal( unsigned num_inputs, fx_pt constellation[DECODE_IN_SIZE_MAX], unsigned* num_outputs, uint8_t * output_data );
void sdr_descrambler(uint8_t* in, int psdusize, char* out_msg);

#endif
