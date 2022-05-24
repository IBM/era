#include "globals.h"
#include "sdr_type.h"
#include "sdr_base.h"
#include "delay.h"
#include "complex_ops.h"
#include "fir.h"
#include "sync_short.h"
#include "sync_long.h"
#include "ofdm.h"
#include "fft.h"

extern fx_pt  delay16_out[DELAY_16_MAX_OUT_SIZE];
extern fx_pt  cmpx_conj_out[CMP_CONJ_MAX_SIZE];
extern fx_pt  firc_input[CMP_MULT_MAX_SIZE + COMPLEX_COEFF_LENGTH]; // holds cmpx_mult_out but pre-pads with zeros
extern fx_pt* cmpx_mult_out;
extern fx_pt  correlation_complex[FIRC_MAVG48_MAX_SIZE]; // (firc mov_avg48 output
//extern fx_pt correlation_complex_m48[MOV_AVG48_MAX_SIZE]; // (mov_avg48 output

extern fx_pt1  correlation[CMP2MAG_MAX_SIZE]; // complex_to_mangitude outpu
extern fx_pt1  fir_input[CMP2MAGSQ_MAX_SIZE + COEFF_LENGTH]; // holds signal_power but pre-pads with zeros
extern fx_pt1* signal_power;
extern fx_pt1  avg_signal_power[FIR_MAVG64_MAX_SIZE]; // fir moving-average-64
//extern fx_pt1 avg_signal_power_m64[MOV_AVG64_MAX_SIZE]; // moving-average64

extern fx_pt1 the_correlation[DIVIDE_MAX_SIZE];

extern fx_pt frame_d[DELAY_320_MAX_OUT_SIZE]; // delay320 output
//extern fx_pt sync_short_out_frames[SYNC_S_OUT_MAX_SIZE]; // sync_short output
extern fx_pt* sync_short_out_frames;
extern fx_pt d_sync_long_out_frames[SYNC_L_OUT_MAX_SIZE]; // sync_long_output

uint8_t  decoded_message[MAX_PAYLOAD_SIZE];   // Holds the resulting decodede message.

// The input data goes through a delay16 that simply re-indexes the data (prepending 16 0+0i values)...
extern fx_pt* input_data;



