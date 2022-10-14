#ifndef INC_OFDM
#define INC_OFDM

#include "sdr_type.h"
#include "sdr_base.h"

#ifdef INT_TIME
/* This is RECV-DECODE (SIGNAL) internal Timing information (gathering resources) */
extern uint64_t rdec_total_sec;
extern uint64_t rdec_total_usec;

extern uint64_t rdec_map_bitr_sec;
extern uint64_t rdec_map_bitr_usec;

extern uint64_t rdec_get_bits_sec;
extern uint64_t rdec_get_bits_usec;

extern uint64_t rdec_dec_call_sec;
extern uint64_t rdec_dec_call_usec;
#endif

//void decode_signal( fx_pt constellation[CHUNK], hls::stream< ap_uint<1> > &output_data  );
void sdr_descrambler(uint8_t* in, int psdusize, char* out_msg);

void decode_signal(unsigned* num_inputs, size_t num_inputs_sz /*= sizeof(unsigned)*/,
                fx_pt* constellation, size_t constellation_sz /*= DECODE_IN_SIZE_MAX*/,
                unsigned* num_outputs, size_t num_outputs_sz /*= sizeof(unsigned)*/,
                uint8_t * output_data, size_t output_data_sz /*= MAX_ENCODED_BITS * 3 / 4*/,
                /*Local variablse used by decode_signal*/
                uint8_t* bit_r, size_t bit_r_sz /*= DECODE_IN_SIZE_MAX*/,
                uint8_t* bit, size_t bit_sz /*= DECODE_IN_SIZE_MAX + OFDM_PAD_ENTRIES*/,
                size_t vit_size,
                ofdm_param* ofdm, size_t ofdm_sz /*= sizeof(ofdm_param)*/,
                frame_param* frame, size_t frame_sz /*= sizeof(frame_param)*/,
		int* n_res_char, size_t n_res_char_sz /*= sizeof(int)*/,
                 /* Local variables for sdr_decode_ofdm*/
                uint8_t* inMemory, size_t inMemory_sz /*= 24852*/,
                uint8_t* outMemory, size_t outMemory_sz /*18585*/,
                int* d_ntraceback_arg, size_t d_ntraceback_arg_sz /*= sizeof(int)*/
                );

#endif
