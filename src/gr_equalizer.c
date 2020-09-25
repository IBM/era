#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <complex.h>
#include <math.h>

/* #ifndef DEBUG_MODE */
/*  #define DEBUG_MODE */
/* #endif */
/* #ifndef VERBOSE_MODE */
/*  #define VERBOSE_MODE */
/* #endif */
#include "debug.h"

#include "sdr_type.h"
#include "sdr_base.h"
#include "sdr_viterbi.h"
#include "gr_equalizer.h"

#ifndef M_PI
 #define M_PI 3.14159265358979323846
#endif

// typedef ap_fixed<32,15> fx_pt1_ext1;
// typedef ap_fixed<64,16> fx_pt1_ext2;
typedef float fx_pt1_ext1;
typedef float fx_pt1_ext2;
/* typedef complex< fx_pt1_ext1 > fx_pt_ext1; */
/* typedef complex< fx_pt1_ext2 > fx_pt_ext2; */
typedef float complex fx_pt_ext1;
typedef float complex fx_pt_ext2;

const fx_pt LONG_ref[] = { 0 ,  0,  0,  0,  0,  0,  1,  1, -1, -1,
			   1 ,  1, -1,  1, -1,  1,  1,  1,  1,  1,
			   1 , -1, -1,  1,  1, -1,  1, -1,  1,  1,
			   1 ,  1,  0,  1, -1, -1,  1,  1, -1,  1,
			   -1,  1, -1, -1, -1, -1, -1,  1,  1, -1,
			   -1,  1, -1,  1, -1,  1,  1,  1,  1,  0,
			   0 ,  0,  0,  0 };

double d_snr;

const int interleaver_pattern[48] = {  0 , 3, 6, 9,12,15,18,21,
				       24,27,30,33,36,39,42,45,
				       1 , 4, 7,10,13,16,19,22,
				       25,28,31,34,37,40,43,46,
				       2 , 5, 8,11,14,17,20,23,
				       26,29,32,35,38,41,44,47};

int d_frame_bytes;
int d_frame_encoding;
int d_frame_symbols;
int d_frame_mod;

bool
decode_signal_field(uint8_t *rx_bits) {
  DEBUG(printf("In decode_signal_field - DSF...\n"));
  ofdm_param ofdm = {   BPSK_1_2, //  encoding   : 0 = BPSK_1_2
			13,       //             : rate field of SIGNAL header //Taken constant
			1,        //  n_bpsc     : coded bits per subcarrier
			48,       //  n_cbps     : coded bits per OFDM symbol
			24 };     //  n_dbps     : data bits per OFDM symbol

  /*frame_param::frame_param(ofdm_param &ofdm, int psdu_length) {
        psdu_size = psdu_length;
        // number of symbols (17-11)
        n_sym = (int) ceil((16 + 8 * psdu_size + 6) / (double) ofdm.n_dbps);
        n_data_bits = n_sym * ofdm.n_dbps;
        // number of padding bits (17-13)
        n_pad = n_data_bits - (16 + 8 * psdu_size + 6);
        n_encoded_bits = n_sym * ofdm.n_cbps;
  */
  frame_param frame = {  0,     // psdu_size      : PSDU size in bytes
			 1,     // n_sym          : number of OFDM symbols
			 2,     // n_pad          : number of padding bits in DATA field
			 48,    // n_encoded_bits : number of encoded bits
			 24 };  // n_data_bits    : number of data bits, including service and padding
  
  DEBUG(printf("DSF : OFDM  : %u %u %u %u %u\n", ofdm.n_bpsc, ofdm.n_cbps, ofdm.n_dbps, ofdm.encoding, ofdm.rate_field);
	printf("DSF : FRAME : %u %u %u %u %u\n", frame.psdu_size, frame.n_sym, frame.n_pad, frame.n_encoded_bits, frame.n_data_bits));
  //deinterleave(rx_bits);
  // void
  // frame_equalizer_impl::deinterleave(uint8_t *rx_bits) {
  uint8_t d_deinterleaved[MAX_ENCODED_BITS]; // This must be huge -- 24528 ?  or "Stack Smashing" -- should not require so much?
  for(int ii = 0; ii < 128; ii++) {
    d_deinterleaved[ii] = 0;
  }
  for(int ii = 0; ii < 48; ii++) {
    DEBUG(printf("DSF: Setting d_deintlvd[%u] = rx_bits[%u] = %u\n", ii, interleaver_pattern[ii], rx_bits[interleaver_pattern[ii]]));
    d_deinterleaved[ii] = rx_bits[interleaver_pattern[ii]];
  }
  // }
  
  //uint8_t *decoded_bits = d_decoder.decode(&ofdm, &frame, d_deinterleaved);
  uint8_t decoded_bits[48]; // extra-big for now (should need 48 bytes)
  int n_bits;
  DEBUG(printf("DSF: Calling decode...\n"));
  sdr_decode(&ofdm, &frame, d_deinterleaved, &n_bits, decoded_bits);
  DEBUG(printf("\nDSF: Back from decode\n");
	for (int i = 0; i < 48; i++) {
	  printf("DSF: decoded_bits[%u] = %u\n", i, decoded_bits[i]);
	});


  DEBUG(printf("\nDSF: Starting analysis...\n"));
  int r = 0;
  d_frame_bytes = 0;
  bool parity = false;
  for(int i = 0; i < 17; i++) {
    parity ^= decoded_bits[i];
    DEBUG(printf("  DSF : i %u :: parity ^ %u = %u\n", i, decoded_bits[i], parity));
    if((i < 4) && decoded_bits[i]) {
      r = r | (1 << i);
      DEBUG(printf("  DSF : i %u :: r = %u\n", i, r));
    }

    if(decoded_bits[i] && (i > 4) && (i < 17)) {
      d_frame_bytes = d_frame_bytes | (1 << (i-5));
      DEBUG(printf("  DSF I : %u :: d_frame_bytes = %u\n", i, d_frame_bytes));
    }
  }

  if (parity != decoded_bits[17]) {
    printf("SIGNAL: wrong parity %u vs %u -- bad message!\n", parity, decoded_bits[17]);  fflush(stdout);
    return false;
  }

  // NOTE: Currently we ONLY work in BPSK_1_2 -- all non-BPSK return a "false" (bad message)
  switch(r) {
  case 11:
    d_frame_encoding = 0;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 24);
    //d_frame_mod = d_bpsk;
    DEBUG(printf("Encoding: 3 Mbit/s with d_frame_enc %u d_frame_sym %u d_frame_bytes %u\n", d_frame_encoding, d_frame_symbols, d_frame_bytes));
    break;
  case 15:
    d_frame_encoding = 1;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 36);
    //d_frame_mod = d_bpsk;
    DEBUG(printf("Encoding: 4.5 Mbit/s   \n"));
    return false;
    break;
  case 10:
    d_frame_encoding = 2;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 48);
    //d_frame_mod = d_qpsk;
    DEBUG(printf("Encoding: 6 Mbit/s   \n"));
    return false;
    break;
  case 14:
    d_frame_encoding = 3;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 72);
    //d_frame_mod = d_qpsk;
    DEBUG(printf("Encoding: 9 Mbit/s   \n"));
    return false;
    break;
  case 9:
    d_frame_encoding = 4;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 96);
    //d_frame_mod = d_16qam;
    DEBUG(printf("Encoding: 12 Mbit/s   \n"));
    return false;
    break;
  case 13:
    d_frame_encoding = 5;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 144);
    //d_frame_mod = d_16qam;
    DEBUG(printf("Encoding: 18 Mbit/s   \n"));
    return false;
    break;
  case 8:
    d_frame_encoding = 6;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 192);
    //d_frame_mod = d_64qam;
    DEBUG(printf("Encoding: 24 Mbit/s   \n"));
    return false;
    break;
  case 12:
    d_frame_encoding = 7;
    d_frame_symbols = (int) ceil((16 + 8 * d_frame_bytes + 6) / (double) 216);
    //d_frame_mod = d_64qam;
    DEBUG(printf("Encoding: 27 Mbit/s   \n"));
    return false;
    break;
  default:
    printf("unknown encoding\n");
    return false;
  }

  //   mylog(boost::format("encoding: %1% - length: %2% - symbols: %3%")
  // 	  % d_frame_encoding % d_frame_bytes % d_frame_symbols);
  DEBUG(printf("\nDSF : RETURNING TRUE -- A GOOD RUN!\n"));
  return true;
}

fx_pt d_H[64];

//void do_LS_equalize(fx_pt in[64], unsigned n, fx_pt symbols[48], fx_pt* output)  // BPSK , d_frame_mod)
void do_LS_equalize(fx_pt *in, int n, fx_pt *symbols, uint8_t *bits) // BPSK , boost::shared_ptr<gr::digital::constellation> mod) {
{
  if(n == 0) {
    for (int ii = 0; ii < 64; ii++) {
      d_H[ii] = in[ii];
    }
  } else if (n == 1) {
    double signal = 0;
    double noise = 0;
    for(int i = 0; i < 64; i++) {
      if((i == 32) || (i < 6) || ( i > 58)) {
	continue;
      }
      noise += pow(cabs(d_H[i] - in[i]), 2);
      signal += pow(cabs(d_H[i] + in[i]), 2);
      d_H[i] += in[i];
      d_H[i] /= LONG_ref[i] * (fx_pt)(2 + 0 * I);
    }
    d_snr = 10 * log10(signal / noise / 2);
  } else {
    int c = 0;
    for(int ii = 0; ii < 64; ii++) {
      if( (ii == 11) || (ii == 25) || (ii == 32) || (ii == 39) || (ii == 53) || (ii < 6) || ( ii > 58)) {
	continue;
      } else {
	symbols[c] = in[ii] / d_H[ii];
	//printf("do_LS_eq : symbols[%u] = in[%u] / d_H[%u] = %12.8f %12.8f / %12.8f %12.8f = %12.8f %12.8f \n", c, ii, ii, crealf(in[ii]), cimagf(in[ii]), crealf(d_H[ii]), cimagf(d_H[ii]), crealf(symbols[c]), cimagf(symbols[c])); 
	// decision_maker is from constellation for BPSK
	//bits[c] = mod->decision_maker(&symbols[c]);
	// unsigned int
	//   constellation_bpsk_impl::decision_maker(const fx_pt *sample) {
	//   return (real(*sample) > 0);
	// }
	bits[c] = (crealf(symbols[c]) > 0);
	c++;
      }
    }
  }

}








void gr_equalize( float wifi_start, unsigned num_inputs, fx_pt inputs[FRAME_EQ_IN_MAX_SIZE], unsigned* num_out_bits, uint8_t outputs[FRAME_EQ_OUT_MAX_SIZE], unsigned* num_out_sym, fx_pt out_symbols[FRAME_EQ_OUT_MAX_SIZE] )
{
  DEBUG(printf("\nIn gr_equalize with %u inputs\n", num_inputs));
  const fx_pt POLARITY[127] = { 1 , 1, 1, 1,-1,-1,-1, 1,-1,-1,-1,-1, 1, 1,-1, 1,
				-1,-1, 1, 1,-1, 1, 1,-1, 1, 1, 1, 1, 1, 1,-1, 1,
				1 , 1,-1, 1, 1,-1,-1, 1, 1, 1,-1, 1,-1,-1,-1, 1,
				-1, 1,-1,-1, 1,-1,-1, 1, 1, 1, 1, 1,-1,-1, 1, 1,
				-1,-1, 1,-1, 1,-1, 1, 1,-1,-1,-1, 1, 1,-1,-1,-1,
				-1, 1,-1,-1, 1,-1, 1, 1, 1, 1,-1, 1,-1, 1,-1, 1,
				-1,-1,-1,-1,-1, 1,-1, 1, 1,-1, 1,-1, 1, 1, 1,-1,
				-1, 1,-1,-1,-1, 1, 1, 1,-1,-1,-1,-1,-1,-1,-1 };
  
  unsigned inp_sym = 0; // This is the input symbol number/count
  unsigned out_sym = 0; // This is the output symbol count/number
  fx_pt symbols[48];
  fx_pt current_symbol[64];

  unsigned d_current_symbol = 0;

  double d_bw   = 10000000.0; // BW = 10M
  double d_freq = 5890000000.0; // Frequency = 5.89GHz
  
  //UNUSED: double d_freq_offset_from_synclong = wifi_start  * d_bw / (2 * M_PI);
  double d_epsilon0 = wifi_start * d_bw / (2 * M_PI * d_freq);
  double d_er = 0;  // is this double?

  fx_pt d_prev_pilots[4] = { (0 + 0*I), (0 + 0*I), (0 + 0*I), (0 + 0*I) };

  //UNUSED: DEBUG(printf(" GR_FREQ : d_freq_offset_from_synclong = %12.8f\n", d_freq_offset_from_synclong));
  DEBUG(printf(" GR_FREQ : d_epsilon0 = %12.8f\n", d_epsilon0));
  unsigned num_inp_sym = num_inputs /*FRAME_EQ_IN_MAX_SIZE*/ / 64;
  DEBUG(printf(" gr_equalizer has %u input symbols\n", num_inp_sym));
  for (inp_sym = 0; inp_sym < num_inp_sym ; inp_sym++) {
    // Set up the values for the current symbols and compensate sampling offset
    DEBUG(printf("Setting up initial current_symbol : i = %u\n", inp_sym));
    for (int ii = 0; ii < 64; ii++) {
      current_symbol[ii] = inputs[64*inp_sym + ii] * exp((fx_pt)(0 + I * 2*M_PI*d_current_symbol*80*(d_epsilon0 + d_er)*(ii-32)/64));
      DEBUG(printf(" compensate: sym %2u from %5u : %12.8f %12.8f -> %12.8f %12.8f \n", ii, (64*inp_sym + ii), crealf(inputs[64*inp_sym + ii]), cimagf(inputs[64*inp_sym + ii]), crealf(current_symbol[ii]), cimagf(current_symbol[ii])));
    }

    fx_pt p = POLARITY[(d_current_symbol - 2) % 127];
    // NOT USED? fx_pt sum = ( (current_symbol[11] *  p) + (current_symbol[25] *  p) + (current_symbol[39] *  p) + (current_symbol[53] * -p) );

    float beta;
    if(d_current_symbol < 2) {
      beta = cargf( current_symbol[11] - current_symbol[25] + current_symbol[39] + current_symbol[53]);
    } else {
      beta = cargf( (current_symbol[11] *  p) + (current_symbol[39] *  p) + (current_symbol[25] *  p) + (current_symbol[53] * -p));
    }

    float er = cargf( (conj(d_prev_pilots[0]) * current_symbol[11] *  p) +  (conj(d_prev_pilots[1]) * current_symbol[25] *  p) +
		      (conj(d_prev_pilots[2]) * current_symbol[39] *  p) +  (conj(d_prev_pilots[3]) * current_symbol[53] * -p));

    er *= d_bw / (2 * M_PI * d_freq * 80);

    d_prev_pilots[0] = current_symbol[11] *  p;
    d_prev_pilots[1] = current_symbol[25] *  p;
    d_prev_pilots[2] = current_symbol[39] *  p;
    d_prev_pilots[3] = current_symbol[53] * -p;
    
    // compensate residual frequency offset
    for(int ii = 0; ii < 64; ii++) {
      current_symbol[ii] *= exp((fx_pt)(0 - beta * I ));
    }

    // update estimate of residual frequency offset
    if(d_current_symbol >= 2) {
      double alpha = 0.1;
      d_er = (1-alpha) * d_er + alpha * er;
    }

    // do equalization -- This uses "LS" Algorithm
    do_LS_equalize(current_symbol, d_current_symbol, symbols, &(outputs[ out_sym * 48])); // BPSK , d_frame_mod);
    
    // signal field -- IF good parirty/checksum, then good to go...
    if (d_current_symbol == 2) {
      // ASSUME GOOD PARITY FOR NOW ?!?
      // Otherwise, I think we decode this frame, and do some checking, etc... in the decode_signal_field (above)
      DEBUG(printf("Calling decode_signal_field with out_sym = %u and d_current_symbol = %u\n", out_sym, d_current_symbol));
      if (!decode_signal_field(&(outputs[out_sym * 48]))) {
        printf("ERROR : Bad decode_signal_field return value ...\n");
	exit(-20); // return false;
      }
      DEBUG(printf("Back from decode_signal_field...\n"));
    }
  
    if(d_current_symbol > 2) {
      // Just put this into the output stream...
      for (int ii = 0; ii < 48; ii++) {
	out_symbols[48*out_sym + ii] = symbols[ii];
	DEBUG(printf("Set out_symbols[%d] = %12.8f %12.8f\n", 48*out_sym + ii, crealf(symbols[ii]), cimagf(symbols[ii])));
      }
      DEBUG(printf("\n"));
      out_sym++;
    }

    d_current_symbol++;
  } //  for (inp_sym = 0; loop through input symbols
  *num_out_bits = out_sym * 48;
  *num_out_sym  = out_sym;
  DEBUG(printf(" gr_equalize setting %u in symbols, %u out symbols and %u out bits\n", num_inp_sym, *num_out_sym, *num_out_bits));
}
