/** This code represents the current ERA-V3 GnuRadio ieee802_11 TRANSMIT pipeline 
    TRANSMIT:
    1. Form a message : input string up to 1500 characters : char[1501] (terminating '\0')? [Input]
    2. WiFi-MAC : gr-ieee802-11/mac.cc             generate_mac_data_frame (was do_wifi_mac)
    3. Wifi-Mapper : gr-ieee802-11/mapper.cc       do_mapper_work
    4. Packet-Header Generator : gnuradio/gr-digital/lib/packet_headergenerator_bb_impl.cc    
    5a. Chunks-to-Symbols : gnuradio/gr-digital/lib/chunks_to_symbols_impl.*
    5b. Chunks-to-Symbols : gr-ieee802-11/chunks_to_symbols_impl.cc
    6. Tagged-Stream-Mux : gnuradio/gr-blocks/lib/tagged_stream_mux_impl.cc  :: <concatenate: header from 5a and data symbols from 5b> :
    7. OFDM-Carrier-Allocator : gnuradio/gr-digital/lib/ofdm_carrier_allocator_cvc_impl.cc
    8. iFFT : gnuradio/gr-fft (uses FFTW lib) : 64-size Inverse FFT
    9. OFDM-Cycle-Prefixer : gnuradio/gr-digital/lib/ofdm_cyclic_prefixer_impl.cc 
   10. Pad2 Block (which just adds 500 0.0+0.0i values before the message?)
    X. OUTPUT : <some number of complex numbers?>
**/


/** TO-DO Yet (To make this more "general"):
 **  This code only works in BPSK_1_2 mode at this time.
 **   There are at least 2 or 3 locations where this assumptions in exploited:
 **    1. We set punctured_data == encoded_data (no puncturing effect) -- TRUE only for BPSK_1_2, QPSK_1_2, and QAM16_1_2
 **    2. We skip the actual "puncturing" code (at this time)
 **    3. In generate_signal_field we explicitly set the BPSK_1_2 parameters, etc.
 **         Actually, this might be general -- I think this is for the header, which is always BPSK_1_2 encoded...?
 **    4. We don't use "chunks_to_symbols" routine -- we use the simple mapping of BPSK_1_2 directly in xmit_pipe code for now
 **         Not sure this matters -- I don't see where the d_mapping is used?
 **
 ** If we decide to go to a more general implementation of the pipeline, then we'll need to address these issues.
 **/

//#define VERBOSE  // Turn on all debug output


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>

#include "debug.h"
#include "sdr_base.h"
#include "xmit_pipe.h"

#include "fft.h"

#ifdef INT_TIME
/* This is XMIT PIPE internal Timing information (gathering resources) */
struct timeval x_pipe_stop, x_pipe_start;
uint64_t x_pipe_sec  = 0LL;
uint64_t x_pipe_usec = 0LL;

struct timeval x_genmacfr_stop; /*, x_genmacfr_start;*/
uint64_t x_genmacfr_sec  = 0LL;
uint64_t x_genmacfr_usec = 0LL;

struct timeval x_domapwk_stop; /*, x_domapwk_start;*/
uint64_t x_domapwk_sec  = 0LL;
uint64_t x_domapwk_usec = 0LL;

struct timeval x_phdrgen_stop; /*, x_phdrgen_start;*/
uint64_t x_phdrgen_sec  = 0LL;
uint64_t x_phdrgen_usec = 0LL;

struct timeval x_ck2sym_stop, x_ck2sym_start;
uint64_t x_ck2sym_sec  = 0LL;
uint64_t x_ck2sym_usec = 0LL;

struct timeval x_ocaralloc_stop, x_ocaralloc_start;
uint64_t x_ocaralloc_sec  = 0LL;
uint64_t x_ocaralloc_usec = 0LL;

struct timeval x_fft_stop; /*, x_fft_start; */
uint64_t x_fft_sec  = 0LL;
uint64_t x_fft_usec = 0LL;

struct timeval x_ocycpref_stop; /*, x_ocycpref_start;*/
uint64_t x_ocycpref_sec  = 0LL;
uint64_t x_ocycpref_usec = 0LL;

#endif

extern bool show_output;
extern bool do_add_pre_pad;

#define scramble_header        false
#define bits_per_header_sym    8
#define d_bits_per_byte        8


/**## This is taken from gr-ieee802_11/mac.cc for WIFI_MAC of xmit pipeline ##**/

// GLOBALS (for now)
uint16_t d_seq_nr;
uint8_t  d_src_mac[6] = { 0x12, 0x34, 0x56, 0x78, 0x90, 0xab };
uint8_t  d_dst_mac[6] = { 0x30, 0x14, 0x4a, 0xe6, 0x46, 0xe4 };
uint8_t  d_bss_mac[6] = { 0x42, 0x42, 0x42, 0x42, 0x42, 0x42 };

uint8_t  d_psdu[MAX_PSDU_SIZE];
uint8_t  d_map_out[32768];

int ones_count(int n) {
  int sum = 0;
  for(int i = 0; i < 8; i++) {
    if(n & (1 << i)) {
      sum++;
    }
  }
  return sum;
}



/* from dsrc/gr-ieee802-11/lib/utils.c */
void convolutional_encoding(const char *in, char *out, int n_data_bits) { //frame_param* frame) {

  int state = 0;

  for(int i = 0; i < /*frame->*/n_data_bits; i++) {
    assert(in[i] == 0 || in[i] == 1);
    state = ((state << 1) & 0x7e) | in[i];
    out[i * 2]     = ones_count(state & 0155) % 2;
    out[i * 2 + 1] = ones_count(state & 0117) % 2;
  }
}

void interleave(const char *in, char *out, int in_sym, /*frame_param* frame,*/ int in_cbps, int in_bpsc, /*ofdm_param* ofdm,*/ bool reverse) {

  int n_cbps = /*ofdm.*/in_cbps;
  int first[n_cbps];
  int second[n_cbps];
  int s = ((/*ofdm.*/in_bpsc / 2) > 1) ? (/*ofdm.*/in_bpsc / 2) :  1;

  for(int j = 0; j < n_cbps; j++) {
    first[j] = s * (j / s) + ((j + (int)(floor(16.0 * j / n_cbps))) % s);
  }

  for(int i = 0; i < n_cbps; i++) {
    second[i] = 16 * i - (n_cbps - 1) * (int)(floor(16.0 * i / n_cbps));
  }

  for(int i = 0; i < /*frame.*/in_sym; i++) {
    for(int k = 0; k < n_cbps; k++) {
      if(reverse) {
	out[i * n_cbps + second[first[k]]] = in[i * n_cbps + k];
      } else {
	out[i * n_cbps + k] = in[i * n_cbps + second[first[k]]];
      }
    }
  }
}



/*********************************************************************************
 * The CRC code is taken from an open source CRC package, which is used here 
 * to provide the proper GnuRadio 32-bit CRC encoding.  The contents taken from
 * are in crc.h and crc.c where copyright and author details are retained.
 *********************************************************************************/
#include "crc.h"

// NOTE: We currently only run in BPSK_1_2 mode

void
init_ofdm_parms(ofdm_param* ofdm, int enc)
{
  ofdm->encoding = enc;

  switch(enc) {
  case BPSK_1_2:
    ofdm->n_bpsc = 1;
    ofdm->n_cbps = 48;
    ofdm->n_dbps = 24;
    ofdm->rate_field = 0x0D; // 0b00001101
    break;

  case BPSK_3_4:
    ofdm->n_bpsc = 1;
    ofdm->n_cbps = 48;
    ofdm->n_dbps = 36;
    ofdm->rate_field = 0x0F; // 0b00001111
    break;

  case QPSK_1_2:
    ofdm->n_bpsc = 2;
    ofdm->n_cbps = 96;
    ofdm->n_dbps = 48;
    ofdm->rate_field = 0x05; // 0b00000101
    break;

  case QPSK_3_4:
    ofdm->n_bpsc = 2;
    ofdm->n_cbps = 96;
    ofdm->n_dbps = 72;
    ofdm->rate_field = 0x07; // 0b00000111
    break;

  case QAM16_1_2:
    ofdm->n_bpsc = 4;
    ofdm->n_cbps = 192;
    ofdm->n_dbps = 96;
    ofdm->rate_field = 0x09; // 0b00001001
    break;

  case QAM16_3_4:
    ofdm->n_bpsc = 4;
    ofdm->n_cbps = 192;
    ofdm->n_dbps = 144;
    ofdm->rate_field = 0x0B; // 0b00001011
    break;

  case QAM64_2_3:
    ofdm->n_bpsc = 6;
    ofdm->n_cbps = 288;
    ofdm->n_dbps = 192;
    ofdm->rate_field = 0x01; // 0b00000001
    break;

  case QAM64_3_4:
    ofdm->n_bpsc = 6;
    ofdm->n_cbps = 288;
    ofdm->n_dbps = 216;
    ofdm->rate_field = 0x03; // 0b00000011
    break;
  default:
    assert(false);
    break;
  }

}


/********************************************************************************
 * This routine does general data assembly and a CRC32 computation
 * This is almost entirely data movement/copy except for the CRC!
 ********************************************************************************/
void generate_mac_data_frame(const char *msdu, int msdu_size, int *psdu_size) {
  // mac header
  mac_header* header = (mac_header*)&d_psdu;
  header->frame_control = 0x0008;
  header->duration      = 0x0000;
  
  for(int i = 0; i < 6; i++) {
    header->addr1[i] = d_dst_mac[i];
    header->addr2[i] = d_src_mac[i];
    header->addr3[i] = d_bss_mac[i];
  }
  
  header->seq_nr = 0;
  for (int i = 0; i < 12; i++) {
    if(d_seq_nr & (1 << i)) {
      header->seq_nr |=  (1 << (i + 4));
    }
  }
  //header->seq_nr = htole16(header->seq_nr);  // Don't need -- #if defined(__APPLE__)

  d_seq_nr++;
  
  //header size is 24, plus 4 for FCS means 28 bytes
  *psdu_size = 28 + msdu_size;
  
  //copy mac header into psdu
  //memcpy(d_psdu, &header, 24);
  //copy msdu into psdu
  //memcpy(d_psdu + 24, msdu, msdu_size);
  for (int i = 0; i < msdu_size; i++) {
    d_psdu[i+24] = msdu[i];
  }

  DEBUG({
      uint8_t* ph = (uint8_t*)(header);
      printf("Header : ");
      for (int i = 0; i < 24; i++) {
	printf("%02x ", ph[i]);
      }
      printf("\n");  fflush(stdout);

      printf("d_psdu : ");
      for (int i = 0; i < 24; i++) {
	printf("%02x ", d_psdu[i]);
      }
      printf("\n");  fflush(stdout);
    });
  //compute and store fcs
  /**** TODO: This is a "standard" crc_32 computation 
	Note that this runs over the header AND payload bytes.
	boost::crc_32_type result;
	result.process_bytes(d_psdu, msdu_size + 24); 
	** typedef crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true> crc_32_type; **
  
	uint32_t fcs = result.checksum();
	memcpy(d_psdu + msdu_size + 24, &fcs, sizeof(uint32_t)); 
  ****/
  DEBUG(printf("Doing fcs computation using %s\n", CRC_NAME));
  uint32_t fcs = crcFast(d_psdu, msdu_size+24);
  DEBUG(printf("FCS = 0x%08x\n", fcs));
  d_psdu[msdu_size+24] = (fcs&0x000000ff);
  d_psdu[msdu_size+25] = (fcs&0x0000ff00)>>8;
  d_psdu[msdu_size+26] = (fcs&0x00ff0000)>>16;
  d_psdu[msdu_size+27] = (fcs&0xff000000)>>24;

  DEBUG({printf("Message: ");
      for (int i = 0; i < 24; i++) {
	printf("%02x ", d_psdu[i]);
      }
      printf("\n");
      for (int i = 24; i < (24+msdu_size); i++) {
	printf("%c", d_psdu[i]);
      }
      printf("\n");
      for (int i = (24+msdu_size); i < *psdu_size; i++) {
	printf("%02x ", d_psdu[i]);
      }
      printf("\n"); fflush(stdout);
    });
}


/* // This is the app_in work from gr-ieee802_11/mac.cc */
/* void do_wifi_mac(int msg_len, char * in_msg, int* psdu_length) { */
/*   // The gr-ieee802.11p code for app_in calls generate_mac_data_frame and then publishes the resulting d_psdu to phy_out */
/*   generate_mac_data_frame(in_msg, msg_len, psdu_length); */
/*   // I think the rest is just setting up output port xfer data, etc. */
/* } */




/**## This is taken from gr-ieee802_11/mapper.cc for WIFI_MAPPER of xmit pipeline ##**/

// GLOBALS
uint8_t      d_scrambler;
bool         d_debug;
char         d_symbols[24528];
int          d_symbols_offset;
int          d_symbols_len;
ofdm_param   d_ofdm;
frame_param  d_frame;

char data_bits[12264];        // = (char*)calloc(frame.n_data_bits, sizeof(char));
char scrambled_data[12264];   // = (char*)calloc(frame.n_data_bits, sizeof(char));
char encoded_data[12264*2];   // = (char*)calloc(frame.n_data_bits * 2, sizeof(char));
//char punctured_data[24528];   // = (char*)calloc(frame.n_encoded_bits, sizeof(char));
char* punctured_data = encoded_data; // TRUE for BPSK_1_2, QPSK_1_2, and QAM16_1_2
char interleaved_data[24528]; // = (char*)calloc(frame.n_encoded_bits, sizeof(char));
char symbols[24528];          // = (char*)calloc((frame.n_encoded_bits / d_ofdm.n_bpsc), sizeof(char)); ## n_bpsc == 1 ##



// From analysis of calls to this, it appears that noutput == 32768 (always) -- max size of data out buffer?.

int do_mapper_work(int psdu_length) // int noutput, gr_vector_int& ninput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& output_items )
{
  /* frame_param frame(d_ofdm, psdu_length); */
  d_frame.psdu_size = psdu_length;
  // number of symbols (17-11)
  d_frame.n_sym = (int) ceil((16 + 8 * d_frame.psdu_size + 6) / (double) d_ofdm.n_dbps);
  d_frame.n_data_bits = d_frame.n_sym * d_ofdm.n_dbps;
  // number of padding bits (17-13)
  d_frame.n_pad = d_frame.n_data_bits - (16 + 8 * d_frame.psdu_size + 6);
  d_frame.n_encoded_bits = d_frame.n_sym * d_ofdm.n_cbps;

  DEBUG(printf("\nOFDM: encoding: %u\n", d_ofdm.encoding);
	printf("      rate_fld: %u\n", d_ofdm.rate_field);
	printf("      n_bpsc  : %u\n", d_ofdm.n_bpsc);
	printf("      n_cbps  : %u\n", d_ofdm.n_cbps);
	printf("      n_dbps  : %u\n", d_ofdm.n_dbps);
	printf("\nFRAME: psdu_sz: %u\n", d_frame.psdu_size);
	printf("       n_sym  : %u\n", d_frame.n_sym);
	printf("       n_pad  : %u\n", d_frame.n_pad);
	printf("       n_enc  : %u\n", d_frame.n_encoded_bits);
	printf("       n_dbits: %u\n", d_frame.n_data_bits));
      
  //generate the WIFI data field, adding service field and pad bits
  //     generate_bits(psdu, data_bits, frame);
  //     void generate_bits(const char *psdu, char *data_bits, frame_param &frame)
  {
    // first 16 bits are zero (SERVICE/DATA field)
    //memset(data_bits, 0, 16);
    //data_bits += 16;
    for (int i = 0; i < 16; i++) {
      data_bits[i] = 0;
    }
    for(int i = 0; i < d_frame.psdu_size; i++) {
      for(int b = 0; b < 8; b++) {
	data_bits[16 + i * 8 + b] = !!(d_psdu[i] & (1 << b));
      }
    }
  }
  DEBUG({
      int di_row = 0;
      int symbols_len = d_frame.n_data_bits;
      printf("\ndata_bits out:\n%6u : ", di_row);
      for (int di = 0; di < symbols_len; di++) {
	printf("%1x", data_bits[di]);
	if ((di % 128) == 127) {
	  di_row++;
	  printf("\n%6u : ", di_row);
	} else if ((di % 8) == 7) {
	  printf(" ");
	}
      }
      printf("\n\n");
    });
	
  // scrambling
  //     scramble(     data_bits, scrambled_data,              frame,      d_scrambler++);
  //     void scramble(const char *in, char *out,      frame_param &frame, char initial_state)
  {
    int state = d_scrambler++; // initial_state;
    int feedback;
      
    for (int i = 0; i < d_frame.n_data_bits; i++) {
      feedback = (!!(state & 64)) ^ (!!(state & 8));
      scrambled_data[i] = feedback ^ data_bits[i];
      DEBUG(printf("  %u : state %u   feedback %u   data %u   scrambled %u\n", i, state, feedback, data_bits[i], scrambled_data[i]));
      state = ((state << 1) & 0x7e) | feedback;
    }
  }
    
  if(d_scrambler > 127) {
    d_scrambler = 1;
  }
    
  // reset tail bits
  //     reset_tail_bits(scrambled_data, frame);
  //     void reset_tail_bits(char *scrambled_data, frame_param &frame)
  {
    //memset(scrambled_data + d_frame.n_data_bits - d_frame.n_pad - 6, 0, 6 * sizeof(char));
    for (int i = 0; i < 6; i++) {
      scrambled_data[d_frame.n_data_bits - d_frame.n_pad - 6 + i] = 0;
    }
  }
  DEBUG({
      int di_row = 0;
      int symbols_len = d_frame.n_data_bits;
      printf("\nscrambled out:\n%6u : ", di_row);
      for (int di = 0; di < symbols_len; di++) {
	printf("%1x", scrambled_data[di]);
	if ((di % 128) == 127) {
	  di_row++;
	  printf("\n%6u : ", di_row);
	} else if ((di % 8) == 7) {
	  printf(" ");
	}
      }
      printf("\n\n");
    });
	
  // encoding
  //     convolutional_encoding(scrambled_data, encoded_data, frame);
  //     void convolutional_encoding(const char *in, char *out, frame_param &frame)
  convolutional_encoding(scrambled_data, encoded_data, d_frame.n_data_bits);
  /* { */
  /* 	int state = 0; */
  /* 	for(int i = 0; i < d_frame.n_data_bits; i++) { */
  /* 	  assert(scrambled_data[i] == 0 || scrambled_data[i] == 1); */
  /* 	  state = ((state << 1) & 0x7e) | scrambled_data[i]; */
  /* 	  encoded_data[i * 2]     = ones_count(state & 0155) % 2; */
  /* 	  encoded_data[i * 2 + 1] = ones_count(state & 0117) % 2; */
  /* 	} */
  /* } */
  DEBUG({
      int di_row = 0;
      int symbols_len = d_frame.n_data_bits;
      printf("\nencoded out:\n%6u : ", di_row);
      for (int di = 0; di < symbols_len; di++) {
	printf("%1x", encoded_data[di]);
	if ((di % 128) == 127) {
	  di_row++;
	  printf("\n%6u : ", di_row);
	} else if ((di % 8) == 7) {
	  printf(" ");
	}
      }
      printf("\n\n");
    });
    
  // puncturing
  //puncturing(encoded_data, punctured_data, frame, d_ofdm);
  /**## TODO: I think we are only using BPSK_1_2 for now -- can hard-code that to avoid the SWITCH ##**/
  /**   NOTE: If we use BPSK_1_2 then this just COPIES the data UNCHANGED -- so it is really a NOP **
   //void puncturing(const char *in, char *out , frame_param *frame, ofdm_param *ofdm)
   {
   int mod;
   int oidx = 0;
   for (int i = 0; i < d_frame.n_data_bits * 2; i++) {
   switch(d_ofdm.encoding) {
   case BPSK_1_2:
   case QPSK_1_2:
   case QAM16_1_2:
   punctured_data[oidx] = encoded_data[i];
   oidx++;
   break;
	    
   case QAM64_2_3:
   if (i % 4 != 3) {
   punctured_data[oidx] = encoded_data[i];
   oidx++;
   }
   break;
	    
   case BPSK_3_4:
   case QPSK_3_4:
   case QAM16_3_4:
   case QAM64_3_4:
   mod = i % 6;
   if (!(mod == 3 || mod == 4)) {
   punctured_data[oidx] = encoded_data[i];
   oidx++;
   }
   break;
   defaut:
   assert(false);
   break;
   }
   }
   }**/
  // EFFECTIVELY: punctured_data = encoded_data
  DEBUG({
      int di_row = 0;
      int symbols_len = d_frame.n_data_bits;
      printf("\npunctured out:\n%6u : ", di_row);
      for (int di = 0; di < symbols_len; di++) {
	printf("%1x", punctured_data[di]);
	if ((di % 128) == 127) {
	  di_row++;
	  printf("\n%6u : ", di_row);
	} else if ((di % 8) == 7) {
	  printf(" ");
	}
      }
      printf("\n\n");
    });
	
  //std::cout << "punctured" << std::endl;
  // interleaving
  //     interleave(punctured_data, interleaved_data, frame, d_ofdm);
  interleave(punctured_data, interleaved_data, d_frame.n_sym, d_ofdm.n_cbps, d_ofdm.n_bpsc, false);
  /* //std::cout << "interleaved" << std::endl; */
  DEBUG({
      int di_row = 0;
      int symbols_len = d_frame.n_sym * 48; // 24528
      printf("\ninterleaved out:\n%6u : ", di_row);
      for (int di = 0; di < symbols_len; di++) {
	printf("%1x", interleaved_data[di]);
	if ((di % 128) == 127) {
	  di_row++;
	  printf("\n%6u : ", di_row);
	} else if ((di % 8) == 7) {
	  printf(" ");
	}
      }
      printf("\n\n");
    });

  // one byte per symbol
  //     split_symbols(interleaved_data, symbols, frame, d_ofdm);
  //     void split_symbols(const char *in, char *out, frame_param &frame, ofdm_param &ofdm)
  {
    int n_symbols = d_frame.n_sym * 48;
    int idx = 0;
    for (int i = 0; i < n_symbols; i++) {
      symbols[i] = 0;
      for(int k = 0; k < d_ofdm.n_bpsc; k++) {
	assert(interleaved_data[idx] == 1 || interleaved_data[idx] == 0);
	symbols[i] |= (interleaved_data[idx] << k);
	idx++;
      }
    }
  }
	  
  d_symbols_len = d_frame.n_sym * 48; // 24528
  //assert(d_symbols_len == 24528);
  DEBUG(printf("d_symbols_len = %u * 48 = %u\n", d_frame.n_sym, d_symbols_len); fflush(stdout));

      	  
  /* d_symbols = (char*)calloc(d_symbols_len, 1); */
  /* std::memcpy(d_symbols, symbols, d_symbols_len); */
  for (int di = 0; di < d_symbols_len; di++) {
    d_symbols[di] = symbols[di];
  }
  //printf("d_symbols_len = %u * 48 = %u\n", d_frame.n_sym, d_symbols_len); fflush(stdout);
  DEBUG({
      int di_row = 0;
      printf("\nd_symbols out:\n%6u : ", di_row);
      for (int di = 0; di < d_symbols_len; di++) {
	printf("%1x", d_symbols[di]);
	if ((di % 128) == 127) {
	  di_row++;
	  printf("\n%6u : ", di_row);
	} else if ((di % 8) == 7) {
	  printf(" ");
	}
      }
      printf("\n\n");
    });

      
  int i = d_symbols_len - d_symbols_offset;
  DEBUG(printf("output i = %u :  d_sym = %p and d_sym_off = %u\n", i, (void*)d_symbols, d_symbols_offset));
  for (int di = 0; di < i; di++) {
    d_map_out[di] = d_symbols[d_symbols_offset + di];
    DEBUG(if (di < 16) { printf("%2u : d_map_out[%2u] = %1x : d_symbols[%2u] = %1x\n", i, di, d_map_out[di], (d_symbols_offset + di), d_symbols[d_symbols_offset + di]); });
  }
  d_symbols_offset += i;

  if(d_symbols_offset == d_symbols_len) {
    d_symbols_offset = 0;
    DEBUG(printf("reset d_symbols_offset to 0\n"));
  }

  DEBUG({
      int di_row = 0;
      printf("\nMapper out:\n%6u : ", di_row);
      for (int di = 0; di < d_frame.n_encoded_bits /*noutput*/; di++) {
	printf("%1x", d_map_out[di]);
	if ((di % 128) == 127) {
	  di_row++;
	  printf("\n%6u : ", di_row);
	} else if ((di % 8) == 7) {
	  printf(" ");
	}
      }
      printf("\n");
      fflush(stdout);
    });
  return i;
}



/**##     4. Packet-Header Generator : gnuradio/gr-digital/lib/packet_headergenerator_bb_impl.cc    ##**/
/* Documentation notes:
 * Encodes the header information in the given tags into bits and places them into out
 *
 * Uses the following header format:
 * Bits 0-11: The packet length (what was stored in the tag with key \p len_tag_key)
 * Bits 12-23: The header number (counts up everytime this function is called)
 * Bit 24-31: 8-Bit CRC
 * All other bits: Are set to zero
 *
 * If the header length is smaller than 32, bits are simply left out. For this
 * reason, they always start with the LSB.
 *
 * However, it is recommended to stay above 32 Bits, in order to have a working CRC.
 */

//GLOBALS:
unsigned int d_header_number;
unsigned int d_header_len;
uint32_t d_mask;
uint32_t d_seed;
uint32_t d_lfsr_len;
uint32_t d_shift_register;
#define  d_shift_register_length     32


uint32_t popCount(uint32_t x)
{
  uint32_t r = (x - ((x >> 1) & 033333333333)
		- ((x >> 2) & 011111111111));
  return ((r + (r >> 3)) & 030707070707) % 63;
}

// NOTE: This routine is not used!
void init_d_scramble_mask(uint32_t mask, uint32_t seed, uint32_t reg_len)
{
  d_mask = mask;
  d_seed = seed;
  d_lfsr_len = reg_len;
  d_shift_register = 0;
  //d_scramble_mask(d_header_len, 0)
  // Init scrambler mask
#if(0)
  if (scramble_header) { // I Think this is always true or false for us -- probably TR
    // These are just random values which already have OK PAPR:
    // gr::digital::lfsr shift_reg(0x8a, 0x6f, 7); :: ~/gnuradio/gnuradio-3.7.9.3/gr-digital/include/gnuradio/digital/lfsr.h
    /* Fibonacci Linear Feedback Shift Register using specified polynomial mask
     *
     * Generates a maximal length pseudo-random sequence of length 2^degree-1
     * Constructor: digital::lfsr(int mask, int seed, int reg_len);
     * mask - polynomial coefficients representing the locations of feedback taps from a shift register
     *             which are xor'ed together to form the new high order bit.
     *             Some common masks might be:  x^4 + x^3 + x^0 = 0x19
     *              x^5 + x^3 + x^0 = 0x29      x^6 + x^5 + x^0 = 0x61
     *
     * seed - the initialization vector placed into the register durring initialization. Low order bit
     *             corresponds to x^0 coefficient -- the first to be shifted as output.
     *
     * reg_len - specifies the length of the feedback shift register to be used. Durring each iteration, the
     *             register is rightshifted one and the new bit is placed in bit reg_len. reg_len should generally be
     *             at least order(mask) + 1
     *
     * see http://en.wikipedia.org/wiki/Linear_feedback_shift_register
     * for more explanation.
     *
     *  next_bit() - Standard LFSR operation
     *
     *      Perform one cycle of the LFSR.  The output bit is taken from the shift register LSB.  
     *          The shift register MSB is assigned from the modulo 2 sum of the masked shift register.
     */
    for (int i = 0; i < d_header_len; i++) {
      for (int k = 0; k < bits_per_header_sym; k++) {
	unsigned char next_bit;
	// unsigned char next_bit()
	{
	  unsigned char output = d_shift_register & 1;
	  unsigned char newbit = popCount( d_shift_register & d_mask )%2;
	  d_shift_register = ((d_shift_register>>1) | (newbit<<d_shift_register_length));
	  next_bit = output;
	}
	//d_scramble_mask[i] ^= shift_reg.next_bit() << k;
	d_scramble_mask[i] ^= next_bit << k;
      }
    }
  }
#endif
}


/* FROM dsrc/gr-ieee802-11/lib/signal_field.cc */

int get_bit(int b, int i) {  // Is this really an efficient way to do this?
  return (b & (1 << i) ? 1 : 0); // This is one shift, one and, one compare, and a branch.
  //return ((b >> i) & 0x1); // Isn't this a better way?  One shift, one and
}


void generate_signal_field(char *out) { // , frame_param &frame, ofdm_param &ofdm) {
  //data bits of the signal header
  //char *signal_header = (char *) malloc(sizeof(char) * 24);
  char signal_header[24];
  //signal header after...
  //convolutional encoding
  //char *encoded_signal_header = (char *) malloc(sizeof(char) * 48);
  char encoded_signal_header[48];
  //interleaving
  //char *interleaved_signal_header = (char *) malloc(sizeof(char) * 48);
  //char interleaved_signal_header[48]; -- this writes into "out"

  int length = d_frame.psdu_size;

  // first 4 bits represent the modulation and coding scheme
  signal_header[ 0] = get_bit(d_ofdm.rate_field, 3);
  signal_header[ 1] = get_bit(d_ofdm.rate_field, 2);
  signal_header[ 2] = get_bit(d_ofdm.rate_field, 1);
  signal_header[ 3] = get_bit(d_ofdm.rate_field, 0);
  // 5th bit is reserved and must be set to 0
  signal_header[ 4] = 0;
  // then 12 bits represent the length
  signal_header[ 5] = get_bit(length,  0);
  signal_header[ 6] = get_bit(length,  1);
  signal_header[ 7] = get_bit(length,  2);
  signal_header[ 8] = get_bit(length,  3);
  signal_header[ 9] = get_bit(length,  4);
  signal_header[10] = get_bit(length,  5);
  signal_header[11] = get_bit(length,  6);
  signal_header[12] = get_bit(length,  7);
  signal_header[13] = get_bit(length,  8);
  signal_header[14] = get_bit(length,  9);
  signal_header[15] = get_bit(length, 10);
  signal_header[16] = get_bit(length, 11);
  //18-th bit is the parity bit for the first 17 bits
  int sum = 0;
  for(int i = 0; i < 17; i++) {
    if(signal_header[i]) {
      sum++;
    }
  }
  signal_header[17] = sum % 2;

  // last 6 bits must be set to 0
  for (int i = 0; i < 6; i++) {
    signal_header[18 + i] = 0;
  }

  ofdm_param signal_ofdm; //(BPSK_1_2);
  signal_ofdm.encoding = BPSK_1_2;
  signal_ofdm.n_bpsc = 1;
  signal_ofdm.n_cbps = 48;
  signal_ofdm.n_dbps = 24;
  signal_ofdm.rate_field = 0x0D; // 0b00001101

  frame_param signal_param; // (signal_ofdm, 0);
  signal_param.psdu_size = 0;
  signal_param.n_sym = (int) ceil((16 + 8 * signal_param.psdu_size + 6) / (double) signal_ofdm.n_dbps);
  signal_param.n_data_bits = signal_param.n_sym * signal_ofdm.n_dbps;
  signal_param.n_pad = signal_param.n_data_bits - (16 + 8 * signal_param.psdu_size + 6);
  signal_param.n_encoded_bits = signal_param.n_sym * signal_ofdm.n_cbps;

  // convolutional encoding (scrambling is not needed)
  convolutional_encoding(signal_header, encoded_signal_header, signal_param.n_data_bits);
  // interleaving
  interleave(encoded_signal_header, out, signal_param.n_sym, signal_ofdm.n_cbps, signal_ofdm.n_bpsc, false);

  /* free(signal_header); */
  /* free(encoded_signal_header); */
  /* free(interleaved_signal_header); */
}

bool signal_field_header_formatter(long packet_len, unsigned char *out) //, const std::vector<tag_t> &tags)
{
  /* bool encoding_found = false; */
  /* bool len_found = false; */
  /* int encoding = 0; */
  /* int len = 0; */
  DEBUG(printf("In signal_field header_formatter with packet_len = %lu\n", packet_len); fflush(stdout));
  /* // read tags */
  /* for(int i = 0; i < tags.size(); i++) { */
  /*   if(pmt::eq(tags[i].key, pmt::mp("encoding"))) { */
  /*     encoding_found = true; */
  /*     encoding = pmt::to_long(tags[i].value); */
  /*   } else if(pmt::eq(tags[i].key, pmt::mp("psdu_len"))) { */
  /*     len_found = true; */
  /*     len = pmt::to_long(tags[i].value); */
  /*   } */
  /* } */

  /* // check if all tags are present */
  /* if( (!encoding_found) || (!len_found)) { */
  /*   return false; */
  /* } */

  /* ofdm_param ofdm((Encoding)encoding); */
  /* frame_param frame(ofdm, len); */

  generate_signal_field((char*)out); //, d_frame, d_ofdm);
  return true;
}

// From     int packet_headergenerator_bb_impl::work (int noutput_items,
int do_packet_header_gen(unsigned int packet_len, uint8_t* out ) // int noutput_items, int ninput_items, uint8_t* input_items, uint8_t* output_items)
{
  //## From     int packet_headergenerator_bb_impl::work (int noutput_items,
  //  NOTE: Need to know what type of header -- default or OFDM (?) as the
  //    OFDM header does a scramble pass after the default header setup.
  //    I'm pretty sure we are using OFDM here... BUT MAYBE NOT OFDM HEADERS?
  /* I'm inlining this call, as it just calles gneerate_signal_field 
     signal_field_header_formatter(packet_len, out); // , const std::vector<tag_t> &tags)
  */
  generate_signal_field((char*)out); //, d_frame, d_ofdm);
  return 48; // this is the length of the output header -- the convolutional encoded and interleaved header...
}


/** this is from gr-digital/lib/consteallation.cc **/
/* void  constellation_calc_arity(int size, int dimensionality) */
/* { */
/*   if (size() % dimensionality != 0) { */
/*     // throw std::runtime_error("Constellation vector size must be a multiple of the dimensionality."); */
/*     printf(" constellation_calc_arity : Constellation vector size must be a multiple of the dimensionality.\n"); */
/*     exit(-5); */
/*   } */
/*   d_arity = size()/dimensionality; */
/* } */

/** This is from constellations_impl.cc **/

/* constellation_bpsk_impl() */
/* { */
/*   d_constellation.resize(2); */
/*   d_constellation[0] = gr_complex(-1, 0); */
/*   d_constellation[1] = gr_complex(1, 0); */
/*   d_rotational_symmetry = 2; */
/*   d_dimensionality = 1; */
/*  // calc_arity(); */
/*  // d_arity = size/dimensionality; */
/*   d_arity = 2/1 = 2; */
/* } */


/* void */
/* constellation::map_to_points(unsigned int value, gr_complex *points) */
/* { */
/*   for(unsigned int i=0; i<d_dimensionality; i++) { */
/*     points[i] = d_constellation[value*d_dimensionality + i]; */
/*   } */
/* } */

/** This is from dsrc/gr-ieee802-11/chunks_to_synbols_impl.cc **/
// NOTE: I beleive that BPSK mapping is 0->-1 and 1->1 (i.e. 0 -> -1+0i and 1 -> 1+0i
//    FOR the IEEE802-11 Version of chunks-to-symbols.
// NOTE: The others are NOT so simple...  CURRENTLY WE ARE ONLY USING BPSK_1_2
// THIS is for the PAYLOAD (which uses the gr-ieee802-11 version of chunk_to_symbols
// The header part seems to use BPSK always (regardless of underlying payload version)?

/* int */
/* do_signal_chunks_to_symbols(int noutput_items, */
/* 			    int ninput_items, */
/* 			    uint8_t* input_items, */
/* 			    float*   output_items) // really complex numbers, so real, imag pairs */
/* {   */
/*   /\* const unsigned char *in = (unsigned char*)input_items[0]; *\/ */
/*   /\* gr_complex *out = (gr_complex*)output_items[0]; *\/ */

/*   /\* std::vector<tag_t> tags; *\/ */
/*   /\* get_tags_in_range(tags, 0, nitems_read(0), *\/ */
/*   /\* 		    nitems_read(0) + ninput_items[0], *\/ */
/*   /\* 		    pmt::mp("encoding")); *\/ */
/*   /\* if(tags.size() != 1) { *\/ */
/*   /\*   throw std::runtime_error("no encoding in input stream"); *\/ */
/*   /\* } *\/ */

/*   Encoding encoding = d_ofdm.encoding; */

/*   switch (encoding) { */
/*   case BPSK_1_2: */
/*   case BPSK_3_4: */
/*     d_mapping = d_bpsk; */
/*     break; */

/*   case QPSK_1_2: */
/*   case QPSK_3_4: */
/*     d_mapping = d_qpsk; */
/*     break; */

/*   case QAM16_1_2: */
/*   case QAM16_3_4: */
/*     d_mapping = d_16qam; */
/*     break; */

/*   case QAM64_2_3: */
/*   case QAM64_3_4: */
/*     d_mapping = d_64qam; */
/*     break; */

/*   default: */
/*     printf("signal_chucks_to_symbols : bad encoding in d_ofdm: %u\n", encoding); //throw std::invalid_argument("wrong encoding"); */
/*     exit(-4); */
/*     break; */
/*   } */

/*   for(int i = 0; i < ninput_items; i++) { */
/*     d_mapping->map_to_points(input_items, out + i); */
/*   } */

/*   return ninput_items[0]; */
/* } */




/** THIS code comes from gr-digital/ofdm_carrier_allocator_cvc_impl.cc **/

#define d_fft_len   64
#define d_fft_logn   6
  

#define d_num_pilot_carriers       1
#define d_size_pilot_carriers_val  4
const int d_size_pilot_carriers[d_num_pilot_carriers] = { d_size_pilot_carriers_val };
int d_pilot_carriers[d_num_pilot_carriers][d_size_pilot_carriers_val] = {{-21, -7, 7, 21}};


#define d_num_pilot_symbols   127
#define d_size_pilot_symbols    4
float d_pilot_symbols_real[d_num_pilot_symbols][d_size_pilot_symbols] = {
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{ 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{-1.0, -1.0, -1.0,  1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, { 1.0,  1.0,  1.0, -1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, 
{-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}, {-1.0, -1.0, -1.0,  1.0}};

float d_pilot_symbols_imag[d_num_pilot_symbols][d_size_pilot_symbols] = {
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, 
{ 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}, { 0.0,  0.0,  0.0,  0.0}};

#define d_num_occupied_carriers    1
#define d_size_occupied_carriers  48
int d_occupied_carriers[d_num_occupied_carriers][d_size_occupied_carriers] = {{-26, -25, -24, -23, -22, -20, -19, -18,   //  8
									       -17, -16, -15, -14, -13, -12, -11, -10,   // 16
									       -9 ,  -8,  -6,  -5,  -4,  -3,  -2,  -1,   // 24
									       1  ,   2,   3,   4,   5,   6,   8,   9,   // 32
									       10 ,  11,  12,  13,  14,  15,  16,  17,   // 40
									       18 ,  19,  20,  22,  23,  24,  25,  26}}; // 48

//int d_occupied_carriers[d_num_occupied_carriers][d_size_occupied_carriers];

#define d_num_sync_words    4
#define d_size_sync_words 128
float d_sync_words_real[d_num_sync_words][d_size_sync_words] = {{0.0, 0.0, 0.0, 0.0,  //   4
								 0.0, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0, // 32
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,  // 64
								 0.0, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0, // 96
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0}, // 128
								{0.0, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 1.4719601443879746, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0},
								{0, 0, 0, 0,
								 0, 0, -1, 0,
								 -1, 0, -1, 0,
								 -1, 0, 1, 0,
								 1, 0, -1, 0,
								 1, 0, 1, 0,
								 1, 0, -1, 0,
								 1, 0, -1, 0,
								 0, 0, 1, 0,
								 1, 0, 1, 0,
								 -1, 0, 1, 0,
								 -1, 0, 1, 0,
								 1, 0, 1, 0,
								 -1, 0, 1, 0,
								 1, 0, -1, 0,
								 0, 0, 0, 0},
								{0, 0, 0, 0,
								 0, 0, 1, 1,
								 -1, -1, 1, 1,
								 -1, 1, -1, 1,
								 1, 1, 1, 1,
								 1, -1, -1, 1,
								 1, -1, 1, -1,
								 1, 1, 1, 1,
								 0, 1, -1, -1,
								 1, 1, -1, 1,
								 -1, 1, -1, -1,
								 -1, -1, -1, 1,
								 1, -1, -1, 1,
								 -1, 1, -1, 1,
								 1, 1, 1, 0,
								 0, 0, 0, 0}};

float d_sync_words_imag[d_num_sync_words][d_size_sync_words] = {{0.0, 0.0, 0.0, 0.0,  //   4
								 0.0, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0, // 32
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,  // 64
								 0.0, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0, // 96
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0}, // 128
								{0.0, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 -1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 +1.4719601443879746, 0.0, 0.0, 0.0,
								 0.0, 0.0, 0.0, 0.0},
								{0, 0, 0, 0,
								 0, 0, 0, 1,
								 0, 1, 0, 1,
								 0, -1, 0, 1,
								 0, -1, 0, 1,
								 0, 1, 0, 1,
								 0, 1, 0, -1,
								 0, -1, 0, 1,
								 0, -1, 0, -1,
								 0, -1, 0, 1,
								 0, -1, 0, -1,
								 0, 1, 0, 1,
								 0, 1, 0, 1,
								 0, -1, 0, 1,
								 0, -1, 0, 0,
								 0, 0, 0, 0},
								{0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0,
								 0, 0, 0, 0}};

#define d_output_is_shifted true

void
init_ofdm_carrier_allocator()
{
  DEBUG(printf("In ofdm_carrier_allocator_cvc_impl constructor\n"); fflush(stdout));

  DEBUG(printf("\nSetting up the occupied carriers\n"));
  for (unsigned i = 0; i < d_num_occupied_carriers; i++) {
    for (unsigned j = 0; j < d_size_occupied_carriers; j++) {
      //d_occupied_carriers[i][j] = b_occupied_carriers[i][j]; // Copy base into ours for use.

      if (d_occupied_carriers[i][j] < 0) {  // Modify the negatives by fft_offset
	d_occupied_carriers[i][j] += d_fft_len;
      }
      if (d_output_is_shifted) { // And with shifted output, re-base the center axis
	d_occupied_carriers[i][j] = (d_occupied_carriers[i][j] + d_fft_len/2) % d_fft_len;
      }
      DEBUG(printf(" init d_occpd_car[%2u][%2u] = %u\n", i, j, d_occupied_carriers[i][j]));
    }
  }
  
  DEBUG(printf("\nSetting up the pilot carriers\n"));
  for (unsigned i = 0; i < d_num_pilot_carriers; i++) {
    for (unsigned j = 0; j < d_size_pilot_carriers[i]; j++) {
      if (d_pilot_carriers[i][j] < 0) {
	d_pilot_carriers[i][j] += d_fft_len;
      }
      if (d_output_is_shifted) {
	d_pilot_carriers[i][j] = (d_pilot_carriers[i][j] + d_fft_len/2) % d_fft_len;
      }
      DEBUG(printf(" init d_pilot_car[%2u][%2u] = %u\n", i, j, d_pilot_carriers[i][j]));
    }
  }
  DEBUG(printf("\n"));
}


int
do_ofdm_carrier_allocator_cvc_impl_work (int noutput_items,
					 int ninput_items,
					 float* in_real,  float* in_imag,   // complex numbers
					 float* out_real, float* out_imag)  // complex numbers
{
  /* const gr_complex *in = (const gr_complex *) input_items[0]; */
  /* gr_complex *out = (gr_complex *) output_items[0]; */
  /* std::vector<tag_t> tags; */

  DEBUG(printf("In ofdm_carrier_allocator_cvc_impl work with nout = %u and d_num_sync_words = %u\n", noutput_items, d_num_sync_words); fflush(stdout));
  // Reset the contents of the output_items to 0x00 (so any not over-written remain 0x00?)
  //memset((void *) out_real, 0x00, sizeof(float) * d_fft_len * noutput_items);
  //memset((void *) out_imag, 0x00, sizeof(float) * d_fft_len * noutput_items);
  for (int ti = 0; ti < d_fft_len * (noutput_items + d_num_sync_words); ti++) {
    out_real[ti] = 0.0;
    out_imag[ti] = 0.0;
  }

  // Copy Sync word
  DEBUG(printf("\nCopy sync words...\n"));
  int o_offset = 0;
  for (unsigned i = 0; i < d_num_sync_words; i++) {
    //memcpy((void *) out, (void *) &d_sync_words[i][0], sizeof(gr_complex) * d_fft_len);
    int oidx = o_offset;
    for (int ti = 0; ti < d_fft_len; ti++) {
      out_real[oidx] = d_sync_words_real[i][ti];
      out_imag[oidx] = d_sync_words_imag[i][ti];
      DEBUG(printf(" out[%4u] = %19.16f + %19.16f i = d_sync_words[%u][%2u] = %19.16f + %19.16f i\n", oidx, out_real[oidx], out_imag[oidx], i, ti, d_sync_words_real[i][ti], d_sync_words_imag[i][ti]));
      oidx++;
    }
    DEBUG(printf("\n"));
    //out += d_fft_len;
    o_offset += d_fft_len;
  }

  // Copy data symbols
  long n_ofdm_symbols = 0; // Number of output items
  int curr_set = 0;
  int symbols_to_allocate = d_size_occupied_carriers;
  int symbols_allocated = 0;
  DEBUG(printf("\nCopy data symbols -- symbols_to_allocate = %u\n", symbols_to_allocate));
  for (int i = 0; i < ninput_items; i++) {
    if (symbols_allocated == 0) {
      /*   // Copy all tags associated with these input symbols onto this OFDM symbol */
      /*   get_tags_in_range(tags, 0, */
      /* 			nitems_read(0)+i, */
      /* 			nitems_read(0)+std::min(i+symbols_to_allocate, (int) ninput_items[0]) */
      /* 			); */
      /*   for (unsigned t = 0; t < tags.size(); t++) { */
      /* 	add_item_tag( */
      /* 		     0, */
      /* 		     nitems_written(0) + n_ofdm_symbols + (n_ofdm_symbols == 0 ? 0 : d_sync_words.size()), */
      /* 		     tags[t].key, */
      /* 		     tags[t].value */
      /* 		     ); */
      /*   } */
      n_ofdm_symbols++;
    }
    int o_idx = o_offset + (n_ofdm_symbols-1) * d_fft_len + d_occupied_carriers[curr_set][symbols_allocated];
    out_real[o_idx] = in_real[i];
    out_imag[o_idx] = in_imag[i];
    DEBUG(printf("  DS: off %u n_sym %lu curr_set %u sym_alloc %u d_o %d :  out[%u] = %f + %f i\n", o_offset, n_ofdm_symbols, curr_set, symbols_allocated, d_occupied_carriers[curr_set][symbols_allocated], o_idx, in_real[i], in_imag[i]));
    symbols_allocated++;
    if (symbols_allocated == symbols_to_allocate) {
      curr_set = (curr_set + 1) % d_num_occupied_carriers; //.size();
      symbols_to_allocate = d_size_occupied_carriers; //[curr_set]; //.size();
      symbols_allocated = 0;
    }
  }

  DEBUG(printf("\nCopy pilot symbols: n_ofdm_symbols = %lu\n", n_ofdm_symbols));
  // Copy pilot symbols
  for (int i = 0; i < n_ofdm_symbols; i++) {
    for (unsigned k = 0; k < d_size_pilot_carriers[i % d_num_pilot_carriers]; k++) {
      int pcidx = i % d_num_pilot_carriers;
      int psidx = i % d_num_pilot_symbols;
      //int oidx = o_offset + i * d_fft_len + d_pilot_carriers[i % d_num_pilot_carriers][k];
      int oidx = o_offset + i * d_fft_len + d_pilot_carriers[pcidx][k];
      out_real[oidx] = d_pilot_symbols_real[psidx][k];
      out_imag[oidx] = d_pilot_symbols_imag[psidx][k];
      DEBUG(printf("  PS: i %3u k %u d_p_c[%u][%u] = %4d d_p_s[%u][%u] : out[%6u] = %4.1f + %4.1f i\n", i, k, pcidx, k, d_pilot_carriers[pcidx][k], psidx, k, oidx, d_pilot_symbols_real[psidx][k], d_pilot_symbols_imag[psidx][k]));
    }
  }
  DEBUG(printf("\nReturning %lu (symbols + sync_words)\n", n_ofdm_symbols + d_num_sync_words));
  return n_ofdm_symbols + d_num_sync_words;
}







void
do_xmit_fft_work(int n_inputs, float scale, float *input_real, float * input_imag, float* output_real, float*output_imag)
{
  // Do the FFT in 64-entry windows, and add the "shift" operation to each
  //   Also add the weighting/scaling for the window
  float fft_in_real[64];
  float fft_in_imag[64];
  bool inverse = true;
  bool shift = true;
  bool swap_odd_signs = false; // We shift the inputs instead?
  int  size = d_fft_len;
  int  log_size = d_fft_logn;
  float recluster[2] = {1.0, 1.0}; // used to alter sign of "odd" fft results
  if (swap_odd_signs) {
    recluster[1] = -1.0;
  }
  DEBUG(printf("Starting do_xmit_fft_work with size %u inverse %u shift %u on n_inputs %u\n", size, inverse, shift, n_inputs));
  for (int k = 0; k < (n_inputs+(size-1)); k += size) {

    DEBUG(printf(" Prepping for FFT call starting at %u\n", k));
    // Set up the (scaled) inputs
    if (shift) {
      for (int i = 0; i < size/2; i++) {
	fft_in_real[32+i] = input_real[k + i] * scale;    // Copy  0 .. 31 into 32 .. 63
	fft_in_imag[32+i] = input_imag[k + i] * scale;    // Copy  0 .. 31 into 32 .. 63
	fft_in_real[i] = input_real[k + 32 + i] * scale;  // Copy 32 .. 63 into  0 .. 31
	fft_in_imag[i] = input_imag[k + 32 + i] * scale;  // Copy 32 .. 63 into  0 .. 31
	DEBUG(if (k == 0) {
	    printf("  set %u IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, 32+i, scale, input_real[k+i], input_imag[k+i], fft_in_real[32+i], fft_in_imag[32+i]);
	    printf("  set %u IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, i, scale, input_real[k+32+i], input_imag[k+32+i], fft_in_real[i], fft_in_imag[i]);
	  });
      }      
    } else {
      for (int i = 0; i < size; i++) {
	fft_in_real[i] = input_real[k + i] * scale;
	fft_in_imag[i] = input_imag[k + i] * scale;
	DEBUG(if (k == 0) {
	    printf(" set %u IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, i, scale, input_real[k+i], input_imag[k+i], fft_in_real[i], fft_in_imag[i]);
	  });
      }
    }
    DEBUG(if (k < 256) {
	printf("\n Iteration %u inputs (in order):\n", k);
	for (int i = 0; i < size; i++) {
	  printf("  FFT_%u_IN[ %2u ] = %11.8f * ( %11.8f + %11.8f i) = %11.8f + %11.8f i\n", k, i, scale, input_real[k+i], input_imag[k+i], fft_in_real[i], fft_in_imag[i]);
	}
	printf("\nCalling FFT function with inverse = %u size = %u\n", inverse, size);
      });
    // NOTE: This version over-writes the input data with output data
    fft_ri(fft_in_real, fft_in_imag, inverse, false, size, log_size);
    for (int i = 0; i < size; i++) {
      // Swap sign on the "odd" FFT results (re-cluster energy around zero?)
      output_real[k + i] = recluster[i&0x1]*fft_in_real[i];
      output_imag[k + i] = recluster[i&0x1]*fft_in_imag[i];
    }

    DEBUG(if (k < 256) {
	for (int i = 0; i < size; i++) {
	  printf(" FFT_%u_OUT[ %2u : %5u ] = %11.8f + %11.8f i\n", k, i, k+i, output_real[k+i], output_imag[k+i]);
	}
      });

    DEBUG(if (k < 4) { printf("\n");} );
  }
  DEBUG(printf(" Done with fft calls... output:\n"));
}




/** This code is adapted from gr-digital/ofdm_cyclic_prefixer.cc **/
#define  d_rolloff_len   2
#define  d_cp_size      16

void
do_ofdm_cyclic_prefixer_impl_work(int n_symbols, float * in_real, float* in_imag, float* out_real, float* out_imag)
{
  int d_input_size = 64;
  int d_output_size = d_input_size + d_cp_size;
  float d_up_flank[d_rolloff_len - 1];
  float d_down_flank[d_rolloff_len - 1];

  float d_delay_line_real[d_rolloff_len - 1];
  float d_delay_line_imag[d_rolloff_len - 1];
  
  int symbols_to_read = n_symbols; // std::min(noutput_items / (int) d_output_size, ninput_items[0]);

  // The actual flanks are one sample shorter than d_rolloff_len, because the
  // first sample of the up- and down flank is always zero and one, respectively
  for (int i = 1; i < d_rolloff_len; i++) {
    d_up_flank[i-1]   = 0.5 * (1 + cos(M_PI * i/d_rolloff_len - M_PI));
    d_down_flank[i-1] = 0.5 * (1 + cos(M_PI * (d_rolloff_len-i)/d_rolloff_len - M_PI));
    d_delay_line_real[i-1] = 0; // Initialize delay line to zero
    d_delay_line_imag[i-1] = 0;
  }


  // 2) Do the cyclic prefixing and, optionally, the pulse shaping
  int out_offset = 0;
  int in_offset = 0;
  for (int sym_idx = 0; sym_idx < symbols_to_read; sym_idx++) {
    //memcpy((void *)(out + d_cp_size), (void *) in, d_fft_len * sizeof(gr_complex));
    for (int i = 0; i < d_fft_len; i++) {
      out_real[out_offset + i + d_cp_size] = in_real[in_offset + i];
      out_imag[out_offset + i + d_cp_size] = in_imag[in_offset + i];
    }
    //memcpy((void *) out, (void *) (in + d_fft_len - d_cp_size), d_cp_size * sizeof(gr_complex));
    for (int i = 0; i < d_cp_size; i++) {
      out_real[out_offset + i] = in_real[in_offset + i + d_fft_len - d_cp_size];
      out_imag[out_offset + i] = in_imag[in_offset + i + d_fft_len - d_cp_size];
    }
    //if (d_rolloff_len) { // always true in this case
    for (int i = 0; i < d_rolloff_len-1; i++) {
      out_real[out_offset + i] = out_real[out_offset + i] * d_up_flank[i] + d_delay_line_real[i];
      out_imag[out_offset + i] = out_imag[out_offset + i] * d_up_flank[i] + d_delay_line_imag[i];
      d_delay_line_real[i] = in_real[in_offset + i] * d_down_flank[i];
      d_delay_line_imag[i] = in_imag[in_offset + i] * d_down_flank[i];
    }
    //}
    in_offset  += d_fft_len;
    out_offset += d_output_size;
  }

  // 3) If we're in packet mode: (we are)
  //    - flush the delay line, if applicable
  //if (!d_length_tag_key_str.empty()) { // TRUE
  // if (d_rolloff_len) { // TRUE
  for (unsigned i = 0; i < (d_rolloff_len - 1); i++) {
    out_real[out_offset+i] = d_delay_line_real[i];
    out_imag[out_offset+i] = d_delay_line_imag[i];
  }
  //d_delay_line.assign(d_delay_line.size(), 0);
  //}

}


/********************************************************************************
 * This routine manages the initializations for all xmit pipeline components
 ********************************************************************************/
void
xmit_pipe_init() {
  d_scrambler = 1;
  crcInit();
  init_ofdm_parms(&d_ofdm, BPSK_1_2);

  init_ofdm_carrier_allocator();
}



/********************************************************************************
 * This routine manages the transmit pipeline functions and components
 ********************************************************************************/
#define MAX_SIZE 24600  // really 24576 ?


void
do_xmit_pipeline(int in_msg_len, char* in_msg, int* num_final_outs, float* final_out_real, float* final_out_imag)
{
  DEBUG(printf("In do_xmit_pipeline: MSG_LEN %u  MSG:\n", in_msg_len);
	for (int i = 0; i < in_msg_len; i++) {
	  printf("%c", in_msg[i]);
	}
	printf("\n"); fflush(stdout));
 #ifdef INT_TIME
  gettimeofday(&x_pipe_start, NULL);
 #endif
  int psdu_len = 0;
  //do_wifi_mac(in_msg_len, in_msg, &psdu_len);
  generate_mac_data_frame(in_msg, in_msg_len, &psdu_len);
 #ifdef INT_TIME
  gettimeofday(&x_genmacfr_stop, NULL);
  x_genmacfr_sec  += x_genmacfr_stop.tv_sec  - x_pipe_start.tv_sec;
  x_genmacfr_usec += x_genmacfr_stop.tv_usec - x_pipe_start.tv_usec;
 #endif

  //do_mapper_work(32768, psdu_len); // noutput always seems to be 32768 ? Actualy data size is 24528 ?
  do_mapper_work(psdu_len); // noutput always seems to be 32768 ? Actualy data size is 24528 ?
  // The mapper results in 24528 output bytes for a 1500 character input payload
 #ifdef INT_TIME
  gettimeofday(&x_domapwk_stop, NULL);
  x_domapwk_sec  += x_domapwk_stop.tv_sec  - x_genmacfr_stop.tv_sec;
  x_domapwk_usec += x_domapwk_stop.tv_usec - x_genmacfr_stop.tv_usec;
 #endif

  int mapper_payload_size = d_frame.n_encoded_bits;
  uint8_t pckt_hdr_out[64]; // I think this only needs to be 48 bytes...
  int pckt_hdr_len = do_packet_header_gen(mapper_payload_size, pckt_hdr_out);
 #ifdef INT_TIME
  gettimeofday(&x_phdrgen_stop, NULL);
  x_phdrgen_sec  += x_phdrgen_stop.tv_sec  - x_domapwk_stop.tv_sec;
  x_phdrgen_usec += x_phdrgen_stop.tv_usec - x_domapwk_stop.tv_usec;
 #endif
  DEBUG(printf("packet_header = ");
	for (int i = 0; i < pckt_hdr_len; i++) {
	  printf("%1x ", pckt_hdr_out[i]);
	}
	printf("\n"));

  // Convert the header chunks to symbols (uses simple BPSK_1_2 map: 0 -> -1+0i and 1 -> +1+0i)
  // Convert the payload chunks to symbols (for now also using simple BPSK_1_2 map: 0 -> -1+0i and 1 -> +1+0i)
  // We will also do the Tagged Stream Mux functionality (concatenate the Payload after the Header)
  DEBUG(printf("\nConverting to chunks, and doing the tagged stream mux stuff...\n"));
 #ifdef INT_TIME
  gettimeofday(&x_ck2sym_start, NULL);
 #endif
  float msg_stream_real[MAX_SIZE];
  float msg_stream_imag[MAX_SIZE];
  int   msg_idx = 0;
  float bpsk_chunks2sym[2] = {-1.0, 1.0};
  for (int i = 0; i < pckt_hdr_len; i++) {
    msg_stream_real[msg_idx] = bpsk_chunks2sym[pckt_hdr_out[i]];
    msg_stream_imag[msg_idx] = 0.0;
    //    DEBUG(printf("HDR: msg_stream[%2u] = %4.1f + %4.1f\n", msg_idx, msg_stream_real[msg_idx], msg_stream_imag[msg_idx]));
    msg_idx++;
  }
  //  printf("\n");
  for (int i = 0; i < mapper_payload_size; i++) {
    msg_stream_real[msg_idx] = bpsk_chunks2sym[d_map_out[i]];
    msg_stream_imag[msg_idx] = 0.0;
    //    DEBUG(printf("PYLD: msg_stream[%4u] = %4.1f + %4.1f\n", msg_idx, msg_stream_real[msg_idx], msg_stream_imag[msg_idx]));
    msg_idx++;
  }
  //  printf("\n");
  // This is to clear any left-over storage locations...
  for (int i = msg_idx; i < MAX_SIZE; i++) {
    msg_stream_real[i] = 0.0;
    msg_stream_imag[i] = 0.0;
    //    DEBUG(printf("LAST: msg_stream[%4u] = %4.1f + %4.1f\n", i, msg_stream_real[i], msg_stream_imag[i]));
  }    
 #ifdef INT_TIME
  gettimeofday(&x_ck2sym_stop, NULL);
  x_ck2sym_sec  += x_ck2sym_stop.tv_sec  - x_ck2sym_start.tv_sec;
  x_ck2sym_usec += x_ck2sym_stop.tv_usec - x_ck2sym_start.tv_usec;
 #endif
  DEBUG(printf("\nTagged Stream Mux output:\n");
	for (int i = 0; i < (pckt_hdr_len + mapper_payload_size); i++) {
	  printf(" TSM_OUT %5u : %4.1f %4.1f\n", i, msg_stream_real[i], msg_stream_imag[i]);
	});
  
  
  //DEBUG(printf("\nCalling do_ofdm_carrier_allocator_cvc_impl_work( %u, %u, msg_stream)\n", 520, 24576));
  DEBUG(printf("\nCalling do_ofdm_carrier_allocator_cvc_impl_work( %u, %u, msg_stream)\n", d_frame.n_sym, d_frame.n_encoded_bits));
  #define ofdm_max_out_size  33280 //520*64  // 33024   // Not sure why, though
  float ofdm_car_str_real[ofdm_max_out_size];
  float ofdm_car_str_imag[ofdm_max_out_size];
  
  //int ofc_res = do_ofdm_carrier_allocator_cvc_impl_work(520, 24576, msg_stream_real, msg_stream_imag, ofdm_car_str_real, ofdm_car_str_imag);
 #ifdef INT_TIME
  gettimeofday(&x_ocaralloc_start, NULL);
 #endif
  int ofc_res = do_ofdm_carrier_allocator_cvc_impl_work(d_frame.n_sym, d_frame.n_encoded_bits, msg_stream_real, msg_stream_imag, ofdm_car_str_real, ofdm_car_str_imag);
  DEBUG(printf(" return value was %u so max %u outputs\n", ofc_res, ofc_res*d_fft_len);
	printf(" do_ofdm_carrier_allocator_cvc_impl_work output:\n");
	for (int ti = 0; ti < (ofc_res * 64); ti++) {
	  printf("  ofdm_car %6u : %9.6f + %9.6f i\n", ti, ofdm_car_str_real[ti], ofdm_car_str_imag[ti]);
	});

 #ifdef INT_TIME
  gettimeofday(&x_ocaralloc_stop, NULL);
  x_ocaralloc_sec  += x_ocaralloc_stop.tv_sec  - x_ocaralloc_start.tv_sec;
  x_ocaralloc_usec += x_ocaralloc_stop.tv_usec - x_ocaralloc_start.tv_usec;
 #endif
  // The FFT operation...  This is where we are currently "broken"
  //   The outputs match for the first one or two 64-entry windows, and then diverge a lot...
  DEBUG(printf("\nCalling do_xmit_fft_work for %u data values\n", ofdm_max_out_size));
  int   n_ins = ofc_res * d_fft_len;  // max is ofdm_max_out_size
  float fft_out_real[ofdm_max_out_size];
  float fft_out_imag[ofdm_max_out_size];
  float scale = 1/sqrt(52.0);

  do_xmit_fft_work(n_ins, scale, ofdm_car_str_real, ofdm_car_str_imag, fft_out_real, fft_out_imag);
 #ifdef INT_TIME
  gettimeofday(&x_fft_stop, NULL);
  x_fft_sec  += x_fft_stop.tv_sec  - x_ocaralloc_stop.tv_sec;
  x_fft_usec += x_fft_stop.tv_usec - x_ocaralloc_stop.tv_usec;
 #endif
  DEBUG(for (int i = 0; i < n_ins; i++) {
      printf(" fft_out %6u : %11.8f + %11.8f i\n", i, fft_out_real[i], fft_out_imag[i]);
    });


  //#include "gold_fft_outputs.c"

  int num_cycpref_outs = ofc_res * (d_fft_len + d_cp_size) + 1;
  float cycpref_out_real[41360]; // Large enough
  float cycpref_out_imag[41360]; // Large enough
  DEBUG(printf("\nCalling do_ofdm_cyclic_prefixer_impl_work(%u, fft_output)\n", ofc_res));
  //do_ofdm_cyclic_prefixer_impl_work(ofc_res, gold_fft_out_real, gold_fft_out_imag, cycpref_out_real, cycpref_out_imag);
 /* #ifdef INT_TIME */
 /*  gettimeofday(&x_ocycpref_start, NULL); */
 /* #endif */
  do_ofdm_cyclic_prefixer_impl_work(ofc_res, fft_out_real, fft_out_imag, cycpref_out_real, cycpref_out_imag);
 #ifdef INT_TIME
  gettimeofday(&x_ocycpref_stop, NULL);
  x_ocycpref_sec  += x_ocycpref_stop.tv_sec  - x_fft_stop.tv_sec;
  x_ocycpref_usec += x_ocycpref_stop.tv_usec - x_fft_stop.tv_usec;
 #endif
  DEBUG(for (int i = 0; i < num_cycpref_outs; i++) {
      printf(" ocypref_out %6u : %11.8f + %11.8f i\n", i, cycpref_out_real[i], cycpref_out_imag[i]);
    }
    printf("\n"));
  
  // The next "stage" is the "packet_pad2" which adds 500 zeros to the front (and no zeros to the rear) of the output
  //   This block may also add some time-stamp tags (for UHD?) for GnuRadio use?
  //   Not sure we care about this padding?
  bool do_add_pre_pad = false;
  DEBUG(printf("\nAdd the pre-padding : %u\n", do_add_pre_pad));
  int num_pre_pad = do_add_pre_pad ? 500 : 0;
  int num_post_pad = 0;
  DEBUG(printf("\n"));

  // Now set the Final Outputs
  DEBUG(printf("\nFinal XMIT output:\n"));
  *num_final_outs = num_pre_pad + num_cycpref_outs + num_post_pad;
  for (int i = 0; i < num_pre_pad; i++) {
    final_out_real[i] = 0.0;
    final_out_imag[i] = 0.0;
    DEBUG(printf(" fin_xmit_out %6u : %11.8f + %11.8f i\n", i, final_out_real[i], final_out_imag[i]));
  }
  for (int i = 0; i < num_cycpref_outs; i++) {
    int iidx = num_pre_pad + i;
    final_out_real[iidx] = cycpref_out_real[i];
    final_out_imag[iidx] = cycpref_out_imag[i];
    DEBUG(printf(" fin_xmit_out %6u : %11.8f + %11.8f i\n", iidx, final_out_real[iidx], final_out_imag[iidx]));
  }
  /* for (int i = 0; i < num_post_pad; i++) { */
  /*   int iidx = num_pre_pad + num_cycpref_outs + i; */
  /*   final_out_real[iidx] = 0.0; */
  /*   final_out_imag[iidx] = 0.0; */
  /*   DEBUG(printf(" fin_xmit_out %6u : %11.8f + %11.8f i\n", iidx, final_out_real[iidx], final_out_imag[iidx])); */
  /* } */
  // These next stages do not appear to have any relationship to a physical system that we might consider accelerating.
    
  // The next "Stage" is the "throttle" block, which does not alter the output/message (just timing?)

  // Then there is the channel_model... 

 #ifdef INT_TIME
  gettimeofday(&x_pipe_stop, NULL);
  x_pipe_sec  += x_pipe_stop.tv_sec  - x_pipe_start.tv_sec;
  x_pipe_usec += x_pipe_stop.tv_usec - x_pipe_start.tv_usec;
 #endif
}


