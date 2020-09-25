
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <complex.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>

#include "debug.h"
#include "sdr_type.h"
#include "sdr_base.h"
#include "delay.h"
#include "complex_ops.h"
#include "fir.h"
#include "sync_short.h"
#include "sync_long.h"
#include "gr_equalizer.h"
#include "ofdm.h"
#include "fft.h"
#include "recv_pipe.h"

#ifdef INT_TIME
/* This is RECV PIPE internal Timing information (gathering resources) */
struct timeval r_pipe_stop, r_pipe_start;
uint64_t r_pipe_sec  = 0LL;
uint64_t r_pipe_usec = 0LL;

struct timeval /*r_cmpcnj_stop,*/ r_cmpcnj_start;
uint64_t r_cmpcnj_sec  = 0LL;
uint64_t r_cmpcnj_usec = 0LL;

struct timeval /*r_cmpmpy_stop,*/ r_cmpmpy_start;
uint64_t r_cmpmpy_sec  = 0LL;
uint64_t r_cmpmpy_usec = 0LL;

struct timeval /*r_firc_stop,*/ r_firc_start;
uint64_t r_firc_sec  = 0LL;
uint64_t r_firc_usec = 0LL;

struct timeval /*r_cmpmag_stop,*/ r_cmpmag_start;
uint64_t r_cmpmag_sec  = 0LL;
uint64_t r_cmpmag_usec = 0LL;

struct timeval /*r_cmpmag2_stop,*/ r_cmpmag2_start;
uint64_t r_cmpmag2_sec  = 0LL;
uint64_t r_cmpmag2_usec = 0LL;

struct timeval r_fir_stop, r_fir_start;
uint64_t r_fir_sec  = 0LL;
uint64_t r_fir_usec = 0LL;

struct timeval r_div_stop, r_div_start;
uint64_t r_div_sec  = 0LL;
uint64_t r_div_usec = 0LL;

struct timeval r_sshort_stop, r_sshort_start;
uint64_t r_sshort_sec  = 0LL;
uint64_t r_sshort_usec = 0LL;

struct timeval r_slong_stop, r_slong_start;
uint64_t r_slong_sec  = 0LL;
uint64_t r_slong_usec = 0LL;

struct timeval r_fft_stop, r_fft_start;
uint64_t r_fft_sec  = 0LL;
uint64_t r_fft_usec = 0LL;

struct timeval r_eqlz_stop, r_eqlz_start;
uint64_t r_eqlz_sec  = 0LL;
uint64_t r_eqlz_usec = 0LL;

struct timeval r_decsignl_stop, r_decsignl_start;
uint64_t r_decsignl_sec  = 0LL;
uint64_t r_decsignl_usec = 0LL;

struct timeval r_descrmbl_stop, r_descrmbl_start;
uint64_t r_descrmbl_sec  = 0LL;
uint64_t r_descrmbl_usec = 0LL;

struct timeval r_zz_stop, r_zz_start;
uint64_t r_zz_sec  = 0LL;
uint64_t r_zz_usec = 0LL;

#endif


fx_pt delay16_out[DELAY_16_MAX_OUT_SIZE];
fx_pt cmpx_conj_out[CMP_CONJ_MAX_SIZE];
fx_pt cmpx_mult_out[CMP_MULT_MAX_SIZE];
fx_pt correlation_complex[FIRC_MAVG48_MAX_SIZE]; // (firc mov_avg48 output
//fx_pt correlation_complex_m48[MOV_AVG48_MAX_SIZE]; // (mov_avg48 output
  
fx_pt1 correlation[CMP2MAG_MAX_SIZE]; // complex_to_mangitude outpu
fx_pt1 signal_power[CMP2MAGSQ_MAX_SIZE];
fx_pt1 avg_signal_power[FIR_MAVG64_MAX_SIZE]; // fir moving-average-64
//fx_pt1 avg_signal_power_m64[MOV_AVG64_MAX_SIZE]; // moving-average64
  
fx_pt1 the_correlation[DIVIDE_MAX_SIZE];

fx_pt frame_d[DELAY_320_MAX_OUT_SIZE]; // delay320 output
//fx_pt sync_short_out_frames[SYNC_S_OUT_MAX_SIZE]; // sync_short output
fx_pt* sync_short_out_frames = &(frame_d[320]); // auto-prepends the delay-320 behavior
fx_pt d_sync_long_out_frames[SYNC_L_OUT_MAX_SIZE]; // sync_long_output

uint8_t  decoded_message[MAX_PAYLOAD_SIZE];   // Holds the resulting decodede message.

// The input data goes through a delay16 that simply re-indexes the data (prepending 16 0+0i values)...
fx_pt*   input_data = &delay16_out[16]; // [2*(RAW_DATA_IN_MAX_SIZE + 16)];  // Holds the input data (plus a "front-pad" of 16 0's for delay16


void compute(unsigned in_msg_len, unsigned num_inputs, fx_pt *inbuff, uint8_t *outbuff);


/********************************************************************************
 * This routine manages the initializations for all recv pipeline components
 ********************************************************************************/
void
recv_pipe_init() {
  ; // Apparently nothing we need to do here?
}


unsigned 
do_rcv_fft_work(unsigned num_fft_frames, fx_pt1 fft_ar_r[FRAME_EQ_IN_MAX_SIZE], fx_pt1 fft_ar_i[FRAME_EQ_IN_MAX_SIZE] ) {
    DEBUG(printf("\nSetting up for FFT...\n"));
  // FFT
  // Here we do the FFT calls in 64-sample chunks... using a "window.rectangluar(64)" and forward fft with "Shift = true"

  unsigned num_fft_outs = 0;
  if (num_fft_frames > MAX_FFT_FRAMES) {
    printf("ERROR : FFT generated %u frames which exceeds current MAX_FFT_FRAMES %u\n", num_fft_frames, MAX_FFT_FRAMES);
    exit(-7);
  }
  DEBUG(printf("FFT_COMP : num_fft_frames = %u\n", num_fft_frames));
  { // The FFT only uses one set of input/outputs (the fft_in) and overwrites the inputs with outputs
    float fft_in_real[64];
    float fft_in_imag[64];
    const bool shift_inputs = false;
    for (unsigned i = 0; i < num_fft_frames /*SYNC_L_OUT_MAX_SIZE/64*/; i++) { // This is the "spin" to invoke the FFT
      if (shift_inputs) { // This is FALSE for this usage
	// Effectively "rotate" the input window to re-center
	for (unsigned j = 0; j < 32; j++) {
	  fx_pt fftSample;
	  fftSample = d_sync_long_out_frames[64*i + j]; // 64 * invocations + offset_j
	  fft_in_real[32 + j] = (float)creal(fftSample);
	  fft_in_imag[32 + j] = (float)cimagf(fftSample);
	  
	  fftSample = d_sync_long_out_frames[64*i + 32 + j]; // 64 * invocations + offset_j
	  fft_in_real[j] = (float)crealf(fftSample);
	  fft_in_imag[j] = (float)cimagf(fftSample);
	}
      } else {
	for (unsigned j = 0; j < 64; j++) {
	  fx_pt fftSample;
	  fftSample = d_sync_long_out_frames[64*i + j]; // 64 * invocations + offset_j
	  fft_in_real[j] = (float)crealf(fftSample);
	  fft_in_imag[j] = (float)cimagf(fftSample);
	}
      }
      // Now we have the data ready -- invoke the FFT function
      DEBUG(printf("\n FFT Call %5u \n", i);
	    for (unsigned j = 0; j < 64; j++) {
	      printf("   FFT_IN %4u %2u : %6u %12.8f %12.8f\n", i, j, 64*i+j, fft_in_real[j], fft_in_imag[j]);
	    });
      fft_ri(fft_in_real, fft_in_imag, false, true, 64, 6); // not-inverse, not-shifting
      DEBUG(printf("  FFT Output %4u \n", i);
	    for (unsigned j = 0; j < 64; j++) {
	      printf("   FFT_OUT %4u %2u : %6u %12.8f %12.8f\n", i, j, 64*i+j, fft_in_real[j], fft_in_imag[j]);
	    });
      // Now put the fft outputs into the full FFT results...
      for (unsigned j = 0; j < 64; j++) {
	fft_ar_r[64*i + j] = fft_in_real[j];
	fft_ar_i[64*i + j] = fft_in_imag[j];
      }
      num_fft_outs += 64;
    } // for (i = 0 to CHUNK/64) (FFT invocations loop)
  }
  DEBUG(printf(" num_fft_outs = %u  vs  %u = %u * 64\n", num_fft_outs, num_fft_frames*64, num_fft_frames));
  DEBUG(printf("\nFFT Results at Frame-Eq interface:\n");
	for (unsigned j = 0; j < num_fft_outs /*FRAME_EQ_IN_MAX_SIZE*/; j++) {
	  printf("  FFT-to-FEQ %6u : %12.8f %12.8f\n", j, fft_ar_r[j], fft_ar_i[j]);
	}
	printf("\n"));

  return num_fft_outs;
}

/********************************************************************************
 * This routine manages the transmit pipeline functions and components
 ********************************************************************************/
void
do_recv_pipeline(int msg_len, int num_recvd_vals, float* recvd_in_real, float* recvd_in_imag, int* recvd_msg_len, char * recvd_msg)
{
  DEBUG(printf("In do_recv_pipeline: num_received_vals = %u\n", num_recvd_vals); fflush(stdout));
  for (int i = 0; i < num_recvd_vals; i++) {
    input_data[i] = recvd_in_real[i] + I * recvd_in_imag[i];
  }
  DEBUG(printf("Calling compute\n"));
 #ifdef INT_TIME
  gettimeofday(&r_pipe_start, NULL);
 #endif
  compute(msg_len, num_recvd_vals, input_data, (uint8_t*)recvd_msg); // outbuff);
  *recvd_msg_len = 1500; // Default max size...
 #ifdef INT_TIME
  gettimeofday(&r_pipe_stop, NULL);
  r_pipe_sec  += r_pipe_stop.tv_sec  - r_pipe_start.tv_sec;
  r_pipe_usec += r_pipe_stop.tv_usec - r_pipe_start.tv_usec;
 #endif
  
  DEBUG(printf("CMP_MSG:\n%s\n", recvd_msg));
}


void compute(unsigned in_msg_len, unsigned num_inputs, fx_pt *input_data, uint8_t *out_msg) {
  uint8_t scrambled_msg[MAX_ENCODED_BITS * 3 / 4];
  DEBUG(for (int ti = 0; ti < num_inputs /*RAW_DATA_IN_MAX_SIZE*/; ti++) {
	  printf("  %6u : TOP_INBUF %12.8f %12.8f\n", ti, crealf(input_data[ti]), cimagf(input_data[ti]));
	});

  DEBUG(printf("\nCalling delay...\n"));
  unsigned num_del16_vals = num_inputs + 16;
  // We don't need to make this call now -- we've done the effect in the memory array already
  // delay(delay16_out, num_inputs, input_data);  
  DEBUG(for (int ti = 0; ti < num_del16_vals; ti++) {
      printf(" DEL16 %5u : TMPBUF %12.8f %12.8f : INBUF %12.8f %12.8f\n", ti, crealf(delay16_out[ti]), cimagf(delay16_out[ti]), crealf(input_data[ti]), cimagf(input_data[ti]));
    });
  
  DEBUG(printf("\nCalling complex_conjugate...\n"));
  unsigned num_cconj_vals = num_del16_vals;
 #ifdef INT_TIME
  gettimeofday(&r_cmpcnj_start, NULL);
 #endif
  complex_conjugate(cmpx_conj_out, num_del16_vals, delay16_out);
  DEBUG(for (int ti = 0; ti < num_cconj_vals; ti++) {
      printf("  CMP_CONJ %5u : CONJ_OUT %12.8f %12.8f : DEL16_IN %12.8f %12.8f\n", ti, crealf(cmpx_conj_out[ti]), cimagf(cmpx_conj_out[ti]), crealf(delay16_out[ti]), cimagf(delay16_out[ti]));
    });
  
  DEBUG(printf("\nCalling complex_mult...\n"));
  unsigned num_cmult_vals = num_cconj_vals;
 #ifdef INT_TIME
  gettimeofday(&r_cmpmpy_start, NULL);
  r_cmpcnj_sec  += r_cmpmpy_start.tv_sec  - r_cmpcnj_start.tv_sec;
  r_cmpcnj_usec += r_cmpmpy_start.tv_usec - r_cmpcnj_start.tv_usec;
 #endif
  complex_multiply(cmpx_mult_out, num_cconj_vals, cmpx_conj_out, input_data);
  DEBUG(for (int ti = 0; ti < num_cmult_vals; ti++) {
      printf("  CMP_MULT %5u : MULT_OUT %12.8f %12.8f : CMP_CONJ %12.8f %12.8f : INBUF %12.8f %12.8f\n", ti, crealf(cmpx_mult_out[ti]), cimagf(cmpx_mult_out[ti]), crealf(cmpx_conj_out[ti]), cimagf(cmpx_conj_out[ti]), crealf(input_data[ti]), cimagf(input_data[ti]));
    });
  
  DEBUG(printf("\nCalling firc (Moving Average 48)...\n"));
  unsigned num_cmpcorr_vals = num_cmult_vals;
 #ifdef INT_TIME
  gettimeofday(&r_firc_start, NULL);
  r_cmpmpy_sec  += r_firc_start.tv_sec  - r_cmpmpy_start.tv_sec;
  r_cmpmpy_usec += r_firc_start.tv_usec - r_cmpmpy_start.tv_usec;
 #endif
  firc(correlation_complex, cmpx_mult_out);
  DEBUG(for (int ti = 0; ti < num_cmpcorr_vals; ti++) {
      printf("  MV_AVG48 %5u : CORR_CMP %12.8f %12.8f : MULT_OUT %12.8f %12.8f\n", ti, crealf(correlation_complex[ti]), cimagf(correlation_complex[ti]), crealf(cmpx_mult_out[ti]), cimagf(cmpx_mult_out[ti]));
    });
  
  DEBUG(printf("\nCalling complex_to_magnitude...\n"));
  unsigned num_cmag_vals = num_cmpcorr_vals;
 #ifdef INT_TIME
  gettimeofday(&r_cmpmag_start, NULL);
  r_firc_sec  += r_cmpmag_start.tv_sec  - r_firc_start.tv_sec;
  r_firc_usec += r_cmpmag_start.tv_usec - r_firc_start.tv_usec;
 #endif
  complex_to_magnitude(correlation, num_cmpcorr_vals, correlation_complex);
  DEBUG(for (unsigned ti = 0; ti < num_cmag_vals; ti++) {
      printf("  MAGNITUDE %5u : CORR %12.8f : CORR_CMP %12.8f %12.8f\n", ti, correlation[ti], crealf(correlation_complex[ti]), cimagf(correlation_complex[ti]));
    });
  
  DEBUG(printf("\nCalling complex_to_mag_squared (signal_power)...\n"));
  unsigned num_cmag2_vals = num_inputs;
 #ifdef INT_TIME
  gettimeofday(&r_cmpmag2_start, NULL);
  r_cmpmag_sec  += r_cmpmag2_start.tv_sec  - r_cmpmag_start.tv_sec;
  r_cmpmag_usec += r_cmpmag2_start.tv_usec - r_cmpmag_start.tv_usec;
 #endif
  complex_to_mag_squared(signal_power, num_inputs, input_data);
  DEBUG(for (int ti = 0; ti < num_cmag2_vals; ti++) {
      printf("  MAG^2 %5u : SIGN_PWR %12.8f : INBUF %12.8f %12.8f\n", ti, signal_power[ti], crealf(input_data[ti]), cimagf(input_data[ti]));
    });
    
  DEBUG(printf("\nCalling fir (Moving Average 64)...\n"));
  // fir filter
  const fx_pt1 coeff_mvgAvg[COEFF_LENGTH]={ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   //   8
  					    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   //  16
  					    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   //  24
  					    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   //  32
  					    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   //  40
  					    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   //  48
  					    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   //  56
  					    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; //  64
  unsigned num_mavg48_vals = num_cmag2_vals;
 #ifdef INT_TIME
  gettimeofday(&r_fir_start, NULL);
  r_cmpmag2_sec  += r_fir_start.tv_sec  - r_cmpmag2_start.tv_sec;
  r_cmpmag2_usec += r_fir_start.tv_usec - r_cmpmag2_start.tv_usec;
 #endif
  fir(avg_signal_power, signal_power, coeff_mvgAvg);
  DEBUG(for (unsigned ti = 0; ti < num_mavg48_vals; ti++) {
    printf(" TOP_FIR_OUT %5u : SIGN_PWR-AVG %12.8f : SIGN_PWR %12.8f\n", ti, avg_signal_power[ti], signal_power[ti]);
    });
  
  DEBUG(printf("\nCalling division...\n"));
  unsigned num_cdiv_vals = num_cmpcorr_vals;
  // Ensure we've picked the MIN(num_mavg48_vals, num_cmag_vals) -- should have an a-priori known relationship!
  if (num_mavg48_vals > num_cmag_vals) {
    printf("ERROR : num_mavg48_vals = %u > %u = num_cmag_vals\n", num_mavg48_vals, num_cmag_vals);
    exit(-8);
  }
 #ifdef INT_TIME
  gettimeofday(&r_div_start, NULL);
  r_fir_sec  += r_div_start.tv_sec  - r_fir_start.tv_sec;
  r_fir_usec += r_div_start.tv_usec - r_fir_start.tv_usec;
 #endif
  division(the_correlation, num_mavg48_vals, correlation, avg_signal_power);
  DEBUG(for (unsigned ti = 0; ti < num_cdiv_vals; ti++) {
    printf(" TOP_DIV_OUT %5u : CORRZ %12.8f : CORRL %12.8f : SPA %12.8f\n", ti, the_correlation[ti], correlation[ti], avg_signal_power[ti]);
    });
  
  DEBUG(printf("\nCalling sync_short...\n"));
  float ss_freq_offset;
  unsigned num_sync_short_vals;
 #ifdef INT_TIME
  gettimeofday(&r_sshort_start, NULL);
  r_div_sec  += r_sshort_start.tv_sec  - r_div_start.tv_sec;
  r_div_usec += r_sshort_start.tv_usec - r_div_start.tv_usec;
 #endif
  sync_short(num_cdiv_vals, delay16_out, correlation_complex, the_correlation, &ss_freq_offset, &num_sync_short_vals, sync_short_out_frames);
  DEBUG(printf("  num_sync_short_vals = %u\n", num_sync_short_vals));
  DEBUG(printf(" ss_freq_offset = %12.8f  and num sync_short_values = %u\n", ss_freq_offset, num_sync_short_vals);
	for (int ti = 0; ti < num_sync_short_vals; ti++) {
	  printf(" S_S_OUT %5u : TMPBUF %12.8f %12.8f : CORR_CMP %12.8f %12.8f : CORRZ %12.8f : SS_FRAME %12.8f %12.8f\n", ti, crealf(delay16_out[ti]), cimagf(delay16_out[ti]), crealf(correlation_complex[ti]), cimagf(correlation_complex[ti]), the_correlation[ti], crealf(sync_short_out_frames[ti]), cimagf(sync_short_out_frames[ti])); 
	});

  DEBUG(printf("In receive pipe compute routine with in_msg_len @ %p = %u\n", (void*)&in_msg_len, in_msg_len));
  DEBUG(printf("\nCalling delay320...\n"));
  // We've used a similar memory-placement trick to avoid having to do the delay320 function; sync_short_out_frames = &(frame_d[320])
  //delay320(frame_d, synch_short_out_frames);
  DEBUG(for (int ti = 0; ti < 320+num_sync_short_vals; ti++) {
      printf("  DELAY_320 %5u : FRAME_D %12.8f %12.8f : FRAME %12.8f %12.8f\n", ti, crealf(frame_d[ti]), cimagf(frame_d[ti]), crealf(sync_short_out_frames[ti]), cimagf(sync_short_out_frames[ti]));
    });
  

  DEBUG(printf("In receive pipe compute routine with in_msg_len @ %p = %u\n", (void*)&in_msg_len, in_msg_len));
  DEBUG(printf("\nCalling sync_long...\n"));
  float sl_freq_offset;
  unsigned num_sync_long_vals;
 #ifdef INT_TIME
  gettimeofday(&r_slong_start, NULL);
  r_sshort_sec  += r_slong_start.tv_sec  - r_sshort_start.tv_sec;
  r_sshort_usec += r_slong_start.tv_usec - r_sshort_start.tv_usec;
 #endif
  sync_long(num_sync_short_vals, sync_short_out_frames, frame_d, &sl_freq_offset, &num_sync_long_vals, d_sync_long_out_frames );
  DEBUG(printf("  num_sync_long_vals = %u\n", num_sync_long_vals));
  DEBUG(printf(" sl_freq_offset = %12.8f\n", sl_freq_offset);
	for (int ti = 0; ti < 32619; ti++) {
	  printf("  SYNC_LONG_OUT %5u  %12.8f %12.8f : FR_D %12.8f %12.8f : D_FR_L %12.8f %12.8f\n", ti, crealf(sync_short_out_frames[ti]), cimagf(sync_short_out_frames[ti]), crealf(frame_d[ti]), cimagf(frame_d[ti]), crealf(d_sync_long_out_frames[ti]), cimagf(d_sync_long_out_frames[ti])); 
	});

  fx_pt1 fft_ar_r[FRAME_EQ_IN_MAX_SIZE];
  fx_pt1 fft_ar_i[FRAME_EQ_IN_MAX_SIZE];
  unsigned num_fft_frames = num_sync_long_vals / 64;
  DEBUG(printf("FFT_COMP : num_fft_frames = %u / 64 = %u\n", num_sync_long_vals,  num_fft_frames));
 #ifdef INT_TIME
  gettimeofday(&r_fft_start, NULL);
  r_slong_sec  += r_fft_start.tv_sec  - r_slong_start.tv_sec;
  r_slong_usec += r_fft_start.tv_usec - r_slong_start.tv_usec;
 #endif
  unsigned num_fft_outs = do_rcv_fft_work(num_fft_frames, fft_ar_r, fft_ar_i);
 #ifdef INT_TIME
  gettimeofday(&r_fft_stop, NULL);
  r_fft_sec  += r_fft_stop.tv_sec  - r_fft_start.tv_sec;
  r_fft_usec += r_fft_stop.tv_usec - r_fft_start.tv_usec;
 #endif
 
  // equalize
  fx_pt toBeEqualized[FRAME_EQ_IN_MAX_SIZE];
  fx_pt equalized[FRAME_EQ_OUT_MAX_SIZE];

  // NOTE: The 509 == FFT_OUT_SIZE / 64
  for(unsigned i = 0; i < num_fft_frames /*509*/ * 64; i++) {
    toBeEqualized[i] = (fx_pt)(fft_ar_r[i] + fft_ar_i[i] * I);
  }
  float wifi_start = ss_freq_offset - sl_freq_offset;
  uint8_t equalized_bits[FRAME_EQ_OUT_MAX_SIZE];
  for (int ii = 0; ii < FRAME_EQ_OUT_MAX_SIZE; ii++) {
    equalized_bits[ii] = 0;
  }
  DEBUG(printf("In receive pipe compute routine with in_msg_len @ %p = %u\n", (void*)&in_msg_len, in_msg_len));
  DEBUG(printf("\nCalling gr_equalize (frame_equalizer) with wifi_start = %12.8f\n", wifi_start));
  unsigned num_eq_out_bits;
  unsigned num_eq_out_sym;
 #ifdef INT_TIME
  gettimeofday(&r_eqlz_start, NULL);
 #endif
  gr_equalize( wifi_start, num_fft_outs, toBeEqualized, &num_eq_out_bits, equalized_bits, &num_eq_out_sym, equalized);
  DEBUG(printf("GR_FR_EQ : fft_outs = %u items %u ffts : num_eq_out_bits = %u %u : num_eq_out_sym = %u\n", num_fft_outs, num_fft_outs/64, num_eq_out_bits, num_eq_out_bits/48, num_eq_out_sym));
  DEBUG(printf("\nback from equalize call (frame_equalizer)\n");
	for (int ti = 0; ti < num_eq_out_bits; ti++) {
	  printf(" FR_EQ_OUT %5u : toBeEQ %12.8f %12.8f : EQLZD %12.8f %12.8f : EQ_BIT %u\n", ti, crealf(toBeEqualized[ti]), cimagf(toBeEqualized[ti]), crealf(equalized[ti]), cimagf(equalized[ti]), equalized_bits[ti]);
	});
  
  //decode signal
  DEBUG(printf("In receive pipe compute routine with in_msg_len @ %p = %u\n", (void*)&in_msg_len, in_msg_len));
  DEBUG(printf("\nCalling decode_signal...\n"));
  unsigned num_dec_bits;
 #ifdef INT_TIME
  gettimeofday(&r_decsignl_start, NULL);
  r_eqlz_sec  += r_decsignl_start.tv_sec  - r_eqlz_start.tv_sec;
  r_eqlz_usec += r_decsignl_start.tv_usec - r_eqlz_start.tv_usec;
 #endif
  decode_signal(num_eq_out_bits, equalized, &num_dec_bits, scrambled_msg);
  DEBUG(for (int ti = 0; ti < num_dec_bits; ti++) {
      printf(" DEC_OUTS %5u : EQLZD %12.8f %12.8f : DEC_BIT %u\n", ti, crealf(toBeEqualized[ti]), cimagf(toBeEqualized[ti]), scrambled_msg[ti]);
    });

  //descrambler
  unsigned psdu = (in_msg_len + 28);
  DEBUG(printf("In receive pipe compute routine with in_msg_len @ %p = %u\n", (void*)&in_msg_len, in_msg_len));
  DEBUG(printf("\nCalling sdr_descrambler with psdu = %u\n", psdu));
 #ifdef INT_TIME
  gettimeofday(&r_descrmbl_start, NULL);
  r_decsignl_sec  += r_descrmbl_start.tv_sec  - r_decsignl_start.tv_sec;
  r_decsignl_usec += r_descrmbl_start.tv_usec - r_decsignl_start.tv_usec;
 #endif
  sdr_descrambler(scrambled_msg, psdu, (char*)out_msg);
 #ifdef INT_TIME
  gettimeofday(&r_descrmbl_stop, NULL);
  r_descrmbl_sec  += r_descrmbl_stop.tv_sec  - r_descrmbl_start.tv_sec;
  r_descrmbl_usec += r_descrmbl_stop.tv_usec - r_descrmbl_start.tv_usec;
 #endif
  DEBUG(printf("\nDESC_MSG:\n%s\n", out_msg));
}
