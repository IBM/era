#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <complex.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>

#undef VERBOSE

#include "debug.h"
#ifdef RECV_HW_FFT
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "contig.h"
#include "mini-era.h"
#endif // RECV_HW_FFT

#include "globals.h"
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

#include "globalsRecv.h"

#include "recv_pipe.h"

#if defined(HPVM)
#include "hpvm.h"
#include "hetero.h"
#endif

#undef INT_TIME // TODO: REMOVE ME; this should be un-set during compilation

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

#ifdef RECV_HW_FFT
struct timeval r_fHtotal_stop, r_fHtotal_start;
uint64_t r_fHtotal_sec  = 0LL;
uint64_t r_fHtotal_usec = 0LL;

struct timeval r_fHcvtin_stop, r_fHcvtin_start;
uint64_t r_fHcvtin_sec  = 0LL;
uint64_t r_fHcvtin_usec = 0LL;

struct timeval r_fHcomp_stop, r_fHcomp_start;
uint64_t r_fHcomp_sec  = 0LL;
uint64_t r_fHcomp_usec = 0LL;

struct timeval r_fHcvtout_stop, r_fHcvtout_start;
uint64_t r_fHcvtout_sec  = 0LL;
uint64_t r_fHcvtout_usec = 0LL;
#endif


#endif


fx_pt  delay16_out[DELAY_16_MAX_OUT_SIZE];
fx_pt  cmpx_conj_out[CMP_CONJ_MAX_SIZE];
fx_pt  firc_input[CMP_MULT_MAX_SIZE + COMPLEX_COEFF_LENGTH]; // holds cmpx_mult_out but pre-pads with zeros
fx_pt* cmpx_mult_out = &(firc_input[COMPLEX_COEFF_LENGTH]);
fx_pt  correlation_complex[FIRC_MAVG48_MAX_SIZE]; // (firc mov_avg48 output
//fx_pt correlation_complex_m48[MOV_AVG48_MAX_SIZE]; // (mov_avg48 output

fx_pt1  correlation[CMP2MAG_MAX_SIZE]; // complex_to_mangitude outpu
fx_pt1  fir_input[CMP2MAGSQ_MAX_SIZE + COEFF_LENGTH]; // holds signal_power but pre-pads with zeros
fx_pt1* signal_power = &(fir_input[COEFF_LENGTH]);
fx_pt1  avg_signal_power[FIR_MAVG64_MAX_SIZE]; // fir moving-average-64
//fx_pt1 avg_signal_power_m64[MOV_AVG64_MAX_SIZE]; // moving-average64

fx_pt1 the_correlation[DIVIDE_MAX_SIZE];

fx_pt frame_d[DELAY_320_MAX_OUT_SIZE]; // delay320 output
//fx_pt sync_short_out_frames[SYNC_S_OUT_MAX_SIZE]; // sync_short output
fx_pt* sync_short_out_frames = &(frame_d[320]); // auto-prepends the delay-320 behavior
fx_pt d_sync_long_out_frames[SYNC_L_OUT_MAX_SIZE]; // sync_long_output

uint8_t  decoded_message[MAX_PAYLOAD_SIZE];   // Holds the resulting decodede message.

// The input data goes through a delay16 that simply re-indexes the data (prepending 16 0+0i values)...
fx_pt*   input_data = &delay16_out[16]; // [2*(RAW_DATA_IN_MAX_SIZE + 16)];  // Holds the input data (plus a "front-pad" of 16 0's for delay16


void compute(unsigned num_inputs, fx_pt *input_data, size_t input_data_sz, 
																int* out_msg_len, size_t out_msg_len_sz, uint8_t *out_msg, size_t out_msg_sz,
																uint8_t* scrambled_msg /*local*/, size_t scrambled_msg_sz /*= MAX_ENCODED_BITS * 3 / 4 */,
																// Local variables for compute
																float* ss_freq_offset /*local*/, size_t ss_freq_offset_sz /*=1*/, 
																unsigned* num_sync_short_vals /*local*/, size_t num_sync_short_vals_sz /*=1*/,
																float* sl_freq_offset /*local*/, size_t sl_freq_offset_sz /*=1*/,
																unsigned* num_sync_long_vals /*local*/, size_t num_sync_long_vals_sz /*=1*/,
																fx_pt1* fft_ar_r/*local*/, size_t fft_ar_r_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																fx_pt1* fft_ar_i /*local*/, size_t fft_ar_i_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																unsigned* num_fft_outs /*local*/, size_t num_fft_outs_sz /*=1*/,
																fx_pt* toBeEqualized /*local*/, size_t toBeEqualized_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																fx_pt* equalized /*local*/, size_t equalized_sz /*= FRAME_EQ_OUT_MAX_SIZE*/,
																unsigned* num_eq_out_bits /*local*/, size_t num_eq_out_bits_sz /*=1*/,
																unsigned* psdu /*local*/, size_t psdu_sz /*=1*/,
																// Globals used by compute
																fx_pt* delay16_out_arg /*= delay16_out -> global*/, size_t delay16_out_arg_sz /*= DELAY_16_MAX_OUT_SIZE*/,
																fx_pt* cmpx_conj_out_arg /*= cmpx_conj_out -> global*/, size_t cmpx_conj_out_arg_sz /*= CMP_CONJ_MAX_SIZE*/,
																fx_pt* cmpx_mult_out_arg /*= cmpx_mult_out -> global*/, size_t cmpx_mult_out_arg_sz /*= CMP_MULT_MAX_SIZE*/,
																fx_pt* correlation_complex_arg /*= correlation_complex -> global*/, size_t correlation_complex_arg_sz /*= FIRC_MAVG48_MAX_SIZE*/,
																fx_pt1* correlation_arg /*= correlation -> global*/, size_t correlation_arg_sz /*= CMP2MAG_MAX_SIZE*/,
																fx_pt1* signal_power_arg /*= signal_power -> global*/, size_t signal_power_arg_sz /*= CMP2MAGSQ_MAX_SIZE*/,
																fx_pt1* avg_signal_power_arg /*= avg_signal_power -> global*/, size_t avg_signal_power_arg_sz /*= FIR_MAVG64_MAX_SIZE*/,
																fx_pt1* the_correlation_arg /*= the_correlation -> global*/, size_t the_correlation_arg_sz /*= DIVIDE_MAX_SIZE*/,
																fx_pt* sync_short_out_frames_arg /*= sync_short_out_frames -> global*/, size_t sync_short_out_frames_arg_sz /*=320*/,
																fx_pt* d_sync_long_out_frames_arg /*= d_sync_long_out_frames -> global*/, size_t d_sync_long_out_frames_arg_sz /*= SYNC_L_OUT_MAX_SIZE*/);


/********************************************************************************
	* This routine manages the initializations for all recv pipeline components
	********************************************************************************/

#ifdef COMPILE_TO_ESP
#include "fixed_point.h"
#endif

#ifdef RECV_HW_FFT
// These are RECV FFT Hardware Accelerator Variables, etc.
char recv_fftAccelName[NUM_RECV_FFT_ACCEL][64];// = {"/dev/recv_fft.0", "/dev/recv_fft.1", "/dev/recv_fft.2", "/dev/recv_fft.3", "/dev/recv_fft.4", "/dev/recv_fft.5"};

int recv_fftHW_fd[NUM_RECV_FFT_ACCEL];
contig_handle_t recv_fftHW_mem[NUM_RECV_FFT_ACCEL];

fftHW_token_t* recv_fftHW_lmem[NUM_RECV_FFT_ACCEL];  // Pointer to local version (mapping) of recv_fftHW_mem
fftHW_token_t* recv_fftHW_li_mem[NUM_RECV_FFT_ACCEL]; // Pointer to input memory block
fftHW_token_t* recv_fftHW_lo_mem[NUM_RECV_FFT_ACCEL]; // Pointer to output memory block
size_t recv_fftHW_in_len[NUM_RECV_FFT_ACCEL];
size_t recv_fftHW_out_len[NUM_RECV_FFT_ACCEL];
size_t recv_fftHW_in_size[NUM_RECV_FFT_ACCEL];
size_t recv_fftHW_out_size[NUM_RECV_FFT_ACCEL];
size_t recv_fftHW_out_offset[NUM_RECV_FFT_ACCEL];
size_t recv_fftHW_size[NUM_RECV_FFT_ACCEL];
struct fftHW_access recv_fftHW_desc[NUM_RECV_FFT_ACCEL];


static void init_recv_fft_parameters(unsigned n, uint32_t log_nsamples, uint32_t num_ffts, uint32_t do_inverse, uint32_t do_shift, uint32_t scale_factor)
{
								size_t fftHW_in_words_adj;
								size_t fftHW_out_words_adj;
								int num_samples = 1 << log_nsamples;
								DEBUG(printf("  In init_recv_fft_params : n %u logn %u nfft %u inv %u shft %u scl %u\n", n, log_nsamples, num_ffts, do_inverse, do_shift, scale_factor));

								recv_fftHW_desc[n].scale_factor = 0;
								recv_fftHW_desc[n].logn_samples = log_nsamples;
								recv_fftHW_desc[n].num_ffts     = num_ffts;
								recv_fftHW_desc[n].do_inverse   = do_inverse;
								recv_fftHW_desc[n].do_shift     = do_shift;

								if (DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)) == 0) {
																fftHW_in_words_adj  = 2 * num_ffts * num_samples;
																fftHW_out_words_adj = 2 * num_ffts * num_samples;
								} else {
																fftHW_in_words_adj  = round_up(2 * num_ffts * num_samples, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
																fftHW_out_words_adj = round_up(2 * num_ffts * num_samples, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
								}
								recv_fftHW_in_len[n]  = fftHW_in_words_adj;
								recv_fftHW_out_len[n] = fftHW_out_words_adj;
								recv_fftHW_in_size[n]  = recv_fftHW_in_len[n]  * sizeof(fftHW_token_t);
								recv_fftHW_out_size[n] = recv_fftHW_out_len[n] * sizeof(fftHW_token_t);
								recv_fftHW_out_offset[n] = 0;
								recv_fftHW_size[n] = (recv_fftHW_out_offset[n] * sizeof(fftHW_token_t)) + recv_fftHW_out_size[n];
								DEBUG(printf("  returning from init_recv_fft_parameters for HW_FFT[%u]\n", n);
																								printf("    in_len %u %u  in size %u tot_size %u\n", recv_fftHW_in_len[n], recv_fftHW_in_size[n], recv_fftHW_size[n]);
																								printf("   out_len %u %u out size %u out_ofst %u\n", recv_fftHW_out_len[n], recv_fftHW_out_size[n], recv_fftHW_out_offset[n]));
}

static void recv_fft_in_hw(int *fd, struct fftHW_access *desc)
{
								if (ioctl(*fd, FFTHW_IOC_ACCESS, *desc)) {
																perror("ERROR : recv_fft_in_hw : IOCTL:\n");
																closeout_and_exit("Bad ioctl invocation for RECV pipe", EXIT_FAILURE);
								}
}

void free_RECV_FFT_HW_RESOURCES() {
								for (int fi = 0; fi < NUM_RECV_FFT_ACCEL; fi++) {
																contig_free(recv_fftHW_mem[fi]);
																close(recv_fftHW_fd[fi]);
								}
}
#endif // RECV_HW_FFT

void
recv_pipe_init() {
								// Initialize the pre-pended zero segments for fir_input and firc_input
								for (int i = 0; i < COEFF_LENGTH; i++) {
																fir_input[i] = 0;
								}
								for (int i = 0; i < COMPLEX_COEFF_LENGTH; i++) {
																firc_input[i] = 0;
								}

#ifdef RECV_HW_FFT
								// This initializes the RECV_FFT Accelerator Pool
								printf("Initializing the %u total RECV_FFT Accelerators...\n", NUM_RECV_FFT_ACCEL);
								for (int fi = 0; fi < NUM_RECV_FFT_ACCEL; fi++) {
																// Inititalize to the "largest legal" RECV_FFT size
																printf("Calling init_recv_fft_parameters for Accel %u (of %u) with LOGN %u and MAX_FFTS %u\n", fi, NUM_RECV_FFT_ACCEL, MAX_RECV_FFT_LOGN, MAX_RECV_NUM_FFTS);
																init_recv_fft_parameters(fi, MAX_RECV_FFT_LOGN, MAX_RECV_NUM_FFTS, 0, 0, 0);

																snprintf(recv_fftAccelName[fi], 63, "%s.%u", FFT_DEV_BASE, (NUM_XMIT_FFT_ACCEL + fi)); /* Start RECV after XMIT */
																printf(" Acclerator %u opening RECV_FFT device %s\n", fi, recv_fftAccelName[fi]);
																recv_fftHW_fd[fi] = open(recv_fftAccelName[fi], O_RDWR, 0);
																if (recv_fftHW_fd[fi] < 0) {
																								fprintf(stderr, "Error: cannot open %s\n", recv_fftAccelName[fi]);
																								closeout_and_exit("Cannot open RECV FFT Accel", EXIT_FAILURE);
																}

																printf(" Allocate hardware buffer of size %u\n", recv_fftHW_size[fi]);
																recv_fftHW_lmem[fi] = contig_alloc(recv_fftHW_size[fi], &(recv_fftHW_mem[fi]));
																if (recv_fftHW_lmem[fi] == NULL) {
																								fprintf(stderr, "Error: cannot allocate %u contig bytes\n", recv_fftHW_size[fi]);
																								closeout_and_exit("Cannot allocate RECV_FFT memory", EXIT_FAILURE);
																}

																recv_fftHW_li_mem[fi] = &(recv_fftHW_lmem[fi][0]);
																recv_fftHW_lo_mem[fi] = &(recv_fftHW_lmem[fi][recv_fftHW_out_offset[fi]]);
																printf(" Set recv_fftHW_li_mem = %p  AND recv_fftHW_lo_mem = %p\n", recv_fftHW_li_mem[fi], recv_fftHW_lo_mem[fi]);

																recv_fftHW_desc[fi].esp.run = true;
																recv_fftHW_desc[fi].esp.coherence = ACC_COH_NONE;
																recv_fftHW_desc[fi].esp.p2p_store = 0;
																recv_fftHW_desc[fi].esp.p2p_nsrcs = 0;
																//recv_fftHW_desc[fi].esp.p2p_srcs = {"", "", "", ""};
																recv_fftHW_desc[fi].esp.contig = contig_to_khandle(recv_fftHW_mem[fi]);

#if USE_RECV_FFT_ACCEL_VERSION == 1
																// Always use BIT-REV in HW for now -- simpler interface, etc.
																recv_fftHW_desc[fi].do_bitrev  = RECV_FFTHW_DO_BITREV;
#elif USE_RECV_FFT_ACCEL_VERSION == 2
																recv_fftHW_desc[fi].num_recv_ffts  = 1;
																recv_fftHW_desc[fi].do_inverse     = RECV_FFTHW_NO_INVERSE;
																recv_fftHW_desc[fi].do_shift       = RECV_FFTHW_NO_SHIFT;
																recv_fftHW_desc[fi].scale_factor   = 1;
#endif
																//recv_fftHW_desc[fi].logn_samples  = log_nsamples; 
																recv_fftHW_desc[fi].src_offset = 0;
																recv_fftHW_desc[fi].dst_offset = 0;
								}
#endif

}


unsigned 
do_rcv_fft_work(unsigned num_fft_frames, fx_pt1 fft_ar_r[FRAME_EQ_IN_MAX_SIZE], fx_pt1 fft_ar_i[FRAME_EQ_IN_MAX_SIZE] ) {
								DEBUG(printf("\nSetting up for FFT...\n"));
								// FFT
								// Here we do the FFT calls in 64-sample chunks... using a "window.rectangluar(64)" and forward fft with "Shift = true"

								unsigned num_fft_outs = 0;
								DO_LIMITS_ANALYSIS(float min_input = 3.0e+038;
																								float max_input = -1.17e-038);  
								if (num_fft_frames > MAX_FFT_FRAMES) {
																printf("ERROR : FFT generated %u frames which exceeds current MAX_FFT_FRAMES %u\n", num_fft_frames, MAX_FFT_FRAMES);
																exit(-7);
								}
								DEBUG(printf("FFT_COMP : num_fft_frames = %u\n", num_fft_frames));
								const uint32_t log_nsamples = 6;
								const uint32_t num_samples = (1<<log_nsamples);
								const uint32_t do_inverse = 0;
								const uint32_t do_shift = 1;
#ifdef RECV_HW_FFT
								// Now we call the init_recv_fft_parameters for the target FFT HWR accelerator and the specific log_nsamples for this invocation
								const uint32_t fn = 0;
								const uint32_t scale_factor = 1;
								printf("Calling init_recv_fft_parms fn %u lgn %u nfft %u inv %u shft %u\n", fn, log_nsamples, num_fft_frames, do_inverse, do_shift, scale_factor);
								init_recv_fft_parameters(fn, log_nsamples, num_fft_frames, do_inverse, do_shift, scale_factor);

#ifdef INT_TIME
								gettimeofday(&(r_fHcvtin_start), NULL);
#endif // INT_TIME
								// convert input from float to fixed point
								// We also SCALE it here (but we should be able to do that in the HWR Accel later)
								{ // scope for jidx
																int jidx = 0;
																FFT_DEBUG(printf(" RECV: Doing convert-inputs...\n"));
																for (unsigned i = 0; i < num_fft_frames /*SYNC_L_OUT_MAX_SIZE/64*/; i++) { // This is the "spin" to invoke the FFT
																								//printf("  RECV: converting FFT frame %u\n",i); fflush(stdout);
																								for (unsigned j = 0; j < 64; j++) {
																																fx_pt fftSample;
																																//printf("    RECV: jidx %u : fftSample = d_sync_long_out_frames[64* %u + %u] = [ %u ] = %f\n", jidx, i, j, 64*i+j, d_sync_long_out_frames[64*i + j]); fflush(stdout);
																																fftSample = d_sync_long_out_frames[64*i + j]; // 64 * invocations + offset_j
																																recv_fftHW_lmem[fn][jidx++] = float2fx((float)crealf(fftSample), FX_IL);
																																recv_fftHW_lmem[fn][jidx++] = float2fx((float)cimagf(fftSample), FX_IL);
																								}
																}
								} // scope for jidx
#ifdef INT_TIME
								gettimeofday(&r_fHcvtin_stop, NULL);
								r_fHcvtin_sec   += r_fHcvtin_stop.tv_sec  - r_fHcvtin_start.tv_sec;
								r_fHcvtin_usec  += r_fHcvtin_stop.tv_usec - r_fHcvtin_start.tv_usec;
#endif // INT_TIME

								// Call the FFT Accelerator
								//    NOTE: Currently this is blocking-wait for call to complete
#ifdef INT_TIME
								gettimeofday(&(r_fHcomp_start), NULL);
#endif // INT_TIME
								//DEBUG(
								FFT_DEBUG(printf(" RECV: calling the HW_RECV_FFT[%u]\n", fn));
								recv_fft_in_hw(&(recv_fftHW_fd[fn]), &(recv_fftHW_desc[fn]));
#ifdef INT_TIME
								gettimeofday(&r_fHcomp_stop, NULL);
								r_fHcomp_sec   += r_fHcomp_stop.tv_sec  - r_fHcomp_start.tv_sec;
								r_fHcomp_usec  += r_fHcomp_stop.tv_usec - r_fHcomp_start.tv_usec;
#endif
								// convert output from fixed point to float
								DEBUG(printf("EHFA:   converting from fixed-point to float\n"));
#ifdef INT_TIME
								gettimeofday(&(r_fHcvtout_start), NULL);
#endif // INT_TIME
								FFT_DEBUG(printf(" RECV: Doing convert-outputs...\n"));
								{ // scope for jidx
																int jidx = 0;
																for (unsigned i = 0; i < num_fft_frames /*SYNC_L_OUT_MAX_SIZE/64*/; i++) { // This is the "spin" to invoke the FFT
																								for (unsigned j = 0; j < 64; j++) {
																																fft_ar_r[64*i + j] = (float)fx2float(recv_fftHW_lmem[fn][jidx++], FX_IL);
																																fft_ar_i[64*i + j] = (float)fx2float(recv_fftHW_lmem[fn][jidx++], FX_IL);
																								}
																}
								} // scope for jidx
								num_fft_outs = 64 * num_fft_frames;
#ifdef INT_TIME
								gettimeofday(&r_fHcvtout_stop, NULL);
								r_fHcvtout_sec   += r_fHcvtout_stop.tv_sec  - r_fHcvtout_start.tv_sec;
								r_fHcvtout_usec  += r_fHcvtout_stop.tv_usec - r_fHcvtout_start.tv_usec;
								r_fHtotal_sec   += r_fHcvtout_stop.tv_sec  - r_fHcvtin_start.tv_sec;
								r_fHtotal_usec  += r_fHcvtout_stop.tv_usec - r_fHcvtin_start.tv_usec;
#endif // INT_TIME

#else
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
																																								DO_LIMITS_ANALYSIS(if (fft_in_real[j] < min_input) { min_input = fft_in_real[j]; } 
																																																								if (fft_in_real[j] > max_input) { max_input = fft_in_real[j]; }
																																																								if (fft_in_imag[j] < min_input) { min_input = fft_in_imag[j]; } 
																																																								if (fft_in_imag[j] > max_input) { max_input = fft_in_imag[j]; } );
																																}
																								}
																								// Now we have the data ready -- invoke the FFT function
																								DEBUG(printf("\n FFT Call %5u \n", i);
																																								for (unsigned j = 0; j < 64; j++) {
																																								printf("   FFT_IN %4u %2u : %6u %12.8f %12.8f\n", i, j, 64*i+j, fft_in_real[j], fft_in_imag[j]);
																																								});
																								fft_ri(fft_in_real, fft_in_imag, do_inverse, do_shift, num_samples, log_nsamples); // not-inverse, but shifting
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
								} // Non-HWR FFT Scope
#endif
								DEBUG(printf(" num_fft_outs = %u  vs  %u = %u * 64\n", num_fft_outs, num_fft_frames*64, num_fft_frames));
								DEBUG(printf("\nFFT Results at Frame-Eq interface:\n");
																								for (unsigned j = 0; j < num_fft_outs /*FRAME_EQ_IN_MAX_SIZE*/; j++) {
																								printf("  FFT-to-FEQ %6u : %12.8f %12.8f\n", j, fft_ar_r[j], fft_ar_i[j]);
																								}
																								printf("\n"));

								DO_LIMITS_ANALYSIS(printf("DO_RECV_FFT_WORK : min_input = %.15g  max_input = %.15g\n", min_input, max_input));
								return num_fft_outs;
}

/********************************************************************************
	* This routine manages the transmit pipeline functions and components
	********************************************************************************/
void do_recv_pipeline(int num_recvd_vals, float* recvd_in_real, size_t recvd_in_real_sz, 
																float* recvd_in_imag, size_t recvd_in_imag_sz,
																int* recvd_msg_len, size_t recvd_msg_len_sz, char * recvd_msg, size_t recvd_msg_sz,
																/*'Custom' variables used compute() which this function calls*/
																uint8_t* scrambled_msg /*local*/, size_t scrambled_msg_sz /*= MAX_ENCODED_BITS * 3 / 4 */,
																float* ss_freq_offset /*local*/, size_t ss_freq_offset_sz /*=1*/, 
																unsigned* num_sync_short_vals /*local*/, size_t num_sync_short_vals_sz /*=1*/,
																float* sl_freq_offset /*local*/, size_t sl_freq_offset_sz /*=1*/,
																unsigned* num_sync_long_vals /*local*/, size_t num_sync_long_vals_sz /*=1*/,
																fx_pt1* fft_ar_r/*local*/, size_t fft_ar_r_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																fx_pt1* fft_ar_i /*local*/, size_t fft_ar_i_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																unsigned* num_fft_outs /*local*/, size_t num_fft_outs_sz /*=1*/,
																fx_pt* toBeEqualized /*local*/, size_t toBeEqualized_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																fx_pt* equalized /*local*/, size_t equalized_sz /*= FRAME_EQ_OUT_MAX_SIZE*/,
																unsigned* num_eq_out_bits /*local*/, size_t num_eq_out_bits_sz /*=1*/,
																unsigned* psdu /*local*/, size_t psdu_sz /*=1*/,
																// Global variables used by compute() which this function calls
																fx_pt* delay16_out_arg /*= delay16_out -> global*/, size_t delay16_out_arg_sz /*= DELAY_16_MAX_OUT_SIZE*/,
																fx_pt* input_data_arg /*= input_data -> global*/, size_t input_data_arg_sz /*=DELAY_16_MAX_OUT_SIZE - 16*/,
																fx_pt* cmpx_conj_out_arg /*= cmpx_conj_out -> global*/, size_t cmpx_conj_out_arg_sz /*= CMP_CONJ_MAX_SIZE*/,
																fx_pt* cmpx_mult_out_arg /*= cmpx_mult_out -> global*/, size_t cmpx_mult_out_arg_sz /*= CMP_MULT_MAX_SIZE*/,
																fx_pt* correlation_complex_arg /*= correlation_complex -> global*/, size_t correlation_complex_arg_sz /*= FIRC_MAVG48_MAX_SIZE*/,
																fx_pt1* correlation_arg /*= correlation -> global*/, size_t correlation_arg_sz /*= CMP2MAG_MAX_SIZE*/,
																fx_pt1* signal_power_arg /*= signal_power -> global*/, size_t signal_power_arg_sz /*= CMP2MAGSQ_MAX_SIZE*/,
																fx_pt1* avg_signal_power_arg /*= avg_signal_power -> global*/, size_t avg_signal_power_arg_sz /*= FIR_MAVG64_MAX_SIZE*/,
																fx_pt1* the_correlation_arg /*= the_correlation -> global*/, size_t the_correlation_arg_sz /*= DIVIDE_MAX_SIZE*/,
																fx_pt* sync_short_out_frames_arg /*= sync_short_out_frames -> global*/, size_t sync_short_out_frames_arg_sz /*=320*/,
																fx_pt* d_sync_long_out_frames_arg /*= d_sync_long_out_frames -> global*/, size_t d_sync_long_out_frames_arg_sz /*= SYNC_L_OUT_MAX_SIZE*/) {

#if defined(HPVM) && true
																								void * Section = __hetero_section_begin();
#endif

#if defined(HPVM) && true
																								void * T0 = __hetero_task_begin(4, input_data_arg, input_data_arg_sz, 
																																								recvd_in_real, recvd_in_real_sz, 
																																								recvd_in_imag, recvd_in_imag_sz, 
																																								num_recvd_vals, 
																																								1, input_data_arg, input_data_arg_sz, "process_input_to_compute_task");
#endif

																								DEBUG(printf("In do_recv_pipeline: num_received_vals = %u\n", num_recvd_vals); fflush(stdout));
																								for (int i = 0; i < num_recvd_vals; i++) { // TODO: HPVM: Parallelize this loop
																																input_data_arg[i] = recvd_in_real[i] + I * recvd_in_imag[i];
																								}
																								DEBUG(printf("Calling compute\n"));

#if defined(HPVM) && true
																								__hetero_task_end(T0);
#endif

#if defined(HPVM)
																								void * T1 = __hetero_task_begin(26,
																																								num_recvd_vals,
																																								recvd_msg_len, recvd_msg_len_sz,
																																								recvd_msg, recvd_msg_sz,
																																								// Local variables used by compute()
																																								scrambled_msg, scrambled_msg_sz,
																																								ss_freq_offset, ss_freq_offset_sz,
																																								num_sync_short_vals, num_sync_short_vals_sz,
																																								sl_freq_offset, sl_freq_offset_sz,
																																								num_sync_long_vals, num_sync_long_vals_sz,
																																								fft_ar_r, fft_ar_r_sz,
																																								fft_ar_i, fft_ar_i_sz,
																																								num_fft_outs, num_fft_outs_sz,
																																								toBeEqualized, toBeEqualized_sz,
																																								equalized, equalized_sz,
																																								num_eq_out_bits, num_eq_out_bits_sz,
																																								psdu, psdu_sz,
																																								// Globals used by compute()
																																								delay16_out_arg, delay16_out_arg_sz,
																																								input_data_arg, input_data_arg_sz,
																																								cmpx_conj_out_arg, cmpx_conj_out_arg_sz,
																																								cmpx_mult_out_arg, cmpx_mult_out_arg_sz,
																																								correlation_complex_arg, correlation_complex_arg_sz,
																																								correlation_arg, correlation_arg_sz,
																																								signal_power_arg, signal_power_arg_sz,
																																								avg_signal_power_arg, avg_signal_power_arg_sz,
																																								the_correlation_arg, the_correlation_arg_sz,
																																								sync_short_out_frames_arg, sync_short_out_frames_arg_sz,
																																								d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz,
																																								3,
																																								input_data_arg, input_data_arg_sz,
																																								recvd_msg_len, recvd_msg_len_sz, recvd_msg, recvd_msg_sz,
																																								"recv_pipeline_impl_task");

#endif

#if defined(INT_TIME) && !defined(HPVM)
																								gettimeofday(&r_pipe_start, NULL);
#endif

																								compute(num_recvd_vals, input_data_arg, input_data_arg_sz, 
																																								recvd_msg_len, recvd_msg_len_sz, 
																																								(uint8_t*)recvd_msg, recvd_msg_sz,
																																								// Local variables for compute
																																								scrambled_msg, scrambled_msg_sz,
																																								ss_freq_offset, ss_freq_offset_sz, 
																																								num_sync_short_vals, num_sync_short_vals_sz,
																																								sl_freq_offset, sl_freq_offset_sz,
																																								num_sync_long_vals, num_sync_long_vals_sz,
																																								fft_ar_r, fft_ar_r_sz,
																																								fft_ar_i, fft_ar_i_sz,
																																								num_fft_outs, num_fft_outs_sz,
																																								toBeEqualized, toBeEqualized_sz,
																																								equalized, equalized_sz,
																																								num_eq_out_bits, num_eq_out_bits_sz,
																																								psdu, psdu_sz,
																																								// Globals used by compute
																																								delay16_out_arg, delay16_out_arg_sz,
																																								cmpx_conj_out_arg, cmpx_conj_out_arg_sz,
																																								cmpx_mult_out_arg, cmpx_mult_out_arg_sz,
																																								correlation_complex_arg, correlation_complex_arg_sz,
																																								correlation_arg, correlation_arg_sz,
																																								signal_power_arg, signal_power_arg_sz,
																																								avg_signal_power_arg, avg_signal_power_arg_sz,
																																								the_correlation_arg, the_correlation_arg_sz,
																																								sync_short_out_frames_arg, sync_short_out_frames_arg_sz,
																																								d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz);

#if defined(INT_TIME) && !defined(HPVM)
																								gettimeofday(&r_pipe_stop, NULL);
																								r_pipe_sec  += r_pipe_stop.tv_sec  - r_pipe_start.tv_sec;
																								r_pipe_usec += r_pipe_stop.tv_usec - r_pipe_start.tv_usec;
#endif

#if !defined(HPVM)
																								DEBUG(printf("CMP_MSG:\n%s\n", recvd_msg));
#endif

#if defined(HPVM)
																								__hetero_task_end(T1);
#endif

#if defined(HPVM) && true
																								__hetero_section_end(Section);
#endif
																}

void gr_equalize_test( float wifi_start, unsigned num_inputs,
                fx_pt inputs[FRAME_EQ_IN_MAX_SIZE],
                  unsigned* msg_psdu,
                  unsigned* num_out_bits, uint8_t outputs[FRAME_EQ_OUT_MAX_SIZE],
                  unsigned* num_out_sym, fx_pt out_symbols[FRAME_EQ_OUT_MAX_SIZE]
                ) {
								*num_out_sym = *num_out_bits;
        printf("Call works");
}

void compute(unsigned num_inputs, fx_pt* input_data_arg, size_t input_data_arg_sz, 
																int* out_msg_len, size_t out_msg_len_sz, uint8_t *out_msg, size_t out_msg_sz,
																uint8_t* scrambled_msg /*local*/, size_t scrambled_msg_sz /*= MAX_ENCODED_BITS * 3 / 4 */,
																// Local variables for compute
																float* ss_freq_offset /*local*/, size_t ss_freq_offset_sz /*=1*/, 
																unsigned* num_sync_short_vals /*local*/, size_t num_sync_short_vals_sz /*=1*/,
																float* sl_freq_offset /*local*/, size_t sl_freq_offset_sz /*=1*/,
																unsigned* num_sync_long_vals /*local*/, size_t num_sync_long_vals_sz /*=1*/,
																fx_pt1* fft_ar_r/*local*/, size_t fft_ar_r_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																fx_pt1* fft_ar_i /*local*/, size_t fft_ar_i_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																unsigned* num_fft_outs /*local*/, size_t num_fft_outs_sz /*=1*/,
																fx_pt* toBeEqualized /*local*/, size_t toBeEqualized_sz /*= FRAME_EQ_IN_MAX_SIZE*/,
																fx_pt* equalized /*local*/, size_t equalized_sz /*= FRAME_EQ_OUT_MAX_SIZE*/,
																unsigned* num_eq_out_bits /*local*/, size_t num_eq_out_bits_sz /*=1*/,
																unsigned* psdu /*local*/, size_t psdu_sz /*=1*/,
																// Globals used by compute
																fx_pt* delay16_out_arg /*= delay16_out -> global*/, size_t delay16_out_arg_sz /*= DELAY_16_MAX_OUT_SIZE*/,
																fx_pt* cmpx_conj_out_arg /*= cmpx_conj_out -> global*/, size_t cmpx_conj_out_arg_sz /*= CMP_CONJ_MAX_SIZE*/,
																fx_pt* cmpx_mult_out_arg /*= cmpx_mult_out -> global*/, size_t cmpx_mult_out_arg_sz /*= CMP_MULT_MAX_SIZE*/,
																fx_pt* correlation_complex_arg /*= correlation_complex -> global*/, size_t correlation_complex_arg_sz /*= FIRC_MAVG48_MAX_SIZE*/,
																fx_pt1* correlation_arg /*= correlation -> global*/, size_t correlation_arg_sz /*= CMP2MAG_MAX_SIZE*/,
																fx_pt1* signal_power_arg /*= signal_power -> global*/, size_t signal_power_arg_sz /*= CMP2MAGSQ_MAX_SIZE*/,
																fx_pt1* avg_signal_power_arg /*= avg_signal_power -> global*/, size_t avg_signal_power_arg_sz /*= FIR_MAVG64_MAX_SIZE*/,
																fx_pt1* the_correlation_arg /*= the_correlation -> global*/, size_t the_correlation_arg_sz /*= DIVIDE_MAX_SIZE*/,
																fx_pt* sync_short_out_frames_arg /*= sync_short_out_frames -> global*/, size_t sync_short_out_frames_arg_sz /*=320*/,
																fx_pt* d_sync_long_out_frames_arg /*= d_sync_long_out_frames -> global*/, size_t d_sync_long_out_frames_arg_sz /*= SYNC_L_OUT_MAX_SIZE*/) {

#if defined(HPVM) && true
																								void * Section = __hetero_section_begin();
#endif

#if defined(HPVM) && true
																								// All tasks must have an output (why?). So, although this task doesn't modify anything, it still has a output
																								void * T0 = __hetero_task_begin(2, num_inputs, input_data_arg, input_data_arg_sz, 
																																								1, input_data_arg, input_data_arg_sz, "logging_task");
#endif
																								{
																																// uint8_t scrambled_msg[MAX_ENCODED_BITS * 3 / 4];
																																DEBUG(for (int ti = 0; ti < num_inputs /*RAW_DATA_IN_MAX_SIZE*/; ti++) {
																																																printf("  %6u : TOP_INBUF %12.8f %12.8f\n", ti, crealf(input_data_arg[ti]), cimagf(input_data_arg[ti]));
																																																});

																																unsigned num_del16_vals = num_inputs + 16;
																																DO_NUM_IOS_ANALYSIS(printf("Calling delay IN %u OUT %u\n", num_inputs, num_del16_vals));
																																// We don't need to make this call now -- we've done the effect in the memory array already
																																// delay(delay16_out, num_inputs, input_data_arg);  
																																DEBUG(for (int ti = 0; ti < num_del16_vals; ti++) {
																																																printf(" DEL16 %5u : TMPBUF %12.8f %12.8f : INBUF %12.8f %12.8f\n", ti, crealf(delay16_out[ti]), 
																																																																cimagf(delay16_out[ti]), crealf(input_data_arg[ti]), cimagf(input_data_arg[ti]));
																																																});
																								}
#if defined(HPVM) && true
																								__hetero_task_end(T0);
#endif

#if defined(HPVM) 
																								void * T1 = __hetero_task_begin(3, num_inputs, cmpx_conj_out_arg, cmpx_conj_out_arg_sz, 
																																								delay16_out_arg, delay16_out_arg_sz,
																																								1, cmpx_conj_out_arg, cmpx_conj_out_arg_sz, "cmplx_conj_task");
#endif

																								{
																																unsigned num_del16_vals = num_inputs + 16;
																																unsigned num_cconj_vals = num_del16_vals;
																																DO_NUM_IOS_ANALYSIS(printf("Calling complex_conjugate: IN %u OUT %u\n", num_del16_vals, num_cconj_vals));
#ifdef INT_TIME
																																gettimeofday(&r_cmpcnj_start, NULL);
#endif
																																complex_conjugate(cmpx_conj_out_arg, num_del16_vals, delay16_out_arg);
																																DEBUG(for (int ti = 0; ti < num_cconj_vals; ti++) {
																																																printf("  CMP_CONJ %5u : CONJ_OUT %12.8f %12.8f : DEL16_IN %12.8f %12.8f\n", ti, 
																																																																crealf(cmpx_conj_out_arg[ti]), cimagf(cmpx_conj_out_arg[ti]), crealf(delay16_out_arg[ti]), 
																																																																cimagf(delay16_out_arg[ti]));
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T1);
#endif

#if defined(HPVM) 
																								void * T2 = __hetero_task_begin(4, cmpx_mult_out_arg, cmpx_mult_out_arg_sz, 
																																								cmpx_conj_out_arg, cmpx_conj_out_arg_sz, num_inputs,
																																								input_data_arg, input_data_arg_sz,
																																								1, cmpx_mult_out_arg, cmpx_mult_out_arg_sz, "cmplx_mult_task");
#endif
																								{
																																unsigned num_del16_vals = num_inputs + 16;
																																// TODO: I think num_cconj_vals and num_del16_vals are identitcal as complex_conjugate (prior task) doesn't modify it; relying on that assumption, I am num_cmult_vals defined in this manner
																																unsigned num_cmult_vals = num_del16_vals; //num_cconj_vals; 
																																DO_NUM_IOS_ANALYSIS(printf("Calling complex_mult: IN %u OUT %u\n", num_del16_vals, num_cmult_vals));

#ifdef INT_TIME
																																gettimeofday(&r_cmpmpy_start, NULL);
																																r_cmpcnj_sec  += r_cmpmpy_start.tv_sec  - r_cmpcnj_start.tv_sec;
																																r_cmpcnj_usec += r_cmpmpy_start.tv_usec - r_cmpcnj_start.tv_usec;
#endif
																																// TODO: HPVM the function has a for-loop that can be parallellized
																																complex_multiply(cmpx_mult_out_arg, num_cmult_vals, cmpx_conj_out_arg, input_data_arg);

																																DEBUG(for (int ti = 0; ti < num_cmult_vals; ti++) {
																																																printf("  CMP_MULT %5u : MULT_OUT %12.8f %12.8f : CMP_CONJ %12.8f %12.8f : INBUF %12.8f %12.8f\n", ti, crealf(cmpx_mult_out[ti]), cimagf(cmpx_mult_out[ti]), crealf(cmpx_conj_out[ti]), cimagf(cmpx_conj_out[ti]), crealf(input_data_arg[ti]), cimagf(input_data_arg[ti]));
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T2);
#endif

#if defined(HPVM) 
																								void * T3 = __hetero_task_begin(3, correlation_complex_arg, correlation_complex_arg_sz, 
																																								cmpx_mult_out_arg, cmpx_mult_out_arg_sz, num_inputs,
																																								1, correlation_complex_arg, correlation_complex_arg_sz, "ffirc_task");
#endif
																								{
																																unsigned num_del16_vals = num_inputs + 16;
																																// TODO: I think num_cmult_vals and num_del16_vals are identitcal as prior tasks don't modify it; relying on that assumption, I am num_cmult_vals defined in this manner
																																unsigned num_cmpcorr_vals = num_del16_vals;
																																DO_NUM_IOS_ANALYSIS(printf("Calling firc (Moving Average 48) : IN %u OUT %u\n", num_del16_vals, num_cmpcorr_vals));

#ifdef INT_TIME
																																gettimeofday(&r_firc_start, NULL);
																																r_cmpmpy_sec  += r_firc_start.tv_sec  - r_cmpmpy_start.tv_sec;
																																r_cmpmpy_usec += r_firc_start.tv_usec - r_cmpmpy_start.tv_usec;
#endif
																																//firc(correlation_complex, cmpx_mult_out);
																																ffirc(correlation_complex_arg, num_cmpcorr_vals, cmpx_mult_out_arg);

																																DEBUG(for (int ti = 0; ti < num_cmpcorr_vals; ti++) {
																																																printf("  MV_AVG48 %5u : CORR_CMP %12.8f %12.8f : MULT_OUT %12.8f %12.8f\n", ti, 
																																																																crealf(correlation_complex_arg[ti]), cimagf(correlation_complex_arg[ti]), 
																																																																crealf(cmpx_mult_out_arg[ti]), cimagf(cmpx_mult_out_arg[ti]));
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T3);
#endif

#if defined(HPVM) 
																								void * T4 = __hetero_task_begin(3, correlation_arg, correlation_arg_sz, num_inputs,
																																								correlation_complex_arg, correlation_complex_arg_sz, 
																																								1, correlation_arg, correlation_arg_sz, "cmplx_mag_task");
#endif
																								{
																																unsigned num_del16_vals = num_inputs + 16;
																																// TODO: I think num_cmpcorr_vals and num_del16_vals are identitcal as prior tasks don't modify it; relying on that assumption, I am num_cmult_vals defined in this manner
																																unsigned num_cmag_vals = num_del16_vals; //num_cmpcorr_vals;
																																DO_NUM_IOS_ANALYSIS(printf("Calling complex_to_magnitude: IN %u OUT %u\n", num_del16_vals, num_cmag_vals));


#ifdef INT_TIME
																																gettimeofday(&r_cmpmag_start, NULL);
																																r_firc_sec  += r_cmpmag_start.tv_sec  - r_firc_start.tv_sec;
																																r_firc_usec += r_cmpmag_start.tv_usec - r_firc_start.tv_usec;
#endif
																																complex_to_magnitude(correlation_arg, num_del16_vals, correlation_complex_arg);

																																DEBUG(for (unsigned ti = 0; ti < num_cmag_vals; ti++) {
																																																printf("  MAGNITUDE %5u : CORR %12.8f : CORR_CMP %12.8f %12.8f\n", ti, 
																																																																correlation_arg[ti], crealf(correlation_complex_arg[ti]), cimagf(correlation_complex_arg[ti]));
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T4);
#endif

#if defined(HPVM) 
																								void * T5 = __hetero_task_begin(3, signal_power_arg, signal_power_arg_sz, num_inputs,
																																								input_data_arg, input_data_arg_sz,
																																								1, signal_power_arg, signal_power_arg_sz, "cmplx_mag2_task");
#endif
																								{
																																unsigned num_cmag2_vals = num_inputs;
																																DO_NUM_IOS_ANALYSIS(printf("Calling complex_to_mag_squared (signal_power): IN %u OUT %u\n", num_inputs, num_cmag2_vals));
#ifdef INT_TIME
																																gettimeofday(&r_cmpmag2_start, NULL);
																																r_cmpmag_sec  += r_cmpmag2_start.tv_sec  - r_cmpmag_start.tv_sec;
																																r_cmpmag_usec += r_cmpmag2_start.tv_usec - r_cmpmag_start.tv_usec;
#endif
																																complex_to_mag_squared(signal_power_arg, num_inputs, input_data_arg);

																																DEBUG(for (int ti = 0; ti < num_cmag2_vals; ti++) {
																																																printf("  MAG^2 %5u : SIGN_PWR %12.8f : INBUF %12.8f %12.8f\n", ti, signal_power_arg[ti], 
																																																																crealf(input_data_arg[ti]), cimagf(input_data_arg[ti]));
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T5);
#endif

#if defined(HPVM) 
																								void * T6 = __hetero_task_begin(3, signal_power_arg, signal_power_arg_sz, 
																																								avg_signal_power_arg, avg_signal_power_arg_sz, num_inputs,
																																								1, avg_signal_power_arg, avg_signal_power_arg_sz, "ffir_task");
#endif
																								{
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

																																unsigned num_cmag2_vals = num_inputs; // copied from previous task (as the task doesn't change the value of this)
																																unsigned num_mavg64_vals = num_cmag2_vals;
																																DO_NUM_IOS_ANALYSIS(printf("Calling fir (Moving Average 64) : IN %u OUT %u\n", num_cmag2_vals, num_mavg64_vals));

#ifdef INT_TIME
																																gettimeofday(&r_fir_start, NULL);
																																r_cmpmag2_sec  += r_fir_start.tv_sec  - r_cmpmag2_start.tv_sec;
																																r_cmpmag2_usec += r_fir_start.tv_usec - r_cmpmag2_start.tv_usec;
#endif
																																//fir(avg_signal_power, signal_power, coeff_mvgAvg);
																																ffir(avg_signal_power_arg, num_cmag2_vals, signal_power_arg);

																																DEBUG(for (unsigned ti = 0; ti < num_mavg64_vals; ti++) {
																																																printf(" TOP_FIR_OUT %5u : SIGN_PWR-AVG %12.8f : SIGN_PWR %12.8f\n", ti, avg_signal_power_arg[ti], signal_power_arg[ti]);
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T6);
#endif

#if defined(HPVM) 
																								void * T7 = __hetero_task_begin(2, cmpx_conj_out_arg, cmpx_conj_out_arg_sz, num_inputs,
																																								1, cmpx_conj_out_arg, cmpx_conj_out_arg_sz, "detect_early_exit_task");
#endif
																								{
																																// TODO: I have copied the definition of num_cmag_vals from task T4. Based on that task, Ithink num_cmag_vals and num_del16_vals are identitcal as prior tasks don't modify it; relying on that assumption, I have num_cmag_vals defined in this manner
																																unsigned num_del16_vals = num_inputs + 16;
																																unsigned num_cmag_vals = num_del16_vals; //num_cmpcorr_vals;

																																unsigned num_mavg64_vals = num_inputs; // copied from previous task (as prior tasks doesn't change the value of this)
																																unsigned num_cdiv_vals = num_mavg64_vals; // num_cmpcorr_vals;
																																DO_NUM_IOS_ANALYSIS(printf("Calling division: IN %u OUT %u : CMAG %u \n", num_mavg64_vals,  num_cdiv_vals, num_cmag_vals));
																																// Ensure we've picked the MIN(num_mavg64_vals, num_cmag_vals) -- should have an a-priori known relationship!
																																if (num_mavg64_vals > num_cmag_vals) {
																																								printf("ERROR : num_mavg64_vals = %u > %u = num_cmag_vals\n", num_mavg64_vals, num_cmag_vals);
																																								// TODO: The following exit call causes the warning "Begin and end marker functions are not correctly nested!" 
																																								// This is expected; but as along the this if path isn't take at runtime everything will run as expected
																																								exit(-8); 
																																}
																								}
#if defined(HPVM) 
																								__hetero_task_end(T7);
#endif

#if defined(HPVM) 
																								void * T8 = __hetero_task_begin(4, correlation_arg, correlation_arg_sz, num_inputs,
																																								avg_signal_power_arg, avg_signal_power_arg_sz, the_correlation_arg, the_correlation_arg_sz, 
																																								1, the_correlation_arg, the_correlation_arg_sz, "division_task");
#endif
																								{
#ifdef INT_TIME
																																gettimeofday(&r_div_start, NULL);
																																r_fir_sec  += r_div_start.tv_sec  - r_fir_start.tv_sec;
																																r_fir_usec += r_div_start.tv_usec - r_fir_start.tv_usec;
#endif
																																unsigned num_mavg64_vals = num_inputs; // copied from previous task (as prior tasks doesn't change the value of this)
																																unsigned num_cdiv_vals = num_mavg64_vals; // num_cmpcorr_vals;

																																division(the_correlation_arg, num_mavg64_vals, correlation_arg, avg_signal_power_arg);

																																DEBUG(for (unsigned ti = 0; ti < num_cdiv_vals; ti++) {
																																																printf(" TOP_DIV_OUT %5u : CORRZ %12.8f : CORRL %12.8f : SPA %12.8f\n", ti, the_correlation_arg[ti], 
																																																																correlation_arg[ti], avg_signal_power_arg[ti]);
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T8);
#endif

#if defined(HPVM) 
																								void * T9 = __hetero_task_begin(7, correlation_complex_arg, correlation_complex_arg_sz, 
																																								sync_short_out_frames_arg, sync_short_out_frames_arg_sz, 
																																								the_correlation_arg, the_correlation_arg_sz, ss_freq_offset, ss_freq_offset_sz, 
																																								num_sync_short_vals, num_sync_short_vals_sz, 
																																								delay16_out_arg, delay16_out_arg_sz, num_inputs,
																																								3, sync_short_out_frames_arg, sync_short_out_frames_arg_sz, 
																																								ss_freq_offset, ss_freq_offset_sz, num_sync_short_vals, num_sync_short_vals_sz, 
																																								"sync_short_task");
#endif
																								{
																																unsigned num_mavg64_vals = num_inputs; // copied from previous task (as prior tasks doesn't change the value of this)
																																unsigned num_cdiv_vals = num_mavg64_vals; // num_cmpcorr_vals;

																																DO_NUM_IOS_ANALYSIS(printf("Calling sync_short: IN %u\n", num_cdiv_vals));
																																// float ss_freq_offset;
																																// unsigned num_sync_short_vals;
#ifdef INT_TIME
																																gettimeofday(&r_sshort_start, NULL);
																																r_div_sec  += r_sshort_start.tv_sec  - r_div_start.tv_sec;
																																r_div_usec += r_sshort_start.tv_usec - r_div_start.tv_usec;
#endif
																																sync_short(num_cdiv_vals, delay16_out_arg, correlation_complex_arg, the_correlation_arg, ss_freq_offset, 
																																																num_sync_short_vals, sync_short_out_frames_arg);

																																DO_NUM_IOS_ANALYSIS(printf("Back from sync_short: OUT num_sync_short_vals = %u\n", *num_sync_short_vals));
																																DEBUG(printf(" ss_freq_offset = %12.8f  and num sync_short_values = %u\n", *ss_freq_offset, 
																																																								*num_sync_short_vals);

																																																for (int ti = 0; ti < *num_sync_short_vals; ti++) {
																																																printf(" S_S_OUT %5u : TMPBUF %12.8f %12.8f : CORR_CMP %12.8f %12.8f : CORRZ %12.8f : SS_FRAME %12.8f" + 
																																																																" %12.8f\n", ti, crealf(delay16_out_arg[ti]), cimagf(delay16_out_arg[ti]), 
																																																																crealf(correlation_complex_arg[ti]), cimagf(correlation_complex_arg[ti]), the_correlation_arg[ti], 
																																																																crealf(sync_short_out_frames_arg[ti]), cimagf(sync_short_out_frames_arg[ti])); 
																																																});

																																DO_NUM_IOS_ANALYSIS(printf("Calling delay320: IN %u OUT %u\n", *num_sync_short_vals, 
																																																								320+(*num_sync_short_vals)));
																																// We've used a similar memory-placement trick to avoid having to do the delay320 function; sync_short_out_frames = &(frame_d[320])
																																//delay320(frame_d, synch_short_out_frames);
																																DEBUG(for (int ti = 0; ti < 320+(*num_sync_short_vals); ti++) {
																																																printf("  DELAY_320 %5u : FRAME_D %12.8f %12.8f : FRAME %12.8f %12.8f\n", ti, 
																																																																crealf(frame_d[ti]), cimagf(frame_d[ti]), crealf(sync_short_out_frames_arg[ti]), 
																																																																cimagf(sync_short_out_frames_arg[ti]));
																																																});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T9);
#endif

#if defined(HPVM) 
																								void * T10 = __hetero_task_begin(5, sl_freq_offset, sl_freq_offset_sz, 
																																								num_sync_long_vals, num_sync_long_vals_sz, // ss_freq_offset, ss_freq_offset_sz, 
																																								num_sync_short_vals, num_sync_short_vals_sz, 
																																								sync_short_out_frames_arg, sync_short_out_frames_arg_sz, 
																																								d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz, 
																																								3, sl_freq_offset, sl_freq_offset_sz, num_sync_long_vals, num_sync_long_vals_sz, 
																																								d_sync_long_out_frames_arg, d_sync_long_out_frames_arg_sz, "sync_long_task");
#endif

																								DO_NUM_IOS_ANALYSIS(printf("Calling sync_long: IN %u\n", *num_sync_short_vals));
																								// float sl_freq_offset;
																								// unsigned num_sync_long_vals;
#ifdef INT_TIME
																								gettimeofday(&r_slong_start, NULL);
																								r_sshort_sec  += r_slong_start.tv_sec  - r_sshort_start.tv_sec;
																								r_sshort_usec += r_slong_start.tv_usec - r_sshort_start.tv_usec;
#endif
																								sync_long(*num_sync_short_vals, sync_short_out_frames_arg, frame_d, sl_freq_offset, num_sync_long_vals, 
																																								d_sync_long_out_frames_arg);
																								DO_NUM_IOS_ANALYSIS(printf("Back from synch_long: OUT num_sync_long_vals = %u\n", *num_sync_long_vals));
																								DEBUG(printf(" sl_freq_offset = %12.8f\n", *sl_freq_offset);
																																								for (int ti = 0; ti < 32619; ti++) {
																																								printf("  SYNC_LONG_OUT %5u  %12.8f %12.8f : FR_D %12.8f %12.8f : D_FR_L %12.8f %12.8f\n", ti, 
																																																								crealf(sync_short_out_frames_arg[ti]), cimagf(sync_short_out_frames_arg[ti]), 
																																																								crealf(frame_d[ti]), cimagf(frame_d[ti]), crealf(d_sync_long_out_frames_arg[ti]), 
																																																								cimagf(d_sync_long_out_frames_arg[ti])); 
																																								});

#if defined(HPVM) 
																								__hetero_task_end(T10);
#endif

#if defined(HPVM) 
																								void * T11 = __hetero_task_begin(4, num_sync_long_vals, num_sync_long_vals_sz, fft_ar_r, fft_ar_r_sz, 
																																								fft_ar_i, fft_ar_i_sz, num_fft_outs, num_fft_outs_sz, 
																																								4, num_sync_long_vals, num_sync_long_vals_sz, fft_ar_r, fft_ar_r_sz, 
																																								fft_ar_i, fft_ar_i_sz, num_fft_outs, num_fft_outs_sz, "rcv_fft_task");
#endif

																								// fx_pt1 fft_ar_r[FRAME_EQ_IN_MAX_SIZE];
																								// fx_pt1 fft_ar_i[FRAME_EQ_IN_MAX_SIZE];
																								{
																																unsigned num_fft_frames = ((*num_sync_long_vals)+63) / 64;
																																DO_NUM_IOS_ANALYSIS(printf("Calling FFT_COMP : num_fft_frames = %u / 64 = %u So %u values\n", 
																																																								*num_sync_long_vals,  num_fft_frames, (num_fft_frames * 64)));
#ifdef INT_TIME
																																gettimeofday(&r_fft_start, NULL);
																																r_slong_sec  += r_fft_start.tv_sec  - r_slong_start.tv_sec;
																																r_slong_usec += r_fft_start.tv_usec - r_slong_start.tv_usec;
#endif
																																*num_fft_outs = do_rcv_fft_work(num_fft_frames, fft_ar_r, fft_ar_i);
#ifdef INT_TIME
																																gettimeofday(&r_fft_stop, NULL);
																																r_fft_sec  += r_fft_stop.tv_sec  - r_fft_start.tv_sec;
																																r_fft_usec += r_fft_stop.tv_usec - r_fft_start.tv_usec;
#endif
																								}
#if defined(HPVM) 
																								__hetero_task_end(T11);
#endif

#if defined(HPVM) 
																								void * T12 = __hetero_task_begin(10, fft_ar_r, fft_ar_r_sz, fft_ar_i, fft_ar_i_sz, 
																																								num_fft_outs, num_fft_outs_sz, toBeEqualized, toBeEqualized_sz, 
																																								equalized, equalized_sz, ss_freq_offset, ss_freq_offset_sz, 
																																								sl_freq_offset, sl_freq_offset_sz, num_eq_out_bits, num_eq_out_bits_sz, psdu, psdu_sz, 
																																								num_sync_long_vals, num_sync_long_vals_sz,
																																								4, toBeEqualized, toBeEqualized_sz, equalized, equalized_sz, 
																																								num_eq_out_bits, num_eq_out_bits_sz, psdu, psdu_sz, "gr_equalize_task");
#endif

																								// equalize
																								// fx_pt toBeEqualized[FRAME_EQ_IN_MAX_SIZE];
																								// fx_pt equalized[FRAME_EQ_OUT_MAX_SIZE];

																								{
																																unsigned num_fft_frames = ((*num_sync_long_vals)+63) / 64;
																																// NOTE: The 509 == FFT_OUT_SIZE / 64
																																for(unsigned i = 0; i < num_fft_frames /*509*/ * 64; i++) {
																																								toBeEqualized[i] = (fx_pt)(fft_ar_r[i] + fft_ar_i[i] * I);
																																}
																																float wifi_start = (*ss_freq_offset) - (*sl_freq_offset);

																																uint8_t equalized_bits[FRAME_EQ_OUT_MAX_SIZE];
																																for (int ii = 0; ii < FRAME_EQ_OUT_MAX_SIZE; ii++) {
																																								equalized_bits[ii] = 0;
																																}

																																DEBUG(printf("\nCalling gr_equalize (frame_equalizer) with wifi_start = %12.8f\n", wifi_start));
																																DO_NUM_IOS_ANALYSIS(printf("Calling gr_equalize (frame_equalizer): IN %u\n", *num_fft_outs));
																																// unsigned num_eq_out_bits;
																																unsigned num_eq_out_sym;
#ifdef INT_TIME
																																gettimeofday(&r_eqlz_start, NULL);
#endif
																																gr_equalize(wifi_start, *num_fft_outs, toBeEqualized, psdu, num_eq_out_bits, equalized_bits, 
																																												&num_eq_out_sym, equalized); 


																																DEBUG(printf("GR_FR_EQ : fft_outs = %u items %u ffts : num_eq_out_bits = %u %u : num_eq_out_sym = %u\n", 
																																																								*num_fft_outs, (*num_fft_outs)/64, *num_eq_out_bits, (*num_eq_out_bits)/48, num_eq_out_sym));
																																DO_NUM_IOS_ANALYSIS(printf("Back from gr_equalize : OUT %u\n", *num_eq_out_bits));

																																DEBUG(for (int ti = 0; ti < *num_eq_out_bits; ti++) {
																																																printf(" FR_EQ_OUT %5u : toBeEQ %12.8f %12.8f : EQLZD %12.8f %12.8f : EQ_BIT %u\n", ti, 
																																																								crealf(toBeEqualized[ti]), cimagf(toBeEqualized[ti]), crealf(equalized[ti]), 
																																																								cimagf(equalized[ti]), equalized_bits[ti]);
																																						});
																								}
#if defined(HPVM) 
																								__hetero_task_end(T12);
#endif

#if defined(HPVM) 
																								void * T13 = __hetero_task_begin(4, num_eq_out_bits, num_eq_out_bits_sz, toBeEqualized, toBeEqualized_sz, 
																																								equalized, equalized_sz, scrambled_msg, scrambled_msg_sz, 
																																								1, scrambled_msg, scrambled_msg_sz, "decode_task");
#endif

																								//decode signal
																								DO_NUM_IOS_ANALYSIS(printf("Calling decode_signal: IN %u\n", *num_eq_out_bits));
																								unsigned num_dec_bits;
#ifdef INT_TIME
																								gettimeofday(&r_decsignl_start, NULL);
																								r_eqlz_sec  += r_decsignl_start.tv_sec  - r_eqlz_start.tv_sec;
																								r_eqlz_usec += r_decsignl_start.tv_usec - r_eqlz_start.tv_usec;
#endif
																								decode_signal(*num_eq_out_bits, equalized, &num_dec_bits, scrambled_msg);
																								DO_NUM_IOS_ANALYSIS(printf("Back from decode_signal: OUT %u\n", num_dec_bits));
																								DEBUG(for (int ti = 0; ti < num_dec_bits; ti++) {
																																								printf(" DEC_OUTS %5u : EQLZD %12.8f %12.8f : DEC_BIT %u\n", ti, crealf(toBeEqualized[ti]), 
																																																								cimagf(toBeEqualized[ti]), scrambled_msg[ti]);
																																								});

#if defined(HPVM) 
																								__hetero_task_end(T13);
#endif

#if defined(HPVM) 
																								void * T14 = __hetero_task_begin(4, scrambled_msg, scrambled_msg_sz, psdu, psdu_sz, 
																																								out_msg_len, out_msg_len_sz, out_msg, out_msg_sz, 
																																								2, out_msg_len, out_msg_len_sz, out_msg, out_msg_sz, "sdr_descrambler_task");
#endif

																								//descrambler
																								DO_NUM_IOS_ANALYSIS(printf("Calling sdr_descrambler with psdu = %u\n", *psdu));
#ifdef INT_TIME
																								gettimeofday(&r_descrmbl_start, NULL);
																								r_decsignl_sec  += r_descrmbl_start.tv_sec  - r_decsignl_start.tv_sec;
																								r_decsignl_usec += r_descrmbl_start.tv_usec - r_decsignl_start.tv_usec;
#endif
																								sdr_descrambler(scrambled_msg, *psdu, (char*)out_msg);
#ifdef INT_TIME
																								gettimeofday(&r_descrmbl_stop, NULL);
																								r_descrmbl_sec  += r_descrmbl_stop.tv_sec  - r_descrmbl_start.tv_sec;
																								r_descrmbl_usec += r_descrmbl_stop.tv_usec - r_descrmbl_start.tv_usec;
#endif
																								DEBUG(printf("\nDESC_MSG:\n%s\n", out_msg));
																								*out_msg_len = (*psdu - 28); // The message length in bytes

#if defined(HPVM) 
																								__hetero_task_end(T14);
#endif

#if defined(HPVM) && true
																								__hetero_section_end(Section);
#endif

																}
