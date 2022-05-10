/*
 * Copyright 2019 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _RECV_PIPE_H
#define _RECV_PIPE_H

#include "sdr_base.h"
#include "gr_equalizer.h"  // for INT_TIME extern declarations
#include "sync_short.h"    // for INT_TIME extern declarations
#include "sync_long.h"     // for INT_TIME extern declarations
#include "ofdm.h"          // for INT_TIME extern declarations

#ifdef INT_TIME
/* This is RECV PIPE internal Timing information (gathering resources) */
extern uint64_t r_pipe_sec;
extern uint64_t r_pipe_usec;

extern uint64_t r_cmpcnj_sec;
extern uint64_t r_cmpcnj_usec;

extern uint64_t r_cmpmpy_sec;
extern uint64_t r_cmpmpy_usec;

extern uint64_t r_firc_sec;
extern uint64_t r_firc_usec;

extern uint64_t r_cmpmag_sec;
extern uint64_t r_cmpmag_usec;

extern uint64_t r_cmpmag2_sec;
extern uint64_t r_cmpmag2_usec;

extern uint64_t r_fir_sec;
extern uint64_t r_fir_usec;

extern uint64_t r_div_sec;
extern uint64_t r_div_usec;

extern uint64_t r_sshort_sec;
extern uint64_t r_sshort_usec;

extern uint64_t r_slong_sec;
extern uint64_t r_slong_usec;

extern uint64_t r_fft_sec;
extern uint64_t r_fft_usec;

extern uint64_t r_eqlz_sec;
extern uint64_t r_eqlz_usec;

extern uint64_t r_decsignl_sec;
extern uint64_t r_decsignl_usec;

extern uint64_t r_descrmbl_sec;
extern uint64_t r_descrmbl_usec;

extern uint64_t r_zz_sec;
extern uint64_t r_zz_usec;

#ifdef RECV_HW_FFT

extern uint64_t r_fHtotal_sec;
extern uint64_t r_fHtotal_usec;

extern uint64_t r_fHcvtin_sec;
extern uint64_t r_fHcvtin_usec;

extern uint64_t r_fHcomp_sec;
extern uint64_t r_fHcomp_usec;

extern uint64_t r_fHcvtout_sec;
extern uint64_t r_fHcvtout_usec;
#endif
#endif

void recv_pipe_init();

#ifdef HPVM
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
                      unsigned* psdu /*local*/, size_t psdu_sz /*=1*/);
#else
void do_recv_pipeline(int n_in, float* in_real, float* in_imag, int* out_msg_len, char* out_msg);
#endif


#endif
