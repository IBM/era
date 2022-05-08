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

#ifndef _XMIT_PIPE_H
#define _XMIT_PIPE_H

#include "sdr_base.h"

#ifdef INT_TIME
/* This is XMIT PIPE internal Timing information (gathering resources) */
extern uint64_t x_pipe_sec;
extern uint64_t x_pipe_usec;

extern uint64_t x_genmacfr_sec;
extern uint64_t x_genmacfr_usec;

extern uint64_t x_domapwk_sec;
extern uint64_t x_domapwk_usec;

extern uint64_t xdmw_total_sec;
extern uint64_t xdmw_total_usec;

extern uint64_t xdmw_genDF_sec;
extern uint64_t xdmw_genDF_usec;

extern uint64_t xdmw_scrmbl_sec;
extern uint64_t xdmw_scrmbl_usec;

extern uint64_t xdmw_total_sec;
extern uint64_t xdmw_total_usec;

extern uint64_t xdmw_cnvEnc_sec;
extern uint64_t xdmw_cnvEnc_usec;

extern uint64_t xdmw_punct_sec;
extern uint64_t xdmw_punct_usec;

extern uint64_t xdmw_intlv_sec;
extern uint64_t xdmw_intlv_usec;

extern uint64_t xdmw_symbls_sec;
extern uint64_t xdmw_symbls_usec;

extern uint64_t xdmw_mapout_sec;
extern uint64_t xdmw_mapout_usec;

extern uint64_t x_phdrgen_sec;
extern uint64_t x_phdrgen_usec;

extern uint64_t x_ck2sym_sec;
extern uint64_t x_ck2sym_usec;

extern uint64_t x_ocaralloc_sec;
extern uint64_t x_ocaralloc_usec;

extern uint64_t x_fft_sec;
extern uint64_t x_fft_usec;

extern uint64_t x_ocycpref_sec;
extern uint64_t x_ocycpref_usec;

#ifdef XMIT_HW_FFT
extern uint64_t x_fHtotal_sec;
extern uint64_t x_fHtotal_usec;

extern uint64_t x_fHcvtin_sec;
extern uint64_t x_fHcvtin_usec;

extern uint64_t x_fHcomp_sec;
extern uint64_t x_fHcomp_usec;

extern uint64_t x_fHcvtout_sec;
extern uint64_t x_fHcvtout_usec;
#endif

#endif

void xmit_pipe_init();

void do_xmit_pipeline(int in_msg_len, char* in_msg, size_t in_msg_sz, 
                      int* num_final_outs, size_t num_final_outs_sz,
                      float* final_out_real, size_t final_out_real_sz,
                      float* final_out_imag, size_t final_out_imag_sz,
                      int* psdu_len /*local*/, size_t psdu_len_sz /*=1*/,
                      uint8_t* pckt_hdr_out, size_t pckt_hdr_out_sz /*=64 -> though 48 may work*/, 
                      int* pckt_hdr_len /*local*/, size_t pckt_hdr_len_sz /*=1*/,
                      float* msg_stream_real /*local*/, size_t msg_stream_real_sz /*= MAX_SIZE*/,
                      float* msg_stream_imag /*local*/, size_t msg_stream_imag_sz /*= MAX_SIZE*/,
                      float* ofdm_car_str_real /*local*/, size_t ofdm_car_str_real_sz /*= ofdm_max_out_size*/,
                      float* ofdm_car_str_imag /*local*/, size_t ofdm_car_str_imag_sz /*= ofdm_max_out_size*/,
                      int* ofc_res /*local*/, size_t ofc_res_sz /*=1*/,
                      float* fft_out_real /*local*/, size_t fft_out_real_sz /*= ofdm_max_out_size*/,
                      float* fft_out_imag /*local*/, size_t fft_out_imag_sz /*= ofdm_max_out_size*/,
                      float* cycpref_out_real, size_t cycpref_out_real_sz /*= 41360*/,
                      float* cycpref_out_imag, size_t cycpref_out_imag_sz /*= 41360*/);



void generate_mac_data_frame(const char *msdu, int msdu_size, /*char **psdu,*/ int *psdu_size /*, char seq*/);

void scramble(const char *input, char *out, frame_param *frame, char initial_state);

void reset_tail_bits(char *scrambled_data, frame_param *frame);

void convolutional_encoding(const char *input, char *out, int n_data_bits); //frame_param *frame);

void puncturing(const char *input, char *out, frame_param *frame, ofdm_param *ofdm);

void interleave(const char *input, char *out, int n_sym, /*frame_param *frame,*/ int n_cbps, int n_bpsc, /*ofdm_param *ofdm,*/ bool reverse);

void split_symbols(const char *input, char *out, frame_param *frame, ofdm_param *ofdm);

void generate_bits(const char *psdu, char *data_bits, frame_param *frame);


#endif

