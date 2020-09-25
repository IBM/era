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

#define TIME

void xmit_pipe_init();

void do_xmit_pipeline(int in_msg_len, char * in_msg, int* n_out, float* out_real, float* out_imag);



void generate_mac_data_frame(const char *msdu, int msdu_size, /*char **psdu,*/ int *psdu_size /*, char seq*/);

void scramble(const char *input, char *out, frame_param *frame, char initial_state);

void reset_tail_bits(char *scrambled_data, frame_param *frame);

void convolutional_encoding(const char *input, char *out, int n_data_bits); //frame_param *frame);

void puncturing(const char *input, char *out, frame_param *frame, ofdm_param *ofdm);

void interleave(const char *input, char *out, int n_sym, /*frame_param *frame,*/ int n_cbps, int n_bpsc, /*ofdm_param *ofdm,*/ bool reverse);

void split_symbols(const char *input, char *out, frame_param *frame, ofdm_param *ofdm);

void generate_bits(const char *psdu, char *data_bits, frame_param *frame);


#endif
