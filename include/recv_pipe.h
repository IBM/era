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
#include "ofdm.h"

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

#endif

void recv_pipe_init();

void do_recv_pipeline(int n_in, float* in_real, float* in_imag, int* out_msg_len, char* out_msg);

#endif
