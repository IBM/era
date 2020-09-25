#ifndef INCLUDED_VITERBI_FLAT_H
#define INCLUDED_VITERBI_FLAT_H

/*
 * Copyright (C) 2016 Bastian Bloessl <bloessl@ccs-labs.org>
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

// Maximum number of traceback bytes
#define TRACEBACK_MAX 24

/* This Viterbi decoder was taken from the gr-dvbt module of
 * GNU Radio. It is a version of the Viterbi Decoder
 * created by Phil Karn. For more info see: gr-dvbt/lib/d_viterbi.h
 */

void reset();
uint8_t* depuncture(uint8_t *in);


/* This Viterbi decoder was taken from the gr-dvbt module of
 * GNU Radio. It is an SSE2 version of the Viterbi Decoder
 * created by Phil Karn. The SSE2 version was made by Bogdan
 * Diaconescu. For more info see: gr-dvbt/lib/d_viterbi.h
 */
void decode(ofdm_param *ofdm, frame_param *frame, uint8_t *in, int* n_dec_char, uint8_t* output);

typedef union branchtab27_u {
  unsigned char c[32];
} t_branchtab27;

extern t_branchtab27 d_branchtab27_generic[2];
extern unsigned char d_metric0_generic[64] __attribute__ ((aligned(16)));
extern unsigned char d_metric1_generic[64] __attribute__ ((aligned(16)));
extern unsigned char d_path0_generic[64] __attribute__ ((aligned(16)));
extern unsigned char d_path1_generic[64] __attribute__ ((aligned(16)));

void reset();
void viterbi_chunks_init_generic();

#ifdef USE_ESP_INTERFACE
void viterbi_butterfly2_generic(unsigned char *inMemory);
#else
void viterbi_butterfly2_generic(unsigned char *symbols,
		unsigned char m0[], unsigned char m1[], unsigned char p0[],
		unsigned char p1[]);
#endif

unsigned char viterbi_get_output_generic(unsigned char *mm0,
		unsigned char *pp0, int ntraceback, unsigned char *outbuf);



#ifdef INT_TIME
extern uint64_t dodec_sec;
extern uint64_t dodec_usec;

extern uint64_t depunc_sec;
extern uint64_t depunc_usec;
#endif


#endif
