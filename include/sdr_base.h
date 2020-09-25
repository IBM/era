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

#ifndef INCLUDED_BASE_H
#define INCLUDED_BASE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif 

#define MAX_PAYLOAD_SIZE    1500
#define MAX_PSDU_SIZE       (MAX_PAYLOAD_SIZE + 28) // MAC, CRC
#define MAX_SYM             (((16 + 8 * MAX_PSDU_SIZE + 6) / 24) + 1)
#define MAX_ENCODED_BITS    ((16 + 8 * MAX_PSDU_SIZE + 6) * 2 + 288)


#define MAX_SAMPLES    540 * 80  // this is 540 symbols * 80 bytes per symbol

/* Types definitions */

enum Encoding {
         BPSK_1_2  = 0,
         BPSK_3_4  = 1,
         QPSK_1_2  = 2,
         QPSK_3_4  = 3,
         QAM16_1_2 = 4,
         QAM16_3_4 = 5,
         QAM64_2_3 = 6,
         QAM64_3_4 = 7,
};

typedef struct mac_header_struct {
	//protocol version, type, subtype, to_ds, from_ds, ...
	uint16_t frame_control;
	uint16_t duration;
	uint8_t addr1[6];
	uint8_t addr2[6];
	uint8_t addr3[6];
	uint16_t seq_nr;
} mac_header; // __attribute__((packed));

/**
 * WIFI parameters
 */
typedef struct {
	//ofdm_param(Encoding e); use init_ofdm_param instead

	// data rate
	enum Encoding encoding;
	// rate field of the SIGNAL header
	char     rate_field;
	// number of coded bits per sub carrier
	int      n_bpsc;
	// number of coded bits per OFDM symbol
	int      n_cbps;
	// number of data bits per OFDM symbol
	int      n_dbps;

	//void print(ofdm_param* ofdm_param);
} ofdm_param;

/**
 * packet specific parameters
 */
typedef struct {
	//frame_param(ofdm_param &ofdm, int psdu_length); use init_frame_param isntead
	// PSDU size in bytes
	int psdu_size;
	// number of OFDM symbols (17-11)
	int n_sym;
	// number of padding bits in the DATA field (17-13)
	int n_pad;
	int n_encoded_bits;
	// number of data bits, including service and padding (17-12)
	int n_data_bits;

	//void print();
} frame_param;


// The folowing are some "MAX SIZE" definitions (used to allocate memory arrays right now)

// complex_ops
#define CMP_CONJ_MAX_SIZE   41789
#define CMP2MAGSQ_MAX_SIZE  41772
#define CMP_MULT_MAX_SIZE   41772
#define CMP2MAG_MAX_SIZE    41772
#define DIVIDE_MAX_SIZE     41722

// delay
#define DELAY_16_MAX_IN_SIZE    41772
#define DELAY_16_MAX_OUT_SIZE   41788
#define DELAY_320_MAX_IN_SIZE   41236
#define DELAY_320_MAX_OUT_SIZE  41556

// fir
#define FIR_MAVG64_MAX_SIZE   41722
#define FIRC_MAVG48_MAX_SIZE  41722

// gr_equalizer
#define MAX_FFT_FRAMES         520
#define FRAME_EQ_IN_MAX_SIZE   MAX_FFT_FRAMES * 64 //32576
#define FRAME_EQ_MAX_PACKETS   FRAME_EQ_IN_MAX_SIZE / 64
#define FRAME_EQ_OUT_MAX_SIZE  FRAME_EQ_MAX_PACKETS * 48

// ofdm
#define OFDM_PAD_ENTRIES        80 // extra pad space used by decode (?)
#define DECODE_IN_SIZE_MAX  FRAME_EQ_OUT_MAX_SIZE // MAX_ENCODED_BITS

// sync_long
#define SYNC_L_OUT_MAX_SIZE 32620

// sync_short
#define SYNC_S_MAX_IN_SIZE    41788
#define SYNC_S_MAX_ABS_SIZE   41772
#define SYNC_S_MAX_COR_SIZE   41772
#define SYNC_S_OUT_MAX_SIZE   41236

#endif
