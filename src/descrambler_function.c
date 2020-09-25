/*
 * Descrambler_function.c
 *
 *  Created on: Jul 11, 2019
 *      Author: Varun Mannam
 *      function: input from Viterbi decoder, gets output of descrambled bytes for
 *      the size of (psdu_size +2) and compares with the reference descrambler data
 *      Input: data bits from Viterbi decode, (psdu_szie) , reference descrambler bytes
 *      Output: nothing
 *      Source: https://github.com/IBM/dsrc/blob/master/gr-ieee802-11/lib/decode_mac.cc
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "debug.h"
#include "sdr_base.h"



typedef unsigned char   uint8_t;

void sdr_descrambler(uint8_t* in, int psdusize, char* out_msg) //definition
{
  uint32_t output_length = (psdusize)+2; //output is 2 more bytes than psdu_size
  uint32_t msg_length = (psdusize)-28;
  uint8_t out[MAX_PSDU_SIZE + 2]; // output_length];
  int state = 0; //start
  DEBUG(printf("\nIn the descrambler with psdu_size = %u\n", psdusize));
  // find the initial state of LFSR (linear feedback shift register: 7 bits) from first 7 input bits
  for(int i = 0; i < 7; i++) {
    if( *(in+i) ) {
      state |= 1 << (6 - i);
    }
  }
  //init o/p array to zeros
  for (uint32_t i = 0; i < output_length; i++ ) {
    out[i] = 0;
  }

  out[0] = state; //initial value
  int feedback;
  int bit;
  int index = 0;
  int mod = 0;
  for(int i = 7; i < (psdusize*8)+16; i++) { // 2 bytes more than psdu_size -> convert to bits
    feedback = ((!!(state & 64))) ^ (!!(state & 8));
    bit = feedback ^ (*(in+i) & 0x1);
    index = i/8;
    mod =  i%8;
    int comp1, comp2, val;//, comp3;
    comp1 = (bit << mod);
    val = out[index];
    comp2 = val | comp1;
    out[index] =  comp2;
    //comp3 = out[index];
    state = ((state << 1) & 0x7e) | feedback;
  }

  
  for (uint32_t i = 0; i < msg_length; i++) {
    out_msg[i] = out[i+26];
    DEBUG(printf("out_msg %4u = `%c` = 0x%02x from out %4u\n", i, out_msg[i], (out_msg[i] & 0xff), i+26));
  }
  DEBUG(printf("\n"));
  out_msg[msg_length] = '\0';

  DEBUG(printf("  Done: message length = %u  MSG: (hex 'x' = %02x)\n", msg_length, 'x');
	for (uint32_t i = 0; i< msg_length; i++) {
	  printf("%02x ", (out_msg[i]&0xff));
	}
	printf("\n"));
}
