#include <complex.h>
#include <math.h>
#include <stdio.h>

#include "debug.h"
#include "sync_short.h"

#define MIN_PLATEAU   2

// typedef ap_fixed<32,12> fx_pt1_ext1;
// typedef ap_fixed<64,32> fx_pt1_ext2;
typedef float fx_pt1_ext1;
typedef float fx_pt1_ext2;

void sync_short( unsigned num_inputs, fx_pt input_sample[SYNC_S_MAX_IN_SIZE], fx_pt input_abs[SYNC_S_MAX_ABS_SIZE], fx_pt1 correlation[SYNC_S_MAX_COR_SIZE], float* freq_offset_out, unsigned* num_outputs, fx_pt* output)
{
  unsigned c_plateau = 0;
  bool frame = false;
  unsigned frame_start = 0;
  fx_pt1_ext1 d_freq_offset = 0;
  DEBUG(printf("In sync_short with %u inputs\n", num_inputs));
  //if (frame == false) {
  for (unsigned i = 0; i < num_inputs /*SYNC_S_MAX_COR_SIZE*/; i++) {
    DEBUG2(printf("S_S_IN %5u : IN %12.8f %12.8f ABS %12.8f %12.8f CORR %12.8f : CP %u\n", i, crealf(input_sample[i]), cimagf(input_sample[i]), crealf(input_abs[i]), cimagf(input_abs[i]), correlation[i], c_plateau));
    if ( correlation[i] > (fx_pt1)0.56 ) { // 0.56 == d_threshold == "sensitivity" parameter
      if (c_plateau < MIN_PLATEAU) {
	c_plateau++;
      } else {
	// We found the start-point of a frame
	fx_pt1_ext1 x = (fx_pt1_ext1) crealf(input_abs[i]);
	fx_pt1_ext1 y = (fx_pt1_ext1) cimagf(input_abs[i]);
	fx_pt1_ext1 raz = y/x ;
	d_freq_offset = atan( raz )/16;  // d_freq_offset
	*freq_offset_out = d_freq_offset;
	frame_start = i;
	frame=true;
	break;
      }
    } else {
      c_plateau = 0;
    }
  } //frame for every sample
  // } // end of if (frame)
  
  if (frame) {
    DEBUG(printf(" S_S : frame_start: %d\n", frame_start));
    //static fx_pt1_ext1 pos = 1;
    //static fx_pt1_ext2 pos_ext = 1.0;

    DEBUG(printf(" S_S : d_freq_offset = %12.8f\n",d_freq_offset));
    /* c_freq_corr: */
    unsigned out_idx = 0;
    for (unsigned k = frame_start; (k < num_inputs /*SYNC_S_MAX_ABS_SIZE*/) && (out_idx < MAX_SAMPLES); k++) {
      fx_pt1_ext2 mult = -(fx_pt1_ext2)d_freq_offset * k; // pos_ext;
      
      fx_pt_ext esp = (fx_pt_ext)(cos((fx_pt1_ext1)mult) + sin((fx_pt1_ext1)mult) * I);
      DEBUG2(printf("  -d_freq_offset * k = %12.8f * %5u = %12.8f :  %12.8f %12.8f\n", d_freq_offset, k, mult, esp.real(), esp.imag() ));
      //DEBUG2(printf("  -d_freq_offset * pos_ext = %12.8f * %12.8f = %12.8f :  %12.8f %12.8f\n", d_freq_offset, pos_ext, mult, esp.real(), esp.imag() ));
      fx_pt_ext prod= (fx_pt_ext)(input_sample[k] *  esp);
      DEBUG2(printf("  input: %12.8f %12.8f    prod : %12.8f %12.8f\n ", input_sample[k].real(), input_sample[k].imag(), prod.real(), prod.imag() ));
      output[out_idx] = (fx_pt)prod;
      DEBUG2(printf("  Set ss_output[%u] = %12.8f  %12.8f\n", out_idx, prod.real(), prod.imag()));
      out_idx++;
      //pos++;
      //pos_ext++;
    } // end-for

    *num_outputs = out_idx;
    DEBUG(printf(" synch_short set num_outputs to %u\n", *num_outputs));
    // if (out_idx >= MAX_SAMPLES) {
    //   frame = false;
    //   DEBUG(printf(" came to the MAX_SAMPLES (%u)...\n", MAX_SAMPLES));
    // }
  } else { // if (frame)
    printf("ERROR: Couldn't find the start_Frame in sync_short... never crossed threshold?\n");
    exit(-9);
  } // if (frame)
}
