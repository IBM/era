#include <complex.h>
#include <math.h>
#include <stdio.h>

#include "debug.h"
#include "complex_ops.h"
#include "fir.h"
#include "sync_long.h"

// typedef ap_fixed<32,15> fx_pt1_ext1;
// typedef ap_fixed<64,13> fx_pt1_ext2;
typedef float fx_pt1_ext1;
typedef float fx_pt1_ext2;
typedef float complex fx_pt_ext1;
typedef float complex fx_pt_ext2;

/*********************************************************************************
 * I'm going ot add some comments about how I think this works, but I am no 
 * expert and find this all a bit confusion as I look through it without any
 * real context, etc.  
 * I think we take in the input, and once we know we have enough input that the 
 * frame_start must be within the first <SYNC_LENGTH> window samples, we search 
 * those samples to find the (most-likely) start of the frame (using correlations).
 * This process corresponds to the SYNC case/part of the original GR code, I think.
 * One we have the frame start, then we know how the data is/should be laid out in 
 * the rest of the samples, and we "extract" (COPY out) the frames...
 **********************************************************************************/
void sync_long( unsigned num_inputs, fx_pt* input, fx_pt* input_delayed, float* freq_offset_out, unsigned* num_outputs, fx_pt* output )
{
  DEBUG(printf("In synch_long with %u inputs\n", num_inputs));
  /**
  const fx_pt_ext coeff[COEFF_LENGTH] = { fx_pt_ext( fx_pt1_ext(-0.0455),fx_pt1_ext(-1.0679) ),
					  fx_pt_ext( fx_pt1_ext(0.3528), fx_pt1_ext(-0.9865) ),
					  fx_pt_ext( fx_pt1_ext(0.8594), fx_pt1_ext(0.7348) ),
					  fx_pt_ext( fx_pt1_ext(0.1874), fx_pt1_ext(0.2475) ),
					  fx_pt_ext( fx_pt1_ext(0.5309), fx_pt1_ext(-0.7784) ),
					  fx_pt_ext( fx_pt1_ext(-1.0218),fx_pt1_ext(-0.4897) ),
					  fx_pt_ext( fx_pt1_ext(-0.3401),fx_pt1_ext(-0.9423) ),
					  fx_pt_ext( fx_pt1_ext(0.8657), fx_pt1_ext(-0.2298) ),
					  fx_pt_ext( fx_pt1_ext(0.4734), fx_pt1_ext(0.0362) ),
					  fx_pt_ext( fx_pt1_ext(0.0088), fx_pt1_ext(-1.0207) ),
					  fx_pt_ext( fx_pt1_ext(-1.2142),fx_pt1_ext(-0.4205) ),
					  fx_pt_ext( fx_pt1_ext(0.2172), fx_pt1_ext(-0.5195) ),
					  fx_pt_ext( fx_pt1_ext(0.5207), fx_pt1_ext(-0.1326) ),
					  fx_pt_ext( fx_pt1_ext(-0.1995),fx_pt1_ext(1.4259) ),
					  fx_pt_ext( fx_pt1_ext(1.0583), fx_pt1_ext(-0.0363) ),
					  fx_pt_ext( fx_pt1_ext(0.5547), fx_pt1_ext(-0.5547) ),
					  fx_pt_ext( fx_pt1_ext(0.3277), fx_pt1_ext(0.8728) ),
					  fx_pt_ext( fx_pt1_ext(-0.5077),fx_pt1_ext(0.3488) ),
					  fx_pt_ext( fx_pt1_ext(-1.165), fx_pt1_ext(0.5789) ),
					  fx_pt_ext( fx_pt1_ext(0.7297), fx_pt1_ext(0.8197) ),
					  fx_pt_ext( fx_pt1_ext(0.6173), fx_pt1_ext(0.1253) ),
					  fx_pt_ext( fx_pt1_ext(-0.5353),fx_pt1_ext(0.7214) ),
					  fx_pt_ext( fx_pt1_ext(-0.5011),fx_pt1_ext(-0.1935) ),
					  fx_pt_ext( fx_pt1_ext(-0.311), fx_pt1_ext(-1.3392) ),
					  fx_pt_ext( fx_pt1_ext(-1.0818),fx_pt1_ext(-0.147) ),
					  fx_pt_ext( fx_pt1_ext(-1.13),  fx_pt1_ext(-0.182) ),
					  fx_pt_ext( fx_pt1_ext(0.6663), fx_pt1_ext(-0.6571) ),
					  fx_pt_ext( fx_pt1_ext(-0.0249),fx_pt1_ext(0.4773) ),
					  fx_pt_ext( fx_pt1_ext(-0.8155),fx_pt1_ext(1.0218) ),
					  fx_pt_ext( fx_pt1_ext(0.814),  fx_pt1_ext(0.9396) ),
					  fx_pt_ext( fx_pt1_ext(0.109),  fx_pt1_ext(0.8662) ),
					  fx_pt_ext( fx_pt1_ext(-1.3868),fx_pt1_ext(0) ),
					  fx_pt_ext( fx_pt1_ext(0.109),  fx_pt1_ext(-0.8662) ),
					  fx_pt_ext( fx_pt1_ext(0.814),  fx_pt1_ext(-0.9396) ),
					  fx_pt_ext( fx_pt1_ext(-0.8155),fx_pt1_ext(-1.0218) ),
					  fx_pt_ext( fx_pt1_ext(-0.0249),fx_pt1_ext(-0.4773) ),
					  fx_pt_ext( fx_pt1_ext(0.6663), fx_pt1_ext(0.6571) ),
					  fx_pt_ext( fx_pt1_ext(-1.13),  fx_pt1_ext(0.182) ),
					  fx_pt_ext( fx_pt1_ext(-1.0818),fx_pt1_ext(0.147) ),
					  fx_pt_ext( fx_pt1_ext(-0.311), fx_pt1_ext(1.3392) ),
					  fx_pt_ext( fx_pt1_ext(-0.5011),fx_pt1_ext(0.1935) ),
					  fx_pt_ext( fx_pt1_ext(-0.5353),fx_pt1_ext(-0.7214) ),
					  fx_pt_ext( fx_pt1_ext(0.6173), fx_pt1_ext(-0.1253) ),
					  fx_pt_ext( fx_pt1_ext(0.7297), fx_pt1_ext(-0.8197) ),
					  fx_pt_ext( fx_pt1_ext(-1.165), fx_pt1_ext(-0.5789) ),
					  fx_pt_ext( fx_pt1_ext(-0.5077),fx_pt1_ext(-0.3488) ),
					  fx_pt_ext( fx_pt1_ext(0.3277), fx_pt1_ext(-0.8728) ),
					  fx_pt_ext( fx_pt1_ext(0.5547), fx_pt1_ext(0.5547) ),
					  fx_pt_ext( fx_pt1_ext(1.0583), fx_pt1_ext(0.0363) ),
					  fx_pt_ext( fx_pt1_ext(-0.1995),fx_pt1_ext(-1.4259) ),
					  fx_pt_ext( fx_pt1_ext(0.5207), fx_pt1_ext(0.1326) ),
					  fx_pt_ext( fx_pt1_ext(0.2172), fx_pt1_ext(0.5195) ),
					  fx_pt_ext( fx_pt1_ext(-1.2142),fx_pt1_ext(0.4205) ),
					  fx_pt_ext( fx_pt1_ext(0.0088), fx_pt1_ext(1.0207) ),
					  fx_pt_ext( fx_pt1_ext(0.4734), fx_pt1_ext(-0.0362) ),
					  fx_pt_ext( fx_pt1_ext(0.8657), fx_pt1_ext(0.2298) ),
					  fx_pt_ext( fx_pt1_ext(-0.3401),fx_pt1_ext(0.9423) ),
					  fx_pt_ext( fx_pt1_ext(-1.0218),fx_pt1_ext(0.4897) ),
					  fx_pt_ext( fx_pt1_ext(0.5309), fx_pt1_ext(0.7784) ),
					  fx_pt_ext( fx_pt1_ext(0.1874), fx_pt1_ext(-0.2475) ),
					  fx_pt_ext( fx_pt1_ext(0.8594), fx_pt1_ext(-0.7348) ),
					  fx_pt_ext( fx_pt1_ext(0.3528), fx_pt1_ext(0.9865) ),
					  fx_pt_ext( fx_pt1_ext(-0.0455),fx_pt1_ext(1.0679) ),
					  fx_pt_ext( fx_pt1_ext(1.3868), fx_pt1_ext(0) ) }; **/

  const float complex rev_coeff[COEFF_LENGTH] = {  1.3868  + 0 * I ,
						   -0.0455 + 1.0679 * I, 
						   0.3528  + 0.9865 * I, 
						   0.8594  - 0.7348 * I, 
						   0.1874  - 0.2475 * I, 
						   0.5309  + 0.7784 * I, 
						   -1.0218 + 0.4897 * I, 
						   -0.3401 + 0.9423 * I, 
						   0.8657  + 0.2298 * I, 
						   0.4734  - 0.0362 * I, 
						   0.0088  + 1.0207 * I, 
						   -1.2142 + 0.4205 * I, 
						   0.2172  + 0.5195 * I, 
						   0.5207  + 0.1326 * I, 
						   -0.1995 - 1.4259 * I, 
						   1.0583  + 0.0363 * I, 
						   0.5547  + 0.5547 * I, 
						   0.3277  - 0.8728 * I, 
						   -0.5077 - 0.3488 * I, 
						   -1.165  - 0.5789 * I, 
						   0.7297  - 0.8197 * I, 
						   0.6173  - 0.1253 * I, 
						   -0.5353 - 0.7214 * I, 
						   -0.5011 + 0.1935 * I, 
						   -0.311  + 1.3392 * I, 
						   -1.0818 + 0.147 * I, 
						   -1.13   + 0.182 * I, 
						   0.6663  + 0.6571 * I, 
						   -0.0249 - 0.4773 * I, 
						   -0.8155 - 1.0218 * I, 
						   0.814   - 0.9396 * I, 
						   0.109   - 0.8662 * I, 
						   -1.3868 + 0 * I, 
						   0.109   + 0.8662 * I, 
						   0.814   + 0.9396 * I, 
						   -0.8155 + 1.0218 * I, 
						   -0.0249 + 0.4773 * I, 
						   0.6663  - 0.6571 * I, 
						   -1.13   - 0.182 * I, 
						   -1.0818 - 0.147 * I, 
						   -0.311  - 1.3392 * I, 
						   -0.5011 - 0.1935 * I, 
						   -0.5353 + 0.7214 * I, 
						   0.6173  + 0.1253 * I, 
						   0.7297  + 0.8197 * I, 
						   -1.165  + 0.5789 * I, 
						   -0.5077 + 0.3488 * I, 
						   0.3277  + 0.8728 * I, 
						   0.5547  - 0.5547 * I, 
						   1.0583  - 0.0363 * I, 
						   -0.1995 + 1.4259 * I, 
						   0.5207  - 0.1326 * I, 
						   0.2172  - 0.5195 * I, 
						   -1.2142 - 0.4205 * I, 
						   0.0088  - 1.0207 * I, 
						   0.4734  + 0.0362 * I, 
						   0.8657  - 0.2298 * I, 
						   -0.3401 - 0.9423 * I, 
						   -1.0218 - 0.4897 * I, 
						   0.5309  - 0.7784 * I, 
						   0.1874  + 0.2475 * I, 
						   0.8594  + 0.7348 * I, 
						   0.3528  - 0.9865 * I, 
						   -0.0455 - 1.0679 * I };


  unsigned d_frame_start = 0;

  fx_pt toBfiltered[SYNC_LENGTH + COEFF_LENGTH]; // Extra space required for the firG routine
  fx_pt filtered[SYNC_LENGTH];  // ensure it is big enough

  fx_pt1_ext1 d_freq_offset = 0;

  DEBUG(printf("\nIn sync_long\n");
	for (unsigned i = 0; i < 500; i++) {
	  printf(" S_L_IN %5u : IN %12.8f %12.8f : IN_D %12.8f %12.8f\n", i, crealf(input[i]), cimagf(input[i]), crealf(input_delayed[i]), cimagf(input_delayed[i]));
	});

  if ( num_inputs < SYNC_LENGTH) {
    printf("ERROR : num_inputs = %u which is less than SYNC_LENGTH %u\n", num_inputs, SYNC_LENGTH);
    exit(-6);
  }
    
  // For some reason we copy the original inputs into this array to do the firG
  DEBUG(printf("\n Setting up toBfiltered data\n"));
  for (unsigned i = 0; i < SYNC_LENGTH; i++) {
    toBfiltered[i] = input[i];
  }
  for (unsigned i = SYNC_LENGTH; i < SYNC_LENGTH + COEFF_LENGTH; i++) {
    toBfiltered[i] = 0 + 0 * I;
  }

  // If we have more than the "SYNC_LENGTH" (320 in this case) samples, we can search for the start of the frames (must be in there)
  DEBUG(printf(" calling firG_using_cmp_volk with num_inputs = %u\n", num_inputs));
  firG_using_cmp_volk( filtered, toBfiltered, rev_coeff);
  DEBUG(for (int ti = 0; ti < SYNC_LENGTH; ti++) {
      printf(" CMP_VOLK_FIRG_FIN %2u : IN_2BFILT %12.8f + %12.8f i : CORR_FILT %12.8f %12.8f\n", ti, crealf(toBfiltered[ti]), cimagf(toBfiltered[ti]), crealf(filtered[ti]), cimagf(filtered[ti]));
    });

  // This code corresponds to the search_frame_start method
  //  We search over the (first 320) samples to find the best fit
  fx_pt first_pick  = filtered[0];
  fx_pt second_pick = filtered[0];
  unsigned idx1 = 0;
  unsigned idx2 = 0;
  // SET BUT NOT USED?  fx_pt1 pick = 0;

  /* find_pick: */
  // In analyzing this code, it looks like this does a 2-deep sort of the highest correlations
  //  At the end, the highest correlation is in first_pick and the second-highest is in second_pick
  //  Then we assume those give the proper d_frame_start (and are 64 apart)
  DEBUG(printf(" Starting the search_d_frame_start...  SYNC_LENGTH = %u\n", SYNC_LENGTH));
  for (unsigned i = 1; i < SYNC_LENGTH; i++) { // Scan the first SYNCH_LENGTH entries
    fx_pt1_ext comp1 = (fx_pt1_ext)abs_c(first_pick)  - (fx_pt1_ext)abs_c(filtered[i]);  // compare the correlations
    fx_pt1_ext comp2 = (fx_pt1_ext)abs_c(second_pick) - (fx_pt1_ext)abs_c(filtered[i] ); // compare the correlations
    DEBUG(printf(" S_L : Corr_Sort %5u : f_p %5u %12.8f %12.8f : s_p %5u %12.8f %12.8f : flt  %5u %12.8f %12.8f : C1 %12.8f C2 %12.8f\n", i, idx1, crealf(first_pick), cimagf(first_pick), idx2, crealf(second_pick), cimagf(second_pick), i, crealf(filtered[i]), cimagf(filtered[i]), comp1, comp2));
    if (comp1 < 0) {  // if Cor(i) > Cor(fp)
      second_pick = first_pick;   // demote firt-pick to second-pick
      first_pick  = filtered[i];  // set first-pick as the higher correlation
      idx2 = idx1;                // save the index of the former first-pick as second-pick
      idx1 = i;                   // and set hte new first-pick index
      // SET BUT NOT USED?  pick = abs_c(first_pick);
    } else if (comp2 < 0) { // if (Cor(i) < Cor(fp) but > Cor(sp)
      second_pick=filtered[i];    // Update second-best pick to the higher correlation
      idx2 = i;		    // and save the index of the second-best pick
    }
  }

  fx_pt first  = 0 + 0 * I;
  fx_pt second = 0 + 0 * I;
  if (idx1 > idx2) { // if the best-pick index > scond-best pick index
    first = filtered[idx2];  // make the second-best pick the first choice
    second = filtered[idx1]; //  and the first-best pick the second choice
    d_frame_start = idx2;
  } else {
    first = filtered[idx1];
    second = filtered[idx2];
    d_frame_start = idx1;
  }
  DEBUG(printf("\n S_L : Sort_Done : idx1 %u idx2 %u d_frame_start %u : first %12.8f %12.8f : second %12.8f %12.8f\n", idx1, idx2, d_frame_start, crealf(first), cimagf(first), crealf(second), cimagf(second)));

  // This code assumes now that first corresponds to the d_frame_start
  fx_pt_ext1 arg  = (fx_pt_ext1)(first) * conjf(second);
  fx_pt1_ext1 x   = (fx_pt1_ext1) crealf(arg);
  fx_pt1_ext1 y   = (fx_pt1_ext1) cimagf(arg);
  fx_pt1_ext1 raz = y/x ;

  d_freq_offset = atan( raz  )/64;
  *freq_offset_out = d_freq_offset;
    
  // // This assumes that the d_frame_start is always 64 before the second-pick index
  // if (idx1 > idx2) {
  //   d_frame_start = idx1 + 64 - 128;
  // } else {
  //   d_frame_start = idx2 + 64 - 128;
  // }

  //////////////////////////////////////////////////////////////////////////////////
  // Now we have a frame start indication, so we determine/generate the outputs
  unsigned d_offset_ui = 0;//168
  unsigned o = d_frame_start;

  DEBUG(printf("  d_frame_start = %d   d_offset_ui = %d   dfreq_offset = %f\n", d_frame_start, d_offset_ui, d_freq_offset));

  unsigned out_idx = 0;
  /* freq_correct: */
  for (unsigned i = 0; i < num_inputs /*SYNC_S_MAX_OUT_SIZE*/; i++) {
    int rel = d_offset_ui - (int)d_frame_start; // the relative index
    d_offset_ui++;
    // This appears to state we should take the first 128 samples, then repeatedly skip 16 and take the next (80-16 = 64)
    if( (rel >= 0) && ((rel < 128) || (((rel - 128) % 80) > 15)) ) {
      fx_pt_ext2 esp = (fx_pt_ext2)( cosf(d_freq_offset*d_offset_ui) + I * sinf(d_freq_offset*d_offset_ui));
      fx_pt_ext2 num = (fx_pt_ext2)(input[i]);
      fx_pt_ext2 dsampl= esp*num;
      output[out_idx] = (fx_pt)dsampl;
      DEBUG(printf("     output[%5u : %5u : %6d : %4d ] = %12.8f %12.8f\n", out_idx, i, rel, ((rel-128)%80), crealf(output[out_idx]), cimagf(output[out_idx])));
      out_idx++;
    } // end if (rel >= 0 && ...
    DEBUG(else { printf("  no_output[ %4s : %5u : %6d : %4d ]\n", "-X- ", i, rel, ((rel-128)%80) ); });
    o++;
  } // for ( i = 0 .. samples )
  *num_outputs = out_idx;
  DEBUG(printf(" synch_long set num_outputs to %u\n", *num_outputs));
}

