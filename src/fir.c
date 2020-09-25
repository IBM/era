#include <stdio.h>
//#include <cstring>

#include "debug.h"
#include "sdr_type.h"
#include "sync_short.h" // Pick up some #defines
#include "fir.h"


void fir(fx_pt1 output[FIR_MAVG64_MAX_SIZE], fx_pt1 input_sample[FIR_MAVG64_MAX_SIZE], const fx_pt1 coefficient[COEFF_LENGTH])
{
  static fx_pt1_ext buffer[COEFF_LENGTH];
  static bool init = true;

  if (init) {
    /* init: */
    for(unsigned i = 0; i < COEFF_LENGTH; i++) {
      buffer[i] = 0;
    }
    init = false;
  }

  /* loop_filter_data: */
  for(unsigned n = 0; n < FIR_MAVG64_MAX_SIZE; n++) {
    //fx_pt1_ext t_output[COEFF_LENGTH];
    fx_pt1_ext t_output_t = 0;
    /* data_shift: */
    for(int i = COEFF_LENGTH-1; i >= 0; i--) {
      if (i == 0) {
	buffer[0]=(fx_pt1_ext) input_sample[n];
      } else {
	buffer[i]=buffer[i-1];
      }
    }
    /* Mply: */
    // for(int i=COEFF_LENGTH-1;i>=0;i--) {
    //   t_output[i]=buffer[i];/*(fx_pt1_ext)coefficient[i];*/
    // }

    /*ACC:*/
    for (int i = COEFF_LENGTH-1; i >= 0; i--) {
      //t_output_t +=_output[i];
      t_output_t +=buffer[i];
    }

    output[n]=(fx_pt1) t_output_t;
  } //end for every sample

} //end function



void firc(fx_pt output[FIRC_MAVG48_MAX_SIZE], fx_pt input_sample[FIRC_MAVG48_MAX_SIZE])
{
  static fx_pt_ext buffer[COMPLEX_COEFF_LENGTH];
  static bool init = true;

  if(init) {
    /* init: */
    for (unsigned i = 0; i < COMPLEX_COEFF_LENGTH; i++) {
      buffer[i] = 0 + 0 * I;
    }
    init = false;
  }

  /* loop_filter_data: */
  for (unsigned n = 0; n < FIRC_MAVG48_MAX_SIZE; n++) {
    //fx_pt_ext t_output[COMPLEX_COEFF_LENGTH];
    fx_pt_ext t_output_t = 0 + 0 * I;
    /* data_shift: */
    for(int i = COMPLEX_COEFF_LENGTH-1; i >= 0; i--) {
      if (i == 0) {
	buffer[0] = (fx_pt_ext)(input_sample[n]);
      } else {
	buffer[i] = buffer[i-1];
      }
    }
    
    for(int i = COMPLEX_COEFF_LENGTH-1; i >= 0; i--) {
      t_output_t += buffer[i];
    }

    output[n]= (fx_pt)(t_output_t);
    
  } //end for every sample
  
} //end function



/*********************************************************************************
 * According to the GnuRadio documentation (sparse as it is) the fir filter
 * (the one in the GR code is fir_filter_ccc and filterN is called)
 * performs the following simple convolution behavior:
 *********************************************************************************/

// This is defined in GnuRadio as the DotProduct of inputs and taps (i.e. input_sample and coefficient)
void convlv(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH], const fx_pt_ext coefficient[COEFF_LENGTH])
{
  for (int n = 0; n < (SYNC_LENGTH + COEFF_LENGTH - 1); n++) {
    int kmin = (n >= COEFF_LENGTH - 1) ? n - (COEFF_LENGTH - 1) : 0;
    int kmax = (n < SYNC_LENGTH - 1) ? n : SYNC_LENGTH - 1;
    output[n] = 0 + 0 * I;
    for (int k = kmin; k <= kmax; k++) {
      output[n] += input_sample[k] * coefficient[n - k];
      DEBUG(printf("    Calc_CONVLV %5u %2u %2u : IN %12.8f %12.8f : COEFF %12.8f %12.8f : SUM %12.8f %12.8f\n", n, k, n-k, crealf(input_sample[n]), cimagf(input_sample[n]), crealf(coefficient[n-k]), cimagf(coefficient[n-k]), creal(output[n]), cimagf(output[n])));
    }
    DEBUG(printf("  Out_CONVLV    %5u -- -- : IN %12.8f %12.8f :                                   OUT %12.8f %12.8f\n", n, crealf(input_sample[n]), cimagf(input_sample[n]), crealf(output[n]), cimagf(output[n])));
  }
}


/*********************************************************************************
 * 
 * So the below should be equivalent to this above...
 * (Note that some care is needed at the edges)
 *********************************************************************************/


void firG(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH + COEFF_LENGTH], const fx_pt_ext coefficient[COEFF_LENGTH])
{
  static fx_pt_ext buffer[COEFF_LENGTH];
  static bool init = true;

  if (init) {
    /* init: */
    for (unsigned i = 0; i < COEFF_LENGTH; i++) {
      buffer[i] = 0 + 0 * I;
    }
    init = false;
  }

  /* loop_filter_data: */
  for(unsigned n = 0; n < SYNC_LENGTH; n++) {
    fx_pt_ext t_output[COEFF_LENGTH];
    fx_pt_ext t_output_t = 0 + 0 * I;
    /* data_shift: */
    for (int i = COEFF_LENGTH-1; i >= 0; i--) {
      if (i==0) {
	buffer[0] = input_sample[n];
      } else {
	buffer[i] = buffer[i-1];
      }
    }
    /* Mply: */
    for (int i = COEFF_LENGTH-1; i >= 0; i--) {
      t_output[i] = buffer[i] * coefficient[i];
      DEBUG(if (n == 0) {
	  printf("FIRG_MULT %5u i %2u : BUF %15.8f %12.8f : COEFF %12.8f %12.8f : T_OUT %12.8f %12.8f\n", n, i, crealf(buffer[i]), cimagf(buffer[i]), crealf(coefficient[i]), cimagf(coefficient[i]), crealf(t_output[i]), cimagf(t_output[i]));
	});
    }
    /* ACC: */
    for (int i = COEFF_LENGTH-1; i >= 0; i--) {
      t_output_t += t_output[i];
      DEBUG(if (n == 0) {
	  printf("FIRG_ADD %5u i %2u : T_OUT %15.8f %12.8f : SUM %12.8f %12.8f\n", n, i, crealf(t_output[i]), cimagf(t_output[i]), crealf(t_output_t), cimagf(t_output_t));
	});
    }
    output[n] = (fx_pt)t_output_t;
      DEBUG(printf("FIRG_OUT %5u : IN %15.8f %12.8f : OUT %12.8f %12.8f\n", n, crealf(input_sample[n]), cimagf(input_sample[n]), crealf(output[n]), cimagf(output[n])));
  } //end for every sample

} //end function


// The following comes from the VOLK library
//static inline void volk_32fc_x2_dot_prod_32fc_generic(lv_32fc_t* result, const lv_32fc_t* input, const lv_32fc_t* taps, unsigned int num_points) {
void my_volk_32fc_x2_dot_prod_32fc_generic(float* result, const float* input, const float* taps) //, unsigned int num_points)
{
  float * res = (float*) result;
  float * in = (float*) input;
  float * tp = (float*) taps;
  unsigned int n_2_ccomplex_blocks = (COEFF_LENGTH/2); //num_points/2;
  // unsigned int isodd = COEFF_LENGTH & 1;

  float sum0[2] = {0,0};
  float sum1[2] = {0,0};
  unsigned int i = 0;

  for(i = 0; i < n_2_ccomplex_blocks; ++i) {
    sum0[0] += in[0] * tp[0] - in[1] * tp[1];
    sum0[1] += in[0] * tp[1] + in[1] * tp[0];
    sum1[0] += in[2] * tp[2] - in[3] * tp[3];
    sum1[1] += in[2] * tp[3] + in[3] * tp[2];
    DEBUG(if (i == 0) {
	printf("MY_VOLK_KERNEL %5u : 0 : IN %12.8f %12.8f : TAP %12.8f %12.8f : SUM0 %12.8f %12.8f\n", i, in[0], in[1], tp[0], tp[1], sum0[0], sum0[1]);
	printf("               %5u : 1 : IN %12.8f %12.8f : TAP %12.8f %12.8f : SUM1 %12.8f %12.8f\n", i, in[2], in[3], tp[2], tp[3], sum1[0], sum1[1]);
      });
    in += 4;
    tp += 4;
  }

  res[0] = sum0[0] + sum1[0];
  res[1] = sum0[1] + sum1[1];

  // // Cleanup if we had an odd number of points
  // for(i = 0; i < isodd; ++i) {
  //   *result += input[COEFF_LENGTH - 1] * taps[COEFF_LENGTH - 1];
  // }
}

void
firG_using_my_volk(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH+COEFF_LENGTH], const float taps_coeff[2*COEFF_LENGTH])
{
  for(unsigned long i = 0; i < SYNC_LENGTH; i++) {
    float t_out[2];
    float t_in[2*(COEFF_LENGTH+1)];
    for (int ti = 0; ti < COEFF_LENGTH+1; ti++) {
      int idx = 2*ti;
      t_in[idx] = crealf(input_sample[i + ti]);
      t_in[idx+1] = cimagf(input_sample[i + ti]);
    }
    my_volk_32fc_x2_dot_prod_32fc_generic(t_out, t_in, taps_coeff);
    output[i] = t_out[0] + t_out[1] * I;
    DEBUG(printf("MY_VOLK_OUT %5lu : IN %12.8f %12.8f : OUT %12.8f, %12.8f : OUTPUT %12.8f %12.8f \n", i, t_in[0], t_in[1], t_out[0], t_out[1], crealf(output[i]), cimagf(output[i])));
  }
}

void my_cmp_volk_32fc_x2_dot_prod_32fc_generic(fx_pt* result, const fx_pt* input, const fx_pt* taps)
{
  fx_pt * res = (fx_pt*) result;
  fx_pt * in  = (fx_pt*) input;
  fx_pt * tp  = (fx_pt*) taps;
  unsigned int n_2_ccomplex_blocks = (COEFF_LENGTH/2); //num_points/2;
  // unsigned int isodd = COEFF_LENGTH & 1;

  fx_pt sum0 = 0 + 0 * I;
  fx_pt sum1 = 0 + 0 * I;
  unsigned int i = 0;
  for(i = 0; i < n_2_ccomplex_blocks; ++i) {
    sum0 += in[0] * tp[0];
    sum1 += in[1] * tp[1];
    DEBUG(printf("CMP_VOLK_KERNEL %5u : 0 : IN %12.8f %12.8f : TAP %12.8f %12.8f : SUM0 %12.8f %12.8f\n", i, crealf(in[0]), cimagf(in[0]), crealf(tp[0]), cimagf(tp[0]), crealf(sum0), cimagf(sum0));
	  printf("                %5u : 1 : IN %12.8f %12.8f : TAP %12.8f %12.8f : SUM1 %12.8f %12.8f\n", i, crealf(in[1]), cimagf(in[1]), crealf(tp[1]), cimagf(tp[1]), crealf(sum1), cimagf(sum1)));
    in += 2;
    tp += 2;
  }

  res[0] = sum0 + sum1;
  //res[1] = sum1;

  // // Cleanup if we had an odd number of points
  // for(i = 0; i < isodd; ++i) {
  //   *result += input[COEFF_LENGTH - 1] * taps[COEFF_LENGTH - 1];
  // }
}

void
firG_using_cmp_volk(fx_pt output[SYNC_LENGTH], fx_pt input_sample[SYNC_LENGTH+COEFF_LENGTH], const fx_pt_ext rev_coeff[COEFF_LENGTH])
{
  for(unsigned long i = 0; i < SYNC_LENGTH; i++) {
    my_cmp_volk_32fc_x2_dot_prod_32fc_generic(&(output[i]), &input_sample[i], rev_coeff);
    DEBUG(printf("CMP_VOLK_OUT %5lu : IN %12.8f %12.8f : OUTPUT %12.8f %12.8f \n", i, crealf(input_sample[0]), cimagf(input_sample[0]), crealf(output[i]), cimagf(output[i])));
  }
}

