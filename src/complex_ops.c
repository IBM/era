
#include <math.h>
#include <complex.h>

#include "complex_ops.h"
//#include "hls_math.h"

void complex_conjugate(fx_pt out[CMP_CONJ_MAX_SIZE], unsigned num_ins, fx_pt data[CMP_CONJ_MAX_SIZE])
{
  /*conj:*/
  for (unsigned i = 0; i < num_ins /*CMP_CONJ_MAX_SIZE*/; i++) {
    out[i] = conj(data[i]);
  }
}

void complex_to_mag_squared(fx_pt1 out[CMP2MAGSQ_MAX_SIZE], unsigned num_ins, fx_pt data[CMP2MAGSQ_MAX_SIZE]) // signal_power
{
  fx_pt1 a,b;
  /*power:*/
  for (unsigned i = 0; i < num_ins /*CMP2MAGSQ_MAX_SIZE*/; i++) {
    a = crealf(data[i]);
    b = cimagf(data[i]);
    out[i] = a*a + b*b;
  }
}

void complex_to_magnitude(fx_pt1 out[CMP2MAG_MAX_SIZE], unsigned num_ins, fx_pt data[CMP2MAG_MAX_SIZE])
{
  fx_pt1_ext tmp,a,b;
  
  /*magn:*/
  for (unsigned i = 0; i < num_ins /*CMP2MAG_MAX_SIZE*/; i++) {
    a = (fx_pt1_ext) crealf(data[i]);
    b = (fx_pt1_ext) cimagf(data[i]);
    tmp = a*a+b*b;
    out[i] = (fx_pt1)sqrt(tmp);
    //printf("a=%f; b=%f; -> %f\n",a.to_float(),b.to_float(), out[i].to_float() );
  }
}

fx_pt1 abs_c(fx_pt data)
{
  fx_pt1_ext tmp,a,b;
  
  /*magn:*/
  a = (fx_pt1_ext) crealf(data);
  b = (fx_pt1_ext) cimagf(data);
  tmp = a*a+b*b;
  return (fx_pt1)sqrt(tmp);
}

#define N_ext 64
#define int_ext 16

void complex_multiply(fx_pt out[CMP_MULT_MAX_SIZE], unsigned num_ins, fx_pt data1[CMP_MULT_MAX_SIZE], fx_pt data2[CMP_MULT_MAX_SIZE])
{
  /*mult:*/
  for(unsigned j = 0; j < num_ins /*CMP_MULT_MAX_SIZE*/; j++) {
    //complex<ap_fixed<N_ext,int_ext> > out_ext = complex<ap_fixed<N_ext,int_ext> > (data1[j])* complex<ap_fixed<N_ext,int_ext> >(data2[j]);
    //    float complex out_ext = complex<float>(data1[j]) * complex<float>(data2[j]);
    //out[j] = fx_pt(out_ext);
    out[j] = data1[j] * data2[j];
  }
}


void division(fx_pt1 output[DIVIDE_MAX_SIZE], unsigned num_inputs, fx_pt1 dividend[DIVIDE_MAX_SIZE], fx_pt1 divisor[DIVIDE_MAX_SIZE])
{
  /* div: */
  for(unsigned j = 0; j < num_inputs /*DIVIDE_MAX_SIZE*/; j++) {
    if (divisor[j] != 0) {
      output[j] = (fx_pt1) ((fx_pt1_ext)dividend[j] / (fx_pt1_ext)divisor[j]);
    }
    /* } else {
       output[j] = -1;
       }*/
  }
} //end function
