/*******************************************************************************/
/* This is a simple implementation of a DFT (Discrete Fourier Xform) which     */
/* supports complex inputs (inreal, inimag) and complex outputs, as well       */
/* as the inverse FFT operation.  Ths input n is the "size" (num samples).     */
/* NOTES:                                                                      */
//  This has been cast from double to float representation
//  The code to support the "shift" options added
/*******************************************************************************/

#include <math.h>

#include "debug.h"
#include "simple_dft.h"

void simple_dft(const float *inreal, const float *inimag,
		float *outreal, float *outimag,
		int inverse, int shift, unsigned n) {
  
  float coef = (inverse ? 2 : -2) * M_PI;
  for (unsigned k = 0; k < n; k++) {  // For each output element
    float sumreal = 0;
    float sumimag = 0;
    for (unsigned t = 0; t < n; t++) {  // For each input element
      float angle = (coef * ((long long)t * k % n)) / n;
      sumreal += inreal[t] * cos(angle) - inimag[t] * sin(angle);
      sumimag += inreal[t] * sin(angle) + inimag[t] * cos(angle);
    }
    outreal[k] = sumreal;
    outimag[k] = sumimag;
  }

  /* shift: */
  if (shift) {
    float swap_r, swap_i;
    int M = (n/2);
    for(unsigned i = 0; i < M; i++) {
      swap_r = outreal[i];
      swap_i = outimag[i];
      outreal[i] = outreal[M+i];
      outimag[i] = outimag[M+i];
      outreal[M+i] = swap_r;
      outimag[M+i] = swap_i;
    }
  }

}

