#ifndef SIMPLE_DFT_H
#define SIMPLE_DFT_H

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif 


void simple_dft(const float *inreal, const float *inimag,
		float *outreal, float *outimag,
		int inverse, int shift, unsigned n);

#endif

  
