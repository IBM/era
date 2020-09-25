#include <stdio.h>
#include "debug.h"
#include "delay.h"

 
void delay(fx_pt out[DELAY_16_MAX_OUT_SIZE], unsigned num_inputs, fx_pt data[DELAY_16_MAX_IN_SIZE])
{
  /* delay_samples: */
  for(unsigned j = 0; j < DELAY_16_MAX_OUT_SIZE; j++) {
    int index = j-DELAY;
    if( index<0 ) {
      out[j] = 0 + 0*I; //fx_pt(0, 0);
    } else {
      out[j] = data[index];
    }
  }
    
  DEBUG(printf("\n  After doing delay...\n");
	for (int ti = 0; ti < (DELAY+8); ti++) {
printf("  DELAY_16 %5u : OUT %12.8f %1.28f : IN %12.8f %12.8f\n", ti, crealf(out[ti]), cimagf(out[ti]), crealf(data[ti]), cimagf(data[ti]));
	});
}

#define DELAY1   320
// NOTES: This version does not work across multiple calls -- 
void delay320(fx_pt out[DELAY_320_MAX_OUT_SIZE], fx_pt data[DELAY_320_MAX_IN_SIZE])
{
  DEBUG(printf("\nIn delay320...\n"));

  /* delay_samples: */
  for(unsigned j = 0; j < DELAY_320_MAX_OUT_SIZE; j++) {
    int index = j-DELAY1;
    if ( index < 0 ) {
      out[j] = 0 + 0*I; // fx_pt(0, 0);
    } else {
      out[j] = data[index];
    }
  }
    
  DEBUG2(printf("\n  After doing delay320...\n");
	for (int ti = 0; ti < DELAY_320_MAX_OUT_SIZE; ti++) {
printf("  DELAY_320 %5u : OUT %12.8f %1.28f : IN %12.8f %12.8f\n", ti, crealf(out[ti]), cimagf(out[ti]), crealf(data[ti]), cimagf(data[ti]));
	});
}
 
