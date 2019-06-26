/* -*-Mode: C;-*- */

#define _XOPEN_SOURCE 500
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "fft-1d.h"

#define LOGN (14)
#define N (1<<LOGN)
#define c 300000000
#define fs 32768000
#define alpha 4800000000000
#define threshold -100

int read_input_file(float* data, unsigned int num, char* filename)
{
  FILE *fp = fopen(filename, "r");
  int i;
  for (i=0; i<num; i++) {
    fscanf(fp, "%f", (data+i));
  }
  fclose(fp);
  return 0;
}

float calculate_peak_dist_from_fmcw(float* data)
{
  fft (data, N, LOGN, -1);
  float max_psd = 0;
  unsigned int max_index;
  unsigned int i;
  float temp;
  for (i=0; i<N; i++) {
    temp = (pow(data[2*i],2) + pow(data[2*i+1],2))/100.0;
    if (temp > max_psd) {
      max_psd = temp;
      max_index = i;
    }
  }
  float distance = ((float)(max_index*((float)fs)/((float)(N))))*0.5*c/((float)(alpha));
  //printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance, max_psd, max_index);
  if (max_psd > 1e-10*pow(8192,2))
    return distance;
  else
    return INFINITY;
}

int main (int argc, char * argv[])
{
  if (argc == 2) {
    printf("FMCW file used: %s\n", argv[1]);
  }
  else {
    printf("Usage: %s file.dat\n", argv[0]);
    return 1;
  }
  
  float * a;
  a = malloc (2 * N * sizeof(float));
  read_input_file (a, N, argv[1]);
  float dist = calculate_peak_dist_from_fmcw(a);
  printf("Distance of object from FMCW data = %.2f m\n", dist);
  free (a);
  return 0;
}
