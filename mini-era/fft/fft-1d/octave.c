#include <stdio.h>
#include <stdlib.h>

int write_array_to_octave (float * data, unsigned int len, char * filename, char * name)
{
  FILE *fp = fopen(filename, "w");
  int i;

  fprintf(fp, "# Created by PERFECT 1D-FFT Benchmark\n");
  fprintf(fp, "# name: %s\n", name);
  fprintf(fp, "# type: complex matrix\n");
  fprintf(fp, "# rows: 1\n");
  fprintf(fp, "# columns: %d\n", len);

  for (i = 0; i < len; i++) {
    fprintf(fp, " (%.15g, %.15g)", data[2*i], data[2*i+1]);
  }
  fprintf(fp, "\n");

  fclose(fp);

  return 0;
}
