#include <stdio.h>
#include <stdlib.h>

int write_array_to_octave (float * data, unsigned int m, unsigned int n, char * filename, char * name)
{
  FILE *fp = fopen(filename, "w");
  int i, j;

  fprintf(fp, "# Created by PERFECT 2D-FFT Benchmark\n");
  fprintf(fp, "# name: %s\n", name);
  fprintf(fp, "# type: complex matrix\n");
  fprintf(fp, "# rows: %d\n", n);
  fprintf(fp, "# columns: %d\n", m);

  for (i = 0; i < n; i++) {
    for (j = 0; j < m; j++) {
      fprintf(fp, " (%.15g, %.15g)", data[2*i*m+2*j], data[2*i*m+2*j+1]);
    }
    fprintf(fp, "\n");
  }

  fclose(fp);

  return 0;
}
