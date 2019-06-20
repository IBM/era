/* -*-Mode: C;-*- */

/**BeginCopyright************************************************************
 *
 * $HeadURL: https://pastec.gtri.gatech.edu/svn/svn-dpc/INNC/projects/PERFECT-TAV-ES/suite/required/fft-2d/src/main.c $
 * $Id: main.c 8546 2014-04-02 21:36:22Z tallent $
 *
 *---------------------------------------------------------------------------
 * Part of PERFECT Benchmark Suite (hpc.pnnl.gov/projects/PERFECT/)
 *---------------------------------------------------------------------------
 *
 * Copyright ((c)) 2014, Battelle Memorial Institute
 * Copyright ((c)) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * 1. Battelle Memorial Institute (hereinafter Battelle) and Georgia Tech
 *    Research Corporation (GTRC) hereby grant permission to any person
 *    or entity lawfully obtaining a copy of this software and associated
 *    documentation files (hereinafter "the Software") to redistribute
 *    and use the Software in source and binary forms, with or without
 *    modification.  Such person or entity may use, copy, modify, merge,
 *    publish, distribute, sublicense, and/or sell copies of the
 *    Software, and may permit others to do so, subject to the following
 *    conditions:
 * 
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimers.
 * 
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 * 
 *    * Other than as used herein, neither the name Battelle Memorial
 *      Institute nor Battelle may be used in any form whatsoever without
 *      the express written consent of Battelle.
 * 
 *      Other than as used herein, neither the name Georgia Tech Research
 *      Corporation nor GTRC may not be used in any form whatsoever
 *      without the express written consent of GTRC.
 * 
 *    * Redistributions of the software in any form, and publications
 *      based on work performed using the software should include the
 *      following citation as a reference:
 * 
 *      Kevin Barker, Thomas Benson, Dan Campbell, David Ediger, Roberto
 *      Gioiosa, Adolfy Hoisie, Darren Kerbyson, Joseph Manzano, Andres
 *      Marquez, Leon Song, Nathan R. Tallent, and Antonino Tumeo.
 *      PERFECT (Power Efficiency Revolution For Embedded Computing
 *      Technologies) Benchmark Suite Manual. Pacific Northwest National
 *      Laboratory and Georgia Tech Research Institute, December 2013.
 *      http://hpc.pnnl.gov/projects/PERFECT/
 *
 * 2. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 *    BATTELLE, GTRC, OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *    STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *    OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **EndCopyright*************************************************************/

#define _XOPEN_SOURCE 500
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "timer.h"
#include "fft-2d.h"

#if INPUT_SIZE == INPUT_SIZE_SMALL
#define LOGN (10)

#elif INPUT_SIZE == INPUT_SIZE_MEDIUM
#define LOGN (12)

#elif INPUT_SIZE == INPUT_SIZE_LARGE
#define LOGN (13)

#else
#error "Unhandled value for INPUT_SIZE"
#endif

#define N (1<<LOGN)
#define M (1<<LOGN)
#define LOGM LOGN


int main (int argc, char * argv[])
{
  float * a;
  float * t;
  unsigned int i, j;

  srand (time (NULL));

  STATS_INIT ();
  PRINT_STAT_STRING ("kernel", "fft-2d");
  PRINT_STAT_INT ("x_length", N);
  PRINT_STAT_INT ("y_length", M);

  a = malloc (2 * M * N * sizeof(float));
  t = malloc (2 * N * sizeof(float));  /* used for columns */

  /* random initialization */
  tic ();
  for (i = 0; i < M * N; i++) {
    a[2*i  ] = (float) rand () / (float) RAND_MAX;
    a[2*i+1] = (float) rand () / (float) RAND_MAX;
  }
  PRINT_STAT_DOUBLE ("time_generate_random_data", toc ());

  /* Write the generated input file to disk */
  write_array_to_octave (a, M, N, "random_input.mat", "input");
  PRINT_STAT_STRING ("input_file", "random_input.mat");

  /* Perform the FFT */
  tic ();
  
  /* 1D FFT on each row */
  for (i = 0; i < N; i++) {
    fft (&a[2*i*M], M, LOGM, -1);
  }

  /* 1D FFT on each column */
  for (i = 0; i < M; i++) {
    for (j = 0; j < N; j++) {
      t[2*j  ] = a[2*(M*j+i)  ];
      t[2*j+1] = a[2*(M*j+i)+1];
    }

    fft (t, N, LOGN, -1);

    for (j = 0; j < N; j++) {
      a[2*(M*j+i)  ] = t[2*j  ];
      a[2*(M*j+i)+1] = t[2*j+1];
    } 
  }
  PRINT_STAT_DOUBLE ("time_fft-2d", toc ());
 
  /* Write the results out to disk */
  write_array_to_octave (a, M, N, "fft_output.mat", "output");
  PRINT_STAT_STRING ("output_file", "fft_output.mat");

  STATS_END ();

  free (t);
  free (a);
  return 0;
}

