/* -*-Mode: C;-*- */
/*****************************************************************
 ** NOTE: This code is adapted from the PERFECT Benchmark Suite
 **  the following Copyright is retained because of its origin.
 *****************************************************************/

/**BeginCopyright************************************************************
 *
 * $HeadURL: https://pastec.gtri.gatech.edu/svn/svn-dpc/INNC/projects/PERFECT-TAV-ES/suite/required/fft-1d/src/fft.c $
 * $Id: fft.c 8546 2014-04-02 21:36:22Z tallent $
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

#include "fft-1d.h"
#include <stdio.h>
#include <stdlib.h>

#include "debug.h"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif 


static unsigned int
_rev (unsigned int v)
{
  unsigned int r = v;
  int s = sizeof(v) * CHAR_BIT - 1;

  for (v >>= 1; v; v >>= 1)
  {
    r <<= 1;
    r |= v & 1;
    s--;
  }
  r <<= s;

  return r;
}


void
bit_reverse (float * rw, float * iw, unsigned int N, unsigned int bits)
{
  unsigned int i, s, shift;
  s = sizeof(i) * CHAR_BIT - 1;
  shift = s - bits + 1;

  for (i = 0; i < N; i++) {
    unsigned int r;
    float t_real, t_imag;
    r = _rev (i);
    r >>= shift;

    if (i < r) {
      t_real = rw[i];
      t_imag = iw[i];
      rw[i] = rw[r];
      iw[i] = iw[r];
      rw[r] = t_real;
      iw[r] = t_imag;
    }
  }
}


int
fft(float * rdata, float * idata, int inverse, int shift, unsigned int N, unsigned int logn)
{
  unsigned int transform_length;
  unsigned int a, b, i, j, bit;
  float theta, t_real, t_imag, w_real, w_imag, s, t, s2, z_real, z_imag;
  int sign = (inverse ? 1 : -1);
  
  transform_length = 1;

  /* bit reversal */
  bit_reverse (rdata, idata, N, logn);

  /* calculation */
  for (bit = 0; bit < logn; bit++) {
    w_real = 1.0;
    w_imag = 0.0;

    theta = 1.0 * sign * M_PI / (float) transform_length;

    s = sin (theta);
    t = sin (0.5 * theta);
    s2 = 2.0 * t * t;

    for (a = 0; a < transform_length; a++) {
      for (b = 0; b < N; b += 2 * transform_length) {
	i = b + a;
	j = b + a + transform_length;

	z_real = rdata[j];
	z_imag = idata[j];

	t_real = w_real * z_real - w_imag * z_imag;
	t_imag = w_real * z_imag + w_imag * z_real;

	/* write the result */
	rdata[j]  = rdata[i] - t_real;
	idata[j]  = idata[i] - t_imag;
	rdata[i] += t_real;
	idata[i] += t_imag;
      }

      /* adjust w */
      t_real = w_real - (s * w_imag + s2 * w_real);
      t_imag = w_imag + (s * w_real - s2 * w_imag);
      w_real = t_real;
      w_imag = t_imag;

    }

    transform_length *= 2;
  }

  if (shift) {
    float swap_r, swap_i;
    int M = (N/2);
    /* shift: */
    for(unsigned i = 0; i < M; i++) {
      swap_r = rdata[i];
      swap_i = idata[i];
      rdata[i] = rdata[M+i];
      idata[i] = idata[M+i];
      rdata[M+i] = swap_r;
      idata[M+i] = swap_i;
    }
  }
  return 0;
}


