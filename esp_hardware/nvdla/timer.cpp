/* -*-Mode: C;-*- */

/**BeginCopyright************************************************************
 *
 * $HeadURL: https://pastec.gtri.gatech.edu/svn/svn-dpc/INNC/projects/PERFECT-TAV-ES/suite/wami/kernels/debayer/wami_debayer.c $
 * $Id: wami_debayer.c 8546 2014-04-02 21:36:22Z tallent $
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

#define _XOPEN_SOURCE 600
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "timer.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#if defined(_OPENMP)
void
init_timer (void)
{
  /* Empty. */
}

double
timer (void)
{
  return omp_get_wtime ();
}

double
timer_getres (void)
{
  return omp_get_wtick ();
}

#elif defined(__MACH__)

/* tallent: quick hack to support MacOS */ 

#warning "MacOS timer is a nop"

void
init_timer (void)
{
  /* Empty. */
}

double
timer (void)
{
  return 0.0;
}

double
timer_getres (void)
{
  return 0.0;
}


#else /* Elsewhere */
static clockid_t clockid;

#if defined(CLOCK_REALTIME_ID)
#define CLKID CLOCK_REALTIME_ID
#define CLKIDNAME "CLOCK_REALTIME_ID"
#elif defined(CLOCK_THREAD_CPUTIME_ID)
#define CLKID CLOCK_THREAD_CPUTIME_ID
#define CLKIDNAME "CLOCK_THREAD_CPUTIME_ID"
#elif defined(CLOCK_REALTIME_ID)
#warning "Falling back to realtime clock."
#define CLKID CLOCK_REALTIME_ID
#define CLKIDNAME "CLOCK_REALTIME_ID"
#else
#error "Cannot find a clock!"
#endif

/**
* @brief Initialize the system timer
*/
void
init_timer (void)
{
  int err;
  err = clock_getcpuclockid (0, &clockid);
  if (err >= 0)
    return;
  fprintf (stderr, "Unable to find CPU clock, falling back to "
           CLKIDNAME "\n");
  clockid = CLKID;
}

double
timer (void)
{
/*  struct timespec tp;
  clock_gettime (clockid, &tp);
  return (double) tp.tv_sec + 1.0e-9 * (double) tp.tv_nsec;*/
    struct timeval tv;
    gettimeofday ( &tv, NULL );
    return (tv.tv_sec +  0.0000001 * tv.tv_usec);
}

/**
* @brief Get the resolution of the system clock
*
* @return Clock Resolution
*/
double
timer_getres (void)
{
  struct timespec tp;
  clock_getres (clockid, &tp);
  return (double) tp.tv_sec + 1.0e-9 * (double) tp.tv_nsec;
}

#endif

static double last_tic = -1.0;

/**
* @brief Start the timer
*/
void
tic (void)
{
  last_tic = timer ();
}

/**
* @brief Stop the timer and return the time taken
*
* @return Time since last tic()
*/
double
toc (void)
{
  const double t = timer ();
  const double out = t - last_tic;
  last_tic = t;
  return out;
}
