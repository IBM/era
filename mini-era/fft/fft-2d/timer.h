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

#if !defined (TIMER_H_)
#define TIMER_H_


#define INPUT_SIZE_SMALL 1
#define INPUT_SIZE_MEDIUM 2
#define INPUT_SIZE_LARGE 3

#ifndef INPUT_SIZE
    #define INPUT_SIZE INPUT_SIZE_LARGE
#endif


void init_timer (void);
double timer_getres (void);
void tic (void);
double toc (void);

#define STATS_INIT()                                                   \
  do {                                                                 \
    init_timer ();                                                     \
    printf ("{\n");                                                    \
    printf ("\t\"timer_res\": %20.15e", timer_getres ());              \
  } while (0)

#define STATS_END() do { printf ("\n}\n"); } while (0)

#define PRINT_STAT_INT(NAME_, VALUE_)                                  \
  do {                                                                 \
    printf(",\n\t\"%s\": %d", NAME_, VALUE_);                          \
  } while (0)

#define PRINT_STAT_INT64(NAME_, VALUE_)                                \
  do {                                                                 \
    printf(",\n\t\"%s\": %ld", NAME_, VALUE_);                         \
  } while (0)

#define PRINT_STAT_INT64_ARRAY_AS_PAIR(NAME_, ARY_, LEN_)              \
  do {                                                                 \
    printf(",\n\t\"%s\": [", NAME_);                                   \
    for(uint64_t i = 0; i < LEN_ - 1; i++) {                           \
      printf("\n\t\t[ %ld, %ld],", i, ARY_ [i]);                       \
    }                                                                  \
    printf("\n\t\t[ %ld, %ld]", LEN_ -1, ARY_ [LEN_ -1]);              \
    printf("\n\t]");                                                   \
  } while (0)

#define PRINT_STAT_DOUBLE(NAME_, VALUE_)                               \
  do {                                                                 \
    printf(",\n\t\"%s\": %20.15e", NAME_, VALUE_);                     \
  } while (0)

#define PRINT_STAT_HEX64(NAME_, VALUE_)                                \
  do {                                                                 \
    printf(",\n\t\"%s\":\"0x%lx\"", NAME_, VALUE_);                    \
  } while (0)

#define PRINT_STAT_STRING(NAME_, VALUE_)                               \
  do {                                                                 \
    printf(",\n\t\"%s\":\"%s\"", NAME_, VALUE_);                    \
  } while (0)


#endif /* TIMER_H_ */
