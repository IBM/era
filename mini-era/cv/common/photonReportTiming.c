/********************************
Author: Sravanthi Kota Venkata
********************************/

/** C File **/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include "timingUtils.h"
#include "sdvbs_common.h"

unsigned int * photonReportTiming(unsigned int* startCycles,unsigned int* endCycles)
{

    unsigned int *elapsed;
    elapsed = (unsigned int*)malloc(sizeof(unsigned int)*2);
	unsigned long long start = (((unsigned long long)0x0) | startCycles[0]) << 32 | startCycles[1];
	unsigned long long end = (((unsigned long long)0x0) | endCycles[0]) << 32 | endCycles[1];
	unsigned long long diff = end - start;
	elapsed[0] = (unsigned int)(diff >> 32);
	elapsed[1] = (unsigned int)(diff & 0xffffffff);
    return elapsed;

}

/** End of C Code **/
