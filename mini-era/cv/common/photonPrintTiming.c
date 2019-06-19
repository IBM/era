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

void photonPrintTiming(unsigned int * elapsed)
{
    if(elapsed[1] == 0)
	    printf("Cycles elapsed\t\t- %u\n\n", elapsed[0]);
    else
	    printf("Cycles elapsed\t\t- %u%u\n\n", elapsed[1], elapsed[0]);
}

/** End of C Code **/
