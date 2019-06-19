/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include "sdvbs_common.h"

I2D* isMinus(I2D* a, int b)
{
    I2D *c;
    int i, j, rows, cols;

    rows = a->height;
    cols = a->width;

    c = iMallocHandle(rows, cols);
    
    for(i=0; i<(rows*cols); i++)
        asubsref(c,i) = asubsref(a,i) - b;

    return c;
}
