/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

I2D* iMinus(I2D* a, I2D* b)
{
    I2D *c;
    int i, j, rows, cols;

    rows = a->height;
    cols = a->width;

    c = iMallocHandle(rows, cols);
    
    for(i=0; i<(rows*cols); i++)
        asubsref(c,i) = asubsref(a,i) - asubsref(b,i);

    return c;
}
