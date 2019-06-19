/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

I2D* iReshape(I2D* in, int rows, int cols)
{
    I2D *out;
    int i, j, k;
    int r, c;

    r = in->height;
    c = in->width;
    
    out = iMallocHandle(rows, cols);
    
    k = 0;
    for(i=0; i<c; i++)
        for(j=0; j<r; j++)
            asubsref(out,k++) = subsref(in,j,i);
    
    return out;
}

