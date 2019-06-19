/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <math.h>
#include "sdvbs_common.h"

I2D* ifDeepCopy(F2D* in)
{
    int i, j;
    I2D *out;
    int rows, cols;
   
    rows = in->height;
    cols = in->width;
    
    out = iMallocHandle(rows, cols);
    
    for(i=0; i<rows; i++)
        for(j=0; j<cols; j++)
            subsref(out,i,j) = (int)(subsref(in,i,j));
    
    return out;
    
}
