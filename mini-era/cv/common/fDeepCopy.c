/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* fDeepCopy(F2D* in)
{
    int i, j;
    F2D* out;
    int rows, cols;
    
    rows = in->height;
    cols = in->width;

    out = fMallocHandle(rows, cols);
    
    for(i=0; i<rows; i++) {
        for(j=0; j<cols; j++) {
            subsref(out,i,j) = subsref(in,i,j);
		}
   	} 
    return out;
}
