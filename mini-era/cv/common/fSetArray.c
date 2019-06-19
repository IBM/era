/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include "sdvbs_common.h"

F2D* fSetArray(int rows, int cols, float val)
{
    int i, j;
    F2D *out;
    out = fMallocHandle(rows, cols);
    
    for(i=0; i<rows; i++) {
        for(j=0; j<cols; j++) {
            subsref(out,i,j) = val;
		}
   	} 
    return out;
    
}
