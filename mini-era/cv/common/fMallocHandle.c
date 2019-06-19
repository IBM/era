/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include "sdvbs_common.h"

F2D* fMallocHandle(int rows, int cols)
{
    int i, j;
    F2D* out;
   
    out = (F2D*)malloc(sizeof(F2D) + sizeof(float)*rows*cols);
    out->height = rows;
    out->width = cols;
    
    return out;
}
