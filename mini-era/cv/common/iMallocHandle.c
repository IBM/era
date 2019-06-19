/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include "sdvbs_common.h"

I2D* iMallocHandle(int rows, int cols)
{
    int i, j;
    I2D* out;
   
    out = (I2D*)malloc(sizeof(I2D) + sizeof(int)*rows*cols);
    out->height = rows;
    out->width = cols;
    
    return out;
}

