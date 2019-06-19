/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include "sdvbs_common.h"

F2D* calcSobel_dX(F2D* imageIn)
{
    int rows, cols;
    F2D *kernel_1, *kernel_2;
    float temp;
    int kernelSize, startCol, endCol, halfKernel, startRow, endRow, i, j, kernelSum;
    int k, kernelSum_1, kernelSum_2;
    F2D *imageOut, *tempOut;

    rows = imageIn->height;
    cols = imageIn->width;
    
    imageOut = fSetArray(rows, cols, 0);
    tempOut = fSetArray(rows, cols, 0);
    kernel_1 = fMallocHandle(1, 3);
    kernel_2 = fMallocHandle(1, 3);

    asubsref(kernel_1,0) = 1;
    asubsref(kernel_1,1) = 2;
    asubsref(kernel_1,2) = 1;

    kernelSize = 3;
    kernelSum_1 = 4;
    
    asubsref(kernel_2,0) = 1;
    asubsref(kernel_2,1) = 0;
    asubsref(kernel_2,2) = -1;

    kernelSum_2 = 2;

    startCol = 1;
    endCol = cols - 1;
    halfKernel = 1;

    startRow = 1;
    endRow = rows - 1;

    for(i=startRow; i<endRow; i++)
    {
        for(j=startCol; j<endCol; j++)
        {
            temp = 0;
            for(k=-halfKernel; k<=halfKernel; k++)
            {
                temp += subsref(imageIn,i,j+k) * asubsref(kernel_2,k+halfKernel);
            }
            subsref(tempOut,i,j) = temp/kernelSum_2;
        }
    }
    
    for(i=startRow; i<endRow; i++)
    {
        for(j=startCol; j<endCol; j++)
        {
            temp = 0;
            for(k=-halfKernel; k<=halfKernel; k++)
            {
                temp += subsref(tempOut,(i+k),j) * asubsref(kernel_1,k+halfKernel);
            }
            subsref(imageOut,i,j) = temp/(float)kernelSum_1;
        }
    }

    fFreeHandle(tempOut);
    fFreeHandle(kernel_1);
    fFreeHandle(kernel_2);
    return imageOut;
    
}
