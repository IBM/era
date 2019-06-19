/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* calcSobel_dY(F2D* imageIn)
{
    int rows, cols;
    I2D *kernel_1, *kernel_2;
	float temp;
    int kernelSize, startCol, endCol, halfKernel, startRow, endRow, i, j, kernelSum;
    int k, kernelSum_2, outputRows, outputCols;
    F2D *imageOut, *tempOut;
    float kernelSum_1;

    rows = imageIn->height;
    cols = imageIn->width;
    
    // level 1 is the base image.

    outputRows = rows; 
    outputCols = cols;

    imageOut = fSetArray(outputRows, outputCols, 0);
    tempOut = fSetArray(outputRows, outputCols, 0);
    kernel_1 = iMallocHandle(1, 3);
    kernel_2 = iMallocHandle(1, 3);

    asubsref(kernel_1,0) = 1;
    asubsref(kernel_1,1) = 0;
    asubsref(kernel_1,2) = -1;
    kernelSize = 3;
    kernelSum_1 = 2.0;
    
    asubsref(kernel_2,0) = 1;
    asubsref(kernel_2,1) = 2;
    asubsref(kernel_2,2) = 1;
    kernelSum_2 = 4;

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
                temp += subsref(imageIn,(i+k),j) * asubsref(kernel_1,k+halfKernel);
            }
            subsref(tempOut,i,j) = temp/kernelSum_1;
        }
    }

    for(i=startRow; i<endRow; i++)
    {
        for(j=startCol; j<endCol; j++)
        {
            temp = 0;
            for(k=-halfKernel; k<=halfKernel; k++)
            {
                temp += subsref(tempOut,i,j+k) * asubsref(kernel_2,k+halfKernel);
            }
            subsref(imageOut,i,j) = temp/(float)kernelSum_2;
        }
    }

    fFreeHandle(tempOut);
    iFreeHandle(kernel_1);
    iFreeHandle(kernel_2);
    return imageOut;
    
}
