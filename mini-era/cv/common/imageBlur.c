/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* imageBlur(I2D* imageIn)
{
    int rows, cols;
    F2D *imageOut, *tempOut;
    float temp;
    I2D *kernel;
    int k, kernelSize, startCol, endCol, halfKernel, startRow, endRow, i, j, kernelSum;

    rows = imageIn->height;
    cols = imageIn->width;

    imageOut = fSetArray(rows, cols, 0);
    tempOut = fSetArray(rows, cols, 0);
    kernel = iMallocHandle(1, 5);

    asubsref(kernel,0) = 1;
    asubsref(kernel,1) = 4;
    asubsref(kernel,2) = 6;
    asubsref(kernel,3) = 4;
    asubsref(kernel,4) = 1;
    kernelSize = 5;
    kernelSum = 16;

    startCol = 2;  
    endCol = cols - 2;  
    halfKernel = 2;   

    startRow = 2;    
    endRow = rows - 2;  

    for(i=startRow; i<endRow; i++){
        for(j=startCol; j<endCol; j++)
        {
            temp = 0;
            for(k=-halfKernel; k<=halfKernel; k++)
            {
                temp += subsref(imageIn,i,j+k) * asubsref(kernel,k+halfKernel);
            }
            subsref(tempOut,i,j) = temp/kernelSum;
        }
    }
    
    for(i=startRow; i<endRow; i++)
    {
        for(j=startCol; j<endCol; j++)
        {
            temp = 0;
            for(k=-halfKernel; k<=halfKernel; k++)
            {
                temp += subsref(tempOut,(i+k),j) * asubsref(kernel,k+halfKernel);
            }
            subsref(imageOut,i,j) = temp/kernelSum;
        }
    }

    fFreeHandle(tempOut);
    iFreeHandle(kernel);
    return imageOut;
}
             

