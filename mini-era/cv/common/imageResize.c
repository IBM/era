/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* imageResize(F2D* imageIn)
{
    int m, k, rows, cols;
    F2D *imageOut;
    I2D *kernel;
    float tempVal;
    int kernelSize, startCol, endCol, halfKernel, startRow, endRow, i, j, kernelSum;
    int outputRows, outputCols;
    F2D *temp;

    rows = imageIn->height;
    cols = imageIn->width;
    
    // level 1 is the base image.

    outputRows = floor((rows+1)/2);
    outputCols = floor((cols+1)/2);

    temp = fSetArray(rows, outputCols, 0);
    imageOut = fSetArray(outputRows, outputCols, 0);
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

    for(i=startRow; i<endRow; i++)
    {
        m = 0;
        for(j=startCol; j<endCol; j+=2)
        {
            tempVal = 0;
            for(k=-halfKernel; k<=halfKernel; k++)
            {
                tempVal += subsref(imageIn,i,j+k) * asubsref(kernel,k+halfKernel);
            }
            subsref(temp,i,m) = tempVal/kernelSum;
            m = m+1;
        }
    }
    
    m = 0;
    for(i=startRow; i<endRow; i+=2)
    {
        for(j=0; j<outputCols; j++)
        {
            tempVal = 0;
            for(k=-halfKernel; k<=halfKernel; k++)
            {
                tempVal += subsref(temp,(i+k),j) * asubsref(kernel,k+halfKernel);
            }
            subsref(imageOut,m,j) = (tempVal/kernelSum);
        }    
        m = m+1;
    }

    fFreeHandle(temp);
    iFreeHandle(kernel);
    return imageOut;
    
}
