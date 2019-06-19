/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* fSum2(F2D* inMat, int dir)
{
    F2D *outMat;
    int rows, cols, i, j, k;
    float temp;
    int newRow, newCols;

    rows = inMat->height;
    cols = inMat->width;

    if(dir == 1)
    { 
        newRow = 1;
        newCols = cols;
    }
    else
    {
        newRow = rows;
        newCols = 1;
    }

    outMat = fSetArray(newRow,newCols,0);

    if(dir == 1)
    {
        for (i=0; i<cols; i++)
        {
            temp = 0;
            for( j=0; j<rows; j++)
                temp = temp + subsref(inMat,j,i);
            asubsref(outMat,i) = temp;
        }
    }
    else
    {
        for( i=0; i<rows; i++)
        {
            temp = 0;
            for( j=0; j<cols; j++)
                temp = temp + subsref(inMat,i,j);
            subsref(outMat,i,0) = temp;
        }
    }

    return outMat;
}




