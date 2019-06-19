/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* ffVertcat(F2D* matrix1, F2D* matrix2)
{
    F2D *outMatrix;
    int row1, col1, row2, col2, i, j, k;

    row1 = matrix1->height;
    col1 = matrix1->width;

    row2 = matrix2->height;
    col2 = matrix2->width;

    outMatrix = fMallocHandle(row1+row2, col1);
    
    for( i=0; i<col1; i++)
    {
        for (j=0; j<row1; j++)
        {
            subsref(outMatrix,j,i) = subsref(matrix1,j,i);
        }
        for( k=0; k<row2; k++)
        {
            subsref(outMatrix,(k+row1),i) = subsref(matrix2,k,i);
        }
    }

    return outMatrix;

}


