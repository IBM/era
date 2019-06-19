/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

I2D* iSortIndices(I2D* in, int dim)
{
    I2D *sorted;
    int rows, cols, i, j, k, temp;
    I2D *ind;

    rows = in->height;
    cols = in->width;

    sorted = iDeepCopy(in);
    ind = iMallocHandle(rows, cols);

    for(i=0; i<cols; i++)
        for(j=0; j<rows; j++)
            subsref(ind,j,i) = 0;

    for(k=0; k<cols; k++)
    {
        for(i=0; i<rows; i++)
        {
            int localMax = subsref(in,i,k);
            int localIndex = i;
            subsref(ind,i,k) = i;
            for(j=0; j<rows; j++)
            {
                if(localMax < subsref(in,j,k))
                {
                    subsref(ind,i,k) = j;
                    localMax = subsref(in,j,k);
                    localIndex = j;
                }
            }
            subsref(in,localIndex,k) = 0;
        }
    }

    return ind;
}



