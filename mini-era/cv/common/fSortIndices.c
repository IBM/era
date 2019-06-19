/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

I2D* fSortIndices(F2D* input, int dim)
{
    int rows, cols;
    F2D *in;
    int i, j, k;
    I2D *ind;

    rows = input->height;
    cols = input->width;

    in = fDeepCopy(input);
    ind = iMallocHandle(rows,cols);

    for(i=0; i<cols; i++) {
        for(j=0; j<rows; j++) {
            subsref(ind,j,i) = 0;
		}
	}

if(dim == 1)
{
    for(k=0; k<rows; k++)
    {
        for(i=0; i<cols; i++)
        {
            float localMax = subsref(in,k,i);
            int localIndex = i;
            subsref(ind,k,i) = i;
            for(j=0; j<cols; j++)
            {
                if(localMax < subsref(in,k,j))
                {
                    subsref(ind,k,i) = j;
                    localMax = subsref(in,k,j);
                    localIndex = j;
                }
            }
            subsref(in,k,localIndex) = 0;
        }
    }
    
    fFreeHandle(in);
    return ind;
}

    for(k=0; k<cols; k++)
    {
        for(i=0; i<rows; i++)
        {
            float localMax = subsref(in,i,k);
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

    fFreeHandle(in);
    return ind;
}



