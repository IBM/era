/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* ifMtimes(I2D* a, F2D* b)
{
    F2D *out;
    int m, p, p1, n, i, j, k;
    float temp;

    m = a->height;
    p = a->width;

    p1 = b->height;
    n = b->width;

    out = fMallocHandle(m,n);

    for(i=0; i<m; i++)
    {
        for(j=0; j<n; j++)
        {
            temp = 0;
            for(k=0; k<p; k++)
            {
                temp += subsref(b,k,j) * subsref(a,i,k);
            }
            subsref(out,i,j) = temp;
        }
    }

    return out;
}



