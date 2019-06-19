/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

I2D* iTranspose(I2D* a)
{
    I2D *out;
    int m, p, p1, n, i, j, k;
    int temp;

    m = a->height;
    n = a->width;

    out = iMallocHandle(n, m);
    k = 0;
    for(i=0; i<n; i++)
    {
        for(j=0; j<m; j++)
            asubsref(out,k++) = subsref(a,j,i);
    }

    return out;
}

