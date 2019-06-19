/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* randWrapper(int m, int n)
{
    F2D *out;
    float seed;
    int i,j;

    out = fSetArray(m, n, 0);
    seed = 0.9;

    for(i=0; i<m; i++)
    {
        for(j=0; j<n; j++)
        {
            if(i<j)
                subsref(out,i,j) = seed * ((i+1.0)/(j+1.0));
            else
                subsref(out,i,j) = seed * ((j+1.0)/(i+1.0));
        }
    }

    return out;
}


