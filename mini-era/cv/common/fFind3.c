/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* fFind3(F2D* in)
{
    int r, k, y, x, i, j;
    F2D *points;

    y = in->height;
    x = in->width;

    r = 0;
    for(i=0; i<y; i++)
    {
        for(j=0; j<x; j++)
        {
            if(subsref(in,i,j) != 0)
                r++;
        }
    }
    
    points = fSetArray(r, 3, 0);

    k = 0;
    for(j=0; j<x; j++)
    {
        for(i=0; i<y; i++)
        {
            if( subsref(in,i,j) != 0)
            {
                subsref(points,k,0) = j*1.0;
                subsref(points,k,1) = i*1.0;
                subsref(points,k,2) = subsref(in,i,j);
                k++;
            }
        }
    }

    return points;
}



