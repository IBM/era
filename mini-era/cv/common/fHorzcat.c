/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* fHorzcat(F2D* a, F2D* b)
{
    F2D* out_, *out, *c;
    int rows=0, cols=0, i, j, k, c_1, c_2, r_3, c_3;
    int r_1;

    r_1 = a->height;
    c_1 = a->width;
    cols += c_1;
    c_2 = b->width;
    cols += c_2;
    rows = r_1;

    out = fMallocHandle(rows, cols);    
    
    for(i=0; i<rows; i++)
    {
        k = 0;
        for(j=0; j<c_1; j++)
        {
            subsref(out,i,k) = subsref(a,i,j);
            k++;
        }
        for(j=0; j<c_2; j++)
        {
            subsref(out,i,k) = subsref(b,i,j);
            k++;
        }
    }

    return out;
}


