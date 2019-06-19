/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* ffiConv2(F2D* a, I2D* b)
{
    F2D *c;
    F2D *out;
    int ma, na, mb, nb, ci, cj, i, j, m, n;
    int r_index, c_index;

    ma = a->height;
    na = a->width;

    mb = b->height;
    nb = b->width;
    
    r_index = ceil((mb + 1.0)/2.0);
    c_index = ceil((nb + 1.0)/2.0);

    ci = ma+mb-1;
    cj = na+nb-1;

    c = fSetArray(ci, cj, 0);

    for(i=0; i<ci; i++)
    {
        for(j=0; j<cj; j++)
        {
            for(m=0; m<ma; m++)
            {
                for(n=0; n<na; n++)
                {
                    if( (i-m)>=0 && (j-n)>=0 && (i-m)<mb && (j-n)<nb )
                        subsref(c,i,j) += subsref(a,m,n) * subsref(b,(i-m),(j-n));
                }
            }

        }
    }

    out = fMallocHandle(ma, na);
    for(i=0; i<ma; i++)
    {
        for(j=0; j<na; j++)
        {
            subsref(out,i,j) = subsref(c,(i+r_index-1),(j+c_index-1));
        }
    }

    return out;
}
