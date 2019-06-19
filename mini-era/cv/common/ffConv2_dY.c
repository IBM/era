/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* ffConv2_dY(F2D* a, F2D* b)
{
    F2D *c, *out;
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
    
    for(i=0; i<cj; i++)
    {
        for(j=0; j<ci; j++)
        {
            for(m=0; m<na; m++)
            {
                for(n=0; n<ma; n++)
                {
                    if( (i-m)>=0 && (j-n)>=0 && (i-m)<nb && (j-n)<mb )
                        subsref(c,j,i) += subsref(a,n,m) * subsref(b,j-n,i-m);
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
