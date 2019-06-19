/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"

I2D* matchFeatures(F2D* vecF1, F2D* vecF2)
{
    int m, n, n1, n2, n1c, n2c, i, j;
    I2D *id, *retMatch, *t;
    F2D *val, *temp;
    int rows, cols;    

    n1 = vecF1->height;
    n1c = vecF1->width;
    n2 = vecF2->height;
    n2c = vecF2->width;

    retMatch = iMallocHandle(1, 2);

    for(i=0; i<n1; i++)
    {
        id = iMallocHandle(1, n2);
        t = iMallocHandle(1, n1c);

        for(j=0; j<n1c; j++)
            asubsref(t,j) = subsref(vecF1,i,j);

        temp = dist2(t, vecF2);
        val = fSort(temp,1);
        id = fSortIndices(temp,1);

        free(temp);
        free(t);
        
        if( asubsref(val,1) != 0 & (( asubsref(val,0) / asubsref(val,1) ) < 0.65))
        {
            t = retMatch;
            rows = t->height + 1;
            cols = t->width;
            retMatch = iMallocHandle(rows, cols);
            n = 0;
    
            for(m=0; m<(t->height*t->width); m++)
            {
                asubsref(retMatch,n++) = asubsref(t,m);
            }

            asubsref(retMatch,n++) = i;
            asubsref(retMatch,n) = asubsref(id,0);
        }
    }

    return retMatch;
}
