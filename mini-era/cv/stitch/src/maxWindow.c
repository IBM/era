/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"

F2D* maxWindow(F2D* im, I2D* window)
{
    int exR, exC, rows, cols, tr, tc, i, j, k;
    F2D *out, *temp;
    float t;
    int m;
    
    exR = asubsref(window,0)/2;
    exC = asubsref(window,1)/2;

    rows = im->height;
    cols = im->width;

    tr = rows+exR-1;
    tc = cols+exC-1;
    temp = fDeepCopy(im);
    out = fMallocHandle(rows, cols);

    for(i=0; i<rows; i++)
    {
        for(j=0; j<cols; j++)
        {
            t = 0;
            for(k=-exR; k<=exR; k++)
            {
                for(m=-exC; m<=exC; m++)
                {
                    if( (i+k) < 0 || (i+k) >= rows || (j+m) < 0 || (j+m) >= cols)
                        continue;
                    if( subsref(temp,(i+k),(j+m)) > t)
                        t = subsref(temp,(i+k),(j+m));
                }
            }
            subsref(out,i,j) = t;
        }
    }

    fFreeHandle(temp);
    return out;
}





