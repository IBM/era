/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sift.h"

/**
    DIFFSS  Difference of scale space
    Returns a scale space DSS obtained by subtracting
    consecutive levels of the scale space SS.

    In SIFT, this function is used to compute the difference of
    Gaussian scale space from the Gaussian scale space of an image.
**/

F2D** diffss(F2D** ss, int num, int intervals)
{
    F2D** dss;
    int o, sizeM, sizeN, s, i, j; 
    F2D *current, *in1, *in2;

    dss = malloc(num*intervals*sizeof(F2D*));

    for(o=0; o<num; o++)
    {
        for(s=0; s<(intervals-1); s++)
        {
            sizeM = ss[o*intervals+s]->height;
            sizeN = ss[o*intervals+s]->width;

            dss[o*intervals+s] = fMallocHandle(sizeM, sizeN);

            current = dss[o*intervals+s];
            in1 = ss[o*intervals+s+1];
            in2 = ss[o*intervals+s];
            
            for(i=0; i<sizeM; i++)
            {
                for(j=0; j<sizeN; j++)
                {
                    subsref(current,i,j) = subsref(in1,i,j) - subsref(in2,i,j);
                }
            }
        }
    }

    return dss;

}




