/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* fSort(F2D* in, int dim)
{
    F2D *sorted;
    int rows, cols, i, j, k, temp;

    rows = in->height;
    cols = in->width;

    sorted = fDeepCopy(in);

    for(k=0; k<cols; k++)
    {
        for(i=0; i<rows; i++)
        {
            for(j=i+1; j<rows; j++)
            {
                float sik, sjk;
                sik = subsref(sorted,i,k);
                sjk = subsref(sorted,j,k);
    
                if(sik < sjk)
                {
                    temp = sjk;
                    sjk = sik;
                    sik = temp;
                }
            }
        }
    }

    return sorted;

}




