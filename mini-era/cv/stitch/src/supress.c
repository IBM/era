/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"

F2D* supress(F2D* im, F2D* im1)
{
    int rows, cols, i, j;
    F2D *out;

    rows = im->height;
    cols = im->width;

    out = fSetArray(rows, cols, 0);

    for(i=0; i<rows; i++)
    {
        for(j=0; j<cols; j++)
        {
            if( subsref(im,i,j) == subsref(im1,i,j))
                subsref(out,i,j) = subsref(im,i,j);
        }
    }
    return out;
}



