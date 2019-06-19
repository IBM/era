/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* ffDivide(F2D* a, F2D* b)
{
    F2D *c;
    int i, j, rows, cols;

    rows = a->height;
    cols = a->width;

    c = fMallocHandle(rows, cols);
    
    for(i=0; i<(rows*cols); i++)
        asubsref(c,i) = asubsref(a,i) / asubsref(b,i);

    return c;
}
