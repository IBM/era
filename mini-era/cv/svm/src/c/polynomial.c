/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "svm.h"

float polynomial(int d, F2D* a, F2D* b, int dim)
{
    float ret;
    F2D *bt, *bt1;
    int i,j,r,c;

    r = b->height;
    c = b->width;

    bt = fMallocHandle(c, r);

    for(i=0; i<r; i++)
    {
        for(j=0; j<c; j++)
        {
            subsref(bt,j,i) = subsref(b,i,j);
        }
    }

    bt1 = fMtimes(a, bt);
    fFreeHandle(bt);
    
    if(bt1->height == 1 && bt1->width ==1)
        ret = pow(asubsref(bt1,0),d)/dim;
    else
    {
        fFreeHandle(bt1);
        return -1;
    }

    fFreeHandle(bt1);
    return ret;
}



