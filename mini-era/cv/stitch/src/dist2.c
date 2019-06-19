/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"

F2D* dist2(I2D* x, F2D* c)
{
    int ndata, dimx, ncentres, dimc, i, j, k;
    F2D *n2, *t1, *t2;
    float temp;
    F2D *s1, *s2, *ctrans;
    F2D *mult1, *mult2, *mult3;

    ndata = x->height;
    dimx = x->width;

    ncentres = c->height;
    dimc = c->width;

    if(dimx != dimc)
        return NULL;

    s1 = fSetArray(ncentres, 1, 1);
    s2 = fSetArray(ndata, 1, 1);
    t1 = fMallocHandle(1, x->height);

    for(j=0; j<t1->width; j++)
    {
        temp = 0;
        for(i=0; i<t1->height; i++)
        {
            temp += subsref(x,j,i) * subsref(x,j,i);
        }

        asubsref(t1,j) = temp;
    }

    mult1 = fMtimes(s1, t1);
    t2 = fMallocHandle(1, c->height);

    for(j=0; j<t2->width; j++)
    {
        temp = 0;
        for(i=0; i<t2->height; i++)
        {
            temp += subsref(c,j,i) * subsref(c,j,i);
        }

        asubsref(t2,j) = temp;
    }

    mult2 = fMtimes(s2, t2);
    ctrans = fTranspose(c);
    mult3 = ifMtimes(x, ctrans);

    for(i=0; i<(mult3->height * mult3->width); i++)
        asubsref(mult3,i) = asubsref(mult3,i) * 2;

    free(t1);
    free(t2);
    free(s1);
    free(s2);
    free(ctrans);

    n2 = fMallocHandle(ndata, ncentres);
    for(i=0; i<(ndata*ncentres); i++)
        asubsref(n2,i) = asubsref(mult1,i) + asubsref(mult2,i) - asubsref(mult3,i);

    free(mult1);
    free(mult2);
    free(mult3);

    return n2;

}
