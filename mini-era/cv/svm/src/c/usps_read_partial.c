/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "svm.h"

F2D* usps_read_partial(F2D* dcell1, F2D* dcell2, int idx, int opt, int dim, int iterations)
{
    F2D *ret, *X, *Y;
    F2D *ADD;
    int i, j, k, m, n;

    F2D *temp, *temp1;

    if(opt == 1)
    {
        for(i=0; i<iterations; i++)
        {
            if(i==0)
            {
                X = fMallocHandle(dim, dcell1->width);
                for(m=0; m<dim; m++)
                {
                    for(n=0; n<dcell1->width; n++)
                    {
                        subsref(X,m,n) = subsref(dcell1,m,n);
                    }
                }
            }
            else
            {
                temp = fDeepCopy(X);
                fFreeHandle(X);
                temp1 = fMallocHandle(dim, dcell2->width);

                for(m=0; m<dim; m++)
                {
                    for(n=0; n<dcell2->width; n++)
                    {
                        subsref(temp1,m,n) = subsref(dcell2,m,n);
                    }
                }
                X = ffVertcat(temp, temp1);
                fFreeHandle(temp);
                fFreeHandle(temp1);
            }
        }
        ret = fDeepCopy(X);
        fFreeHandle(X);
    }
    else
    {
        for(i=0; i<iterations; i++)
        {
            if(idx == -1)
                ADD = fSetArray(dim, 1, i+1);
            else
            {
                if( i!= idx)
                    ADD = fSetArray(dim, 1, -1);
                else
                    ADD = fSetArray(dim, 1, 1);
            }
            if(i==0)
            {
                Y = fDeepCopy(ADD);
            }
            else
            {
                F2D* t = fDeepCopy(Y);
                fFreeHandle(Y);
                Y = ffVertcat(t, ADD);
                fFreeHandle(t);
            }
            fFreeHandle(ADD);
        }
        ret = fDeepCopy(Y);
        fFreeHandle(Y);
    }
    
    return ret;
}



