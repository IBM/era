/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "svm.h"

float cal_learned_func(int k, F2D* a, float* b, int N, F2D* Y, F2D* X, int dim)
{
    float s, ret;
    int i, j, m, n;
    F2D *temp, *temp1;

    s=0;
    for(i=0; i<N; i++)
    {
        if( subsref(a,i,0) > 0)
        {
            temp = fMallocHandle(1, X->width);
            temp1 = fMallocHandle(1, X->width);

            for(m=0; m<X->width; m++)
            {
                asubsref(temp,m) = subsref(X,i,m);
                asubsref(temp1,m) = subsref(X,k,m);
            }

            s += asubsref(a,i) * asubsref(Y,i) * polynomial(3, temp, temp1, dim);

            free(temp);
            free(temp1);
        }
    }

    s = s- arrayref(b,0);
    ret = s;

    return ret;
}




