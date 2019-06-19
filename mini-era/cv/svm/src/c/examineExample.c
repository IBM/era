/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "svm.h"

int examineExample(int i, F2D* a, float* b, float C, F2D* e, F2D* X, F2D* Y, float tolerance, int N, float eps, int dim)
{
    int  ret, j, k, m, n;
    float E, r1, randVal;
    float maxDiff, temp;

    if( ( asubsref(a,i) > 0) && ( asubsref(a,i) <C) )
        E = asubsref(e,i);
    else
        E = cal_learned_func(i, a, b, N, Y, X, dim) - asubsref(Y,i);

    r1 = subsref(Y,i,0) * E;
    if( ((r1 < (-1*tolerance)) && ( asubsref(a,i) < C)) || ((r1 >tolerance) && ( asubsref(a,i) > 0)) )
    {
        maxDiff = 0;
        j = i;

        for(k=0; k<N; k++)
        {
            if( ( asubsref(a,k) > 0) && ( asubsref(a,k) < C) )
            {
                temp = fabsf( E - asubsref(e,k));
                if (temp > maxDiff)
                    j = k;
            }
        }

        if ( i!=j)
        {
            ret = takeStep(i, j, a, C, e, Y, X, eps, b, N, dim);
            if(ret == 1)
                return ret;
        }

        randVal = 1.0;
        for( k= (randVal*(N-2)); k<N; k++)
        {
            if( ( asubsref(a,k) > 0) && ( asubsref(a,k) <C) )
            {
                ret = takeStep(i, k, a, C, e, Y, X, eps, b, N, dim);
                if (ret == 1)
                    return ret;
            }
        }

        for(k=0; k<N; k++)
        {
            ret = takeStep(i, k, a, C, e, Y, X, eps, b, N, dim);
            if(ret == 1)
                return ret;
        }
    }

    
    ret = 0;
    return ret;
}




