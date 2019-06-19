/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "svm.h"

int takeStep(int i, int j, F2D* a, float C, F2D* e, F2D* Y, F2D* X, float eps, float* b, int N, int dim)
{
    int ret=1;
    float s;
    int m, n, k;
    float Ei, Ej, gamma, L, H;
    F2D *a_old;
    float k11, k12, k22, eta;
    F2D *temp, *temp1, *temp2;
    float t, t1, t2;
    float bnew, delta_b;
    float c1, c2, Lobj, Hobj;
    
    if(i==j)
        return 0;
        
    a_old = fDeepCopy(a);
    
    if( asubsref(a_old,i)>0 && asubsref(a_old,i)<C ) 
        Ei = asubsref(e,i);
    else
        Ei = cal_learned_func(i, a, b, N, Y, X, dim) - asubsref(Y,i);

    if( asubsref(a_old,j)>0 && asubsref(a_old,j)<C ) 
        Ej = asubsref(e,j);
    else
        Ej = cal_learned_func(j, a, b, N, Y, X, dim) - asubsref(Y,j);

    s = asubsref(Y,i) * asubsref(Y,j);

    if( asubsref(Y,i) == asubsref(Y,j) )
    {
        gamma = asubsref(a_old,i) + asubsref(a_old,j);
        if(gamma > C)
        {
            L = gamma - C;
            H = C;
        }
        else
        {
            L = 0;
            H = gamma;
        }
    }
    else
    {
        gamma = asubsref(a_old,i) - asubsref(a_old,j);
        if(gamma > 0)
        {
            L = 0;
            H = C - gamma;
        }
        else
        {
            L = -gamma;
            H = C;
        }
    }
    

    if(L==H)
    {
        fFreeHandle(a_old);
        return 0;
    }
           
    temp = fMallocHandle(1, X->width);
    temp1 = fMallocHandle(1, X->width);

    for(m=0; m<X->width; m++)
    {
        asubsref(temp,m) = subsref(X,i,m);
        asubsref(temp1,m) = subsref(X,j,m);
    }

    k11 = polynomial(3, temp, temp, dim);
    k12 = polynomial(3, temp, temp1, dim);
    k22 = polynomial(3, temp1, temp1, dim);
    eta = 2 * k12 - k11 - k22;

    fFreeHandle(temp1);
    fFreeHandle(temp);

    if(eta<0)
    {
        asubsref(a,j) = asubsref(a_old,j) + asubsref(Y,j) * (Ej-Ei)/eta;
        if( asubsref(a,j) < L)
            asubsref(a,j) = L;
        else if( asubsref(a,j) > H )
            asubsref(a,j) = H;
    }
    else
    {
        c1 = eta/2;
        c2 = asubsref(Y,j) * (Ei-Ej) - eta * asubsref(a_old,j);
        Lobj = c1 * L * L + c2 * L;
        Hobj = c1 * H * H + c2 * H;

        if (Lobj > (Hobj+eps))
            asubsref(a,j) = L;
        else if (Lobj < (Hobj-eps))
            asubsref(a,j) = H;
        else
            asubsref(a,j) = asubsref(a_old,j);
    }
    
    if( fabsf( asubsref(a,j)- asubsref(a_old,j) ) < (eps* (asubsref(a,j) + asubsref(a_old,j) +eps)) )
    {
        fFreeHandle(a_old);
        return 0;
    }

    asubsref(a,i) = asubsref(a_old,i) - s * ( asubsref(a,j) - asubsref(a_old,j) );

    if( asubsref(a,i) < 0)
    {
        asubsref(a,j) = asubsref(a,j) + s * asubsref(a,i);
        asubsref(a,i) = 0;
    }
    else if (asubsref(a,i) > C)
    {
        t = asubsref(a,i) - C;
        asubsref(a,j) = asubsref(a,j) + s * t;
        asubsref(a,i) = C;
    }

    /** Update threshold to react change in Lagrange multipliers **/

    if( asubsref(a,i) > 0 && asubsref(a,i) < C )
        bnew = arrayref(b,0) + Ei + asubsref(Y,i) * (asubsref(a,i) - asubsref(a_old,i)) * k11 + asubsref(Y,j) * (asubsref(a,j) - asubsref(a_old,j)) * k12;
    else
    {
        if( asubsref(a,j) > 0 && asubsref(a,j) < C )
            bnew = arrayref(b,0) + Ej + asubsref(Y,i) * (asubsref(a,i) - asubsref(a_old,i)) * k12 + asubsref(Y,j) * (asubsref(a,j) - asubsref(a_old,j)) * k22;
        else
        {
            float b1, b2;
            b1 = arrayref(b,0) + Ei + asubsref(Y,i) * (asubsref(a,i) - asubsref(a_old,i)) * k11 + asubsref(Y,j) * (asubsref(a,j) - asubsref(a_old,j)) * k12;
            b2 = arrayref(b,0) + Ej + asubsref(Y,i) * (asubsref(a,i) - asubsref(a_old,i)) * k12 + asubsref(Y,j) * (asubsref(a,j) - asubsref(a_old,j)) * k22;
            bnew = (b1 + b2) / 2;
        }
    }
    delta_b = bnew - arrayref(b,0);
    arrayref(b,0) = bnew;

    /** Update error cache using new Lagrange multipliers 24ai **/

    t1 = asubsref(Y,i) * (asubsref(a,i)-asubsref(a_old,i));
    t2 = asubsref(Y,j) * (asubsref(a,j)-asubsref(a_old,j));

    temp = fMallocHandle(1, X->width);
    temp1 = fMallocHandle(1, X->width);
    temp2 = fMallocHandle(1, X->width);

    for (k=0; k<N; k++)
    {
        if (0 < asubsref(a_old,i) && asubsref(a_old,i) < C )
        {

            for(m=0; m<X->width; m++)
            {
                asubsref(temp,m) = subsref(X,i,m);
                asubsref(temp1,m) = subsref(X,k,m);
                asubsref(temp2,m) = subsref(X,j,m);
            }

            asubsref(e,k) = asubsref(e,k)+t1 * polynomial(3, temp, temp1, dim) + t2 * polynomial(3, temp2, temp1, dim) - delta_b;
            asubsref(e,i) = 0;
            asubsref(e,j) = 0;
        }
    }
    
    fFreeHandle(a_old);
    fFreeHandle(temp);
    fFreeHandle(temp1);
    fFreeHandle(temp2);
    ret = 1;
    return ret;
}



