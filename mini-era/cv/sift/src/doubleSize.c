/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sift.h"

F2D* doubleSize(F2D* I)
{
    F2D *J;
    int M, N, i, j;

    M = I->height;
    N = I->width;
    J = fSetArray(2*M, 2*N, 0);

    for(i=0; i<M; i++)
    {
        for(j=0; j<N; j++)
        {
            subsref(J,2*i,j*2) = subsref(I,i,j);
        }
    }
    
    for(i=0; i<M-1; i++)
    {
        for(j=0; j<N-1; j++)
        {
            subsref(J,i*2+1,j*2+1) = (0.25 * (subsref(I,i,j) + subsref(I,i+1,j) + subsref(I,i,j+1) + subsref(I,(i+1),j+1)));
        }
    }

    for(i=0; i<M-1; i++)
    {
        for(j=0; j<N; j++)
        {
            subsref(J,i*2+1,j*2) = (0.5 * (subsref(I,i,j) + subsref(I,i+1,j)));
        }
    }
   
    for(i=0; i<M; i++)
    {
        for(j=0; j<N-1; j++)
        {
            subsref(J,i*2,j*2+1) = (0.5 * (subsref(I,i,j) + subsref(I,i,j+1)));
        }
    }
    
    return J;
}






