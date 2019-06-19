/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "svm.h"

alphaRet* getAlphaFromTrainSet(int N, F2D* trn1, F2D* trn2, int iterations)
{
    float tolerance, C, eps, *b;
    F2D *a_result, *b_result;
    int NumChanged, r, ExamineAll, cnt, d, dim, ret, iter, i;
    F2D *X, *Y;
    F2D *a, *e;

    b = malloc(sizeof(float));
    alphaRet* alpha;
    alpha = (alphaRet*)malloc(sizeof(alphaRet));
    tolerance = 0.001;
    C = 0.05;
    d = -1;
    dim = 256;
    eps = 0.001;
    a_result = fSetArray(iterations, N, 0);
    b_result = fSetArray(iterations, 1, 0);
    ret = 0;
    
    X = usps_read_partial( trn1, trn2, 0, 1, (N/iterations), iterations);
    
    for(iter=0; iter<iterations; iter++)
    {
        Y = usps_read_partial( trn1, trn2, iter, 0, N/iterations, iterations);
        
        a = fSetArray(N, 1, 0);
        arrayref(b,0) = 0;                  /** check if ptr **/
        e = fSetArray(N, 1, 0);
        ExamineAll = 1;
        cnt = 0;
        NumChanged = 0;

        while(NumChanged>0 || ExamineAll == 1)
        {
            cnt = cnt + 1;
            NumChanged = 0;
            if(ExamineAll == 1)
            {
                for(i=0; i<N; i++)
                {
                    ret = examineExample(i, a, b, C, e, X, Y, tolerance, N, eps, dim);
                    NumChanged = NumChanged + ret;
                }
            }
            else
            {
                for(i=0; i<N; i++)
                {
                    if( asubsref(a,i) > 0 && asubsref(a,i) <C )
                    {
                        ret = examineExample(i, a, b, C, e, X, Y, tolerance, N, eps, dim);
                        NumChanged = NumChanged + ret;
                    }
                }
            }
            if(ExamineAll == 1)
                ExamineAll = 0;
            else if(NumChanged == 0)
                ExamineAll = 1;
        }

        for(r=0; r<N; r++)
            subsref(a_result,iter,r) = asubsref(a,r);   /** a_result has size iteration,N .. Check **/
        asubsref(b_result,iter) = arrayref(b,0);

        fFreeHandle(Y);
        fFreeHandle(e);
        fFreeHandle(a);
    }
  
    alpha->C = C;
    alpha->d = d;
    alpha->dim = dim;
    alpha->eps = eps;
    alpha->a_result = a_result;
    alpha->b_result = b_result;
    alpha->a = a;
    alpha->b = arrayref(b,0);
    alpha->X = X;
    alpha->tolerance = tolerance;
    alpha->ret;
    
    free(b);
   
    return alpha; 

}




