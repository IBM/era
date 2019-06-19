/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include "svm.h"

int main(int argc, char* argv[])
{
    int iter, N, Ntst, i, j, k, n;
    F2D* trn1, *tst1, *trn2, *tst2, *Yoffset;
    alphaRet* alpha;
    F2D *a_result, *result;
    F2D *s;
    F2D *b_result;
    F2D *Xtst, *Ytst;
    unsigned int* start, *stop, *elapsed;
    char im1[256];
    int dim = 256;

    N = 100;
    Ntst = 100;
    iter = 10;

    #ifdef test
    N = 4;
    Ntst = 4;
    iter = 2;
    #endif
    
    #ifdef sim_fast
    N = 20;
    Ntst = 20;
    iter = 2;
    #endif

    #ifdef sim
    N = 16;
    Ntst = 16;
    iter = 8;
    #endif
    
    #ifdef sqcif
    N = 60;
    Ntst = 60;
    iter = 6;
    #endif
    
    #ifdef qcif
    N = 72;
    Ntst = 72;
    iter = 8;
    #endif
    
    #ifdef vga
    N = 450;
    Ntst = 450;
    iter = 15;
    #endif
    
    #ifdef wuxga
    N = 1000;
    Ntst = 1000;
    iter = 20;
    #endif
    printf("Input size\t\t- (%dx%dx%d)\n", N, Ntst, iter);

    if(argc < 2) 
    {
        printf("We need input image path\n");
        return -1;
    }

    sprintf(im1, "%s/d16trn_1.txt", argv[1]);
    trn1 = readFile(im1);   

    sprintf(im1, "%s/d16trn_2.txt", argv[1]);
    trn2 = readFile(im1);   

    sprintf(im1, "%s/d16tst_1.txt", argv[1]);
    tst1 = readFile(im1);   

    sprintf(im1, "%s/d16tst_2.txt", argv[1]);
    tst2 = readFile(im1);       

    /** Start timing **/
    start = photonStartTiming();

    alpha = getAlphaFromTrainSet(N, trn1, trn2, iter);
    a_result = alpha->a_result;
    b_result = alpha->b_result;
    Yoffset = fSetArray(iter, N, 0);

    Xtst = usps_read_partial(tst1, tst2, -1, 1, Ntst/iter, iter);
    Ytst = usps_read_partial(tst1, tst2, -1, 0, Ntst/iter, iter);

    for(i=0; i<iter; i++)
    {
        F2D *temp;
        temp = usps_read_partial(trn1, trn2, i, 0, N/iter, iter);
        for(j=0; j<N; j++)
            subsref(Yoffset,i,j) = asubsref(temp,j);
        fFreeHandle(temp);
    }


    result = fSetArray(Ntst,1,0);
    for( n=0; n<Ntst; n++)
    {
        float maxs=0;
        s=fSetArray(iter,1,0);
        for( i=0; i<iter; i++)
        {
            for (j=0; j<N; j++)
            {
                if (subsref(a_result,i,j) > 0)
                {
                    F2D *Xtemp, *XtstTemp, *X;
                    X = alpha->X;
                    Xtemp = fDeepCopyRange(X,j,1,0,X->width);
                    XtstTemp = fDeepCopyRange(Xtst, n,1,0,Xtst->width);
                    asubsref(s,i) = asubsref(s,i) + subsref(a_result,i,j) * subsref(Yoffset,i,j) * polynomial(3,Xtemp,XtstTemp, dim);
                    fFreeHandle(Xtemp); 
                    fFreeHandle(XtstTemp); 
                }
            }
            asubsref(s,i) = asubsref(s,i) - asubsref(b_result,i);
            if( asubsref(s,i) > maxs)
                maxs = asubsref(s,i);
        }
        
        fFreeHandle(s);
        asubsref(result,n) = maxs;
    }

    /** Timing utils */
    stop = photonEndTiming();

#ifdef CHECK   
    /** Self checking - use expected.txt from data directory  **/
    {
        int ret=0;
        float tol = 0.5;
#ifdef GENERATE_OUTPUT
        fWriteMatrix(result, argv[1]);
#endif
        ret = fSelfCheck(result, argv[1], tol);
        if (ret == -1)
            printf("Error in SVM\n");
    }
    /** Self checking done **/
#endif
        
    fFreeHandle(trn1);
    fFreeHandle(tst1);
    fFreeHandle(trn2);
    fFreeHandle(tst2);
    fFreeHandle(Yoffset);
    fFreeHandle(result);
    fFreeHandle(alpha->a_result);
    fFreeHandle(alpha->b_result);
    fFreeHandle(alpha->X);
    free(alpha);
    fFreeHandle(Xtst);
    fFreeHandle(Ytst);
    elapsed = photonReportTiming(start, stop);
    photonPrintTiming(elapsed);
    free(start);
    free(stop);
    free(elapsed);

    return 0;
}



