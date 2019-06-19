/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"
#include <math.h>

#define min(a,b) (a<b)?a:b

F2D* extractFeatures(I2D* I, F2D* x, F2D* y)
{
    int n, i, j, k;
    F2D* I1;
    F2D *Iconv;
    F2D *Isub;
    int nr, nc;
    F2D *w, *wt, *vecF;
    I2D *Xsub, *Ysub;
    float temp, mean, std;
    int m;
    F2D *g1, *g;

    g1 = fSetArray(5,5,0);

    asubsref(g1,0) = 1;
    asubsref(g1,1) = 4;
    asubsref(g1,2) = 6;
    asubsref(g1,3) = 4;
    asubsref(g1,4) = 1;

    asubsref(g1,5) = 4;
    asubsref(g1,6) = 16;
    asubsref(g1,7) = 24;
    asubsref(g1,8) = 16;
    asubsref(g1,9) = 4;

    asubsref(g1,10) = 6;
    asubsref(g1,11) = 24;
    asubsref(g1,12) = 36;
    asubsref(g1,13) = 24;
    asubsref(g1,14) = 6;

    asubsref(g1,15) = 4;
    asubsref(g1,16) = 16;
    asubsref(g1,17) = 24;
    asubsref(g1,18) = 16;
    asubsref(g1,19) = 4;

    asubsref(g1,20) = 1;
    asubsref(g1,21) = 4;
    asubsref(g1,22) = 6;
    asubsref(g1,23) = 4;
    asubsref(g1,24) = 1;

    g = fDivide(g1, 256);
    n = x->height;

    vecF = fMallocHandle(n, 64);
    I1 = fiDeepCopy(I);

    Iconv = ffConv2(I1, g);
    fFreeHandle(I1);
    I1 = ffConv2(Iconv, g);
    fFreeHandle(Iconv);
    Iconv = fDeepCopy(I1);

    {
        int i = (Iconv->height/5);
        int j = (Iconv->width/5);
        Isub = fMallocHandle(i, j);
    }

    for(i=0, m=0; m<Isub->height; i+=5, m++)
    {
        for(j=0, k=0; k<Isub->width; j+=5, k++)
        {
            subsref(Isub,m,k) = subsref(Iconv,i,j);
        }
    }

    fFreeHandle(Iconv);
    fFreeHandle(g1);
    fFreeHandle(g);
    fFreeHandle(I1);

    nr = Isub->height;
    nc = Isub->width;

    Xsub = iMallocHandle(x->height, x->width);
    Ysub = iMallocHandle(y->height, y->width);
        
//    printf("Sizes = %d\t%d\t%d\t%d\n", Isub->height, Isub->width, x->height, x->width);
   
    for(i=0; i<(x->height*x->width); i++)
    {
        asubsref(Xsub,i) = min( ( asubsref(x,i) /5), nc-4 );
        asubsref(Ysub,i) = min( ( asubsref(y,i) /5), nr-4 );
    }

    {
        int maxX, maxY;
        maxX = Xsub->height>Xsub->width?Xsub->height:Xsub->width;
        maxY = Ysub->height>Ysub->width?Ysub->height:Ysub->width;
        if(maxX < 6 || maxY < 10)
        {
            fFreeHandle(vecF);
            vecF = fSetArray(n,2,0);
            for(i=0; i<(x->height); i++)
            {
                subsref(vecF, i, 0) = asubsref(Xsub,i)*1.0;
                subsref(vecF, i, 1) = asubsref(Ysub,i)*1.0;
            }

            fFreeHandle(Isub); 
            iFreeHandle(Xsub); 
            iFreeHandle(Ysub); 
            return vecF;
        }
    }

    {
        int newSize = 4;
        if(I->height > 32 && I->width >32)
            newSize = 64;
        fFreeHandle(vecF);
        vecF = fMallocHandle(n, newSize);
    }

//    printf("Size of Isub = %d\t%d\n", Isub->height, Isub->width);      
    for(i=0; i<n; i++)
    {
    
        if( asubsref(Ysub,i) < 3)
            asubsref(Ysub,i) = 3;
        if( asubsref(Xsub,i) < 3)
            asubsref(Xsub,i) = 3;
        
        if( asubsref(Ysub,i) >= (Isub->height-4))
            asubsref(Ysub,i) = Isub->height-5;
        if( asubsref(Xsub,i) >= (Isub->width-4))
            asubsref(Xsub,i) = Isub->width-5;

        m = 0;
        temp = 0;
//        printf("SUBS %d\t%d\n", asubsref(Ysub,i), asubsref(Xsub,i));
  
        for(k= asubsref(Xsub,i)-3; k<=( asubsref(Xsub,i)+4); k++)
        {
            for(j= asubsref(Ysub,i)-3; j<=( asubsref(Ysub,i)+4); j++)
            {
//                printf("%d\t%d\n", j, k);
                subsref(vecF,i,m) = subsref(Isub,j,k);
                temp += subsref(vecF,i,m);
                m++;
            }
        }
        mean = temp/64.0;

        std = 0;
        for(j=0; j<64; j++)
        {
            subsref(vecF,i,j) = subsref(vecF,i,j) - mean;
            std += subsref(vecF,i,j) * subsref(vecF,i,j);
        }
         
        std = std/64;
        std = sqrt(std);
        for(j=0; j<64; j++)
        {
            subsref(vecF,i,j) = subsref(vecF,i,j)/std; 
        }
    }

    iFreeHandle(Xsub);
    fFreeHandle(Isub);
    iFreeHandle(Ysub);
    return vecF;
}



