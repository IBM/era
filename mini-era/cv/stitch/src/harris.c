/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"

F2D* harris(I2D* im)
{
    F2D *img1;
    F2D *g1, *g2, *g;
    F2D *Ix, *Iy;
    F2D *Ix2, *Iy2, *IxIy;
    F2D *v, *R, *Rmax, *Rnm;
    float eps;
    F2D *sobel, *sob, *temp, *temp1;    
    I2D *win, *x, *y;
    int i;
    
    g1 = fSetArray(5,5,0);
    g2 = fSetArray(3,3,0);

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

    asubsref(g2,0) = 1;
    asubsref(g2,1) = 2;
    asubsref(g2,2) = 1;
    
    asubsref(g2,3) = 2;
    asubsref(g2,4) = 4;
    asubsref(g2,5) = 2;

    asubsref(g2,6) = 1;
    asubsref(g2,7) = 2;
    asubsref(g2,8) = 1;
    
    g = fDivide(g1, 256);
    sob = fMallocHandle(1,3);
    asubsref(sob,0) = -0.5;
    asubsref(sob,1) = 0;
    asubsref(sob,2) = 0.5;

    {
        F2D* imf;
        imf = fiDeepCopy(im);
        img1 = ffConv2(imf, g);
        fFreeHandle(imf);
    }

    Ix = ffConv2(img1, sob); 
    fFreeHandle(sob);
    sob = fMallocHandle(3,1);
    asubsref(sob,0) = -0.5;
    asubsref(sob,1) = 0;
    asubsref(sob,2) = 0.5;
    Iy = ffConv2(img1, sob);

    fFreeHandle(g);
    g = fDivide(g2, 16);
    eps = 2.2204e-16;
    sobel = fTimes(Ix, Ix);
    Ix2 = ffConv2(sobel, g);
    fFreeHandle(sobel);

    sobel = fTimes(Iy, Iy);
    Iy2 = ffConv2(sobel, g);
    fFreeHandle(sobel);

    sobel = fTimes(Ix, Iy);
    IxIy = ffConv2(sobel, g);
    fFreeHandle(sobel);
 
    temp = fTimes(Ix2, Iy2);
    temp1 = fTimes(IxIy, IxIy);
    sobel = fMinus(temp, temp1);

    fFreeHandle(temp);
    temp = fPlus(Ix2, Iy2);

    for(i=0; i<(temp->height*temp->width); i++)
        asubsref(temp,i) += eps;

    R = ffDivide(sobel, temp);
    
    win = iSetArray(1,2,3);
    Rmax = maxWindow(R, win);
    Rnm = supress(R, Rmax);

    v = fFind3(Rnm);
    
    iFreeHandle(win);
    fFreeHandle(Rmax);
    fFreeHandle(Rnm);
    fFreeHandle(R);
    
    fFreeHandle(img1);
    fFreeHandle(g1);
    fFreeHandle(g2);
    fFreeHandle(g);
    fFreeHandle(Ix);
    fFreeHandle(Iy);
    fFreeHandle(Ix2);
    fFreeHandle(Iy2);
    fFreeHandle(IxIy);
    fFreeHandle(sobel);
    fFreeHandle(sob);
    fFreeHandle(temp);
    fFreeHandle(temp1);
//    iFreeHandle(x);
//    iFreeHandle(y);
    
    return v;
}

