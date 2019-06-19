/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"

int main(int argc, char* argv[])
{
    int rows, cols;
    F2D *x, *y, *v, *interestPnts, *Fcur, *int1, *int2;
    I2D *Icur;
    int i, j;
    unsigned int* start, *endC, *elapsed;
    char im1[100], im2[100];
    
    if(argc < 2)
    {
        printf("We need input image path\n");
        return -1;
    }

    sprintf(im1, "%s/1.bmp", argv[1]);
    sprintf(im2, "%s/2.bmp", argv[1]);

    Icur = readImage(im1);
    rows = Icur->height;
    cols = Icur->width;

    printf("Input size\t\t- (%dx%d)\n", rows, cols);
    start = photonStartTiming();

    v = harris(Icur);
    interestPnts = getANMS(v, 24);

    int1 = fMallocHandle(interestPnts->height, 1);
    int2 = fSetArray(interestPnts->height, 1, 0);
    
    for(i=0; i<int1->height; i++)
    {
        asubsref(int1,i) = subsref(interestPnts,i,0);
        asubsref(int2,i) = subsref(interestPnts,i,1);
    }

    Fcur = extractFeatures(Icur, int1, int2);  
    
    endC = photonEndTiming();
    elapsed = photonReportTiming(start, endC);

#ifdef CHECK   
    /** Self checking - use expected.txt from data directory  **/
    {
        int ret=0;
        float tol = 0.02;
#ifdef GENERATE_OUTPUT
        fWriteMatrix(Fcur, argv[1]);
#endif
        ret = fSelfCheck(Fcur, argv[1], tol);
        if (ret == -1)
            printf("Error in Stitch\n");
    }
//    /** Self checking done **/
#endif

    iFreeHandle(Icur);
    fFreeHandle(v);
    fFreeHandle(interestPnts);
    fFreeHandle(int1);
    fFreeHandle(int2);
    fFreeHandle(Fcur);
    free(start);
    free(endC);

    photonPrintTiming(elapsed);
    free(elapsed);
    return 0;
}
