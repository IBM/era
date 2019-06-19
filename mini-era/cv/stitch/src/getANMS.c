/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "stitch.h"

F2D* getANMS (F2D *points, int r)
{
  unsigned int MAX_LIMIT = 10000000;
  F2D *suppressR;
  float C_ROBUST = 0.9;
  F2D *srtdPnts;
  int n, k;
  I2D *srtdVIdx, *supId;
  float r_sq, t, t1;
  F2D *tempF, *srtdV, *interestPnts;
  int i, j, validCount=0, cnt, end;
  F2D *v;
    int iter, rows, cols;
    F2D* temp;
    int supIdPtr = 0;

  v = fMallocHandle(points->height, 1);
  for(i=0; i<v->height; i++)
      asubsref(v,i) = subsref(points,i,2);

  r_sq = r * r * 1.0;
  n = v->height;

    srtdVIdx = fSortIndices (v, 1);
    srtdPnts = fMallocHandle (srtdVIdx->height, points->width);

    for (i = 0; i < srtdVIdx->height; i++)
        for(j=0; j<points->width; j++)
            subsref(srtdPnts,i,j) = subsref(points, asubsref(srtdVIdx,i), j);

    temp = fSetArray (1, 3, 0);
    suppressR = fSetArray(n, 1, MAX_LIMIT);

    validCount = 0;
    iter = 0;
    for (i = 0; i < suppressR->height; i++)
    {
	    if ( asubsref(suppressR,i) > r_sq)
        {
            validCount++;
        }
    }
   
    k = 0;
    supId = iMallocHandle(validCount, 1);
    for (i = 0; i < (suppressR->height*suppressR->width); i++)
    {
        if ( asubsref(suppressR,i) > r_sq)
        {
            asubsref(supId,k++) = i;
        }
    }
         
    while (validCount > 0)
    {
        F2D *tempp, *temps;
        asubsref(temp,0) = subsref(srtdPnts, asubsref(supId,0), 0);
        asubsref(temp,1) = subsref(srtdPnts, asubsref(supId,0), 1);
        asubsref(temp,2) = subsref(srtdPnts, asubsref(supId,0), 2);
       
        if(iter == 0)
            interestPnts = fDeepCopy(temp);
        else
        {
            tempp = fDeepCopy(interestPnts);
            fFreeHandle(interestPnts); 
            interestPnts = ffVertcat(tempp, temp);
            fFreeHandle(tempp);
        }
        iter++;

        tempp = fDeepCopy(srtdPnts);
        temps = fDeepCopy(suppressR);

        fFreeHandle(srtdPnts);
        fFreeHandle(suppressR);

        srtdPnts = fMallocHandle(supId->height-1, 3);
        suppressR = fMallocHandle(supId->height-1, 1);
        
        k=0;
        for(i=1; i<supId->height; i++)
        {
            subsref(srtdPnts,k,0) = subsref(tempp, asubsref(supId,i) ,0);
            subsref(srtdPnts,k,1) = subsref(tempp, asubsref(supId,i) ,1);
            subsref(srtdPnts,k,2) = subsref(tempp, asubsref(supId,i) ,2);
            subsref(suppressR,k,0) = subsref(temps, asubsref(supId,i) ,0);
            k++;
        }
         
        fFreeHandle(tempp);
        fFreeHandle(temps);
        rows = interestPnts->height-1;
        cols = interestPnts->width;
        for (i = 0; i < srtdPnts->height; i++)
	    {
    	    t = 0;
	        t1 = 0;

	        if ((C_ROBUST * subsref(interestPnts,rows,2)) >= subsref(srtdPnts, i,2))
	        {
        		t = subsref(srtdPnts, i,0) - subsref(interestPnts,rows,0);
        		t1 = subsref(srtdPnts, i,1) - subsref(interestPnts,rows,1);
        		t = t * t + t1 * t1;
                t1 = 0;
            }

	        if ((C_ROBUST * subsref(interestPnts,rows,2)) < subsref(srtdPnts, i,2))
    	        t1 = 1 * MAX_LIMIT;

	        if ( asubsref(suppressR, i) > (t + t1))
            {
	            asubsref(suppressR, i) = t + t1;
            }  
        }
        
        validCount=0;
        for (i = 0; i < suppressR->height; i++)
	        if ( asubsref(suppressR,i) > r_sq)
                validCount++;
   
        k = 0;
        iFreeHandle(supId);
        supId = iMallocHandle(validCount, 1);
        
        for (i = 0; i < suppressR->height*suppressR->width; i++)
            if ( asubsref(suppressR,i) > r_sq)
                asubsref(supId,k++) = i;
    }
  
    iFreeHandle(supId);
    iFreeHandle(srtdVIdx);
    fFreeHandle(srtdPnts);
    fFreeHandle(temp);
    fFreeHandle(suppressR);
    fFreeHandle(v);

    return interestPnts;
}
