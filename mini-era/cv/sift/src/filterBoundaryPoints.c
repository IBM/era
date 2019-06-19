#include "sift.h"

/**
    Filter out points on the boundaries. 
    The function returns oframes, which
    contains the row, col and the interval number 
**/

F2D* filterBoundaryPoints(int M, int N, F2D* oframes) 
{
    int i, k=0, m=0;
	int cnt;
    I2D* sel;
    F2D *ret;
    
    cnt = 0;
    for(i=0; i<oframes->width; i++)
	{
        if(asubsref(oframes,i)>3 && asubsref(oframes,i)<(N-3) && 
			subsref(oframes,1,i)>3 && subsref(oframes,1,i)<(M-3))
		{
            cnt++;
		}
	}

    sel = iSetArray(cnt, 1, 0);
    for(i=0; i<oframes->width; i++)
    {
        if(asubsref(oframes,i)>3 && asubsref(oframes,i)<(N-3) && 
			subsref(oframes,1,i)>3 && subsref(oframes,1,i)<(M-3))
        {
            asubsref(sel,k) = i;
            k++;
        }
        m++;
    }
   
    if( sel->height > 0)
    {
        ret = fSetArray(oframes->height, sel->height, 0);
        {
            for(i=0; i<sel->height; i++)
            {
                subsref(ret,0,i) = subsref(oframes,0,asubsref(sel,i));
                subsref(ret,1,i) = subsref(oframes,1,asubsref(sel,i));
                subsref(ret,2,i) = subsref(oframes,2,asubsref(sel,i));
            }
        }
    }
    else
        ret = fSetArray(1,1,0);

    iFreeHandle(sel);
    return ret;
}






