/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sdvbs_common.h"

F2D* fDeepCopyRange(F2D* in, int startRow, int numberRows, int startCol, int numberCols)
{
    int i, j, k;
    F2D *out;
    int rows, cols;
    
    rows = numberRows + startRow;
    cols = numberCols + startCol;
    out = fMallocHandle(numberRows, numberCols);
    
    k = 0;
    for(i=startRow; i<rows; i++)
        for(j=startCol; j<cols; j++)
            asubsref(out,k++) = subsref(in,i,j);
    
    return out;
    
}
