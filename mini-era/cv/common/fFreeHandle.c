/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include "sdvbs_common.h"

void fFreeHandle(F2D* out)
{
    if(out != NULL)
        free(out);

    return;
}

