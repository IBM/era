/********************************
Author: Sravanthi Kota Venkata
********************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sdvbs_common.h"

int selfCheck(I2D* in1, char* path, int tol)
{
    int r1, c1, ret=1;
    FILE* fd;
    int count=0, *buffer, i, j;
    char file[100];
    int* data = in1->data;
    
    r1 = in1->height;
    c1 = in1->width;

    buffer = (int*)malloc(sizeof(int)*r1*c1);

    sprintf(file, "%s/expected_C.txt", path);
    fd = fopen(file, "r");
    if(fd == NULL)
    {
        printf("Error: Expected file not opened \n");
        return -1;
    }
   
    while(!feof(fd))
    {
        fscanf(fd, "%d", &buffer[count]);
        count++;
    }
    count--;

    if(count < (r1*c1))
    {
        printf("Checking error: dimensions mismatch. Expected = %d, Observed = %d \n", count, (r1*c1));
        return -1;
    }
    
    for(i=0; i<r1*c1; i++)
    {
        if((abs(data[i])-abs(buffer[i]))>tol || (abs(buffer[i])-abs(data[i]))>tol)
        {
            printf("Checking error: Values mismtach at %d element\n", i);
            printf("Expected value = %d, observed = %d\n", buffer[i], data[i] );
            return -1;
        }
    }
   
    fclose(fd);
    free(buffer); 
    printf("Verification\t\t- Successful\n");
    return ret;
}







