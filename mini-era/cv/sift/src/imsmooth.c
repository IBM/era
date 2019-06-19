/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sift.h"
#include <math.h>
#include <assert.h>
const double win_factor = 1.5 ;
const int nbins = 36 ;
const float threshold = 0.01;

/**
    This function is similar to imageBlur in common/c folder.
    Here, we can specify the sigma value for the gaussian filter
    function.
**/

void imsmooth(F2D* array, float dsigma, F2D* out)
{
  int M,N ;
  int i,j,k;
  float s ;

  /* ------------------------------------------------------------------
  **                                                Check the arguments
  ** --------------------------------------------------------------- */ 

    M = array->height;
    N = array->width;
    s = dsigma;
    
  /* ------------------------------------------------------------------
  **                                                         Do the job
  ** --------------------------------------------------------------- */ 
  if(s > threshold) 
  {
    int W = (int) ceil(4*s) ;
    float temp[2*W+1];
    F2D* buffer;
    float acc = 0.0;
   
    buffer = fSetArray(M,N,0);
    
    for(j = 0 ; j < (2*W+1) ; ++j) 
    {
      temp[j] = (float)(expf(-0.5 * (j - W)*(j - W)/(s*s))) ;
      acc += temp[j];
    }

    for(j = 0 ; j < (2*W+1) ; ++j) 
    {
      temp[j] /= acc ;
    }
    
    /*
    ** Convolve along the columns
    **/

    for(j = 0 ; j < M ; ++j) 
    {
      for(i = 0 ; i < N ; ++i) 
      {
        int startCol = MAX(i-W,0);
        int endCol = MIN(i+W, N-1);
        int filterStart = MAX(0, W-i);

		assert(j < array->height);
		assert(j < buffer->height);
		assert(i < buffer->width);
        for(k=startCol; k<=endCol; k++) {
			assert(k < array->width);
			assert(filterStart < 2*W+1);
            subsref(buffer,j,i) += subsref(array, j, k) * temp[filterStart++];
		}
      }
    }
 
    /*
    ** Convolve along the rows
    **/
    for(j = 0 ; j < M ; ++j) 
    {
      for(i = 0 ; i < N ; ++i) 
      {
        int startRow = MAX(j-W,0);
        int endRow = MIN(j+W, M-1);
        int filterStart = MAX(0, W-j);
        for(k=startRow; k<=endRow; k++)
            subsref(out,j,i) += subsref(buffer,k,i) * temp[filterStart++];
      }
    }  
  
    fFreeHandle(buffer);
      
  } 
  else 
  {
      for(i=0;i<M*N;i++)
          asubsref(out, i) = asubsref(array, i);
  }


  return;
}
