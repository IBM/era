#include "sift.h"

/**
    SIFTLOCALMAX  Find local maximizers
   Returns the indexes of the local maximizers of
   the 3-dimensional array F.

    Say, we have a Q-dimensional array F.
   A local maximizer is an element whose value is greater than the
   value of all its neighbors.  The neighbors of an element i1...iQ
   are the subscripts j1...jQ such that iq-1 <= jq <= iq (excluding
   i1...iQ itself).  For example, if Q=1 the neighbors of an element
   are its predecessor and successor in the linear order; if Q=2, its
   neighbors are the elements immediately to its north, south, west,
   est, north-west, north-est, south-west and south-est
   (8-neighborhood).

   Points on the boundary of F are ignored (and never selected as
   local maximizers).

   SEL=SIFTLOCALMAX(F,THRESH) accepts an element as a mazimizer only
   if it is at least THRES greater than all its neighbors.

   SEL=SIFTLOCALMAX(F,THRESH,P) look for neighbors only in the first
   P dimensions of the Q-dimensional array F. This is useful to
   process F in ``slices''.

   REMARK.  Matrices (2-array) with a singleton dimension are
   interpreted as vectors (1-array). So for example SIFTLOCALMAX([0 1
   0]) and SIFTLOCALMAX([0 1 0]') both return 2 as an aswer. However,
   if [0 1 0] is to be interpreted as a 1x2 matrix, then the correct
   answer is the empty set, as all elements are on the boundary.
   Unfortunately MATLAB does not distinguish between vectors and
   2-matrices with a singleton dimension.  To forece the
   interpretation of all matrices as 2-arrays, use
   SIFTLOCALMAX(F,TRESH,2) (but note that in this case the result is
   always empty!).
**/

#define NDIMS 3

F2D* siftlocalmax(F2D* in, float thresh, int intervals, int M, int N)
{  
  int ndims, ptoffset=0, maxIter = 0; 
  int offsets[NDIMS];
  int midx[NDIMS];
  int dims[NDIMS];
  I2D* neighbors ;
  int nneighbors ;
  F2D* out;
    
 /**  
    We pass dogss[o], which has >1 intervals
    If >1 intervals, then the dimensions of in[F] will be 1 x intervals
    If =1 interval, then the dimensions of in[F] will be the size of the dogss[o] image
    We will check for this condition.
 **/
    
    ndims = NDIMS;      /* Since we have intervals number of images of size M x N */
	dims[0] = M;
	dims[1] = N;
	dims[2] = intervals-1;

  /* 
     If there are only two dimensions and if one is singleton, then
     assume that a vector has been provided as input (and treat this
     as a COLUMN matrix with p=1). We do this because Matlab does not
     distinguish between vectors and 1xN or Mx1 matrices and because
     the cases 1xN and Mx1 are trivial (the result is alway empty).
  */
  
  
 
  /* ------------------------------------------------------------------
   *                                                         Do the job
   * --------------------------------------------------------------- */  
    int maxima_size = M*N ;

    I2D* maxima_start = iMallocHandle(1, maxima_size);
    int* maxima_iterator = maxima_start->data ;
    int* maxima_end = maxima_start->data + maxima_size ;

    int i,h,o,j; 
    F2D* pt;  
    
    pt = in;

    /* Compute the offsets between dimensions. */
    offsets[0] = 1 ;
    for(i = 1 ; i < ndims ; ++i)
    {
      offsets[i] = offsets[i-1]*dims[i-1] ;
    }

    /* Multi-index. */
    for(i = 0 ; i < ndims ; ++i)
      midx[i] = 1 ;

    /* Neighbors. */
    nneighbors = 1 ;
    o=0 ;
    for(i = 0 ; i < ndims ; ++i) 
    {
      nneighbors *= 3 ;
      midx[i] = -1 ;
      o -= offsets[i] ;
    }
    nneighbors -= 1 ;
    neighbors = iMallocHandle(1,nneighbors);

    /* Precompute offsets from offset(-1,...,-1,0,...0) to
     * offset(+1,...,+1,0,...,0). */
    i = 0 ;

    while(1) 
    {
      if(o != 0)
      {
        asubsref(neighbors, i) = o ;
        i++;
      }
      h = 0 ;
      while( o += offsets[h], (++midx[h]) > 1 ) 
      {
        o -= 3*offsets[h] ;
        midx[h] = -1 ;
        if(++h >= ndims)
          goto stop ;
      }
    }

  stop: ;

    /* Starts at the corner (1,1,...,1,0,0,...0) */
    for(h = 0 ; h < ndims ; ++h) 
    {
      midx[h] = 1 ;
      ptoffset += offsets[h] ;
    }


    /* ---------------------------------------------------------------
     *                                                            Loop
     * ------------------------------------------------------------ */

    /*
      If any dimension in the first P is less than 3 elements wide
      then just return the empty matrix (if we proceed without doing
      anything we break the carry reporting algorithm below).
    */

    for(h=0 ; h < ndims ; ++h)
      if(dims[h] < 3) 
		goto end ;
   
    while(1) 
    {
      /* Propagate carry along multi index midx */
      
      h = 0 ;
      while(midx[h] >= dims[h] - 1) 
      {
        midx[h] = 1 ;
        if(++h >= ndims) 
          goto next_layer ;
        ++midx[h] ;
      }
      
      /*  Scan neighbors */
      {
        float v = asubsref(pt, ptoffset);  //*pt ;
        int is_greater = (v>=thresh) ? 1 : 0;
        
		assert(ptoffset < pt->width*pt->height);

        i = 0  ;
        while(is_greater && i < nneighbors)
        {
            if(v > asubsref(pt, ptoffset + asubsref(neighbors,i)))
                is_greater *= 1;
            else
                is_greater *= 0;
            i = i+1;
        }
        
        /* Add the local maximum */
        if(is_greater) 
        {
          /* Need more space? */
          if( &(maxima_iterator[maxIter])  == maxima_end) 
          {
              int *temp, i, j;
              maxima_size += M*N ;
            
            free(maxima_start); 
            maxima_start = iMallocHandle(1, maxima_size);

            maxima_end = maxima_start->data + maxima_size ;
            maxima_iterator = maxima_end - M*N ;
            maxIter = 0;
          }
          
          maxima_iterator[maxIter] = ptoffset + 1;
          maxIter++;
        }
        
        /* Go to next element */
        ptoffset += 1 ;
        ++midx[0] ;
        continue ;
        
      next_layer: ;
        if( h >= ndims )
          goto end ;
        
        while((++midx[h]) >= dims[h]) 
        {
          midx[h] = 0 ;
          if(++h >= ndims)
            goto end ;
        }
      }
    }   
     
  end:;
    /* Return. */
    {
        int i=0;
        out = fMallocHandle(1, maxIter);
      
        for(i=0; i<maxIter; i++)
            asubsref(out,i) = maxima_iterator[i] ;
    }
    
    /* Release space. */
    free(neighbors) ;
    free(maxima_start);
  
  return out;
}







