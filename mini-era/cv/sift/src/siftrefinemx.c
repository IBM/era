#include "sift.h"

const int max_iter = 5 ;

/**

    SIFTREFINEMX  Subpixel localization, thresholding and on-edge test
   Q = SIFTREFINEMX(P, OCTAVE, SMIN) refines, thresholds and performs
   the on-edge test for the SIFT frames P extracted from the DOG
   octave OCTAVE with parameter SMIN (see GAUSSIANSS()).

   Q = SIFTREFINEMX(P, OCTAVE, SMIN, THRESH, R) specifies custom
   values for the local maximum strength threshold THRESH and the
   local maximum peakedeness threshold R.

   OCTAVE is an octave of the Difference Of Gaussian scale space. P
   is a 3xK matrix specifying the indexes (X,Y,S) of the points of
   extremum of the octave OCTAVE. The spatial indexes X,Y are integer
   with base zero. The scale index S is integer with base SMIN and
   represents a scale sublevel in the specified octave.

   The function returns a matrix Q containing the refined keypoints.
   The matrix has the same format as P, except that the indexes are
   now fractional. The function drops the points that do not satisfy
   the strength and peakedness tests.

**/

F2D* siftrefinemx(F2D* oframes, F2D* dogss, int smin, float thresh, int r, int M, int N, int intervals)
{
  int S,K ;
  F2D* out;
	
  /* -----------------------------------------------------------------
  **                                               Check the arguments
  ** -------------------------------------------------------------- */ 
 
  S = intervals;
  K = oframes->height;

  /* If the input array is empty, then output an empty array as well. */
  if( K == 0) {
      out = fDeepCopy(oframes);
    return out;
  }

  /* -----------------------------------------------------------------
   *                                                        Do the job
   * -------------------------------------------------------------- */
  {    
    F2D* buffer = fMallocHandle(K,3);
    int p ;
    const int yo = 1 ;
    const int xo = M ;
    const int so = M*N;
    int buffCounter = 0;
    int pptcount = 0;
    
    for(p = 0 ; p < K ; ++p) 
    {
        float tx, ty, ts;
        int x,y,s;
        int iter ;
        float b[3] ;

        tx = asubsref(oframes, pptcount++);
        ty = asubsref(oframes, pptcount++);
        ts = asubsref(oframes, pptcount++);

        x = ceil(tx);
        y = ceil(ty);
        s = ceil(ts) - smin; 

    
      /* Local maxima extracted from the DOG
       * have coorrinates 1<=x<=N-2, 1<=y<=M-2
       * and 1<=s-mins<=S-2. This is also the range of the points
       * that we can refine.
       */
      
      if(x < 1 || x > N-2 ||
         y < 1 || y > M-2 ||
         s < 1 || s > S-2) {
        continue ;
      }

#define at(dx,dy,ds) asubsref(dogss, (dx)*xo + (dy)*yo + (ds)*so)

      {
        float Dx=0,Dy=0,Ds=0,Dxx=0,Dyy=0,Dss=0,Dxy=0,Dxs=0,Dys=0 ;
        int dx = 0 ;
        int dy = 0 ;
        
        for(iter = 0 ; iter < max_iter ; ++iter) 
        {

          float A[3*3] ; 

#define Aat(i,j) (A[(i)+(j)*3])    

          x += dx ;
          y += dy ;
          
          /* Compute the gradient. */
          Dx = 0.5 * (at(x+1,y+0,s+0) - at(x-1,y+0,s+0)) ;
          Dy = 0.5 * (at(x+0,y+1,s+0) - at(x+0,y-1,s+0));
          Ds = 0.5 * (at(x+0,y+0,s+1) - at(x+0,y+0,s-1)) ;
          
          /* Compute the Hessian. */
          Dxx = (at(x+1,y,s) + at(x-1,y,s) - 2.0 * at(x,y,s)) ;
          Dyy = (at(x,y+1,s) + at(x,y-1,s) - 2.0 * at(x,y,s)) ;
          Dss = (at(x,y,s+1) + at(x,y,s-1) - 2.0 * at(x,y,s)) ;
          
          Dxy = 0.25 * ( at(x+1,y+1,s) + at(x-1,y-1,s) - at(x-1,y+1,s) - at(x+1,y-1,s) ) ;
          Dxs = 0.25 * ( at(x+1,y,s+1) + at(x-1,y,s-1) - at(x-1,y,s+1) - at(x+1,y,s-1) ) ;
          Dys = 0.25 * ( at(x,y+1,s+1) + at(x,y-1,s-1) - at(x,y-1,s+1) - at(x,y+1,s-1) ) ;
          
          /* Solve linear system. */
          Aat(0,0) = Dxx ;
          Aat(1,1) = Dyy ;
          Aat(2,2) = Dss ;
          Aat(0,1) = Aat(1,0) = Dxy ;
          Aat(0,2) = Aat(2,0) = Dxs ;
          Aat(1,2) = Aat(2,1) = Dys ;
          
          b[0] = - Dx ;
          b[1] = - Dy ;
          b[2] = - Ds ;
          
          /* If the translation of the keypoint is big, move the keypoint
           * and re-iterate the computation. Otherwise we are all set.
           */
          dx= ((b[0] >  0.6 && x < N-2) ?  1 : 0 )
            + ((b[0] < -0.6 && x > 1  ) ? -1 : 0 ) ;
          
          dy= ((b[1] >  0.6 && y < M-2) ?  1 : 0 )
            + ((b[1] < -0.6 && y > 1  ) ? -1 : 0 ) ;
          
          if( dx == 0 && dy == 0 ) 
			break ;
          
        }
				
        {
          float val = at(x,y,s) + 0.5 * (Dx * b[0] + Dy * b[1] + Ds * b[2]) ; 
          float score = (Dxx+Dyy)*(Dxx+Dyy) / (Dxx*Dyy - Dxy*Dxy) ; 
          float xn = x + b[0] ;
          float yn = y + b[1] ;
          float sn = s + b[2] ;
        
          if(fabs(val) > thresh &&
             score < (r+1)*(r+1)/r && 
             score >= 0 &&
             fabs(b[0]) < 1.5 &&
             fabs(b[1]) < 1.5 &&
             fabs(b[2]) < 1.5 &&
             xn >= 0 &&
             xn <= N-1 &&
             yn >= 0 &&
             yn <= M-1 &&
             sn >= 0 &&
             sn <= S-1) 
		  {
            asubsref(buffer,buffCounter++) = xn ;
            asubsref(buffer,buffCounter++) = yn ;
            asubsref(buffer,buffCounter++) = sn+smin ;
          }
        }
      }
    }      

    /* Copy the result into an array. */
    {
      int i, j, k=0;
      int NL = buffCounter/3;
      out = fMallocHandle(3, NL);

        for(i=0; i<NL; i++)
        {
            for(j=0; j<3; j++)
            {
                subsref(out,j,i) = asubsref(buffer,k);
                k++;
            }
        }
    }
    free(buffer) ;
  }
    
    return out;
}





