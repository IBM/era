/********************************
Author: Sravanthi Kota Venkata
********************************/

#include "sift.h"

F2D* resizeArray(F2D* array, int omin) 
{
	F2D* prev = NULL;
	F2D* current = array;
	int o;
    if(omin<0)
    {
        for(o=1; o>=-omin; o--)
        {
			prev = current;
            current = doubleSize(current);
			fFreeHandle(prev);
        }
    }
    if(omin>0)
    {
        for(o=1; o<= omin; o++)
		{
			prev = current;
            current = halveSize(current);
			fFreeHandle(prev);
		}
    }
	return current;
}

/**
    Returns the Gaussian scale space of image I. Image I is assumed to be
    pre-smoothed at level SIGMAN. O,S,OMIN,SMIN,SMAX,SIGMA0 are the
    parameters of the scale space.
**/

F2D** gaussianss(F2D* array, float sigman, int O, int S, int omin, int smin, int smax, float sigma0)
{
   /* We compute the following items in the function
    1. Smooth input image per octave
    2. Smooth each octave for different intervals
    3. Subtract each "interval-1" smooth image from "interval" image per octave. So, per octave, we have "interval" * DOG images.
    4. So, octave * intervals * image
    5. Note: At each octave, the image is scaled down in both x and y directions
    */

    float k, dsigma0, dsigma;
    int s, i, j, o, so, M, N, sbest;
    int intervals = smax-smin+1;
    float temp, target_sigma, prev_sigma;
    F2D *TMP, **gss;
    F2D* I = array;

    // Scale multiplicative step
    k = pow(2, (1.0/S));
    dsigma0 = sigma0 * sqrt(1-(1.0/pow(k,2)));  // Scale step factor
    
    // If omin < 0, multiply the size of the image.
	I = resizeArray(I, omin);
    M = I->height;
    N = I->width;
    so = -smin+1;       // Index offset

    gss = malloc(O*intervals*sizeof(F2D*));
    if(gss == NULL)
    {
        printf("Could not allocate memory\n");
        return NULL;
    }

/**
                                                         First octave
--------------------------------------------------------------------

The first level of the first octave has scale index (o,s) =
(omin,smin) and scale coordinate

    sigma(omin,smin) = sigma0 2^omin k^smin 

The input image I is at nominal scale sigman. Thus in order to get
the first level of the pyramid we need to apply a smoothing of
  
    sqrt( (sigma0 2^omin k^smin)^2 - sigman^2 ).

As we have pre-scaled the image omin octaves (up or down,
depending on the sign of omin), we need to correct this value
by dividing by 2^omin, getting

   sqrt( (sigma0 k^smin)^2 - (sigman/2^omin)^2 )

**/

    temp = sqrt(pow((sigma0*pow(k,smin)),2) - pow((sigman/pow(2,omin)),2));

    {
        gss[0] = fSetArray(I->height, I->width, 0);
        imsmooth(I, temp, gss[0] );
    }

    for(s=smin; s<smax; s++)
    {
	
    /**
        Here we go from (omin,s-1) to (omin,s). The extra smoothing
	    standard deviation is
	
	    (sigma0 2^omin 2^(s/S) )^2 - (simga0 2^omin 2^(s/S-1/S) )^2
	
	    After dividing by 2^omin (to take into account the fact
        that the image has been pre-scaled omin octaves), the 
        standard deviation of the smoothing kernel is
  
	    dsigma = sigma0 k^s sqrt(1-1/k^2)
    **/
    
        dsigma = pow(k,s+1) * dsigma0;
        gss[s+so] = fSetArray(gss[s+so-1]->height, gss[s+so-1]->width, 0);
        imsmooth( gss[(s+so-1)] , dsigma, gss[(s+so)] );
    }     
    
    /** Other octaves **/
    
    for(o=1; o<O; o++)
    {
        /**
            We need to initialize the first level of octave (o,smin) from
	        the closest possible level of the previous octave. A level (o,s)
            in this octave corrsponds to the level (o-1,s+S) in the previous
            octave. In particular, the level (o,smin) correspnds to
            (o-1,smin+S). However (o-1,smin+S) might not be among the levels
            (o-1,smin), ..., (o-1,smax) that we have previously computed.
            The closest pick is
  
	                               /  smin+S    if smin+S <= smax
	        (o-1,sbest) , sbest = |
                                   \  smax      if smin+S > smax
	
	        The amount of extra smoothing we need to apply is then given by
	
	        ( sigma0 2^o 2^(smin/S) )^2 - ( sigma0 2^o 2^(sbest/S - 1) )^2
	
            As usual, we divide by 2^o to cancel out the effect of the
            downsampling and we get

	        ( sigma 0 k^smin )^2 - ( sigma0 2^o k^(sbest - S) )^2
        **/

        sbest = MIN(smin+S-1, smax-1);
        TMP = halveSize( gss[(o-1)*intervals+sbest+so]);
        target_sigma = sigma0 * pow(k,smin);
        prev_sigma = sigma0 * pow(k, (sbest+1)-S);

        temp = sqrt(pow(target_sigma,2) - pow(prev_sigma, 2));
        if(target_sigma > prev_sigma)
        {
            gss[o*intervals] = fSetArray(TMP->height, TMP->width, 0);
            imsmooth(TMP, temp, gss[o*intervals] );
        }
        else
        {
            int i;
            gss[o*intervals] = fSetArray(TMP->height, TMP->width, 0);
            for(i=0; i<(TMP->height*TMP->width); i++)
                asubsref(gss[o*intervals],i) = asubsref(TMP,i);
        }

        M = TMP->height;
        N = TMP->width;

        fFreeHandle(TMP);

        for(s=smin; s<smax; s++)
        {
		    // The other levels are determined as above for the first octave.		
            dsigma = pow(k,s+1) * dsigma0;
            gss[o*intervals+s+so] = fSetArray(gss[o*intervals+s-1+so]->height, gss[o*intervals+s-1+so]->width, 0);
            imsmooth( gss[o*intervals+s-1+so] , dsigma, gss[o*intervals+s+so]);
        }    
    }

    fFreeHandle(I);
    return gss;
}
