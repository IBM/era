// This file defines some DEBUG and VERBOSE macros (in one location)

#ifndef DEBUG_H
#define DEBUG_H 

#ifdef DEBUG_MODE
 #define DEBUG(X) X
 #define DEBUG2(X)
 #define DO_NUM_IOS_ANALYSIS(x) x
#else 
 #define DEBUG(X)
 #define DEBUG2(X)
 #ifdef DO_NUM_IOS
  #define DO_NUM_IOS_ANALYSIS(x) x
 #else
  #define DO_NUM_IOS_ANALYSIS(x)
 #endif
#endif

#ifdef DEBUG_FFT
 #define FFT_DEBUG(X) X
#else
 #define FFT_DEBUG(X)
#endif

#ifdef DBG_THREADS
 #define TDEBUG(x) x
#else
 #define TDEBUG(x)
#endif

#ifdef SUPER_VERBOSE
 #define SDEBUG(x) x
 #define DO_VERBOSE(x) x
#else
 #define SDEBUG(x)
 #define DO_VERBOSE(x)
#endif

//#define DO_LIMITS
#ifdef DO_LIMITS
 #define DO_LIMITS_ANALYSIS(x) x
#else
 #define DO_LIMITS_ANALYSIS(x)
#endif


/*
  #ifdef DEBUG_MODE
  #define DEBUG(X) X
  #define VERBOSE_MODE
  #else 
  #define DEBUG(X)
  #endif
  
  #ifdef VERBOSE_MODE
  #define VERBOSE(X) X
  #else
  #define VERBOSE(X) 
  #endif
*/

#endif
