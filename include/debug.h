// This file defines some DEBUG and VERBOSE macros (in one location)

#ifndef DEBUG_H
#define DEBUG_H 

 #ifdef DEBUG_MODE
  #define DEBUG(X) X
  #define DEBUG2(X)
 #else 
  #define DEBUG(X)
  #define DEBUG2(X)
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
