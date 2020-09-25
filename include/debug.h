// This file defines some DEBUG and VERBOSE macros (in one location)

#ifndef DEBUG_H
#define DEBUG_H 

 #ifdef DEBUG_MODE
  #define DEBUG(X) X
  #define DEBUG2(X)
  #define VERBOSE_MODE
 #else 
  #define DEBUG(X)
  #define DEBUG2(X)
 #endif

 #ifdef VERBOSE_MODE
  #define VERBOSE(X) X
 #else
  #define VERBOSE(X)
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
