#ifdef GCC
#define magic_timing_begin(cycleLo, cycleHi) {\
    asm volatile( "rdtsc": "=a" (cycleLo), "=d" (cycleHi)); \
}\

#define magic_timing_end(cycleLo, cycleHi) {\
    unsigned tempCycleLo, tempCycleHi; \
    asm volatile( "rdtsc": "=a" (tempCycleLo), "=d" (tempCycleHi)); \
    cycleLo = tempCycleLo-cycleLo;\
    cycleHi = tempCycleHi - cycleHi;\
}\



#define magic_timing_report(cycleLo, cycleHi) {\
    printf("Timing report: %d %d\n", cycleLo, cycleHi); \
}\

 


#endif

#ifdef METRO
 
#define magic_timing_begin(cycleLo, cycleHi) {\
    asm volatile( "mfsr $8, CYCLE_LO\n\t"  \
        "mfsr $9, CYCLE_HI\n\t"  \
        "addu %0, $8, $0\n\t"  \
        "addu %1, $9, $0\n\t"  \
        :"=r" (cycleLo), "=r" (cycleHi) \
        : \
        :"$8", "$9"\
    );\
}

#define magic_timing_end(cycleLo, cycleHi) {\
    asm volatile( \
        "mfsr $8, CYCLE_LO\n\t"  \
        "mfsr $9, CYCLE_HI\n\t" \
        "subu %0, $8, %0\n\t" \
        "subu %1, $9, %1\n\t" \
        :"=r" (cycleLo), "=r" (cycleHi) \
        : \
        :"$8", "$9"\
    ); \
}

#define magic_timing_report(cycleLo, cycleHi) {\
    asm volatile( "addu $8, %0, $0\n\t" \
        "mtsr PASS $8\n\t" \
        "mtsr PASS $9\n\t" \
        : \
        :"r" (cycleLo), "r" (cycleHi) \
        : "$8", "$9" \
    );\
}

//#define metro_magic_timing_report(cycleLo, cycleHi) {\
//    asm volatile( "nop\n\t");\
//}

#endif

#ifdef BTL

#include "/u/kvs/raw/rawlib/archlib/include/raw.h"

#define magic_timing_begin(cycleLo, cycleHi) {\
    raw_magic_timing_report_begin();\
}

#define magic_timing_end(cycleLo, cycleHi) {\
    raw_magic_timing_report_end(); \
}

#define magic_timing_report(cycleLo, cycleHi) {\
    raw_magic_timing_report_print(); \
}

    
//
//void metro_magic_timing_begin(int cycleLo, int cycleHi)
//{
//    raw_magic_timing_report_begin();
//}
//
//void metro_magic_timing_end(int cycleLo, int cycleHi)
//{
//    raw_magic_timing_report_end();
//}
//
//void metro_magic_timing_report(int cycleLo, int cycleHi)
//{
//    raw_magic_timing_report_print();
//    return;
//}

#endif
