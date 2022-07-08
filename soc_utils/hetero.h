
#include <stdlib.h>
#ifdef __cplusplus
extern "C" {
#endif
    extern void * __hpvm_launch(void *, ...);
    extern void __hpvm_wait(void *);
    extern void * __hpvm_parallel_section_begin();
    extern void __hpvm_parallel_section_end(void *);
    extern void * __hpvm_task_begin(unsigned, ...);
    extern void __hpvm_task_end(void *);
    extern void __hpvm_parallel_loop(unsigned, ...);
    extern void * __hpvm_launch_begin(unsigned, ...);
    extern void __hpvm_launch_end(void *);
    extern void __hpvm_priv(unsigned, ...);
    extern void __hpvm__isNonZeroLoop(long, ...);
    extern void __hpvm__init();
    extern void __hpvm__cleanup();


    extern void __hetero_priv(unsigned, ...);
    extern void * __hetero_launch(void *, ...);
    extern void __hetero_wait(void *);
    extern void * __hetero_section_begin();
    extern void __hetero_section_end(void *);
    extern void * __hetero_task_begin(unsigned, ...);
    extern void __hetero_task_end(void *);
    extern void __hetero_parallel_loop(unsigned, ...);
    extern void * __hetero_launch_begin(unsigned, ...);
    extern void __hetero_launch_end(void *);
    extern void __hetero_copy_mem(void *, void *, size_t);
    extern void __hetero_request_mem(void *, size_t);
    extern void * __hetero_malloc(size_t);
    extern void __hetero_free(void *);
    extern void __hetero_hint(int);
#ifdef __cplusplus
}
#endif

