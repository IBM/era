#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h> // Unbuntu 18 on x86_64 has this at /usr/include/x86_64-linux-gnu/sys/socket.h
#include <signal.h>
#include <pthread.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <arpa/inet.h> // for inet_addr
#include <sys/time.h>


#include "hpvm.h"
#include "hetero.h"

int main(int argc, char * argv[]) {
        int* P1 = NULL;
        size_t P1Sz = sizeof(int);
        int* P2 = NULL;
        size_t P2Sz = sizeof(int);
        void * LaunchInner = __hetero_launch_begin(2, P1, P1Sz, P2, P2Sz, 1, P1, P1Sz);
        void * SectionWrap = __hetero_section_begin();
        {
                void * TWrap = __hetero_task_begin(2, P1, P1Sz, P2, P2Sz, 1, P1, P1Sz);
                printf("hello");
                __hetero_task_end(TWrap);
        }
        {
                void * T4 = __hetero_task_begin(2, P1, P1Sz, P2, P2Sz, 1, P1, P1Sz);
                printf("hello again");
                __hetero_task_end(T4);
        }
        __hetero_section_end(SectionWrap);
        __hetero_launch_end(LaunchInner);
        return 0;
}
