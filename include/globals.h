#ifndef GLOBALS_H_
#define GLOBALS_H_

//#define DBGOUT(x) x  // Uncomment for DEBUG messages
#define DBGOUT(x)  // Uncomment for NO DEBUG messages

//#define DBGOUT2(x) x  // Uncomment for DEBUG Level-2 (more detail) messages
#define DBGOUT2(x)  // Uncomment for NO DEBUG Level-2 (more detail) messages

#define CHECK(x) x  // Uncomment for RUN-TIME CHECK logic
//#define CHECK(x)  // Uncomment for NO RUN-TIME CHECK logic

void closeout_and_exit(char* last_msg, int rv);

#endif // GLOBALS_H_
