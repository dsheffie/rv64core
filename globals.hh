#ifndef __GLOBALSH__
#define __GLOBALSH__

namespace globals {
  extern bool enClockFuncts;
  extern int sysArgc;
  extern char **sysArgv;
  extern bool isMipsEL;
  extern uint64_t icountMIPS;
  extern uint64_t cycle;
  extern bool trace_retirement;
  extern bool trace_fp;
};

#endif
