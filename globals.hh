#ifndef __GLOBALSH__
#define __GLOBALSH__

struct SDL_Window;
struct SDL_Surface;

namespace globals {
  extern uint32_t tohost_addr;
  extern uint32_t fromhost_addr;
  extern int sysArgc;
  extern char **sysArgv;
  extern bool silent;
  extern bool log;
  extern std::map<std::string, uint32_t> symtab;
  extern SDL_Window *sdlwin;
  extern SDL_Surface *sdlscr;
};

#define FB_WIDTH 320
#define FB_HEIGHT 200

#endif
