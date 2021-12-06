#ifndef __linux_user_emulation_monitor__
#define __linux_user_emulation_monitor__

#include "linux_o32_syscall.hh"


static inline bool emulate_linux_syscall(Vcore_l1d_l1i *tb, state_t *s) {
  if(not(tb->monitor_req_valid)) {
    return false;
  }
  bool is_set_thread_area = s->gpr[R_v0] == 4283;
  linux_o32_syscall(s);
  if(is_set_thread_area) {
    tb->monitor_rsp_data = s->cpr0[29];
  }
  else {
    tb->monitor_rsp_data = s->gpr[R_v0];
  }
  tb->monitor_rsp_data_valid = 1;
  /* HACK!!! */
  s->gpr[R_a3] = 0;
  tb->monitor_rsp_valid = 1;
  return true;
}


#endif
