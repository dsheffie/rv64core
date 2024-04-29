/* Selectively copied from tinyemu
 * 
 * Copyright (c) 2016-2017 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "interpret.hh"
#include "temu_code.hh"

int64_t take_interrupt(state_t *s) {
  int64_t pending_irq_bitvec = s->mip & s->mie;
  int64_t enabled_ints = 0;  
  switch(s->priv)
    {
    case priv_machine:
      if (s->mstatus & MSTATUS_MIE) {
	enabled_ints = ~s->mideleg;
      }
      break;
    case priv_supervisor:
      enabled_ints = ~s->mideleg;
      if (s->mstatus & MSTATUS_SIE) {
	enabled_ints |= s->mideleg;
      }
      break;
    default:
    case priv_user:
      enabled_ints = -1L;
      break;
    }
  pending_irq_bitvec &= enabled_ints;
  
  if(pending_irq_bitvec != 0) {
    for(int32_t p = 31; p >= 0; p--) {
      if((pending_irq_bitvec >> p) & 1) {
	return p;
      }
    }
  }
  return 0;
    
}
