/* this was stolen from xv6 - https://github.com/mit-pdos/xv6-riscv */
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>


// use riscv's sv39 page table scheme.
#define SATP_SV39 (8L << 60)

#define MAKE_SATP(pagetable) (SATP_SV39 | (((uint64_t)pagetable) >> 12))

#define PGSIZE 4096 // bytes per page
#define PGSHIFT 12  // bits of offset within a page

#define PGROUNDUP(sz)  (((sz)+PGSIZE-1) & ~(PGSIZE-1))
#define PGROUNDDOWN(a) (((a)) & ~(PGSIZE-1))

#define PTE_V (1L << 0) // valid
#define PTE_R (1L << 1)
#define PTE_W (1L << 2)
#define PTE_X (1L << 3)
#define PTE_U (1L << 4) // user can access

// shift a physical address to the right place for a PTE.
#define PA2PTE(pa) ((((uint64_t)pa) >> 12) << 10)
#define PTE2PA(pte) (((pte) >> 10) << 12)
#define PTE_FLAGS(pte) ((pte) & 0x3FF)

// extract the three 9-bit page table indices from a virtual address.
#define PXMASK          0x1FF // 9 bits
#define PXSHIFT(level)  (PGSHIFT+(9*(level)))
#define PX(level, va) ((((uint64_t) (va)) >> PXSHIFT(level)) & PXMASK)

typedef uint64_t pte_t;
typedef uint64_t pde_t;
typedef uint64_t *pagetable_t; // 512 PTEs

// flush the TLB.
static inline void
sfence_vma()
{
  // the zero, zero means flush all TLB entries.
  asm volatile("sfence.vma zero, zero");
}

static inline void 
w_satp(uint64_t x)
{
  asm volatile("csrw satp, %0" : : "r" (x));
}

// Return the address of the PTE in page table pagetable
// that corresponds to virtual address va.  If alloc!=0,
// create any required page-table pages.
//
// The risc-v Sv39 scheme has three levels of page-table
// pages. A page-table page contains 512 64-bit PTEs.
// A 64-bit virtual address is split into five fields:
//   39..63 -- must be zero.
//   30..38 -- 9 bits of level-2 index.
//   21..29 -- 9 bits of level-1 index.
//   12..20 -- 9 bits of level-0 index.
//    0..11 -- 12 bits of byte offset within the page.
pte_t *
walk(pagetable_t pagetable, uint64_t va, int alloc)
{
  for(int level = 2; level > 0; level--) {
    pte_t *pte = &pagetable[PX(level, va)];
    if(*pte & PTE_V) {
      pagetable = (pagetable_t)PTE2PA(*pte);
    } else {
      if(!alloc || (pagetable = (pde_t*)malloc(4096)) == 0)	 
        return 0;
      memset(pagetable, 0, PGSIZE);
      *pte = PA2PTE(pagetable) | PTE_V;
    }
  }
  return &pagetable[PX(0, va)];
}


// Create PTEs for virtual addresses starting at va that refer to
// physical addresses starting at pa.
// va and size MUST be page-aligned.
// Returns 0 on success, -1 if walk() couldn't
// allocate a needed page-table page.
int
mappages(pagetable_t pagetable, uint64_t va, uint64_t size, uint64_t pa, int perm)
{
  uint64_t a, last;
  pte_t *pte;

  assert((va % PGSIZE) == 0);
  assert((size % PGSIZE) == 0);
  assert(size != 0);
  a = va;
  last = va + size - PGSIZE;
  for(;;){
    //printf("a = %lx, pa = %lx\n", a, pa);
    if((pte = walk(pagetable, a, 1)) == 0) {
      return -1;
    }
    //printf("pte = %p, bits %lx\n", pte, *pte);
    *pte = PA2PTE(pa) | perm | PTE_V;
    if(a == last)
      break;
    a += PGSIZE;
    pa += PGSIZE;
  }
  return 0;
}

pagetable_t kernel_pagetable;
int main_();
int main() {
  kernel_pagetable = (pagetable_t)malloc(4096);
  memset(kernel_pagetable, 0, PGSIZE);
  mappages(kernel_pagetable, 0x80000000, 1UL<<20, 0x80000000, PTE_R | PTE_W | PTE_X);
  sfence_vma();
  w_satp(MAKE_SATP(kernel_pagetable));
  sfence_vma();
  return main_();
}
