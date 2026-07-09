#include "ch32fun.h"

// Fast memcpy implementation using custom QingKe V3[BC] mcpy instruction.
// I have a ton of questions about this instruction. For example, what happens
// with unaligned sources and destinations? What about non-word sizes? Who can truly say.

#if defined(FUNCONF_ENABLE_XWMEMCPY) && FUNCONF_ENABLE_XWMEMCPY == 1
__attribute__((always_inline)) inline void *memcpy(void *__restrict dst, const void *__restrict src, size_t size) {
    // GCC doesn't realize that mcpy clobbers the destination register, so manually save it.
    void *return_dest = dst;

    // in case you want to know the actual encoding of this instruction:
	// __asm volatile(".insn r4 MISC_MEM, 0b111, 0b00, zero, %2, %0, %1" : \
    //                "+r"(start), "+r"(dst) : "r"(end) : "memory");
    __asm volatile("mcpy %2, %0, %1" : \
                   "+r"(src), "+r"(dst) : "r"((void *)((uint32_t)src+size)) : "memory");

    return return_dest;
}
#endif