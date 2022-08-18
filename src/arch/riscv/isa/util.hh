#ifndef __ARCH_RISCV_ISA_UTIL_HH__
#define __ARCH_RISCV_ISA_UTIL_HH__

#ifdef USE_RISCV_RV32
#define IS_RV32() true
#else
#define IS_RV32() false
#endif

#define ILLEGAL_INST_IN_RV32() \
    do { \
        if (IS_RV32()) { \
            return std::make_shared<IllegalInstFault>( \
                "Invalid instruction in rv32", machInst); \
        } \
    } while (false)

#endif  // __ARCH_RISCV_ISA_UTIL_HH__
