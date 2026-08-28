//*****************************************************************************
//
//! @file am_hal_cc312_pka.h
//!
//! @brief Hardware abstraction for the CryptoCell-312 PKA (big-integer engine)
//!
//! Purpose: This module drives the CryptoCell-312 Public Key Accelerator (PKA)
//! directly through its hardware registers. The PKA is an opcode-driven
//! big-integer / modular-arithmetic engine with its own SRAM register file; it
//! provides the modular primitives (add/sub/mul/exp/inv/reduction) on top of
//! which the elliptic-curve and ECDSA layers are built.
//!
//! This is a register-level port of the CryptoCell-312 runtime PKA driver
//! (pka.c / pka.h / pka_hw_defs.h / pka_defs.h). The runtime register names map
//! 1:1 onto the Ambiq CMSIS CRYPTO registers:
//!   OPCODE, NNPT0T1ADDR, PKASTATUS, PKAL0..7, PKAPIPERDY, PKADONE,
//!   PKASRAM{ADDR,WDATA,RDATA,RADDR}, PKAWORDACCESS, PKABUFFADDR, MEMORYMAP0..31.
//!
//! @addtogroup cc312_pka_ap510L CC312 PKA - Big-Integer Engine
//! @ingroup apollo510L_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_HAL_CC312_PKA_H
#define AM_HAL_CC312_PKA_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name PKA word / register geometry (CC312: 64-bit PKA word).
//! @{
//
//*****************************************************************************
#define AM_HAL_PKA_WORD_SIZE_IN_BITS            64
#define AM_HAL_PKA_WORD_SIZE_IN_BYTES           (AM_HAL_PKA_WORD_SIZE_IN_BITS / 8)
#define AM_HAL_PKA_WORD_SIZE_IN_32BIT_WORDS     (AM_HAL_PKA_WORD_SIZE_IN_BITS / 32)
#define AM_HAL_PKA_EXTRA_BITS                   8
#define AM_HAL_PKA_MAX_COUNT_OF_PHYS_MEM_REGS   32
#define AM_HAL_PKA_SRAM_BASE_ADDRESS            0x0UL
#define AM_HAL_PKA_ADDRESS_ENTRY_NOT_USED       0xFFCUL
//! @}

//*****************************************************************************
//
//! @name Reserved PKA virtual registers (N=0, NP=1, T0=30, T1=31).
//! @{
//
//*****************************************************************************
#define AM_HAL_PKA_REG_N                        0
#define AM_HAL_PKA_REG_NP                       1
#define AM_HAL_PKA_REG_T0                       30
#define AM_HAL_PKA_REG_T1                       31
//! @}

//*****************************************************************************
//
//! @name Elliptic-curve sizing (largest supported Weierstrass curve secp521r1).
//! @{
//
//*****************************************************************************
#define AM_HAL_PKA_MAX_EC_MOD_SIZE_BITS         521
#define AM_HAL_PKA_MAX_EC_MOD_SIZE_32BIT_WORDS  17  //!< ceil(521/32).
//! Full register operation size (modulus + extra PKA word), in PKA/32-bit words.
#define AM_HAL_PKA_GET_FULL_OP_SIZE_PKA_WORDS(opSizeInBits)                     \
    ((((opSizeInBits) / AM_HAL_PKA_WORD_SIZE_IN_BITS) +                         \
      ((((opSizeInBits) & (AM_HAL_PKA_WORD_SIZE_IN_BITS - 1)) > 0) ? 1U : 0U)) + 1U)
//! @}

//*****************************************************************************
//
//! PKA register-size table entry IDs (regsSizesTable). Matches LenIdTypes_t.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_PKA_LEN_ID_N_BITS          = 0,  //!< Modulus size (EC).
    AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS  = 1,  //!< Operation size (EC).
    AM_HAL_PKA_LEN_ID_PQ_BITS         = 2,  //!< P and Q size (RSA CRT).
    AM_HAL_PKA_LEN_ID_PQ_PKA_REG_BITS = 3,  //!< Operation size on P and Q (RSA CRT).
    AM_HAL_PKA_LEN_ID_AUX_PRIME_BITS  = 4,  //!< Auxiliary prime size (RSA keygen).
    AM_HAL_PKA_LEN_ID_AUX_PRIME_PKA_REG_BITS = 5, //!< Operation size on auxiliary primes.
    AM_HAL_PKA_LEN_ID_NP_BITS         = 6,  //!< Size used for Np (Barrett) calc.
    AM_HAL_PKA_LEN_ID_MAX_BITS        = 7,  //!< Full PKA register size.
    AM_HAL_PKA_LEN_ID_MAX             = 8   //!< Out-of-range marker.
}
am_hal_cc312_pka_len_id_e;

//*****************************************************************************
//
//! @name PKA HW opcode IDs (per CC312 HW documentation).
//! @{
//
//*****************************************************************************
#define AM_HAL_PKA_OPCODE_ID_ADD            0x04U
#define AM_HAL_PKA_OPCODE_ID_SUB            0x05U
#define AM_HAL_PKA_OPCODE_ID_MODADD         0x06U
#define AM_HAL_PKA_OPCODE_ID_MODSUB         0x07U
#define AM_HAL_PKA_OPCODE_ID_AND            0x08U
#define AM_HAL_PKA_OPCODE_ID_OR             0x09U
#define AM_HAL_PKA_OPCODE_ID_XOR            0x0AU
#define AM_HAL_PKA_OPCODE_ID_SHR0           0x0CU
#define AM_HAL_PKA_OPCODE_ID_SHR1           0x0DU
#define AM_HAL_PKA_OPCODE_ID_SHL0           0x0EU
#define AM_HAL_PKA_OPCODE_ID_SHL1           0x0FU
#define AM_HAL_PKA_OPCODE_ID_MULLOW         0x10U
#define AM_HAL_PKA_OPCODE_ID_MODMUL         0x11U
#define AM_HAL_PKA_OPCODE_ID_MODMULN        0x12U
#define AM_HAL_PKA_OPCODE_ID_MODEXP         0x13U
#define AM_HAL_PKA_OPCODE_ID_DIVISION       0x14U
#define AM_HAL_PKA_OPCODE_ID_MODINV         0x15U
#define AM_HAL_PKA_OPCODE_ID_MULHIGH        0x17U
#define AM_HAL_PKA_OPCODE_ID_MODMLAC        0x18U
#define AM_HAL_PKA_OPCODE_ID_MODMLACNR      0x19U
#define AM_HAL_PKA_OPCODE_ID_REDUCTION      0x1BU
#define AM_HAL_PKA_OPCODE_ID_TERMINATE      0x00U
//! @}

//*****************************************************************************
//
//! @name OPCODE register field bit positions and result-discard sentinel.
//! @{
//
//*****************************************************************************
#define AM_HAL_PKA_OPCODE_TAG_POS               0
#define AM_HAL_PKA_OPCODE_RESULT_POS            6
#define AM_HAL_PKA_OPCODE_OPERAND_2_POS         12
#define AM_HAL_PKA_OPCODE_OPERAND_1_POS         18
#define AM_HAL_PKA_OPCODE_LEN_POS               24
#define AM_HAL_PKA_OPCODE_OPERATION_ID_POS      27
#define AM_HAL_PKA_OPCODE_RES_OPERAND_MSBIT_OFFSET  5
#define AM_HAL_PKA_OPCODE_R_DISCARD_POS         (AM_HAL_PKA_OPCODE_RESULT_POS + AM_HAL_PKA_OPCODE_RES_OPERAND_MSBIT_OFFSET)
#define AM_HAL_PKA_OPCODE_OPERAND_2_IMMED_POS   (AM_HAL_PKA_OPCODE_OPERAND_2_POS + AM_HAL_PKA_OPCODE_RES_OPERAND_MSBIT_OFFSET)
#define AM_HAL_PKA_OPCODE_OPERAND_1_IMMED_POS   (AM_HAL_PKA_OPCODE_OPERAND_1_POS + AM_HAL_PKA_OPCODE_RES_OPERAND_MSBIT_OFFSET)
#define AM_HAL_PKA_RES_DISCARD                  0x3FU
//! @}

//*****************************************************************************
//
//! @name PKA_STATUS register field bit positions and N_NP_T0_T1 field positions.
//! @{
//
//*****************************************************************************
#define AM_HAL_PKA_STATUS_ALU_OUT_ZERO_POS      12
#define AM_HAL_PKA_STATUS_ALU_CARRY_POS         9
#define AM_HAL_PKA_STATUS_DIV_BY_ZERO_POS       14

#define AM_HAL_PKA_N_NP_T0_T1_REG_N_POS         0
#define AM_HAL_PKA_N_NP_T0_T1_REG_NP_POS        5
#define AM_HAL_PKA_N_NP_T0_T1_REG_T0_POS        10
#define AM_HAL_PKA_N_NP_T0_T1_REG_T1_POS        15
#define AM_HAL_PKA_N_NP_T0_T1_REG_DEFAULT_VAL                                   \
    ((uint32_t)((AM_HAL_PKA_REG_N  << AM_HAL_PKA_N_NP_T0_T1_REG_N_POS)  |       \
                (AM_HAL_PKA_REG_NP << AM_HAL_PKA_N_NP_T0_T1_REG_NP_POS) |       \
                (AM_HAL_PKA_REG_T0 << AM_HAL_PKA_N_NP_T0_T1_REG_T0_POS) |       \
                (AM_HAL_PKA_REG_T1 << AM_HAL_PKA_N_NP_T0_T1_REG_T1_POS)))
//! @}

//*****************************************************************************
//
//! @brief Wait until the PKA pipe is ready to accept a new opcode.
//
//*****************************************************************************
static inline void
am_hal_cc312_pka_wait_pipe_ready(void)
{
    while ((CRYPTO->PKAPIPERDY & 0x1U) != 0x1U) { }
}

//*****************************************************************************
//
//! @brief Wait until the current PKA operation has completed (pipe empty).
//
//*****************************************************************************
static inline void
am_hal_cc312_pka_wait_done(void)
{
    while ((CRYPTO->PKADONE & 0x1U) != 0x1U) { }
}

//*****************************************************************************
//
//! @name Indexed PKA register helpers (PKAL0..7 and MEMORYMAP0..31 are
//!       laid out contiguously in the CMSIS CRYPTO struct).
//! @{
//
//*****************************************************************************
//! Set regsSizesTable entry @p entry to @p sizeBits.
static inline void
am_hal_cc312_pka_set_reg_size(uint32_t sizeBits, uint32_t entry)
{
    am_hal_cc312_pka_wait_done();
    (&CRYPTO->PKAL0)[entry] = sizeBits;
}

//! Get regsSizesTable entry @p entry.
static inline uint32_t
am_hal_cc312_pka_get_reg_size(uint32_t entry)
{
    am_hal_cc312_pka_wait_done();
    return (&CRYPTO->PKAL0)[entry];
}

//! Set the physical address of virtual register @p virtReg in the mapping table.
static inline void
am_hal_cc312_pka_set_reg_address(uint32_t virtReg, uint32_t physAddr)
{
    am_hal_cc312_pka_wait_done();
    (&CRYPTO->MEMORYMAP0)[virtReg] = physAddr;
}

//! Get the physical address of virtual register @p virtReg from the mapping table.
static inline uint32_t
am_hal_cc312_pka_get_reg_address(uint32_t virtReg)
{
    am_hal_cc312_pka_wait_done();
    return (&CRYPTO->MEMORYMAP0)[virtReg];
}
//! @}

//*****************************************************************************
//
//! @brief Set N/NP/T0/T1 virtual register pointers in the N_NP_T0_T1 register.
//
//*****************************************************************************
static inline void
am_hal_cc312_pka_set_n_np_t0_t1(uint32_t n, uint32_t np, uint32_t t0, uint32_t t1)
{
    am_hal_cc312_pka_wait_done();
    CRYPTO->NNPT0T1ADDR =
        (uint32_t)((n  << AM_HAL_PKA_N_NP_T0_T1_REG_N_POS)  |
                   (np << AM_HAL_PKA_N_NP_T0_T1_REG_NP_POS) |
                   (t0 << AM_HAL_PKA_N_NP_T0_T1_REG_T0_POS) |
                   (t1 << AM_HAL_PKA_N_NP_T0_T1_REG_T1_POS));
}

//*****************************************************************************
//
//! @name PKA_STATUS accessors. Each waits for completion before sampling.
//! @{
//
//*****************************************************************************
static inline uint32_t
am_hal_cc312_pka_status_alu_out_zero(void)
{
    am_hal_cc312_pka_wait_done();
    return (CRYPTO->PKASTATUS >> AM_HAL_PKA_STATUS_ALU_OUT_ZERO_POS) & 1UL;
}

static inline uint32_t
am_hal_cc312_pka_status_carry(void)
{
    am_hal_cc312_pka_wait_done();
    return (CRYPTO->PKASTATUS >> AM_HAL_PKA_STATUS_ALU_CARRY_POS) & 1UL;
}

static inline uint32_t
am_hal_cc312_pka_status_div_by_zero(void)
{
    am_hal_cc312_pka_wait_done();
    return (CRYPTO->PKASTATUS >> AM_HAL_PKA_STATUS_DIV_BY_ZERO_POS) & 1UL;
}
//! @}

//*****************************************************************************
//
//! @brief Build a full PKA OPCODE word from its fields.
//
//*****************************************************************************
#define AM_HAL_PKA_SET_FULL_OPCODE(Opcode, LenID, IsAImmed, OpA, IsBImmed, OpB, ResDiscard, Res, Tag) \
    (((uint32_t)(Opcode)     << AM_HAL_PKA_OPCODE_OPERATION_ID_POS)    |        \
     ((uint32_t)(LenID)      << AM_HAL_PKA_OPCODE_LEN_POS)             |        \
     ((uint32_t)(IsAImmed)   << AM_HAL_PKA_OPCODE_OPERAND_1_IMMED_POS) |        \
     ((uint32_t)(OpA)        << AM_HAL_PKA_OPCODE_OPERAND_1_POS)       |        \
     ((uint32_t)(IsBImmed)   << AM_HAL_PKA_OPCODE_OPERAND_2_IMMED_POS) |        \
     ((uint32_t)(OpB)        << AM_HAL_PKA_OPCODE_OPERAND_2_POS)       |        \
     ((uint32_t)(ResDiscard) << AM_HAL_PKA_OPCODE_R_DISCARD_POS)       |        \
     ((uint32_t)(Res)        << AM_HAL_PKA_OPCODE_RESULT_POS)          |        \
     ((uint32_t)(Tag)        << AM_HAL_PKA_OPCODE_TAG_POS))

//! Issue a PKA operation: wait for the pipe, then write the OPCODE register.
#define AM_HAL_PKA_EXEC_OPERATION(Opcode, LenID, IsAImmed, OpA, IsBImmed, OpB, ResDiscard, Res, Tag) \
    do {                                                                       \
        uint32_t fullOpCode = AM_HAL_PKA_SET_FULL_OPCODE((Opcode), (LenID),    \
            (IsAImmed), (OpA), (IsBImmed), (OpB), (ResDiscard), (Res), (Tag)); \
        am_hal_cc312_pka_wait_pipe_ready();                                    \
        CRYPTO->OPCODE = fullOpCode;                                           \
    } while (0)

//*****************************************************************************
//
//! @name PKA arithmetic operation macros (Res/OpA/OpB are virtual reg numbers).
//! @{
//
//*****************************************************************************
#define AM_HAL_PKA_ADD(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_ADD, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_ADD_IM(lenId, Res, OpA, OpBIm) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_ADD, (lenId), 0, (OpA), 1, (OpBIm), 0, (Res), 0)
#define AM_HAL_PKA_SUB(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_SUB, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_SUB_IM(lenId, Res, OpA, OpBIm) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_SUB, (lenId), 0, (OpA), 1, (OpBIm), 0, (Res), 0)
#define AM_HAL_PKA_NEG(lenId, Res, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_SUB, (lenId), 1, 0, 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_ADD(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODADD, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_ADD_IM(lenId, Res, OpA, OpBIm) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODADD, (lenId), 0, (OpA), 1, (OpBIm), 0, (Res), 0)
#define AM_HAL_PKA_MOD_SUB(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODSUB, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_SUB_IM(lenId, Res, OpA, OpBIm) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODSUB, (lenId), 0, (OpA), 1, (OpBIm), 0, (Res), 0)
#define AM_HAL_PKA_MOD_NEG(lenId, Res, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODSUB, (lenId), 1, 0, 0, (OpB), 0, (Res), 0)

#define AM_HAL_PKA_AND(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_AND, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_AND_IM(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_AND, (lenId), 0, (OpA), 1, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_TEST_BIT0(lenId, OpA) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_AND, (lenId), 0, (OpA), 1, 0x01, 1, AM_HAL_PKA_RES_DISCARD, 0)
#define AM_HAL_PKA_TEST_BIT(lenId, OpA, i) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_AND, (lenId), 0, (OpA), 1, (0x01U << (i)), 1, AM_HAL_PKA_RES_DISCARD, 0)
#define AM_HAL_PKA_CLEAR_BIT0(lenId, Res, OpA) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_AND, (lenId), 0, (OpA), 1, 0x1E, 0, (Res), 0)
#define AM_HAL_PKA_CLEAR(lenId, OpA) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_AND, (lenId), 0, (OpA), 1, 0x00, 0, (OpA), 0)
#define AM_HAL_PKA_2CLEAR(lenId, OpA) \
    do { AM_HAL_PKA_CLEAR((lenId), (OpA)); AM_HAL_PKA_CLEAR((lenId), (OpA)); } while (0)
#define AM_HAL_PKA_OR(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_OR, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_OR_IM(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_OR, (lenId), 0, (OpA), 1, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_COPY(lenId, OpDest, OpSrc) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_OR, (lenId), 0, (OpSrc), 1, 0x00, 0, (OpDest), 0)
#define AM_HAL_PKA_SET_BIT0(lenId, Res, OpA) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_OR, (lenId), 0, (OpA), 1, 0x01, 0, (Res), 0)
#define AM_HAL_PKA_XOR(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_XOR, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_COMPARE(lenId, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_XOR, (lenId), 0, (OpA), 0, (OpB), 1, 0, 0)
#define AM_HAL_PKA_COMPARE_IM(lenId, OpA, OpBim) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_XOR, (lenId), 0, (OpA), 1, (OpBim), 1, 0, 0)

#define AM_HAL_PKA_SHR_FILL0(lenId, Res, OpA, S) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_SHR0, (lenId), 0, (OpA), 0, (S), 0, (Res), 0)
#define AM_HAL_PKA_SHL_FILL0(lenId, Res, OpA, S) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_SHL0, (lenId), 0, (OpA), 0, (S), 0, (Res), 0)

#define AM_HAL_PKA_MUL_LOW(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MULLOW, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MUL_HIGH(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MULHIGH, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_MUL(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODMUL, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_MUL_NFR(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODMULN, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_MUL_ACC(lenId, Res, OpA, OpB, OpC) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODMLAC, (lenId), 0, (OpA), 0, (OpB), 0, (Res), (OpC))
#define AM_HAL_PKA_MOD_EXP(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODEXP, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_DIV(lenId, Res, OpA, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_DIVISION, (lenId), 0, (OpA), 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_INV(lenId, Res, OpB) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODINV, (lenId), 1, 1, 0, (OpB), 0, (Res), 0)
#define AM_HAL_PKA_MOD_MUL_ACC_NFR(lenId, Res, OpA, OpB, OpC) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_MODMLACNR, (lenId), 0, (OpA), 0, (OpB), 0, (Res), (OpC))
#define AM_HAL_PKA_REDUCE(lenId, Res, OpA) \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_REDUCTION, (lenId), 0, (OpA), 0, 0, 0, (Res), 0)
//! @}

//*****************************************************************************
//
//! @name Second-level PKA helpers.
//! @{
//
//*****************************************************************************
//! Set register @p a to immediate value @p v (clear then OR).
#define AM_HAL_PKA_SET_VAL(a, v)                                               \
    do {                                                                       \
        AM_HAL_PKA_AND_IM(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, (a), (a), 0);      \
        AM_HAL_PKA_OR_IM(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, (a), (a), (v));     \
    } while (0)

//! Constant-time modular inverse via Fermat: res = a^(N-2) mod N (N prime).
//! @p nm2 is a scratch register that receives N-2.
#define AM_HAL_PKA_MOD_INV_W_EXP(res, a, nm2)                                  \
    do {                                                                       \
        AM_HAL_PKA_SUB_IM(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, (nm2), AM_HAL_PKA_REG_N, 2); \
        AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, (res), (a), (nm2));       \
    } while (0)

//! Compare register @p a to immediate @p b; @p stat = 1 if equal.
#define AM_HAL_PKA_COMPARE_IM_STATUS(lenId, a, b, stat)                        \
    do {                                                                       \
        AM_HAL_PKA_COMPARE_IM((lenId), (a), (b));                             \
        (stat) = am_hal_cc312_pka_status_alu_out_zero();                       \
    } while (0)

//! Compare registers @p a and @p b; @p stat = 1 if equal.
#define AM_HAL_PKA_COMPARE_STATUS(lenId, a, b, stat)                           \
    do {                                                                       \
        AM_HAL_PKA_COMPARE((lenId), (a), (b));                                \
        (stat) = am_hal_cc312_pka_status_alu_out_zero();                       \
    } while (0)

//! Invert bit 0 of @p OpA into @p Res (XOR with immediate 1).
#define AM_HAL_PKA_FLIP_BIT0(lenId, Res, OpA)                                  \
    AM_HAL_PKA_EXEC_OPERATION(AM_HAL_PKA_OPCODE_ID_XOR, (lenId), 0, (OpA), 1, 0x01, 0, (Res), 0)
//! @}

//*****************************************************************************
//
// Core PKA driver API (register-level).
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Enable the PKA and DMA engine clocks.
//
//*****************************************************************************
extern void am_hal_cc312_pka_enable_clocks(void);

//*****************************************************************************
//
//! @brief Disable the PKA and DMA engine clocks.
//
//*****************************************************************************
extern void am_hal_cc312_pka_disable_clocks(void);

//*****************************************************************************
//
//! @brief Initialize the PKA engine for an operation of the given modulus size.
//!
//! Enables clocks, configures the register sizes table and the physical-memory
//! mapping table for the requested operation size, and sets default N/NP/T0/T1.
//!
//! @param opSizeInBits - Operation (modulus) size in bits.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_pka_init(uint32_t opSizeInBits);

//*****************************************************************************
//
//! @brief Tear down the PKA engine (terminate op, clear regs, disable clocks).
//
//*****************************************************************************
extern void am_hal_cc312_pka_finish(void);

//*****************************************************************************
//
//! @brief Configure the PKA register sizes table for a given operation size.
//!
//! @param opSizeInBits - Operation (modulus) size in bits.
//
//*****************************************************************************
extern void am_hal_cc312_pka_set_regs_sizes_tab(uint32_t opSizeInBits);

//*****************************************************************************
//
//! @brief Copy a little-endian 32-bit word buffer into a PKA register.
//!
//! @param dstReg      - Destination virtual register number.
//! @param lenId       - Size-table entry id for the register's full length.
//! @param src         - Source buffer (little-endian words).
//! @param sizeWords   - Source size in 32-bit words.
//
//*****************************************************************************
extern void am_hal_cc312_pka_copy_data_into_reg(uint32_t dstReg,
                                                uint32_t lenId,
                                                const uint32_t *src,
                                                uint32_t sizeWords);

//*****************************************************************************
//
//! @brief Copy a PKA register out to a little-endian 32-bit word buffer.
//!
//! @param dst         - Destination buffer (little-endian words).
//! @param sizeWords   - Number of 32-bit words to read.
//! @param srcReg      - Source virtual register number.
//
//*****************************************************************************
extern void am_hal_cc312_pka_copy_data_from_reg(uint32_t *dst,
                                                uint32_t sizeWords,
                                                uint32_t srcReg);

//*****************************************************************************
//
//! @brief Copy a big-endian byte buffer into a PKA register.
//!
//! @param dstReg      - Destination virtual register number.
//! @param lenId       - Size-table entry id for the register's full length.
//! @param src         - Source buffer (big-endian, word-multiple length).
//! @param sizeWords   - Source size in 32-bit words.
//
//*****************************************************************************
extern void am_hal_cc312_pka_copy_be_bytes_into_reg(uint32_t dstReg,
                                                    uint32_t lenId,
                                                    const uint8_t *src,
                                                    uint32_t sizeWords);

//*****************************************************************************
//
//! @brief Copy a PKA register out to a big-endian byte buffer.
//!
//! @param dst         - Destination buffer (big-endian, word-multiple length).
//! @param sizeWords   - Number of 32-bit words to write.
//! @param srcReg      - Source virtual register number.
//
//*****************************************************************************
extern void am_hal_cc312_pka_copy_reg_into_be_bytes(uint8_t *dst,
                                                    uint32_t sizeWords,
                                                    uint32_t srcReg);

//*****************************************************************************
//
//! @brief Clear a contiguous block of PKA registers.
//!
//! @param firstReg    - First virtual register number.
//! @param countOfRegs - Number of registers to clear.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Zero the tail of a PKA register.
//!
//! Clears from @p wordOffset to the end of the register, leaving the low
//! words untouched. Operations that change size class - RSA CRT moves
//! between the P/Q size and the modulus size - leave junk in the high words
//! of a register, and the next operation at the wider size would consume it.
//!
//! @param reg        - Virtual register number.
//! @param wordOffset - First 32-bit word to clear.
//
//*****************************************************************************
extern void am_hal_cc312_pka_clear_reg_words(uint32_t reg,
                                             uint32_t wordOffset);

extern void am_hal_cc312_pka_clear_block_of_regs(uint32_t firstReg,
                                                 uint32_t countOfRegs);

//*****************************************************************************
//
//! @brief Compute the Barrett tag Np for the modulus in register N into NP.
//!
//! @param lenId     - Size-table entry id for the modulus length (with extension).
//! @param sizeNbits - Exact modulus size in bits.
//! @param regN      - Virtual register holding the modulus n.
//! @param regNp     - Destination virtual register for Np.
//! @param regTemp1  - Temporary virtual register.
//! @param regTempN  - Temporary virtual register.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_pka_calc_np(uint32_t lenId,
                                         uint32_t sizeNbits,
                                         int8_t regN,
                                         int8_t regNp,
                                         int8_t regTemp1,
                                         int8_t regTempN);

//*****************************************************************************
//
//! @brief Return the effective size in bits of the value in a PKA register.
//!
//! @param reg - Virtual register number.
//!
//! @return Effective size in bits.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_pka_get_reg_effective_size_in_bits(uint32_t reg);

//*****************************************************************************
//
//! @brief Read a single 32-bit word from a PKA register's SRAM.
//!
//! @param reg        - Virtual register number.
//! @param wordOffset - 32-bit word offset within the register.
//!
//! @return The 32-bit word value.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_pka_read_word_from_reg(uint32_t reg,
                                                    uint32_t wordOffset);

//
// Note: ECDSA/ECDH invert only modulo primes (the field prime p and the group
// order n), which are odd, so callers use the AM_HAL_PKA_MOD_INV() macro
// directly. The general even-modulus inversion path is not required here.
//

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_PKA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
