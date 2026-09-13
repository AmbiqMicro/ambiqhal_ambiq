//*****************************************************************************
//
//! @file am_hal_cc312_pka.c
//!
//! @brief Hardware abstraction for the CryptoCell-312 PKA (big-integer engine)
//!
//! Purpose: Register-level driver for the CryptoCell-312 Public Key Accelerator.
//!          Ported from the CC312 runtime PKA driver (pka.c). Provides PKA
//!          initialization (register sizes table + physical-memory mapping
//!          table), big-integer transfer to/from the PKA SRAM register file,
//!          the Barrett tag (Np) computation, and the supporting helpers used
//!          by the elliptic-curve and ECDSA layers. The modular arithmetic
//!          primitives themselves are the AM_HAL_PKA_* opcode macros declared
//!          in am_hal_cc312_pka.h.
//!
//! @addtogroup cc312_pka_ap510 CC312 PKA - Big-Integer Engine
//! @ingroup apollo510_hal
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

// Redirect libc memcpy/memset to volatile helpers in am_hal_cc312.h.
#define AM_HAL_CC312_C

#include "am_mcu_apollo.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//! @name Size helper macros (mirror the CC312 runtime CALC_* / GET_FULL_* set).
//! @{
//
//*****************************************************************************
#define PKA_BITS_IN_32BIT_WORD          32U
#define PKA_32BIT_WORD_SIZE             4U
#define PKA_CALC_FULL_32BIT_WORDS(numBits)                                     \
    ((numBits) / PKA_BITS_IN_32BIT_WORD +                                      \
     (((numBits) & (PKA_BITS_IN_32BIT_WORD - 1U)) > 0U))
#define PKA_CALC_32BIT_WORDS_FROM_BYTES(sizeBytes)                             \
    ((sizeBytes) / PKA_32BIT_WORD_SIZE +                                       \
     (((sizeBytes) & (PKA_32BIT_WORD_SIZE - 1U)) > 0U))
#define PKA_GET_FULL_OP_SIZE_BITS(opSizeInBits)                                \
    (AM_HAL_PKA_WORD_SIZE_IN_BITS *                                            \
     AM_HAL_PKA_GET_FULL_OP_SIZE_PKA_WORDS(opSizeInBits))

//! PKA data SRAM size (CC312: 6 KB).
#define PKA_SRAM_SIZE_IN_BYTES          (6U * 1024U)
//! Minimum supported operation size.
#define PKA_MIN_OPERATION_SIZE_BITS     PKA_BITS_IN_32BIT_WORD
//! @}

//*****************************************************************************
//
//! @brief Byte-reverse a 32-bit word (BE <-> LE conversion).
//
//*****************************************************************************
static inline uint32_t
pka_rev32(uint32_t x)
{
    return ((x >> 24) & 0x000000FFU) |
           ((x >> 8)  & 0x0000FF00U) |
           ((x << 8)  & 0x00FF0000U) |
           ((x << 24) & 0xFF000000U);
}

//*****************************************************************************
//
// Low-level PKA SRAM access. The CC312 SRAM data-ready wait is a no-op on this
// platform (matches the runtime default), so transfers stream straight through
// the auto-incrementing PKA_SRAM_{W,R}DATA registers.
//
//*****************************************************************************

//! Round a 32-bit-word count up to a whole PKA word.
static inline uint32_t
pka_round_up_to_pka_word(uint32_t sizeWords)
{
    return ((sizeWords + (AM_HAL_PKA_WORD_SIZE_IN_32BIT_WORDS - 1U)) /
            AM_HAL_PKA_WORD_SIZE_IN_32BIT_WORDS) *
           AM_HAL_PKA_WORD_SIZE_IN_32BIT_WORDS;
}

//! Load a block of 32-bit words into PKA SRAM at @p addr, zero-padding to a
//! whole PKA word.
static void
pka_hw_load_block(uint32_t addr, const uint32_t *ptr, uint32_t sizeWords)
{
    uint32_t ii;
    uint32_t padded = pka_round_up_to_pka_word(sizeWords);

    am_hal_cc312_pka_wait_done();
    CRYPTO->PKASRAMADDR = addr;
    for (ii = 0; ii < sizeWords; ii++)
    {
        CRYPTO->PKASRAMWDATA = ptr[ii];
    }
    for (; ii < padded; ii++)
    {
        CRYPTO->PKASRAMWDATA = 0U;
    }
}

//! Zero @p sizeWords 32-bit words of PKA SRAM starting at @p addr.
static void
pka_hw_clear_mem(uint32_t addr, uint32_t sizeWords)
{
    uint32_t ii;
    uint32_t padded = pka_round_up_to_pka_word(sizeWords);

    am_hal_cc312_pka_wait_done();
    CRYPTO->PKASRAMADDR = addr;
    for (ii = 0; ii < padded; ii++)
    {
        CRYPTO->PKASRAMWDATA = 0U;
    }
}

//! Read a block of 32-bit words from PKA SRAM at @p addr.
static void
pka_hw_read_block(uint32_t addr, uint32_t *ptr, uint32_t sizeWords)
{
    uint32_t ii;

    am_hal_cc312_pka_wait_done();
    CRYPTO->PKASRAMRADDR = addr;
    for (ii = 0; ii < sizeWords; ii++)
    {
        ptr[ii] = CRYPTO->PKASRAMRDATA;
    }
}

//! Write a single 32-bit value into PKA SRAM at @p addr.
static void
pka_hw_load_value(uint32_t addr, uint32_t val)
{
    am_hal_cc312_pka_wait_done();
    CRYPTO->PKASRAMADDR = addr;
    CRYPTO->PKASRAMWDATA = val;
}

//! Read a single 32-bit value from PKA SRAM at @p addr.
static uint32_t
pka_hw_read_value(uint32_t addr)
{
    am_hal_cc312_pka_wait_done();
    CRYPTO->PKASRAMRADDR = addr;
    return CRYPTO->PKASRAMRDATA;
}

//! Write a single 32-bit word at word offset @p off of virtual register @p reg.
static void
pka_write_word_to_reg(uint32_t reg, uint32_t off, uint32_t val)
{
    uint32_t addr = am_hal_cc312_pka_get_reg_address(reg);
    pka_hw_load_value(addr + off, val);
}

//! Compute the PKA register size in PKA words for a given operation size.
static uint32_t
pka_calc_reg_size_in_pka_words(uint32_t opSizeInBits)
{
    uint32_t regSizeIn32BitWords;

    if (opSizeInBits < (2U * (AM_HAL_PKA_WORD_SIZE_IN_BITS + AM_HAL_PKA_EXTRA_BITS)))
    {
        regSizeIn32BitWords = PKA_CALC_FULL_32BIT_WORDS(
            opSizeInBits + AM_HAL_PKA_WORD_SIZE_IN_BITS + AM_HAL_PKA_EXTRA_BITS - 1U);
        if ((opSizeInBits + AM_HAL_PKA_WORD_SIZE_IN_BITS + AM_HAL_PKA_EXTRA_BITS - 1U) %
            PKA_BITS_IN_32BIT_WORD)
        {
            regSizeIn32BitWords++;
        }
    }
    else
    {
        regSizeIn32BitWords = PKA_CALC_FULL_32BIT_WORDS(opSizeInBits);
    }

    return AM_HAL_PKA_GET_FULL_OP_SIZE_PKA_WORDS(regSizeIn32BitWords * PKA_BITS_IN_32BIT_WORD);
}

//*****************************************************************************
//
//! @brief Configure the physical-memory mapping table (register addresses).
//
//*****************************************************************************
static void
pka_set_regs_map_tab(int32_t countOfRegs, int32_t regSizeInPkaWords)
{
    uint32_t currentAddr = AM_HAL_PKA_SRAM_BASE_ADDRESS;
    int32_t  i;
    uint32_t step = (uint32_t)regSizeInPkaWords * AM_HAL_PKA_WORD_SIZE_IN_32BIT_WORDS;

    for (i = 0; i < AM_HAL_PKA_MAX_COUNT_OF_PHYS_MEM_REGS - 2; i++)
    {
        if (i < countOfRegs - 2)
        {
            (&CRYPTO->MEMORYMAP0)[i] = currentAddr;
            currentAddr += step;
        }
        else
        {
            (&CRYPTO->MEMORYMAP0)[i] = AM_HAL_PKA_ADDRESS_ENTRY_NOT_USED;
        }
    }

    //
    // Temp registers T0=30, T1=31.
    //
    (&CRYPTO->MEMORYMAP0)[AM_HAL_PKA_REG_T0] = currentAddr;
    (&CRYPTO->MEMORYMAP0)[AM_HAL_PKA_REG_T1] = currentAddr + step;

    //
    // Default virtual N/NP/T0/T1 pointers.
    //
    am_hal_cc312_pka_set_n_np_t0_t1(AM_HAL_PKA_REG_N, AM_HAL_PKA_REG_NP,
                                    AM_HAL_PKA_REG_T0, AM_HAL_PKA_REG_T1);
}

//*****************************************************************************
//
// Enable / disable PKA clock.
//
//*****************************************************************************
void
am_hal_cc312_pka_enable_clocks(void)
{
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_PKA);
}

void
am_hal_cc312_pka_disable_clocks(void)
{
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_PKA);
}

//*****************************************************************************
//
// Configure the PKA register sizes table.
//
//*****************************************************************************
void
am_hal_cc312_pka_set_regs_sizes_tab(uint32_t opSizeInBits)
{
    uint32_t i;
    uint32_t regSizeInPkaWords = pka_calc_reg_size_in_pka_words(opSizeInBits);
    uint32_t fullRegSizeBits   = regSizeInPkaWords * AM_HAL_PKA_WORD_SIZE_IN_BITS;

    //
    // Entry 0: exact operation size. Entry 1: extended (with extra word) size.
    //
    (&CRYPTO->PKAL0)[0] = opSizeInBits;
    (&CRYPTO->PKAL0)[1] = PKA_GET_FULL_OP_SIZE_BITS(opSizeInBits);

    //
    // Remaining entries: full register size in bits.
    //
    for (i = 2; i < 8U; i++)
    {
        (&CRYPTO->PKAL0)[i] = fullRegSizeBits;
    }
}

//*****************************************************************************
//
// Initialize the PKA engine for a given operation (modulus) size.
//
//*****************************************************************************
uint32_t
am_hal_cc312_pka_init(uint32_t opSizeInBits)
{
    uint32_t regSizeInPkaWords;
    uint32_t regsCount;

    if (opSizeInBits < PKA_MIN_OPERATION_SIZE_BITS)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    regSizeInPkaWords = pka_calc_reg_size_in_pka_words(opSizeInBits);

    //
    // Number of physical registers that fit in the PKA SRAM (capped at 32).
    //
    regsCount = PKA_SRAM_SIZE_IN_BYTES /
                (regSizeInPkaWords * AM_HAL_PKA_WORD_SIZE_IN_BYTES);
    if (regsCount > AM_HAL_PKA_MAX_COUNT_OF_PHYS_MEM_REGS)
    {
        regsCount = AM_HAL_PKA_MAX_COUNT_OF_PHYS_MEM_REGS;
    }

    //
    // Mask the PKA "exp" interrupt (operations are polled via PKA_DONE).
    //
    CRYPTO->HOSTRGFIMR |= CRYPTO_HOSTRGFIMR_PKAEXPMASK_Msk;

    //
    // Enable the PKA clock, then program the mapping and sizes tables.
    //
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_PKA);
    pka_set_regs_map_tab((int32_t)regsCount, (int32_t)regSizeInPkaWords);
    am_hal_cc312_pka_set_regs_sizes_tab(opSizeInBits);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Tear down the PKA engine.
//
//*****************************************************************************
void
am_hal_cc312_pka_finish(void)
{
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_PKA);
}

//*****************************************************************************
//
// Zero the tail of a PKA register.
//
//*****************************************************************************
void
am_hal_cc312_pka_clear_reg_words(uint32_t reg, uint32_t wordOffset)
{
    uint32_t addr;
    uint32_t regSizeWords;

    //
    // The register spans the full PKA register width, so the tail runs from
    // the offset up to that size.
    //
    regSizeWords = (am_hal_cc312_pka_get_reg_size(AM_HAL_PKA_LEN_ID_MAX_BITS) +
                    31U) / 32U;

    if (regSizeWords <= wordOffset)
    {
        return;
    }

    addr = am_hal_cc312_pka_get_reg_address(reg) + wordOffset;

    pka_hw_clear_mem(addr, regSizeWords - wordOffset);
}

//*****************************************************************************
//
// Clear a block of PKA registers (plus the two temp registers).
//
//*****************************************************************************
void
am_hal_cc312_pka_clear_block_of_regs(uint32_t firstReg, uint32_t countOfRegs)
{
    uint32_t i;
    uint32_t size;
    uint32_t addr;

    size = am_hal_cc312_pka_get_reg_size(AM_HAL_PKA_LEN_ID_MAX_BITS);
    size = PKA_CALC_FULL_32BIT_WORDS(size);

    for (i = 0; i < countOfRegs; i++)
    {
        addr = am_hal_cc312_pka_get_reg_address(firstReg + i);
        pka_hw_clear_mem(addr, size);
    }

    addr = am_hal_cc312_pka_get_reg_address(AM_HAL_PKA_REG_T1);
    pka_hw_clear_mem(addr, size);
    addr = am_hal_cc312_pka_get_reg_address(AM_HAL_PKA_REG_T0);
    pka_hw_clear_mem(addr, size);
}

//*****************************************************************************
//
// Copy a little-endian word buffer into a PKA register (high words zeroed).
//
//*****************************************************************************
void
am_hal_cc312_pka_copy_data_into_reg(uint32_t dstReg,
                                    uint32_t lenId,
                                    const uint32_t *src,
                                    uint32_t sizeWords)
{
    uint32_t currAddr = am_hal_cc312_pka_get_reg_address(dstReg);
    uint32_t regSize;
    uint32_t alignedWords;

    pka_hw_load_block(currAddr, src, sizeWords);

    //
    // PKA SRAM is addressed in 64-bit words. An odd 32-bit count leaves a
    // half-word that must be zeroed - same padding the BE loader already did.
    // Without it, leftover bits in NP (3 words) or Poly1305 DATA (5 words)
    // corrupt Barrett reduction / modular add.
    //
    alignedWords = pka_round_up_to_pka_word(sizeWords);
    if (alignedWords > sizeWords)
    {
        pka_hw_clear_mem(currAddr + sizeWords, alignedWords - sizeWords);
    }
    currAddr += alignedWords;

    regSize = am_hal_cc312_pka_get_reg_size(lenId);
    regSize = PKA_CALC_FULL_32BIT_WORDS(regSize);

    if (regSize > alignedWords)
    {
        pka_hw_clear_mem(currAddr, regSize - alignedWords);
    }
}

//*****************************************************************************
//
// Copy a PKA register out to a little-endian word buffer.
//
//*****************************************************************************
void
am_hal_cc312_pka_copy_data_from_reg(uint32_t *dst,
                                    uint32_t sizeWords,
                                    uint32_t srcReg)
{
    uint32_t currAddr = am_hal_cc312_pka_get_reg_address(srcReg);
    pka_hw_read_block(currAddr, dst, sizeWords);
}

//*****************************************************************************
//
// Copy a big-endian byte (word-multiple) buffer into a PKA register.
//
//*****************************************************************************
void
am_hal_cc312_pka_copy_be_bytes_into_reg(uint32_t dstReg,
                                        uint32_t lenId,
                                        const uint8_t *src,
                                        uint32_t sizeWords)
{
    uint32_t currAddr = am_hal_cc312_pka_get_reg_address(dstReg);
    uint32_t regSize;
    uint32_t alignedWords;
    uint32_t tempWord;
    int32_t  ii;
    int32_t  size = (int32_t)(sizeWords * PKA_32BIT_WORD_SIZE);

    am_hal_cc312_pka_wait_done();
    CRYPTO->PKASRAMADDR = currAddr;

    //
    // Source is big-endian: most-significant word is at offset 0, so write the
    // register low word (last source word) first, reversing byte order.
    //
    for (ii = size - (int32_t)PKA_32BIT_WORD_SIZE; ii >= 0; ii -= (int32_t)PKA_32BIT_WORD_SIZE)
    {
        memcpy((uint8_t *)&tempWord, &src[ii], PKA_32BIT_WORD_SIZE);
        CRYPTO->PKASRAMWDATA = pka_rev32(tempWord);
    }

    alignedWords = pka_round_up_to_pka_word(sizeWords);

    //
    // Pad the partially-written PKA word with zeros.
    //
    for (ii = (int32_t)sizeWords; ii < (int32_t)alignedWords; ii++)
    {
        CRYPTO->PKASRAMWDATA = 0U;
    }

    currAddr += alignedWords;

    regSize = am_hal_cc312_pka_get_reg_size(lenId);
    regSize = PKA_CALC_FULL_32BIT_WORDS(regSize);

    if (regSize > alignedWords)
    {
        pka_hw_clear_mem(currAddr, regSize - alignedWords);
    }
}

//*****************************************************************************
//
// Copy a PKA register out to a big-endian byte (word-multiple) buffer.
//
//*****************************************************************************
void
am_hal_cc312_pka_copy_reg_into_be_bytes(uint8_t *dst,
                                        uint32_t sizeWords,
                                        uint32_t srcReg)
{
    uint32_t currAddr = am_hal_cc312_pka_get_reg_address(srcReg);
    uint32_t tempWord;
    int32_t  ii;

    am_hal_cc312_pka_wait_done();
    CRYPTO->PKASRAMRADDR = currAddr;

    //
    // Register low word goes to the most-significant (last) buffer word.
    //
    for (ii = (int32_t)sizeWords - 1; ii >= 0; ii--)
    {
        tempWord = CRYPTO->PKASRAMRDATA;
        tempWord = pka_rev32(tempWord);
        memcpy(&dst[ii * (int32_t)PKA_32BIT_WORD_SIZE], (uint8_t *)&tempWord, PKA_32BIT_WORD_SIZE);
    }
}

//*****************************************************************************
//
// Return the effective size in bits of the value in a PKA register.
//
//*****************************************************************************
uint32_t
am_hal_cc312_pka_get_reg_effective_size_in_bits(uint32_t reg)
{
    int32_t  size;
    int32_t  i;
    uint32_t addr;
    uint32_t currWord = 0;
    uint32_t mask = 1UL << 31;

    addr = am_hal_cc312_pka_get_reg_address(reg);
    size = (int32_t)am_hal_cc312_pka_get_reg_size(AM_HAL_PKA_LEN_ID_MAX_BITS);
    size = (int32_t)PKA_CALC_FULL_32BIT_WORDS((uint32_t)size);

    for (i = size - 1; i >= 0; i--)
    {
        currWord = pka_hw_read_value(addr + (uint32_t)i);
        if (currWord != 0)
        {
            break;
        }
    }

    size = (int32_t)PKA_BITS_IN_32BIT_WORD * (i + 1);

    if (currWord == 0)
    {
        return (uint32_t)size;
    }

    for (i = 1; i <= (int32_t)PKA_BITS_IN_32BIT_WORD; i++)
    {
        if (currWord & mask)
        {
            break;
        }
        size--;
        mask >>= 1;
    }

    return (uint32_t)size;
}

//*****************************************************************************
//
// Read a single 32-bit word from a PKA register's SRAM.
//
//*****************************************************************************
uint32_t
am_hal_cc312_pka_read_word_from_reg(uint32_t reg, uint32_t wordOffset)
{
    uint32_t addr = am_hal_cc312_pka_get_reg_address(reg);
    return pka_hw_read_value(addr + wordOffset);
}

//*****************************************************************************
//
// Compute the Barrett tag Np for the modulus in regN into regNp.
//
//*****************************************************************************
uint32_t
am_hal_cc312_pka_calc_np(uint32_t lenId,
                         uint32_t sizeNbits,
                         int8_t regN,
                         int8_t regNp,
                         int8_t regTemp1,
                         int8_t regTempN)
{
    int32_t  i;
    uint32_t A = AM_HAL_PKA_WORD_SIZE_IN_BITS;
    uint32_t X = AM_HAL_PKA_EXTRA_BITS;
    int32_t  bNom, wNom;
    uint32_t val;
    int32_t  sh, st, wT;

    //
    // Clear temporaries and copy the modulus into regTempN.
    //
    AM_HAL_PKA_2CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, regTemp1);
    AM_HAL_PKA_2CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, regTempN);
    AM_HAL_PKA_2CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, regNp);
    AM_HAL_PKA_COPY(AM_HAL_PKA_LEN_ID_MAX_BITS, regTempN, regN);

    if (sizeNbits <= (2U * A + 2U * X))
    {
        //
        // Full size: Np = floor(2^(N+A+X-1) / n).
        //
        wNom = (int32_t)PKA_CALC_FULL_32BIT_WORDS(sizeNbits + A + X - 1U);
        bNom = (int32_t)((sizeNbits + A + X - 1U) % PKA_BITS_IN_32BIT_WORD);
        if (bNom)
        {
            val = 1UL << bNom;
        }
        else
        {
            wNom++;
            val = 1UL;
        }

        pka_write_word_to_reg((uint32_t)regTemp1, (uint32_t)(wNom - 1), val);
        AM_HAL_PKA_DIV(AM_HAL_PKA_LEN_ID_MAX_BITS, regNp, regTemp1, regTempN);
    }
    else
    {
        //
        // Truncated: nominator D = 3*A + 3*X - 1.
        //
        wNom = (int32_t)PKA_CALC_FULL_32BIT_WORDS(3U * A + 3U * X - 1U);
        bNom = (int32_t)((3U * A + 3U * X - 1U) % PKA_BITS_IN_32BIT_WORD);
        if (bNom)
        {
            val = 1UL << bNom;
        }
        else
        {
            wNom++;
            val = 1UL;
        }

        pka_write_word_to_reg((uint32_t)regTemp1, (uint32_t)(wNom - 1), val);

        st = (int32_t)(sizeNbits - 2U * A - 2U * X);
        wT = st / (int32_t)PKA_BITS_IN_32BIT_WORD;
        sh = st % (int32_t)PKA_BITS_IN_32BIT_WORD;

        AM_HAL_PKA_SUB_IM(lenId + 1, regTempN, regTempN, 1);

        for (i = 0; i < wT; i++)
        {
            AM_HAL_PKA_SHR_FILL0(lenId + 1, regTempN, regTempN, PKA_BITS_IN_32BIT_WORD - 1U);
        }
        if (sh)
        {
            AM_HAL_PKA_SHR_FILL0(lenId + 1, regTempN, regTempN, sh - 1);
        }

        AM_HAL_PKA_ADD_IM(lenId + 1, regTempN, regTempN, 1);
        AM_HAL_PKA_DIV(AM_HAL_PKA_LEN_ID_MAX_BITS, regNp, regTemp1, regTempN);
    }

    AM_HAL_PKA_2CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, regTemp1);
    AM_HAL_PKA_2CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, regTempN);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
