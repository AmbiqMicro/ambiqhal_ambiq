//*****************************************************************************
//
//! @file am_hal_cc312_rsa.c
//!
//! @brief Hardware abstraction for CryptoCell-312 RSA primitives
//!
//! RSA has no dedicated engine. Both of its operations are one modular
//! exponentiation, which the PKA provides as a single opcode, so this file
//! is the PKA setup around that opcode: load the modulus, derive the Barrett
//! tag, load the base and exponent, run MODEXP and read the result back.
//!
//! Padding is deliberately absent. PKCS#1 v1.5, OAEP and PSS are encodings
//! over the primitive rather than hardware behaviour, and they belong in the
//! layer that owns the message.
//!
//! @addtogroup cc312_rsa_ap330P CC312 RSA - Modular Exponentiation
//! @ingroup apollo330P_hal
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! @name PKA register allocation for the exponentiation.
//!
//! N and NP are fixed by the hardware. The remaining four are the two
//! scratch registers the Barrett tag calculation needs plus the operands.
//! @{
//
//*****************************************************************************
#define RSA_REG_N       AM_HAL_PKA_REG_N     //!< Modulus.
#define RSA_REG_NP      AM_HAL_PKA_REG_NP    //!< Barrett tag.
#define RSA_REG_T       2                    //!< Scratch for the tag calc.
#define RSA_REG_T1      3                    //!< Scratch for the tag calc.
#define RSA_REG_BASE    4                    //!< Base operand.
#define RSA_REG_EXP     5                    //!< Exponent operand.
#define RSA_REG_RES     6                    //!< Result.
#define RSA_REG_COUNT   7                    //!< Registers to scrub on exit.
//! @}

//*****************************************************************************
//
// RSA primitive: result = base ^ exponent mod modulus.
//
//*****************************************************************************
uint32_t
am_hal_cc312_rsa_mod_exp(const uint8_t *modulus,
                         uint32_t modulusBytes,
                         const uint8_t *exponent,
                         const uint8_t *base,
                         uint8_t *result)
{
    uint32_t status;
    uint32_t modulusWords;
    uint32_t modulusBits;

    if ((modulus == NULL) || (exponent == NULL) ||
        (base == NULL) || (result == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // The PKA works in whole words, and the modulus has to fit the register
    // width the hardware was built with.
    //
    if ((modulusBytes == 0) || ((modulusBytes % 4) != 0) ||
        (modulusBytes > AM_HAL_RSA_MAX_MODULUS_BYTES))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    modulusWords = modulusBytes / 4U;
    modulusBits  = modulusBytes * 8U;

    //
    // An even modulus has no inverse to build the Barrett tag from, and a
    // modulus of one leaves nothing to reduce against.
    //
    if ((modulus[modulusBytes - 1U] & 0x01U) == 0U)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // am_hal_cc312_pka_init() enables the PKA clock and am_hal_cc312_pka_finish()
    // drops it again, so the clock is not managed here.
    //
    status = am_hal_cc312_pka_init(modulusBits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Load the modulus and derive the Barrett tag the PKA reduces with.
    //
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_REG_N,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            modulus, modulusWords);
    am_hal_cc312_pka_calc_np(AM_HAL_PKA_LEN_ID_N_BITS, modulusBits,
                             RSA_REG_N, RSA_REG_NP, RSA_REG_T, RSA_REG_T1);

    //
    // Load the operands and run the exponentiation.
    //
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_REG_BASE,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            base, modulusWords);
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_REG_EXP,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            exponent, modulusWords);

    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, RSA_REG_RES,
                       RSA_REG_BASE, RSA_REG_EXP);

    am_hal_cc312_pka_copy_reg_into_be_bytes(result, modulusWords, RSA_REG_RES);

    //
    // Scrub the operands; the private exponent passes through these.
    //
    am_hal_cc312_pka_clear_block_of_regs(RSA_REG_N, RSA_REG_COUNT);
    am_hal_cc312_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
