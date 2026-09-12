//*****************************************************************************
//
//! @file am_hal_cc312_poly1305.c
//!
//! @brief Hardware abstraction for CryptoCell-312 Poly1305
//!
//! Poly1305 has no engine of its own. It is arithmetic in the field modulo
//! 2^130 - 5, so the accumulator runs on the PKA: every block becomes
//! Acc = (Acc + block) * r mod p, and the authenticator is Acc + s taken
//! modulo 2^128.
//!
//! The prime and its Barrett tag are constants here rather than derived,
//! matching the runtime, since the modulus never changes.
//!
//! @addtogroup cc312_poly1305_ap510 CC312 Poly1305 - One Time Authenticator
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! @name PKA register allocation for the accumulation.
//! @{
//
//*****************************************************************************
#define POLY_REG_PRIME      AM_HAL_PKA_REG_N   //!< 2^130 - 5.
#define POLY_REG_NP         AM_HAL_PKA_REG_NP  //!< Barrett tag for the prime.
#define POLY_REG_ACC        2                  //!< Running accumulator.
#define POLY_REG_KEY_R      3                  //!< Clamped multiplier r.
#define POLY_REG_KEY_S      4                  //!< Addend s.
#define POLY_REG_DATA       5                  //!< Current message block.
#define POLY_REG_COUNT      6                  //!< Registers to scrub on exit.
//! @}

//! Field size. The prime needs 130 bits, so five 32-bit words.
#define POLY_PRIME_BITS     130
#define POLY_PRIME_WORDS    5
//! A block plus the high bit occupies five words.
#define POLY_BLOCK_WORDS    5

//
//! The prime 2^130 - 5, little endian by word.
//
static const uint32_t g_am_hal_poly_prime[POLY_PRIME_WORDS] =
{
    0xfffffffb, 0xffffffff, 0xffffffff, 0xffffffff, 0x00000003
};

//
//! Clamp applied to r, per RFC 8439: the top four bits of each of the four
//! words are cleared, and the low two bits of the upper three.
//
static const uint32_t g_am_hal_poly_r_mask[4] =
{
    0x0fffffff, 0x0ffffffc, 0x0ffffffc, 0x0ffffffc
};

//*****************************************************************************
//
//! @brief Read a little endian 32-bit word from a byte buffer.
//
//*****************************************************************************
static uint32_t
poly_load_le32(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

//*****************************************************************************
//
//! @brief Write a little endian 32-bit word into a byte buffer.
//
//*****************************************************************************
static void
poly_store_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

//*****************************************************************************
//
//! @brief Fold one message block into the accumulator.
//!
//! Acc = (Acc + block) * r mod p. The block carries an extra set bit above
//! its data, which is what distinguishes a short final block from a full
//! one and stops trailing zeros from being absorbed silently.
//!
//! @param block     - Up to 16 bytes of message.
//! @param blockLen  - Number of message bytes in the block.
//
//*****************************************************************************
static void
poly_accumulate_block(const uint8_t *block, uint32_t blockLen)
{
    uint32_t words[POLY_BLOCK_WORDS];
    uint8_t  padded[AM_HAL_POLY1305_BLOCK_SIZE + 1];
    uint32_t i;

    memset(padded, 0, sizeof(padded));
    memcpy(padded, block, blockLen);

    //
    // Set the bit just above the message bytes.
    //
    padded[blockLen] = 1;

    for (i = 0; i < 4U; i++)
    {
        words[i] = poly_load_le32(&padded[i * 4U]);
    }
    words[4] = (uint32_t)padded[16];

    am_hal_cc312_pka_copy_data_into_reg(POLY_REG_DATA,
                                        AM_HAL_PKA_LEN_ID_MAX_BITS,
                                        words, POLY_BLOCK_WORDS);

    AM_HAL_PKA_MOD_ADD(AM_HAL_PKA_LEN_ID_N_BITS, POLY_REG_ACC,
                       POLY_REG_ACC, POLY_REG_DATA);
    AM_HAL_PKA_MOD_MUL(AM_HAL_PKA_LEN_ID_N_BITS, POLY_REG_ACC,
                       POLY_REG_ACC, POLY_REG_KEY_R);
}

//*****************************************************************************
//
//! @brief Fold a whole buffer into the accumulator, block by block.
//
//*****************************************************************************
static void
poly_accumulate(const uint8_t *data, uint32_t length)
{
    uint32_t offset = 0;

    while (offset < length)
    {
        uint32_t chunk = length - offset;

        if (chunk > AM_HAL_POLY1305_BLOCK_SIZE)
        {
            chunk = AM_HAL_POLY1305_BLOCK_SIZE;
        }

        poly_accumulate_block(&data[offset], chunk);
        offset += chunk;
    }
}

//*****************************************************************************
//
//! @brief Set up the PKA and load the key for an accumulation.
//
//*****************************************************************************
static uint32_t
poly_start(const uint8_t *key)
{
    uint32_t status;
    uint32_t r[4];
    uint32_t s[4];
    uint32_t i;

    status = am_hal_cc312_pka_init(POLY_PRIME_BITS);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Load the prime and derive Barrett Np (same path as X25519 / Ed25519).
    // A hardcoded tag used to be loaded with a 3-word LE copy that left the
    // PKA half-word dirty - calc_np clears and builds Np correctly.
    //
    am_hal_cc312_pka_copy_data_into_reg(POLY_REG_PRIME,
                                        AM_HAL_PKA_LEN_ID_MAX_BITS,
                                        g_am_hal_poly_prime,
                                        POLY_PRIME_WORDS);
    am_hal_cc312_pka_calc_np(AM_HAL_PKA_LEN_ID_N_BITS, POLY_PRIME_BITS,
                             POLY_REG_PRIME, POLY_REG_NP,
                             POLY_REG_DATA, POLY_REG_ACC);

    //
    // r is clamped before use; s is taken as it stands.
    //
    for (i = 0; i < 4U; i++)
    {
        r[i] = poly_load_le32(&key[i * 4U]) & g_am_hal_poly_r_mask[i];
        s[i] = poly_load_le32(&key[16U + (i * 4U)]);
    }

    am_hal_cc312_pka_copy_data_into_reg(POLY_REG_KEY_R,
                                        AM_HAL_PKA_LEN_ID_MAX_BITS, r, 4);
    am_hal_cc312_pka_copy_data_into_reg(POLY_REG_KEY_S,
                                        AM_HAL_PKA_LEN_ID_MAX_BITS, s, 4);

    AM_HAL_PKA_CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, POLY_REG_ACC);
    AM_HAL_PKA_CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, POLY_REG_DATA);
    AM_HAL_PKA_CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, AM_HAL_PKA_REG_T0);
    AM_HAL_PKA_CLEAR(AM_HAL_PKA_LEN_ID_MAX_BITS, AM_HAL_PKA_REG_T1);

    am_hal_cc312_context_clear(r, sizeof(r));
    am_hal_cc312_context_clear(s, sizeof(s));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Finish an accumulation and emit the authenticator.
//
//*****************************************************************************
static void
poly_finish(uint8_t *mac)
{
    uint32_t acc[POLY_BLOCK_WORDS];
    uint32_t i;

    //
    // The final addition of s is a plain add: the result is taken modulo
    // 2^128 by keeping only the low four words. Use the full register length
    // so a carry out of bit 129 is not lost before that truncation.
    //
    AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_MAX_BITS, POLY_REG_ACC,
                   POLY_REG_ACC, POLY_REG_KEY_S);

    am_hal_cc312_pka_copy_data_from_reg(acc, POLY_BLOCK_WORDS, POLY_REG_ACC);

    for (i = 0; i < 4U; i++)
    {
        poly_store_le32(&mac[i * 4U], acc[i]);
    }

    am_hal_cc312_context_clear(acc, sizeof(acc));

    am_hal_cc312_pka_clear_block_of_regs(POLY_REG_PRIME, POLY_REG_COUNT);
    am_hal_cc312_pka_finish();
}

//*****************************************************************************
//
// Compute a Poly1305 authenticator over a message.
//
//*****************************************************************************
uint32_t
am_hal_cc312_poly1305(const uint8_t *key,
                      const uint8_t *input,
                      uint32_t length,
                      uint8_t *mac)
{
    uint32_t status;
    uint8_t keyLocal[AM_HAL_POLY1305_KEY_SIZE];

    if ((key == NULL) || (mac == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((length != 0) && (input == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Clamp/load from SRAM. Keys are often const in MRAM.
    //
    memcpy(keyLocal, key, AM_HAL_POLY1305_KEY_SIZE);

    status = poly_start(keyLocal);
    am_hal_cc312_context_clear(keyLocal, sizeof(keyLocal));
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    poly_accumulate(input, length);
    poly_finish(mac);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Compute a Poly1305 authenticator in the ChaCha20-Poly1305 layout.
//
//*****************************************************************************
uint32_t
am_hal_cc312_poly1305_aead(const uint8_t *key,
                           const uint8_t *aad,
                           uint32_t aadLen,
                           const uint8_t *input,
                           uint32_t length,
                           uint8_t *mac)
{
    uint32_t status;
    uint8_t lengths[AM_HAL_POLY1305_BLOCK_SIZE];
    uint8_t keyLocal[AM_HAL_POLY1305_KEY_SIZE];

    if ((key == NULL) || (mac == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (((aadLen != 0) && (aad == NULL)) ||
        ((length != 0) && (input == NULL)))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memcpy(keyLocal, key, AM_HAL_POLY1305_KEY_SIZE);

    status = poly_start(keyLocal);
    am_hal_cc312_context_clear(keyLocal, sizeof(keyLocal));
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Each part is authenticated independently, and because a short final
    // block is padded rather than merged, the two are separated exactly as
    // the padding rule in RFC 8439 describes.
    //
    poly_accumulate(aad, aadLen);
    poly_accumulate(input, length);

    //
    // The trailing block carries both lengths as 64 bit little endian
    // values, which is what binds the two parts together.
    //
    memset(lengths, 0, sizeof(lengths));
    poly_store_le32(&lengths[0], aadLen);
    poly_store_le32(&lengths[8], length);

    poly_accumulate_block(lengths, AM_HAL_POLY1305_BLOCK_SIZE);

    poly_finish(mac);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
