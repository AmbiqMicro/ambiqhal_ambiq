//*****************************************************************************
//
//! @file am_util_multi_boot_private.h
//!
//! @brief Internal definitions/structures shared within multiboot
//!
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_MULTI_BOOT_PRIVATE_H
#define AM_MULTI_BOOT_PRIVATE_H

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
// Include config before other bootloader files
#ifdef AM_MULTIBOOT_CONFIG_FILE
#include AM_MULTIBOOT_CONFIG_FILE
#endif

#include "am_util_bootloader.h"

#ifdef MULTIBOOT_SECURE
#include "am_util_multi_boot_secure.h"
#endif

//*****************************************************************************
//
// I2C Address to use
//
//*****************************************************************************
#ifndef I2C_SLAVE_ADDR
#define I2C_SLAVE_ADDR                     0x10
#endif

//*****************************************************************************
//
// Run without flag page.
//
//*****************************************************************************
#ifndef USE_FLAG_PAGE
#define USE_FLAG_PAGE                       0
#endif

//*****************************************************************************
//
// Location of the flag page.
//
//*****************************************************************************
#ifndef FLAG_PAGE_LOCATION
#define FLAG_PAGE_LOCATION                 0x00004000
#endif

//*****************************************************************************
//
// Max Size of Bootloader.
//
//*****************************************************************************
// The value here must match (at least) with the ROLength restriction imposed at
// bootloader linker configuration
#ifndef MAX_BOOTLOADER_SIZE
#define MAX_BOOTLOADER_SIZE                0x00004000
#endif
// The value here must match (at least) with the RWLength restriction imposed at
// bootloader linker configuration
#ifndef MAX_SRAM_USED
#define MAX_SRAM_USED                      0x00004000
#endif

extern am_util_bootloader_image_t *g_psBootImage;

//*****************************************************************************
//
// Safety Checks.
//
//*****************************************************************************
#if USE_FLAG_PAGE == 1
#if FLAG_PAGE_LOCATION & (AM_HAL_FLASH_PAGE_SIZE - 1)
#error "Flag Page address not page aligned"
#endif
#if FLAG_PAGE_LOCATION < MAX_BOOTLOADER_SIZE
#error "Flag Page overlaps with Bootloader"
#endif
#endif

//*****************************************************************************
//
// Default settings.
//
//*****************************************************************************
#ifndef DEFAULT_LINK_ADDRESS
#define DEFAULT_LINK_ADDRESS                ((uint32_t *) 0x00008000)
#endif
// Default override configured as invalid
#ifndef DEFAULT_OVERRIDE_GPIO
#define DEFAULT_OVERRIDE_GPIO               (0xFFFFFFFF)
#endif
#ifndef DEFAULT_OVERRIDE_POLARITY
#define DEFAULT_OVERRIDE_POLARITY           AM_BOOTLOADER_OVERRIDE_LOW
#endif

//*****************************************************************************
//
// Boot Loader Version Number
//
//*****************************************************************************
#define AM_BOOTLOADER_VERSION_NUM           0x00000001

//*****************************************************************************
//
// Boot messages.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK                   0x00000000
#define AM_BOOTLOADER_NAK                   0x00000001
#define AM_BOOTLOADER_READY                 0x00000002
#define AM_BOOTLOADER_IMAGE_COMPLETE        0x00000003
#define AM_BOOTLOADER_BAD_CRC               0x00000004
#define AM_BOOTLOADER_ERROR                 0x00000005
#define AM_BOOTLOADER_BL_VERSION            0x00000006
#define AM_BOOTLOADER_FW_VERSION            0x00000007

//*****************************************************************************
//
// Boot Commands.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK_CMD               0x00000000
#define AM_BOOTLOADER_NAK_CMD               0x00000001
#define AM_BOOTLOADER_NEW_IMAGE             0x00000002
#define AM_BOOTLOADER_NEW_PACKET            0x00000003
#define AM_BOOTLOADER_RESET                 0x00000004
#define AM_BOOTLOADER_SET_OVERRIDE_CMD      0x00000005
#define AM_BOOTLOADER_BL_VERSION_CMD        0x00000006
#define AM_BOOTLOADER_FW_VERSION_CMD        0x00000007
#define AM_BOOTLOADER_NEW_ENCRYPTED_IMAGE   0x00000008
#define AM_BOOTLOADER_RESTART               0x00000009

//*****************************************************************************
//
// Globals to keep track of the image write state.
//
//*****************************************************************************
extern uint32_t g_ui32BytesReceived;
extern uint32_t g_ui32CRC;

//*****************************************************************************
//
// Image structure to hold data about the downloaded boot image.
//
//*****************************************************************************
extern am_util_bootloader_image_t g_sImage;

//*****************************************************************************
//
// Function declarations.
//
//*****************************************************************************
extern bool
image_start_packet_read(am_util_bootloader_image_t *psImage, uint32_t *pui32Packet);
extern void
image_data_packet_read(uint8_t *pui8Src, uint32_t ui32Size);
extern void
program_image(uint32_t bEncrypted);

#ifdef MULTIBOOT_SECURE
extern void wipe_sram(void);
#endif

#endif // AM_MULTI_BOOT_PRIVATE_H
