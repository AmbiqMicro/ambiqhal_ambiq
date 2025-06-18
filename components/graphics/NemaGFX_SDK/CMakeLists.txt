
#
# Copyright (c) 2025 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_include_directories(port)
zephyr_include_directories(headers/common/mem)
zephyr_include_directories(headers/include/tsi/common)
zephyr_include_directories(headers/include/tsi/NemaDC)
zephyr_include_directories(headers/include/tsi/NemaGFX)
zephyr_include_directories(headers/include/tsi/NemaVG)
zephyr_include_directories(headers/NemaDC)
zephyr_include_directories(headers/NemaGFX/Nema)

# Include all used C sources.
zephyr_library_sources(
    port/nema_hal.c
    port/nema_dc_hal.c
    )

# Include the library in project
zephyr_link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/libraries/lib_nema_${CONFIG_SOC_SERIES}_nemagfx.a)
