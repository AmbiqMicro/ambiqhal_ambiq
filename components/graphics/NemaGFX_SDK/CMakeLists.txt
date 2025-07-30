
#
# Copyright (c) 2025 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

# Extract current SOC and its family.
set(SOC_NAME ${CONFIG_SOC})
set(SOC_SERIES_NAME ${CONFIG_SOC_SERIES})

if(SOC_SERIES_NAME STREQUAL "apollo5x")
    set(NEMA_LIB_SOC apollo510)
# elseif(SOC_SERIES_NAME STREQUAL "apollo4x")
#     set(NEMA_LIB_SOC apollo4p)
else()
    message(FATAL_ERROR "Unsupported SOC_SERIES: ${SOC_SERIES_NAME}")
endif()

zephyr_include_directories(port)

include(include_sub_dirs.cmake)
# Include all subdirs
include_all_subdirs(headers)

# Include all used C sources.
zephyr_library_sources(
    port/nema_hal.c
    port/nema_dc_hal.c
    )

# Include the library in project
zephyr_library_import(nemasdk_lib ${CMAKE_CURRENT_SOURCE_DIR}/libraries/lib_nema_${NEMA_LIB_SOC}_nemagfx.a)
