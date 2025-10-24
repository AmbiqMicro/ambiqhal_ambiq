
#
# Copyright (c) 2025 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

# Common header
zephyr_include_directories(port)

# DC header and source files
zephyr_include_directories_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMADC headers/include/tsi/NemaDC)
zephyr_include_directories_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMADC headers/NemaDC)

zephyr_library_sources_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMADC port/nema_dc_hal.c)

# GPU header and source files
zephyr_include_directories_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMAGFX headers/include/tsi/NemaGFX)
zephyr_include_directories_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMAGFX headers/include/tsi/NemaVG)
zephyr_include_directories_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMAGFX headers/NemaGFX/Nema)
zephyr_include_directories_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMAGFX extensions)

zephyr_library_sources_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMAGFX port/nema_hal.c)

zephyr_link_libraries_ifdef(CONFIG_AMBIQ_COMPONENT_USE_NEMAGFX ${CMAKE_CURRENT_SOURCE_DIR}/extensions/gpu_patch.a)

# Include the common library in project
zephyr_link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/libraries/lib_nema_${CONFIG_SOC_SERIES}_nemagfx.a)
