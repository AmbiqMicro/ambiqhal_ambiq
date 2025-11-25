# Ambiq HAL
#
# Copyright (c) 2023 Antmicro Ltd <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_APOLLO4P OR CONFIG_SOC_APOLLO4P_BLUE)
    zephyr_include_directories(apollo4p)
    add_subdirectory(apollo4p)
elseif(CONFIG_SOC_APOLLO3P OR CONFIG_SOC_APOLLO3P_BLUE)
    zephyr_include_directories(apollo3p)
    add_subdirectory(apollo3p)
elseif(CONFIG_SOC_APOLLO3 OR CONFIG_SOC_APOLLO3_BLUE)
    zephyr_include_directories(apollo3)
    add_subdirectory(apollo3)
elseif(CONFIG_SOC_APOLLO510)
    zephyr_include_directories(apollo510)
    add_subdirectory(apollo510)
elseif(CONFIG_SOC_APOLLO2)
    zephyr_include_directories(apollo2)
    add_subdirectory(apollo2)
endif()
