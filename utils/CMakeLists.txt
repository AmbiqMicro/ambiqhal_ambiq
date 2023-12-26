# Copyright (c) 2023 Ambiq Micro Inc. 

zephyr_library()

if(CONFIG_BT_AMOTA)
    zephyr_library_sources(am_util_bootloader.c)
    zephyr_library_sources(am_util_multi_boot.c)
endif()
