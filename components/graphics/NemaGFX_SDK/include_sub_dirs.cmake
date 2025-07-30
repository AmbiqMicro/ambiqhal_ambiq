# Copyright (c) 2025 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0

# The function to collect all subdirs
function(collect_all_subdirs BASE_DIR OUT_VAR)
    file(GLOB children LIST_DIRECTORIES true RELATIVE ${BASE_DIR} ${BASE_DIR}/*)
    foreach(child ${children})
        set(full_path "${BASE_DIR}/${child}")
        if(IS_DIRECTORY "${full_path}")
            list(APPEND ${OUT_VAR} "${full_path}")
            collect_all_subdirs("${full_path}" ${OUT_VAR})
        endif()
    endforeach()
    set(${OUT_VAR} "${${OUT_VAR}}" PARENT_SCOPE)
endfunction()

# The function to include all subdirs
function(include_all_subdirs base_dir)
    set(all_dirs "")
    collect_all_subdirs("${CMAKE_CURRENT_SOURCE_DIR}/${base_dir}" all_dirs)

    foreach(dir ${all_dirs})
        # printing the subdirs for debug
        # message(STATUS "Include dir: ${dir}")
        zephyr_include_directories(${dir})
    endforeach()
endfunction()
