//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

/*******************************************************************************
 * Copyright (c) 2022 Think Silicon S.A.
 *
   Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this header file and/or associated documentation files to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Materials, and to permit persons to whom the Materials are furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Materials.
 *
 * MODIFICATIONS TO THIS FILE MAY MEAN IT NO LONGER ACCURATELY REFLECTS
 * NEMAGFX API. THE UNMODIFIED, NORMATIVE VERSIONS OF THINK-SILICON NEMAGFX
 * SPECIFICATIONS AND HEADER INFORMATION ARE LOCATED AT:
 *   https://think-silicon.com/products/software/nemagfx-api
 *
 *  The software is provided 'as is', without warranty of any kind, express or
 *  implied, including but not limited to the warranties of merchantability,
 *  fitness for a particular purpose and noninfringement. In no event shall
 *  Think Silicon S.A. be liable for any claim, damages or other liability, whether
 *  in an action of contract, tort or otherwise, arising from, out of or in
 *  connection with the software or the use or other dealings in the software.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/pm/device_runtime.h>

#include "nema_hal.h"
#include "nema_regs.h"
#include "nema_vg.h"
#include "nema_ringbuffer.h"
#include "nema_error.h"
#include "nema_vg_context.h"

// nema_hal.c serves as the hardware interface layer for the NemaSDK.
// It calls low-level APIs provided in gpu.c to operate hardware registers
// and retrieve hardware operational status.
#include "gpu.h"

LOG_MODULE_REGISTER(nemagfx, CONFIG_NEMAGFX_LOG_LEVEL);

// Get the device pointer for the GPU using its devicetree node label
#define GPU_NODE DT_CHOSEN(ambiq_gpu)
static const struct device *gpu_dev = DEVICE_DT_GET(GPU_NODE);

// Define memory sections for different heap types based on Kconfig
#if CONFIG_NEMAGFX_ASSETS_HEAP_IN_PSRAM
#define ASSETS_HEAP_ATTR                                                                           \
	Z_GENERIC_SECTION(LINKER_DT_NODE_REGION_NAME_TOKEN(DT_CHOSEN(ambiq_external_ram_region)))
#else
#define ASSETS_HEAP_ATTR
#endif

#if CONFIG_NEMAGFX_RENDER_HEAP_IN_NONCACHE_SRAM
#define RENDER_HEAP_ATTR Z_GENERIC_SECTION(SRAM_NO_CACHE)
#else
#define RENDER_HEAP_ATTR
#endif

#if CONFIG_NEMAGFX_CPU_ONLY_HEAP_IN_DTCM
#define CPU_ONLY_HEAP_ATTR Z_GENERIC_SECTION(DTCM)
#else
#define CPU_ONLY_HEAP_ATTR
#endif

// Define heap buffers and structures
static uint8_t assets_buffer[CONFIG_NEMAGFX_HEAP_SIZE_ASSETS * 1024] __aligned(8) ASSETS_HEAP_ATTR;
static struct k_heap assets_heap;

static uint8_t render_buffer[CONFIG_NEMAGFX_HEAP_SIZE_RENDER * 1024] __aligned(8) RENDER_HEAP_ATTR;
static struct k_heap render_heap;

static uint8_t cpu_only_buffer[CONFIG_NEMAGFX_HEAP_SIZE_CPU_ONLY * 1024]
	__aligned(8) CPU_ONLY_HEAP_ATTR;
static struct k_heap cpu_only_heap;

// Define the ring buffer for GPU command lists
#define RING_BUFFER_LENGTH ((CONFIG_NEMAGFX_MAX_PENDING_COMMAND_LIST * 12 + 16) * 4)
static uint8_t ring_buffer[RING_BUFFER_LENGTH] __aligned(8) Z_GENERIC_SECTION(SRAM_NO_CACHE);

static nema_ringbuffer_t ring_buffer_str = {
	.bo.base_phys = (uintptr_t)ring_buffer,
	.bo.base_virt = (void *)ring_buffer,
	.bo.size = RING_BUFFER_LENGTH,
	.bo.fd = -1,
	.last_submission_id = -1,
	.offset = 0,
};

int32_t nema_sys_init(void)
{
	// Initialize the various memory heaps used by the graphics SDK
	k_heap_init(&cpu_only_heap, cpu_only_buffer, sizeof(cpu_only_buffer));
	cpu_only_heap.heap.init_mem = cpu_only_buffer;
	cpu_only_heap.heap.init_bytes = sizeof(cpu_only_buffer);

	k_heap_init(&render_heap, render_buffer, sizeof(render_buffer));
	render_heap.heap.init_mem = render_buffer;
	render_heap.heap.init_bytes = sizeof(render_buffer);

#if CONFIG_NEMAGFX_ASSETS_HEAP_IN_PSRAM
	pm_device_runtime_get(DEVICE_DT_GET(DT_CHOSEN(ambiq_psram)));
#endif

	k_heap_init(&assets_heap, assets_buffer, sizeof(assets_buffer));
	assets_heap.heap.init_mem = assets_buffer;
	assets_heap.heap.init_bytes = sizeof(assets_buffer);

	// Initialize the GPU command list ring buffer
	nema_rb_init(&ring_buffer_str, 1);

	return 0;
}

bool nema_sdk_initialized(void)
{
	return (ring_buffer_str.last_submission_id != -1);
}

int nema_wait_irq_cl(int cl_id)
{
	int ret;
	// Set a generous retry counter to prevent infinite loops.
	int max_retries = CONFIG_NEMAGFX_WAIT_GPU_IRQ_TIMEOUT_MS * 2;

	while (gpu_ambiq_get_last_cl_id(gpu_dev) < cl_id) {
		ret = nema_wait_irq();
		if (ret != 0) {
			// If waiting for the interrupt itself timed out, return the error.
			LOG_ERR("nema_wait_irq timed out while waiting for CL ID %d", cl_id);
			return ret;
		}

		if (--max_retries == 0) {
			LOG_ERR("Timeout waiting for command list ID %d to complete. Last "
				"completed ID was %d.",
				cl_id, gpu_ambiq_get_last_cl_id(gpu_dev));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

int nema_wait_irq(void)
{
	return gpu_ambiq_wait_interrupt(gpu_dev, CONFIG_NEMAGFX_WAIT_GPU_IRQ_TIMEOUT_MS);
}

uint32_t nema_reg_read(uint32_t reg)
{
	return gpu_ambiq_reg_read(gpu_dev, reg);
}

void nema_reg_write(uint32_t reg, uint32_t value)
{
	gpu_ambiq_reg_write(gpu_dev, reg, value);
}

nema_buffer_t nema_buffer_create(int size)
{
	return nema_buffer_create_pool(NEMA_MEM_POOL_CL, size);
}

nema_buffer_t nema_buffer_create_pool(int pool, int size)
{
	nema_buffer_t bo;
	bo.base_virt = NULL;
	bo.base_phys = 0;
	bo.size = 0;
	bo.fd = -1;

	if (size <= 0) {
		return bo;
	}

	uint32_t alloc_size;
	struct k_heap *target_heap;
	bool align;

	switch (pool) {
	case NEMA_MEM_POOL_FB:
	case NEMA_MEM_POOL_CL:
		alloc_size = (size + 31) & (~31);
		target_heap = &render_heap;
		align = true;
		break;
	case NEMA_MEM_POOL_ASSETS:
		alloc_size = (size + 31) & (~31);
		target_heap = &assets_heap;
		align = true;
		break;
	case NEMA_MEM_POOL_CLIPPED_PATH:
		alloc_size = size;
		target_heap = &cpu_only_heap;
		align = false;
		break;
	default:
		LOG_ERR("Unsupported memory pool ID: %d", pool);
		return bo;
	}

	if (target_heap == NULL) {
		LOG_ERR("Target heap for pool ID %d is not initialized.", pool);
		return bo;
	}

	if (align) {
		bo.base_virt = k_heap_aligned_alloc(target_heap, 32, alloc_size, K_NO_WAIT);
	} else {
		bo.base_virt = k_heap_alloc(target_heap, alloc_size, K_NO_WAIT);
	}

	if (bo.base_virt == NULL) {
		LOG_ERR("Failed to allocate %u bytes from pool %d", alloc_size, pool);
		// Reset structure to a clean state on failure
		return bo;
	}

	bo.base_phys = (uintptr_t)bo.base_virt;
	bo.fd = pool;
	bo.size = alloc_size;

	return bo;
}

void *nema_buffer_map(nema_buffer_t *bo)
{
	return bo->base_virt;
}

void nema_buffer_unmap(nema_buffer_t *bo)
{
	// This is a no-op in this HAL implementation as memory is always mapped.
}

void nema_buffer_destroy(nema_buffer_t *bo)
{
	struct k_heap *target_heap = NULL;

	if (bo == NULL || bo->base_virt == NULL) {
		return;
	}

	switch (bo->fd) {
	case NEMA_MEM_POOL_FB:
	case NEMA_MEM_POOL_CL:
		target_heap = &render_heap;
		break;
	case NEMA_MEM_POOL_ASSETS:
		target_heap = &assets_heap;
		break;
	case NEMA_MEM_POOL_CLIPPED_PATH:
		target_heap = &cpu_only_heap;
		break;
	default:
		LOG_ERR("Cannot destroy buffer: Invalid pool ID %d", bo->fd);
		return;
	}

	if (target_heap == NULL) {
		LOG_ERR("Cannot destroy buffer: Pool %d is not initialized.", bo->fd);
		return;
	}

	k_heap_free(target_heap, bo->base_virt);

	// Reset buffer structure to prevent dangling pointers
	bo->base_virt = NULL;
	bo->base_phys = 0;
	bo->size = 0;
	bo->fd = -1;
}

uintptr_t nema_buffer_phys(nema_buffer_t *bo)
{
	return bo->base_phys;
}

void nema_buffer_flush(nema_buffer_t *bo)
{
	if (!buf_in_nocache(bo->base_phys, bo->size)) {
		sys_cache_data_flush_range(bo->base_virt, bo->size);
	}
	__DSB();
}

void nema_buffer_invalidate(nema_buffer_t *bo)
{
	if (!buf_in_nocache(bo->base_phys, bo->size)) {
		sys_cache_data_invd_range(bo->base_virt, bo->size);
	}
	__DSB();
}

bool nema_buffer_is_within_pool(int pool, uint32_t buf_start, uint32_t buf_length)
{
	struct k_heap *target_heap;
	switch (pool) {
	case NEMA_MEM_POOL_FB:
	case NEMA_MEM_POOL_CL:
		target_heap = &render_heap;
		break;
	case NEMA_MEM_POOL_ASSETS:
		target_heap = &assets_heap;
		break;
	case NEMA_MEM_POOL_CLIPPED_PATH:
		target_heap = &cpu_only_heap;
		break;
	default:
		return false;
	}

	if (!target_heap) {
		return false;
	}

	uintptr_t heap_start = (uintptr_t)target_heap->heap.init_mem;
	uintptr_t heap_end = heap_start + target_heap->heap.init_bytes;

	if ((buf_start >= heap_start) && ((buf_start + buf_length) <= heap_end)) {
		return true;
	}

	return false;
}

void *nema_host_malloc(size_t size)
{
	return k_heap_alloc(&cpu_only_heap, size, K_NO_WAIT);
}

void nema_host_free(void *ptr)
{
	k_heap_free(&cpu_only_heap, ptr);
}

int nema_mutex_lock(int mutex_id)
{
	// This HAL implementation assumes single-threaded access to the NemaGFX API.
	// If multi-threaded access is required, a k_mutex should be implemented here.
	return 0;
}

int nema_mutex_unlock(int mutex_id)
{
	// This HAL implementation assumes single-threaded access to the NemaGFX API.
	return 0;
}

am_hal_status_e nema_get_cl_status(int32_t cl_id)
{
	// A negative command list ID is always considered completed.
	if (cl_id < 0) {
		return AM_HAL_STATUS_SUCCESS;
	}

	// If the GPU is idle, all submitted command lists are completed.
	if (nema_reg_read(NEMA_STATUS) == 0) {
		return AM_HAL_STATUS_SUCCESS;
	}

	// Check if the last completed ID is greater than or equal to the requested ID.
	int last_completed_cl_id = (int)nema_reg_read(NEMA_CLID);
	if (last_completed_cl_id >= cl_id) {
		return AM_HAL_STATUS_SUCCESS;
	}

	return AM_HAL_STATUS_IN_USE;
}

bool nema_rb_check_full(void)
{
	uint32_t total_pending_cl = 0;
	int last_executed_cl_id = gpu_ambiq_get_last_cl_id(gpu_dev);

	if (ring_buffer_str.last_submission_id > last_executed_cl_id) {
		total_pending_cl = ring_buffer_str.last_submission_id - last_executed_cl_id;
	} else if (ring_buffer_str.last_submission_id < last_executed_cl_id) {
		// This handles the wrap-around case for the command list IDs.
		if (last_executed_cl_id == 0xFFFFFF) {
			total_pending_cl = ring_buffer_str.last_submission_id + 1;
		} else {
			// This case should ideally not be reached if CL IDs wrap around correctly.
			LOG_WRN("Unexpected CL ID wrap-around state.");
		}
	}

	return (total_pending_cl >= CONFIG_NEMAGFX_MAX_PENDING_COMMAND_LIST);
}

int nema_get_last_cl_id(void)
{
	return gpu_ambiq_get_last_cl_id(gpu_dev);
}

int nema_get_last_submission_id(void)
{
	return ring_buffer_str.last_submission_id;
}

//
// Currently, GPU power management code exists in both nema_hal.c and gpu.c.
// This is a temporary compromise to maintain compatibility with LVGL.
// In the future, we plan to upgrade the power management code within LVGL.
// LVGL calls power operations in nema_hal.c, which then calls get/put and 
// triggers pm action in gpu.c. Additionally, nema_hal.c is responsible 
// for software-level NemaSDK reinitialization.
//
//*****************************************************************************
//
//! @brief Controls the power state of the GPU peripheral.
//!
//! @param ePowerState - The desired power state (e.g., wake, normal sleep, deep sleep).
//! @param bRetainState - Indicates whether to reinitialize the NemaSDK and NemaVG
//!                       when waking up the GPU peripheral.
//!
//! This function manages the power state of the GPU peripheral based on
//! the requested power state. It checks the current power status and either
//! powers up or powers down the peripheral as needed.
//!
//! When waking up the GPU (`AM_HAL_SYSCTRL_WAKE`), if \e bRetainState is true,
//! the function reinitializes the NemaSDK and NemaVG. If powering down, the
//! function ensures the peripheral is inactive before disabling power.
//!
//! \e ePowerState The desired power state. Valid values are:
//! - AM_HAL_SYSCTRL_WAKE: Wake up the peripheral.
//! - AM_HAL_SYSCTRL_NORMALSLEEP: Put the peripheral in normal sleep mode.
//! - AM_HAL_SYSCTRL_DEEPSLEEP: Put the peripheral in deep sleep mode.
//!
//! \e bRetainState Determines whether to reinitialize the NemaSDK and NemaVG
//! when powering up. This is used for scenarios where the GPU context needs
//! to be retained.
//!
//! @note Ensure that the GPU peripheral is not in use before attempting to
//! power it down to avoid conflicts. This API is not thread-safe. If it is
//! called from multiple threads or from an interrupt, appropriate critical
//! section protection must be added.
//!
//! @return Returns the status of the operation. Possible return values are:
//! - AM_HAL_STATUS_SUCCESS: Operation was successful.
//! - AM_HAL_STATUS_IN_USE: Peripheral is currently in use and cannot be powered down.
//! - AM_HAL_STATUS_INVALID_OPERATION: Invalid power state requested.
//! - AM_HAL_STATUS_FAIL: Reinitialization of NemaSDK or NemaVG failed.
//
//*****************************************************************************
uint32_t nemagfx_power_control(am_hal_sysctrl_power_state_e ePowerState, bool bRetainState)
{
	uint32_t ui32Status;
	bool bEnabled;
	uint32_t ui32ErrorCode;

	//
	// Check the current GPU power state
	//
	ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_GFX, &bEnabled);
	if (ui32Status == AM_HAL_STATUS_SUCCESS) {
		//
		// Decode the requested power state and take action
		//
		switch (ePowerState) {
		case AM_HAL_SYSCTRL_WAKE: {
			if (bEnabled) {
				//
				// GPU is already powered up
				//
				ui32Status = AM_HAL_STATUS_SUCCESS;
			} else {
				//
				// Enable power control
				//
				ui32Status =
					am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);

				//
				// Reinitialize NemaSDK and NemaVG
				//
				if (ui32Status == AM_HAL_STATUS_SUCCESS && bRetainState) {

					nema_get_error(); // clear raster graphics error

					gpu_ambiq_reset_last_cl_id(gpu_dev);
					nema_reinit();
					ui32ErrorCode = nema_get_error();
					if (ui32ErrorCode != NEMA_ERR_NO_ERROR) {
						ui32Status = AM_HAL_STATUS_FAIL;
						break;
					}

#ifndef NEMA_MULTI_THREAD
					// If NEMA_MULTI_THREAD is not defined, the nemavg_context
					// is defined as global variable, and if nema_vg_init is not
					// called, nemavg_context will be initialized as NULL, so we
					// should prevent to write to the NULL pointer.
					struct nema_vg_context_t_;
					extern struct nema_vg_context_t_ *nemavg_context;

					if (nemavg_context != NULL) {
#endif
						nema_vg_get_error(); // clear vector graphics error

						nema_vg_reinit();
						ui32ErrorCode = nema_vg_get_error();
						if (ui32ErrorCode != NEMA_VG_ERR_NO_ERROR) {
							ui32Status = AM_HAL_STATUS_FAIL;
							break;
						}
#ifndef NEMA_MULTI_THREAD
					}
#endif
				}
			}
			break;
		}
		case AM_HAL_SYSCTRL_NORMALSLEEP:
		case AM_HAL_SYSCTRL_DEEPSLEEP: {
			if (!bEnabled) {
				//
				// GPU is already powered down
				//
				ui32Status = AM_HAL_STATUS_SUCCESS;
			} else {
				//
				// Ensure GPU is currently inactive
				//
				if (nema_reg_read(NEMA_STATUS) != 0) {
					ui32Status = AM_HAL_STATUS_IN_USE;
				} else {
					//
					// Power down the GPU
					//
					ui32Status = am_hal_pwrctrl_periph_disable(
						AM_HAL_PWRCTRL_PERIPH_GFX);
				}
			}
			break;
		}
		default: {
			ui32Status = AM_HAL_STATUS_INVALID_OPERATION;
			break;
		}
		}
	}

	return ui32Status;
}
