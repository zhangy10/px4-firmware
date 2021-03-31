/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "px4muorb.hpp"
#include "uORBProtobufChannel.hpp"

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <dspal_platform.h>
#include "uORB.h"
#include <parameters/param.h>
#include <px4_platform_common/shmem.h>
#include <px4_platform_common/log.h>

/*update value and param's change bit in shared memory*/
int px4muorb_param_update_to_shmem(uint32_t param, const uint8_t *value,
				   int data_len_in_bytes)
{
	unsigned int byte_changed, bit_changed;
	union param_value_u *param_value = (union param_value_u *) value;

	if (!shmem_info_p) {
		init_shared_memory();
	}

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_INFO("Could not get shmem lock\n");
		return -1;
	}

	shmem_info_p->params_val[param] = *param_value;

	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->krait_changed_index[byte_changed] |= bit_changed;

	release_shmem_lock(__FILE__, __LINE__);

	return 0;
}

int px4muorb_param_update_index_from_shmem(unsigned char *data, int data_len_in_bytes)
{
	if (!shmem_info_p) {
		return -1;
	}

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_INFO("Could not get shmem lock\n");
		return -1;
	}

	for (int i = 0; i < data_len_in_bytes; i++) {
		data[i] = shmem_info_p->adsp_changed_index[i];
	}

	release_shmem_lock(__FILE__, __LINE__);

	return 0;
}

int px4muorb_param_update_value_from_shmem(uint32_t param, const uint8_t *value,
		int data_len_in_bytes)
{
	unsigned int byte_changed, bit_changed;
	union param_value_u *param_value = (union param_value_u *) value;

	if (!shmem_info_p) {
		return -1;
	}

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_INFO("Could not get shmem lock\n");
		return -1;
	}

	*param_value = shmem_info_p->params_val[param];

	/*also clear the index since we are holding the lock*/
	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->adsp_changed_index[byte_changed] &= ~bit_changed;

	release_shmem_lock(__FILE__, __LINE__);

	return 0;
}
