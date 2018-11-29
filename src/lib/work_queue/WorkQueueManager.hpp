/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <containers/BlockingQueue.hpp>
#include <containers/List.hpp>
#include <containers/Queue.hpp>

#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>

extern "C" __EXPORT int wq_manager_main(int argc, char *argv[]);

namespace px4
{

// work queues

// SPI1 : PRIORITY MAX : STACK 750


enum PX4_WQS {
	SPI1 = 0,
	SPI2,

	I2C1,
	I2C2,

	rate_ctrl,
	hp_default,

	test1,
	test2,
};

struct wq_config {
	const char *name;
	int16_t priority;
	uint16_t stacksize;
};

// TODO: set priorities appropriately for linux, macos, qurt (unify with px4_tasks.h)
static constexpr wq_config wq_configurations[] = {

	[SPI1] = { "wq:SPI1", 255, 1200 },
	[SPI2] = { "wq:SPI2", 255, 1200 },

	[I2C1] = { "wq:I2C1", 250, 1000 },
	[I2C2] = { "wq:I2C2", 250, 1000 },

	[rate_ctrl] = { "wq:rate_ctrl", 254, 1000 },
	[hp_default] = { "wq:hp_default", 249, 1000 },

	[test1] = { "wq:test1", 255, 800 },
	[test2] = { "wq:test2", 255, 800 },

};

class WorkQueue;

// list of all px4 work queues
extern pthread_mutex_t px4_work_queues_list_mutex;
extern List<WorkQueue *> px4_work_queues_list;

extern BlockingQueue<wq_config *, 1> px4_wq_manager_new;

class WorkQueueManager : public ModuleBase<WorkQueueManager>
{

public:

	WorkQueueManager() = default;
	~WorkQueueManager() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static WorkQueueManager *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	static void task_main_trampoline(int argc, char *argv[]);

	int print_status() override;

private:
	void create_work_queue_thread(wq_config *wq);

	static void *work_queue_runner(void *context);

};

WorkQueue *work_queue_create(const wq_config &new_wq);

// helper for drivers (TODO: find a better home for this)
const wq_config &device_bus_to_wq(uint32_t device_id);

} // namespace px4
