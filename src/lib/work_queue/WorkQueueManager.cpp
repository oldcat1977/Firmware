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

#include "WorkQueueManager.hpp"

#include "WorkQueue.hpp"

#include <string.h>

#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <drivers/drv_hrt.h>

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;

namespace px4
{

List<WorkQueue *> px4_work_queues_list;
pthread_mutex_t px4_work_queues_list_mutex = PTHREAD_MUTEX_INITIALIZER;

BlockingQueue<wq_config *, 1> px4_wq_manager_new;

static WorkQueue *
find_work_queue(const char *name)
{
	pthread_mutex_lock(&px4_work_queues_list_mutex);

	// search list
	for (WorkQueue *wq = px4_work_queues_list.getHead(); wq != nullptr; wq = wq->getSibling()) {
		if (strcmp(wq->get_name(), name) == 0) {
			pthread_mutex_unlock(&px4_work_queues_list_mutex);
			return wq;
		}
	}

	pthread_mutex_unlock(&px4_work_queues_list_mutex);

	return nullptr;
}

static void add_workqueue(WorkQueue *wq)
{
	// add to work queue list
	pthread_mutex_lock(&px4_work_queues_list_mutex);
	px4_work_queues_list.add(wq);
	pthread_mutex_unlock(&px4_work_queues_list_mutex);
}

static void remove_workqueue(WorkQueue *wq)
{
	// remove from work queue list
	pthread_mutex_lock(&px4_work_queues_list_mutex);
	px4_work_queues_list.remove(wq);
	pthread_mutex_unlock(&px4_work_queues_list_mutex);
}

WorkQueue *
work_queue_create(const wq_config &new_wq)
{
	// search list for existing work queue
	WorkQueue *wq = find_work_queue(new_wq.name);

	// create work queue if it doesn't exist
	if (wq == nullptr) {

		// add to list
		// main thread wakes up, creates the thread
		px4_wq_manager_new.push((wq_config *)&new_wq);

		// we wait until new wq is created, then return
		uint64_t t = 0;
		wq = nullptr;

		while (wq == nullptr && t < 10_s) {
			wq = find_work_queue(new_wq.name);

			// Wait up to 1s
			t += 2500;
			px4_usleep(2500);
		}

		if (wq == nullptr) {
			PX4_ERR("work queue: %s failed to create", new_wq.name);

		} else {
			// work queue found, set PID
			//wq->set_task_id(pid);
		}
	}

	return wq;
}

const wq_config &device_bus_to_wq(uint32_t device_id_int)
{
	union device::Device::DeviceId device_id;
	device_id.devid = device_id_int;

	const device::Device::DeviceBusType bus_type = device_id.devid_s.bus_type;
	const uint8_t bus = device_id.devid_s.bus;

	if (bus_type == device::Device::DeviceBusType_I2C) {
		if (bus == 1) {
			return wq_configurations[I2C1];

		} else if (bus == 2) {
			return wq_configurations[I2C2];
		}

	} else if (bus_type == device::Device::DeviceBusType_SPI) {
		if (bus == 1) {
			return wq_configurations[SPI1];

		} else if (bus == 2) {
			return wq_configurations[SPI2];
		}
	}

	// otherwise use high priority
	return wq_configurations[hp_default];
};



/*****************************************************************************
 *
 *
 *
 *
 */
void
WorkQueueManager::run()
{
	while (!should_exit()) {

		// create new work queues as needed
		wq_config *new_wq_config = px4_wq_manager_new.pop();

		PX4_INFO("New WQ: %s priority: %d stack: %d", new_wq_config->name, new_wq_config->priority, new_wq_config->stacksize);

		/* start the MAVLink receiver last to avoid a race */
		WorkQueueManager::create_work_queue_thread(new_wq_config);
	}

	// TODO: iterate and join all threads before shutdown
	// pthread_join(_receive_thread, nullptr);
}

void
WorkQueueManager::create_work_queue_thread(wq_config *wq)
{
	pthread_t thread; // TODO: store this

	pthread_attr_t attr{};
	struct sched_param param {};

	pthread_attr_init(&attr);
	pthread_attr_getschedparam(&attr, &param);

	if (pthread_attr_setstacksize(&attr, PX4_STACK_ADJUSTED(wq->stacksize)) != OK) {
		PX4_ERR("setting stack size failed");
	}

	param.sched_priority = wq->priority;

	if (pthread_attr_setschedparam(&attr, &param) != OK) {
		PX4_ERR("setting sched params failed");
	}

	int ret_create = pthread_create(&thread, &attr, WorkQueueManager::work_queue_runner, (void *)wq);

	if (ret_create != 0) {
		PX4_ERR("failed to create thread");
	}

	// TODO: set thread name cross platform (likely in work_queue_runner)
#if !defined(__PX4_DARWIN) && !defined(__PX4_QURT)
	pthread_setname_np(thread, wq->name);
#endif

	pthread_attr_destroy(&attr);
}

void *WorkQueueManager::work_queue_runner(void *context)
{
	// TODO: set thread name cross platform
	// set the threads name
//#ifdef __PX4_DARWIN
//	(void)pthread_setname_np(app_name);
//#else
//	(void)pthread_setname_np(pthread_self(), app_name);
//#endif


	wq_config *config = static_cast<wq_config *>(context);
	WorkQueue wq(config->name);

	add_workqueue(&wq);

	// Loop forever processing
	for (;;) {
		// TODO: shutdown mechanism
		wq.process();
	}

	remove_workqueue(&wq);

	return nullptr;
}

WorkQueueManager *
WorkQueueManager::instantiate(int argc, char *argv[])
{
	WorkQueueManager *instance = new WorkQueueManager();

	return instance;
}

int
WorkQueueManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
WorkQueueManager::print_status()
{
	if (px4_work_queues_list.getHead() == nullptr) {
		// list empty
		PX4_INFO("empty");
	}

	// status of each work queue
	int i = 0;

	for (WorkQueue *wq = px4_work_queues_list.getHead(); wq != nullptr; wq = wq->getSibling()) {
		PX4_INFO("wq %d: %s", i, wq->get_name());
		wq->print_status();

		printf("\n");
		i++;
	}

	return 0;
}

int
WorkQueueManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wq_manager", "work queue manager");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int
WorkQueueManager::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("wq_manager",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX,
				      1200,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	} else {
		PX4_INFO("starting");
	}

	return 0;
}

} // namespace px4

int wq_manager_main(int argc, char *argv[])
{
	return px4::WorkQueueManager::main(argc, argv);
}
