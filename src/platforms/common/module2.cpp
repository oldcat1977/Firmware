/****************************************************************************
 *
 * Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

/**
 * @file module.cpp
 * Implementation of the API declared in px4_module.h.
 */

#include <px4_module2.h>
#include <px4_log.h>

pthread_mutex_t px4_modules2_mutex = PTHREAD_MUTEX_INITIALIZER;

List<PX4ModuleBase *> px4_modules_list;

PX4ModuleBase *get_module_instance(const char *name)
{
	// search list
	for (PX4ModuleBase *module = px4_modules_list.getHead(); module != nullptr; module = module->getSibling()) {
		if (module->name() == name) {

			// TODO:
			// return module->running();

			return module;
		}
	}

	return nullptr;
}

bool module_running(const char *name)
{
	// search list
	PX4ModuleBase *module = get_module_instance(name);

	if (module != nullptr) {
		// TODO:
		// return module->running();

		return true;
	}

	return false;
}


int module_start(const char *name, int priority, int stack, px4_main_t main)
{
	// check that the module is not already running

	if (!module_running(name)) {
		// start

		// create new task
		// new task allocates object
		// add to List of running

		int task_id = px4_task_spawn_cmd("MODULE_NAME",
						 SCHED_DEFAULT,
						 priority, // TODO: set by module
						 stack, // TODO: set by module + add sizeof(class)
						 main,
						 (char *const *)nullptr); // TODO: arguments?

		if (task_id < 0) {
			PX4_ERR("task_id: %d", task_id);
			//	_task_id = -1;
			//	return -errno;
		}

	} else {
		PX4_INFO("already running");
	}

	return PX4_OK;
}

int module_stop(const char *name)
{
	int ret = 0;
	//lock_module();

	if (module_running(name)) {

		PX4ModuleBase *module = get_module_instance(name);

		if (module) {
			module->request_stop();
		}
	}

	//unlock_module();
	return ret;
}

int module_status(const char *name)
{
	int ret = -1;
	//lock_module();

	if (module_running(name)) {
		PX4ModuleBase *module = get_module_instance(name);

		if (module) {
			ret = module->print_status();
		}

	} else {
		PX4_INFO("not running");
	}

	//unlock_module();
	return ret;
}

