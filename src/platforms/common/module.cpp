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

#include <px4_module.h>
#include <px4_log.h>

pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;

List<ModuleBaseInterface *> px4_modules_list;


ModuleBaseInterface *get_module_instance(const char *name)
{
	// search list
	for (ModuleBaseInterface *module = px4_modules_list.getHead(); module != nullptr; module = module->getSibling()) {
		if (strcmp(module->name(), name) == 0) {

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
	ModuleBaseInterface *module = get_module_instance(name);

	if (module != nullptr) {
		// TODO:
		// return module->running();

		return true;
	}

	return false;
}

/**
 * @brief Waits until _object is initialized, (from the new thread). This can be called from task_spawn().
 * @return Returns 0 iff successful, -1 on timeout or otherwise.
 */
int module_wait_until_running(const char *name)
{
	int i = 0;

	ModuleBaseInterface *object = nullptr;

	do {
		object = get_module_instance(name);

		/* Wait up to 1s. */
		usleep(2500);

	} while (!object && ++i < 400);

	if (i == 400) {
		PX4_ERR("Timed out while waiting for thread to start");
		return -1;
	}

	return 0;
}

int module_stop(const char *name)
{
	int ret = 0;
	ModuleBaseInterface::lock_module();

	if (module_running(name)) {

		ModuleBaseInterface *object = get_module_instance(name);

		if (object) {

			object->request_stop();

			unsigned int i = 0;

			do {
				ModuleBaseInterface::unlock_module();
				usleep(20000); // 20 ms
				ModuleBaseInterface::lock_module();

				if (++i > 100 && object->task_id() != -1) { // wait at most 2 sec
					if (object->task_id() != task_id_is_work_queue) {
						px4_task_delete(object->task_id());
					}

					if (object) {
						px4_modules_list.remove(object);
						delete object;
					}

					ret = -1;
					break;
				}
			} while (object->task_id() != -1);

		} else {
			// In the very unlikely event that can only happen on work queues,
			// if the starting thread does not wait for the work queue to initialize,
			// and inside the work queue, the allocation of _object fails
			// and exit_and_cleanup() is not called, set the _task_id as invalid.

			//_task_id = -1;
		}
	}

	ModuleBaseInterface::unlock_module();
	return ret;
}

void module_exit_and_cleanup(const char *name)
{
	// Take the lock here:
	// - if startup fails and we're faster than the parent thread, it will set
	//   _task_id and subsequently it will look like the task is running.
	// - deleting the object must take place inside the lock.
	ModuleBaseInterface::lock_module();

	ModuleBaseInterface *object = get_module_instance(name);

	if (object) {
		px4_modules_list.remove(object);
		delete object;
	}

	//_task_id = -1; // Signal a potentially waiting thread for the module to exit that it can continue.
	ModuleBaseInterface::unlock_module();
}

int module_status(const char *name)
{
	int ret = -1;
	ModuleBaseInterface::lock_module();

	ModuleBaseInterface *object = get_module_instance(name);

	if (module_running(name) && object) {
		ret = object->print_status();

	} else {
		PX4_INFO("not running");
	}

	ModuleBaseInterface::unlock_module();
	return ret;
}


#ifndef __PX4_NUTTX

void PRINT_MODULE_DESCRIPTION(const char *description)
{
	// TODO: the output could be improved by:
	// - mark titles in bold (lines starting with ##)
	// - highlight commands (lines starting with $, or `cmd`)
	PX4_INFO_RAW("%s\n\n", description);
}

#endif /* __PX4_NUTTX */

void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category)
{
	PX4_INFO_RAW("Usage: %s <command> [arguments...]\n", executable_name);
	PX4_INFO_RAW(" Commands:\n");
}

void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category)
{
	PX4_INFO_RAW("Usage: %s [arguments...]\n", executable_name);
}

void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description)
{
	if (description) {
		PX4_INFO_RAW("\n   %-13s %s\n", name, description);

	} else {
		PX4_INFO_RAW("\n   %s\n", name);
	}
}

void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment)
{
	PX4_INFO_RAW("\n %s\n", comment);
}

void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
				  const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);
		PX4_INFO_RAW("                 default: %i\n", default_val);

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
				    const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);
		PX4_INFO_RAW("                 default: %.1f\n", (double)default_val);

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c]        %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c          %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
				     const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}

	if (values) {
		if (default_val) {
			PX4_INFO_RAW("                 values: %s, default: %s\n", values, default_val);

		} else {
			PX4_INFO_RAW("                 values: %s\n", values);
		}

	} else {
		if (default_val) {
			PX4_INFO_RAW("                 default: %s\n", default_val);
		}
	}
}


void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [%-9s] %s\n", values, description);

	} else {
		PX4_INFO_RAW("     %-11s %s\n", values, description);
	}
}

