/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file px4_module.h
 */

#pragma once

#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>

#include <px4_log.h>
#include <px4_tasks.h>
#include <containers/List.hpp>
#include <systemlib/px4_macros.h>

#include <cstring>

/**
 * @brief This mutex protects against race conditions during startup & shutdown of modules.
 *        There could be one mutex per module instantiation, but to reduce the memory footprint
 *        there is only a single global mutex. This sounds bad, but we actually don't expect
 *        contention here, as module startup is sequential.
 */
extern pthread_mutex_t px4_modules2_mutex;

class PX4ModuleBase;
extern List<PX4ModuleBase *> px4_modules_list;

class PX4ModuleBase : public ListNode<PX4ModuleBase *>
{
public:
	PX4ModuleBase(const char *name) : _name(name) {}
	virtual ~PX4ModuleBase() = default;

	const char *name() const { return _name; }

	virtual void run() = 0;

	virtual int print_status() { return PX4_OK; }

	/**
	 * @brief Checks if the module should stop (used within the module thread).
	 * @return Returns True iff successful, false otherwise.
	 */
	bool should_exit() const { return _task_should_exit; }

	void request_stop() { _task_should_exit = true; }

	/**
	 * @brief lock_module Mutex to lock the module thread.
	 */
	static void lock_module() { pthread_mutex_lock(&px4_modules2_mutex); }

	/**
	 * @brief unlock_module Mutex to unlock the module thread.
	 */
	static void unlock_module() { pthread_mutex_unlock(&px4_modules2_mutex); }

private:
	const char *_name;

	bool _task_should_exit{false};

};


PX4ModuleBase *get_module_instance(const char *name);
bool module_running(const char *name);
int module_start(const char *name, int priority, int stack, px4_main_t main);
int module_stop(const char *name);
int module_status(const char *name);


template<class T>
class PX4Module : public PX4ModuleBase
{
public:
	PX4Module() : PX4ModuleBase(get_name_static()) {}
	virtual ~PX4Module() = default;

	// static module properties
	static const char *get_name_static() { return "MODULE_NAME"; }
	static int get_stack_static() { return sizeof(PX4Module<T>) + 4000; } // TODO: use define
	static int get_priority_static() { return SCHED_PRIORITY_DEFAULT; } // TODO: use define

	/**
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int main(int argc, char *argv[])
	{
		if (argc <= 1 ||
		    strcmp(argv[1], "-h")    == 0 ||
		    strcmp(argv[1], "help")  == 0 ||
		    strcmp(argv[1], "info")  == 0 ||
		    strcmp(argv[1], "usage") == 0) {

			//return T::print_usage();
			return 0;
		}

		if (strcmp(argv[1], "start") == 0) {
			// Pass the 'start' argument too, because later on px4_getopt() will ignore the first argument.
			return start();
		}

		if (strcmp(argv[1], "status") == 0) {
			return status_command();
		}

		if (strcmp(argv[1], "stop") == 0) {
			return stop_command();
		}

		//lock_module(); // Lock here, as the method could access _object.
		//int ret = T::custom_command(argc - 1, argv + 1);
		//unlock_module();

		return 0;
	}

	static T *get_instance() { return (T *)get_module_instance(get_name_static()); }
	static bool is_running() { return module_running(get_name_static()); }

	static int start() { return module_start(get_name_static(), get_priority_static(), get_stack_static(), (px4_main_t)&T::run_trampoline); }
	static int stop_command() { return module_stop(get_name_static()); }
	static int status_command()  { return module_status(get_name_static()); }

	static int run_trampoline(int argc, char *argv[])
	{
		int ret = 0;

		// stack allocate object
		T object;

		// add to list
		// lock?
		PX4ModuleBase *base = &object;
		px4_modules_list.add(base);

		while (!object.should_exit()) {
			object.run();
		}

		// remove from list
		// lock?
		px4_modules_list.remove(base);

		return ret;
	}

};
