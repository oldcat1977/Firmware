/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_log.h
 * MAVLink text logging.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/mavlink_log.h>

#ifdef __cplusplus
extern "C" {
#endif

__EXPORT void mavlink_log_publish(uint8_t severity, orb_advert_t *mavlink_log_pub, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

/*
 * The va_args implementation here is not beautiful, but obviously we run into the same issues
 * the GCC devs saw, and are using their solution:
 *
 * http://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
 */

/**
 * Send a mavlink info message (not printed to console).
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_log_info(_pub, _text, ...) mavlink_log_publish(mavlink_log_s::SEVERITY_INFO, _pub, _text, ##__VA_ARGS__);

/**
 * Send a mavlink emergency message and print to console.
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_log_emergency(_pub, _text, ...) mavlink_log_publish(mavlink_log_s::SEVERITY_EMERGENCY, _pub, _text, ##__VA_ARGS__);

/**
 * Send a mavlink critical message and print to console.
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_log_critical(_pub, _text, ...) mavlink_log_publish(mavlink_log_s::SEVERITY_CRITICAL, _pub, _text, ##__VA_ARGS__);

/**
 * Send a mavlink emergency message and print to console.
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_and_console_log_info(_pub, _text, ...) mavlink_log_publish(mavlink_log_s::SEVERITY_INFO, _pub, _text, ##__VA_ARGS__);
