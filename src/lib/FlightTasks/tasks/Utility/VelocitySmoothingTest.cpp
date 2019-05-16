/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * Test code for the Velocity Smoothing library
 * Run this test only using make tests TESTFILTER=VelocitySmoothing
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "VelocitySmoothing.hpp"

using namespace matrix;

TEST(VelocitySmoothingTest, AllZeroCase)
{
	VelocitySmoothing trajectory[3];

	matrix::Vector3f a0(0.22, 0.f, 0.22f);
	matrix::Vector3f v0(2.47f, -5.59e-6f, 2.47f);
	matrix::Vector3f x0(0.f, 0.f, 0.f);

	float j_max = 55.2f;
	float a_max = 6.f;
	float v_max = 6.f;

	for (int i = 0; i < 3; i++) {
		trajectory[i].setMaxJerk(j_max);
		trajectory[i].setMaxAccel(a_max);
		trajectory[i].setMaxVel(v_max);
		trajectory[i].setCurrentAcceleration(a0(i));
		trajectory[i].setCurrentVelocity(v0(i));
	}

	float dt = 0.01f;

	matrix::Vector3f velocity_setpoint(0.f, 1.f, 0.f);

	for (int i = 0; i < 3; i++) {
		trajectory[i].updateDurations(dt, velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(trajectory, 2);

	for (int i = 0; i < 3; i++) {
		trajectory[i].integrate(a0(i), v0(i), x0(i));
	}

	// Check results
	EXPECT_GE(trajectory[0].getT1(), 0.1f);
}
