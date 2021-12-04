/*
 * mixer.cpp
 *
 * Implements motor mixing functionality for maritime control
 *
 * Essentially, the mixer takes force and torque setpoints from the velocity
 * and angular velocity controller and outputs motor thrust setpoints
 *
 * @author Kalyan Sriram <kalyan@coderkalyan.com>
 */

#include <matrix/math.hpp>
#include <iostream>
#include "mec/control.h"

#define NUM_DOF 6
#define NUM_THRUSTERS 8 /* TODO: find a better place for this */

void mec_mix(struct mec_force_setpoint *force_sp, struct mec_torque_setpoint *torque_sp,
		Matrix<float, NUM_DOF, NUM_THRUSTERS> &mix, float *thruster_outputs)
{
	Vector<float, NUM_DOF> sp_vec;
	Vector<float, NUM_THRUSTERS> out_vec;

	/* order: north, east, down, roll, pitch, yaw */
	sp_vec(0) = force_sp->north;
	sp_vec(1) = force_sp->east;
	sp_vec(2) = force_sp->down;
	sp_vec(3) = torque_sp->roll;
	sp_vec(4) = torque_sp->pitch;
	sp_vec(5) = torque_sp->yaw;

	out_vec = mix.T() * sp_vec;

	for (int i = 0;i < NUM_THRUSTERS;i++) {
		thruster_outputs[i] = out_vec(i);
	}
}
