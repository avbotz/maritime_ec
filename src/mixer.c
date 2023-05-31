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

#include "mec/control.h"
#include "mec/util.h"

#define NUM_DOF 6
#define NUM_THRUSTERS 8 /* TODO: find a better place for this */

void mec_mix(struct mec_force_setpoint *force_sp, struct mec_torque_setpoint *torque_sp,
		float mix[8][6], float power, float *thruster_outputs)
{
	/* order: north, east, down, roll, pitch, yaw */
	float force_setpoints[] = {
		force_sp->forward,
		force_sp->right,
		force_sp->down,
		torque_sp->roll,
		torque_sp->pitch,
		torque_sp->yaw,
	};

	/* 
	 * Calculate thrust on each thruster based on desired forces
	 * and the mix (the thruster configuration) of the sub 
	 */
	for (int thruster = 0; thruster < NUM_THRUSTERS; thruster++)
		for (int dof = 0; dof < NUM_DOF; dof++) 
			thruster_outputs[thruster] += 
				force_setpoints[dof] * mix[thruster][dof];

	/* We restrict the power level of the thrusters b/c of electrical concerns */
	for (int i = 0; i < NUM_THRUSTERS; i++)
		thruster_outputs[i] = normalize(thruster_outputs[i], -power, power);
}
