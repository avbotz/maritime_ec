/*
 * angvel_controller.cpp
 *
 * Implements a PID controller to correct error in vehicle angular rate
 * The outputs of this controller (torque setpoints) are fed through the mixer
 *
 * @author Kalyan Sriram, Vincent Wang
 */

#include <iostream>
#include <mec/control.h>
#include <mec/pid_controller.h>

void angvel_controller_init(struct angvel_controller *ctrl)
{
	pid_set_gains(&ctrl->pid[0], 1.0, 0.5, 0.0008);
	pid_set_gains(&ctrl->pid[1], 1.0, 0.5, 0.0008);
	pid_set_gains(&ctrl->pid[2], 1.0, 0.8, 0.0008);
}

void angvel_controller_update_sp(struct angvel_controller *ctrl,
		struct mec_vehicle_angvel *angvel_sp)
{
	ctrl->angvel_sp.roll_rad_s = angvel_sp->roll_rad_s;
	ctrl->angvel_sp.pitch_rad_s = angvel_sp->pitch_rad_s;
	ctrl->angvel_sp.yaw_rad_s = angvel_sp->yaw_rad_s;
}

void angvel_controller_update(struct angvel_controller *ctrl, struct mec_vehicle_angvel *angvel,
		struct mec_torque_setpoint *output, float dt)
{
	struct mec_vehicle_angvel error;

	error.roll_rad_s = ctrl->angvel_sp.roll_rad_s - angvel->roll_rad_s;
	error.pitch_rad_s = ctrl->angvel_sp.pitch_rad_s - angvel->pitch_rad_s;
	error.yaw_rad_s = ctrl->angvel_sp.yaw_rad_s - angvel->yaw_rad_s;

	output->roll = pid_calculate(&ctrl->pid[0], error.roll_rad_s, dt);
	output->pitch = pid_calculate(&ctrl->pid[1], error.pitch_rad_s, dt);
	output->yaw = pid_calculate(&ctrl->pid[2], error.yaw_rad_s, dt);
}
