/*
 * att_controller.cpp
 *
 * Implements a P controller to correct error in vehicle attitude
 * The outputs of this controller (angular velocity commands) are fed
 * into the angular rate controller, which in turn commands torque
 * setpoints which are fed through the mixer
 *
 * @author Kalyan Sriram, Vincent Wang
 */

#include <iostream>
#include <mec/control.h>
#include <mec/pid_controller.h>

/*
 * the attitude controller operates as a proportional only controller
 * (since the angular rate controller uses PID) so just keep the
 * integral and derivative constants to 0
 */

void att_controller_init(struct att_controller *ctrl)
{
	pid_set_gains(&ctrl->pid[0], 3.1, 0.0, 0.0);
	pid_set_gains(&ctrl->pid[1], 3.1, 0.0, 0.0);
	pid_set_gains(&ctrl->pid[2], 3.1, 0.0, 0.0);
}

void att_controller_update_sp(struct att_controller *ctrl, struct mec_vehicle_attitude *att_sp)
{
	ctrl->att_sp.roll = att_sp->roll;
	ctrl->att_sp.pitch = att_sp->pitch;
	ctrl->att_sp.yaw = att_sp->yaw;
}

void att_controller_update(struct att_controller *ctrl, struct mec_vehicle_attitude *att,
		struct mec_vehicle_angvel *output, float dt)
{
	struct mec_vehicle_attitude error;

	error.roll = ctrl->att_sp.roll - att->roll;
	error.pitch = ctrl->att_sp.pitch - att->pitch;
	error.yaw = ctrl->att_sp.yaw - att->yaw;

	output->roll_rad_s = pid_calculate(&ctrl->pid[0], error.roll, dt);
	output->pitch_rad_s = pid_calculate(&ctrl->pid[1], error.pitch, dt);
	output->yaw_rad_s = pid_calculate(&ctrl->pid[2], error.yaw, dt);
}
