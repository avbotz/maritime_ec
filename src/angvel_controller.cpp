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
#include <mec/util.h>

void angvel_controller_init(struct angvel_controller *ctrl)
{
	pid_set_gains(&ctrl->pid[0], 3.4, 0.7, 0.012);
	pid_set_gains(&ctrl->pid[1], 3.4, 0.7, 0.012);
	pid_set_gains(&ctrl->pid[2], 3.4, 0.7, 0.012);
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

	output->roll = normalize(pid_calculate(&ctrl->pid[0], error.roll_rad_s, dt), -1, 1);
	output->pitch = normalize(pid_calculate(&ctrl->pid[1], error.pitch_rad_s, dt), -1, 1);
	output->yaw = normalize(pid_calculate(&ctrl->pid[2], error.yaw_rad_s, dt), -1, 1);

    // ARW
    if (output->roll >= 1 || output->roll <= -1)
    {
        ctrl->pid[0].integral -= error.roll_rad_s * dt;
    }
    if (output->pitch >= 1 || output->pitch <= -1)
    {
        ctrl->pid[1].integral -= error.pitch_rad_s * dt;
    }
    if (output->yaw >= 1 || output->yaw <= -1)
    {
        ctrl->pid[2].integral -= error.yaw_rad_s * dt;
    }
}
