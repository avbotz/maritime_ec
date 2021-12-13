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

void velocity_controller_init(struct velocity_controller *ctrl)
{
	pid_set_gains(&ctrl->pid[0], 5.2, 0.9, 0.004);
	pid_set_gains(&ctrl->pid[1], 5.2, 0.9, 0.004);
	pid_set_gains(&ctrl->pid[2], 1.2, 0.9, 0.004);
}

void velocity_controller_update_sp(struct velocity_controller *ctrl,
		struct mec_vehicle_velocity *vel_sp)
{
	ctrl->velocity_sp.north_m_s = vel_sp->north_m_s;
	ctrl->velocity_sp.east_m_s = vel_sp->east_m_s;
	ctrl->velocity_sp.down_m_s = vel_sp->down_m_s;
}

void velocity_controller_update(struct velocity_controller *ctrl, struct mec_vehicle_velocity *vel,
		struct mec_force_setpoint *output, float dt)
{
	struct mec_vehicle_velocity error;

	error.north_m_s = ctrl->velocity_sp.north_m_s - vel->north_m_s;
	error.east_m_s = ctrl->velocity_sp.east_m_s - vel->east_m_s;
	error.down_m_s = ctrl->velocity_sp.down_m_s - vel->down_m_s;

	output->north = pid_calculate(&ctrl->pid[0], error.north_m_s, dt);
	output->east = pid_calculate(&ctrl->pid[1], error.east_m_s, dt);
	output->down = pid_calculate(&ctrl->pid[2], error.down_m_s, dt);
}
