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

void position_controller_init(struct position_controller *ctrl)
{
	pid_set_gains(&ctrl->pid[0], 1.2, 0.0, 0.0);
	pid_set_gains(&ctrl->pid[1], 1.2, 0.0, 0.0);
	pid_set_gains(&ctrl->pid[2], 1.2, 0.0, 0.0);
}

void position_controller_update_sp(struct position_controller *ctrl, struct mec_vehicle_position *pos_sp)
{
	ctrl->position_sp.north = pos_sp->north;
	ctrl->position_sp.east = pos_sp->east;
	ctrl->position_sp.down = pos_sp->down;
	ctrl->position_sp.depth = pos_sp->depth;
}

void position_controller_update(struct position_controller *ctrl, struct mec_vehicle_position *pos,
		struct mec_vehicle_velocity *output, float dt)
{
	struct mec_vehicle_position error;

	error.north = ctrl->position_sp.north - pos->north;
	error.east = ctrl->position_sp.east - pos->east;
	error.depth = ctrl->position_sp.depth - pos->depth;

	output->north_m_s = pid_calculate(&ctrl->pid[0], error.north, dt);
	output->east_m_s = pid_calculate(&ctrl->pid[1], error.east, dt);
	output->down_m_s = pid_calculate(&ctrl->pid[2], error.depth, dt);
}
