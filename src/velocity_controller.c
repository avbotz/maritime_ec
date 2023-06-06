/*
 * angvel_controller.cpp
 *
 * Implements a PID controller to correct error in vehicle angular rate
 * The outputs of this controller (torque setpoints) are fed through the mixer
 *
 * @author Kalyan Sriram, Vincent Wang
 */

#include <mec/control.h>
#include <mec/pid_controller.h>
#include <mec/util.h>

void velocity_controller_init(struct velocity_controller *ctrl)
{
	pid_set_gains(&ctrl->pid[0], 20, 15.0, 0.0);
	pid_set_gains(&ctrl->pid[1], 20, 15.0, 0.0);
	pid_set_gains(&ctrl->pid[2], 5.2, 2.0, 0.0);
}

void velocity_controller_update_sp(struct velocity_controller *ctrl,
		struct mec_vehicle_velocity_body *vel_sp)
{
	ctrl->velocity_sp.forward_m_s = vel_sp->forward_m_s;
	ctrl->velocity_sp.right_m_s = vel_sp->right_m_s;
	ctrl->velocity_sp.down_m_s = vel_sp->down_m_s;
}

void velocity_controller_update(struct velocity_controller *ctrl, struct mec_vehicle_velocity_body *vel,
		struct mec_force_setpoint *output, float dt)
{
	struct mec_vehicle_velocity_body error;

	error.forward_m_s = ctrl->velocity_sp.forward_m_s - vel->forward_m_s;
	error.right_m_s = ctrl->velocity_sp.right_m_s - vel->right_m_s;
	error.down_m_s = ctrl->velocity_sp.down_m_s - vel->down_m_s;

	output->forward = normalize(pid_calculate(&ctrl->pid[0], error.forward_m_s, dt), -1, 1);
	output->right = normalize(pid_calculate(&ctrl->pid[1], error.right_m_s, dt), -1, 1);
	output->down = normalize(pid_calculate(&ctrl->pid[2], error.down_m_s, dt), -1, 1);

    // ARW
    if (output->forward >= 1 || output->forward <= -1)
    {
        ctrl->pid[0].integral -= error.forward_m_s * dt;
    }
    if (output->right >= 1 || output->right <= -1)
    {
        ctrl->pid[1].integral -= error.right_m_s * dt;
    }
    if (output->down >= 1 || output->down <= -1)
    {
        ctrl->pid[2].integral -= error.down_m_s * dt;
    }

}
