#ifndef _MARITIME_EC_CONTROL_H
#define _MARITIME_EC_CONTROL_H

#include <matrix/math.hpp>

#include <mec/estimation.h>
#include <mec/pid_controller.h>

using namespace matrix;

/* NED frame */
struct mec_force_setpoint {
	float north;
	float east;
	float down;
};

/* NED, right-handed */
struct mec_torque_setpoint {
	float roll;
	float pitch;
	float yaw;
};

struct att_controller {
	struct pid_controller pid[3];

	struct mec_vehicle_attitude att_sp;
};


struct angvel_controller {
	struct pid_controller pid[3];

	struct mec_vehicle_angvel angvel_sp;
};

struct velocity_controller {
	struct pid_controller pid[3];

	struct mec_vehicle_velocity velocity_sp;
};


struct position_controller {
	struct pid_controller pid[3];

	struct mec_vehicle_position position_sp;
};

static float nemo_mix_data[6][8] =
{
	{ 0,  0,  0,  0,  1,  -1,  -1,  1, },
	{ 0,  0,  0,  0,  1,  1,  1,  1, },
	{ 1,  -1,  -1,  1,  0,  0,  0,  0, },
	{-1,  -1, 1,  1,  0,  0,  0,  0, },
	{ -1,  1,  -1,  1,  0,  0,  0,  0, },
	{ 0,  0,  0,  0,  -1,  -1,  1,  1, },
};

static Matrix<float, 6, 8> nemo_mix_mat(nemo_mix_data);

void mec_mix(struct mec_force_setpoint *force_sp, struct mec_torque_setpoint *torque_sp,
		Matrix<float, 6, 8> &mix, float *thruster_outputs);

void att_controller_init(struct att_controller *ctrl);
void att_controller_update_sp(struct att_controller *ctrl, struct mec_vehicle_attitude *att_sp);
void att_controller_update(struct att_controller *ctrl, struct mec_vehicle_attitude *att,
		struct mec_vehicle_angvel *output, float dt);

void angvel_controller_init(struct angvel_controller *ctrl);
void angvel_controller_update_sp(struct angvel_controller *ctrl,
		struct mec_vehicle_angvel *angvel_sp);
void angvel_controller_update(struct angvel_controller *ctrl, struct mec_vehicle_angvel *angvel,
		struct mec_torque_setpoint *output, float dt);

void position_controller_init(struct position_controller *ctrl);
void position_controller_update_sp(struct position_controller *ctrl, struct mec_vehicle_position *pos_sp);
void position_controller_update(struct position_controller *ctrl, struct mec_vehicle_position *pos,
		struct mec_vehicle_velocity *output, float dt);

void velocity_controller_init(struct velocity_controller *ctrl);
void velocity_controller_update_sp(struct velocity_controller *ctrl, struct mec_vehicle_velocity *vel_sp);
void velocity_controller_update(struct velocity_controller *ctrl, struct mec_vehicle_velocity *att,
		struct mec_force_setpoint *output, float dt);

#endif /* _MARITIME_EC_CONTROL_H */
