#ifndef _MARITIME_EC_CONTROL_H
#define _MARITIME_EC_CONTROL_H

#include <mec/estimation.h>
#include <mec/pid_controller.h>
#include <stdbool.h>

/* NED frame */
struct mec_force_setpoint {
	float forward;
	float right;
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

	struct mec_vehicle_velocity_body velocity_sp;
};


struct position_controller {
	struct pid_controller pid[3];

	struct mec_vehicle_position position_sp;
    bool use_floor_altitude;
};

/*
 * Columns are each degree of freedom:
 *  x, y, z, roll, pitch, yaw.
 * Rows are thrusters:
 *  vertical front left, vertical front right,
 *  vertical back left, vertical back right,
 *  horizontal front left, horizontal front right,
 *  horizontal back left, horizontal back right.
 * -1 = thruster pushes you in the negative on that axis
 *  0 = thruster cannot affect this axis
 *  1 = thruster pushes you in the positive on that axis
 * Thrusters can have right propellers or left propellers,
 *  which is why the sign is different for each thruster.
 *  The propeller config is right left left right right left left right
 *  in the same order as the thrusters in the rows.
 */
static float sub_mix_data[8][6] =
{
	{ 0.00, 0.00, 1.00, -1.0, -1.0, 0.00},
	{ 0.00, 0.00, -1.0, -1.0, 1.00, 0.00},
	{ 0.00, 0.00, -1.0, 1.00, -1.0, 0.00},
	{ 0.00, 0.00, 1.00, 1.00, 1.00, 0.00},
	{ 1.00, 1.00, 0.00, 0.00, 0.00, 1.00},
	{ -1.0, 1.00, 0.00, 0.00, 0.00, -1.0},
	{ -1.0, 1.00, 0.00, 0.00, 0.00, -1.0},
	{ 1.00, 1.00, 0.00, 0.00, 0.00, -1.0},
};

void mec_mix(struct mec_force_setpoint *force_sp, struct mec_torque_setpoint *torque_sp,
		float mix[8][6], float power, float *thruster_outputs);

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
		struct mec_vehicle_attitude *att, struct mec_vehicle_velocity_body *output, float dt);

void velocity_controller_init(struct velocity_controller *ctrl);
void velocity_controller_update_sp(struct velocity_controller *ctrl, struct mec_vehicle_velocity_body *vel_sp);
void velocity_controller_update(struct velocity_controller *ctrl, struct mec_vehicle_velocity_body *att,
		struct mec_force_setpoint *output, float dt);

#endif /* _MARITIME_EC_CONTROL_H */
