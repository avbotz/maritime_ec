#ifndef _MARITIME_EC_ESTIMATION_H
#define _MARITIME_EC_ESTIMATION_H

#include <matrix/math.hpp>

struct mec_vehicle_position {
	float north;
	float east;
	float down;
};

struct mec_vehicle_dvl_data {
	float north_m_s;
	float east_m_s;
	float down; /* downwards is a range finder */
};

struct mec_vehicle_attitude {
	float roll;
	float pitch;
	float yaw;
};

struct mec_vehicle_angvel {
	float roll_rad_s;
	float pitch_rad_s;
	float yaw_rad_s;
};

void mec_vehicle_position_init(struct mec_vehicle_position *pos);
void mec_vehicle_position_update(struct mec_vehicle_position *pos,
		struct mec_vehicle_dvl_data *vel, float dt);

#endif /* _MARITIME_EC_ESTIMATION_H */
