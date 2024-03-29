#include <mec/util.h>
#include <mec/estimation.h>

void mec_vehicle_position_init(struct mec_vehicle_position *pos)
{
	/* local frame, so just initialize with the current position as 0 */
	pos->north = 0;
	pos->east = 0;
	pos->down = 0;
	pos->altitude = 0;
}

void mec_vehicle_position_update(struct mec_vehicle_velocity_body *vel,
        float altitude,
        struct mec_vehicle_position *pos,
        struct mec_vehicle_attitude *att, float dt)
{
    /* 
     * Takes in relative velocity, converts to absolute velocity, and 
     * multiplies by dt to get change in absolute position 
     */
    struct mec_vehicle_velocity rectified_velocity;
    velocity_body_to_ned(vel, &rectified_velocity, att);

    // Down value is from pressure sensor, not DVL
    pos->north += rectified_velocity.north_m_s * dt;
    pos->east += rectified_velocity.east_m_s * dt;

    /* TODO filter the rangefinder data instead of just setting it */
    pos->altitude = altitude;
}
