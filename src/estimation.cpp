#include <matrix/math.hpp>

#include <mec/estimation.h>

void mec_vehicle_position_init(struct mec_vehicle_position *pos)
{
	/* local frame, so just initialize with the current position as 0 */
	pos->north = 0;
	pos->east = 0;
	pos->down = 0;
	pos->depth = 0;
}

void mec_vehicle_position_update(struct mec_vehicle_velocity *vel,
        float depth,
        struct mec_vehicle_position *pos, float dt)
{
	pos->north += vel->north_m_s * dt;
	pos->east += vel->east_m_s * dt;
	pos->down += vel->down_m_s * dt;

	/* TODO filter the rangefinder data instead of just setting it */
	pos->depth = depth;
}
