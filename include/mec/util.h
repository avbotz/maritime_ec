#ifndef _MARITIME_EC_UTIL_H
#define _MARITIME_EC_UTIL_H

double normalize(double value, double min, double max);

void velocity_body_to_ned(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att);

void velocity_ned_to_body(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att);

#endif
