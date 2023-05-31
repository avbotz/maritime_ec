#ifndef _MARITIME_EC_UTIL_H
#define _MARITIME_EC_UTIL_H

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#include <mec/estimation.h>

double normalize(double value, double min, double max);

void offsets_to_frame(float *input, float *angles, float *output);

void position_ned_to_body(struct mec_vehicle_position_body *body,
        struct mec_vehicle_position *ned, struct mec_vehicle_attitude *att);

void velocity_body_to_ned(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att);

void velocity_ned_to_body(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att);

float angle_add(float a1, float add);

float angle_difference(float a1, float a2);

#endif
