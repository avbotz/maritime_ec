/*
 * util.cpp
 *
 * Implements various utilities for normalization and other
 * tasks.
 *
 * @author Vincent Wang
 */

#include <math.h>
#include <mec/util.h>
#include <mec/estimation.h>
#include <mec/control.h>

double normalize(double value, double min, double max)
{
    /* Normalize a value between extremes.
     * Output will be between -1.0 and 1.0.
     */

    // Clamp values greater or smaller than extremes
    if (value < min) value = min;
    if (value > max) value = max;

    double zero = (max + min) / 2;
    double mag = (max - min) / 2;
    value += zero;

    return value;
}

void offsets_to_frame(float *input, float *angles, float *output)
{
    /* 
     * Input offsets at those angles from your frame,
     * Converts those offsets to your frame's offsets.
     * Input = x, y, z relative offsets
     * Angles = roll, pitch, yaw from your frame the input is applied at
     * Output = your frame's x, y, z absolute offsets
     */

    // Pre-compute trig functions.
    float sphi = sin(angles[0]);
    float sthe = sin(angles[1]);
    float spsi = sin(angles[2]);
    float cphi = cos(angles[0]);
    float cthe = cos(angles[1]);
    float cpsi = cos(angles[2]);

    // Calculate rotation matrix for Euler transformation.
    float r11 =  cthe*cpsi;
    float r12 = -cphi*spsi + sphi*sthe*cpsi;
    float r13 = -sphi*spsi + cphi*sthe*cpsi;
    float r21 =  cthe*spsi;
    float r22 =  cphi*cpsi + sphi*sthe*spsi;
    float r23 =  sphi*cpsi + cphi*sthe*spsi;
    float r31 = -sthe;
    float r32 =  sphi*cthe;
    float r33 =  cphi*cthe;

    // Calculate matrix.
    output[0] = r11*input[0] + r12*input[1] + r13*input[2];
    output[1] = r21*input[0] + r22*input[1] + r23*input[2];
    output[2] = r31*input[0] + r32*input[1] + r33*input[2];
}

void position_ned_to_body(struct mec_vehicle_position_body *body,
        struct mec_vehicle_position *ned, struct mec_vehicle_attitude *att)
{
    float ned_positions[] = { ned->north, ned->east, ned->down };
    float angle[] = { -att->roll, -att->pitch, -att->yaw };
    float body_positions[3];

    offsets_to_frame(ned_positions, angle, body_positions);
    body->forward = body_positions[0];
    body->right = body_positions[1];
    body->down = body_positions[2];
}

void velocity_ned_to_body(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att)
{
    float ned_velocities[] = { ned->north_m_s, ned->east_m_s, ned->down_m_s };
    float angle[] = { -att->roll, -att->pitch, -att->yaw };
    float body_velocities[3];

    offsets_to_frame(ned_velocities, angle, body_velocities);
    body->forward_m_s = body_velocities[0];
    body->right_m_s = body_velocities[1];
    body->down_m_s = body_velocities[2];
}

void velocity_body_to_ned(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att)
{
    float body_velocities[] = { body->forward_m_s, body->right_m_s, body->down_m_s };
    float angle[] = { att->roll, att->pitch, att->yaw };
    float ned_velocities[3];

    offsets_to_frame(body_velocities, angle, ned_velocities);
    ned->north_m_s = ned_velocities[0];
    ned->east_m_s = ned_velocities[1];
    ned->down_m_s = ned_velocities[2];
}

float angle_add(float a1, float add)
{
    float temp = a1 + add;
    if (temp > M_PI)
        return temp - 2*M_PI;
    else if (temp < -M_PI)
        return temp + 2*M_PI;

    return temp;
}

float angle_difference(float a1, float a2)
{
    // For [-180, 180].
    float b1 = a1-a2;
    if (fabs(b1) > M_PI)
    {
        if (a1 < a2)
            a1 += 2*M_PI;
        else 
            a2 += 2*M_PI;
        b1 = a1-a2;
    }
    return b1;
}
