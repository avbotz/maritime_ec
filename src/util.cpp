/*
 * util.cpp
 *
 * Implements various utilities for normalization and other
 * tasks.
 *
 * @author Vincent Wang
 */

#include <matrix/math.hpp>
#include <math.h>
#include <iostream>
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

void transform_frame(float frame1[3], float frame2[3], float angle[3])
{
    /* Rotate one frame to another. Angles are (roll, pitch, yaw). */
    float xrot_arr[3][3] =
    {
        { 1, 0, 0 },
        { 0, cos(angle[1]), -sin(angle[1]) },
        { 0, sin(angle[1]), cos(angle[1]) }
    };

    float yrot_arr[3][3] =
    {
        { cos(angle[0]), 0, sin(angle[0]) },
        { 0, 1, 0 },
        { -sin(angle[0]), 0, cos(angle[0]) }
    };

    float zrot_arr[3][3] =
    {
        { cos(angle[2]), -sin(angle[2]), 0 },
        { sin(angle[2]), cos(angle[2]), 0 },
        { 0, 0, 1 }
    };

    Matrix<float, 3, 3> xrot(xrot_arr);
    Matrix<float, 3, 3> yrot(yrot_arr);
    Matrix<float, 3, 3> zrot(zrot_arr);

    Vector<float, 3> vec_1;
    Vector<float, 3> vec_2;

    vec_1(0) = frame1[0];
    vec_1(1) = frame1[1];
    vec_1(2) = frame1[2];

    vec_2 = xrot * yrot * zrot * vec_1;
    frame2[0] = vec_2(0);
    frame2[1] = vec_2(1);
    frame2[2] = vec_2(2);
}

void velocity_ned_to_body(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att)
{
    float body_frame[] = { body->forward_m_s, body->right_m_s, body->down_m_s };
    float ned_frame[] = { ned->north_m_s, ned->east_m_s, ned->down_m_s };
    float angle[] = { att->roll, att->pitch, att->yaw };

    transform_frame(ned_frame, body_frame, angle);
    body->forward_m_s = body_frame[0];
    body->right_m_s = body_frame[1];
    body->down_m_s = body_frame[2];
}

void velocity_body_to_ned(struct mec_vehicle_velocity_body *body,
        struct mec_vehicle_velocity *ned, struct mec_vehicle_attitude *att)
{
    float body_frame[] = { body->forward_m_s, body->right_m_s, body->down_m_s };
    float ned_frame[] = { ned->north_m_s, ned->east_m_s, ned->down_m_s };
    float angle[] = { -att->roll, -att->pitch, -att->yaw };

    transform_frame(body_frame, ned_frame, angle);
    ned->north_m_s = ned_frame[0];
    ned->east_m_s = ned_frame[1];
    ned->down_m_s = ned_frame[2];
}
