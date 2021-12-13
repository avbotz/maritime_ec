/*
 * util.cpp
 *
 * Implements various utilities for normalization and other
 * tasks.
 *
 * @author Vincent Wang
 */

#include <iostream>
#include <mec/util.h>

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
