#include "../include/utils.hpp"

double sinc(double t)
{
    double s;
    if (abs(t) < 0.002)
    {
        s = 1 - pow(t, 2) / 6 * (1 - pow(t, 2) / 20);
    }
    else
    {
        s = sin(t) / t;
    }
    return s;
}

double mod2pi(double ang)
{
    double out = ang;
    while (out < 0)
    {
        out = out + 2 * M_PI;
    }
    while (out >= 2 * M_PI)
    {
        out = out - 2 * M_PI;
    }
    return out;
}

double rangeSymm(double ang)
{
    double out = ang;
    while (out <= -M_PI)
    {
        out = out + 2 * M_PI;
    }
    while (out > M_PI)
    {
        out = out - 2 * M_PI;
    }
    return out;
}
