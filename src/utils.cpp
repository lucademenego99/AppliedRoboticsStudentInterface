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

void cross_product(double vector_a[], double vector_b[], double temp) {
   temp = vector_a[0]*vector_b[1] - vector_a[1]*vector_b[0];
}

double dot2D(double vector_a[], double vector_b[]){
    double temp = vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1];
    return temp;
}
