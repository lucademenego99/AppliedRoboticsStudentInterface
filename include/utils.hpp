#ifndef UTILS
#define UTILS

#define _USE_MATH_DEFINES
#include <cmath>

struct Point
{
    double x, y, th;

    Point(double i_x, double i_y) {
        x = i_x;
        y = i_y;
        th = -1;
    }

    Point(double i_x, double i_y, double i_th) {
        x = i_x;
        y = i_y;
        th = i_th;
    }
};

double sinc(double t);
double mod2pi(double angle);
double rangeSymm(double angle);
void cross_product(double vector_a[], double vector_b[], double temp);
double dot2D(double vector_a[], double vector_b[]);

#endif