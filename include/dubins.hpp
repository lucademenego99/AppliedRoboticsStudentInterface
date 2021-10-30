#ifndef DUBINSCURVES
#define DUBINSCURVES

#include "../include/utils.hpp"
#include <vector>
#include <iostream>

//region Structures

struct DubinsLine
{
    long double x, y, th;

    DubinsLine(double s, double x0, double y0, double th0, double k)
    {
        long double tmp = (s * sinc(k * s / 2.0) * sin(th0 + ((k * s) / 2)));
        long double xtmp = y0 + tmp;
        x = x0 + (s * sinc(k * s / 2.0) * cos(th0 + ((k * s) / 2)));
        y = y0 + (s * sinc(k * s / 2.0) * sin(th0 + ((k * s) / 2)));
        th = mod2pi(th0 + (k * s));
    }
};

struct DubinsArc
{
    double x0, y0, th0, k, L;
    DubinsLine *dubins_line;

    DubinsArc(double i_x0, double i_y0, double i_th0, double i_k, double i_L)
    {
        dubins_line = new DubinsLine(i_L, i_x0, i_y0, i_th0, i_k);
        x0 = i_x0;
        y0 = i_y0;
        th0 = i_th0;
        k = i_k;
        L = i_L;
    }

    ~DubinsArc()
    {
        if (dubins_line)
            delete dubins_line;
    }
};

struct DubinsCurve
{
    DubinsArc *a1;
    DubinsArc *a2;
    DubinsArc *a3;
    double L;

    DubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2)
    {
        a1 = new DubinsArc(x0, y0, th0, k0, s1);
        a2 = new DubinsArc(a1->dubins_line->x, a1->dubins_line->y, a1->dubins_line->th, k1, s2);
        a3 = new DubinsArc(a2->dubins_line->x, a2->dubins_line->y, a2->dubins_line->th, k2, s3);

        L = a1->L + a2->L + a3->L;
    }

    ~DubinsCurve()
    {
        if (a1)
            delete a1;
        if (a2)
            delete a2;
        if (a3)
            delete a3;
    }
};

struct ParametersResult
{
    double scaled_th0, scaled_thf, scaled_k_max, lambda;

    ParametersResult(double sc_th0, double sc_thf, double sc_k_max, double l)
    {
        scaled_th0 = sc_th0;
        scaled_thf = sc_thf;
        scaled_k_max = sc_k_max;
        lambda = l;
    }
};

struct CurveSegmentsResult
{
    bool ok;
    double s1, s2, s3;

    CurveSegmentsResult(bool i_ok, double i_s1, double i_s2, double i_s3)
    {
        ok = i_ok;
        s1 = i_s1;
        s2 = i_s2;
        s3 = i_s3;
    }
};

//endregion Structures

class Dubins
{
private:
    //region Predefined dubins curves sets

    const int TOTAL_POSSIBLE_CURVES = 6;

    enum possible_curves
    {
        LSL,
        RSR,
        LSR,
        RSL,
        RLR,
        LRL,
    };

    const int ksigns[6][3] = {
        {1, 0, 1},
        {-1, 0, -1},
        {1, 0, -1},
        {-1, 0, 1},
        {-1, 1, -1},
        {1, -1, 1},
    };
    //endregion Predefined dubins curves sets

    double k_max = 1;
    double discritizer_size = 0.005; // in meters

    // Check the solution
    bool checkValidity(CurveSegmentsResult *curve_segments_result, double k0, double k1, double k2, double th0, double thf);

    // Helpers
    ParametersResult *scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf);
    CurveSegmentsResult *scaleFromStandard(double lambda, CurveSegmentsResult *curve_segments);
    CurveSegmentsResult *useLSL(double scaled_th0, double scaled_thf, double scaled_k_max);
    CurveSegmentsResult *useRSR(double scaled_th0, double scaled_thf, double scaled_k_max);
    CurveSegmentsResult *useLSR(double scaled_th0, double scaled_thf, double scaled_k_max);
    CurveSegmentsResult *useRSL(double scaled_th0, double scaled_thf, double scaled_k_max);
    CurveSegmentsResult *useRLR(double scaled_th0, double scaled_thf, double scaled_k_max);
    CurveSegmentsResult *useLRL(double scaled_th0, double scaled_thf, double scaled_k_max);

public:
    Dubins() = default;
    explicit Dubins(double k_max, double discritizer_size);
    DubinsCurve *findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf);
    DubinsCurve **multipointShortestPath(Point **points, uint numberOfPoints);
};

#endif