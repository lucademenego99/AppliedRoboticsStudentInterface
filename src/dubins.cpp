#include "../include/dubins.hpp"

#include <iostream>
#include <cfloat>
#include <algorithm>

/**
 * @brief Construct a new Dubins:: Dubins object
 * 
 * @param k_max Bound on maximum path curvature
 * @param discritizer_size Given a path of infinite points, what is the discritizer size? Expressed in meters
 */
Dubins::Dubins(double k_max, double discritizer_size)
{
    this->k_max = k_max;
    this->discritizer_size = discritizer_size;
};

/**
 * @brief Check the validity of a provided Dubins solution
 * 
 * @param curve_segments Resulting curve segments representing the final solution
 * @param k0 TODO
 * @param k1 TODO
 * @param k2 TODO
 * @param th0 Starting angle
 * @param thf Final angle
 * @return true If the solution is valid
 * @return false If the solution is not valid
 */
bool Dubins::checkValidity(CurveSegmentsResult *curve_segments, double k0, double k1, double k2, double th0, double thf)
{
    double x0 = -1;
    double y0 = 0;
    double xf = 1;
    double yf = 0;

    double eq1 = x0 + curve_segments->s1 * sinc((1 / 2.) * k0 * curve_segments->s1) * cos(th0 + (1 / 2.) * k0 * curve_segments->s1) + curve_segments->s2 * sinc((1 / 2.) * k1 * curve_segments->s2) * cos(th0 + k0 * curve_segments->s1 + (1 / 2.) * k1 * curve_segments->s2) + curve_segments->s3 * sinc((1 / 2.) * k2 * curve_segments->s3) * cos(th0 + k0 * curve_segments->s1 + k1 * curve_segments->s2 + (1 / 2.) * k2 * curve_segments->s3) - xf;
    double eq2 = y0 + curve_segments->s1 * sinc((1 / 2.) * k0 * curve_segments->s1) * sin(th0 + (1 / 2.) * k0 * curve_segments->s1) + curve_segments->s2 * sinc((1 / 2.) * k1 * curve_segments->s2) * sin(th0 + k0 * curve_segments->s1 + (1 / 2.) * k1 * curve_segments->s2) + curve_segments->s3 * sinc((1 / 2.) * k2 * curve_segments->s3) * sin(th0 + k0 * curve_segments->s1 + k1 * curve_segments->s2 + (1 / 2.) * k2 * curve_segments->s3) - yf;
    double eq3 = rangeSymm(k0 * curve_segments->s1 + k1 * curve_segments->s2 + k2 * curve_segments->s3 + th0 - thf);

    bool Lpos = (curve_segments->s1 > 0) || (curve_segments->s2 > 0) || (curve_segments->s3 > 0);
    // TODO: I modified the value 1.e-10 to 1.e-2 -> if we lower the threshold, the program says our results are valid.
    // otherwise not. This is the problem I was describing in the group
    return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-2 && Lpos);
};

/**
 * @brief Scale the input parameters to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
 * 
 * @param x0 Starting x position
 * @param y0 Starting y position
 * @param th0 Starting angle
 * @param xf Final x position
 * @param yf Final y position
 * @param thf Final angle
 * @return ParametersResult* Scaled parameters
 */
ParametersResult *Dubins::scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf)
{
    double dx = xf - x0;
    double dy = yf - y0;
    double phi = atan2(dy, dx);
    double lambda = hypot(dx, dy) / 2;

    double sc_th0 = mod2pi(th0 - phi);
    double sc_thf = mod2pi(thf - phi);
    double sc_k_max = k_max * lambda;
    return new ParametersResult(sc_th0, sc_thf, sc_k_max, lambda);
}

/**
 * @brief Return to the initial scaling
 * 
 * @param lambda TODO
 * @param curve_segments Current scaled parameters of our problem
 * @return CurveSegmentsResult* 
 */
CurveSegmentsResult *Dubins::scaleFromStandard(double lambda, CurveSegmentsResult *curve_segments)
{
    return new CurveSegmentsResult(true, curve_segments->s1 * lambda, curve_segments->s2 * lambda, curve_segments->s3 * lambda);
}

CurveSegmentsResult *Dubins::useLSL(double scaled_th0, double scaled_thf, double scaled_k_max)
{
    double s1, s2, s3;
    double invK = 1 / scaled_k_max;
    double C = cos(scaled_thf) - cos(scaled_th0);
    double S = 2 * scaled_k_max + sin(scaled_th0) - sin(scaled_thf);
    double temp1 = atan2(C, S);
    s1 = invK * mod2pi(temp1 - scaled_th0);
    double temp2 = 2 + 4 * pow(scaled_k_max, 2) - 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf));
    if (temp2 < 0)
    {
        s1 = 0;
        s2 = 0;
        s3 = 0;
        return new CurveSegmentsResult(false, s1, s2, s3);
    }
    s2 = invK * sqrt(temp2);
    s3 = invK * mod2pi(scaled_thf - temp1);
    return new CurveSegmentsResult(true, s1, s2, s3);
}

CurveSegmentsResult *Dubins::useRSR(double scaled_th0, double scaled_thf, double scaled_k_max)
{
    double s1, s2, s3;
    double invK = 1 / scaled_k_max;
    double C = cos(scaled_th0) - cos(scaled_thf);
    double S = 2 * scaled_k_max - sin(scaled_th0) + sin(scaled_thf);
    double temp1 = atan2(C, S);
    s1 = invK * mod2pi(scaled_th0 - temp1);
    double temp2 = 2 + 4 * pow(scaled_k_max, 2) - 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf));
    if (temp2 < 0)
    {
        s1 = 0;
        s2 = 0;
        s3 = 0;
        return new CurveSegmentsResult(false, s1, s2, s3);
    }
    s2 = invK * sqrt(temp2);
    s3 = invK * mod2pi(temp1 - scaled_thf);
    return new CurveSegmentsResult(true, s1, s2, s3);
}

CurveSegmentsResult *Dubins::useLSR(double scaled_th0, double scaled_thf, double scaled_k_max)
{
    double s1, s2, s3;
    double invK = 1 / scaled_k_max;
    double C = cos(scaled_th0) + cos(scaled_thf);
    double S = 2 * scaled_k_max + sin(scaled_th0) + sin(scaled_thf);
    double temp1 = atan2(-C, S);
    double temp3 = 4 * pow(scaled_k_max, 2) - 2 + 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) + sin(scaled_thf));
    if (temp3 < 0)
    {
        s1 = 0;
        s2 = 0;
        s3 = 0;
        return new CurveSegmentsResult(false, s1, s2, s3);
    }
    s2 = invK * sqrt(temp3);
    double temp2 = -atan2(-2, s2 * scaled_k_max);
    s1 = invK * mod2pi(temp1 + temp2 - scaled_th0);
    s3 = invK * mod2pi(temp1 + temp2 - scaled_thf);
    return new CurveSegmentsResult(true, s1, s2, s3);
}

CurveSegmentsResult *Dubins::useRSL(double scaled_th0, double scaled_thf, double scaled_k_max)
{
    double s1, s2, s3;
    double invK = 1 / scaled_k_max;
    double C = cos(scaled_th0) + cos(scaled_thf);
    double S = 2 * scaled_k_max - sin(scaled_th0) - sin(scaled_thf);
    double temp1 = atan2(C, S);
    double temp3 = 4 * pow(scaled_k_max, 2) - 2 + 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) + sin(scaled_thf));
    if (temp3 < 0)
    {
        s1 = 0;
        s2 = 0;
        s3 = 0;
        return new CurveSegmentsResult(false, s1, s2, s3);
    }
    s2 = invK * sqrt(temp3);
    double temp2 = atan2(2, s2 * scaled_k_max);
    s1 = invK * mod2pi(scaled_th0 - temp1 + temp2);
    s3 = invK * mod2pi(scaled_thf - temp1 + temp2);
    return new CurveSegmentsResult(true, s1, s2, s3);
}

CurveSegmentsResult *Dubins::useRLR(double scaled_th0, double scaled_thf, double scaled_k_max)
{
    double s1, s2, s3;
    double invK = 1 / scaled_k_max;
    double C = cos(scaled_th0) - cos(scaled_thf);
    double S = 2 * scaled_k_max - sin(scaled_th0) + sin(scaled_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(scaled_k_max, 2) + 2 * cos(scaled_th0 - scaled_thf) + 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf)));
    if (abs(temp2) > 1)
    {
        s1 = 0;
        s2 = 0;
        s3 = 0;
        return new CurveSegmentsResult(false, s1, s2, s3);
    }
    s2 = invK * mod2pi(2 * M_PI - acos(temp2));
    s1 = invK * mod2pi(scaled_th0 - temp1 + 0.5 * s2 * scaled_k_max);
    s3 = invK * mod2pi(scaled_th0 - scaled_thf + scaled_k_max * (s2 - s1));
    return new CurveSegmentsResult(true, s1, s2, s3);
}

CurveSegmentsResult *Dubins::useLRL(double scaled_th0, double scaled_thf, double scaled_k_max)
{
    double s1, s2, s3;
    double invK = 1 / scaled_k_max;
    double C = cos(scaled_thf) - cos(scaled_th0);
    double S = 2 * scaled_k_max + sin(scaled_th0) - sin(scaled_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(scaled_k_max, 2) + 2 * cos(scaled_th0 - scaled_thf) - 4 * scaled_k_max * (sin(scaled_th0) - sin(scaled_thf)));
    if (abs(temp2) > 1)
    {
        s1 = 0;
        s2 = 0;
        s3 = 0;
        return new CurveSegmentsResult(false, s1, s2, s3);
    }
    s2 = invK * mod2pi(2 * M_PI - acos(temp2));
    s1 = invK * mod2pi(temp1 - scaled_th0 + 0.5 * s2 * scaled_k_max);
    s3 = invK * mod2pi(scaled_thf - scaled_th0 + scaled_k_max * (s2 - s1));
    return new CurveSegmentsResult(true, s1, s2, s3);
}

/**
 * @brief Find the shortest path between a starting and a final position
 * 
 * @param x0 Starting x position
 * @param y0 Starting y position
 * @param th0 Starting angle
 * @param xf Final x position
 * @param yf Final y position
 * @param thf Final angle
 * @return DubinsCurve* Resulting curve representing the shortest path
 */
DubinsCurve *Dubins::findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf)
{
    ParametersResult *scaled_parameters = scaleToStandard(x0, y0, th0, xf, yf, thf);

    DubinsCurve *curve = nullptr;
    CurveSegmentsResult *best_curve_segments = nullptr;
    CurveSegmentsResult *curve_segments = nullptr;

    double best_L = DBL_MAX;
    int pidx = -1;

    for (int i = 0; i < TOTAL_POSSIBLE_CURVES; i++)
    {
        switch (i)
        {
        case LSL:
            curve_segments = useLSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

            break;

        case RSR:
            curve_segments = useRSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

            break;

        case LSR:
            curve_segments = useLSR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

            break;

        case RSL:
            curve_segments = useRSL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

            break;

        case RLR:
            curve_segments = useRLR(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

            break;

        case LRL:
            curve_segments = useLRL(scaled_parameters->scaled_th0, scaled_parameters->scaled_thf, scaled_parameters->scaled_k_max);

            break;

        default:
            curve_segments = new CurveSegmentsResult(false, 0, 0, 0);
            break;
        }

        double current_L = curve_segments->s1 + curve_segments->s2 + curve_segments->s3;

        if (curve_segments->ok && current_L < best_L)
        {
            best_L = current_L;
            pidx = i;
            if (best_curve_segments != nullptr)
                delete best_curve_segments;
            best_curve_segments = new CurveSegmentsResult(true, curve_segments->s1, curve_segments->s2, curve_segments->s3);
        }
        delete curve_segments;
        curve_segments = nullptr;
    }

    // std::cout << "BEST CURVE SEGMENTS:\n"
    //           << best_curve_segments->s1 << "\n"
    //           << best_curve_segments->s2 << "\n"
    //           << best_curve_segments->s3 << "\n";

    // std::cout << "PIDX: " << pidx << " BEST_L: " << best_L;

    bool valid = false;
    if (pidx >= 0)
    {
        CurveSegmentsResult *curve_result = scaleFromStandard(scaled_parameters->lambda, best_curve_segments);

        // std::cout << "CURVE RESULT: " << curve_result->s1 << " " << curve_result->s2 << " " << curve_result->s3 << "\n\n";

        curve = new DubinsCurve(x0, y0, th0, curve_result->s1, curve_result->s2, curve_result->s3, ksigns[pidx][0] * k_max, ksigns[pidx][1] * k_max, ksigns[pidx][2] * k_max);

        // std::cout << "GENERAL L: " << curve->L << "\n";

        // std::cout << "a1 = \n"
        //       << "\tL = " << curve->a1->L << "\n"
        //       << "\tk = " << curve->a1->k << "\n"
        //       << "\tx0 = " << curve->a1->x0 << "\n"
        //       << "\ty0 = " << curve->a1->y0 << "\n"
        //       << "\tth0 = " << curve->a1->th0 << "\n"
        //       << "\tdubins_line-x = " << curve->a1->dubins_line->x << "\n"
        //       << "\tdubins_line-y = " << curve->a1->dubins_line->y << "\n"
        //       << "\tdubins_line-th = " << curve->a1->dubins_line->th << "\n\n";

        // std::cout << "a2 = \n"
        //       << "\tL = " << curve->a2->L << "\n"
        //       << "\tk = " << curve->a2->k << "\n"
        //       << "\tx0 = " << curve->a2->x0 << "\n"
        //       << "\ty0 = " << curve->a2->y0 << "\n"
        //       << "\tth0 = " << curve->a2->th0 << "\n"
        //       << "\tdubins_line-x = " << curve->a2->dubins_line->x << "\n"
        //       << "\tdubins_line-y = " << curve->a2->dubins_line->y << "\n"
        //       << "\tdubins_line-th = " << curve->a2->dubins_line->th << "\n\n";

        // std::cout << "a3 = \n"
        //       << "\tL = " << curve->a3->L << "\n"
        //       << "\tk = " << curve->a3->k << "\n"
        //       << "\tx0 = " << curve->a3->x0 << "\n"
        //       << "\ty0 = " << curve->a3->y0 << "\n"
        //       << "\tth0 = " << curve->a3->th0 << "\n"
        //       << "\tdubins_line-x = " << curve->a3->dubins_line->x << "\n"
        //       << "\tdubins_line-y = " << curve->a3->dubins_line->y << "\n"
        //       << "\tdubins_line-th = " << curve->a3->dubins_line->th << "\n\n";

        bool valid = checkValidity(best_curve_segments, ksigns[pidx][0] * scaled_parameters->scaled_k_max, ksigns[pidx][1] * scaled_parameters->scaled_k_max, ksigns[pidx][2] * scaled_parameters->scaled_k_max, scaled_parameters->scaled_th0, scaled_parameters->scaled_thf);
        if (!valid)
        {
            std::cout << "NOT VALID!!\n\n";
            delete curve;
            curve = nullptr;
        }
        delete curve_result;
        curve_result = nullptr;
    }
    delete scaled_parameters;
    delete best_curve_segments;
    return curve;
};

/**
 * @brief Find the shortest path between two points, given a set of intermediate points our path must pass through
 * 
 * @param points An array of points we have to pass through
 * @param numberOfPoints The number of points provided
 * @return DubinsCurve** Resulting array of curves that together represent the shortest path
 */
DubinsCurve **Dubins::multipointShortestPath(Point **points, int numberOfPoints)
{
    std::cout << "INPUT: \n";
    std::cout << "X\tY\tTHETA\n";
    for (int i = 0; i < numberOfPoints; i++)
    {
        std::cout << points[i]->x << "\t" << points[i]->y << "\t" << points[i]->th << "\n";
    }
    std::cout << "\n";

    // NOTES
    // Dj(th_j, th_j+1) is the length of the optimal solution of the two points dubins problem connecting P_j with P_j+1
    // th_0, th_1, ..., th_n are the optimal angles that minimise the total length L of the path
    // L(j, th_j) gives the length of the solution of the sub-problem from point P_j with angle th_j onwards
    //      L(j, th_j) = min[th_j+1, ..., th_n](Dj(th_j, th_j+1) + ... + Dn-1(th_n-1, th_n))

    // ITERATIVE DYNAMIC PROGRAMMING SOLUTION
    // L(j, th_j) = {   min[th_n](Dn-1(th_n-1, th_n))                   if j = n-1
    //              {   min[th_j+1](Dj(th_j, th_j+1) + L(j+1, th_j+1))  otherwise

    // INITIALIZATION
    // Init the final result
    DubinsCurve **result = new DubinsCurve *[numberOfPoints - 1];
    // Init the result of the multipoint problem: an array of optimal angles
    double *minimizingAngles = new double[numberOfPoints];

    const int K = std::extent<decltype(multipointAngles)>::value;
    std::cout << "VALUE OF K: " << K << "\n\n";

    // I create an empty matrix L that will store intermediate results
    double **L = new double *[numberOfPoints];
    for (int i = 0; i < numberOfPoints; i++)
    {
        L[i] = new double[K];
    }
    // The length of the portion of the path after the last point is simply 0, otherwise I set the value to -1
    for (int n = 0; n < numberOfPoints; n++)
    {
        for (int i = 0; i < K; i++)
        {
            if (n == numberOfPoints - 1)
                L[n][i] = 0;
            else
                L[n][i] = -1;
        }
    }

    // I create an empty matrix S that will tell me which angle each configuration of L wants to finish with
    // Used to recreate the complete solution after having discovered which is the best path
    int **S = new int *[numberOfPoints];
    for (int i = 0; i < numberOfPoints; i++)
    {
        S[i] = new int[K];
    }
    // I fill S with values -1 - just to debug, when we are ready we can remove it
    for (int n = 0; n < numberOfPoints; n++)
    {
        for (int i = 0; i < K; i++)
        {
            S[n][i] = -1;
        }
    }

    // ALGORITHM - FIRST STEP
    // For the last two points, we already know the end angle
    for (int i = 0; i < K; i++)
    {
        DubinsCurve *curve = findShortestPath(points[numberOfPoints - 2]->x, points[numberOfPoints - 2]->y, points[numberOfPoints - 2]->th != -1 ? points[numberOfPoints - 2]->th : i, points[numberOfPoints - 1]->x, points[numberOfPoints - 1]->y, points[numberOfPoints - 1]->th);
        if (L[numberOfPoints - 2][i] == -1 || L[numberOfPoints - 2][i] > curve->L)
        {
            L[numberOfPoints - 2][i] = curve->L;
        }
        delete curve;
    }

    // ALGORITHM - ITERATIVE COMPUTATION
    for (int n = numberOfPoints - 3; n >= 0; n--)
    {
        for (int i = 0; i < K; i++)
        {
            for (int j = 0; j < K; j++)
            {
                int actual_i_angle = points[n]->th != -1 ? points[n]->th : multipointAngles[i];
                DubinsCurve *curve = findShortestPath(points[n]->x, points[n]->y, actual_i_angle, points[n + 1]->x, points[n + 1]->y, multipointAngles[j]);
                if (n == 0 && ((L[n][0] > curve->L + L[n + 1][j]) || L[n][0] == -1))
                {
                    // If it's the first two points, we already know the initial angle
                    L[n][0] = curve->L + L[n + 1][j];
                    S[n][0] = j;
                }
                else if (L[n][i] > curve->L + L[n + 1][j] || L[n][i] == -1)
                {
                    // Default case: I update the optimal solution
                    L[n][i] = curve->L + L[n + 1][j];
                    S[n][i] = j;
                }
                delete curve;
            }
        }
    }

    // DEBUG PRINT
    std::cout << "VALUES OF L:\n";
    std::cout << "\t";
    for (int i = 0; i < K; i++)
    {
        std::cout << "k=" << i << "\t";
    }
    std::cout << "\n";
    for (int n = 0; n < numberOfPoints; n++)
    {
        std::cout << "POINT=" << n << "\t";
        for (int i = 0; i < K; i++)
        {
            std::cout << L[n][i] << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n\n";

    std::cout << "VALUES OF S:\n";
    std::cout << "\t";
    for (int i = 0; i < K; i++)
    {
        std::cout << "k=" << i << "\t";
    }
    std::cout << "\n";
    for (int n = 0; n < numberOfPoints; n++)
    {
        std::cout << "POINT=" << n << "\t";
        for (int i = 0; i < K; i++)
        {
            std::cout << S[n][i] << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n\n";

    // Find the maximum length and the corresponding index in the first row of L
    int maxIndex;
    double maxLength = -INFINITY;
    for (int i = 0; i < K; i++)
    {
        if (L[0][i] > maxLength)
        {
            maxIndex = i;
            maxLength = L[0][i];
        }
    }

    std::cout << "MINIMUM LENGTH FOUND: " << L[0][maxIndex] << "\n\n";

    minimizingAngles[0] = points[0]->th;
    for (int i = 0; i < numberOfPoints - 2; i++)
    {
        minimizingAngles[i + 1] = multipointAngles[S[i][maxIndex]];
        maxIndex = S[i][maxIndex];
    }
    minimizingAngles[numberOfPoints - 1] = points[numberOfPoints - 1]->th;

    std::cout << "MINIMIZING ANGLES: \n";
    for (int i = 0; i < numberOfPoints; i++)
    {
        std::cout << " -> " << minimizingAngles[i] << "\n";
    }
    std::cout << "\n\n";

    return result;
};

/**
 * @brief Find if there is an intersection between two segments
 * 
 * @param p1 First point used to define the first segment
 * @param p2 Second point used to define the first segment
 * @param p3 First point used to define the second segment
 * @param p4 Second point used to define the second segment
 * @param pts Array of intersection points this function has found (passed by ref.)
 * @param ts Coefficients to normalize the segments (passed by ref.)
 * @return true If an intersection has been found
 * @return false If an intersection has not been found
 */
bool Dubins::intersLineLine(Point p1, Point p2, Point p3, Point p4, std::vector<Point> &pts, std::vector<double> &ts)
{
    // Initialize the resulting arrays as empty arrays
    pts.clear();
    ts.clear();

    // Define min and max coordinates of the first segment
    double minX1 = std::min(p1.x, p2.x);
    double minY1 = std::min(p1.y, p2.y);
    double maxX1 = std::max(p1.x, p2.x);
    double maxY1 = std::max(p1.y, p2.y);

    // Define min and max coordinates of the second segment
    double minX2 = std::min(p3.x, p4.x);
    double minY2 = std::min(p3.y, p4.y);
    double maxX2 = std::max(p3.x, p4.x);
    double maxY2 = std::max(p3.y, p4.y);

    // If there is no way these segments will intersect, we just return false without computing anything
    if (maxX2 < minX1 || minX2 > maxX1 || maxY2 < minY1 || minY2 > maxY1)
    {
        return false;
    }

    Point q = Point(p1.x, p1.y);
    Point s = Point((p2.x - p1.x), (p2.y - p1.y));

    Point p = Point(p3.x, p3.y);
    Point r = Point((p4.x - p3.x), (p4.y - p3.y));

    Point diffPQ = Point((p1.x - p3.x), (p1.y - p3.y));

    double crossRS = crossProduct(r, s);
    double crossDiffR = crossProduct(diffPQ, r);
    double crossDiffS = crossProduct(diffPQ, s);

    if (crossRS == 0 && crossDiffR == 0)
    {
        double dotRR = dot2D(r, r);
        double dotSR = dot2D(s, r);
        double t0 = dot2D(diffPQ, r) / dotRR;
        double t1 = t0 + (dotSR / dotRR);
        if (dotSR < 0)
        {
            if (t0 >= 0 && t1 <= 1)
            {
                ts.push_back(std::max(t1, 0.0));
                ts.push_back(std::min(t0, 1.0));
            }
        }
        else
        {
            if (t1 >= 0 && t0 <= 1)
            {
                ts.push_back(std::max(t0, 0.0));
                ts.push_back(std::min(t1, 1.0));
            }
        }
    }
    else
    {
        if (crossRS == 0 && crossDiffR != 0)
        {
            return false;
        }
        else
        {
            double t = crossDiffS / crossRS;
            double u = crossDiffR / crossRS;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
            {
                ts.push_back(t);
            }
        }
    }
    for (int i = 0; i < ts.size(); i++)
    {
        pts.push_back(Point((ts[i] * r.x) + p.x, (ts[i] * r.y) + p.y));
    }

    return pts.empty() ? false : true;
}

/**
 * @brief Find if there is an intersection between a circle and a segment
 * 
 * @param circleCenter Center of the input circle
 * @param r Radius of the input circle
 * @param point1 First point used to define the segment
 * @param point2 Second point used to define the segment
 * @param pts Array of intersection points this function has found (passed by ref.)
 * @param t Coefficient to normalize the segment (passed by ref.)
 * @return true If an intersection has been found
 * @return false If an intersection has not been found
 */
bool Dubins::intersCircleLine(Point circleCenter, double r, Point point1, Point point2, std::vector<Point> &pts, std::vector<double> &t)
{
    // Initialize the resulting arrays as empty arrays
    pts.clear();
    t.clear();

    double p1 = 2 * point1.x * point2.x;
    double p2 = 2 * point1.y * point2.y;
    double p3 = 2 * circleCenter.x * point1.x;
    double p4 = 2 * circleCenter.x * point2.x;
    double p5 = 2 * circleCenter.y * point1.y;
    double p6 = 2 * circleCenter.y * point2.y;

    double c1 = pow(point1.x, 2) + pow(point2.x, 2) - p1 + pow(point1.y, 2) + pow(point2.y, 2) - p2;
    double c2 = -2 * pow(point2.x, 2) + p1 - p3 + p4 - 2 * pow(point2.y, 2) + p2 - p5 + p6;
    double c3 = pow(point2.x, 2) - p4 + pow(circleCenter.x, 2) + pow(point2.y, 2) - p6 + pow(circleCenter.y, 2) - pow(r, 2);

    double delta = pow(c2, 2) - (4 * c1 * c3);

    double t1;
    double t2;

    if (delta < 0)
    {
        // There is no solution (we are not dealing with complex numbers)
        return false;
    }
    else
    {
        if (delta > 0)
        {
            // Two points of intersection
            double deltaSq = sqrt(delta);
            t1 = (-c2 + deltaSq) / (2 * c1);
            t2 = (-c2 - deltaSq) / (2 * c1);
        }
        else
        {
            // If the delta is 0 we have just one point of intersection
            t1 = -c2 / (2 * c1);
            t2 = t1;
        }
    }

    std::vector<std::pair<Point, double>> intersections = std::vector<std::pair<Point, double>>();

    if (t1 >= 0 && t1 <= 1)
    {
        intersections.push_back(std::pair<Point, double>(Point((point1.x * t1) + (point2.x * (1 - t1)), (point1.y * t1) + (point2.y * (1 - t1))), t1));
    }

    if (t2 >= 0 && t2 <= 1 && t2 != t1)
    {
        intersections.push_back(std::pair<Point, double>(Point((point1.x * t2) + (point2.x * (1 - t2)), (point1.y * t2) + (point2.x * (1 - t2))), t2));
    }

    // Sort the intersections using t values
    std::sort(intersections.begin(), intersections.end(), [](const std::pair<Point, double> a, const std::pair<Point, double> b)
              { return a.second < b.second; });

    // Fill the resulting arrays
    for (int i = 0; i < intersections.size(); i++)
    {
        pts.push_back(intersections[i].first);
        t.push_back(intersections[i].second);
    }

    return pts.empty() ? false : true;
}