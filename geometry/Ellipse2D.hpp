#ifndef ELLIPSE_HPP
#define ELLIPSE_HPP

#include <vector>
#include "Vector2.hpp"
#include "QuadraticForm.hpp"

struct Ellipse2D {

public:

    Ellipse2D(double smj = 0.0,
              double smn = 0.0,
              double rt = 0.0,
              const Point2D<double>& pt = Point2D<double>()) :
        semi_major_axis(smj),
        semi_minor_axis(smn),
        rot_angle(rt),
        center(pt) {
        calculate_algebraic_equation_in_wxWidget_coordinates();
    }

    Ellipse2D(double a, double b, double c, double d, double e, double f) {
        coeff[0] = a;
        coeff[1] = b;
        coeff[2] = c;
        coeff[3] = d;
        coeff[4] = e;
        coeff[5] = f;
    }

    void generate_points_on_the_ellipse(int num, double* data) const;
    void generate_points_on_the_ellipse(int num, std::vector<Point2D<double> >& data) const;

    void calculate_algebraic_equation_in_wxWidget_coordinates();
    void calculate_algebraic_equation_in_normalized_device_coordinates(int w, int h);
    void calculate_algebraic_equation_in_projected_coordinates(int w, int h, double n, double half_fovy);

    Point2D<double> points[3];
    Point2D<double> center;
    double rot_angle;
    double semi_minor_axis;
    double semi_major_axis;
    double coeff[6];
};

#endif // ELLIPSE_HPP
