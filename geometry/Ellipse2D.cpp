#include "Ellipse2D.hpp"
#include "../utility/utility.hpp"
#include <cmath>

void Ellipse2D::generate_points_on_the_ellipse(int num, double* data) const {

    double step = TWO_PI/num;
    int i = 0;
    for(double d = 0.0; d < TWO_PI; d += step) {
        data[i++] = semi_major_axis*std::cos(d);
        data[i++] = semi_minor_axis*std::sin(d);
    }

    // rotate and translate
    double x(0.0), y(0.0), cs(std::cos(rot_angle)), sn(std::sin(rot_angle));
    int size = 2*num;
    for(int i = 0; i < size; i += 2) {
        x = data[i];
        y = data[i+1];
        data[i] = x*cs - y*sn + center.x;
        data[i+1] = x*sn + y*cs + center.y;
    }
}

void Ellipse2D::generate_points_on_the_ellipse(int num, std::vector<Point2D<double> >& data) const {

    double step = TWO_PI/num;
    Vector2D<double> e1(cos(rot_angle), sin(rot_angle));
    Vector2D<double> e2(-sin(rot_angle), cos(rot_angle));
    for(double d = 0.0; d < TWO_PI; d += step)
        data.push_back(center + semi_major_axis*cos(d)*e1 + semi_minor_axis*sin(d)*e2);
}

void Ellipse2D::calculate_algebraic_equation_in_wxWidget_coordinates() {

    double as = semi_major_axis * semi_major_axis;
    double bs = semi_minor_axis * semi_minor_axis;
    coeff[0] = 0.5 * (as + bs - cos(2*rot_angle) * (as - bs));
    coeff[1] = (bs - as) * sin(2*rot_angle);
    coeff[2] = 0.5 * (as + bs + cos(2*rot_angle) * (as - bs));
    coeff[3] = -2 * center.x * coeff[0] - center.y * coeff[1];
    coeff[4] = -2 * center.y * coeff[2] - center.x * coeff[1];
    coeff[5] = center.x * center.x * coeff[0] +
               center.x * center.y * coeff[1] +
               center.y * center.y * coeff[2] - as*bs;
}

void Ellipse2D::calculate_algebraic_equation_in_projected_coordinates(int w, int h, double n, double half_fovy) {

    double consx = 2.0/static_cast<double>(w);
    double consy = 2.0/static_cast<double>(h);

    Point2D<double> points_ndc[3];
    points_ndc[0] = Point2D<double>(consx * points[0].x, consy * points[0].y);
    points_ndc[1] = Point2D<double>(consx * points[1].x, consy * points[1].y);
    points_ndc[2] = Point2D<double>(consx * points[2].x, consy * points[2].y);

    double aspect = static_cast<double>(w)/static_cast<double>(h);
    double c1 = n * tan(half_fovy);
    double c2 = aspect * c1;

    Point2D<double> points_projected[3];
    points_projected[0] = Point2D<double>(points_ndc[0].x*c2, points_ndc[0].y*c1);
    points_projected[1] = Point2D<double>(points_ndc[1].x*c2, points_ndc[1].y*c1);
    points_projected[2] = Point2D<double>(points_ndc[2].x*c2, points_ndc[2].y*c1);

    Point2D<double> center_projected((points_projected[0].x + points_projected[1].x)/2.0,
                                     (points_projected[0].y + points_projected[1].y)/2.0);

    double smj = dist(points_projected[0], points_projected[1]) / 2;
    double smn = dist(points_projected[2], center_projected);
    double as = smj * smj;
    double bs = smn * smn;

    coeff[0] = 0.5 * (as + bs - cos(2*rot_angle) * (as - bs));
    coeff[1] = (bs - as) * sin(2*rot_angle);
    coeff[2] = 0.5 * (as + bs + cos(2*rot_angle) * (as - bs));
    coeff[3] = -2 * center_projected.x * coeff[0] - center_projected.y * coeff[1];
    coeff[4] = -2 * center_projected.y * coeff[2] - center_projected.x * coeff[1];
    coeff[5] = center_projected.x * center_projected.x * coeff[0] +
               center_projected.x * center_projected.y * coeff[1] +
               center_projected.y * center_projected.y * coeff[2] - as * bs;
}

void Ellipse2D::calculate_algebraic_equation_in_normalized_device_coordinates(int w, int h) {

    double consx = 2.0/static_cast<double>(w);
    double consy = 2.0/static_cast<double>(h);

    Point2D<double> points_ndc[3];
    points_ndc[0] = Point2D<double>(consx * points[0].x, consy * points[0].y);
    points_ndc[1] = Point2D<double>(consx * points[1].x, consy * points[1].y);
    points_ndc[2] = Point2D<double>(consx * points[2].x, consy * points[2].y);

    Point2D<double> center_ndc((points_ndc[0].x + points_ndc[1].x)/2.0,
                               (points_ndc[0].y + points_ndc[1].y)/2.0);
    double smj = dist(points_ndc[0], points_ndc[1]) / 2;
    double smn = dist(points_ndc[2], center_ndc);
    double as = smj * smj;
    double bs = smn * smn;

    coeff[0] = 0.5 * (as + bs - cos(2*rot_angle) * (as - bs));
    coeff[1] = (bs - as) * sin(2*rot_angle);
    coeff[2] = 0.5 * (as + bs + cos(2*rot_angle) * (as - bs));
    coeff[3] = -2 * center_ndc.x * coeff[0] - center_ndc.y * coeff[1];
    coeff[4] = -2 * center_ndc.y * coeff[2] - center_ndc.x * coeff[1];
    coeff[5] = center_ndc.x * center_ndc.x * coeff[0] +
               center_ndc.x * center_ndc.y * coeff[1] +
               center_ndc.y * center_ndc.y * coeff[2] - as * bs;
}
