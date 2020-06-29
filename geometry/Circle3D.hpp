#ifndef CIRCLE3D_HPP
#define CIRCLE3D_HPP

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include<Eigen/Dense>
#include "../utility/random_num_generator.hpp"
#include "../utility/utility.hpp"


struct Circle3D
{
    double radius;
    Eigen::Vector3d center;
    Eigen::Vector3d normal;

    Circle3D() : radius(0.0), center(Eigen::Vector3d::Zero()), normal(Eigen::Vector3d::Zero()) { }

    Circle3D(const Eigen::Vector3d& c, const Eigen::Vector3d& n, double r) :
        center(c), normal(n.normalized()), radius(r) { }

    void generate_data(std::vector<Eigen::Vector3d>& data, int num_points)
    {
        // initialize the random number generator
        random_num_generator gen;
        gen.initialize_uniform_double_distributor(-2.0, 2.0);

        // Construct the coordinate frame of the 3d circle find 3 othonormal basis
        // vectors: 2 in the plane of the circle and one along the direction of the
        // normal
        Eigen::Vector3d e1, e2;
        Eigen::Vector3d random_vec(gen.generate_double(),
                                   gen.generate_double(),
                                   gen.generate_double());
        e1 = normal.cross(random_vec);
        e1.normalize();
        assert(e1 != Eigen::Vector3d::Zero());
        e2 = normal.cross(e1);
        e2.normalize();

        // Check that e1 and e2 is on the plane:
        Eigen::Vector4d plane(normal(0), normal(1), normal(2),
                              -normal(0)*center(0)-normal(1)*center(1)-normal(2)*center(2));
        Eigen::Vector3d v1 = e1 + center;
        Eigen::Vector3d v2 = e2 + center;
        assert(plane.dot(Eigen::Vector4d(v1(0), v1(1), v1(2), 1)) < 0.0000000001);
        assert(plane.dot(Eigen::Vector4d(v2(0), v2(1), v2(2), 1)) < 0.0000000001);

        // Construct the data points for the given 3d circle
        const double step = TWO_PI/static_cast<double>(num_points);
        Eigen::Vector3d temp;
        for(double t = 0.0; t < TWO_PI; t+= step)
        {
            temp = center + radius*cos(t)*e1 + radius*sin(t)*e2;
            data.push_back(temp);
        }
    }

    void print() const
    {
        std::cout << "center:\n" << center << std::endl;
        std::cout << "normal:\n" << normal << std::endl;
        std::cout << "radius:\n" << radius << std::endl;
    }
};


#endif // CIRCLE3D_HPP
