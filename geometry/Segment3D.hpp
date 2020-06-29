#ifndef SEGMENT_3D_HPP
#define SEGMENT_3D_HPP

#include <vector>
#include <Eigen/Dense>

struct Segment3D
{
    Eigen::Vector3d pt0, pt1;

    Segment3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) : pt0(p0), pt1(p1) { }

    void generate_data(std::vector<Eigen::Vector3d>& data, int num_points)
    {
        Eigen::Vector3d v = pt1 - pt0;
        for(int i = 0; i < num_points; ++i)
        {
            data.push_back(pt0 + ((1.0/(num_points-1))*i)*v);
        }
    }
};

#endif // SEGMENT_3D_HPP
