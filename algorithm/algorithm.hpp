#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP

#include <Eigen/Dense>

class Ellipse2D;
class Circle3D;

/**********************************************************************************/

void estimate_3D_circles_under_perspective_transformation_method_1(
        const Eigen::Matrix4d& pMat,
        const Ellipse2D& ellipse,
        Circle3D* circles);

bool check_eigenvalue_constraints(const Eigen::Vector3d& eigenvalues);

/**********************************************************************************/

void estimate_3D_circles_under_perspective_transformation_method_2(
        const Ellipse2D& ellipse,
        Circle3D* circles,
        double near);

void construct_change_of_basis_matrix(Eigen::Matrix3d& mat, const Eigen::Vector3d& vec2);

/**********************************************************************************/

void estimate_3D_circles_under_perspective_transformation_method_3(
        const Eigen::Matrix3d& mat,
        const Ellipse2D& ellipse,
        Circle3D* circles);

/**********************************************************************************/

int estimate_3D_circles_under_perspective_transformation_method_4(
        const Ellipse2D& ellipse,
        Circle3D* circles,
        double near);

/**********************************************************************************/
#endif // ALGORITHM_HPP
