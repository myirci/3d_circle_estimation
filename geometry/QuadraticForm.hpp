#ifndef QUADRATIC_FORM_HPP
#define QUADRATIC_FORM_HPP

#include <Eigen/Dense>

// quadraticform in three variables
struct QuadraticForm{
    // x1, x2, x3
    // A*x1*x1 + B*x1*x2 + C*x2*x2 + D*x1*x3 + E*x2*x3 + F*x3*x3 = 0
    double A, B, C, D, E, F;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Matrix3d mat;

    QuadraticForm(double a, double b, double c, double d, double e, double f) :
        A(a), B(b), C(c), D(d), E(e), F(f) {
        mat << A, B/2, D/2,
               B/2, C, E/2,
               D/2, E/2, F;
        eigensolver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(mat);
    }

    QuadraticForm& operator*=(double k) {
        A*=k; B*=k; C*=k; D*=k; E*=k; F*=k;
        Eigen::Matrix3d _mat;
        _mat << A, B/2, D/2,
                B/2, C, E/2,
                D/2, E/2, F;
        mat = _mat;
        eigensolver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(mat);
        return *this;
    }

    double evaluate(const Eigen::Vector3d& vec) {
        return vec.transpose()*mat*vec;
    }

};

#endif // QUADRATIC_FORM_HPP
