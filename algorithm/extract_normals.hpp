#ifndef EXTRACT_NORMALS_HPP
#define EXTRACT_NORMALS_HPP

#include <ceres/ceres.h>

struct F1 {
    F1(double s) : scalar(s) { }
    template <typename T>
    bool operator()(const T* const v1, const T* const w1, T* residual) const {
        residual[0] = v1[0] * w1[0] + T(scalar);
        return true;
    }
    double scalar;
};

struct F2 {
    F2(double s) : scalar(s) { }
    template <typename T>
    bool operator()(const T* const v2, const T* const w2, T* residual) const {
        residual[0] = v2[0] * w2[0] + T(scalar);
        return true;
    }
    double scalar;
};

struct F3 {
    F3(double s) : scalar(s) { }
    template <typename T>
    bool operator()(const T* const v3, const T* const w3, T* residual) const {
        residual[0] = v3[0] * w3[0] + T(scalar);
        return true;
    }
    double scalar;
};

struct F4 {
    F4(double s) : scalar(s) { }
    template <typename T>
    bool operator()(const T* const v1, const T* const v2,
                    const T* const w1, const T* const w2,
                    T* residual) const {
        residual[0] = v1[0] * w2[0] + v2[0] * w1[0] + T(scalar);
        return true;
    }
    double scalar;
};

struct F5 {
    F5(double s) : scalar(s) { }
    template <typename T>
    bool operator()(const T* const v2, const T* const v3,
                    const T* const w2, const T* const w3,
                    T* residual) const {
        residual[0] = v2[0] * w3[0] + v3[0] * w2[0] + T(scalar);
        return true;
    }
    double scalar;
};

struct F6 {
    F6(double s) : scalar(s) { }
    template <typename T>
    bool operator()(const T* const v1, const T* const v3,
                    const T* const w1, const T* const w3,
                    T* residual) const {
        residual[0] = v1[0] * w3[0] + v3[0] * w1[0] + T(scalar);
        return true;
    }
    double scalar;
};

void solve_for_normals(const Eigen::Matrix3d& mat,
                       Eigen::Vector3d& n1,
                       Eigen::Vector3d& n2) {

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(mat);
    double k2 = eigensolver.eigenvalues()(1);
    n1 = Eigen::Vector3d::Ones();
    n2 = Eigen::Vector3d::Ones();

    ceres::Problem problem;
    problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1(k2-mat(0,0))),
                NULL, &n1(0), &n2(0));
    problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2(k2-mat(1,1))),
                NULL, &n1(1), &n2(1));
    problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3(k2-mat(2,2))),
                NULL, &n1(2), &n2(2));
    problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<F4, 1, 1, 1, 1, 1>(new F4(-2*mat(0,1))),
                NULL, &n1(0), &n1(1), &n2(0), &n2(1));
    problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<F5, 1, 1, 1, 1, 1>(new F5(-2*mat(1, 2))),
                NULL, &n1(1), &n1(2), &n2(1), &n2(2));
    problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<F6, 1, 1, 1, 1, 1>(new F6(-2*mat(0, 2))),
                NULL, &n1(0), &n1(2), &n2(0), &n2(2));

    ceres::Solver::Options options;
    ceres::StringToMinimizerType("trust_region", &options.minimizer_type);
    options.max_num_iterations = 100;
    options.max_trust_region_radius = 1e6;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    n1.normalize();
    n2.normalize();
}

#endif // EXTRACT_NORMALS_HPP
