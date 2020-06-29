#include "algorithm.hpp"
#include "../geometry/Ellipse2D.hpp"
#include "../geometry/Circle3D.hpp"
#include "../geometry/QuadraticForm.hpp"
#include "extract_normals.hpp"

// Based on the paper by Johan Philip
// An algorithm for determining the position of a circle in 3D from its perspective
// 2D projection - 1997
void estimate_3D_circles_under_perspective_transformation_method_1(
        const Eigen::Matrix4d& pMat,
        const Ellipse2D& ellipse,
        Circle3D* circles)
{
    double k[6];
    k[0] = ellipse.coeff[0] * pMat(0,0) * pMat(0,0);  // A
    k[1] = ellipse.coeff[1] * pMat(0,0) * pMat(1,1);  // B
    k[2] = ellipse.coeff[2] * pMat(1,1) * pMat(1,1);  // C
    k[3] = ellipse.coeff[3] * -pMat(0,0);             // D
    k[4] = ellipse.coeff[4] * -pMat(1,1);             // E
    k[5] = ellipse.coeff[5];

    QuadraticForm qform(k[0], k[1], k[2], k[3], k[4], k[5]);
    if(!check_eigenvalue_constraints(qform.eigensolver.eigenvalues())) {
        qform *= -1;
        if(!check_eigenvalue_constraints(qform.eigensolver.eigenvalues())) {
            std::cerr << "Eigenvales does not macth with the ellipse constraints"
                      << std::endl;
        }
    }

    double lambda1 = qform.eigensolver.eigenvalues()(2);
    double lambda2 = qform.eigensolver.eigenvalues()(1);
    double lambda3 = qform.eigensolver.eigenvalues()(0);

    Eigen::Matrix3d hMat;
    hMat.col(0) = qform.eigensolver.eigenvectors().col(2);
    hMat.col(1) = qform.eigensolver.eigenvectors().col(1);
    hMat.col(2) = qform.eigensolver.eigenvectors().col(0);

    double cosqs = (lambda2 - lambda3) / (lambda1 - lambda3);
    double cosq = std::sqrt(cosqs);
    double sinq = std::sqrt(1.0-cosqs);

    Eigen::Vector3d normal1 = hMat*Eigen::Vector3d(-sinq, 0, cosq);
    Eigen::Vector3d normal2 = -normal1;
    double kk = circles[0].radius / sqrt(-lambda1*lambda3);
    Eigen::Vector3d pos1 = hMat * Eigen::Vector3d(-lambda3*sinq, 0, lambda1*cosq);
    pos1 *= kk;
    Eigen::Vector3d pos2 = -pos1;

    circles[0].normal = normal1.normalized();
    circles[0].center = pos1;
    circles[1].normal = normal2.normalized();
    circles[1].center = pos2;

    Eigen::Vector3d f1(cosq, 1, sinq);
    f1 *= (1.0/ sqrt(2.0));
    Eigen::Vector3d f2(-cosq, 1, -sinq);
    f2 *= (1.0/ sqrt(2.0));
}

bool check_eigenvalue_constraints(const Eigen::Vector3d& eigenvalues) {
    int pos(0), neg(0);
    if(eigenvalues(0) > 0) { ++pos;} else if(eigenvalues(0) < 0) { ++neg; }
    else { std::cerr << "Eigenvalue equal to zero" << std::endl; }
    if(eigenvalues(1) > 0) { ++pos;} else if(eigenvalues(1) < 0) { ++neg; }
    else { std::cerr << "Eigenvalue equal to zero" << std::endl; }
    if(eigenvalues(2) > 0) { ++pos;} else if(eigenvalues(2) < 0) { ++neg; }
    else { std::cerr << "Eigenvalue equal to zero" << std::endl; }
    if(pos == 2 && neg == 1) { return true; }
    return false;
}


/**********************************************************************************/
// Based on the paper by M. Ferri, F. Mangili and G.Viano
// Projective Pose Estimation of Linear and Quadratic Primitives in Monocular
// Computer Vision- 1993
void estimate_3D_circles_under_perspective_transformation_method_2(
        const Ellipse2D& ellipse,
        Circle3D* circles,
        double near) {

    Eigen::Matrix3d M;
    M << ellipse.coeff[0],          ellipse.coeff[1]/2.0,      ellipse.coeff[3]/(2*near),
         ellipse.coeff[1]/2.0,      ellipse.coeff[2],          ellipse.coeff[4]/(2*near),
         ellipse.coeff[3]/(2*near), ellipse.coeff[4]/(2*near), ellipse.coeff[5]/(near*near);

    Eigen::Vector3d n1, n2, n3, n4;
    solve_for_normals(M, n1, n2);
    n3 = -n1;
    n4 = -n2;

    std::cout << "\n**************************************************\n\n";
    std::cout << "n1:\n" << n1 << std::endl;
    std::cout << "n2:\n" << n2 << std::endl;
    std::cout << "n3:\n" << n3 << std::endl;
    std::cout << "n4:\n" << n4 << std::endl;
    std::cout << "\n**************************************************\n\n";

    Eigen::Matrix3d E1, E2, E3, E4;
    construct_change_of_basis_matrix(E1, n1);
    construct_change_of_basis_matrix(E2, n2);
    construct_change_of_basis_matrix(E3, n3);
    construct_change_of_basis_matrix(E4, n4);

    Eigen::Matrix3d Mp1 = E1.transpose()*M*E1;
    Eigen::Matrix3d Mp2 = E2.transpose()*M*E2;
    Eigen::Matrix3d Mp3 = E3.transpose()*M*E3;
    Eigen::Matrix3d Mp4 = E4.transpose()*M*E4;

    int s1 = (Mp1(0,0) >= 0 ? 1 : -1);
    double c1 = (s1*circles[0].radius) /
                std::sqrt(Mp1(0,2)*Mp1(0,2) + Mp1(1,2)*Mp1(1,2) - Mp1(2,2)*Mp1(0,0));
    int s2 = (Mp2(0,0) >= 0 ? 1 : -1);
    double c2 = (s2*circles[1].radius) /
                std::sqrt(Mp2(0,2)*Mp2(0,2) + Mp2(1,2)*Mp2(1,2) - Mp2(2,2)*Mp2(0,0));
    int s3 = (Mp3(0,0) >= 0 ? 1 : -1);
    double c3 = (s3*circles[2].radius) /
                std::sqrt(Mp3(0,2)*Mp3(0,2) + Mp3(1,2)*Mp3(1,2) - Mp3(2,2)*Mp3(0,0));
    int s4 = (Mp4(0,0) >= 0 ? 1 : -1);
    double c4 = (s4*circles[3].radius) /
                std::sqrt(Mp4(0,2)*Mp4(0,2) + Mp4(1,2)*Mp4(1,2) - Mp4(2,2)*Mp4(0,0));

    Eigen::Vector3d center1(-Mp1(0,2), -Mp1(1,2), Mp1(0,0));
    Eigen::Vector3d center2(-Mp2(0,2), -Mp2(1,2), Mp2(0,0));
    Eigen::Vector3d center3(-Mp3(0,2), -Mp3(1,2), Mp3(0,0));
    Eigen::Vector3d center4(-Mp4(0,2), -Mp4(1,2), Mp4(0,0));

    center1 *= c1;
    center1 = E1*center1;
    center2 *= c2;
    center2 = E2*center2;
    center3 *= c3;
    center3 = E3*center3;
    center4 *= c4;
    center4 = E4*center4;

    circles[0].center = center1;
    circles[0].normal = n1;
    circles[1].center = center2;
    circles[1].normal = n2;
    circles[2].center = center3;
    circles[2].normal = n3;
    circles[3].center = center4;
    circles[3].normal = n4;

    std::cout << "Center-1:\n" << center1 << std::endl;
    std::cout << "Center-2:\n" << center2 << std::endl;
    std::cout << "Center-3:\n" << center3 << std::endl;
    std::cout << "Center-4:\n" << center4 << std::endl;

    std::cout << "\n**************************************************\n\n";

}

void construct_change_of_basis_matrix(Eigen::Matrix3d& mat, const Eigen::Vector3d& vec2) {
    Eigen::Vector3d vec0, vec1;
    if(std::abs(vec2(0)) < std::abs(vec2(1)) && std::abs(vec2(0)) < std::abs(vec2(2))) {
        vec0(0) = 0;
        vec0(1) = vec2(2);
        vec0(2) = -vec2(1);
    }
    else if(std::abs(vec2(1)) < std::abs(vec2(0)) && std::abs(vec2(1)) < std::abs(vec2(2))) {
        vec0(0) = -vec2(2);
        vec0(1) = 0;
        vec0(2) = vec2(0);
    }
    else {
        vec0(0) = vec2(1);
        vec0(1) = -vec2(0);
        vec0(2) = 0;
    }
    vec1 = vec2.cross(vec0);
    vec0.normalize();
    vec1.normalize();

    mat(0, 0) = vec0(0);
    mat(1, 0) = vec0(1);
    mat(2, 0) = vec0(2);

    mat(0, 1) = vec1(0);
    mat(1, 1) = vec1(1);
    mat(2, 1) = vec1(2);

    mat(0, 2) = vec2(0);
    mat(1, 2) = vec2(1);
    mat(2, 2) = vec2(2);
}

/**********************************************************************************/
// Mathiue Bredif's Method - 2014
void estimate_3D_circles_under_perspective_transformation_method_3(
        const Eigen::Matrix3d& mat,
        const Ellipse2D& ellipse,
        Circle3D* circles) {

    Eigen::Matrix3d C;
    C << ellipse.coeff[0],       ellipse.coeff[1]/2.0,   ellipse.coeff[3]/2.0,
         ellipse.coeff[1]/2.0,   ellipse.coeff[2],       ellipse.coeff[4]/2.0,
         ellipse.coeff[3]/2.0,   ellipse.coeff[4]/2.0,   ellipse.coeff[5];

    Eigen::Matrix3d Cinv = C.inverse();
    Eigen::Matrix3d Minv = mat.inverse();
    Eigen::Matrix3d E = Minv*Cinv*Minv.transpose();
    if(E.determinant() < 0) { E = - E; }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_decomposer(E);

    double c1 = std::sqrt(eigen_decomposer.eigenvalues()(2) -
                          eigen_decomposer.eigenvalues()(0));
    double c2 = std::sqrt(eigen_decomposer.eigenvalues()(1) -
                          eigen_decomposer.eigenvalues()(0));

    Eigen::Vector3d v1 = c1 * eigen_decomposer.eigenvectors().col(2);
    Eigen::Vector3d v2 = c2 * eigen_decomposer.eigenvectors().col(1);

    double mu2 = std::sqrt(-eigen_decomposer.eigenvalues()(1));
    double mu1 = std::sqrt(eigen_decomposer.eigenvalues()(2));

    circles[0].normal = mu2*v1 + mu1*v2;
    circles[0].center = mu1*v1 - mu2*v2;
    circles[0].center /= circles[0].normal.norm();
    circles[0].normal.normalize();

    circles[1].normal = mu2*v1 - mu1*v2;
    circles[1].center = mu1*v1 + mu2*v2;
    circles[1].center /= circles[1].normal.norm();
    circles[1].normal.normalize();

    circles[2].normal = -mu2*v1 - mu1*v2;
    circles[2].center = mu1*v1 - mu2*v2;
    circles[2].center /= circles[2].normal.norm();
    circles[2].normal.normalize();

    circles[3].normal = -mu2*v1 + mu1*v2;
    circles[3].center = mu1*v1 + mu2*v2;
    circles[3].center /= circles[3].normal.norm();
    circles[3].normal.normalize();

    circles[4].center = -circles[0].center;
    circles[4].normal = -circles[0].normal;

    circles[5].center = - circles[1].center;
    circles[5].normal = - circles[1].normal;

    circles[6].center = - circles[2].center;
    circles[6].normal = - circles[2].normal;

    circles[7].center = - circles[3].center;
    circles[7].normal = - circles[3].normal;
}


/**********************************************************************************/
// Based on the paper by Reza Safaee-Rad, Ivo Tchoukanov, Kenneth Carless Smith, and
// Bensiyon Benhabib
// Three Dimensional Location Estimation of Circular Features for machine vision - 1992

int estimate_3D_circles_under_perspective_transformation_method_4(
        const Ellipse2D& ellipse,
        Circle3D* circles,
        double near) {

    int count = 0;

    // Step-1: Construct the associated quadratic form matrix of the 3D cone.
    Eigen::Matrix3d M;
    M << ellipse.coeff[0],          ellipse.coeff[1]/2.0,      ellipse.coeff[3]/(2*near),
         ellipse.coeff[1]/2.0,      ellipse.coeff[2],          ellipse.coeff[4]/(2*near),
         ellipse.coeff[3]/(2*near), ellipse.coeff[4]/(2*near), ellipse.coeff[5]/(near*near);

    // Step-2: Estimate the orientation of the circles.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M);
    if(eigensolver.info() != Eigen::Success) {
        std::cout << "Eigen solver is not successful!" << std::endl;
    }

    if(!check_eigenvalue_constraints(eigensolver.eigenvalues())) {
        M *= -1;
        eigensolver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(M);
        if(!check_eigenvalue_constraints(eigensolver.eigenvalues())) {
            std::cerr << "Eigenvales does not macth with the ellipse constraints"
                      << std::endl;
        }
    }

    Eigen::Matrix3d P;
    P.col(2) << eigensolver.eigenvectors().col(0); // lamda_3
    double lambda_3 = eigensolver.eigenvalues()(0);

    P.col(0) << eigensolver.eigenvectors().col(1); // lamda_1
    double lambda_1 = eigensolver.eigenvalues()(1);

    P.col(1) << eigensolver.eigenvectors().col(2); // lamda_2
    double lambda_2 = eigensolver.eigenvalues()(2);

    if(P.determinant() < 0) {
        P.col(0) << eigensolver.eigenvectors().col(2); // lamda_1
        P.col(1) << eigensolver.eigenvectors().col(1); // lamda_2
        std::swap(lambda_1, lambda_2);
    }

    double c1 = 0.0;
    double c2 = 0.0;
    double c3 = 0.0;
    double c4 = 0.0;
    double radius = 1.0;

    Eigen::Vector3d center_a;
    Eigen::Vector3d center_b;
    Eigen::Matrix3d mat;

    if(lambda_1 > lambda_2) {
        c1 = std::sqrt((lambda_1-lambda_2)/(lambda_1-lambda_3));
        c2 = std::sqrt((lambda_2-lambda_3)/(lambda_1-lambda_3));
        c3 = radius / std::sqrt(-lambda_1*lambda_3);
        c4 = std::sqrt((lambda_1-lambda_2)*(lambda_2-lambda_3));

        // In the x'y'z' coordinate frame
        center_a = c3* Eigen::Vector3d(0, c4, lambda_2);
        center_b = c3* Eigen::Vector3d(0, -c4, lambda_2);

        circles[0].normal = P * Eigen::Vector3d(c1, 0, c2);
        mat << 0, -c2, c1, 1, 0, 0, 0, c1, c2;
        circles[0].center = P * mat * center_a;

        circles[1].normal = P * Eigen::Vector3d(-c1, 0, c2);
        mat << 0, c2, -c1, -1, 0, 0, 0, c1, c2;
        circles[1].center = P * mat * center_a;

        circles[2].normal = P * Eigen::Vector3d(c1, 0, -c2);
        mat << 0, c2, c1, 1, 0, 0, 0, c1, -c2;
        circles[2].center = P * mat * center_b;

        circles[3].normal = P * Eigen::Vector3d(-c1, 0, -c2);
        mat << 0, -c2, -c1, -1, 0, 0, 0, c1, -c2;
        circles[3].center = P * mat * center_b;

        count = 4;
    }
    else if(lambda_2 > lambda_1) {
        c1 = std::sqrt((lambda_2-lambda_1)/(lambda_2-lambda_3));
        c2 = std::sqrt((lambda_1-lambda_3)/(lambda_2-lambda_3));
        c3 = radius / std::sqrt(-lambda_2*lambda_3);
        c4 = std::sqrt((lambda_2-lambda_1)*(lambda_1-lambda_3));

        // In the x'y'z' coordinate frame
        center_a = c3* Eigen::Vector3d(0, c4, lambda_1);   // c > 0, d < 0
        center_b = c3* Eigen::Vector3d(0, -c4, lambda_1);  // c < 0, d < 0

        circles[0].normal = P * Eigen::Vector3d(0, c1, c2);
        mat << -1, 0, 0, 0, -c2, c1, 0, c1, c2;
        circles[0].center = P * mat * center_a;

        circles[1].normal = P * Eigen::Vector3d(0, -c1, c2);
        mat << 1, 0, 0, 0, c2, -c1, 0, c1, c2;
        circles[1].center = P * mat * center_a;

        circles[2].normal = P * Eigen::Vector3d(0, c1, -c2);
        mat << -1, 0, 0, 0, c2, c1, 0, c1, -c2;
        circles[2].center = P * mat * center_b;

        circles[3].normal = P * Eigen::Vector3d(0, -c1, -c2);
        mat << 1, 0, 0, 0, -c2, -c1, 0, c1, -c2;
        circles[3].center = P * mat * center_b;

        count = 4;
    }
    else {
        circles[0].normal = P * Eigen::Vector3d(0, 0, 1);
        count = 1;
    }
    return count;
}




















































