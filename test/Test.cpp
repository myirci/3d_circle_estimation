#include "../geometry/Ellipse2D.hpp"
#include "../geometry/Circle3D.hpp"
#include "../render/renderer.hpp"
#include "../utility/utility.hpp"
#include "../algorithm/algorithm.hpp"
#include <iostream>
#include <sstream>
#include <vector>

void test_1() {

    int glx(0), gly(0), scx(0), scy(0);
    double xndc(0.0), yndc(0.0), zndc(0.0), depth(0.0);
    double xclip(0.0), yclip(0.0), zclip(0.0), wclip(0.0);

    std::cout <<"\n*************************\n" << std::endl;

    renderer myrenderer(640, 480);
    myrenderer.gluPerspective(45.0, 0.1, 20.0);
    myrenderer.print_converted_parameters(45.0, 0.1, 20.0);

    std::cout <<"\n*************************\n" << std::endl;

    myrenderer.print_projection_matrix();

    std::cout <<"\n*************************\n" << std::endl;

    std::vector<Eigen::Vector3d> data;
    data.push_back(Eigen::Vector3d(0.0137, 0.02123 , -0.1));
    myrenderer.render_test(data, glx, gly, scx, scy,
                           depth, xndc, yndc, zndc,
                           xclip, yclip, zclip, wclip);

    std::cout <<"\n*************************\n" << std::endl;

    std::cout << "Clip Coordinates:\n"
              << xclip << " " << yclip << " "
              << zclip << " " << wclip << " " << std::endl;
    std::cout << "\nNormalized device coordinates:\n"
              << xndc << " " << yndc << " " << zndc << std::endl;

    std::cout <<"\n*************************\n" << std::endl;

    std::cout << "OpenGL Screen Coordinates:\n" << glx << " " << gly << std::endl;
    std::cout << "\nScreen_coordinates:\n" << scx << " " << scy << std::endl;
    std::cout << "\ndepth_val:\n" << depth << std::endl;

    std::cout <<"\n*************************\n" << std::endl;

    // Inversion from clip coordinates
    Eigen::Matrix4d mat;
    myrenderer.get_inverse_projection_matrix(45.0, 0.1, 20.0, mat);
    Eigen::Vector4d vec = mat*Eigen::Vector4d(xclip, yclip, zclip, wclip);
    std::cout << "Inverted coordinates from clip coordinates:\n"
              << vec/vec(3) << std::endl;

    std::cout <<"\n*************************\n" << std::endl;

    // Inversion from Normalized device coordinates
    vec = mat*Eigen::Vector4d(xndc, yndc, zndc, 1);
    std::cout << "Inverted coordinates from normalized device coordinates:\n"
              << vec/vec(3) << std::endl;

    std::cout <<"\n*************************\n" << std::endl;

    // Inversion from OpenGL Screen Coordinates:
    double xn = glx * (2.0/640.0) - 1;
    double yn = gly * (2.0/480.0) - 1;
    double zn = 2*depth - 1;
    std::cout << "Calculated normalized device coordinates:\n"
              << xn << " " << yn << " " << zn << std::endl;
    vec = mat*Eigen::Vector4d(xn, yn, zn, 1);
    std::cout << "\nInverted coordinates from OpenGL screen coordinates:\n"
              << vec/vec(3) << std::endl;

    std::cout <<"\n*************************\n" << std::endl;

    // Inversion from WxWidgets Screen Coordinates:
    // 1) wxWidgets to OpenGL
    int gl_x = scx + 640/2.0;
    int gl_y = scy + 480/2.0;
    std::cout << "\n Calculated OpenGL Screen Coordinates:\n"
              << gl_x << " " << gl_y << std::endl;
    // 2) OpenGL to NDC
    double x_n = gl_x * (2.0/640.0) - 1;
    double y_n = gl_y * (2.0/480.0) - 1;
    double z_n = 2*depth - 1;
    std::cout << "\nCalculated normalized device coordinates:\n"
              << x_n << " " << y_n << " " << z_n << std::endl;
    // 3) NDC to Eye Coordinates
    vec = mat*Eigen::Vector4d(x_n, y_n, z_n, 1);
    std::cout << "\nInverted coordinates from wxWidgets screen coordinates:\n"
              << vec/vec(3) << std::endl;
    std::cout <<"\n*************************\n" << std::endl;
}
void test_2() {
    renderer myrenderer(640, 480);
    myrenderer.gluPerspective(45.0, 0.1, 20.0);
    Circle3D circle(Eigen::Vector3d(1, 2, 3),
                    Eigen::Vector3d(1, 1.5, 2.5), 0.5);
    std::vector<Eigen::Vector3d> data;
    circle.generate_data(data, 80);

    Eigen::MatrixXd vertices(4, data.size());
    for(int i = 0; i < data.size(); ++i) {
        vertices(0,i) = data[i](0);
        vertices(1,i) = data[i](1);
        vertices(2,i) = data[i](2);
        vertices(3,i) = 1;
    }

    Eigen::Matrix4d projMat;
    myrenderer.get_projection_matrix(projMat);

    Eigen::MatrixXd projected_vertices(4, data.size());
    projected_vertices = projMat*vertices;

    // Next-step: Fit ellipse to these projected vertices
    // Then estimate the 3D circle whose projection matches with the
    // calculated ellipse

}

int main() {
    return 0;
}
