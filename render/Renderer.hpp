#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "Screen.hpp"
#include <vector>
#include <Eigen/Dense>

class wxPaintDC;

class renderer {
public:
    // constructors:
    renderer(int width, int height);
    renderer();

    // setter and getters
    void set_screen_width_and_height(int w, int h);
    void set_projection_matrix(const Eigen::Matrix4d& mat);
    void get_projection_matrix(Eigen::Matrix4d& mat) const;
    void get_pure_projection_matrix(Eigen::Matrix4d& mat) const;

    // rendering functions:
    void render(const double* data, const unsigned int num_verts, wxPaintDC& dc);
    void render(const std::vector<Eigen::Vector3d>& data, wxPaintDC& dc);
    void render_test(const std::vector<Eigen::Vector3d>& data,
                     int& glx, int& gly,
                     int& scx, int& scy,
                     double& depth, double& xndc, double& yndc,
                     double& zndc, double& xclip, double& yclip,
                     double& zclip, double& wclip);

    // similar to OpenGL functions
    void glFrustrum(double left, double right, double bottom, double top, double near, double far);
    void gluPerspective(double fovy, double near, double far);
    void gluPerspective_via_glFrustrum(double fovy, double near, double far);
    void glOrtho(double left, double right, double bottom, double top, double near, double far);
    void glViewport(unsigned int x, unsigned int y, unsigned int width, unsigned int height);
    void glDepthRange(double nearVal, double farVal);

    // test and print fucntions
    void print_projection_matrix();
    void print_inverse_projection_matrix();
    void get_inverse_projection_matrix(double fovy, double near, double far, Eigen::Matrix4d& mat);
    void get_inverse_projection_matrix(double left, double right, double bottom, double top, double near, double far, Eigen::Matrix4d& mat);
    void print_converted_parameters(double fovy, double near, double far);
    void print_converted_parameters(double left, double right,
                                  double bottom, double top,
                                  double near, double far);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    screen m_screen;
    Eigen::Matrix4d m_projection;
    Eigen::Matrix4d m_pure_projection;
    void project(const double* data, const unsigned int num_verts, wxPaintDC& dc);
    void project(const std::vector<Eigen::Vector3d>& data, wxPaintDC& dc);
    inline void project(const Eigen::MatrixXd& vertices, wxPaintDC& dc);
    void calculate_pure_projection_matrix(double near, double far);
};

#endif // RENDERER_HPP
