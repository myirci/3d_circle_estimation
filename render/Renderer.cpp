#include "Renderer.hpp"
#include "../utility/utility.hpp"
#include <iostream>
#include <wx/dcclient.h>

renderer::renderer(int width, int height) : m_screen(width, height), m_projection(Eigen::Matrix4d::Identity()) { }

renderer::renderer() : m_screen(0, 0), m_projection(Eigen::Matrix4d::Identity()), m_pure_projection(Eigen::Matrix4d::Identity()) { }

void renderer::set_screen_width_and_height(int w, int h) {
    m_screen.w = w;
    m_screen.h = h;
    m_screen.vpw = w;
    m_screen.vph = h;
}

void renderer::render(const double* data, const unsigned int num_verts, wxPaintDC& dc)
{
    project(data, num_verts, dc);
}

void renderer::render(const std::vector<Eigen::Vector3d>& data, wxPaintDC& dc)
{
    project(data, dc);
}

void renderer::render_test(
        const std::vector<Eigen::Vector3d>& data,
        int& glx, int& gly, int& scx, int& scy, double& depth, double& xndc, double& yndc,
        double& zndc, double& xclip, double& yclip, double& zclip, double& wclip)
{

    Eigen::MatrixXd vertices(4, data.size());
    int i = 0;
    for(auto it = data.begin(); it != data.end(); ++it, ++i) {
        vertices(0,i) = (*it)(0);
        vertices(1,i) = (*it)(1);
        vertices(2,i) = (*it)(2);
        vertices(3,i) = 1.0;
    }

    // Perform projection
    Eigen::MatrixXd projected = m_projection*vertices;
    double half_vpw = static_cast<double>(m_screen.vpw) / 2.0;
    double half_vph = static_cast<double>(m_screen.vph) / 2.0;
    double half_diff_range = (m_screen.depth_far_val - m_screen.depth_near_val)/2;
    double half_sum_range  = (m_screen.depth_far_val + m_screen.depth_near_val)/2;
    // clipping, perspective division and viewport mapping
    for(int i = 0; i < projected.cols(); i++) {
        // these coordinates are called clipping coordinates on which the
        // clipping is applied.
        xclip = projected.col(i)(0);
        yclip = projected.col(i)(1);
        zclip = projected.col(i)(2);
        wclip = projected.col(i)(3);

        if(xclip < -wclip || xclip > wclip ||
           yclip < -wclip || yclip > wclip ||
           zclip < -wclip || zclip > wclip) {
            std::cout << "Coordinate clipped" << std::endl;
            continue;
        }

        // if clipping test is passed, perform perspective division to find
        // normalized device coorniates
        xndc = xclip/wclip;
        yndc = yclip/wclip;
        zndc = zclip/wclip;

        // after getting the normalized device coordinates, viewport mapping is
        // done to get the pixel coordinates. note that OpenGL screen coordinate
        // frame origin is located at the bottom-left corner of the screen. The
        // calculated px and py are according to the OpenGL coordinate system.
        glx = static_cast<int>(half_vpw*(xndc+1) + m_screen.x0);
        gly = static_cast<int>(half_vph*(yndc+1) + m_screen.y0);
        depth = half_diff_range*zndc + half_sum_range;

        // We need to convert from OpenGL screen coordinate system to our
        // windows coordinate system. Our Logical coordinate system's origin is
        // located at the mid point of the screen.
        scx = glx - half_vpw;
        scy = gly - half_vph;
    }
}

void renderer::set_projection_matrix(const Eigen::Matrix4d& mat)
{
    m_projection = mat;
}

void renderer::get_projection_matrix(Eigen::Matrix4d& mat) const
{
    mat = m_projection;
}

void renderer::get_pure_projection_matrix(Eigen::Matrix4d& mat) const
{
    mat = m_pure_projection;
}

void renderer::glFrustrum(double left, double right, double bottom, double top, double near, double far)
{
    m_projection = Eigen::Matrix4d::Zero();
    m_projection(0,0) = (2 * near) / (right - left);
    m_projection(0,2) = (right + left) / (right - left);
    m_projection(1,1) = (2 * near) / (top - bottom);
    m_projection(1,2) = (top + bottom) / (top - bottom);
    m_projection(2,2) = -(far + near) / (far - near);
    m_projection(2,3) = (-2*far*near) / (far - near);
    m_projection(3,2) = -1;
    calculate_pure_projection_matrix(far, near);
}

void renderer::gluPerspective_via_glFrustrum(double fovy, double near, double far)
{
    double aspect = static_cast<double>(m_screen.vpw) / static_cast<double>(m_screen.vph);
    double t = tan(deg2rad(0.5*fovy))*near;
    double b = -t;
    double r = t * aspect;
    double l = -r;
    this->glFrustrum(l, r, b, t, near, far);
}

void renderer::gluPerspective(double fovy, double near, double far)
{
    m_projection = Eigen::Matrix4d::Zero();
    double aspect = static_cast<double>(m_screen.vpw) / static_cast<double>(m_screen.vph);
    double half_fov = deg2rad(0.5*fovy);
    m_projection(0,0) = cos(half_fov)/(sin(half_fov)*aspect);
    m_projection(1,1) = cos(half_fov)/sin(half_fov);
    m_projection(2,2) = -(far + near) / (far - near);
    m_projection(2,3) = (-2*far*near) / (far - near);
    m_projection(3,2) = -1;
    calculate_pure_projection_matrix(far, near);
}

void renderer::get_inverse_projection_matrix(double fovy, double near, double far, Eigen::Matrix4d& mat)
{
    mat = Eigen::Matrix4d::Zero();
    double aspect = static_cast<double>(m_screen.vpw) / static_cast<double>(m_screen.vph);
    mat(0,0) = aspect*tan(deg2rad(0.5*fovy));
    mat(1,1) = tan(deg2rad(0.5*fovy));
    mat(2,3) = -1;
    mat(3,2) = (near-far)/(2*far*near);
    mat(3,3) = (far+near)/(2*far*near);
}

void renderer::get_inverse_projection_matrix(double left, double right, double bottom, double top, double near, double far, Eigen::Matrix4d& mat)
{
    mat = Eigen::Matrix4d::Zero();
    mat(0,0) = (right-left) / (2*near);
    mat(0,3) = (right+left) / (2*near);
    mat(1,1) = (top-bottom) / (2*near);
    mat(1,3) = (top+bottom) / (2*near);
    mat(2,3) = -1;
    mat(3,2) = (near-far)/(2*far*near);
    mat(3,3) = (near+far)/(2*far*near);
}

void renderer::glOrtho(double left, double right, double bottom, double top, double near, double far)
{
    m_projection = Eigen::Matrix4d::Zero();
    m_projection(0,0) = (2) / (right - left);
    m_projection(0,3) = -(right + left) / (right - left);
    m_projection(1,1) = (2) / (top - bottom);
    m_projection(1,3) = -(top + bottom) / (top - bottom);
    m_projection(2,2) = (-2) / (far - near);
    m_projection(2,3) = -(far + near) / (far - near);
    m_projection(3,3) = 1;
    calculate_pure_projection_matrix(far, near);
}

void renderer::glViewport(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
    m_screen.x0 = x;
    m_screen.y0 = y;
    m_screen.vpw = width;
    m_screen.vph = height;
}

void renderer::glDepthRange(double nearVal, double farVal)
{
    m_screen.depth_near_val = nearVal;
    m_screen.depth_far_val = farVal;
}

void renderer::project(const std::vector<Eigen::Vector3d>& data, wxPaintDC& dc)
{
    Eigen::MatrixXd vertices(4, data.size());
    int i = 0;
    for(auto it = data.begin(); it != data.end(); ++it, ++i) {
        vertices(0,i) = (*it)(0);
        vertices(1,i) = (*it)(1);
        vertices(2,i) = (*it)(2);
        vertices(3,i) = 1.0;
    }
    project(vertices, dc);
}

void renderer::project(const double* data, const unsigned int num_verts, wxPaintDC& dc)
{
    // Combine the vertex data into a matrix
    Eigen::MatrixXd vertices(4, num_verts);
    int k = 0;
    for(int i = 0; i < num_verts; ++i) {
        vertices(0,i) = data[k++]; vertices(1,i) = data[k++];
        vertices(2,i) = data[k++]; vertices(3,i) = 1.0;
    }
    project(vertices, dc);
}

void renderer::project(const Eigen::MatrixXd& vertices, wxPaintDC& dc)
{
    // Perform projection
    Eigen::MatrixXd projected = m_projection*vertices;
    int px(0), py(0);
    double x(0.0), y(0.0), z(0.0), w(0.0);
    double half_vpw = static_cast<double>(m_screen.vpw) / 2.0;
    double half_vph = static_cast<double>(m_screen.vph) / 2.0;
    // clipping, perspective division and viewport mapping
    for(int i = 0; i < projected.cols(); i++) {
        // these coordinates are called clipping coordinates on which the
        // clipping is applied.
        x = projected.col(i)(0);
        y = projected.col(i)(1);
        z = projected.col(i)(2);
        w = projected.col(i)(3);
        if(x < -w || x > w || y < -w || y > w || z < -w || z > w) { continue; }

        // if clipping test is passed, perform perspective division to find
        // normalized device coorniates
        x /= w;
        y /= w;
        z /= w;

        // after getting the normalized device coordinates, viewport mapping is
        // done to get the pixel coordinates. note that OpenGL screen coordinate
        // frame origin is located at the bottom-left corner of the screen. The
        // calculated px and py are according to the OpenGL coordinate system.
        px = static_cast<int>(half_vpw*(x+1) + m_screen.x0);
        py = static_cast<int>(half_vph*(y+1) + m_screen.y0);

        // We need to convert from OpenGL screen coordinate system to our
        // windows coordinate system. Our Logical coordinate system's origin is
        // located at the mid point of the screen.
        px = px - half_vpw;
        py = py - half_vph;

        // render the point
        // dc.DrawPoint(px, py);
        dc.DrawLine(px, py, px, py);
    }
}

void renderer::print_projection_matrix()
{
    std::cout << "projection matrix: " << std::endl;
    std::cout << m_projection << std::endl;
}

void renderer::print_inverse_projection_matrix()
{
    std::cout << "inverse projection matrix: " << std::endl;
    if(m_projection.determinant() != 0) {
        std::cout << m_projection.inverse() << std::endl;
    }
    else {
        std::cout << "projection matrix is not invertible" << std::endl;
    }
}

void renderer::print_converted_parameters(double fovy, double near, double far)
{
    double aspect = static_cast<double>(m_screen.vpw) / static_cast<double>(m_screen.vph);
    double half_fov = deg2rad(0.5*fovy);
    double t = tan(half_fov)*near;
    double b = -t;
    double r = t * aspect;
    double l = -r;
    std::cout << "left: " << l << std::endl;
    std::cout << "right: " << r << std::endl;
    std::cout << "bottom: " << b << std::endl;
    std::cout << "top: " << t << std::endl;
    std::cout << "near: " << near << std::endl;
    std::cout << "far: " << far << std::endl;
}

void renderer::print_converted_parameters(double left, double right, double bottom, double top, double near, double far)
{
    if(right != -left || top != -bottom) {
        std::cout << "Viewing volume is not symmetric" << std::endl;
        return;
    }
    double fovy = rad2deg(2*atan(top/near));
    std::cout << "fovy: " << fovy << std::endl;
    std::cout << "aspect: " << static_cast<double>(m_screen.vpw) / m_screen.vph
              << std::endl;
    std::cout << "near: " << near << std::endl;
    std::cout << "far: " << far << std::endl;
}

void renderer::calculate_pure_projection_matrix(double near, double far)
{
    m_pure_projection = Eigen::Matrix4d::Zero();
    m_pure_projection(0,0) = near;
    m_pure_projection(1,1) = near;
    m_pure_projection(2,2) = far + near;
    m_pure_projection(2,3) = far*near;
    m_pure_projection(3,2) = -1;
}
