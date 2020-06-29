#include "MyCanvas.hpp"
#include "MyFrame.hpp"
#include "../geometry/Segment3D.hpp"
#include "../geometry/Circle3D.hpp"
#include "../data/teapot.hpp"
#include "../algorithm/algorithm.hpp"
#include "../utility/utility.hpp"
#include <sstream>

BEGIN_EVENT_TABLE(MyCanvas, wxScrolledWindow)
    EVT_PAINT(MyCanvas::OnPaint)
    EVT_LEFT_DOWN(MyCanvas::OnMouseLeftClick)
    EVT_RIGHT_DOWN(MyCanvas::OnMouseRightClick)
    EVT_MOTION(MyCanvas::OnMouseMove)
    EVT_KEY_DOWN(MyCanvas::OnKeyDown)
    EVT_SIZE(MyCanvas::OnResize)
END_EVENT_TABLE()

MyCanvas::MyCanvas(MyFrame* parent) :
    wxScrolledWindow(parent,
                     wxID_ANY,
                     wxDefaultPosition,
                     wxDefaultSize,
                     wxHSCROLL | wxVSCROLL | wxFULL_REPAINT_ON_RESIZE),
    m_parent(parent) {
    wxSize size = m_parent->GetClientSize();
    m_renderer.set_screen_width_and_height(size.GetWidth(), size.GetHeight());
    m_render_mode = render_mode::two_dim;
    m_renderer.gluPerspective(45.0, 1.0, 100.0);
    SetBackgroundColour(wxColour(*wxWHITE));

    Eigen::Vector3d normal(-1, 2, -3);
    normal.normalize();
    Circle3D circ(Eigen::Vector3d(1, 2, -8), // center
                  normal, // normal
                  1.0);                      // radius
    std::cout << "Ground truth circle normal:\n " << normal << std::endl;
    m_groud_truth_3DCircles.push_back(circ);
    std::cout << "Ground truth circle center:\n" << circ.center << std::endl;
}

void MyCanvas::Estimate_3DCircles(estimation_algorithm method) {

    if(method == estimation_algorithm::method_1) {
        estimate_3d_circles_1();
    }
    else if(method == estimation_algorithm::method_2) {
        estimate_3d_circles_2();
    }
    else if(method == estimation_algorithm::method_3) {
        estimate_3d_circles_3();
    }
    else if(method == estimation_algorithm::method_4) {
        estimate_3d_circles_4();
    }
}

void MyCanvas::estimate_3d_circles_1() {

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    m_renderer.get_projection_matrix(mat);
    Circle3D circles[2];
    circles[0].radius = 1.0;
    circles[1].radius = 1.0;

    int h{0}, w{0};
    this->GetSize(&w, &h);
    for(auto it = m_ellipses.begin(); it != m_ellipses.end(); ++it) {
        it->calculate_algebraic_equation_in_normalized_device_coordinates(w, h);
        // it->calculate_algebraic_equation_in_projected_coordinates(w, h, 1, deg2rad(45.0/2.0));
        estimate_3D_circles_under_perspective_transformation_method_1(mat, *it, circles);
        m_3DCircles.push_back(circles[0]);
        m_3DCircles.push_back(circles[1]);
    }
}

void MyCanvas::estimate_3d_circles_2() {

    Circle3D circle[4];
    circle[0].radius = 1.0;
    circle[1].radius = 1.0;
    circle[2].radius = 1.0;
    circle[3].radius = 1.0;

    int h{0}, w{0};
    this->GetSize(&w, &h);
    for(auto it = m_ellipses.begin(); it != m_ellipses.end(); ++it) {
        it->calculate_algebraic_equation_in_projected_coordinates(w, h, 1, deg2rad(45.0/2.0));
        estimate_3D_circles_under_perspective_transformation_method_2(*it, circle, -1);
        if(circle[0].center(2) <= -1) {
            m_3DCircles.push_back(circle[0]);
            std::cout << "Circle-1: Valid" << std::endl;
        }
        if(circle[1].center(2) <= -1) {
            m_3DCircles.push_back(circle[1]);
            std::cout << "Circle-2: Valid" << std::endl;
        }
        if(circle[2].center(2) <= -1) {
            m_3DCircles.push_back(circle[2]);
            std::cout << "Circle-3: Valid" << std::endl;
        }
        if(circle[3].center(2) <= -1) {
            m_3DCircles.push_back(circle[3]);
            std::cout << "Circle-4: Valid" << std::endl;
        }
    }
}

void MyCanvas::estimate_3d_circles_3() {
    Circle3D circle[8];
    circle[0].radius = 1.0;
    circle[1].radius = 1.0;
    circle[2].radius = 1.0;
    circle[3].radius = 1.0;
    circle[4].radius = 1.0;
    circle[5].radius = 1.0;
    circle[6].radius = 1.0;
    circle[7].radius = 1.0;

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    m_renderer.get_pure_projection_matrix(mat);

    Eigen::Matrix3d mat2;
    mat2.row(0) = mat.block(0,0,1,3);
    mat2.row(1) = mat.block(1,0,1,3);
    mat2.row(2) = mat.block(3,0,1,3);

    int h{0}, w{0};
    this->GetSize(&w, &h);

    for(auto it = m_ellipses.begin(); it != m_ellipses.end(); ++it) {
        it->calculate_algebraic_equation_in_projected_coordinates(w, h, 1, deg2rad(45.0/2.0));
        estimate_3D_circles_under_perspective_transformation_method_3(mat2, *it, circle);
        m_3DCircles.push_back(circle[0]);
        m_3DCircles.push_back(circle[1]);
        m_3DCircles.push_back(circle[2]);
        m_3DCircles.push_back(circle[3]);
        m_3DCircles.push_back(circle[4]);
        m_3DCircles.push_back(circle[5]);
        m_3DCircles.push_back(circle[6]);
        m_3DCircles.push_back(circle[7]);
    }
}

void MyCanvas::estimate_3d_circles_4() {

    Circle3D circle[4];
    circle[0].radius = 1.0;
    circle[1].radius = 1.0;
    circle[2].radius = 1.0;
    circle[3].radius = 1.0;

    int h{0}, w{0};
    this->GetSize(&w, &h);

    for(auto it = m_ellipses.begin(); it != m_ellipses.end(); ++it) {

        it->calculate_algebraic_equation_in_projected_coordinates(w, h, 1, deg2rad(45.0/2.0));     
        int count = estimate_3D_circles_under_perspective_transformation_method_4(*it, circle, -1);
        if(count == 4) {
            if(circle[0].center(2) <= -1) {
                m_3DCircles.push_back(circle[0]);
                std::cout << "Circle-1: Valid" << std::endl;
            }
            if(circle[1].center(2) <= -1) {
                m_3DCircles.push_back(circle[1]);
                std::cout << "Circle-2: Valid" << std::endl;
            }
            if(circle[2].center(2) <= -1) {
                m_3DCircles.push_back(circle[2]);
                std::cout << "Circle-3: Valid" << std::endl;
            }
            if(circle[3].center(2) <= -1) {
                m_3DCircles.push_back(circle[3]);
                std::cout << "Circle-4: Valid" << std::endl;
            }
        }
        else if(count == 1) {
            if(circle[0].center(2) <= -1) {
                m_3DCircles.push_back(circle[0]);
                std::cout << "Circle-1: Valid" << std::endl;
            }
        }
    }
}

void MyCanvas::SetRenderingMode(render_mode mode) {
    m_render_mode = mode;
    Refresh();
}

void MyCanvas::OnPaint(wxPaintEvent& event) {
    wxPaintDC dc(this);
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    SetupCoordinateFrame(dc);
    if(m_render_mode == render_mode::two_dim)        { Render2D(dc); }
    else if(m_render_mode == render_mode::three_dim) { Render3D(dc); }
    else if(m_render_mode == render_mode::both)      { Render2D(dc);
                                                       Render3D(dc); }
}

void MyCanvas::Render2D(wxPaintDC& dc) {
    dc.SetPen(*wxBLUE_PEN);
    if(!m_ellipses.empty()) { RenderEllipses(dc); }
    if(!m_points.empty()) { RenderEllipse(dc); }
}

void MyCanvas::Render3D(wxPaintDC& dc) {
    dc.SetPen(wxPen(*wxRED, 3));
    std::vector<Eigen::Vector3d> data;

    for(auto it = m_3DCircles.begin(); it != m_3DCircles.end(); ++it) {
        data.clear();
        it->generate_data(data, 8);
        m_renderer.render(data, dc);
    }
    dc.SetPen(wxPen(*wxCYAN, 3));
    for(auto it = m_groud_truth_3DCircles.begin(); it != m_groud_truth_3DCircles.end(); ++it) {
        data.clear();
        it->generate_data(data, 100);
        m_renderer.render(data, dc);
    }
}

void MyCanvas::SetupCoordinateFrame(wxPaintDC& dc) {
    wxSize size = m_parent->GetClientSize();
    dc.SetAxisOrientation(true, true);
    dc.SetDeviceOrigin(size.GetWidth()/2, size.GetHeight()/2);
    dc.DrawLine(-size.GetWidth()/2,0,size.GetWidth()/2,0);
    dc.DrawLine(0,-size.GetHeight()/2,0,size.GetHeight()/2);
}

void MyCanvas::RenderEllipse(wxPaintDC& dc) {
    if(m_points.size() == 1) {
        dc.DrawLine(m_points[0], m_mouse_pos);
        Point2D<double> p0(m_points[0].x, m_points[0].y);
        Point2D<double> center((m_points[0].x + m_mouse_pos.x)/2.0,
                              (m_points[0].y + m_mouse_pos.y)/2.0);
        dc.DrawCircle(static_cast<int>(center.x),
                      static_cast<int>(center.y),
                      dist(p0, center));
    }
    else if(m_points.size() == 2) {
        Point2D<double> p0(m_points[0].x, m_points[0].y);
        Point2D<double> p1(m_points[1].x, m_points[1].y);
        Point2D<double> center((p0.x + p1.x)/2, (p0.y + p1.y)/2);
        double smajor_axis = dist(p0, center);
        double sminor_axis = 0.0;
        Vector2D<double> vec_mj = p1 - p0;
        Vector2D<double> vec_mn(-vec_mj.y, vec_mj.x);
        Vector2D<double> minor_unit = vec_mn.normalized();
        Point2D<double> p2 = center - smajor_axis*minor_unit;
        Point2D<double> p3 = center + smajor_axis*minor_unit;
        Point2D<double> mouse_pos(m_mouse_pos.x, m_mouse_pos.y);
        Vector2D<double> mouse_vec = mouse_pos - p2;
        double ratio = vec_mn.dot(mouse_vec) / vec_mn.dot(vec_mn);
        if(ratio >= 0.0 && ratio <= 1.0) {
            Vector2D<double> proj_vec = (2*ratio*smajor_axis) * minor_unit;
            Point2D<double> proj_point = p2 + proj_vec;
            sminor_axis = dist(center, proj_point);
            // draw a mini circle on the projection point
            dc.DrawCircle(static_cast<int>(proj_point.x), static_cast<int>(proj_point.y), 2);
            m_ellipse.center = Point2D<double>(center.x, center.y);
            m_ellipse.semi_major_axis = smajor_axis;
            m_ellipse.semi_minor_axis = sminor_axis;
            if(vec_mj.y > 0) {
                m_ellipse.rot_angle = std::acos(vec_mj.x/vec_mj.norm());
            }
            else {
                m_ellipse.rot_angle = -std::acos(vec_mj.x/vec_mj.norm());
            }
            double data[80];
            m_ellipse.generate_points_on_the_ellipse(40, data);
            for(int i = 0; i < 78; i += 2) {
                dc.DrawLine(static_cast<int>(data[i]),
                            static_cast<int>(data[i+1]),
                            static_cast<int>(data[i+2]),
                            static_cast<int>(data[i+3]));
            }
            dc.DrawLine(static_cast<int>(data[0]),
                        static_cast<int>(data[1]),
                        static_cast<int>(data[78]),
                        static_cast<int>(data[79]));
            m_ellipse.points[2] = proj_point;
            DisplayAlgebraicEquation();
        }

        // Draw the end points
        dc.DrawCircle(static_cast<int>(p0.x), static_cast<int>(p0.y), 2);
        dc.DrawCircle(static_cast<int>(p1.x), static_cast<int>(p1.y), 2);
        dc.DrawCircle(static_cast<int>(center.x), static_cast<int>(center.y), 2);
        dc.DrawCircle(static_cast<int>(p2.x), static_cast<int>(p2.y), 2);
        dc.DrawCircle(static_cast<int>(p3.x), static_cast<int>(p3.y), 2);
        // draw the major axis and minor axis guide line
        dc.DrawLine(static_cast<int>(p0.x), static_cast<int>(p0.y),
                    static_cast<int>(p1.x), static_cast<int>(p1.y));
        dc.SetPen(*wxBLACK_DASHED_PEN);
        dc.DrawLine(static_cast<int>(p2.x), static_cast<int>(p2.y),
                    static_cast<int>(p3.x), static_cast<int>(p3.y));
    }
    else {
        m_points.clear();
    }
}

void MyCanvas::RenderEllipses(wxPaintDC& dc) {
    const int num = 80;
    const int size = num*2;
    double data[size];
    for(auto it = m_ellipses.begin(); it != m_ellipses.end(); ++it){
        it->generate_points_on_the_ellipse(num, data);
        for(int i = 0; i < size-2; i+= 2) {
            dc.DrawLine(static_cast<int>(data[i]),
                        static_cast<int>(data[i+1]),
                        static_cast<int>(data[i+2]),
                        static_cast<int>(data[i+3]));
        }
        dc.DrawLine(static_cast<int>(data[0]),
                    static_cast<int>(data[1]),
                    static_cast<int>(data[size-2]),
                    static_cast<int>(data[size-1]));
    }
}

void MyCanvas::DisplayAlgebraicEquation() {
    m_ellipse.calculate_algebraic_equation_in_wxWidget_coordinates();
    wxString str;
    str << "(" << m_ellipse.coeff[0] << "xx) + "
        << "(" << m_ellipse.coeff[1] << "xy) + "
        << "(" << m_ellipse.coeff[2] << "yy) + "
        << "(" << m_ellipse.coeff[3] << "x) + "
        << "(" << m_ellipse.coeff[4] << "y) + "
        << "(" << m_ellipse.coeff[5] << ")";
    m_parent->SetStatusText(str, 2);
    wxString str2;
    str2 << rad2deg(m_ellipse.rot_angle);
    m_parent->SetStatusText(str2, 1);
}

void MyCanvas::OnMouseLeftClick(wxMouseEvent& event) {
    m_points.push_back(DeviceToLogical(event.GetPosition()));
    Refresh();
}

void MyCanvas::OnMouseRightClick(wxMouseEvent& event) {
    if(m_points.size() == 2) {
        m_ellipse.points[0] = Point2D<double>(m_points[0].x, m_points[0].y);
        m_ellipse.points[1] = Point2D<double>(m_points[1].x, m_points[1].y);
        m_ellipses.push_back(m_ellipse);
    }
    m_points.clear();
    Refresh();
}

void MyCanvas::OnMouseMove(wxMouseEvent& event) {
    m_mouse_pos = DeviceToLogical(event.GetPosition());
    wxString str;
    str << "(" << m_mouse_pos.x << ", " << m_mouse_pos.y << ")";
    m_parent->SetStatusText(str, 0);
    Refresh();
}

void MyCanvas::OnKeyDown(wxKeyEvent &event) {
    if(event.GetKeyCode() == WXK_ESCAPE) {
        m_points.clear();
        m_ellipses.clear();
        m_3DCircles.clear();
        Refresh();
    }
}

void MyCanvas::OnResize(wxSizeEvent& event) {
    m_renderer.set_screen_width_and_height(event.GetSize().GetWidth(),
                                           event.GetSize().GetHeight());
}

wxPoint MyCanvas::DeviceToLogical(const wxPoint& pt) {
    wxSize size = m_parent->GetClientSize();
    return wxPoint(pt.x - (size.GetWidth()-1)/2, (size.GetHeight()-1)/2 - pt.y);
}

wxPoint MyCanvas::LogicalToDevice(const wxPoint& pt) {
   wxSize size = m_parent->GetClientSize();
   return wxPoint(pt.x + (size.GetWidth()-1)/2, (size.GetHeight()-1)/2 - pt.y);
}
