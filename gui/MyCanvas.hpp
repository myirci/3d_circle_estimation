#ifndef MYCANVAS_HPP
#define MYCANVAS_HPP

#include <wx/scrolwin.h>
#include <vector>
#include "../geometry/Ellipse2D.hpp"
#include "../geometry/Circle3D.hpp"
#include "../render/Renderer.hpp"

class MyFrame;
class wxPaintDC;
enum class render_mode : unsigned char {two_dim, three_dim, both };
enum class estimation_algorithm : unsigned char {
    method_1,
    method_2,
    method_3,
    method_4
};
class MyCanvas : public wxScrolledWindow {
public:
    MyCanvas(MyFrame* parent);
    void SetRenderingMode(render_mode mode);
    void Estimate_3DCircles(estimation_algorithm method);
    void ClearEstimatedCircles() { m_3DCircles.clear(); }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    MyFrame* m_parent;
    Ellipse2D m_ellipse;
    wxPoint m_mouse_pos;
    std::vector<wxPoint> m_points;
    std::vector<Ellipse2D> m_ellipses;
    std::vector<Circle3D> m_3DCircles;
    std::vector<Circle3D> m_groud_truth_3DCircles;
    renderer m_renderer;
    render_mode m_render_mode;
    void OnPaint(wxPaintEvent& event);
    void OnMouseLeftClick(wxMouseEvent& event);
    void OnMouseRightClick(wxMouseEvent& event);
    void OnMouseMove(wxMouseEvent& event);
    void OnKeyDown(wxKeyEvent& event);
    void OnResize(wxSizeEvent& event);
    void Render2D(wxPaintDC& dc);
    void Render3D(wxPaintDC& dc);
    void estimate_3d_circles_1();
    void estimate_3d_circles_2();
    void estimate_3d_circles_3();
    void estimate_3d_circles_4();
    inline void SetupCoordinateFrame(wxPaintDC& dc);
    inline void RenderEllipses(wxPaintDC& dc);
    inline void RenderEllipse(wxPaintDC& dc);
    inline void DisplayAlgebraicEquation();
    inline wxPoint DeviceToLogical(const wxPoint& pt);
    inline wxPoint LogicalToDevice(const wxPoint& pt);
    DECLARE_EVENT_TABLE()
};

#endif // MYCANVAS_HPP
