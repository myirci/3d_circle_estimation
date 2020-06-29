#include <wx/wx.h>

#define wxID_RENDER_MODE_2D                     wxID_HIGHEST + 1
#define wxID_RENDER_MODE_3D                     wxID_HIGHEST + 2
#define wxID_RENDER_MODE_BOTH                   wxID_HIGHEST + 3
#define wxID_CLEAR_ESTIMATED_CIRCLES            wxID_HIGHEST + 4
#define wxID_ESTIMATE_3D_CIRCLES_METHOD_1       wxID_HIGHEST + 5
#define wxID_ESTIMATE_3D_CIRCLES_METHOD_2       wxID_HIGHEST + 6
#define wxID_ESTIMATE_3D_CIRCLES_METHOD_3       wxID_HIGHEST + 7
#define wxID_ESTIMATE_3D_CIRCLES_METHOD_4       wxID_HIGHEST + 8

class MyCanvas;

class MyFrame : public wxFrame {
public:
    MyFrame(const wxString& title);
private:
    MyCanvas* m_canvas;
    void OnQuit(wxCommandEvent& event);
    void OnChangeRenderMode(wxCommandEvent& event);
    void OnEstimate3DCircles(wxCommandEvent& event);
    void OnClearEstimatedCircles(wxCommandEvent& event);
    DECLARE_EVENT_TABLE()
};		
