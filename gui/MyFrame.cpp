#include "MyFrame.hpp"
#include "MyCanvas.hpp"

BEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_MENU(wxID_EXIT, MyFrame::OnQuit)
    EVT_MENU(wxID_RENDER_MODE_2D, MyFrame::OnChangeRenderMode)
    EVT_MENU(wxID_RENDER_MODE_3D, MyFrame::OnChangeRenderMode)
    EVT_MENU(wxID_RENDER_MODE_BOTH, MyFrame::OnChangeRenderMode)
    EVT_MENU(wxID_ESTIMATE_3D_CIRCLES_METHOD_1, MyFrame::OnEstimate3DCircles)
    EVT_MENU(wxID_ESTIMATE_3D_CIRCLES_METHOD_2, MyFrame::OnEstimate3DCircles)
    EVT_MENU(wxID_ESTIMATE_3D_CIRCLES_METHOD_3, MyFrame::OnEstimate3DCircles)
    EVT_MENU(wxID_ESTIMATE_3D_CIRCLES_METHOD_4, MyFrame::OnEstimate3DCircles)
    EVT_MENU(wxID_CLEAR_ESTIMATED_CIRCLES, MyFrame::OnClearEstimatedCircles)
END_EVENT_TABLE()

MyFrame::MyFrame(const wxString& title) :
    wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(640, 480)) {
    m_canvas = new MyCanvas(this);
    CreateStatusBar(3);
    int widths[3] = { -1, -1, -4 };
    SetStatusWidths(3, widths);

    wxMenuBar* menubar = new wxMenuBar;
    wxMenu* file = new wxMenu;
    file->Append(wxID_EXIT, wxT("&Quit"));
    menubar->Append(file, wxT("&File"));

    wxMenu* edit = new wxMenu;
    edit->Append(wxID_CLEAR_ESTIMATED_CIRCLES, wxT("Clear Estimated Circles"));
    menubar->Append(edit, wxT("&Edit"));

    wxMenu* rmode = new wxMenu;
    rmode->AppendRadioItem(wxID_RENDER_MODE_2D, wxT("2D"));
    rmode->AppendRadioItem(wxID_RENDER_MODE_3D, wxT("3D"));
    rmode->AppendRadioItem(wxID_RENDER_MODE_BOTH, wxT("Both"));
    wxMenu* render = new wxMenu;
    render->AppendSubMenu(rmode, wxT("Mode"));
    menubar->Append(render, wxT("&Render"));

    wxMenu* estimate = new wxMenu;
    estimate->Append(wxID_ESTIMATE_3D_CIRCLES_METHOD_1, wxT("3D Circles (Philip)"));
    estimate->Append(wxID_ESTIMATE_3D_CIRCLES_METHOD_2, wxT("3D Circles (Ferri)"));
    // estimate->Append(wxID_ESTIMATE_3D_CIRCLES_METHOD_3, wxT("3D Circles (Bredif)"));
    estimate->Append(wxID_ESTIMATE_3D_CIRCLES_METHOD_4, wxT("3D Circles (Safaee-Rad)"));
    menubar->Append(estimate, wxT("Estimate"));

    SetMenuBar(menubar);

    menubar->FindItem(wxID_RENDER_MODE_BOTH)->Check(true);
    m_canvas->SetRenderingMode(render_mode::both);

    SetClientSize(640, 480);
	this->Centre();
}

void MyFrame::OnChangeRenderMode(wxCommandEvent& event) {
    if(event.GetId() == wxID_RENDER_MODE_2D) {
        m_canvas->SetRenderingMode(render_mode::two_dim);
    }
    else if(event.GetId() == wxID_RENDER_MODE_3D) {
        m_canvas->SetRenderingMode(render_mode::three_dim);
    }
    else if(event.GetId() == wxID_RENDER_MODE_BOTH) {
        m_canvas->SetRenderingMode(render_mode::both);
    }
}

void MyFrame::OnQuit(wxCommandEvent& event) {
    Close();
}

void MyFrame::OnEstimate3DCircles(wxCommandEvent& event) {

    int id = event.GetId();
    if(id == wxID_ESTIMATE_3D_CIRCLES_METHOD_1) {
        m_canvas->Estimate_3DCircles(estimation_algorithm::method_1);
    }
    else if(id == wxID_ESTIMATE_3D_CIRCLES_METHOD_2) {
        m_canvas->Estimate_3DCircles(estimation_algorithm::method_2);
    }
    else if(id == wxID_ESTIMATE_3D_CIRCLES_METHOD_3) {
        m_canvas->Estimate_3DCircles(estimation_algorithm::method_3);
    }
    else if(id == wxID_ESTIMATE_3D_CIRCLES_METHOD_4) {
        m_canvas->Estimate_3DCircles(estimation_algorithm::method_4);
    }
}

void MyFrame::OnClearEstimatedCircles(wxCommandEvent& event) {
    m_canvas->ClearEstimatedCircles();
}
