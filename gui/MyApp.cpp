#include "MyApp.hpp"
#include "MyFrame.hpp"

IMPLEMENT_APP(MyApp)

bool MyApp::OnInit() {
    MyFrame *frame = new MyFrame(wxT("Basic Drawing"));
    frame ->Show(true);
	return true;
}
