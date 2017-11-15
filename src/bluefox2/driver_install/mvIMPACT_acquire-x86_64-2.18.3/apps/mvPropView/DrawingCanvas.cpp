#include "DrawingCanvas.h"

//=============================================================================
//================= Implementation DrawingCanvas ==============================
//=============================================================================
BEGIN_EVENT_TABLE( DrawingCanvas, wxScrolledWindow )
    EVT_SIZE( DrawingCanvas::OnSize )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
DrawingCanvas::DrawingCanvas( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                              const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "DrawingCanvas" */, bool boActive /* = false */ )
    : wxScrolledWindow( parent, id, pos, size, style, name ), m_boActive( boActive )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
void DrawingCanvas::HandleSizeEvent( wxSizeEvent& e )
//-----------------------------------------------------------------------------
{
    Refresh( true );
    e.Skip();
}

//-----------------------------------------------------------------------------
void DrawingCanvas::OnSize( wxSizeEvent& e )
//-----------------------------------------------------------------------------
{
    HandleSizeEvent( e );
}

//-----------------------------------------------------------------------------
void DrawingCanvas::SetActive( bool boActive )
//-----------------------------------------------------------------------------
{
    m_boActive = boActive;
    Refresh( !boActive );
}
