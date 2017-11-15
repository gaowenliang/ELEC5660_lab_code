//-----------------------------------------------------------------------------
#ifndef HistogramCanvasPixelH
#define HistogramCanvasPixelH HistogramCanvasPixelH
//-----------------------------------------------------------------------------
#include "HistogramCanvas.h"

//-----------------------------------------------------------------------------
class HistogramCanvasPixel : public HistogramCanvas
//-----------------------------------------------------------------------------
{
    virtual bool        CustomRefreshData( const RequestData& /*data*/, int /*x*/ = -1, int /*y*/ = -1, int /*w*/ = -1, int /*h*/ = -1 )
    {
        return false;
    }
public:
    explicit            HistogramCanvasPixel() {}
    explicit            HistogramCanvasPixel( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "Pixel Histogram" ), bool boActive = false );
    explicit            HistogramCanvasPixel( wxWindow* parent, const wxString& configName, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "Pixel Histogram" ), bool boActive = false );
    virtual void        RefreshData( const RequestData& data, int x = -1, int y = -1, int w = -1, int h = -1, bool boForceRefresh = false );
};

#endif // HistogramCanvasPixelH
