//-----------------------------------------------------------------------------
#ifndef HistogramCanvasSpatialNoiseH
#define HistogramCanvasSpatialNoiseH HistogramCanvasSpatialNoiseH
//-----------------------------------------------------------------------------
#include "HistogramCanvas.h"

//-----------------------------------------------------------------------------
class HistogramCanvasSpatialNoise : public HistogramCanvas
//-----------------------------------------------------------------------------
{
public:
    explicit            HistogramCanvasSpatialNoise() {}
    explicit            HistogramCanvasSpatialNoise( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "Spatial Noise Histogram" ), bool boActive = false );
    virtual void        RefreshData( const RequestData& data, int x = -1, int y = -1, int w = -1, int h = -1, bool boForceRefresh = false );
};

#endif // HistogramCanvasSpatialNoiseH
