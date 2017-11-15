//-----------------------------------------------------------------------------
#include "HistogramCanvasPixel.h"

//=============================================================================
//================= Implementation HistogramCanvasPixel =======================
//=============================================================================
//-----------------------------------------------------------------------------
HistogramCanvasPixel::HistogramCanvasPixel( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "Pixel Histogram" */, bool boActive /* = true */ )
    : HistogramCanvas( parent, wxT( "PixelHistogram" ), id, pos, size, style, name, boActive ) {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
HistogramCanvasPixel::HistogramCanvasPixel( wxWindow* parent, const wxString& configName, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "Pixel Histogram" */, bool boActive /* = true */ )
    : HistogramCanvas( parent, configName, id, pos, size, style, name, boActive ) {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void HistogramCanvasPixel::RefreshData( const RequestData& data, int x /* = -1 */, int y /* = -1 */, int w /* = -1 */, int h /* = -1 */, bool boForceRefresh /* = false */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );

    bool boImageFormatChanged = false;
    const ImageBuffer* pIB = data.image_.getBuffer();
    const TBayerMosaicParity bayerParity = GetProcessBayerParity() ? data.bayerParity_ : bmpUndefined;
    const int channelCount = ( ( bayerParity != bmpUndefined ) ? 4 : pIB->iChannelCount );
    if( !MustUpdate( pIB, channelCount, x, y, w, h, boForceRefresh, 0, &boImageFormatChanged ) )
    {
        return;
    }

    if( boImageFormatChanged || !m_ppHistogramBuffer || !m_pHistogramAverage || !m_pHistogramMax || ( m_bayerParity != bayerParity ) )
    {
        SetupGDIData( pIB, channelCount, bayerParity );
        m_bayerParity = bayerParity;
    }

    PrepareHistogramBuffer( boImageFormatChanged, channelCount, pIB->pixelFormat );
    CalculatePixelHistogram( data );
    CalculateMaxValue();
    boImageFormatChanged |= CustomRefreshData( data, x, y, w, h );
    UpdateAnalysisOutput( boImageFormatChanged );
}
