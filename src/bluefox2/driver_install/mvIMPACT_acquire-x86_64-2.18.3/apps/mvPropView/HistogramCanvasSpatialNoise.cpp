//-----------------------------------------------------------------------------
#include "HistogramCanvasSpatialNoise.h"

//=============================================================================
//================= Implementation HistogramCanvasSpatialNoise ================
//=============================================================================
//-----------------------------------------------------------------------------
HistogramCanvasSpatialNoise::HistogramCanvasSpatialNoise( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "Spatial Noise Histogram" */, bool boActive /* = true */ )
    : HistogramCanvas( parent, wxT( "SpatialNoiseHistogram" ), id, pos, size, style, name, boActive, true )
//-----------------------------------------------------------------------------
{
    for( int i = 0; i < PLANE_CNT; i++ )
    {
        m_Pens[i].description_ = wxString::Format( wxT( "C%d%s" ), i / 2, ( i % 2 ? wxT( "Ver" ) : wxT( "Hor" ) ) );
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvasSpatialNoise::RefreshData( const RequestData& data, int x /* = -1 */, int y /* = -1 */, int w /* = -1 */, int h /* = -1 */, bool boForceRefresh /* = false */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );

    bool boImageFormatChanged = false;
    const ImageBuffer* pIB = data.image_.getBuffer();
    const TBayerMosaicParity bayerParity = GetProcessBayerParity() ? data.bayerParity_ : bmpUndefined;
    const int channelCount = ( bayerParity != bmpUndefined ) ? 4 : pIB->iChannelCount;
    if( !MustUpdate( pIB, 2 * channelCount, x, y, w, h, boForceRefresh, 0, &boImageFormatChanged ) )
    {
        return;
    }

    if( boImageFormatChanged || ( m_bayerParity != bayerParity ) )
    {
        SetDefaultPens( pIB->pixelFormat );
        m_bayerParity = bayerParity;
    }

    PrepareHistogramBuffer( boImageFormatChanged, 2 * channelCount, pIB->pixelFormat );
    CalculateSpatialNoiseHistogram( data );
    CalculateMaxValue();
    UpdateAnalysisOutput( boImageFormatChanged );
}
