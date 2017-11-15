#include "LineProfileCanvas.h"

//=============================================================================
//============== Implementation LineProfileCanvas =============================
//=============================================================================
//-----------------------------------------------------------------------------
LineProfileCanvas::LineProfileCanvas( wxWindow* parent, const wxString& configName, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                                      const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "line profile" */, bool boActive /* = false */ )
    : PlotCanvasImageAnalysis( parent, configName, id, pos, size, style, name, boActive, 0 ), m_ppData( 0 ), m_horDataCount( 0 ), m_verDataCount( 0 )
//-----------------------------------------------------------------------------
{
    m_plotFeatures.insert( pfPercentageWindow );
    m_plotFeatures.insert( pfProcessBayerParity );
    SetGridValueFormatString( wxT( "%d" ) );
}

//-----------------------------------------------------------------------------
LineProfileCanvas::~LineProfileCanvas()
//-----------------------------------------------------------------------------
{
    DeallocateProfileBuffer();
}

//-----------------------------------------------------------------------------
void LineProfileCanvas::DeallocateProfileBuffer( void )
//-----------------------------------------------------------------------------
{
    for( int channel = 0; channel < m_ChannelCount; channel++ )
    {
        if( m_ppData )
        {
            delete [] m_ppData[channel];
        }
    }
    delete [] m_ppData;
    m_ppData = 0;
}

//-----------------------------------------------------------------------------
wxString LineProfileCanvas::GetGridValue( int row, int col ) const
//-----------------------------------------------------------------------------
{
    if( col == 0 )
    {
        return ( row == 0 ) ? wxT( "Data Point" ) : wxString::Format( wxT( "%d" ), row - 1 );
    }
    else if( m_ppData && ( col > 0 ) && ( row >= 0 ) )
    {
        if( col - 1 < m_ChannelCount )
        {
            if( row == 0 )
            {
                return m_Pens[col - 1].description_ ;
            }
            else if( row <= GetDataCount() )
            {
                return wxString::Format( GetGridValueFormatString().c_str(), m_ppData[col - 1][row - 1] );
            }
        }
    }
    return wxEmptyString;
}

//-----------------------------------------------------------------------------
double LineProfileCanvas::GetScaleY( wxCoord h ) const
//-----------------------------------------------------------------------------
{
    int diagramHeight = h - ( 2 * GetBorderWidth() );
    return static_cast<double>( diagramHeight ) / ( static_cast<double>( m_valCount ) * static_cast<double>( GetPercentageToDraw() ) / 100. );
}

//-----------------------------------------------------------------------------
void LineProfileCanvas::OnPaintCustom( wxPaintDC& dc )
//-----------------------------------------------------------------------------
{
    wxCoord yOffset( 1 ), w( 0 ), h( 0 );
    dc.GetSize( &w, &h );
    const double scaleX = static_cast<double>( w - 2 * GetBorderWidth() ) / static_cast<double>( ( GetDataCount() == 0 ) ? 1 : GetDataCount() - 1 );
    const double scaleY = GetScaleY( h );
    DrawMarkerLines( dc, w, h, scaleX );
    unsigned int from, to;
    GetDrawRange( &from, &to );
    const int borderWidth = GetBorderWidth();
    const wxString YMarkerString( wxString::Format( wxT( "Draw Range(absolute): %d / %d, " ), from, to ) );
    wxCoord xOffset;
    dc.GetTextExtent( YMarkerString, &xOffset, 0 );
    xOffset += borderWidth / 2;
    dc.DrawText( YMarkerString, borderWidth / 2, yOffset );
    if( m_boUnsupportedPixelFormat )
    {
        dc.SetTextForeground( *wxRED );
        dc.DrawText( wxString( wxT( "Unsupported pixel format" ) ), xOffset, yOffset );
        dc.SetTextForeground( *wxBLACK );
    }
    else if( m_ppData )
    {
        for( int channel = 0; channel < m_ChannelCount; channel++ )
        {
            DrawProfileLine( dc, h, borderWidth + 1, scaleX, scaleY, from, to, m_ppData[channel], GetDataCount(), *m_Pens[channel].pColour_ );
            DrawInfoString( dc, wxString::Format( wxT( "%s%s: " ), ( channel != 0 ) ? wxT( ", " ) : wxT( "" ), m_Pens[channel].description_.c_str() ), xOffset, yOffset, *( m_Pens[channel].pColour_ ) );
        }
    }
}

//-----------------------------------------------------------------------------
void LineProfileCanvas::RefreshData( const RequestData& data, int x /* = -1 */, int y /* = -1 */, int w /* = -1 */, int h /* = -1 */, bool boForceRefresh /* = false */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );

    bool boAOIChanged = false;
    bool boImageFormatChanged = false;
    const ImageBuffer* pIB = data.image_.getBuffer();
    const TBayerMosaicParity bayerParity = GetProcessBayerParity() ? data.bayerParity_ : bmpUndefined;
    const int channelCount = ( ( bayerParity != bmpUndefined ) ? 4 : pIB->iChannelCount );
    if( !MustUpdate( pIB, channelCount, x, y, w, h, boForceRefresh, &boAOIChanged, &boImageFormatChanged ) )
    {
        return;
    }

    if( boImageFormatChanged || ( m_bayerParity != bayerParity ) )
    {
        SetupGDIData( pIB, channelCount, bayerParity );
    }

    if( boAOIChanged || !m_ppData || boImageFormatChanged )
    {
        // free old memory
        DeallocateProfileBuffer();
        // allocate new memory
        m_ppData = new int* [channelCount];
        for( int channel = 0; channel < channelCount; channel++ )
        {
            m_horDataCount = ( bayerParity != bmpUndefined ) ? ( ( m_AOIw + 1 ) / 2 ) : m_AOIw;
            m_verDataCount = ( bayerParity != bmpUndefined ) ? ( ( m_AOIh + 1 ) / 2 ) : m_AOIh;
            m_ppData[channel] = new int[GetDataCount()];
        }

        if( m_pNumericalDisplay )
        {
            const int columnCount = m_pNumericalDisplay->GetTable()->GetNumberCols();
            const int columnsNeeded = channelCount + 1;
            if( columnCount < columnsNeeded )
            {
                m_pNumericalDisplay->AppendCols( columnsNeeded - columnCount );
            }
            else if( columnCount > columnsNeeded )
            {
                m_pNumericalDisplay->DeleteCols( columnCount - 1, columnCount - columnsNeeded );
            }

            int rowCount = m_pNumericalDisplay->GetTable()->GetNumberRows();
            int rowsNeeded = 1 + GetDataCount();
            if( rowCount < rowsNeeded )
            {
                m_pNumericalDisplay->AppendRows( rowsNeeded - rowCount );
            }
            else if( rowCount > rowsNeeded )
            {
                m_pNumericalDisplay->DeleteRows( rowsNeeded - 1, rowCount - rowsNeeded );
            }
            m_pNumericalDisplay->SetCellValue( 0, 0, wxT( "Data Point" ) );
            for( int i = 0; i < channelCount; i++ )
            {
                m_pNumericalDisplay->SetCellBackgroundColour( 0, i + 1, *m_Pens[i].pColour_ );
                m_pNumericalDisplay->SetCellTextColour( 0, i + 1, *wxWHITE );
            }
        }
    }

    assert( ( channelCount <= PLANE_CNT ) && wxT( "A pixel format defines more channels then this tool can handle" ) );
    for( int channel = 0; channel < channelCount; channel++ )
    {
        memset( m_ppData[channel], 0, sizeof( int )*GetDataCount() );
    }

    m_bayerParity = bayerParity;
    m_ChannelCount = channelCount;

    switch( pIB->pixelFormat )
    {
    case ibpfMono10:
    case ibpfBGR101010Packed_V2:
    case ibpfRGB101010Packed:
    case ibpfYUV422_10Packed:
    case ibpfYUV422_UYVY_10Packed:
    case ibpfYUV444_UYV_10Packed:
    case ibpfYUV444_10Packed:
        m_valCount = VAL_COUNT_10_BIT;
        break;
    case ibpfRGB121212Packed:
    case ibpfMono12:
    case ibpfMono12Packed_V1:
    case ibpfMono12Packed_V2:
        m_valCount = VAL_COUNT_12_BIT;
        break;
    case ibpfRGB141414Packed:
    case ibpfMono14:
        m_valCount = VAL_COUNT_14_BIT;
        break;
    case ibpfRGB161616Packed:
    case ibpfMono16:
        m_valCount = VAL_COUNT_16_BIT;
        break;
    case ibpfRGBx888Packed:
    case ibpfBGR888Packed:
    case ibpfRGB888Packed:
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
    case ibpfYUV411_UYYVYY_Packed:
    case ibpfYUV422Packed:
    case ibpfYUV422_UYVYPacked:
    case ibpfYUV444_UYVPacked:
    case ibpfYUV444Packed:
    case ibpfYUV422Planar:
    case ibpfMono8:
    default:
        m_valCount = VAL_COUNT_8_BIT;
        break;
    }

    CalculateData( pIB, bayerParity );
    bool boFullUpdate = boAOIChanged || boImageFormatChanged;
    UpdateAnalysisOutput( boFullUpdate );
}
