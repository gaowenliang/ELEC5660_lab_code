#include "PlotCanvas.h"

//-----------------------------------------------------------------------------
bool DoTextsOverlap( int xPos1, wxCoord textWidth1, int xPos2, wxCoord textWidth2 )
//-----------------------------------------------------------------------------
{
    if( xPos1 == xPos2 )
    {
        return true;
    }
    if( ( xPos1 < xPos2 ) &&
        ( xPos1 + ( textWidth1 + textWidth2 ) / 2 ) >= xPos2 )
    {
        return true;
    }
    if( ( xPos1 > xPos2 ) &&
        ( xPos2 + ( textWidth1 + textWidth2 ) / 2 ) >= xPos1 )
    {
        return true;
    }

    return false;
}

//=============================================================================
//================= Implementation PlotCanvas =================================
//=============================================================================
BEGIN_EVENT_TABLE( PlotCanvas, DrawingCanvas )
    EVT_PAINT( PlotCanvas::OnPaint )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
PlotCanvas::PlotCanvas( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */,
                        const wxString& name /* = "DrawingCanvas" */, bool boActive /* = false */ ) :
    DrawingCanvas( parent, id, pos, size, style, name, boActive ), m_BorderWidth( 25 ), m_ImageCount( -1 ),
    m_UpdateFrequency( 3 ), m_UpdateFrequencyMin( 1 ), m_UpdateFrequencyMax( 100 )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
void PlotCanvas::DrawInfoString( wxPaintDC& dc, const wxString& info, wxCoord& xOffset, wxCoord yOffset, const wxColour& colour ) const
//-----------------------------------------------------------------------------
{
    static const wxCoord rectSize( 10 );

    dc.DrawText( info, xOffset, yOffset );
    wxCoord textWidth, textHeight;
    dc.GetTextExtent( info, &textWidth, &textHeight );
    wxCoord yOffsetRect( ( ( textHeight - rectSize ) / 2 ) + 1 );
    xOffset += textWidth;
    dc.SetPen( wxPen( *wxBLACK, 1, wxPENSTYLE_SOLID ) );
    dc.SetBrush( wxBrush( colour ) );
    dc.DrawRectangle( xOffset, yOffsetRect, rectSize, rectSize );
    dc.SetBrush( wxNullBrush );
    dc.SetPen( wxNullPen );
    xOffset += rectSize + 2;
}

//-----------------------------------------------------------------------------
void PlotCanvas::DrawMarkerLines( wxPaintDC& dc, const wxCoord w, const wxCoord h, const double scaleX ) const
//-----------------------------------------------------------------------------
{
    const int borderWidth = GetBorderWidth();

    // white rectangle for the background
    dc.SetPen( *wxBLACK );
    dc.SetBrush( *wxWHITE_BRUSH );
    dc.DrawRectangle( 0, 0, w, h );
    dc.SetBrush( wxNullBrush );
    // lower line
    dc.DrawLine( borderWidth, h - borderWidth, w - borderWidth, h - borderWidth );
    // line on left side
    dc.DrawLine( borderWidth, h - borderWidth, borderWidth, borderWidth );

    // markers for X-axis
    int markerStartHeight = h - borderWidth - 3;
    int markerEndHeight = h - borderWidth + 4;
    unsigned int from = 0, to = 0;
    const unsigned int XMarkerStepWidth = GetXMarkerParameters( from, to );
    wxCoord textWidth;
    int xPosPrev = 0;
    wxCoord textWidthPrev = 0;
    for( unsigned int i = to; i >= from; )
    {
        const int xPos = static_cast<int>( borderWidth + ( ( i - from ) * scaleX ) );
        dc.DrawLine( xPos, markerStartHeight, xPos, markerEndHeight );
        const wxString XMarkerString( wxString::Format( wxT( "%d" ), i ) );
        dc.GetTextExtent( XMarkerString, &textWidth, 0 );
        if( ( xPosPrev == 0 ) || ( DoTextsOverlap( xPos, textWidth, xPosPrev, textWidthPrev ) == false ) )
        {
            dc.DrawText( XMarkerString, static_cast<int>( xPos - ( textWidth / 2 ) ), h - borderWidth + 3 );
        }
        xPosPrev = xPos;
        textWidthPrev = textWidth;
        if( i == from )
        {
            break;
        }
        if( ( i > XMarkerStepWidth ) && ( ( i - XMarkerStepWidth ) > from ) )
        {
            i = ( ( i % XMarkerStepWidth ) == 0 ) ? i - XMarkerStepWidth : ( i / XMarkerStepWidth ) * XMarkerStepWidth;
        }
        else
        {
            i = from;
        }
    }

    // markers for y-axis
    int markerStart = borderWidth - 3;
    int markerEnd = borderWidth + 4;
    dc.DrawLine( markerStart, borderWidth, markerEnd, borderWidth );
}

//-----------------------------------------------------------------------------
void PlotCanvas::DrawProfileLine( wxPaintDC& dc, int h, int startOffset, double scaleX, double scaleY, unsigned int from, unsigned int to, int* pData, int elementCount, const wxColour& colour ) const
//-----------------------------------------------------------------------------
{
    dc.SetPen( colour );
    int lowerStart = h - GetBorderWidth();
    const int maxY = static_cast<int>( to - from );
    for( int i = 0; i < elementCount - 1; i++ )
    {
        dc.DrawLine( static_cast<int>( startOffset + ( CalculateXStart( from, to, i, pData[i], i + 1, pData[i + 1], i, pData[i] ) * scaleX ) ),
                     static_cast<int>( lowerStart - ( saveAssign( pData[i] - static_cast<int>( from ), 0, maxY ) * scaleY ) ),
                     static_cast<int>( startOffset + ( CalculateXStart( from, to, i, pData[i], i + 1, pData[i + 1], i + 1, pData[i + 1] ) * scaleX ) ),
                     static_cast<int>( lowerStart - ( saveAssign( pData[i + 1] - static_cast<int>( from ), 0, maxY ) * scaleY ) ) );
    }
    dc.SetPen( wxNullPen );
}

//-----------------------------------------------------------------------------
unsigned int PlotCanvas::GetXMarkerStepWidth( const unsigned int from, const unsigned int to )
//-----------------------------------------------------------------------------
{
    if( ( to - from ) < 16 )
    {
        return 1;
    }

    // X marker width for value ranges
    // 5 bit and smaller: 4
    // 6 bit: 8
    // 7 bit: 16
    // 8 bit: 32
    // etc.
    unsigned int XMarkerStepWidth = 4;
    unsigned int i = to - from;
    while( i > 32 )
    {
        i /= 2;
        XMarkerStepWidth *= 2;
    }
    return XMarkerStepWidth;
}

//-----------------------------------------------------------------------------
void PlotCanvas::OnPaint( wxPaintEvent& )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    wxPaintDC dc( this );

    if( !IsActive() )
    {
        return;
    }

    OnPaintCustom( dc );

    dc.SetBrush( wxNullBrush );
    dc.SetPen( wxNullPen );
}

//-----------------------------------------------------------------------------
void PlotCanvas::SetBorderWidth( int borderWidth )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_BorderWidth = borderWidth;
}

//-----------------------------------------------------------------------------
bool PlotCanvas::SetUpdateFrequency( int frequency )
//-----------------------------------------------------------------------------
{
    if( ( frequency >= m_UpdateFrequencyMin ) && ( frequency <= m_UpdateFrequencyMax ) )
    {
        m_UpdateFrequency = frequency;
        OnUpdateFrequencyChanged();
        return true;
    }
    return false;
}
