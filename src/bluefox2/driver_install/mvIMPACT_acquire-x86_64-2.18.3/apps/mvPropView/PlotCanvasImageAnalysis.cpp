#include <apps/Common/wxAbstraction.h>
#include "ImageCanvas.h"
#include "PlotCanvasImageAnalysis.h"

using namespace mvIMPACT::acquire;

//=============================================================================
//================= Implementation PlotCanvasImageAnalysis ====================
//=============================================================================
const wxColour PlotCanvasImageAnalysis::m_ADarkerKindOfGreen = wxColour( 0, 150, 0 );
const wxColour PlotCanvasImageAnalysis::m_CyanLight = wxColour( 0, 200, 200 );
const wxColour PlotCanvasImageAnalysis::m_Magenta = wxColour( 255, 0, 255 );
const wxColour PlotCanvasImageAnalysis::m_MagentaLight = wxColour( 200, 0, 200 );

//-----------------------------------------------------------------------------
PlotCanvasImageAnalysis::PlotCanvasImageAnalysis( wxWindow* parent, const wxString& configName, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */, const wxSize& size /* = wxDefaultSize */,
        long style /* = wxBORDER_NONE */, const wxString& name /* = "DrawingCanvas" */, bool boActive /* = false */, int channelCount /* = 0 */ ) :
    PlotCanvas( parent, id, pos, size, style, name, boActive ), m_AOIColour( *wxBLUE ),
    m_pImageCanvas( 0 ), m_ConfigName( configName ), m_boAOIFullMode( false ), m_boProcessBayerParity( false ), m_DisplayMethod( dmGraphical ),
    m_valCount( VAL_COUNT_8_BIT ), m_AOIx( 0 ), m_AOIy( 0 ), m_AOIw( 640 ), m_AOIh( 480 ), m_pAOI( 0 ), m_HistoryDepth( 20 ), m_GridStepsX( 1 ), m_GridStepsY( 1 ),
    m_ChannelCount( channelCount ), m_pixelFormat( ibpfRaw ), m_bayerParity( bmpUndefined ), m_boUnsupportedPixelFormat( false ), m_DrawStart_percent( 0 ), m_DrawWindow_percent( 100 ),
    m_pNumericalDisplay( 0 )
//-----------------------------------------------------------------------------
{
    SetDefaultPens( ibpfMono8 );
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::CreateGrid( wxWindow* pParent, wxWindowID gridID )
//-----------------------------------------------------------------------------
{
    m_pNumericalDisplay = new DataGridWithClipboardFeature( pParent, gridID );
    m_pNumericalDisplay->SetDataProvider( this );
    m_pNumericalDisplay->EnableEditing( false );
    CreateGridCustom();
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::CreateGridCustom( void )
//-----------------------------------------------------------------------------
{
    m_pNumericalDisplay->SetTable( new DataGridTable( this, GetRowCountNeededForNumericalDisplay(), m_ChannelCount ), true );
}

//-----------------------------------------------------------------------------
int PlotCanvasImageAnalysis::GetRowCountNeededForNumericalDisplay( void ) const
//-----------------------------------------------------------------------------
{
    return m_valCount + 1;
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetActive( bool boActive )
//-----------------------------------------------------------------------------
{
    if( boActive == IsActive() )
    {
        return;
    }

    m_boActive = boActive;
    if( m_pImageCanvas )
    {
        if( boActive )
        {
            m_pAOI = m_pImageCanvas->RegisterAOI( wxRect( m_AOIx, m_AOIy, m_AOIw, m_AOIh ), m_AOIColour, this );
        }
        else
        {
            m_pImageCanvas->RemoveAOI( this );
            m_pAOI = 0;
        }
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetDefaultPens( mvIMPACT::acquire::TImageBufferPixelFormat pixelFormat )
//-----------------------------------------------------------------------------
{
    switch( pixelFormat )
    {
    case ibpfYUV411_UYYVYY_Packed:
    case ibpfYUV422_UYVYPacked:
    case ibpfYUV422_UYVY_10Packed:
    case ibpfYUV444_UYVPacked:
    case ibpfYUV444_UYV_10Packed:
        m_Pens[pChannel0Hor].pColour_ = wxCYAN;
        m_Pens[pChannel0Ver].pColour_ = wxBLACK;
        m_Pens[pChannel1Hor].pColour_ = &m_Magenta;
        m_Pens[pChannel1Ver].pColour_ = &m_CyanLight;
        m_Pens[pChannel2Hor].pColour_ = wxLIGHT_GREY;
        m_Pens[pChannel2Ver].pColour_ = &m_MagentaLight;
        m_Pens[pChannel3Hor].pColour_ = &m_ADarkerKindOfGreen;
        m_Pens[pChannel3Ver].pColour_ = wxGREEN;
        break;
    case ibpfYUV422Packed:
    case ibpfYUV444Planar:
    case ibpfYUV422Planar:
    case ibpfYUV422_10Packed:
    case ibpfYUV444Packed:
    case ibpfYUV444_10Packed:
        m_Pens[pChannel0Hor].pColour_ = wxBLACK;
        m_Pens[pChannel0Ver].pColour_ = wxCYAN;
        m_Pens[pChannel1Hor].pColour_ = &m_Magenta;
        m_Pens[pChannel1Ver].pColour_ = wxLIGHT_GREY;
        m_Pens[pChannel2Hor].pColour_ = &m_CyanLight;
        m_Pens[pChannel2Ver].pColour_ = &m_MagentaLight;
        m_Pens[pChannel3Hor].pColour_ = &m_ADarkerKindOfGreen;
        m_Pens[pChannel3Ver].pColour_ = wxGREEN;
        break;
    case ibpfMono8:
    case ibpfMono16:
    case ibpfMono10:
    case ibpfMono12:
    case ibpfMono12Packed_V1:
    case ibpfMono12Packed_V2:
    case ibpfMono14:
    case ibpfMono32:
    case ibpfRaw:
        m_Pens[pChannel0Hor].pColour_ = wxBLACK;
        m_Pens[pChannel0Ver].pColour_ = wxGREEN;
        m_Pens[pChannel1Hor].pColour_ = wxBLUE;
        m_Pens[pChannel1Ver].pColour_ = wxLIGHT_GREY;
        m_Pens[pChannel2Hor].pColour_ = wxCYAN;
        m_Pens[pChannel2Ver].pColour_ = wxRED;
        m_Pens[pChannel3Hor].pColour_ = &m_ADarkerKindOfGreen;
        m_Pens[pChannel3Ver].pColour_ = &m_Magenta;
        break;
    case ibpfBGR888Packed:
    case ibpfBGR101010Packed_V2:
    case ibpfRGBx888Packed:
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
    case ibpfRGB888Packed:
    case ibpfRGB101010Packed:
    case ibpfRGB121212Packed:
    case ibpfRGB141414Packed:
    case ibpfRGB161616Packed:
    default:
        m_Pens[pChannel0Hor].pColour_ = wxRED;
        m_Pens[pChannel0Ver].pColour_ = wxGREEN;
        m_Pens[pChannel1Hor].pColour_ = wxBLUE;
        m_Pens[pChannel1Ver].pColour_ = wxLIGHT_GREY;
        m_Pens[pChannel2Hor].pColour_ = wxCYAN;
        m_Pens[pChannel2Ver].pColour_ = wxBLACK;
        m_Pens[pChannel3Hor].pColour_ = &m_ADarkerKindOfGreen;
        m_Pens[pChannel3Ver].pColour_ = &m_Magenta;
        break;
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetBayerDescriptions( mvIMPACT::acquire::TBayerMosaicParity bayerParity )
//-----------------------------------------------------------------------------
{
    switch( bayerParity )
    {
    case bmpBG:
        m_Pens[pEvenEven].description_ = wxString( wxT( "B" ) );
        m_Pens[pEvenOdd].description_ = wxString( wxT( "G(blue)" ) );
        m_Pens[pOddEven].description_ = wxString( wxT( "G(red)" ) );
        m_Pens[pOddOdd].description_ = wxString( wxT( "R" ) );
        break;
    case bmpGB:
        m_Pens[pEvenEven].description_ = wxString( wxT( "G(blue)" ) );
        m_Pens[pEvenOdd].description_ = wxString( wxT( "B" ) );
        m_Pens[pOddEven].description_ = wxString( wxT( "R" ) );
        m_Pens[pOddOdd].description_ = wxString( wxT( "G(red)" ) );
        break;
    case bmpGR:
        m_Pens[pEvenEven].description_ = wxString( wxT( "G(red)" ) );
        m_Pens[pEvenOdd].description_ = wxString( wxT( "R" ) );
        m_Pens[pOddEven].description_ = wxString( wxT( "B" ) );
        m_Pens[pOddOdd].description_ = wxString( wxT( "G(blue)" ) );
        break;
    case bmpRG:
        m_Pens[pEvenEven].description_ = wxString( wxT( "R" ) );
        m_Pens[pEvenOdd].description_ = wxString( wxT( "G(red)" ) );
        m_Pens[pOddEven].description_ = wxString( wxT( "G(blue)" ) );
        m_Pens[pOddOdd].description_ = wxString( wxT( "B" ) );
        break;
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetBayerPens( mvIMPACT::acquire::TImageBufferPixelFormat pixelFormat, mvIMPACT::acquire::TBayerMosaicParity bayerParity )
//-----------------------------------------------------------------------------
{
    switch( bayerParity )
    {
    case bmpBG:
        m_Pens[pEvenEven].pColour_ = wxBLUE;
        m_Pens[pEvenOdd].pColour_ = wxGREEN;
        m_Pens[pOddEven].pColour_ = &m_ADarkerKindOfGreen;
        m_Pens[pOddOdd].pColour_ = wxRED;
        break;
    case bmpGB:
        m_Pens[pEvenEven].pColour_ = wxGREEN;
        m_Pens[pEvenOdd].pColour_ = wxBLUE;
        m_Pens[pOddEven].pColour_ = wxRED;
        m_Pens[pOddOdd].pColour_ = &m_ADarkerKindOfGreen;
        break;
    case bmpGR:
        m_Pens[pEvenEven].pColour_ = &m_ADarkerKindOfGreen;
        m_Pens[pEvenOdd].pColour_ = wxRED;
        m_Pens[pOddEven].pColour_ = wxBLUE;
        m_Pens[pOddOdd].pColour_ = wxGREEN;
        break;
    case bmpRG:
        m_Pens[pEvenEven].pColour_ = wxRED;
        m_Pens[pEvenOdd].pColour_ = &m_ADarkerKindOfGreen;
        m_Pens[pOddEven].pColour_ = wxGREEN;
        m_Pens[pOddOdd].pColour_ = wxBLUE;
        break;
    default:
        SetDefaultPens( pixelFormat );
        break;
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetupGDIData( const mvIMPACT::acquire::ImageBuffer* pIB, int channelCount, mvIMPACT::acquire::TBayerMosaicParity bayerParity )
//-----------------------------------------------------------------------------
{
    if( bayerParity == bmpUndefined )
    {
        for( int channel = 0; channel < channelCount; channel++ )
        {
            if( channel < PLANE_CNT )
            {
                switch( pIB->pixelFormat )
                {
                case ibpfBGR888Packed:
                case ibpfBGR101010Packed_V2:
                    m_Pens[channel].description_ = ConvertedString( pIB->pChannels[pIB->iChannelCount - 1 - channel].szChannelDesc );
                    break;
                default:
                    m_Pens[channel].description_ = ConvertedString( pIB->pChannels[channel].szChannelDesc );
                    break;
                }
            }
        }
        SetDefaultPens( pIB->pixelFormat );
    }
    else
    {
        SetBayerDescriptions( bayerParity );
        SetBayerPens( pIB->pixelFormat, bayerParity );
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetDisplayMethod( TDisplayMethod displayMethod )
//-----------------------------------------------------------------------------
{
    if( displayMethod == m_DisplayMethod )
    {
        return;
    }

    m_DisplayMethod = displayMethod;
    UpdateAnalysisOutput( true );
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetAOIFullMode( bool boActive )
//-----------------------------------------------------------------------------
{
    m_boAOIFullMode = boActive;
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetHistoryDepth( unsigned int value )
//-----------------------------------------------------------------------------
{
    if( value < 1 )
    {
        return;
    }
    if( AssignDisplayParameter( m_HistoryDepth, value ) )
    {
        UpdateInternalData();
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetDrawStartPercent( int value_percent )
//-----------------------------------------------------------------------------
{
    if( ( value_percent < 0 ) || ( value_percent > 99 ) )
    {
        return;
    }
    RefreshAnalysisData();
    AssignDisplayParameter( m_DrawStart_percent, value_percent );
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetDrawWindowWidthPercent( int value_percent )
//-----------------------------------------------------------------------------
{
    if( ( value_percent < 1 ) || ( value_percent > 100 ) )
    {
        return;
    }
    RefreshAnalysisData();
    AssignDisplayParameter( m_DrawWindow_percent, value_percent );
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::GetDrawRange( unsigned int* pFrom, unsigned int* pTo ) const
//-----------------------------------------------------------------------------
{
    unsigned int from = static_cast<unsigned int>( static_cast<double>( m_valCount ) * ( static_cast<double>( m_DrawStart_percent ) / 100. ) );
    unsigned int to = from + static_cast<unsigned int>( static_cast<double>( m_valCount - 1 ) * ( static_cast<double>( GetPercentageToDraw() ) / 100. ) );
    if( pFrom )
    {
        *pFrom = from;
    }
    if( pTo )
    {
        *pTo = to;
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetProcessBayerParity( bool boActive )
//-----------------------------------------------------------------------------
{
    m_boProcessBayerParity = boActive;
}

//-----------------------------------------------------------------------------
wxGrid* PlotCanvasImageAnalysis::GetGrid( void ) const
//-----------------------------------------------------------------------------
{
    return m_pNumericalDisplay;
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::HandleSizeEvent( wxSizeEvent& )
//-----------------------------------------------------------------------------
{
    if( GetDisplayMethod() == PlotCanvasImageAnalysis::dmGraphical )
    {
        // otherwise not every window message will correctly repaint the window
        UpdateAnalysisOutput( true );
    }
}

//-----------------------------------------------------------------------------
bool PlotCanvasImageAnalysis::MustUpdate( const ImageBuffer* pIB, int plotChannelCount, int x, int y, int w, int h, bool boForceRefresh, bool* pboAOIChanged /* = 0 */, bool* pboImageFormatChanged /* = 0 */ )
//-----------------------------------------------------------------------------
{
    if( !pIB || !pIB->vpData || !IsActive() || ( pIB->iWidth == 0 ) || ( pIB->iHeight == 0 ) )
    {
        return false;
    }

    bool boResult = false;

    if( ( m_ChannelCount != plotChannelCount ) || ( m_pixelFormat != pIB->pixelFormat ) )
    {
        boResult = true;
        if( pboImageFormatChanged )
        {
            *pboImageFormatChanged = true;
        }
        m_pixelFormat = pIB->pixelFormat;
        m_boUnsupportedPixelFormat = false;
    }

    if( SetAOI( pIB, x, y, w, h ) )
    {
        if( pboAOIChanged )
        {
            *pboAOIChanged = true;
        }
        boResult = true;
    }
    else if( pboAOIChanged )
    {
        *pboAOIChanged = false;
    }

    if( ( ( m_ImageCount++ % m_UpdateFrequency ) == 0 ) || boForceRefresh )
    {
        boResult = true;
    }

    return boResult;
}

//-----------------------------------------------------------------------------
bool PlotCanvasImageAnalysis::SetAOI( const ImageBuffer* pIB, int x, int y, int w, int h )
//-----------------------------------------------------------------------------
{
    bool boChanged = false;

    if( m_boAOIFullMode )
    {
        x = 0;
        y = 0;
        w = pIB->iWidth;
        h = pIB->iHeight;
    }

    // validate values
    // x-offset
    if( ( x >= 0 ) && ( m_AOIx != x ) )
    {
        m_AOIx = ( x >= pIB->iWidth ) ? 0 : x;
        boChanged = true;
    }
    else if( m_AOIx >= pIB->iWidth )
    {
        m_AOIx = 0;
        boChanged = true;
    }

    // y-offset
    if( ( y >= 0 ) && ( m_AOIy != y ) )
    {
        m_AOIy = ( y >= pIB->iHeight ) ? 0 : y;
        boChanged = true;
    }
    else if( m_AOIy >= pIB->iHeight )
    {
        m_AOIy = 0;
        boChanged = true;
    }

    // aoi width
    if( ( w > 0 ) && ( m_AOIw != w ) )
    {
        int newValue = ( w + m_AOIx > pIB->iWidth ) ? pIB->iWidth - m_AOIx : w;
        // if the AOI width is larger than x-offset + image width, the AOI width must be clipped
        // this might have been the case in the previous image as well and thus doesn't
        // implies that this is definitely a change -> check twice
        if( newValue != m_AOIw )
        {
            m_AOIw = newValue;
            boChanged = true;
        }
    }
    else if( ( m_AOIw + m_AOIx ) > pIB->iWidth )
    {
        m_AOIw = pIB->iWidth - m_AOIx;
        boChanged = true;
    }

    // aoi height
    if( ( h > 0 ) && ( m_AOIh != h ) )
    {
        int newValue = ( h + m_AOIy > pIB->iHeight ) ? pIB->iHeight - m_AOIy : h;
        // if the AOI height is larger than y-offset + image height, the AOI height must be clipped
        // this might have been the case in the previous image as well and thus doesn't
        // implies that this is definitely a change -> check twice
        if( newValue != m_AOIh )
        {
            m_AOIh = newValue;
            boChanged = true;
        }
    }
    else if( ( m_AOIh + m_AOIy ) > pIB->iHeight )
    {
        m_AOIh = pIB->iHeight - m_AOIy;
        boChanged = true;
    }

    if( m_pImageCanvas && ( m_pAOI != 0 ) && boChanged )
    {
        m_pImageCanvas->SetAOI( this, m_AOIx, m_AOIy, m_AOIw, m_AOIh );
    }

    return boChanged;
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetAOIColour( const wxColour& c )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_AOIColour = c;
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetGridSteps( int XSteps, int YSteps )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    if( XSteps > 0 )
    {
        m_GridStepsX = XSteps;
    }
    if( YSteps > 0 )
    {
        m_GridStepsY = YSteps;
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::RegisterImageCanvas( ImageCanvas* p )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_pImageCanvas = p;
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::SetGridValueFormatString( const wxString& gridValueFormatString )
//-----------------------------------------------------------------------------
{
    m_GridValueFormatString = gridValueFormatString;
    UpdateAnalysisOutput( true );
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::UpdateAnalysisOutput( bool boEraseBackground )
//-----------------------------------------------------------------------------
{
    switch( m_DisplayMethod )
    {
    case dmGraphical:
        Refresh( boEraseBackground );
        break;
    case dmNumerical:
        UpdateGrid();
        break;
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasImageAnalysis::UpdateGrid( void )
//-----------------------------------------------------------------------------
{
    if( m_pNumericalDisplay )
    {
        m_pNumericalDisplay->Refresh();
    }
}
