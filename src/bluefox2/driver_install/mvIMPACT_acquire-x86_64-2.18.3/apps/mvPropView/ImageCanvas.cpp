#include <algorithm>
#include "ImageCanvas.h"
#include "PropViewFrame.h"
#include <wx/dcbuffer.h>
#include <wx/settings.h>
#ifdef USE_RAW_BITMAP_SUPPORT
#   include <wx/rawbmp.h>
#else
#   include <wx/image.h>
#endif // USE_RAW_BITMAP_SUPPORT

#ifdef USE_RAW_BITMAP_SUPPORT
typedef wxPixelData<wxBitmap, wxNativePixelFormat> PixelData;
#endif // USE_RAW_BITMAP_SUPPORT

using namespace std;

//-----------------------------------------------------------------------------
struct ImageCanvasImpl
//-----------------------------------------------------------------------------
{
    const wxBrush backgroundBrush;
    wxBitmap currentImage;
    TImageBufferPixelFormat format;
    int currentShiftValue;
    int appliedShiftValue;
    ImageCanvasImpl() : backgroundBrush(
#ifdef __GNUC__
            wxColour( wxT( "GREY" ) ), // needed on Linux to get the same ugly grey pattern as on Windows
#else
            wxSystemSettings::GetColour( wxSYS_COLOUR_APPWORKSPACE ),
#endif
            wxBRUSHSTYLE_CROSSDIAG_HATCH ), format( ibpfRaw ), currentShiftValue( 0 ), appliedShiftValue( 0 ) {}
    int GetShift( int maxShift )
    {
        appliedShiftValue = ( maxShift > currentShiftValue ) ? maxShift - currentShiftValue : 0;
        return appliedShiftValue;
    }
};

//-----------------------------------------------------------------------------
template<typename _Ty>
inline void YUV2RGB8( int& r, int& g, int& b, _Ty y, _Ty u, _Ty v )
//-----------------------------------------------------------------------------
{
    const int OFFSET = 128;
    r = static_cast<int>( static_cast<double>( y )                                             + 1.140 * static_cast<double>( v - OFFSET ) );
    g = static_cast<int>( static_cast<double>( y ) - 0.394 * static_cast<double>( u - OFFSET ) - 0.581 * static_cast<double>( v - OFFSET ) );
    b = static_cast<int>( static_cast<double>( y ) + 2.032 * static_cast<double>( u - OFFSET )                                             );
    r = max( 0, min( r, 255 ) );
    g = max( 0, min( g, 255 ) );
    b = max( 0, min( b, 255 ) );
}

#ifdef USE_RAW_BITMAP_SUPPORT
//-----------------------------------------------------------------------------
void Copy2ByteMonoImage( const ImageBuffer* pIB, PixelData::Iterator& p, PixelData& data, unsigned int shift )
//-----------------------------------------------------------------------------
{
    char val = 0;
    for( int y = 0; y < pIB->iHeight; ++y )
    {
        unsigned short* pSrc = reinterpret_cast<unsigned short*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch );
        PixelData::Iterator rowStart = p;
        for( int x = 0; x < pIB->iWidth; ++x, ++p )
        {
            val = max( 0, min( ( *pSrc ) >> shift, 255 ) );
            ++pSrc;
            p.Red() = val;
            p.Green() = val;
            p.Blue() = val;
        }
        p = rowStart;
        p.OffsetY( data, 1 );
    }
}

//-----------------------------------------------------------------------------
void CopyRGBImage( const ImageBuffer* pIB, PixelData::Iterator& p, PixelData& data, const int order[3], unsigned int inc )
//-----------------------------------------------------------------------------
{
    for( int y = 0; y < pIB->iHeight; y++ )
    {
        unsigned char* pSrc = reinterpret_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch;
        PixelData::Iterator rowStart = p;
        for( int x = 0; x < pIB->iWidth; x++, p++ )
        {
            p.Red() = pSrc[order[0]];
            p.Green() = pSrc[order[1]];
            p.Blue() = pSrc[order[2]];
            pSrc += inc;
        }
        p = rowStart;
        p.OffsetY( data, 1 );
    }
}
//-----------------------------------------------------------------------------
void Copy2ByteRGBImage( const ImageBuffer* pIB, PixelData::Iterator& p, PixelData& data, unsigned int shift, unsigned int inc )
//-----------------------------------------------------------------------------
{
    for( int y = 0; y < pIB->iHeight; ++y )
    {
        unsigned short* pSrc = reinterpret_cast<unsigned short*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch );
        PixelData::Iterator rowStart = p;
        for( int x = 0; x < pIB->iWidth; ++x, ++p )
        {
            p.Red() = max( 0, min( pSrc[2] >> shift, 255 ) );
            p.Green() = max( 0, min( pSrc[1] >> shift, 255 ) );
            p.Blue() = max( 0, min( pSrc[0] >> shift, 255 ) );
            pSrc += inc;
        }
        p = rowStart;
        p.OffsetY( data, 1 );
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void ConvertPackedYUV411_UYYVYYToRGB( const ImageBuffer* pIB, PixelData::Iterator& p, PixelData& data, const int shift = 0 )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        PixelData::Iterator rowStart = p;
        _Ty* pSrc = reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + line * pIB->pChannels[0].iLinePitch );
        _Ty* y = pSrc + 1;
        _Ty* u = pSrc;
        _Ty* v = pSrc + 3;

        for( int x = 0; x < pIB->iWidth; x++, ++p )
        {
            YUV2RGB8( r, g, b,
                      max( 0, min( *y >> shift, 255 ) ),
                      max( 0, min( *u >> shift, 255 ) ),
                      max( 0, min( *v >> shift, 255 ) ) );
            p.Red() = r;
            p.Green() = g;
            p.Blue() = b;
            y += ( x % 2 ) ? 2 : 1;
            if( ( x > 0 ) && ( ( x % 4 ) == 0 ) )
            {
                u += 6;
                v += 6;
            }
        }
        p = rowStart;
        p.OffsetY( data, 1 );
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void ConvertPackedYUV422ToRGB( const ImageBuffer* pIB, PixelData::Iterator& p, PixelData& data, const int shift = 0 )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        PixelData::Iterator rowStart = p;
        _Ty* pSrc = reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + line * pIB->pChannels[0].iLinePitch );
        _Ty* y = ( ( pIB->pixelFormat == ibpfYUV422Packed ) || ( pIB->pixelFormat == ibpfYUV422_10Packed ) ) ? pSrc     : pSrc + 1;
        _Ty* u = ( ( pIB->pixelFormat == ibpfYUV422Packed ) || ( pIB->pixelFormat == ibpfYUV422_10Packed ) ) ? pSrc + 1 : pSrc;
        _Ty* v = ( ( pIB->pixelFormat == ibpfYUV422Packed ) || ( pIB->pixelFormat == ibpfYUV422_10Packed ) ) ? pSrc + 3 : pSrc + 2;

        for( int x = 0; x < pIB->iWidth; x++, ++p )
        {
            YUV2RGB8( r, g, b,
                      max( 0, min( *y >> shift, 255 ) ),
                      max( 0, min( *u >> shift, 255 ) ),
                      max( 0, min( *v >> shift, 255 ) ) );
            p.Red() = r;
            p.Green() = g;
            p.Blue() = b;
            y += 2;
            if( x & 1 )
            {
                u += 4;
                v += 4;
            }
        }
        p = rowStart;
        p.OffsetY( data, 1 );
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void ConvertPackedYUV444ToRGB( const ImageBuffer* pIB, PixelData::Iterator& p, PixelData& data, const int order[3], const int shift = 0 )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        PixelData::Iterator rowStart = p;
        _Ty* pSrc = reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + line * pIB->pChannels[0].iLinePitch );
        for( int x = 0; x < pIB->iWidth; x++, ++p )
        {
            YUV2RGB8( r, g, b,
                      max( 0, min( pSrc[order[0]] >> shift, 255 ) ),
                      max( 0, min( pSrc[order[1]] >> shift, 255 ) ),
                      max( 0, min( pSrc[order[2]] >> shift, 255 ) ) );
            pSrc += 3;
            p.Red() = r;
            p.Green() = g;
            p.Blue() = b;
        }
        p = rowStart;
        p.OffsetY( data, 1 );
    }
}

//-----------------------------------------------------------------------------
void ConvertPlanarYUVToRGB( const ImageBuffer* pIB, PixelData::Iterator& p, PixelData& data )
//-----------------------------------------------------------------------------
{
    unsigned char* pSrc = reinterpret_cast<unsigned char*>( pIB->vpData );
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        PixelData::Iterator rowStart = p;
        unsigned char* y = pSrc + pIB->pChannels[0].iLinePitch * line;
        unsigned char* u = pSrc + pIB->pChannels[1].iChannelOffset + pIB->pChannels[1].iLinePitch * line;
        unsigned char* v = pSrc + pIB->pChannels[2].iChannelOffset + pIB->pChannels[2].iLinePitch * line;

        for( int x = 0; x < pIB->iWidth; x++, ++p )
        {
            YUV2RGB8( r, g, b, *y, *u, *v );
            p.Red() = r;
            p.Green() = g;
            p.Blue() = b;
            ++y;
            if( x & 1 )
            {
                ++u;
                ++v;
            }
        }
        p = rowStart;
        p.OffsetY( data, 1 );
    }
}
#else
//-----------------------------------------------------------------------------
void Copy2ByteMonoImage( const ImageBuffer* pIB, unsigned char* pDst, unsigned int shift )
//-----------------------------------------------------------------------------
{
    unsigned char val;
    for( int y = 0; y < pIB->iHeight; ++y )
    {
        unsigned short* pSrc = reinterpret_cast<unsigned short*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch );
        for( int x = 0; x < pIB->iWidth; x++ )
        {
            val = max( 0, min( ( *pSrc ) >> shift, 255 ) );
            ++pSrc;
            *pDst++ = val;
            *pDst++ = val;
            *pDst++ = val;
        }
    }
}

//-----------------------------------------------------------------------------
void CopyRGBImage( const ImageBuffer* pIB, unsigned char* pDst, const int order[3], unsigned int inc )
//-----------------------------------------------------------------------------
{
    for( int y = 0; y < pIB->iHeight; ++y )
    {
        unsigned char* pSrc = reinterpret_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch;
        for( int x = 0; x < pIB->iWidth; ++x )
        {
            *pDst++ = pSrc[order[0]];
            *pDst++ = pSrc[order[1]];
            *pDst++ = pSrc[order[2]];
            pSrc += inc;
        }
    }
}

//-----------------------------------------------------------------------------
void Copy2ByteRGBImage( const ImageBuffer* pIB, unsigned char* pDst, unsigned int shift, unsigned int inc )
//-----------------------------------------------------------------------------
{
    for( int y = 0; y < pIB->iHeight; ++y )
    {
        unsigned short* pSrc = reinterpret_cast<unsigned short*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch );
        for( int x = 0; x < pIB->iWidth; ++x )
        {
            *pDst++ = max( 0, min( pSrc[2] >> shift, 255 ) );
            *pDst++ = max( 0, min( pSrc[1] >> shift, 255 ) );
            *pDst++ = max( 0, min( pSrc[0] >> shift, 255 ) );
            pSrc += inc;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void ConvertPackedYUV411_UYYVYYToRGB( const ImageBuffer* pIB, unsigned char* pDst, const int shift = 0 )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        _Ty* pSrc = reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + line * pIB->pChannels[0].iLinePitch );
        _Ty* y = pSrc + 1;
        _Ty* u = pSrc;
        _Ty* v = pSrc + 3;

        for( int x = 0; x < pIB->iWidth; x++ )
        {
            YUV2RGB8( r, g, b,
                      max( 0, min( *y >> shift, 255 ) ),
                      max( 0, min( *u >> shift, 255 ) ),
                      max( 0, min( *v >> shift, 255 ) ) );
            *pDst++ = r;
            *pDst++ = g;
            *pDst++ = b;
            y += ( x % 2 ) ? 2 : 1;
            if( ( x > 0 ) && ( ( x % 4 ) == 0 ) )
            {
                u += 6;
                v += 6;
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void ConvertPackedYUV422ToRGB( const ImageBuffer* pIB, unsigned char* pDst, const int shift = 0 )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        _Ty* pSrc = reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + line * pIB->pChannels[0].iLinePitch );
        _Ty* y = ( ( pIB->pixelFormat == ibpfYUV422Packed ) || ( pIB->pixelFormat == ibpfYUV422_10Packed ) ) ? pSrc     : pSrc + 1;
        _Ty* u = ( ( pIB->pixelFormat == ibpfYUV422Packed ) || ( pIB->pixelFormat == ibpfYUV422_10Packed ) ) ? pSrc + 1 : pSrc;
        _Ty* v = ( ( pIB->pixelFormat == ibpfYUV422Packed ) || ( pIB->pixelFormat == ibpfYUV422_10Packed ) ) ? pSrc + 3 : pSrc + 2;

        for( int x = 0; x < pIB->iWidth; x++ )
        {
            YUV2RGB8( r, g, b,
                      max( 0, min( *y >> shift, 255 ) ),
                      max( 0, min( *u >> shift, 255 ) ),
                      max( 0, min( *v >> shift, 255 ) ) );
            *pDst++ = r;
            *pDst++ = g;
            *pDst++ = b;
            y += 2;
            if( x & 1 )
            {
                u += 4;
                v += 4;
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void ConvertPackedYUV444ToRGB( const ImageBuffer* pIB, unsigned char* pDst, const int order[3], const int shift = 0 )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        _Ty* pSrc = reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pIB->vpData ) + line * pIB->pChannels[0].iLinePitch );
        for( int x = 0; x < pIB->iWidth; x++ )
        {
            YUV2RGB8( r, g, b,
                      max( 0, min( pSrc[order[0]] >> shift, 255 ) ),
                      max( 0, min( pSrc[order[1]] >> shift, 255 ) ),
                      max( 0, min( pSrc[order[2]] >> shift, 255 ) ) );
            pSrc += 3;
            *pDst++ = r;
            *pDst++ = g;
            *pDst++ = b;
        }
    }
}

//-----------------------------------------------------------------------------
void ConvertPlanarYUVToRGB( const ImageBuffer* pIB, unsigned char* pDst )
//-----------------------------------------------------------------------------
{
    unsigned char* pSrc = reinterpret_cast<unsigned char*>( pIB->vpData );
    for( int line = 0; line < pIB->iHeight; ++line )
    {
        int r, g, b;
        unsigned char* y = pSrc + pIB->pChannels[0].iLinePitch * line;
        unsigned char* u = pSrc + pIB->pChannels[1].iChannelOffset + pIB->pChannels[1].iLinePitch * line;
        unsigned char* v = pSrc + pIB->pChannels[2].iChannelOffset + pIB->pChannels[2].iLinePitch * line;
        for( int x = 0; x < pIB->iWidth; x++ )
        {
            YUV2RGB8( r, g, b, *y, *u, *v );
            *pDst++ = r;
            *pDst++ = g;
            *pDst++ = b;
            ++y;

            if( x & 1 )
            {
                ++u;
                ++v;
            }
        }
    }
}
#endif // #ifdef USE_RAW_BITMAP_SUPPORT

//-----------------------------------------------------------------------------
ImageCanvas::ImageCanvas( wxWindow* pApp, wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                          const wxSize& size /* = wxDefaultSize */, long style /* = wxSUNKEN_BORDER */, const wxString& name /* = "ImageCanvas" */, bool boActive /* = true */ )
    : DrawingCanvas( parent, id, pos, size, style, name, boActive ), m_boSupportsFullScreenMode( false ), m_boSupportsDifferentScalingModes( false )
//-----------------------------------------------------------------------------
{
    Init( pApp );
    m_pImpl = new ImageCanvasImpl();
}

//-----------------------------------------------------------------------------
ImageCanvas::~ImageCanvas()
//-----------------------------------------------------------------------------
{
    DeleteAOIs();
    delete m_pImpl;
}

//-----------------------------------------------------------------------------
wxPoint ImageCanvas::GetScaledMousePos( int mouseXPos, int mouseYPos ) const
//-----------------------------------------------------------------------------
{
    return wxPoint( static_cast<int>( mouseXPos / m_lastScaleFactor ) - m_lastStartPoint.x,
                    static_cast<int>( mouseYPos / m_lastScaleFactor ) - m_lastStartPoint.y );
}

//-----------------------------------------------------------------------------
void ImageCanvas::IncreaseShiftValue( void )
//-----------------------------------------------------------------------------
{
    if( m_pImpl->currentShiftValue < 8 )
    {
        ++m_pImpl->currentShiftValue;
        SetImage( m_pIB );
    }
}

//-----------------------------------------------------------------------------
void ImageCanvas::DecreaseShiftValue( void )
//-----------------------------------------------------------------------------
{
    if( m_pImpl->currentShiftValue > 0 )
    {
        --m_pImpl->currentShiftValue;
        SetImage( m_pIB );
    }
}

//-----------------------------------------------------------------------------
int ImageCanvas::GetShiftValue( void ) const
//-----------------------------------------------------------------------------
{
    return m_pImpl->currentShiftValue;
}


//-----------------------------------------------------------------------------
void ImageCanvas::SetShiftValue( int value )
//-----------------------------------------------------------------------------
{
    if( m_pImpl->currentShiftValue != value )
    {
        m_pImpl->currentShiftValue = value;
        SetImage( m_pIB );
    }
}


//-----------------------------------------------------------------------------
int ImageCanvas::GetAppliedShiftValue( void ) const
//-----------------------------------------------------------------------------
{
    return m_pImpl->appliedShiftValue;
}

//-----------------------------------------------------------------------------
bool ImageCanvas::IsFullScreen( void ) const
//-----------------------------------------------------------------------------
{
    return false;
}

//-----------------------------------------------------------------------------
void ImageCanvas::OnPaint( wxPaintEvent& )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    wxPaintDC dc( this );
    DoPrepareDC( dc );

    if( !IsActive() )
    {
        m_boRefreshInProgress = false;
        return;
    }

    wxCoord clientWidth, clientHeight;
    dc.GetSize( &clientWidth, &clientHeight );
    wxMemoryDC memDC;

    dc.SetBrush( m_pImpl->backgroundBrush );
    dc.SetPen( *wxTRANSPARENT_PEN ); // no borders round rectangles

    if( !m_pImpl->currentImage.IsOk() )
    {
        dc.DrawRectangle( 0, 0, clientWidth, clientHeight ); // draw background
        m_lastStartPoint.x = -1;
        m_lastStartPoint.y = -1;
        m_lastScaleFactor = -1.0;
    }
    else
    {
        const int imageHeight = m_pImpl->currentImage.GetHeight();
        const int imageWidth = m_pImpl->currentImage.GetWidth();
        memDC.SelectObject( m_pImpl->currentImage );
        if( ( imageHeight > 0 ) && ( imageWidth > 0 ) )
        {
            double scaleFactor;
            if( m_boScaleToClientSize )
            {
                const double scaleX = static_cast<double>( clientWidth ) / static_cast<double>( imageWidth );
                const double scaleY = static_cast<double>( clientHeight ) / static_cast<double>( imageHeight );
                // always keep the aspect ratio
                scaleFactor = ( scaleX <= scaleY ) ? scaleX : scaleY;
            }
            else
            {
                scaleFactor = m_currentZoomFactor;
            }
            const int scaledImageWidth = static_cast<int>( imageWidth * scaleFactor );
            const int scaledImageHeight = static_cast<int>( imageHeight * scaleFactor );
            int scaleCorrectedBlitXOffsetInClient = 0, scaleCorrectedBlitYOffsetInClient = 0;
            int blitXOffsetInClient = 0, blitYOffsetInClient = 0;
            int viewXStart, viewYStart;
            GetViewStart( &viewXStart, &viewYStart );
            // draw bounding box pattern with the background brush...
            if( clientWidth > scaledImageWidth )
            {
                // ... on the left and right
                const int clientRestWidth = clientWidth - scaledImageWidth;
                scaleCorrectedBlitXOffsetInClient = static_cast<int>( static_cast<double>( clientRestWidth ) / ( 2. * scaleFactor ) );
                blitXOffsetInClient = static_cast<int>( clientRestWidth / 2 );
                dc.DrawRectangle( 0, 0, blitXOffsetInClient, clientHeight );
                dc.DrawRectangle( scaledImageWidth + blitXOffsetInClient, 0, clientRestWidth - blitXOffsetInClient, clientHeight );
            }
            if( clientHeight > scaledImageHeight )
            {
                // ... on top and bottom
                const int clientRestHeight = clientHeight - scaledImageHeight;
                scaleCorrectedBlitYOffsetInClient = static_cast<int>( static_cast<double>( clientRestHeight ) / ( 2. * scaleFactor ) );
                blitYOffsetInClient = static_cast<int>( clientRestHeight / 2 );
                dc.DrawRectangle( blitXOffsetInClient, 0, scaledImageWidth, blitYOffsetInClient );
                dc.DrawRectangle( blitXOffsetInClient, scaledImageHeight + blitYOffsetInClient, scaledImageWidth, clientRestHeight - blitYOffsetInClient );
            }
            m_lastStartPoint.x = static_cast<int>( ( ( blitXOffsetInClient > 0 ) ? blitXOffsetInClient : -viewXStart ) / scaleFactor );
            m_lastStartPoint.y = static_cast<int>( ( ( blitYOffsetInClient > 0 ) ? blitYOffsetInClient : -viewYStart ) / scaleFactor );
            m_lastScaleFactor = scaleFactor;

            if( m_pMonitorDisplay && m_pVisiblePartOfImage )
            {
                m_pVisiblePartOfImage->m_rect = wxRect( ( static_cast<int>( scaledImageWidth ) > clientWidth ) ? static_cast<int>( viewXStart / scaleFactor ) : 0,
                                                        ( static_cast<int>( scaledImageHeight ) > clientHeight ) ? static_cast<int>( viewYStart / scaleFactor ) : 0,
                                                        ( static_cast<int>( scaledImageWidth ) > clientWidth ) ? static_cast<int>( clientWidth / scaleFactor ) : imageWidth,
                                                        ( static_cast<int>( scaledImageHeight ) > clientHeight ) ? static_cast<int>( clientHeight / scaleFactor ) : imageHeight );
                m_pMonitorDisplay->Refresh( false );
            }

            dc.SetUserScale( scaleFactor, scaleFactor );
            // produces false colors when scaled and is not faster than the draw method!
            // dc.DrawBitmap( m_pImpl->currentImage, scaleCorrectedBlitXOffsetInClient, scaleCorrectedBlitYOffsetInClient, false );
            dc.Blit( scaleCorrectedBlitXOffsetInClient, scaleCorrectedBlitYOffsetInClient, imageWidth, imageHeight, &memDC, 0, 0 );
            BlitAOIs( dc, 1.0, static_cast<int>( blitXOffsetInClient / scaleFactor ), static_cast<int>( blitYOffsetInClient / scaleFactor ), imageWidth, imageHeight );
            dc.SetUserScale( 1.0, 1.0 );
            BlitPerformanceMessages( dc, blitXOffsetInClient, blitYOffsetInClient, m_pImpl->format );
            BlitInfoStrings( dc, scaleFactor, static_cast<int>( blitXOffsetInClient - ( viewXStart * scaleFactor ) ), static_cast<int>( blitYOffsetInClient - ( viewYStart * scaleFactor ) ), blitXOffsetInClient, blitYOffsetInClient, imageWidth, imageHeight );
            // restore old brush
            dc.SetBrush( wxNullBrush );
        }
        else
        {
            m_lastStartPoint.x = -1;
            m_lastStartPoint.y = -1;
            m_lastScaleFactor = -1.0;
            BlitInfoStrings( dc, 1.0, 10, 10, 10, 10, imageWidth, imageHeight );
        }
    }
    m_boRefreshInProgress = false;
}

//-----------------------------------------------------------------------------
/// \brief Not supported without the mvDisplay library (which uses DirectX internally)
void ImageCanvas::OnPopUpFullScreen( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    return;
}

//-----------------------------------------------------------------------------
void ImageCanvas::RefreshScrollbars( bool /* boMoveToMousePos = false */ )
//-----------------------------------------------------------------------------
{
    const int HScroll_Y = wxSystemSettings::GetMetric( wxSYS_HSCROLL_Y );
    const int VScroll_X = wxSystemSettings::GetMetric( wxSYS_VSCROLL_X );
    const int HScrollRange = GetScrollRange( wxHORIZONTAL );
    const int VScrollRange = GetScrollRange( wxVERTICAL );
    int clientWidth, clientHeight;
    GetClientSize( &clientWidth, &clientHeight );
    int scaledImageWidth = m_pIB ? static_cast<int>( m_pIB->iWidth * m_currentZoomFactor ) : 0;
    int scaledImageHeight = m_pIB ? static_cast<int>( m_pIB->iHeight * m_currentZoomFactor ) : 0;

    int virtualWidth = 0;
    int virtualHeight = 0;
    if( HScrollRange > 0 )
    {
        if( clientWidth + HScroll_Y < scaledImageWidth )
        {
            virtualWidth = scaledImageWidth;
        }
    }
    else if( clientWidth < scaledImageWidth )
    {
        virtualWidth = scaledImageWidth;
    }

    if( VScrollRange > 0 )
    {
        if( clientHeight + VScroll_X < scaledImageHeight )
        {
            virtualHeight = scaledImageHeight;
        }
    }
    else if( clientHeight < scaledImageHeight )
    {
        virtualHeight = scaledImageHeight;
    }

    if( m_boScaleToClientSize || ( ( virtualWidth == 0 ) && ( virtualHeight == 0 ) ) )
    {
        SetScrollbars( 1, 1, 0, 0 );
    }
    else
    {
        SetVirtualSize( virtualWidth, virtualHeight );
    }
}

//-----------------------------------------------------------------------------
ImageCanvas::TSaveResult ImageCanvas::SaveCurrentImage( const wxString& filenameAndPath, const wxString& extension ) const
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker scopeLock( m_critSect );
    if( !m_pImpl->currentImage.IsOk() )
    {
        return srNoImage;
    }

    wxImage img = m_pImpl->currentImage.ConvertToImage();
    if( ( extension == wxT( "bmp" ) ) && ( m_pImpl->format == ibpfMono8 ) )
    {
        img.SetOption( wxIMAGE_OPTION_BMP_FORMAT, wxBMP_8BPP_RED );
    }

    return StoreImage( img, filenameAndPath, extension );
}

//-----------------------------------------------------------------------------
/// \brief Not supported without the mvDisplay library (which uses DirectX internally)
void ImageCanvas::SetFullScreenMode( bool )
//-----------------------------------------------------------------------------
{
    return;
}

//-----------------------------------------------------------------------------
bool ImageCanvas::SetImage( const ImageBuffer* pIB, bool boMustRefresh /* = true */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );

    if( !IsActive() || !pIB )
    {
        return true;
    }

    m_pIB = pIB;
    if( !boMustRefresh )
    {
        return true;
    }

    if( m_boRefreshInProgress )
    {
        ++m_skippedPaintEvents;
        return true;
    }

    bool boImageSizeChanged = false;
    if( !m_pImpl->currentImage.IsOk() ||
        ( m_pImpl->currentImage.GetWidth() != pIB->iWidth ) ||
        ( m_pImpl->currentImage.GetHeight() != pIB->iHeight ) )
    {
        boImageSizeChanged = true;
        if( m_pImpl->currentImage.IsOk() )
        {
            SetMaxZoomFactor( pIB, std::max( m_pImpl->currentImage.GetWidth(), m_pImpl->currentImage.GetHeight() ) );
        }
        else
        {
            SetMaxZoomFactor( pIB, 0 );
        }
    }

    m_pImpl->appliedShiftValue = 0;
    if( !pIB->vpData || ( pIB->iWidth == 0 ) || ( pIB->iHeight == 0 ) )
    {
        // restore initial status as no valid image is currently available
        m_pImpl->currentImage = wxBitmap();
        Refresh( false );
        return true;
    }
#ifdef USE_RAW_BITMAP_SUPPORT
    if( boImageSizeChanged )
    {
        m_pImpl->currentImage = wxBitmap( pIB->iWidth, pIB->iHeight, 24 );
    }
    PixelData data( m_pImpl->currentImage );
    PixelData::Iterator p( data );
    switch( pIB->pixelFormat )
    {
    case ibpfMono8:
        for( int y = 0; y < pIB->iHeight; y++ )
        {
            unsigned char* pSrc = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch;
            PixelData::Iterator rowStart = p;
            for( int x = 0; x < pIB->iWidth; x++, p++ )
            {
                p.Red() = *pSrc;
                p.Green() = *pSrc;
                p.Blue() = *pSrc++;
            }
            p = rowStart;
            p.OffsetY( data, 1 );
        }
        break;
    case ibpfMono16:
        Copy2ByteMonoImage( pIB, p, data, m_pImpl->GetShift( 8 ) );
        break;
    case ibpfRGBx888Packed:
        {
            const int order[3] = { 2, 1, 0 };
            CopyRGBImage( pIB, p, data, order, 4 );
        }
        break;
    case ibpfRGB888Packed:
        {
            const int order[3] = { 2, 1, 0 };
            CopyRGBImage( pIB, p, data, order, 3 );
        }
        break;
    case ibpfBGR888Packed:
        {
            const int order[3] = { 0, 1, 2 };
            CopyRGBImage( pIB, p, data, order, 3 );
        }
        break;
    case ibpfBGR101010Packed_V2:
        {
            unsigned short red, green, blue;
            const int shift = m_pImpl->GetShift( 2 );
            for( int y = 0; y < pIB->iHeight; y++ )
            {
                unsigned int* pPixel = reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch );
                PixelData::Iterator rowStart = p;
                for( int x = 0; x < pIB->iWidth; x++ )
                {
                    DrawingCanvas::GetBGR101010Packed_V2Pixel( *pPixel++, red, green, blue );
                    p.Red() = max( 0, min( red >> shift, 255 ) );
                    p.Green() = max( 0, min( green >> shift, 255 ) );
                    p.Blue() = max( 0, min( blue >> shift, 255 ) );
                    ++p;
                }
                p = rowStart;
                p.OffsetY( data, 1 );
            }
        }
        break;
    case ibpfYUV411_UYYVYY_Packed:
        ConvertPackedYUV411_UYYVYYToRGB<unsigned char>( pIB, p, data );
        break;
    case ibpfYUV422Packed:
    case ibpfYUV422_UYVYPacked:
        ConvertPackedYUV422ToRGB<unsigned char>( pIB, p, data );
        break;
    case ibpfYUV422_10Packed:
    case ibpfYUV422_UYVY_10Packed:
        ConvertPackedYUV422ToRGB<unsigned short>( pIB, p, data, m_pImpl->GetShift( 2 ) );
        break;
    case ibpfYUV444_UYVPacked:
        {
            const int order[3] = { 1, 0, 2 };
            ConvertPackedYUV444ToRGB<unsigned char>( pIB, p, data, order );
        }
        break;
    case ibpfYUV444_UYV_10Packed:
        {
            const int order[3] = { 1, 0, 2 };
            ConvertPackedYUV444ToRGB<unsigned short>( pIB, p, data, order, m_pImpl->GetShift( 2 ) );
        }
        break;
    case ibpfYUV444Packed:
        {
            const int order[3] = { 0, 1, 2 };
            ConvertPackedYUV444ToRGB<unsigned char>( pIB, p, data, order );
        }
        break;
    case ibpfYUV444_10Packed:
        {
            const int order[3] = { 0, 1, 2 };
            ConvertPackedYUV444ToRGB<unsigned short>( pIB, p, data, order, m_pImpl->GetShift( 2 ) );
        }
        break;
    case ibpfYUV422Planar:
        ConvertPlanarYUVToRGB( pIB, p, data );
        break;
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
        for( int y = 0; y < pIB->iHeight; y++ )
        {
            unsigned char* pr = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch;
            unsigned char* pg = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[1].iLinePitch + pIB->pChannels[1].iChannelOffset;
            unsigned char* pb = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[2].iLinePitch + pIB->pChannels[2].iChannelOffset;
            PixelData::Iterator rowStart = p;
            for( int x = 0; x < pIB->iWidth; x++, p++ )
            {
                p.Red() = *pr++;
                p.Green() = *pg++;
                p.Blue() = *pb++;
            }
            p = rowStart;
            p.OffsetY( data, 1 );
        }
        break;
    case ibpfMono10:
        Copy2ByteMonoImage( pIB, p, data, m_pImpl->GetShift( 2 ) );
        break;
    case ibpfMono12:
        Copy2ByteMonoImage( pIB, p, data, m_pImpl->GetShift( 4 ) );
        break;
    case ibpfMono12Packed_V2:
        {
            /// \todo how can we deal with padding in x-direction here???
            const unsigned char* const pSrc = static_cast<unsigned char*>( pIB->vpData );
            const int shift = m_pImpl->GetShift( 4 );
            for( int y = 0; y < pIB->iHeight; y++ )
            {
                PixelData::Iterator rowStart = p;
                const int pixOffset = y * pIB->iWidth;
                for( int x = 0; x < pIB->iWidth; x++ )
                {
                    const unsigned char value = static_cast<unsigned char>( max( 0, min( DrawingCanvas::GetMonoPacked_V2Pixel( pSrc, pixOffset + x, 4 ) >> shift, 255 ) ) );
                    p.Red() = value;
                    p.Green() = value;
                    p.Blue() = value;
                    ++p;
                }
                p = rowStart;
                p.OffsetY( data, 1 );
            }
        }
        break;
    case ibpfMono14:
        Copy2ByteMonoImage( pIB, p, data, m_pImpl->GetShift( 6 ) );
        break;
    case ibpfRGB101010Packed:
        Copy2ByteRGBImage( pIB, p, data, m_pImpl->GetShift( 2 ), 3 );
        break;
    case ibpfRGB121212Packed:
        Copy2ByteRGBImage( pIB, p, data, m_pImpl->GetShift( 4 ), 3 );
        break;
    case ibpfRGB141414Packed:
        Copy2ByteRGBImage( pIB, p, data, m_pImpl->GetShift( 6 ), 3 );
        break;
    case ibpfRGB161616Packed:
        Copy2ByteRGBImage( pIB, p, data, m_pImpl->GetShift( 8 ), 3 );
        break;
    default:
        return false;
    }
#else
    wxImage img( pIB->iWidth, pIB->iHeight, false );
    unsigned char* pDst = img.GetData();
    switch( pIB->pixelFormat )
    {
    case ibpfMono8:
        for( int y = 0; y < pIB->iHeight; y++ )
        {
            unsigned char* pSrc = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch;
            for( int x = 0; x < pIB->iWidth; x++ )
            {
                *pDst++ = *pSrc;
                *pDst++ = *pSrc;
                *pDst++ = *pSrc++;
            }
        }
        break;
    case ibpfMono16:
        Copy2ByteMonoImage( pIB, pDst, m_pImpl->GetShift( 8 ) );
        break;
    case ibpfRGBx888Packed:
        {
            const int order[3] = { 2, 1, 0 };
            CopyRGBImage( pIB, pDst, order, 4 );
        }
        break;
    case ibpfRGB888Packed:
        {
            const int order[3] = { 2, 1, 0 };
            CopyRGBImage( pIB, pDst, order, 3 );
        }
        break;
    case ibpfBGR888Packed:
        {
            const int order[3] = { 0, 1, 2 };
            CopyRGBImage( pIB, pDst, order, 3 );
        }
        break;
    case ibpfBGR101010Packed_V2:
        {
            unsigned short red, green, blue;
            const int shift = m_pImpl->GetShift( 2 );
            for( int y = 0; y < pIB->iHeight; y++ )
            {
                unsigned int* pPixel = reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch );
                for( int x = 0; x < pIB->iWidth; x++ )
                {
                    DrawingCanvas::GetBGR101010Packed_V2Pixel( *pPixel++, red, green, blue );
                    *pDst++ = max( 0, min( red >> shift, 255 ) );
                    *pDst++ = max( 0, min( green >> shift, 255 ) );
                    *pDst++ = max( 0, min( blue >> shift, 255 ) );
                }
            }
        }
        break;
    case ibpfYUV411_UYYVYY_Packed:
        ConvertPackedYUV411_UYYVYYToRGB<unsigned char>( pIB, pDst );
        break;
    case ibpfYUV422Packed:
    case ibpfYUV422_UYVYPacked:
        ConvertPackedYUV422ToRGB<unsigned char>( pIB, pDst );
        break;
    case ibpfYUV422_10Packed:
    case ibpfYUV422_UYVY_10Packed:
        ConvertPackedYUV422ToRGB<unsigned short>( pIB, pDst, m_pImpl->GetShift( 2 ) );
        break;
    case ibpfYUV444_UYVPacked:
        {
            const int order[3] = { 1, 0, 2 };
            ConvertPackedYUV444ToRGB<unsigned char>( pIB, pDst, order );
        }
        break;
    case ibpfYUV444_UYV_10Packed:
        {
            const int order[3] = { 1, 0, 2 };
            ConvertPackedYUV444ToRGB<unsigned short>( pIB, pDst, order, m_pImpl->GetShift( 2 ) );
        }
        break;
    case ibpfYUV444Packed:
        {
            const int order[3] = { 0, 1, 2 };
            ConvertPackedYUV444ToRGB<unsigned char>( pIB, pDst, order );
        }
        break;
    case ibpfYUV444_10Packed:
        {
            const int order[3] = { 0, 1, 2 };
            ConvertPackedYUV444ToRGB<unsigned short>( pIB, pDst, order, m_pImpl->GetShift( 2 ) );
        }
        break;
    case ibpfYUV422Planar:
        ConvertPlanarYUVToRGB( pIB, pDst );
        break;
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
        {
            for( int y = 0; y < pIB->iHeight; y++ )
            {
                unsigned char* pr = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[0].iLinePitch;
                unsigned char* pg = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[1].iLinePitch + pIB->pChannels[1].iChannelOffset;
                unsigned char* pb = static_cast<unsigned char*>( pIB->vpData ) + y * pIB->pChannels[2].iLinePitch + pIB->pChannels[2].iChannelOffset;
                for( int x = 0; x < pIB->iWidth; x++ )
                {
                    *pDst++ = *pr++;
                    *pDst++ = *pg++;
                    *pDst++ = *pb++;
                }
            }
        }
        break;
    case ibpfMono10:
        Copy2ByteMonoImage( pIB, pDst, m_pImpl->GetShift( 2 ) );
        break;
    case ibpfMono12:
        Copy2ByteMonoImage( pIB, pDst, m_pImpl->GetShift( 4 ) );
        break;
    case ibpfMono12Packed_V2:
        {
            /// \todo how can we deal with padding in x-direction here???
            const unsigned char* const pSrc = static_cast<unsigned char*>( pIB->vpData );
            const int shift = m_pImpl->GetShift( 4 );
            for( int y = 0; y < pIB->iHeight; y++ )
            {
                const int pixOffset = y * pIB->iWidth;
                for( int x = 0; x < pIB->iWidth; x++ )
                {
                    const unsigned char value = static_cast<unsigned char>( max( 0, min( DrawingCanvas::GetMonoPacked_V2Pixel( pSrc, pixOffset + x, 4 ) >> shift, 255 ) ) );
                    *pDst++ = value;
                    *pDst++ = value;
                    *pDst++ = value;
                }
            }
        }
        break;
    case ibpfMono14:
        Copy2ByteMonoImage( pIB, pDst, m_pImpl->GetShift( 6 ) );
        break;
    case ibpfRGB101010Packed:
        Copy2ByteRGBImage( pIB, pDst, m_pImpl->GetShift( 2 ), 3 );
        break;
    case ibpfRGB121212Packed:
        Copy2ByteRGBImage( pIB, pDst, m_pImpl->GetShift( 4 ), 3 );
        break;
    case ibpfRGB141414Packed:
        Copy2ByteRGBImage( pIB, pDst, m_pImpl->GetShift( 6 ), 3 );
        break;
    case ibpfRGB161616Packed:
        Copy2ByteRGBImage( pIB, pDst, m_pImpl->GetShift( 8 ), 3 );
        break;
    default:
        return false;
    }
    m_pImpl->currentImage = wxBitmap( img, -1 );
#endif // USE_RAW_BITMAP_SUPPORT
    m_pImpl->format = pIB->pixelFormat;
    if( boImageSizeChanged )
    {
        RefreshScrollbars();
    }
    m_boRefreshInProgress = true;
    Refresh( boImageSizeChanged );
    return true;
}

//-----------------------------------------------------------------------------
void ImageCanvas::SetScalingMode( TScalingMode /*mode*/ )
//-----------------------------------------------------------------------------
{

}
