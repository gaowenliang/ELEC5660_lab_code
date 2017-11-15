#include "HistogramCanvas.h"
#include <math.h>
#ifdef _OPENMP
#   include <omp.h>
#endif // #ifdef _OPENMP

using namespace mvIMPACT::acquire;

//=============================================================================
//================= Implementation HistogramCanvas ============================
//=============================================================================
//-----------------------------------------------------------------------------
HistogramCanvas::HistogramCanvas( wxWindow* parent, const wxString& configName, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                                  const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = wxT("Histogram") */, bool boActive /* = false */, bool boHandleStandardDeviation /* = false */ ) :
    PlotCanvasImageAnalysis( parent, configName, id, pos, size, style, name, boActive, 1 ),
    m_DrawStepWidth( 0 ), m_ppHistogramBuffer( 0 ), m_pHistogramMax( 0 ), m_pHistogramAverage( 0 ), m_pStandardDeviation( 0 ),
    m_boHandleStandardDeviation( boHandleStandardDeviation ), m_CurrentMax( 1, 1 )
//-----------------------------------------------------------------------------
{
    m_plotFeatures.insert( pfPercentageWindow );
    m_plotFeatures.insert( pfProcessBayerParity );
    m_plotFeatures.insert( pfStepWidth );
    SetGridValueFormatString( wxT( "%d" ) );
}

//-----------------------------------------------------------------------------
HistogramCanvas::~HistogramCanvas()
//-----------------------------------------------------------------------------
{
    DeallocateHistogramBuffer();
}

//-----------------------------------------------------------------------------
void HistogramCanvas::CalculateMaxValue( void )
//-----------------------------------------------------------------------------
{
    if( !m_ppHistogramBuffer )
    {
        return;
    }

    // evaluate new max value in the current draw range
    unsigned int from = 0;
    unsigned int to = 0;
    GetDrawRange( &from, &to );
    m_CurrentMax = PlotPoint( m_ppHistogramBuffer[0][from], 0 );
    for( int j = 0; j < m_ChannelCount; j++ )
    {
        m_pHistogramMax[j] = PlotPoint( m_ppHistogramBuffer[j][from], 0 );
        if( from < to )
        {
            for( unsigned int i = from + 1; i <= to; i++ )
            {
                if( m_ppHistogramBuffer[j][i] > m_pHistogramMax[j].cnt_ )
                {
                    m_pHistogramMax[j] = PlotPoint( m_ppHistogramBuffer[j][i], i );
                }
            }
        }
        const double PIX_CNT = static_cast<double>( m_AOIw * m_AOIh ) / ( ( m_bayerParity == bmpUndefined ) ? 1. : 4. );
        m_pHistogramAverage[j] = m_pHistogramAverage[j] / PIX_CNT;
        if( m_pHistogramMax[j].cnt_ == 0 )
        {
            m_pHistogramMax[j].cnt_ = 1;
        }
        if( m_CurrentMax.cnt_ < m_pHistogramMax[j].cnt_ )
        {
            m_CurrentMax = m_pHistogramMax[j];
        }
        if( m_boHandleStandardDeviation )
        {
            double sumOfSquares = 0;
            for( unsigned int i = from; i < to; i++ )
            {
                if( m_ppHistogramBuffer[j][i] > 0 )
                {
                    sumOfSquares += pow( static_cast<double>( i ), 2 ) * m_ppHistogramBuffer[j][i];
                }
            }
            m_pStandardDeviation[j] = sqrt( sumOfSquares / PIX_CNT );
        }
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvas::CalculateSpatialNoiseHistogram( const RequestData& data )
//-----------------------------------------------------------------------------
{
    const ImageBuffer* pIB = data.image_.getBuffer();
    switch( pIB->pixelFormat )
    {
    case ibpfMono8:
        {
            if( m_bayerParity == bmpUndefined )
            {
#ifdef _OPENMP
                #pragma omp parallel for
#endif // #ifdef _OPENMP
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        if( pixel < m_AOIw - 1 )
                        {
                            unsigned char difference = abs( *pData - * ( pData + 1 ) );
                            m_pHistogramAverage[pChannel0Hor] += difference;
                            ++m_ppHistogramBuffer[pChannel0Hor][difference];
                        }
                        if( line < m_AOIh - 1 )
                        {
                            unsigned char difference = abs( *pData - * ( pData + pIB->pChannels[0].iLinePitch ) );
                            m_pHistogramAverage[pChannel0Ver] += difference;
                            ++m_ppHistogramBuffer[pChannel0Ver][difference];
                        }
                        ++pData;
                    }
                }
            }
            else
            {
#ifdef _OPENMP
                #pragma omp parallel for
#endif // #ifdef _OPENMP
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                        if( pixel < m_AOIw - 2 )
                        {
                            unsigned char difference = abs( *pData - * ( pData + 2 ) );
                            m_pHistogramAverage[2 * index] += difference;
                            ++m_ppHistogramBuffer[2 * index][difference];
                        }
                        if( line < m_AOIh - 2 )
                        {
                            unsigned char difference = abs( *pData - * ( pData + 2 * pIB->pChannels[0].iLinePitch ) );
                            m_pHistogramAverage[2 * index + 1] += difference;
                            ++m_ppHistogramBuffer[2 * index + 1][difference];
                        }
                        ++pData;
                    }
                }
            }
        }
        break;
    case ibpfMono10:
    case ibpfMono12:
    case ibpfMono14:
    case ibpfMono16:
        {
            if( m_bayerParity == bmpUndefined )
            {
#ifdef _OPENMP
                #pragma omp parallel for
#endif // #ifdef _OPENMP
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        unsigned short current_pixel = saveAssign( *pData, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                        if( pixel < m_AOIw - 1 )
                        {
                            unsigned short next_pixel_right = saveAssign( *( pData + 1 ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                            unsigned short difference = abs( current_pixel - next_pixel_right );
                            m_pHistogramAverage[pChannel0Hor] += difference;
                            ++m_ppHistogramBuffer[pChannel0Hor][difference];
                        }
                        if( line < m_AOIh - 1 )
                        {
                            unsigned short next_pixel_down = saveAssign( *reinterpret_cast<unsigned short*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                            unsigned short difference = abs( current_pixel - next_pixel_down );
                            m_pHistogramAverage[pChannel0Ver] += difference;
                            ++m_ppHistogramBuffer[pChannel0Ver][difference];
                        }
                        ++pData;
                    }
                }
            }
            else
            {
#ifdef _OPENMP
                #pragma omp parallel for
#endif // #ifdef _OPENMP
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        unsigned short current_pixel = saveAssign( *pData, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                        int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                        if( pixel < m_AOIw - 2 )
                        {
                            unsigned short next_pixel_right = saveAssign( *( pData + 2 ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                            unsigned short difference = abs( current_pixel - next_pixel_right );
                            m_pHistogramAverage[2 * index] += difference;
                            ++m_ppHistogramBuffer[2 * index][difference];
                        }
                        if( line < m_AOIh - 2 )
                        {
                            unsigned short next_pixel_down = saveAssign( *reinterpret_cast<unsigned short*>( reinterpret_cast<unsigned char*>( pData ) + 2 * pIB->pChannels[0].iLinePitch ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                            unsigned short difference = abs( current_pixel - next_pixel_down );
                            m_pHistogramAverage[2 * index + 1] += difference;
                            ++m_ppHistogramBuffer[2 * index + 1][difference];
                        }
                        ++pData;
                    }
                }
            }
        }
        break;
    case ibpfMono12Packed_V1:
        CalculateSpatialNoiseHistogram_MonoPacked( pIB, GetMono12Packed_V1Pixel );
        break;
    case ibpfMono12Packed_V2:
        CalculateSpatialNoiseHistogram_MonoPacked( pIB, GetMono12Packed_V2Pixel );
        break;
    case ibpfBGR888Packed:
    case ibpfYUV444_UYVPacked:
        {
            int order[3] = { 2, 1, 0 };
            CalculateSpatialNoiseHistogram_8u_C3Packed( pIB, order );
        }
        break;
    case ibpfRGBx888Packed:
    case ibpfRGB888Packed:
    case ibpfYUV444Packed:
        {
            int order[3] = { 0, 1, 2 };
            CalculateSpatialNoiseHistogram_8u_C3Packed( pIB, order );
        }
        break;
    case ibpfBGR101010Packed_V2:
        {
#ifdef _OPENMP
            #pragma omp parallel for
#endif // #ifdef _OPENMP
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short red0, green0, blue0;
                unsigned short red1, green1, blue1;
                unsigned short difference = 0;
                unsigned int* pPixel =  reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    GetBGR101010Packed_V2Pixel( *pPixel, red0, green0, blue0 );
                    if( pixel < m_AOIw - 1 )
                    {
                        GetBGR101010Packed_V2Pixel( *( pPixel + 1 ), red1, green1, blue1 );
                        difference = abs( red0 - red1 );
                        m_pHistogramAverage[pChannel0Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel0Hor][difference];
                        difference = abs( green0 - green1 );
                        m_pHistogramAverage[pChannel1Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel1Hor][difference];
                        difference = abs( blue0 - blue1 );
                        m_pHistogramAverage[pChannel2Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel2Hor][difference];
                    }
                    if( line < m_AOIh - 1 )
                    {
                        GetBGR101010Packed_V2Pixel( *reinterpret_cast<unsigned int*>( reinterpret_cast<unsigned char*>( pPixel ) + pIB->pChannels[0].iLinePitch ), red1, green1, blue1 );
                        difference = abs( red0 - red1 );
                        m_pHistogramAverage[pChannel0Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel0Ver][difference];
                        difference = abs( green0 - green1 );
                        m_pHistogramAverage[pChannel1Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel1Ver][difference];
                        difference = abs( blue0 - blue1 );
                        m_pHistogramAverage[pChannel2Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel2Ver][difference];
                    }
                    ++pPixel;
                }
            }
        }
        break;
    case ibpfRGB101010Packed:
    case ibpfRGB121212Packed:
    case ibpfRGB141414Packed:
    case ibpfRGB161616Packed:
    case ibpfYUV444_UYV_10Packed:
    case ibpfYUV444_10Packed:
        {
#ifdef _OPENMP
            #pragma omp parallel for
#endif // #ifdef _OPENMP
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
                unsigned short difference = 0;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {

                    if( pixel < m_AOIw - 1 )
                    {
                        difference = abs( saveAssign( pData[0], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) - saveAssign( pData[0 + pIB->iChannelCount], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) );
                        m_pHistogramAverage[pChannel0Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel0Hor][difference];
                        difference = abs( saveAssign( pData[1], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) - saveAssign( pData[1 + pIB->iChannelCount], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) );
                        m_pHistogramAverage[pChannel1Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel1Hor][difference];
                        difference = abs( saveAssign( pData[2], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) - saveAssign( pData[2 + pIB->iChannelCount], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) );
                        m_pHistogramAverage[pChannel2Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel2Hor][difference];
                    }
                    if( line < m_AOIh - 1 )
                    {
                        unsigned short* pNext_pixel_down = reinterpret_cast<unsigned short*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch );
                        difference = abs( saveAssign( pData[0], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) - saveAssign( pNext_pixel_down[0], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) );
                        m_pHistogramAverage[pChannel0Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel0Ver][difference];
                        difference = abs( saveAssign( pData[1], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) - saveAssign( pNext_pixel_down[1], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) );
                        m_pHistogramAverage[pChannel1Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel1Ver][difference];
                        difference = abs( saveAssign( pData[2], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) - saveAssign( pNext_pixel_down[2], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) ) );
                        m_pHistogramAverage[pChannel2Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel2Ver][difference];
                    }
                    pData += pIB->iChannelCount;
                }
            }
        }
        break;
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
        {
#ifdef _OPENMP
            #pragma omp parallel for
#endif // #ifdef _OPENMP
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned char* pr = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[0].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) + m_AOIx;
                unsigned char* pg = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[1].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[1].iLinePitch ) + m_AOIx;
                unsigned char* pb = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[2].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[2].iLinePitch ) + m_AOIx;
                unsigned char difference = 0;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    if( pixel < m_AOIw - 1 )
                    {
                        difference = abs( pr[0] - pr[1] );
                        m_pHistogramAverage[pChannel0Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel0Hor][difference];
                        difference = abs( pg[0] - pg[1] );
                        m_pHistogramAverage[pChannel1Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel1Hor][difference];
                        difference = abs( pb[0] - pb[1] );
                        m_pHistogramAverage[pChannel2Hor] += difference;
                        ++m_ppHistogramBuffer[pChannel2Hor][difference];
                    }
                    if( line < m_AOIh - 1 )
                    {
                        difference = abs( pr[0] - pr[pIB->pChannels[0].iLinePitch] );
                        m_pHistogramAverage[pChannel0Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel0Ver][difference];
                        difference = abs( pg[0] - pg[pIB->pChannels[1].iLinePitch] );
                        m_pHistogramAverage[pChannel1Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel1Ver][difference];
                        difference = abs( pb[0] - pb[pIB->pChannels[2].iLinePitch] );
                        m_pHistogramAverage[pChannel2Ver] += difference;
                        ++m_ppHistogramBuffer[pChannel2Ver][difference];
                    }
                    ++pr;
                    ++pg;
                    ++pb;
                }
            }
        }
        break;
    case ibpfYUV411_UYYVYY_Packed:
        CalculateSpatialNoiseHistogramYUV411_UYYVYY<unsigned char>( pIB );
        break;
    case ibpfYUV422Packed:
        CalculateSpatialNoiseHistogramYUV422<unsigned char>( pIB );
        break;
    case ibpfYUV422_10Packed:
        CalculateSpatialNoiseHistogramYUV422<unsigned short>( pIB );
        break;
    case ibpfYUV422_UYVYPacked:
        CalculateSpatialNoiseHistogramUYV422<unsigned char>( pIB );
        break;
    case ibpfYUV422_UYVY_10Packed:
        CalculateSpatialNoiseHistogramUYV422<unsigned short>( pIB );
        break;
    case ibpfYUV422Planar:
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned char* py = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pY].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + m_AOIx;
            unsigned char* pu = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pU].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pU].iLinePitch ) + m_AOIx;
            unsigned char* pv = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pV].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pV].iLinePitch ) + m_AOIx;
            unsigned char difference = 0;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                if( pixel < m_AOIw - 1 )
                {
                    difference = abs( py[0] - py[1] );
                    m_pHistogramAverage[pChannel0Hor] += difference;
                    ++m_ppHistogramBuffer[pChannel0Hor][difference];
                    difference = abs( pu[0] - pu[1] );
                    m_pHistogramAverage[pChannel1Hor] += difference;
                    ++m_ppHistogramBuffer[pChannel1Hor][difference];
                    difference = abs( pv[0] - pv[1] );
                    m_pHistogramAverage[pChannel2Hor] += difference;
                    ++m_ppHistogramBuffer[pChannel2Hor][difference];
                }
                if( line < m_AOIh - 1 )
                {
                    difference = abs( py[0] - py[pIB->pChannels[0].iLinePitch] );
                    m_pHistogramAverage[pChannel0Ver] += difference;
                    ++m_ppHistogramBuffer[pChannel0Ver][difference];
                    difference = abs( pu[0] - pu[pIB->pChannels[1].iLinePitch] );
                    m_pHistogramAverage[pChannel1Ver] += difference;
                    ++m_ppHistogramBuffer[pChannel1Ver][difference];
                    difference = abs( pv[0] - pv[pIB->pChannels[2].iLinePitch] );
                    m_pHistogramAverage[pChannel2Ver] += difference;
                    ++m_ppHistogramBuffer[pChannel2Ver][difference];
                }
                ++py;
                if( pixel & 1 )
                {
                    ++pu;
                    ++pv;
                }
            }
        }
        break;
    default:
        // unsupported colour mode
        m_ChannelCount = 0;
        m_boUnsupportedPixelFormat = true;
        break;
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvas::CalculateSpatialNoiseHistogram_8u_C3Packed( const ImageBuffer* pIB, int order[3] )
//-----------------------------------------------------------------------------
{
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
        unsigned char difference = 0;
        for( int pixel = 0; pixel < m_AOIw; pixel++ )
        {
            if( pixel < m_AOIw - 1 )
            {
                difference = abs( pData[order[0]] - pData[order[0] + pIB->iBytesPerPixel] );
                m_pHistogramAverage[pChannel0Hor] += difference;
                ++m_ppHistogramBuffer[pChannel0Hor][difference];
                difference = abs( pData[order[1]] - pData[order[1] + pIB->iBytesPerPixel] );
                m_pHistogramAverage[pChannel1Hor] += difference;
                ++m_ppHistogramBuffer[pChannel1Hor][difference];
                difference = abs( pData[order[2]] - pData[order[2] + pIB->iBytesPerPixel] );
                m_pHistogramAverage[pChannel2Hor] += difference;
                ++m_ppHistogramBuffer[pChannel2Hor][difference];
            }
            if( line < m_AOIh - 1 )
            {
                difference = abs( pData[order[0]] - pData[order[0] + pIB->pChannels[0].iLinePitch] );
                m_pHistogramAverage[pChannel0Ver] += difference;
                ++m_ppHistogramBuffer[pChannel0Ver][difference];
                difference = abs( pData[order[1]] - pData[order[1] + pIB->pChannels[0].iLinePitch] );
                m_pHistogramAverage[pChannel1Ver] += difference;
                ++m_ppHistogramBuffer[pChannel1Ver][difference];
                difference = abs( pData[order[2]] - pData[order[2] + pIB->pChannels[0].iLinePitch] );
                m_pHistogramAverage[pChannel2Ver] += difference;
                ++m_ppHistogramBuffer[pChannel2Ver][difference];
            }
            pData += pIB->iBytesPerPixel;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculateSpatialNoiseHistogram_MonoPacked( const ImageBuffer* pIB, _Ty pixelAccessFn )
//-----------------------------------------------------------------------------
{
    const unsigned char* pData = static_cast<unsigned char*>( pIB->vpData );
    if( m_bayerParity == bmpUndefined )
    {
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            const int pixOffset = ( m_AOIy + line ) * pIB->iWidth;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short current_pixel = pixelAccessFn( pData, pixOffset + m_AOIx + pixel );
                if( pixel < m_AOIw - 1 )
                {
                    unsigned short next_pixel_right = pixelAccessFn( pData, pixOffset + m_AOIx + pixel + 1 );
                    unsigned short difference = abs( current_pixel - next_pixel_right );
                    m_pHistogramAverage[pChannel0Hor] += difference;
                    ++m_ppHistogramBuffer[pChannel0Hor][difference];
                }
                if( line < m_AOIh - 1 )
                {
                    unsigned short next_pixel_down = pixelAccessFn( pData, pixOffset + pIB->iWidth + m_AOIx + pixel );
                    unsigned short difference = abs( current_pixel - next_pixel_down );
                    m_pHistogramAverage[pChannel0Ver] += difference;
                    ++m_ppHistogramBuffer[pChannel0Ver][difference];
                }
            }
        }
    }
    else
    {
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                const int pixOffset = ( m_AOIy + line ) * pIB->iWidth;
                unsigned short current_pixel = pixelAccessFn( pData, pixOffset + m_AOIx + pixel );
                int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                if( pixel < m_AOIw - 2 )
                {
                    unsigned short next_pixel_right = pixelAccessFn( pData, pixOffset + m_AOIx + pixel + 2 );
                    unsigned short difference = abs( current_pixel - next_pixel_right );
                    m_pHistogramAverage[2 * index] += difference;
                    ++m_ppHistogramBuffer[2 * index][difference];
                }
                if( line < m_AOIh - 2 )
                {
                    unsigned short next_pixel_down = pixelAccessFn( pData, pixOffset + 2 * pIB->iWidth + m_AOIx + pixel );
                    unsigned short difference = abs( current_pixel - next_pixel_down );
                    m_pHistogramAverage[2 * index + 1] += difference;
                    ++m_ppHistogramBuffer[2 * index + 1][difference];
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculateSpatialNoiseHistogramUYV422( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    const bool boIsVY = m_AOIx & 1;
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
        _Ty difference = 0;
        for( int pixel = 0; pixel < m_AOIw / 2; pixel++ )
        {
            if( pixel < ( m_AOIw - 3 ) / 2 )
            {
                difference = abs( pData[1] - pData[3] );
                m_pHistogramAverage[pChannel0Hor] += difference;
                ++m_ppHistogramBuffer[pChannel0Hor][difference];
                difference = abs( pData[3] - pData[5] );
                m_pHistogramAverage[pChannel0Hor] += difference;
                ++m_ppHistogramBuffer[pChannel0Hor][difference];
                difference = abs( pData[0] - pData[4] );
                m_pHistogramAverage[boIsVY ? pChannel2Hor : pChannel1Hor] += 2 * difference;
                m_ppHistogramBuffer[boIsVY ? pChannel2Hor : pChannel1Hor][difference] += 2;
                difference = abs( pData[2] - pData[6] );
                m_pHistogramAverage[boIsVY ? pChannel1Hor : pChannel2Hor] += 2 * difference;
                m_ppHistogramBuffer[boIsVY ? pChannel1Hor : pChannel2Hor][difference] += 2;
            }
            if( line < m_AOIh - 1 )
            {
                difference = abs( pData[1] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 1 ) );
                m_pHistogramAverage[pChannel0Ver] += difference;
                ++m_ppHistogramBuffer[pChannel0Ver][difference];
                difference = abs( pData[0] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 0 ) );
                m_pHistogramAverage[boIsVY ? pChannel2Ver : pChannel1Ver] += 2 * difference;
                m_ppHistogramBuffer[boIsVY ? pChannel2Ver : pChannel1Ver][difference] += 2;
                difference = abs( pData[3] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 3 ) );
                m_pHistogramAverage[pChannel0Ver] += difference;
                ++m_ppHistogramBuffer[pChannel0Ver][difference];
                difference = abs( pData[2] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 2 ) );
                m_pHistogramAverage[boIsVY ? pChannel1Ver : pChannel2Ver] += 2 * difference;
                m_ppHistogramBuffer[boIsVY ? pChannel1Ver : pChannel2Ver][difference] *= 2;
            }
            pData += 4;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculateSpatialNoiseHistogramYUV411_UYYVYY( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    ProcessingHelperYUV411_UYYVYY ph( m_AOIx );
    int incToNextYComponent = 0;
    _Ty difference = 0;
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) ) + ph.GetOffsetToFirstY();
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            ph.RefreshUAndVOffsets( pixel );
            incToNextYComponent = ( pixel % 2 ) ? 2 : 1;
            if( pixel < LOOP_END - 1 )
            {
                difference = abs( pData[0] - pData[incToNextYComponent] );
                m_pHistogramAverage[pChannel1Hor] += difference;
                ++m_ppHistogramBuffer[pChannel1Hor][difference];
                if( pixel < LOOP_END - 4 )
                {
                    difference = abs( pData[ph.GetUOffset()] - pData[ph.GetUOffset() + 6] );
                    m_pHistogramAverage[pChannel0Hor] += difference;
                    ++m_ppHistogramBuffer[pChannel0Hor][difference];
                    difference = abs( pData[ph.GetVOffset()] - pData[ph.GetVOffset() + 6] );
                    m_pHistogramAverage[pChannel2Hor] += difference;
                    ++m_ppHistogramBuffer[pChannel2Hor][difference];
                }
            }
            if( line < m_AOIh - 1 )
            {
                difference = abs( pData[0] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 0 ) );
                m_pHistogramAverage[pChannel1Ver] += difference;
                ++m_ppHistogramBuffer[pChannel1Ver][difference];
                difference = abs( pData[ph.GetUOffset()] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + ph.GetUOffset() ) );
                m_pHistogramAverage[pChannel0Ver] += difference;
                ++m_ppHistogramBuffer[pChannel0Ver][difference];
                difference = abs( pData[ph.GetVOffset()] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + ph.GetVOffset() ) );
                m_pHistogramAverage[pChannel2Ver] += difference;
                ++m_ppHistogramBuffer[pChannel2Ver][difference];
            }
            pData += incToNextYComponent;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculateSpatialNoiseHistogramYUV422( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    const bool boIsYV = m_AOIx & 1;
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
        _Ty difference = 0;
        for( int pixel = 0; pixel < m_AOIw / 2; pixel++ )
        {
            if( pixel < ( m_AOIw - 3 ) / 2 )
            {
                difference = abs( pData[0] - pData[2] );
                m_pHistogramAverage[pChannel0Hor] += difference;
                ++m_ppHistogramBuffer[pChannel0Hor][difference];
                difference = abs( pData[2] - pData[4] );
                m_pHistogramAverage[pChannel0Hor] += difference;
                ++m_ppHistogramBuffer[pChannel0Hor][difference];
                difference = abs( pData[1] - pData[5] );
                m_pHistogramAverage[boIsYV ? pChannel2Hor : pChannel1Hor] += 2 * difference;
                m_ppHistogramBuffer[boIsYV ? pChannel2Hor : pChannel1Hor][difference] += 2;
                difference = abs( pData[3] - pData[7] );
                m_pHistogramAverage[boIsYV ? pChannel1Hor : pChannel2Hor] += 2 * difference;
                m_ppHistogramBuffer[boIsYV ? pChannel1Hor : pChannel2Hor][difference] += 2;
            }
            if( line < m_AOIh - 1 )
            {
                difference = abs( pData[0] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 0 ) );
                m_pHistogramAverage[pChannel0Ver] += difference;
                ++m_ppHistogramBuffer[pChannel0Ver][difference];
                difference = abs( pData[1] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 1 ) );
                m_pHistogramAverage[boIsYV ? pChannel2Ver : pChannel1Ver] += 2 * difference;
                m_ppHistogramBuffer[boIsYV ? pChannel2Ver : pChannel1Ver][difference] += 2;
                difference = abs( pData[2] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 2 ) );
                m_pHistogramAverage[pChannel0Ver] += difference;
                ++m_ppHistogramBuffer[pChannel0Ver][difference];
                difference = abs( pData[3] - * ( reinterpret_cast<_Ty*>( reinterpret_cast<unsigned char*>( pData ) + pIB->pChannels[0].iLinePitch ) + 1 ) );
                m_pHistogramAverage[boIsYV ? pChannel1Ver : pChannel2Ver] += 2 * difference;
                m_ppHistogramBuffer[boIsYV ? pChannel1Ver : pChannel2Ver][difference] += 2;
            }
            pData += 4;
        }
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvas::CalculatePixelHistogram( const RequestData& data )
//-----------------------------------------------------------------------------
{
    const ImageBuffer* pIB = data.image_.getBuffer();
    switch( pIB->pixelFormat )
    {
    case ibpfMono8:
        if( m_bayerParity == bmpUndefined )
        {
#ifdef _OPENMP
            #pragma omp parallel for
#endif // #ifdef _OPENMP
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    m_pHistogramAverage[pRed] += *pData;
                    ++m_ppHistogramBuffer[pRed][*pData++];
                }
            }
        }
        else
        {
#ifdef _OPENMP
            #pragma omp parallel for
#endif // #ifdef _OPENMP
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                    m_pHistogramAverage[index] += *pData;
                    ++m_ppHistogramBuffer[index][*pData++];
                }
            }
        }
        break;
    case ibpfMono10:
    case ibpfMono12:
    case ibpfMono14:
    case ibpfMono16:
        if( m_bayerParity == bmpUndefined )
        {
#ifdef _OPENMP
            #pragma omp parallel for
#endif // #ifdef _OPENMP
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    unsigned short value = saveAssign( *pData, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                    m_pHistogramAverage[pRed] += value;
                    ++m_ppHistogramBuffer[pRed][value];
                    ++pData;
                }
            }
        }
        else
        {
#ifdef _OPENMP
            #pragma omp parallel for
#endif // #ifdef _OPENMP
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    unsigned short value = saveAssign( *pData, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                    int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                    m_pHistogramAverage[index] += value;
                    ++m_ppHistogramBuffer[index][value];
                    ++pData;
                }
            }
        }
        break;
    case ibpfMono12Packed_V1:
        CalculatePixelHistogram_MonoPacked( pIB, GetMono12Packed_V1Pixel );
        break;
    case ibpfMono12Packed_V2:
        CalculatePixelHistogram_MonoPacked( pIB, GetMono12Packed_V2Pixel );
        break;
    case ibpfBGR888Packed:
        {
            int order[3] = { 2, 1, 0 };
            CalculatePixelHistogram_8u_RGBPacked( pIB, order );
        }
        break;
    case ibpfRGBx888Packed:
    case ibpfRGB888Packed:
        {
            int order[3] = { 0, 1, 2 };
            CalculatePixelHistogram_8u_RGBPacked( pIB, order );
        }
        break;
    case ibpfBGR101010Packed_V2:
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned short red, green, blue;
            unsigned int* p =  reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                GetBGR101010Packed_V2Pixel( *p++, red, green, blue );
                m_pHistogramAverage[pBlue] += blue;
                ++m_ppHistogramBuffer[pBlue][blue];
                m_pHistogramAverage[pGreen] += green;
                ++m_ppHistogramBuffer[pGreen][green];
                m_pHistogramAverage[pRed] += red;
                ++m_ppHistogramBuffer[pRed][red];
            }
        }
        break;
    case ibpfRGB101010Packed:
    case ibpfRGB121212Packed:
    case ibpfRGB141414Packed:
    case ibpfRGB161616Packed:
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short value = saveAssign( *pData, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                m_pHistogramAverage[pBlue] += value;
                ++m_ppHistogramBuffer[pBlue][value];
                ++pData;
                value = saveAssign( *pData, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                m_pHistogramAverage[pGreen] += value;
                ++m_ppHistogramBuffer[pGreen][value];
                ++pData;
                value = saveAssign( *pData, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                m_pHistogramAverage[pRed] += value;
                ++m_ppHistogramBuffer[pRed][value];
                ++pData;
            }
        }
        break;
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned char* pr = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pRed].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + m_AOIx;
            unsigned char* pg = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pGreen].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pGreen].iLinePitch ) + m_AOIx;
            unsigned char* pb = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pBlue].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pBlue].iLinePitch ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                m_pHistogramAverage[pRed] += *pr;
                ++m_ppHistogramBuffer[pRed][*pr++];
                m_pHistogramAverage[pGreen] += *pg;
                ++m_ppHistogramBuffer[pGreen][*pg++];
                m_pHistogramAverage[pBlue] += *pb;
                ++m_ppHistogramBuffer[pBlue][*pb++];
            }
        }
        break;
    case ibpfYUV411_UYYVYY_Packed:
        CalculatePixelHistogramYUV411_UYYVYY<unsigned char>( pIB );
        break;
    case ibpfYUV422Packed:
        CalculatePixelHistogramYUV422<unsigned char>( pIB );
        break;
    case ibpfYUV422_10Packed:
        CalculatePixelHistogramYUV422<unsigned short>( pIB );
        break;
    case ibpfYUV422_UYVYPacked:
        CalculatePixelHistogramUYV422<unsigned char>( pIB );
        break;
    case ibpfYUV422_UYVY_10Packed:
        CalculatePixelHistogramUYV422<unsigned short>( pIB );
        break;
    case ibpfYUV444_UYVPacked:
        CalculatePixelHistogramYUV444<unsigned char>( pIB );
        break;
    case ibpfYUV444_UYV_10Packed:
        CalculatePixelHistogramYUV444<unsigned short>( pIB );
        break;
    case ibpfYUV444Packed:
        CalculatePixelHistogramYUV444<unsigned char>( pIB );
        break;
    case ibpfYUV444_10Packed:
        CalculatePixelHistogramYUV444<unsigned short>( pIB );
        break;
    case ibpfYUV422Planar:
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned char* py = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pY].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + m_AOIx;
            unsigned char* pu = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pU].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pU].iLinePitch ) + m_AOIx;
            unsigned char* pv = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pV].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pV].iLinePitch ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                m_pHistogramAverage[pY] += *py;
                ++m_ppHistogramBuffer[pY][*py++];
                m_pHistogramAverage[pU] += *pu;
                ++m_ppHistogramBuffer[pU][*pu];
                m_pHistogramAverage[pV] += *pv;
                ++m_ppHistogramBuffer[pV][*pv];
                if( pixel & 1 )
                {
                    ++pu;
                    ++pv;
                }
            }
        }
        break;
    default:
        // unsupported colour mode
        m_ChannelCount = 0;
        m_boUnsupportedPixelFormat = true;
        break;
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvas::CalculatePixelHistogram_8u_RGBPacked( const ImageBuffer* pIB, int order[3] )
//-----------------------------------------------------------------------------
{
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
        for( int pixel = 0; pixel < m_AOIw; pixel++ )
        {
            m_pHistogramAverage[pBlue] += pData[order[0]];
            ++m_ppHistogramBuffer[pBlue][pData[order[0]]];
            m_pHistogramAverage[pGreen] += pData[order[1]];
            ++m_ppHistogramBuffer[pGreen][pData[order[1]]];
            m_pHistogramAverage[pRed] += pData[order[2]];
            ++m_ppHistogramBuffer[pRed][pData[order[2]]];
            pData += pIB->iBytesPerPixel;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculatePixelHistogram_MonoPacked( const ImageBuffer* pIB, _Ty pixelAccessFn )
//-----------------------------------------------------------------------------
{
    if( m_bayerParity == bmpUndefined )
    {
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            const int pixOffset = ( m_AOIy + line ) * pIB->iWidth;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short value = pixelAccessFn( static_cast<unsigned char*>( pIB->vpData ), pixOffset + m_AOIx + pixel );
                m_pHistogramAverage[pRed] += value;
                ++m_ppHistogramBuffer[pRed][value];
            }
        }
    }
    else
    {
#ifdef _OPENMP
        #pragma omp parallel for
#endif // #ifdef _OPENMP
        for( int line = 0; line < m_AOIh; line++ )
        {
            const int pixOffset = ( m_AOIy + line ) * pIB->iWidth;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short value = pixelAccessFn( static_cast<unsigned char*>( pIB->vpData ), pixOffset + m_AOIx + pixel );
                int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                m_pHistogramAverage[index] += value;
                ++m_ppHistogramBuffer[index][value];
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculatePixelHistogramUYV422( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            if( pixel % 2 )
            {
                m_pHistogramAverage[pChannel2] += *pData;
                ++m_ppHistogramBuffer[pChannel2][*pData];
                m_pHistogramAverage[pChannel2] += *pData;
                ++m_ppHistogramBuffer[pChannel2][*pData++];
            }
            else
            {
                m_pHistogramAverage[pChannel0] += *pData;
                ++m_ppHistogramBuffer[pChannel0][*pData];
                m_pHistogramAverage[pChannel0] += *pData;
                ++m_ppHistogramBuffer[pChannel0][*pData++];
            }
            m_pHistogramAverage[pChannel1] += *pData;
            ++m_ppHistogramBuffer[pChannel1][*pData++];
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculatePixelHistogramYUV411_UYYVYY( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    ProcessingHelperYUV411_UYYVYY ph( m_AOIx );
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) ) + ph.GetOffsetToFirstY();
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            m_pHistogramAverage[pChannel1] += *pData;
            ++m_ppHistogramBuffer[pChannel1][*pData];
            ph.RefreshUAndVOffsets( pixel );
            m_pHistogramAverage[pChannel0] += pData[ph.GetUOffset()];
            ++m_ppHistogramBuffer[pChannel0][pData[ph.GetUOffset()]];
            m_pHistogramAverage[pChannel2] += pData[ph.GetVOffset()];
            ++m_ppHistogramBuffer[pChannel2][pData[ph.GetVOffset()]];
            pData += ( pixel % 2 ) ? 2 : 1;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculatePixelHistogramYUV422( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            m_pHistogramAverage[pY] += *pData;
            ++m_ppHistogramBuffer[pY][*pData++];
            if( pixel % 2 )
            {
                m_pHistogramAverage[pV] += 2 * pData[0];
                m_ppHistogramBuffer[pV][*pData++] += 2;
            }
            else
            {
                m_pHistogramAverage[pU] += 2 * pData[0];
                m_ppHistogramBuffer[pU][*pData++] += 2;
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvas::CalculatePixelHistogramYUV444( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            m_pHistogramAverage[0] += pData[0];
            ++m_ppHistogramBuffer[0][pData[0]];
            m_pHistogramAverage[1] += pData[1];
            ++m_ppHistogramBuffer[1][pData[1]];
            m_pHistogramAverage[2] += pData[2];
            ++m_ppHistogramBuffer[2][pData[2]];
            pData += 3;
        }
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvas::DeallocateHistogramBuffer( void )
//-----------------------------------------------------------------------------
{
    if( m_ppHistogramBuffer )
    {
        for( int channel = 0; channel < m_ChannelCount; channel++ )
        {
            delete [] m_ppHistogramBuffer[channel];
        }
        delete [] m_ppHistogramBuffer;
        m_ppHistogramBuffer = 0;
    }
    delete [] m_pHistogramAverage;
    m_pHistogramAverage = 0;
    delete [] m_pStandardDeviation;
    m_pStandardDeviation = 0;
    delete [] m_pHistogramMax;
    m_pHistogramMax = 0;
    CustomDealloc();
}

//-----------------------------------------------------------------------------
void HistogramCanvas::DrawHistogramLine( wxPaintDC& dc, int h, double scaleX, double scaleY, TPlane plane )
//-----------------------------------------------------------------------------
{
    int lowerStart = h - GetBorderWidth();
    dc.SetPen( *m_Pens[plane].pColour_ );
    unsigned int stepWidth = m_DrawStepWidth;
    if( stepWidth == 0 )
    {
        // '0' is auto mode, which results in no more than 256 different values will be displayed no matter which pixel format is used.
        stepWidth = static_cast<unsigned int>( static_cast<double>( ( m_valCount / 256 ) ) * ( static_cast<double>( GetPercentageToDraw() ) / 100. ) );
    }
    if( stepWidth < 1 )
    {
        stepWidth = 1;
    }
    unsigned int from, to;
    GetDrawRange( &from, &to );
    for( unsigned int i = from; ( i < to ) && ( i + stepWidth < m_valCount ); i += stepWidth )
    {
        dc.DrawLine( static_cast<int>( GetBorderWidth() + ( ( i - from ) * scaleX ) + 1 ),
                     static_cast<int>( lowerStart - ( m_ppHistogramBuffer[plane][i] * scaleY ) ),
                     static_cast<int>( GetBorderWidth() + ( ( i - from + stepWidth ) * scaleX ) + 1 ),
                     static_cast<int>( lowerStart - ( m_ppHistogramBuffer[plane][i + stepWidth] * scaleY ) ) );
    }
    dc.SetPen( wxNullPen );
}

//-----------------------------------------------------------------------------
wxString HistogramCanvas::GetGridValue( int row, int col ) const
//-----------------------------------------------------------------------------
{
    if( col == 0 )
    {
        return ( row == 0 ) ? wxT( "Pixel Value" ) : wxString::Format( wxT( "%d" ), row - 1 );
    }
    else if( m_ppHistogramBuffer && ( col > 0 ) && ( row >= 0 ) && ( col <= m_ChannelCount ) && ( static_cast<unsigned int>( row ) <= m_valCount ) )
    {
        return ( row == 0 ) ? m_Pens[col - 1].description_ : wxString::Format( GetGridValueFormatString().c_str(), m_ppHistogramBuffer[col - 1][row - 1] );
    }
    return wxEmptyString;
}

//-----------------------------------------------------------------------------
double HistogramCanvas::GetScaleX( wxCoord w ) const
//-----------------------------------------------------------------------------
{
    return static_cast<double>( w - 2 * GetBorderWidth() ) / ( static_cast<double>( m_valCount ) * ( static_cast<double>( GetPercentageToDraw() ) / 100. ) );
}

//-----------------------------------------------------------------------------
double HistogramCanvas::GetScaleY( wxCoord h ) const
//-----------------------------------------------------------------------------
{
    return static_cast<double>( h - 2 * GetBorderWidth() ) / m_CurrentMax.cnt_;
}

//-----------------------------------------------------------------------------
unsigned int HistogramCanvas::GetXMarkerParameters( unsigned int& from, unsigned int& to ) const
//-----------------------------------------------------------------------------
{
    GetDrawRange( &from, &to );
    return GetXMarkerStepWidth( from, to );
}

//-----------------------------------------------------------------------------
bool HistogramCanvas::PrepareHistogramBuffer( bool boFormatChanged, int channelCount, mvIMPACT::acquire::TImageBufferPixelFormat pixelFormat )
//-----------------------------------------------------------------------------
{
    bool boAllocated = false;
    if( boFormatChanged || !m_ppHistogramBuffer || !m_pHistogramAverage || !m_pStandardDeviation || !m_pHistogramMax )
    {
        DeallocateHistogramBuffer();
        m_ChannelCount = channelCount;
        m_ppHistogramBuffer = new int* [channelCount];
        m_pHistogramAverage = new double[channelCount];
        m_pStandardDeviation = new double[channelCount];
        m_pHistogramMax = new PlotPoint[channelCount];
        assert( ( channelCount <= PLANE_CNT ) && wxT( "Somebody has defined more channels then this tool can handle" ) );
        for( int channel = 0; channel < channelCount; channel++ )
        {
            switch( pixelFormat )
            {
            case ibpfMono10:
            case ibpfBGR101010Packed_V2:
            case ibpfRGB101010Packed:
            case ibpfYUV422_10Packed:
            case ibpfYUV422_UYVY_10Packed:
            case ibpfYUV444_UYV_10Packed:
            case ibpfYUV444_10Packed:
                m_ppHistogramBuffer[channel] = new int[VAL_COUNT_10_BIT];
                m_valCount = VAL_COUNT_10_BIT;
                break;
            case ibpfMono12:
            case ibpfMono12Packed_V1:
            case ibpfMono12Packed_V2:
            case ibpfRGB121212Packed:
                m_ppHistogramBuffer[channel] = new int[VAL_COUNT_12_BIT];
                m_valCount = VAL_COUNT_12_BIT;
                break;
            case ibpfMono14:
            case ibpfRGB141414Packed:
                m_ppHistogramBuffer[channel] = new int[VAL_COUNT_14_BIT];
                m_valCount = VAL_COUNT_14_BIT;
                break;
            case ibpfMono16:
            case ibpfRGB161616Packed:
                m_ppHistogramBuffer[channel] = new int[VAL_COUNT_16_BIT];
                m_valCount = VAL_COUNT_16_BIT;
                break;
            default:
                m_ppHistogramBuffer[channel] = new int[VAL_COUNT_8_BIT];
                m_valCount = VAL_COUNT_8_BIT;
                break;
            }
        }
        CustomAlloc( channelCount );
        SetupNumericalDisplay( m_ChannelCount, GetRowCountNeededForNumericalDisplay() );
        boAllocated = true;
    }

    for( int channel = 0; channel < channelCount; channel++ )
    {
        memset( m_ppHistogramBuffer[channel], 0, m_valCount * sizeof( int ) );
        m_pHistogramAverage[channel] = 0;
        m_pStandardDeviation[channel] = 0;
        m_pHistogramMax[channel] = PlotPoint();
    }

    return boAllocated;
}

//-----------------------------------------------------------------------------
void HistogramCanvas::OnPaintCustom( wxPaintDC& dc )
//-----------------------------------------------------------------------------
{
    wxCoord xOffset( 1 ), yOffset( 1 ), w( 0 ), h( 0 );
    dc.GetSize( &w, &h );
    const double scaleX = GetScaleX( w );
    const double scaleY = GetScaleY( h );
    DrawMarkerLines( dc, w, h, scaleX );
    if( m_pHistogramAverage && m_pHistogramMax )
    {
        if( m_boUnsupportedPixelFormat )
        {
            dc.SetTextForeground( *wxRED );
            dc.DrawText( wxString( wxT( "Unsupported pixel format" ) ), xOffset, yOffset );
        }
        else if( m_ppHistogramBuffer )
        {
            for( int channel = 0; channel < m_ChannelCount; channel++ )
            {
                DrawHistogramLine( dc, h, scaleX, scaleY, static_cast<TPlane>( channel ) );
                wxString completeDescString = wxString::Format( wxT( "%s%s(%.2f, %5d/%5d" ), ( channel != 0 ) ? wxT( ", " ) : wxT( "" ), m_Pens[channel].description_.c_str(), m_pHistogramAverage[channel], m_pHistogramMax[channel].cnt_, m_pHistogramMax[channel].val_ );
                if( m_pStandardDeviation && m_boHandleStandardDeviation )
                {
                    completeDescString.Append( wxString::Format( wxT( ", %.2f" ), m_pStandardDeviation[channel] ) );
                }
                completeDescString.Append( wxT( "): " ) );
                DrawInfoString( dc, completeDescString, xOffset, yOffset, *( m_Pens[channel].pColour_ ) );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvas::RefreshAnalysisData( void )
//-----------------------------------------------------------------------------
{
    CalculateMaxValue();
}

//-----------------------------------------------------------------------------
void HistogramCanvas::SetDrawStepWidth( int value )
//-----------------------------------------------------------------------------
{
    if( ( value < 0 ) || ( value > GetDrawStepWidthMax() ) )
    {
        return;
    }
    AssignDisplayParameter( m_DrawStepWidth, value );
}

//-----------------------------------------------------------------------------
void HistogramCanvas::SetupNumericalDisplay( int channelCount, int rowCountNeeded )
//-----------------------------------------------------------------------------
{
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

        m_pNumericalDisplay->SetCellValue( 0, 0, wxT( "Pixel Value:" ) );
        for( int i = 0; i < channelCount; i++ )
        {
            m_pNumericalDisplay->SetCellBackgroundColour( 0, i + 1, *m_Pens[i].pColour_ );
            m_pNumericalDisplay->SetCellTextColour( 0, i + 1, *wxWHITE );
            m_pNumericalDisplay->SetCellValue( 0, i + 1, m_Pens[i].description_ );
        }

        int rowCount = m_pNumericalDisplay->GetTable()->GetNumberRows();
        if( rowCount < rowCountNeeded )
        {
            m_pNumericalDisplay->AppendRows( rowCountNeeded - rowCount );
        }
        else if( rowCount > rowCountNeeded )
        {
            m_pNumericalDisplay->DeleteRows( rowCount - 1, rowCount - rowCountNeeded );
        }
    }
}
