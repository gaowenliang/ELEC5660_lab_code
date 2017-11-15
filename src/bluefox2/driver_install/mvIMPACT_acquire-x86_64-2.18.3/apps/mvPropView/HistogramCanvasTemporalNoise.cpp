//-----------------------------------------------------------------------------
#include "HistogramCanvasTemporalNoise.h"
#ifdef _OPENMP
#   include <omp.h>
#endif // #ifdef _OPENMP

//=============================================================================
//================= Implementation HistogramCanvasTemporalNoise ===============
//=============================================================================
//-----------------------------------------------------------------------------
HistogramCanvasTemporalNoise::HistogramCanvasTemporalNoise( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "Temporal Noise Histogram" */, bool boActive /* = true */ )
    : HistogramCanvas( parent, wxT( "TemporalNoiseHistogram" ), id, pos, size, style, name, boActive, true ), m_lastImage( 1 )
//-----------------------------------------------------------------------------
{
    for( int i = 0; i < PLANE_CNT; i++ )
    {
        m_Pens[i].description_ = wxString::Format( wxT( "C%d" ), i );
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvasTemporalNoise::CalculateTemporalNoiseHistogram( const RequestData& data )
//-----------------------------------------------------------------------------
{
    if( ( m_lastImage.getBuffer()->iWidth > 0 ) &&
        ( m_lastImage.getBuffer()->iHeight > 0 ) )
    {
        const ImageBuffer* pIBNew = data.image_.getBuffer();
        const ImageBuffer* pIBOld = m_lastImage.getBuffer();
        switch( pIBNew->pixelFormat )
        {
        case ibpfMono8:
            if( m_bayerParity == bmpUndefined )
            {
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned char* pDataNew = static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel );
                    unsigned char* pDataOld = static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        unsigned char difference = abs( pDataNew[pixel] - pDataOld[pixel] );
                        m_pHistogramAverage[pRed] += difference;
                        ++m_ppHistogramBuffer[pRed][difference];
                    }
                }
            }
            else
            {
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned char* pDataNew = static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel );
                    unsigned char* pDataOld = static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                        unsigned char difference = abs( pDataNew[pixel] - pDataOld[pixel] );
                        m_pHistogramAverage[index] += difference;
                        ++m_ppHistogramBuffer[index][difference];
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
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned short* pDataNew = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel ) );
                    unsigned short* pDataOld = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel ) );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        unsigned short value = saveAssign( static_cast<unsigned short>( abs( pDataNew[pixel] - pDataOld[pixel] ) ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                        m_pHistogramAverage[pRed] += value;
                        ++m_ppHistogramBuffer[pRed][value];
                    }
                }
            }
            else
            {
                for( int line = 0; line < m_AOIh; line++ )
                {
                    unsigned short* pDataNew = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel ) );
                    unsigned short* pDataOld = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel ) );
                    for( int pixel = 0; pixel < m_AOIw; pixel++ )
                    {
                        unsigned short value = saveAssign( static_cast<unsigned short>( abs( pDataNew[pixel] - pDataOld[pixel] ) ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                        int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                        m_pHistogramAverage[index] += value;
                        ++m_ppHistogramBuffer[index][value];
                    }
                }
            }
            break;
        case ibpfMono12Packed_V1:
            CalculateTemporalNoiseHistogram_MonoPacked( pIBNew, pIBOld, GetMono12Packed_V1Pixel );
            break;
        case ibpfMono12Packed_V2:
            CalculateTemporalNoiseHistogram_MonoPacked( pIBNew, pIBOld, GetMono12Packed_V2Pixel );
            break;
        case ibpfBGR888Packed:
            {
                int order[3] = { 2, 1, 0 };
                CalculateTemporalNoiseHistogram_8u_RGBPacked( pIBNew, pIBOld, order );
            }
            break;
        case ibpfRGBx888Packed:
        case ibpfRGB888Packed:
            {
                int order[3] = { 0, 1, 2 };
                CalculateTemporalNoiseHistogram_8u_RGBPacked( pIBNew, pIBOld, order );
            }
            break;
        case ibpfBGR101010Packed_V2:
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short redNew, greenNew, blueNew;
                unsigned int* pDataNew =  reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[0].iLinePitch ) ) + m_AOIx;
                unsigned short redOld, greenOld, blueOld;
                unsigned int* pDataOld =  reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[0].iLinePitch ) ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    GetBGR101010Packed_V2Pixel( *pDataNew++, redNew, greenNew, blueNew );
                    GetBGR101010Packed_V2Pixel( *pDataOld++, redOld, greenOld, blueOld );
                    unsigned short difference = abs( blueNew - blueOld );
                    m_pHistogramAverage[pBlue] += difference;
                    ++m_ppHistogramBuffer[pBlue][difference];
                    difference = abs( greenNew - greenOld );
                    m_pHistogramAverage[pGreen] += difference;
                    ++m_ppHistogramBuffer[pGreen][difference];
                    difference = abs( redNew - redOld );
                    m_pHistogramAverage[pRed] += difference;
                    ++m_ppHistogramBuffer[pRed][difference];
                }
            }
            break;
        case ibpfRGB101010Packed:
        case ibpfRGB121212Packed:
        case ibpfRGB141414Packed:
        case ibpfRGB161616Packed:
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short* pDataNew = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel ) );
                unsigned short* pDataOld = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel ) );
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    unsigned short value = saveAssign( static_cast<unsigned short>( abs( *pDataNew - *pDataOld ) ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                    m_pHistogramAverage[pBlue] += value;
                    ++m_ppHistogramBuffer[pBlue][value];
                    ++pDataNew;
                    ++pDataOld;
                    value = saveAssign( static_cast<unsigned short>( abs( *pDataNew - *pDataOld ) ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                    m_pHistogramAverage[pGreen] += value;
                    ++m_ppHistogramBuffer[pGreen][value];
                    ++pDataNew;
                    ++pDataOld;
                    value = saveAssign( static_cast<unsigned short>( abs( *pDataNew - *pDataOld ) ), static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                    m_pHistogramAverage[pRed] += value;
                    ++m_ppHistogramBuffer[pRed][value];
                    ++pDataNew;
                    ++pDataOld;
                }
            }
            break;
        case ibpfRGB888Planar:
        case ibpfRGBx888Planar:
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned char* prNew = static_cast<unsigned char*>( pIBNew->vpData ) + pIBNew->pChannels[pRed].iChannelOffset + ( ( m_AOIy + line ) * pIBNew->pChannels[pRed].iLinePitch ) + m_AOIx;
                unsigned char* pgNew = static_cast<unsigned char*>( pIBNew->vpData ) + pIBNew->pChannels[pGreen].iChannelOffset + ( ( m_AOIy + line ) * pIBNew->pChannels[pGreen].iLinePitch ) + m_AOIx;
                unsigned char* pbNew = static_cast<unsigned char*>( pIBNew->vpData ) + pIBNew->pChannels[pBlue].iChannelOffset + ( ( m_AOIy + line ) * pIBNew->pChannels[pBlue].iLinePitch ) + m_AOIx;
                unsigned char* prOld = static_cast<unsigned char*>( pIBOld->vpData ) + pIBOld->pChannels[pRed].iChannelOffset + ( ( m_AOIy + line ) * pIBOld->pChannels[pRed].iLinePitch ) + m_AOIx;
                unsigned char* pgOld = static_cast<unsigned char*>( pIBOld->vpData ) + pIBOld->pChannels[pGreen].iChannelOffset + ( ( m_AOIy + line ) * pIBOld->pChannels[pGreen].iLinePitch ) + m_AOIx;
                unsigned char* pbOld = static_cast<unsigned char*>( pIBOld->vpData ) + pIBOld->pChannels[pBlue].iChannelOffset + ( ( m_AOIy + line ) * pIBOld->pChannels[pBlue].iLinePitch ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    unsigned char difference = abs( *prNew++ - *prOld++ );
                    m_pHistogramAverage[pRed] += difference;
                    ++m_ppHistogramBuffer[pRed][difference];
                    difference = abs( *pgNew++ - *pgOld++ );
                    m_pHistogramAverage[pGreen] += difference;
                    ++m_ppHistogramBuffer[pGreen][difference];
                    difference = abs( *pbNew++ - *pbOld++ );
                    m_pHistogramAverage[pBlue] += difference;
                    ++m_ppHistogramBuffer[pBlue][difference];
                }
            }
            break;
        case ibpfYUV411_UYYVYY_Packed:
            CalculateTemporalNoiseHistogramYUV411_UYYVYY<unsigned char>( pIBNew, pIBOld );
            break;
        case ibpfYUV422Packed:
            CalculateTemporalNoiseHistogramYUV422<unsigned char>( pIBNew, pIBOld );
            break;
        case ibpfYUV422_10Packed:
            CalculateTemporalNoiseHistogramYUV422<unsigned short>( pIBNew, pIBOld );
            break;
        case ibpfYUV422_UYVYPacked:
            CalculateTemporalNoiseHistogramUYV422<unsigned char>( pIBNew, pIBOld );
            break;
        case ibpfYUV422_UYVY_10Packed:
            CalculateTemporalNoiseHistogramUYV422<unsigned short>( pIBNew, pIBOld );
            break;
        case ibpfYUV444_UYVPacked:
            CalculateTemporalNoiseHistogramYUV444<unsigned char>( pIBNew, pIBOld );
            break;
        case ibpfYUV444_UYV_10Packed:
            CalculateTemporalNoiseHistogramYUV444<unsigned short>( pIBNew, pIBOld );
            break;
        case ibpfYUV444Packed:
            CalculateTemporalNoiseHistogramYUV444<unsigned char>( pIBNew, pIBOld );
            break;
        case ibpfYUV444_10Packed:
            CalculateTemporalNoiseHistogramYUV444<unsigned short>( pIBNew, pIBOld );
            break;
        case ibpfYUV422Planar:
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned char* pyNew = static_cast<unsigned char*>( pIBNew->vpData ) + pIBNew->pChannels[pY].iChannelOffset + ( ( m_AOIy + line ) * pIBNew->pChannels[pY].iLinePitch ) + m_AOIx;
                unsigned char* puNew = static_cast<unsigned char*>( pIBNew->vpData ) + pIBNew->pChannels[pU].iChannelOffset + ( ( m_AOIy + line ) * pIBNew->pChannels[pU].iLinePitch ) + m_AOIx;
                unsigned char* pvNew = static_cast<unsigned char*>( pIBNew->vpData ) + pIBNew->pChannels[pV].iChannelOffset + ( ( m_AOIy + line ) * pIBNew->pChannels[pV].iLinePitch ) + m_AOIx;
                unsigned char* pyOld = static_cast<unsigned char*>( pIBOld->vpData ) + pIBOld->pChannels[pY].iChannelOffset + ( ( m_AOIy + line ) * pIBOld->pChannels[pY].iLinePitch ) + m_AOIx;
                unsigned char* puOld = static_cast<unsigned char*>( pIBOld->vpData ) + pIBOld->pChannels[pU].iChannelOffset + ( ( m_AOIy + line ) * pIBOld->pChannels[pU].iLinePitch ) + m_AOIx;
                unsigned char* pvOld = static_cast<unsigned char*>( pIBOld->vpData ) + pIBOld->pChannels[pV].iChannelOffset + ( ( m_AOIy + line ) * pIBOld->pChannels[pV].iLinePitch ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    unsigned char difference = abs( *pyNew++ - *pyOld++ );
                    m_pHistogramAverage[pY] += difference;
                    ++m_ppHistogramBuffer[pY][difference];
                    difference = abs( *puNew - *puOld );
                    m_pHistogramAverage[pU] += difference;
                    ++m_ppHistogramBuffer[pU][difference];
                    difference = abs( *pvNew - *pvOld );
                    m_pHistogramAverage[pV] += difference;
                    ++m_ppHistogramBuffer[pV][difference];
                    if( pixel & 1 )
                    {
                        ++puNew;
                        ++pvNew;
                        ++puOld;
                        ++pvOld;
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
    m_lastImage = data.image_.clone();
}

//-----------------------------------------------------------------------------
void HistogramCanvasTemporalNoise::CalculateTemporalNoiseHistogram_8u_RGBPacked( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld, int order[3] )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < m_AOIh; line++ )
    {
        unsigned char* pDataNew = static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel );
        unsigned char* pDataOld = static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel );
        for( int pixel = 0; pixel < m_AOIw; pixel++ )
        {
            unsigned char difference = abs( pDataNew[order[0]] - pDataOld[order[0]] );
            m_pHistogramAverage[pBlue] += difference;
            ++m_ppHistogramBuffer[pBlue][difference];
            difference = abs( pDataNew[order[1]] - pDataOld[order[1]] );
            m_pHistogramAverage[pGreen] += difference;
            ++m_ppHistogramBuffer[pGreen][difference];
            difference = abs( pDataNew[order[2]] - pDataOld[order[2]] );
            m_pHistogramAverage[pRed] += difference;
            ++m_ppHistogramBuffer[pRed][difference];
            pDataNew += pIBNew->iBytesPerPixel;
            pDataOld += pIBOld->iBytesPerPixel;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvasTemporalNoise::CalculateTemporalNoiseHistogram_MonoPacked( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld, _Ty pixelAccessFn )
//-----------------------------------------------------------------------------
{
    if( m_bayerParity == bmpUndefined )
    {
        for( int line = 0; line < m_AOIh; line++ )
        {
            const int pixOffset = ( m_AOIy + line ) * pIBNew->iWidth;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short valueNew = pixelAccessFn( static_cast<unsigned char*>( pIBNew->vpData ), pixOffset + m_AOIx + pixel );
                unsigned short valueOld = pixelAccessFn( static_cast<unsigned char*>( pIBOld->vpData ), pixOffset + m_AOIx + pixel );
                unsigned short difference = abs( valueNew - valueOld );
                m_pHistogramAverage[pRed] += difference;
                ++m_ppHistogramBuffer[pRed][difference];
            }
        }
    }
    else
    {
        for( int line = 0; line < m_AOIh; line++ )
        {
            const int pixOffset = ( m_AOIy + line ) * pIBNew->iWidth;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short valueNew = pixelAccessFn( static_cast<unsigned char*>( pIBNew->vpData ), pixOffset + m_AOIx + pixel );
                unsigned short valueOld = pixelAccessFn( static_cast<unsigned char*>( pIBOld->vpData ), pixOffset + m_AOIx + pixel );
                unsigned short difference = abs( valueNew - valueOld );
                int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                m_pHistogramAverage[index] += difference;
                ++m_ppHistogramBuffer[index][difference];
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvasTemporalNoise::CalculateTemporalNoiseHistogramUYV422( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pDataNew = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pY].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel ) );
        _Ty* pDataOld = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pY].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel ) );
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            _Ty difference = 0;
            if( pixel % 2 )
            {
                difference = static_cast<_Ty>( abs( *pDataNew++ - *pDataOld++ ) );
                m_pHistogramAverage[pChannel2] += difference;
                ++m_ppHistogramBuffer[pChannel2][difference];
                m_pHistogramAverage[pChannel2] += difference;
                ++m_ppHistogramBuffer[pChannel2][difference];
            }
            else
            {
                difference = static_cast<_Ty>( abs( *pDataNew++ - *pDataOld++ ) );
                m_pHistogramAverage[pChannel0] += difference;
                ++m_ppHistogramBuffer[pChannel0][difference];
                m_pHistogramAverage[pChannel0] += difference;
                ++m_ppHistogramBuffer[pChannel0][difference];
            }
            difference = static_cast<_Ty>( abs( *pDataNew++ - *pDataOld++ ) );
            m_pHistogramAverage[pChannel1] += difference;
            ++m_ppHistogramBuffer[pChannel1][difference];
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvasTemporalNoise::CalculateTemporalNoiseHistogramYUV411_UYYVYY( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld )
//-----------------------------------------------------------------------------
{
    ProcessingHelperYUV411_UYYVYY ph( m_AOIx );
#ifdef _OPENMP
    #pragma omp parallel for
#endif // #ifdef _OPENMP
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pDataNew = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pY].iLinePitch ) ) + ph.GetOffsetToFirstY();
        _Ty* pDataOld = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pY].iLinePitch ) ) + ph.GetOffsetToFirstY();
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            _Ty difference = static_cast<_Ty>( abs( *pDataNew - *pDataOld ) );
            m_pHistogramAverage[pChannel1] += difference;
            ++m_ppHistogramBuffer[pChannel1][difference];
            ph.RefreshUAndVOffsets( pixel );
            difference = static_cast<_Ty>( abs( pDataNew[ph.GetUOffset()] - pDataOld[ph.GetUOffset()] ) );
            m_pHistogramAverage[pChannel0] += difference;
            ++m_ppHistogramBuffer[pChannel0][difference];
            difference = static_cast<_Ty>( abs( pDataNew[ph.GetVOffset()] - pDataOld[ph.GetVOffset()] ) );
            m_pHistogramAverage[pChannel2] += difference;
            ++m_ppHistogramBuffer[pChannel2][difference];
            pDataNew += ( pixel % 2 ) ? 2 : 1;
            pDataOld += ( pixel % 2 ) ? 2 : 1;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvasTemporalNoise::CalculateTemporalNoiseHistogramYUV422( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pDataNew = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pY].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel ) );
        _Ty* pDataOld = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pY].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel ) );
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            _Ty difference = static_cast<_Ty>( abs( *pDataNew++ - *pDataOld++ ) );
            m_pHistogramAverage[pY] += difference;
            ++m_ppHistogramBuffer[pY][difference];
            if( pixel % 2 )
            {
                difference = static_cast<_Ty>( abs( *pDataNew++ - *pDataOld++ ) );
                m_pHistogramAverage[pV] += difference;
                ++m_ppHistogramBuffer[pV][difference];
                m_pHistogramAverage[pV] += difference;
                ++m_ppHistogramBuffer[pV][difference];
            }
            else
            {
                difference = static_cast<_Ty>( abs( *pDataNew++ - *pDataOld++ ) );
                m_pHistogramAverage[pU] += difference;
                ++m_ppHistogramBuffer[pU][difference];
                m_pHistogramAverage[pU] += difference;
                ++m_ppHistogramBuffer[pU][difference];
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void HistogramCanvasTemporalNoise::CalculateTemporalNoiseHistogramYUV444( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pDataNew = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBNew->vpData ) + ( ( m_AOIy + line ) * pIBNew->pChannels[pY].iLinePitch ) + ( m_AOIx * pIBNew->iBytesPerPixel ) );
        _Ty* pDataOld = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIBOld->vpData ) + ( ( m_AOIy + line ) * pIBOld->pChannels[pY].iLinePitch ) + ( m_AOIx * pIBOld->iBytesPerPixel ) );
        const int LOOP_END = m_AOIx + m_AOIw;
        for( int pixel = m_AOIx; pixel < LOOP_END; pixel++ )
        {
            _Ty difference = static_cast<_Ty>( abs( pDataNew[0] - pDataOld[0] ) );
            m_pHistogramAverage[0] += difference;
            ++m_ppHistogramBuffer[0][difference];
            difference = static_cast<_Ty>( abs( pDataNew[1] - pDataOld[1] ) );
            m_pHistogramAverage[1] += difference;
            ++m_ppHistogramBuffer[1][difference];
            difference = static_cast<_Ty>( abs( pDataNew[2] - pDataOld[2] ) );
            m_pHistogramAverage[2] += difference;
            ++m_ppHistogramBuffer[2][difference];
            pDataNew += 3;
            pDataOld += 3;
        }
    }
}

//-----------------------------------------------------------------------------
void HistogramCanvasTemporalNoise::CustomAlloc( int channelCount )
//-----------------------------------------------------------------------------
{
    m_lastImage = ImageBufferDesc( channelCount );
    m_lastImage.getBuffer()->pixelFormat = ibpfMono8;
    m_lastImage.getBuffer()->iWidth = 0;
    m_lastImage.getBuffer()->iHeight = 0;
}

//-----------------------------------------------------------------------------
void HistogramCanvasTemporalNoise::RefreshData( const RequestData& data, int x /* = -1 */, int y /* = -1 */, int w /* = -1 */, int h /* = -1 */, bool boForceRefresh /* = false */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );

    bool boImageFormatChanged = false;
    const ImageBuffer* pIB = data.image_.getBuffer();
    const TBayerMosaicParity bayerParity = GetProcessBayerParity() ? data.bayerParity_ : bmpUndefined;
    const int channelCount = ( bayerParity != bmpUndefined ) ? 4 : pIB->iChannelCount;
    if( !MustUpdate( pIB, channelCount, x, y, w, h, boForceRefresh, 0, &boImageFormatChanged ) )
    {
        return;
    }

    if( boImageFormatChanged || ( m_bayerParity != bayerParity ) )
    {
        SetDefaultPens( pIB->pixelFormat );
        m_bayerParity = bayerParity;
    }

    if( !PrepareHistogramBuffer( boImageFormatChanged, channelCount, pIB->pixelFormat ) )
    {
        if( ( ( m_lastImage.getBuffer()->iWidth > 0 ) && ( m_lastImage.getBuffer()->iWidth != pIB->iWidth ) ) ||
            ( ( m_lastImage.getBuffer()->iHeight > 0 ) && ( m_lastImage.getBuffer()->iHeight != pIB->iHeight ) ) )
        {
            CustomAlloc( pIB->iChannelCount );
        }
    }
    CalculateTemporalNoiseHistogram( data );
    CalculateMaxValue();
    UpdateAnalysisOutput( boImageFormatChanged );
}
