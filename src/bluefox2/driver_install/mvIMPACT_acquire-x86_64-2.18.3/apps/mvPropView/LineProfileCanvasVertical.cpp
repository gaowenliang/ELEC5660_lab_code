#include "LineProfileCanvasVertical.h"

//=============================================================================
//============== Implementation LineProfileCanvasVertical =====================
//=============================================================================
//-----------------------------------------------------------------------------
LineProfileCanvasVertical::LineProfileCanvasVertical( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "Vertical Line Profile" */, bool boActive /* = false */ )
    : LineProfileCanvas( parent, wxT( "VerticalLineProfile" ), id, pos, size, style, name, boActive )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
void LineProfileCanvasVertical::CalculateData( const ImageBuffer* pIB, const TBayerMosaicParity bayerParity )
//-----------------------------------------------------------------------------
{
    switch( pIB->pixelFormat )
    {
    case ibpfMono8:
        if( bayerParity == bmpUndefined )
        {
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    m_ppData[pRed][line] += *pData++;
                }
            }
        }
        else
        {
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                    m_ppData[index][line / 2] += *pData++;
                }
            }
        }
        break;
    case ibpfMono10:
    case ibpfMono12:
    case ibpfMono14:
    case ibpfMono16:
        if( bayerParity == bmpUndefined )
        {
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short* pData = static_cast<unsigned short*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->iWidth ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    m_ppData[pRed][line] += saveAssign( *pData++, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                }
            }
        }
        else
        {
            for( int line = 0; line < m_AOIh; line++ )
            {
                unsigned short* pData = static_cast<unsigned short*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->iWidth ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel++ )
                {
                    int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                    m_ppData[index][line / 2] += saveAssign( *pData++, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                }
            }
        }
        break;
    case ibpfMono12Packed_V1:
        ProcessMonoPackedData( pIB, bayerParity, GetMono12Packed_V1Pixel );
        break;
    case ibpfMono12Packed_V2:
        ProcessMonoPackedData( pIB, bayerParity, GetMono12Packed_V2Pixel );
        break;
    case ibpfRGBx888Packed:
        {
            int order[3] = { 0, 1, 2 };
            ProcessRGB_8u_CxData( pIB, 4, order );
        }
        break;
    case ibpfBGR888Packed:
        {
            int order[3] = { 2, 1, 0 };
            ProcessRGB_8u_CxData( pIB, 3, order );
        }
        break;
    case ibpfRGB888Packed:
        {
            int order[3] = { 0, 1, 2 };
            ProcessRGB_8u_CxData( pIB, 3, order );
        }
        break;
    case ibpfBGR101010Packed_V2:
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned short red, green, blue;
            unsigned int* p =  reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                GetBGR101010Packed_V2Pixel( *p++, red, green, blue );
                m_ppData[pBlue][line] += blue;
                m_ppData[pGreen][line] += green;
                m_ppData[pRed][line] += red;
            }
        }
        break;
    case ibpfRGB101010Packed:
    case ibpfRGB121212Packed:
    case ibpfRGB141414Packed:
    case ibpfRGB161616Packed:
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                m_ppData[pBlue][line] += saveAssign( *pData++, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                m_ppData[pGreen][line] += saveAssign( *pData++, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
                m_ppData[pRed][line] += saveAssign( *pData++, static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_valCount - 1 ) );
            }
        }
        break;
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned char* pr = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pRed].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + m_AOIx;
            unsigned char* pg = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pGreen].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pGreen].iLinePitch ) + m_AOIx;
            unsigned char* pb = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pBlue].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pBlue].iLinePitch ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                m_ppData[pRed][line] += *pr++;
                m_ppData[pGreen][line] += *pg++;
                m_ppData[pBlue][line] += *pb++;
            }
        }
        break;
    case ibpfYUV411_UYYVYY_Packed:
        ProcessYUV411_UYYVYYData<unsigned char>( pIB );
        break;
    case ibpfYUV422Packed:
        ProcessYUV422Data<unsigned char>( pIB );
        break;
    case ibpfYUV422_10Packed:
        ProcessYUV422Data<unsigned short>( pIB );
        break;
    case ibpfYUV422_UYVYPacked:
        ProcessUYV422Data<unsigned char>( pIB );
        break;
    case ibpfYUV422_UYVY_10Packed:
        ProcessUYV422Data<unsigned short>( pIB );
        break;
    case ibpfYUV444_UYVPacked:
        ProcessYUV444Data<unsigned char>( pIB );
        break;
    case ibpfYUV444_UYV_10Packed:
        ProcessYUV444Data<unsigned short>( pIB );
        break;
    case ibpfYUV444Packed:
        ProcessYUV444Data<unsigned char>( pIB );
        break;
    case ibpfYUV444_10Packed:
        ProcessYUV444Data<unsigned short>( pIB );
        break;
    case ibpfYUV422Planar:
        for( int line = 0; line < m_AOIh; line++ )
        {
            unsigned char* py = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pY].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + m_AOIx;
            unsigned char* pu = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pU].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pU].iLinePitch ) + m_AOIx;
            unsigned char* pv = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pV].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pV].iLinePitch ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                m_ppData[pY][line] += *py;
                m_ppData[pU][line] += *pu;
                m_ppData[pV][line] += *pv;
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
        // unsupported colour format
        m_boUnsupportedPixelFormat = true;
        break;
    }

    /// \todo This is not correct for bayer raw data with AOI with a AOIw or AOIh that is not diviable by 2
    for( int channel = 0; channel < m_ChannelCount; channel++ )
    {
        for( int i = 0; i < m_verDataCount; i++ )
        {
            m_ppData[channel][i] /= m_horDataCount;
        }
    }
}

//-----------------------------------------------------------------------------
void LineProfileCanvasVertical::ProcessRGB_8u_CxData( const ImageBuffer* pIB, const int inc, const int order[3] )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < m_AOIh; line++ )
    {
        unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
        for( int pixel = 0; pixel < m_AOIw; pixel++ )
        {
            m_ppData[pBlue][line] += pData[order[0]];
            m_ppData[pGreen][line] += pData[order[1]];
            m_ppData[pRed][line] += pData[order[2]];
            pData += inc;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void LineProfileCanvasVertical::ProcessMonoPackedData( const ImageBuffer* pIB, const TBayerMosaicParity bayerParity, _Ty pixelAccessFn )
//-----------------------------------------------------------------------------
{
    if( bayerParity == bmpUndefined )
    {
        for( int line = 0; line < m_AOIh; line++ )
        {
            const int pixOffset = ( m_AOIy + line ) * pIB->iWidth;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short value = pixelAccessFn( static_cast<unsigned char*>( pIB->vpData ), pixOffset + m_AOIx + pixel );
                m_ppData[pRed][line] += value;
            }
        }
    }
    else
    {
        for( int line = 0; line < m_AOIh; line++ )
        {
            const int pixOffset = ( m_AOIy + line ) * pIB->iWidth;
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                unsigned short value = pixelAccessFn( static_cast<unsigned char*>( pIB->vpData ), pixOffset + m_AOIx + pixel );
                int index = ( ( ( line + m_AOIy ) & 1 ) ? ( ( ( pixel + m_AOIx ) & 1 ) ? pOddOdd : pOddEven ) : ( ( ( pixel + m_AOIx ) & 1 ) ? pEvenOdd : pEvenEven ) );
                m_ppData[index][line / 2] += value;
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void LineProfileCanvasVertical::ProcessYUV411_UYYVYYData( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    if( pIB->iWidth > 1 )
    {
        ProcessingHelperYUV411_UYYVYY ph( m_AOIx );
        int* py = m_ppData[pChannel1];
        int* pu = m_ppData[pChannel0];
        int* pv = m_ppData[pChannel2];
        for( int line = 0; line < m_AOIh; line++ )
        {
            _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) ) + ph.GetOffsetToFirstY();
            for( int pixel = m_AOIx; pixel < m_AOIw + m_AOIx; pixel++ )
            {
                *py += *pData;
                ph.RefreshUAndVOffsets( pixel );
                *pu += pData[ph.GetUOffset()];
                *pv += pData[ph.GetVOffset()];
                pData += ( pixel % 2 ) ? 2 : 1;
            }
            py++;
            pu++;
            pv++;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void LineProfileCanvasVertical::ProcessYUV422Data( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    if( pIB->iWidth > 1 )
    {
        for( int line = 0; line < m_AOIh; line++ )
        {
            _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                m_ppData[pY][line] += *pData;
                if( ( m_AOIx + pixel ) & 1 )
                {
                    m_ppData[pV][line] += *( pData + 1 );
                    m_ppData[pU][line] += *( pData - 1 );
                }
                else
                {
                    m_ppData[pV][line] += *( pData + 3 );
                    m_ppData[pU][line] += *( pData + 1 );
                }
                pData += 2;
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void LineProfileCanvasVertical::ProcessYUV444Data( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    for( int line = 0; line < m_AOIh; line++ )
    {
        _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
        for( int pixel = 0; pixel < m_AOIw; pixel++ )
        {
            m_ppData[pChannel0][line] += pData[pChannel0];
            m_ppData[pChannel1][line] += pData[pChannel1];
            m_ppData[pChannel2][line] += pData[pChannel2];
            pData += 3;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void LineProfileCanvasVertical::ProcessUYV422Data( const ImageBuffer* pIB )
//-----------------------------------------------------------------------------
{
    if( pIB->iWidth > 1 )
    {
        for( int line = 0; line < m_AOIh; line++ )
        {
            _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel++ )
            {
                m_ppData[pChannel1][line] += *( pData + 1 );
                if( ( m_AOIx + pixel ) & 1 )
                {
                    m_ppData[pChannel2][line] += *pData;
                    m_ppData[pChannel0][line] += *( pData - 2 );
                }
                else
                {
                    m_ppData[pChannel2][line] += *( pData + 2 );
                    m_ppData[pChannel0][line] += *pData;
                }
                pData += 2;
            }
        }
    }
}
