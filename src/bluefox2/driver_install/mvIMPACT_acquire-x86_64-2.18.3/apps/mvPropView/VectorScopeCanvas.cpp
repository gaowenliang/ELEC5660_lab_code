#include "DataGrid.h"
#include "VectorScopeCanvas.h"

//#define TEST_VECTOR_SCOPE
#ifdef TEST_VECTOR_SCOPE
#   define VECTOR_SCOPE_TEST_VALUE_COUNT 1024
#endif // #ifdef TEST_VECTOR_SCOPE

//=============================================================================
//============== Implementation VectorScopeCanvas =============================
//=============================================================================
//-----------------------------------------------------------------------------
VectorScopeCanvas::VectorScopeCanvas( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                                      const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = wxT("Vector Scope") */, bool boActive /* = false */ ) :
    PlotCanvasImageAnalysis( parent, wxT( "VectorScope" ), id, pos, size, style, name, boActive, 1 ),
    m_pDataBuffer( 0 ), m_MaxPixelValue( VAL_COUNT_8_BIT )
//-----------------------------------------------------------------------------
{
    m_plotFeatures.insert( pfGridSteps );
    InitReferencePoints( 0 );
    SetGridValueFormatString( wxT( "%.3f" ) );
}
//-----------------------------------------------------------------------------
VectorScopeCanvas::~VectorScopeCanvas()
//-----------------------------------------------------------------------------
{
    DeallocateDataBuffer();
}

//-----------------------------------------------------------------------------
void VectorScopeCanvas::CreateGridCustom( void )
//-----------------------------------------------------------------------------
{
    m_pNumericalDisplay->SetTable( new DataGridTable( this, m_valCount, 3 ), true );
    m_pNumericalDisplay->SetCellBackgroundColour( 0, pCb, *wxBLUE );
    m_pNumericalDisplay->SetCellTextColour( 0, pCb, *wxWHITE );
    m_pNumericalDisplay->SetCellBackgroundColour( 0, pCr, *wxRED );
    m_pNumericalDisplay->SetCellTextColour( 0, pCr, *wxWHITE );
}

//-----------------------------------------------------------------------------
void VectorScopeCanvas::DeallocateDataBuffer( void )
//-----------------------------------------------------------------------------
{
    delete [] m_pDataBuffer;
    m_pDataBuffer = 0;
}

//-----------------------------------------------------------------------------
wxString VectorScopeCanvas::GetGridValue( int row, int col ) const
//-----------------------------------------------------------------------------
{
    if( col == 0 )
    {
        return ( row == 0 ) ? wxT( "Data Point in AOI" ) : wxString::Format( wxT( "%d" ), row - 1 );
    }
    else if( ( row >= 0 ) && ( col > 0 ) )
    {
        if( row == 0 )
        {
            switch( col - 1 )
            {
            case 0:
                return wxString( wxT( "Cb" ) );
            case 1:
                return wxString( wxT( "Cr" ) );
            default:
                wxASSERT_MSG( false, wxT( "Invalid colum count" ) );
                break;
            }
        }
        else if( m_pDataBuffer && ( static_cast<unsigned int>( row - 1 ) < m_valCount ) )
        {
            switch( col - 1 )
            {
            case 0:
                return wxString::Format( GetGridValueFormatString().c_str(), m_pDataBuffer[row - 1].Cb_ );
            case 1:
                return wxString::Format( GetGridValueFormatString().c_str(), m_pDataBuffer[row - 1].Cr_ );
            default:
                wxASSERT_MSG( false, wxT( "Invalid colum count" ) );
                break;
            }
        }
    }
    return wxEmptyString;
}

//-----------------------------------------------------------------------------
void VectorScopeCanvas::InitReferencePoints( int bitShift )
//-----------------------------------------------------------------------------
{
    m_ReferencePoints[rcRed    ].Set( 255 << bitShift,   0            ,   0            , bitShift );
    m_ReferencePoints[rcYellow ].Set( 255 << bitShift, 255 << bitShift,   0            , bitShift );
    m_ReferencePoints[rcGreen  ].Set(   0            , 255 << bitShift,   0            , bitShift );
    m_ReferencePoints[rcCyan   ].Set(   0            , 255 << bitShift, 255 << bitShift, bitShift );
    m_ReferencePoints[rcBlue   ].Set(   0            ,   0            , 255 << bitShift, bitShift );
    m_ReferencePoints[rcMagenta].Set( 255 << bitShift,   0            , 255 << bitShift, bitShift );
}

//-----------------------------------------------------------------------------
void VectorScopeCanvas::OnPaintCustom( wxPaintDC& dc )
//-----------------------------------------------------------------------------
{
    if( !m_pDataBuffer )
    {
        return;
    }

    wxCoord w( 0 ), h( 0 );
    dc.GetSize( &w, &h );
    const int RADIUS = ( ( ( w < h ) ? w : h ) - GetBorderWidth() ) / 2;
    const double scaleFactor = static_cast<double>( 2 * RADIUS ) / ( 1.0554 * static_cast<double>( m_MaxPixelValue ) );
    const wxPoint CENTER( w / 2, h / 2 );

    if( m_boUnsupportedPixelFormat )
    {
        dc.SetBrush( *wxBLACK_BRUSH );
        dc.DrawRectangle( 0, 0, w, h );
        dc.SetTextForeground( *wxRED );
        dc.DrawText( wxString( wxT( "Unsupported pixel format" ) ), CENTER.x, CENTER.y );
    }
    else
    {
        wxImage pixelCloud( w, h );
        for( unsigned int i = 0; i < m_valCount; i++ )
        {
            pixelCloud.SetRGB( CENTER.x + static_cast<int>( m_pDataBuffer[i].Cb_ * scaleFactor ),
                               CENTER.y - static_cast<int>( m_pDataBuffer[i].Cr_ * scaleFactor ),
                               m_pDataBuffer[i].colour_.Red(),
                               m_pDataBuffer[i].colour_.Green(),
                               m_pDataBuffer[i].colour_.Blue() );
        }
        wxMemoryDC memDC;
        wxBitmap pixelBitmap( pixelCloud, -1 );
        memDC.SelectObject( pixelBitmap );
        dc.Blit( 0, 0, w, h, &memDC, 0, 0 );
    }

    dc.SetBrush( *wxTRANSPARENT_BRUSH );
    const wxCoord rectSize( 6 );
    for( int i = 0; i < rcLAST; i++ )
    {
        dc.SetPen( wxColour( m_ReferencePoints[i].colour_ ) );
        int xOffsetFromCenter = static_cast<int>( m_ReferencePoints[i].Cb_ * scaleFactor );
        int yOffsetFromCenter = static_cast<int>( m_ReferencePoints[i].Cr_ * scaleFactor );
        dc.DrawRectangle( CENTER.x + xOffsetFromCenter - ( rectSize / 2 ),
                          CENTER.y - yOffsetFromCenter - ( rectSize / 2 ),
                          rectSize,
                          rectSize );
    }
    dc.SetPen( wxColour( 128, 128, 128 ) );
    dc.DrawCircle( CENTER, RADIUS );
    dc.DrawCircle( CENTER, 3 * RADIUS / 4 );
    dc.DrawCircle( CENTER, RADIUS / 2 );
    dc.DrawCircle( CENTER, RADIUS / 4 );
}

//-----------------------------------------------------------------------------
void VectorScopeCanvas::PrepareDataBuffer( bool boAOIChanged )
//-----------------------------------------------------------------------------
{
    unsigned int newValCount = ( ( m_AOIw + m_GridStepsX - 1 ) / m_GridStepsX ) * ( ( m_AOIh + m_GridStepsY - 1 ) / m_GridStepsY );
    if( boAOIChanged || !m_pDataBuffer || ( newValCount != m_valCount ) )
    {
        DeallocateDataBuffer();
#ifndef TEST_VECTOR_SCOPE
        m_valCount = newValCount;
#else
        m_valCount = VECTOR_SCOPE_TEST_VALUE_COUNT;
#endif // #ifndef TEST_VECTOR_SCOPE
        m_pDataBuffer = new DataPoint[m_valCount];
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void VectorScopeCanvas::ProcessMonoPackedData( const ImageBuffer* pIB, DataPoint* p, const int bitShift, _Ty pixelAccessFn )
//-----------------------------------------------------------------------------
{
    // no colour information, thus it doesn't make much sense to calculate all the information as the one
    // and only point drawn will be in the center of the plot using the intensity of the last pixel calculated.
    // However no one shall be able to say we are not trying hard.
    for( int line = 0; line < m_AOIh; line += m_GridStepsY )
    {
        const int pixOffset = ( m_AOIy + line ) * pIB->iWidth;
        for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
        {
            unsigned short value = pixelAccessFn( static_cast<unsigned char*>( pIB->vpData ), pixOffset + m_AOIx + m_AOIw );
            p->Set( value, value, value, bitShift );
            ++p;
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void VectorScopeCanvas::ProcessUYV422Data( const ImageBuffer* pIB, DataPoint* p, const int bitShift )
//-----------------------------------------------------------------------------
{
    if( pIB->iWidth > 1 )
    {
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                if( ( m_AOIx + pixel ) & 1 )
                {
                    p->SetYUV( pData[1], pData[-2], pData[0], bitShift );
                }
                else
                {
                    p->SetYUV( pData[1], pData[0], pData[2], bitShift );
                }
                ++p;
                pData += pIB->iBytesPerPixel * m_GridStepsX;
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void VectorScopeCanvas::ProcessYUV411_UYYVYYData( const ImageBuffer* pIB, DataPoint* p, const int bitShift )
//-----------------------------------------------------------------------------
{
    if( pIB->iWidth > 1 )
    {
        ProcessingHelperYUV411_UYYVYY ph( m_AOIx );
        int incOdd = 0;
        int incEven = 0;
        for( int i = 0; i < m_GridStepsX; i++ )
        {
            incEven += ( i % 2 ) ? 2 : 1;
            incOdd += ( i % 2 ) ? 1 : 2;
        }
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) ) + ph.GetOffsetToFirstY();
            for( int pixel = m_AOIx; pixel < m_AOIw + m_AOIx; pixel += m_GridStepsX )
            {
                ph.RefreshUAndVOffsets( pixel );
                p->SetYUV( pData[0], pData[ph.GetUOffset()], pData[ph.GetVOffset()], bitShift );
                ++p;
                pData += ( pixel % 2 ) ? incOdd : incEven;
            }
        }
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void VectorScopeCanvas::ProcessYUV422Data( const ImageBuffer* pIB, DataPoint* p, const int bitShift )
//-----------------------------------------------------------------------------
{
    if( pIB->iWidth > 1 )
    {
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            _Ty* pData = reinterpret_cast<_Ty*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                if( ( m_AOIx + pixel ) & 1 )
                {
                    p->SetYUV( pData[0], pData[-1], pData[1], bitShift );
                }
                else
                {
                    p->SetYUV( pData[0], pData[1], pData[3], bitShift );
                }
                ++p;
                pData += pIB->iBytesPerPixel * m_GridStepsX;
            }
        }
    }
}

//-----------------------------------------------------------------------------
void VectorScopeCanvas::RefreshData( const RequestData& data, int x /* = -1 */, int y /* = -1 */, int w /* = -1 */, int h /* = -1 */, bool boForceRefresh /* = false */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );

    bool boAOIChanged = false;
    bool boImageFormatChanged = false;
    const ImageBuffer* pIB = data.image_.getBuffer();
    if( !MustUpdate( pIB, pIB->iChannelCount, x, y, w, h, boForceRefresh, &boAOIChanged, &boImageFormatChanged ) )
    {
        return;
    }

    m_ChannelCount = pIB->iChannelCount;
    PrepareDataBuffer( boAOIChanged );
    DataPoint* p = m_pDataBuffer;
    int bitShift = 0;

#ifndef TEST_VECTOR_SCOPE
    switch( pIB->pixelFormat )
    {
    case ibpfMono10:
    case ibpfBGR101010Packed_V2:
    case ibpfRGB101010Packed:
    case ibpfYUV422_10Packed:
    case ibpfYUV422_UYVY_10Packed:
    case ibpfYUV444_UYV_10Packed:
    case ibpfYUV444_10Packed:
        m_MaxPixelValue = VAL_COUNT_10_BIT;
        bitShift = 2;
        break;
    case ibpfMono12:
    case ibpfMono12Packed_V1:
    case ibpfMono12Packed_V2:
    case ibpfRGB121212Packed:
        m_MaxPixelValue = VAL_COUNT_12_BIT;
        bitShift = 4;
        break;
    case ibpfMono14:
    case ibpfRGB141414Packed:
        m_MaxPixelValue = VAL_COUNT_14_BIT;
        bitShift = 6;
        break;
    case ibpfMono16:
    case ibpfRGB161616Packed:
        m_MaxPixelValue = VAL_COUNT_16_BIT;
        bitShift = 8;
        break;
    case ibpfMono8:
    case ibpfBGR888Packed:
    case ibpfRGBx888Packed:
    case ibpfRGB888Packed:
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
    case ibpfYUV411_UYYVYY_Packed:
    case ibpfYUV422Packed:
    case ibpfYUV422_UYVYPacked:
    case ibpfYUV444_UYVPacked:
    case ibpfYUV444Packed:
    case ibpfYUV422Planar:
    default:
        m_MaxPixelValue = VAL_COUNT_8_BIT;
        break;
    }

    InitReferencePoints( bitShift );

    switch( pIB->pixelFormat )
    {
    case ibpfMono8:
        // no colour information, thus it doesn't make much sense to calculate all the information as the one
        // and only point drawn will be in the center of the plot using the intensity of the last pixel calculated.
        // However no one shall be able to say we are not trying hard.
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->Set( *pData, *pData, *pData, bitShift );
                ++p;
            }
        }
        break;
    case ibpfMono10:
    case ibpfMono12:
    case ibpfMono14:
    case ibpfMono16:
        // no colour information, thus it doesn't make much sense to calculate all the information as the one
        // and only point drawn will be in the center of the plot using the intensity of the last pixel calculated.
        // However no one shall be able to say we are not trying hard.
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->Set( *pData, *pData, *pData, bitShift );
                ++p;
            }
        }
        break;
    case ibpfMono12Packed_V1:
        ProcessMonoPackedData( pIB, p, bitShift, GetMono12Packed_V1Pixel );
        break;
    case ibpfMono12Packed_V2:
        ProcessMonoPackedData( pIB, p, bitShift, GetMono12Packed_V2Pixel );
        break;
    case ibpfBGR888Packed:
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->Set( pData[0], pData[1], pData[2], bitShift );
                ++p;
                pData += pIB->iBytesPerPixel * m_GridStepsX;
            }
        }
        break;
    case ibpfBGR101010Packed_V2:
        {
            unsigned short red, green, blue;
            for( int line = 0; line < m_AOIh; line += m_GridStepsY )
            {
                unsigned int* pPixel =  reinterpret_cast<unsigned int*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[0].iLinePitch ) ) + m_AOIx;
                for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
                {
                    GetBGR101010Packed_V2Pixel( *pPixel++, red, green, blue );
                    p->Set( red, green, blue, bitShift );
                    ++p;
                }
            }
        }
        break;
    case ibpfRGBx888Packed:
    case ibpfRGB888Packed:
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->Set( pData[2], pData[1], pData[0], bitShift );
                ++p;
                pData += pIB->iBytesPerPixel * m_GridStepsX;
            }
        }
        break;
    case ibpfRGB101010Packed:
    case ibpfRGB121212Packed:
    case ibpfRGB141414Packed:
    case ibpfRGB161616Packed:
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned short* pData = reinterpret_cast<unsigned short*>( static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel ) );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                unsigned short r = saveAssign( pData[2], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_MaxPixelValue - 1 ) );
                unsigned short g = saveAssign( pData[1], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_MaxPixelValue - 1 ) );
                unsigned short b = saveAssign( pData[0], static_cast<unsigned short>( 0 ), static_cast<unsigned short>( m_MaxPixelValue - 1 ) );
                p->Set( r, g, b, bitShift );
                ++p;
                pData += 3 * m_GridStepsX;
            }
        }
        break;
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned char* pr = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pRed].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + m_AOIx;
            unsigned char* pg = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pGreen].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pGreen].iLinePitch ) + m_AOIx;
            unsigned char* pb = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pBlue].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pBlue].iLinePitch ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->Set( *pr, *pg, *pb );
                ++p;
                pr += m_GridStepsX;
                pg += m_GridStepsX;
                pb += m_GridStepsX;
            }
        }
        break;
    case ibpfYUV444_UYVPacked:
    case ibpfYUV444_UYV_10Packed:
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->SetYUV( pData[1], pData[2], pData[2], bitShift );
                ++p;
                pData += pIB->iBytesPerPixel * m_GridStepsX;
            }
        }
        break;
    case ibpfYUV444_10Packed:
    case ibpfYUV444Packed:
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned char* pData = static_cast<unsigned char*>( pIB->vpData ) + ( ( m_AOIy + line ) * pIB->pChannels[pRed].iLinePitch ) + ( m_AOIx * pIB->iBytesPerPixel );
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->SetYUV( pData[0], pData[1], pData[2], bitShift );
                ++p;
                pData += pIB->iBytesPerPixel * m_GridStepsX;
            }
        }
        break;
    case ibpfYUV422Planar:
        for( int line = 0; line < m_AOIh; line += m_GridStepsY )
        {
            unsigned char* py = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pY].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pY].iLinePitch ) + m_AOIx;
            unsigned char* pu = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pU].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pU].iLinePitch ) + m_AOIx;
            unsigned char* pv = static_cast<unsigned char*>( pIB->vpData ) + pIB->pChannels[pV].iChannelOffset + ( ( m_AOIy + line ) * pIB->pChannels[pV].iLinePitch ) + m_AOIx;
            for( int pixel = 0; pixel < m_AOIw; pixel += m_GridStepsX )
            {
                p->SetYUV( *py, *pu, *pv );
                ++p;
                py += m_GridStepsX;
                if( pixel % 2 )
                {
                    pu += m_GridStepsX;
                    pv += m_GridStepsX;
                }
            }
        }
        break;
    case ibpfYUV411_UYYVYY_Packed:
        ProcessYUV411_UYYVYYData<unsigned char>( pIB, p, bitShift );
        break;
    case ibpfYUV422Packed:
        ProcessYUV422Data<unsigned char>( pIB, p, bitShift );
        break;
    case ibpfYUV422_10Packed:
        ProcessYUV422Data<unsigned short>( pIB, p, bitShift );
        break;
    case ibpfYUV422_UYVYPacked:
        ProcessUYV422Data<unsigned char>( pIB, p, bitShift );
        break;
    case ibpfYUV422_UYVY_10Packed:
        ProcessUYV422Data<unsigned short>( pIB, p, bitShift );
        break;
    default:
        m_boUnsupportedPixelFormat = true;
        break;
    }
#else
    for( int i = 0; i <= 8; i++ )
    {
        for( int j = 0; j <= 8; j++ )
        {
            for( int k = 0; k <= 8; k++ )
            {
                p->Set( saveAssign( 1 << k, 0, 255 ),
                        saveAssign( 1 << j, 0, 255 ),
                        saveAssign( 1 << i, 0, 255 ) );
                ++p;
            }
        }
    }
#endif // #ifndef TEST_VECTOR_SCOPE

    UpdateAnalysisOutput( boAOIChanged );
}

//-----------------------------------------------------------------------------
void VectorScopeCanvas::UpdateGrid( void )
//-----------------------------------------------------------------------------
{
    if( m_pNumericalDisplay )
    {
        unsigned int rowCount = static_cast<unsigned int>( m_pNumericalDisplay->GetTable()->GetNumberRows() );
        if( rowCount < ( m_valCount + 1 ) )
        {
            m_pNumericalDisplay->AppendRows( m_valCount + 1 - rowCount );
        }
        else if( rowCount > ( m_valCount + 1 ) )
        {
            m_pNumericalDisplay->DeleteRows( rowCount - 1, rowCount - ( m_valCount + 1 ) );
        }
    }
    PlotCanvasImageAnalysis::UpdateGrid();
}
