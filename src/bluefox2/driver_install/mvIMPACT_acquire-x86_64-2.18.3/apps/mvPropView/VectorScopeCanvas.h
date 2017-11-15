//-----------------------------------------------------------------------------
#ifndef VectorScopeCanvasH
#define VectorScopeCanvasH VectorScopeCanvasH
//-----------------------------------------------------------------------------
#include "PlotCanvasImageAnalysis.h"

//-----------------------------------------------------------------------------
class VectorScopeCanvas : public PlotCanvasImageAnalysis
//-----------------------------------------------------------------------------
{
    //-----------------------------------------------------------------------------
    struct DataPoint
            //-----------------------------------------------------------------------------
    {
        wxColour colour_;
        double Cb_;
        double Cr_;
        explicit DataPoint()
        {
            Set( 255, 255, 255 );
        }
        explicit DataPoint( int R, int G, int B, int bitShift = 0 )
        {
            Set( R, G, B, bitShift );
        }
        void Set( int R, int G, int B, int bitShift = 0 )
        {
            colour_ = wxColour( R >> bitShift, G >> bitShift, B >> bitShift );
            Cb_ = -0.168736 * static_cast<double>( R ) - 0.331264 * static_cast<double>( G ) + ( B >> 1 );
            Cr_ =                         ( R >> 1 ) - 0.418688 * static_cast<double>( G ) - 0.081312 * static_cast<double>( B );
        }
        void SetYUV( int Y, int U, int V, int bitShift = 0 )
        {
            Y = Y >> bitShift;
            U = ( U >> bitShift ) - 128;
            V = ( V >> bitShift ) - 128;
            const int R = saveAssign( static_cast<int>( static_cast<double>( Y ) + 1.402 * static_cast<double>( V ) - 0.701 ), 0, 255 );
            const int G = saveAssign( static_cast<int>( static_cast<double>( Y ) - 0.344136 * static_cast<double>( U ) - 0.714136 * static_cast<double>( V ) + 0.513804 ), 0, 255 );
            const int B = saveAssign( static_cast<int>( static_cast<double>( Y ) + 1.772 * static_cast<double>( U ) - 0.886 ), 0, 255 );
            colour_ = wxColour( R, G, B );
            Cb_ = U;
            Cr_ = V;
        }
    };
    enum TRefColours
    {
        rcRed,
        rcYellow,
        rcGreen,
        rcCyan,
        rcBlue,
        rcMagenta,
        rcLAST
    };
    DataPoint*              m_pDataBuffer;
    DataPoint               m_ReferencePoints[6];
    unsigned int            m_MaxPixelValue;
protected:
    virtual void            CreateGridCustom( void );
    void                    DeallocateDataBuffer( void );
    virtual unsigned int    GetXMarkerParameters( unsigned int& from, unsigned int& to ) const
    {
        from = 0;
        to = 0;
        return 1;
    }
    void                    InitReferencePoints( int bitShift );
    virtual void            OnPaintCustom( wxPaintDC& dc );
    void                    PrepareDataBuffer( bool boFormatChanged );
    template<typename _Ty>
    void                    ProcessMonoPackedData( const ImageBuffer* pIB, DataPoint* p, const int bitShift, _Ty pixelAccessFn );
    template<typename _Ty>
    void                    ProcessYUV411_UYYVYYData( const ImageBuffer* pIB, DataPoint* p, const int bitShift );
    template<typename _Ty>
    void                    ProcessYUV422Data( const ImageBuffer* pIB, DataPoint* p, const int bitShift );
    template<typename _Ty>
    void                    ProcessUYV422Data( const ImageBuffer* pIB, DataPoint* p, const int bitShift );
    virtual void            UpdateGrid( void );
public:
    explicit                VectorScopeCanvas() {}
    explicit                VectorScopeCanvas( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "Vector Scope" ), bool boActive = false );
    ~VectorScopeCanvas();
    virtual wxString        GetGridValue( int row, int col ) const;
    virtual void            RefreshData( const RequestData& data, int x = -1, int y = -1, int w = -1, int h = -1, bool boForceRefresh = false );
};

#endif // VectorScopeCanvasH
