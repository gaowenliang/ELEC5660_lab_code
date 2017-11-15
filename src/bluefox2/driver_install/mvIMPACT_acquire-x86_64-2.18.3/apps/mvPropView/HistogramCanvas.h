//-----------------------------------------------------------------------------
#ifndef HistogramCanvasH
#define HistogramCanvasH HistogramCanvasH
//-----------------------------------------------------------------------------
#include "DevData.h"
#include "PlotCanvasImageAnalysis.h"

//-----------------------------------------------------------------------------
struct PlotPoint
//-----------------------------------------------------------------------------
{
    int cnt_;
    int val_;
    PlotPoint( int cnt, int val ) : cnt_( cnt ), val_( val ) {}
    PlotPoint() : cnt_( 0 ), val_( 0 ) {}
};

//-----------------------------------------------------------------------------
class HistogramCanvas : public PlotCanvasImageAnalysis
//-----------------------------------------------------------------------------
{
    int                     m_DrawStepWidth;
protected:
    int**                   m_ppHistogramBuffer;
    PlotPoint*              m_pHistogramMax;
    double*                 m_pHistogramAverage;
    double*                 m_pStandardDeviation;
    bool                    m_boHandleStandardDeviation;
    PlotPoint               m_CurrentMax;

    void                    CalculateMaxValue( void );
    void                    CalculateSpatialNoiseHistogram( const RequestData& data );
    void                    CalculateSpatialNoiseHistogram_8u_C3Packed( const ImageBuffer* pIB, int order[3] );
    template<typename _Ty>
    void                    CalculateSpatialNoiseHistogramUYV422( const ImageBuffer* pIB );
    template<typename _Ty>
    void                    CalculateSpatialNoiseHistogramYUV411_UYYVYY( const ImageBuffer* pIB );
    template<typename _Ty>
    void                    CalculateSpatialNoiseHistogramYUV422( const ImageBuffer* pIB );
    void                    CalculatePixelHistogram( const RequestData& data );
    void                    CalculatePixelHistogram_8u_RGBPacked( const ImageBuffer* pIB, int order[3] );
    template<typename _Ty>
    void                    CalculateSpatialNoiseHistogram_MonoPacked( const ImageBuffer* pIB, _Ty pixelAccessFn );
    template<typename _Ty>
    void                    CalculatePixelHistogram_MonoPacked( const ImageBuffer* pIB, _Ty pixelAccessFn );
    template<typename _Ty>
    void                    CalculatePixelHistogramUYV422( const ImageBuffer* pIB );
    template<typename _Ty>
    void                    CalculatePixelHistogramYUV411_UYYVYY( const ImageBuffer* pIB );
    template<typename _Ty>
    void                    CalculatePixelHistogramYUV422( const ImageBuffer* pIB );
    template<typename _Ty>
    void                    CalculatePixelHistogramYUV444( const ImageBuffer* pIB );
    virtual void            CustomAlloc( int /*channelCount*/ ) {}
    virtual void            CustomDealloc( void ) {}
    void                    DeallocateHistogramBuffer( void );
    virtual void            OnPaintCustom( wxPaintDC& dc );
    void                    DrawHistogramLine( wxPaintDC& dc, int h, double scaleX, double scaleY, TPlane plane );
    virtual double          GetScaleX( wxCoord w ) const;
    virtual double          GetScaleY( wxCoord h ) const;
    virtual unsigned int    GetXMarkerParameters( unsigned int& from, unsigned int& to ) const;
    bool                    PrepareHistogramBuffer( bool boImageFormatChanged, int channelCount, mvIMPACT::acquire::TImageBufferPixelFormat pixelFormat );
    virtual void            RefreshAnalysisData( void );
    void                    SetupNumericalDisplay( int channelCount, int rowCountNeeded );
public:
    explicit                HistogramCanvas() {}
    explicit                HistogramCanvas( wxWindow* parent, const wxString& configName, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "Histogram" ), bool boActive = false, bool boHandleStandardDeviation = false );
    virtual                ~HistogramCanvas();
    void                    SetDrawStepWidth( int value );
    static int              GetDrawStepWidthMax( void )
    {
        return 8;
    }
    virtual wxString        GetGridValue( int row, int col ) const;
};

#endif // HistogramCanvasH
