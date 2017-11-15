//-----------------------------------------------------------------------------
#ifndef HistogramCanvasTemporalNoiseH
#define HistogramCanvasTemporalNoiseH HistogramCanvasTemporalNoiseH
//-----------------------------------------------------------------------------
#include "HistogramCanvas.h"

//-----------------------------------------------------------------------------
class HistogramCanvasTemporalNoise : public HistogramCanvas
//-----------------------------------------------------------------------------
{
    mvIMPACT::acquire::ImageBufferDesc m_lastImage;

    void                CalculateTemporalNoiseHistogram( const RequestData& data );
    void                CalculateTemporalNoiseHistogram_8u_RGBPacked( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld, int order[3] );
    template<typename _Ty>
    void                CalculateTemporalNoiseHistogram_MonoPacked( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld, _Ty pixelAccessFn );
    template<typename _Ty>
    void                CalculateTemporalNoiseHistogramUYV422( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld );
    template<typename _Ty>
    void                CalculateTemporalNoiseHistogramYUV411_UYYVYY( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld );
    template<typename _Ty>
    void                CalculateTemporalNoiseHistogramYUV422( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld );
    template<typename _Ty>
    void                CalculateTemporalNoiseHistogramYUV444( const ImageBuffer* pIBNew, const ImageBuffer* pIBOld );
protected:
    virtual void        CustomAlloc( int channelCount );
public:
    explicit            HistogramCanvasTemporalNoise() : m_lastImage( 1 ) {}
    explicit            HistogramCanvasTemporalNoise( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "Temporal Noise Histogram" ), bool boActive = false );
    virtual void        RefreshData( const RequestData& data, int x = -1, int y = -1, int w = -1, int h = -1, bool boForceRefresh = false );
};

#endif // HistogramCanvasTemporalNoiseH
