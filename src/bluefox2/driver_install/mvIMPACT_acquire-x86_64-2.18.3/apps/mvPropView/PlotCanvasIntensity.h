//-----------------------------------------------------------------------------
#ifndef PlotCanvasIntensityH
#define PlotCanvasIntensityH PlotCanvasIntensityH
//-----------------------------------------------------------------------------
#include <deque>
#include "HistogramCanvasPixel.h"

//-----------------------------------------------------------------------------
class PlotCanvasIntensity : public HistogramCanvasPixel
//-----------------------------------------------------------------------------
{
public:
    explicit                PlotCanvasIntensity() {}
    explicit                PlotCanvasIntensity( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "Intensity Plot" ), bool boActive = false );
    virtual                ~PlotCanvasIntensity();
    virtual wxString        GetPlotSelection( void ) const
    {
        return m_PlotNames[m_SelectedPlot];
    }
    virtual void            SetPlotSelection( const wxString& plotName );
    virtual wxString        GetGridValue( int row, int col ) const;
protected:
    virtual void            CustomAlloc( int channelCount );
    virtual void            CustomDealloc( void );
    virtual bool            CustomRefreshData( const RequestData& data, int x = -1, int y = -1, int w = -1, int h = -1 );
    virtual void            OnPaintCustom( wxPaintDC& dc );
    virtual int             GetRowCountNeededForNumericalDisplay( void ) const;
    virtual double          GetScaleX( wxCoord w ) const;
    virtual double          GetScaleY( wxCoord h ) const;
    virtual unsigned int    GetXMarkerParameters( unsigned int& from, unsigned int& to ) const;
    virtual void            UpdateInternalData( void );
private:
    typedef double plot_data_type;

    //-----------------------------------------------------------------------------
    enum TPlotSelection
    //-----------------------------------------------------------------------------
    {
        psFIRST,
        psAverageIntensity = psFIRST,
        psMostFrequentValue,
        psLAST
    };

    TPlotSelection               m_SelectedPlot;
    std::deque<plot_data_type>** m_ppPlotValues;
    std::vector<plot_data_type>  m_CurrentMaxPlotValues;
    std::vector<plot_data_type>  m_CurrentMinPlotValues;
    size_t                       m_PlotCount;
    int                          m_CurrentMaxPlotValue;
    int                          m_CurrentMinPlotValue;

    bool                         RefreshPlotData( void );
};

#endif // PlotCanvasIntensityH
