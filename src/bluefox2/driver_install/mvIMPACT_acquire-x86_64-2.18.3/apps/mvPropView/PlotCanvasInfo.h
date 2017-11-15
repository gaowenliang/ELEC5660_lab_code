//-----------------------------------------------------------------------------
#ifndef PlotCanvasInfoH
#define PlotCanvasInfoH PlotCanvasInfoH
//-----------------------------------------------------------------------------
#include "DevData.h"
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "PlotCanvas.h"
#include <deque>
#include <wx/timer.h>

//-----------------------------------------------------------------------------
class PlotCanvasInfoBase : public PlotCanvas
//-----------------------------------------------------------------------------
{
public:
    explicit                     PlotCanvasInfoBase() {}
    explicit                     PlotCanvasInfoBase( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "PlotCanvasInfoBase" ), bool boActive = false );
    virtual                     ~PlotCanvasInfoBase();
    void                         ClearCache( void );
    void                         SetHistoryDepth( unsigned int historyDepth );
    unsigned int                 GetHistoryDepth( void ) const
    {
        return m_HistoryDepth;
    }
    void                         SetAutoScale( bool boActive );
    void                         SetPlotDifferences( bool boActive );
protected:
    void                         AllocateDataBuffer( unsigned int plotCount );
    virtual double               GetScaleX( wxCoord w ) const;
    virtual double               GetScaleY( wxCoord h ) const;
    virtual unsigned int         GetXMarkerParameters( unsigned int& from, unsigned int& to ) const;
    virtual void                 OnPaintCustom( wxPaintDC& dc );

    bool                         RefreshPlotData( void );

    typedef double               plot_data_type;
    typedef std::map<int, int>   PlotHashTable;
    std::vector<plot_data_type>  m_CurrentPlotValues;
    bool                         m_boAutoScale;
    bool                         m_boPlotDifferences;
    std::vector<wxString>        m_PlotIdentifiers;
    std::deque<plot_data_type>** m_ppPlotValues;
    TComponentType               m_dataType;
private:
    enum
    {
        COLOUR_COUNT = 4
    };

    unsigned int                 m_HistoryDepth;
    std::vector<plot_data_type>  m_CurrentMaxPlotValues;
    std::vector<plot_data_type>  m_CurrentMinPlotValues;
    wxColour                     m_pens[COLOUR_COUNT];
    size_t                       m_PlotCount;
    plot_data_type               m_CurrentMaxPlotValue;
    plot_data_type               m_CurrentMinPlotValue;

    void                         ClearCache_Internal( void );
    void                         DeallocateDataBuffer( void );
    plot_data_type               GetOffset( void ) const
    {
        return m_boAutoScale ? m_CurrentMinPlotValue : ( ( m_CurrentMinPlotValue < 0 ) ? m_CurrentMinPlotValue : 0 );
    }
    virtual const wxString&      GetPlotIdentifierPrefix( void ) = 0;
};

//-----------------------------------------------------------------------------
class PlotCanvasInfo : public PlotCanvasInfoBase
//-----------------------------------------------------------------------------
{
public:
    explicit                     PlotCanvasInfo() {}
    explicit                     PlotCanvasInfo( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "PlotCanvasInfo" ), bool boActive = false );
    void                         RefreshData( const RequestInfoData& infoData, bool boForceRefresh = false );
    void                         SetupPlotIdentifiers( const std::vector<std::pair<std::string, int> >& plotIdentifiers );
private:
    PlotHashTable                m_PlotHashTable;

    virtual const wxString&      GetPlotIdentifierPrefix( void )
    {
        static wxString s_Prefix( wxT( "Setting" ) );
        return s_Prefix;
    }
    bool                         MustUpdate( bool boForceRefresh );
};

//-----------------------------------------------------------------------------
class PlotCanvasFeatureVsTime : public PlotCanvasInfoBase
//-----------------------------------------------------------------------------
{
public:
    explicit                     PlotCanvasFeatureVsTime()
    {
        Init();
    }
    explicit                     PlotCanvasFeatureVsTime( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "PlotCanvasFeatureVsTime" ) );
    mvIMPACT::acquire::Component GetComponentToPlot( void ) const
    {
        return m_SelectedComponent;
    }
    const wxString&              GetComponentToPlotFullPath( void ) const
    {
        return m_PlotIdentifiers[0];
    }
    void                         SetComponentToPlot( mvIMPACT::acquire::Component comp, const wxString& fullPath = wxEmptyString );
    void                         UnsetComponentToPlot( void )
    {
        SetComponentToPlot( mvIMPACT::acquire::Component() );
    }
    virtual void                 SetActive( bool boActive );
    void                         RefreshData( void );
protected:
    void                         OnTimer( wxTimerEvent& e );
    virtual void                 OnUpdateFrequencyChanged( void );
private:
    //-----------------------------------------------------------------------------
    enum TTimerEvent
    //-----------------------------------------------------------------------------
    {
        teUpdate
    };
    mvIMPACT::acquire::Component m_SelectedComponent;
    wxTimer                      m_UpdateTimer;

    virtual const wxString&      GetPlotIdentifierPrefix( void )
    {
        static wxString s_Prefix( wxT( "Feature" ) );
        return s_Prefix;
    }
    void                         Init();

    DECLARE_EVENT_TABLE()
};
#endif // PlotCanvasInfoH
