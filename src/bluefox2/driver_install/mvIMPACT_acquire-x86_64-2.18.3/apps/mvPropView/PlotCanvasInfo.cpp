#include <apps/Common/wxAbstraction.h>
#include <algorithm>
#include <limits>
#include "PlotCanvasInfo.h"
#include "PropViewFrame.h"
#include <wx/dcbuffer.h>

using namespace std;
using namespace mvIMPACT::acquire;

//=============================================================================
//============== Implementation PlotCanvasInfoBase ============================
//=============================================================================
//-----------------------------------------------------------------------------
PlotCanvasInfoBase::PlotCanvasInfoBase( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                                        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */,
                                        const wxString& name /* = "PlotCanvasInfoBase" */, bool boActive /* = false */ )
    : PlotCanvas( parent, id, pos, size, style, name, boActive ),
      m_CurrentPlotValues(), m_boAutoScale( false ), m_boPlotDifferences( false ), m_PlotIdentifiers(), m_ppPlotValues( 0 ),
      m_dataType( ctPropInt64 ), m_HistoryDepth( 20 ), m_CurrentMaxPlotValues(), m_CurrentMinPlotValues(), m_PlotCount( 1 ),
      m_CurrentMaxPlotValue( numeric_limits<plot_data_type>::min() ), m_CurrentMinPlotValue( numeric_limits<plot_data_type>::max() )
//-----------------------------------------------------------------------------
{
    AllocateDataBuffer( 1 );
    m_pens[0] = *wxRED;
    m_pens[1] = *wxGREEN;
    m_pens[2] = *wxBLUE;
    m_pens[3] = wxColour( 128, 128, 0 );
}

//-----------------------------------------------------------------------------
PlotCanvasInfoBase::~PlotCanvasInfoBase()
//-----------------------------------------------------------------------------
{
    DeallocateDataBuffer();
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::AllocateDataBuffer( unsigned int plotCount )
//-----------------------------------------------------------------------------
{
    DeallocateDataBuffer();
    m_PlotCount = plotCount;
    m_ppPlotValues = new deque<plot_data_type>* [m_PlotCount];
    m_CurrentMaxPlotValues.resize( m_PlotCount );
    m_CurrentMinPlotValues.resize( m_PlotCount );
    m_CurrentPlotValues.resize( m_PlotCount );
    m_PlotIdentifiers.resize( m_PlotCount );
    for( unsigned int i = 0; i < m_PlotCount; i++ )
    {
        m_ppPlotValues[i] = new deque<plot_data_type>();
        m_CurrentMaxPlotValues[i] = numeric_limits<plot_data_type>::min();
        m_CurrentMinPlotValues[i] = numeric_limits<plot_data_type>::max();
        m_CurrentPlotValues[i] = 0;
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::ClearCache( void )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    ClearCache_Internal();
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::ClearCache_Internal( void )
//-----------------------------------------------------------------------------
{
    for( unsigned int i = 0; i < m_PlotCount; i++ )
    {
        m_ppPlotValues[i]->clear();
        m_CurrentMaxPlotValues[i] = numeric_limits<plot_data_type>::min();
        m_CurrentMinPlotValues[i] = numeric_limits<plot_data_type>::max();
    }
    m_CurrentMaxPlotValue = numeric_limits<plot_data_type>::min();
    m_CurrentMinPlotValue = numeric_limits<plot_data_type>::max();
    Refresh( true );
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::DeallocateDataBuffer( void )
//-----------------------------------------------------------------------------
{
    if( m_ppPlotValues )
    {
        for( unsigned int i = 0; i < m_PlotCount; i++ )
        {
            delete m_ppPlotValues[i];
        }
        delete [] m_ppPlotValues;
        m_ppPlotValues = 0;
        m_CurrentMaxPlotValues.clear();
        m_CurrentMinPlotValues.clear();
        m_CurrentPlotValues.clear();
        m_PlotIdentifiers.clear();
        m_PlotCount = 0;
    }
}

//-----------------------------------------------------------------------------
double PlotCanvasInfoBase::GetScaleX( wxCoord w ) const
//-----------------------------------------------------------------------------
{
    return static_cast<double>( w - 2 * GetBorderWidth() ) / static_cast<double>( m_HistoryDepth );
}

//-----------------------------------------------------------------------------
double PlotCanvasInfoBase::GetScaleY( wxCoord h ) const
//-----------------------------------------------------------------------------
{
    const plot_data_type currentYRange = m_CurrentMaxPlotValue - GetOffset();
    return static_cast<double>( h - 2 * GetBorderWidth() ) / static_cast<double>( ( currentYRange != 0 ) ? currentYRange : 1 );
}

//-----------------------------------------------------------------------------
unsigned int PlotCanvasInfoBase::GetXMarkerParameters( unsigned int& from, unsigned int& to ) const
//-----------------------------------------------------------------------------
{
    from = 0;
    to = m_HistoryDepth;
    return m_HistoryDepth / 5;
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::OnPaintCustom( wxPaintDC& dc )
//-----------------------------------------------------------------------------
{
    wxCoord xOffset( 1 ), w( 0 ), h( 0 );
    dc.GetSize( &w, &h );
    const double scaleX = GetScaleX( w );
    const double scaleY = GetScaleY( h );
    DrawMarkerLines( dc, w, h, scaleX );
    const plot_data_type offset = GetOffset();
    for( unsigned int i = 0; i < m_PlotCount; i++ )
    {
        const unsigned int valCount = static_cast<unsigned int>( m_ppPlotValues[i]->size() );
        if( valCount > 1 )
        {
            int lowerStart = h - GetBorderWidth();
            dc.SetPen( m_pens[i % COLOUR_COUNT] );
            for( unsigned int j = 0; j < valCount - 1; j++ )
            {
                dc.DrawLine( static_cast<int>( GetBorderWidth() + ( j * scaleX ) + 1 ),
                             static_cast<int>( lowerStart - ( ( ( *m_ppPlotValues[i] )[j] - offset ) * scaleY ) ),
                             static_cast<int>( GetBorderWidth() + ( ( j + 1 ) * scaleX ) + 1 ),
                             static_cast<int>( lowerStart - ( ( ( *m_ppPlotValues[i] )[j + 1] - offset ) * scaleY ) ) );
            }
            dc.SetPen( wxNullPen );
        }
        // info in the top left corner
        if( m_dataType == ctPropFloat )
        {
            DrawInfoString( dc, wxString::Format( wxT( "Range: %.3f - %.3f, %s: %s " ), m_CurrentMinPlotValues[i], m_CurrentMaxPlotValues[i], GetPlotIdentifierPrefix().c_str(), m_PlotIdentifiers[i].c_str() ), xOffset, 1, m_pens[i] );
        }
        else
        {
            DrawInfoString( dc, wxString::Format( wxT( "Range: %lld - %lld, %s: %s " ), static_cast<int64_type>( m_CurrentMinPlotValues[i] ), static_cast<int64_type>( m_CurrentMaxPlotValues[i] ), GetPlotIdentifierPrefix().c_str(), m_PlotIdentifiers[i].c_str() ), xOffset, 1, m_pens[i] );
        }
    }
}

//-----------------------------------------------------------------------------
bool PlotCanvasInfoBase::RefreshPlotData( void )
//-----------------------------------------------------------------------------
{
    bool boFullUpdate = false;

    for( unsigned int i = 0; i < m_PlotCount; i++ )
    {
        bool boMaxRemovedFromQueue = false;
        bool boMinRemovedFromQueue = false;
        while( m_ppPlotValues[i]->size() > m_HistoryDepth )
        {
            if( m_ppPlotValues[i]->front() == m_CurrentMaxPlotValues[i] )
            {
                boMaxRemovedFromQueue = true;
            }
            if( m_ppPlotValues[i]->front() == m_CurrentMinPlotValues[i] )
            {
                boMinRemovedFromQueue = true;
            }
            m_ppPlotValues[i]->pop_front();
        }

        if( boMaxRemovedFromQueue )
        {
            m_CurrentMaxPlotValues[i] = *( max_element( m_ppPlotValues[i]->begin(), m_ppPlotValues[i]->end() ) );
        }
        else if( !m_ppPlotValues[i]->empty() && ( m_ppPlotValues[i]->back() > m_CurrentMaxPlotValues[i] ) )
        {
            m_CurrentMaxPlotValues[i] = m_ppPlotValues[i]->back();
        }

        if( boMinRemovedFromQueue )
        {
            m_CurrentMinPlotValues[i] = *( min_element( m_ppPlotValues[i]->begin(), m_ppPlotValues[i]->end() ) );
        }
        else if( !m_ppPlotValues[i]->empty() && ( m_ppPlotValues[i]->back() < m_CurrentMinPlotValues[i] ) )
        {
            m_CurrentMinPlotValues[i] = m_ppPlotValues[i]->back();
        }
    }

    plot_data_type maxVal = *( max_element( m_CurrentMaxPlotValues.begin(), m_CurrentMaxPlotValues.end() ) );
    if( m_CurrentMaxPlotValue != maxVal )
    {
        m_CurrentMaxPlotValue = maxVal;
        boFullUpdate = true;
    }

    plot_data_type minVal = *( min_element( m_CurrentMinPlotValues.begin(), m_CurrentMinPlotValues.end() ) );
    if( m_CurrentMinPlotValue != minVal )
    {
        m_CurrentMinPlotValue = minVal;
        boFullUpdate = true;
    }

    return boFullUpdate;
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::SetAutoScale( bool boActive )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    if( m_boAutoScale != boActive )
    {
        m_boAutoScale = boActive;
        Refresh( RefreshPlotData() );
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::SetHistoryDepth( unsigned int historyDepth )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_HistoryDepth = historyDepth;
    Refresh( RefreshPlotData() );
}

//-----------------------------------------------------------------------------
void PlotCanvasInfoBase::SetPlotDifferences( bool boActive )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    if( m_boPlotDifferences != boActive )
    {
        m_boPlotDifferences = boActive;
        ClearCache_Internal();
        Refresh( RefreshPlotData() );
    }
}

//=============================================================================
//============== Implementation PlotCanvasInfo ================================
//=============================================================================
//-----------------------------------------------------------------------------
PlotCanvasInfo::PlotCanvasInfo( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                                const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */,
                                const wxString& name /* = "PlotCanvasInfo" */, bool boActive /* = false */ )
    : PlotCanvasInfoBase( parent, id, pos, size, style, name, boActive ), m_PlotHashTable()
//-----------------------------------------------------------------------------
{

}


//-----------------------------------------------------------------------------
bool PlotCanvasInfo::MustUpdate( bool boForceRefresh )
//-----------------------------------------------------------------------------
{
    if( ( ( m_ImageCount++ % m_UpdateFrequency ) == 0 ) || boForceRefresh )
    {
        return true;
    }
    return false;
}

//-----------------------------------------------------------------------------
void PlotCanvasInfo::RefreshData( const RequestInfoData& infoData, bool boForceRefresh /* = false */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    if( !IsActive() )
    {
        return;
    }

    plot_data_type value;
    switch( infoData.plotValue_.type )
    {
    case ctPropInt:
        value = static_cast<plot_data_type>( infoData.plotValue_.value.intRep );
        break;
    case ctPropInt64:
        value = static_cast<plot_data_type>( infoData.plotValue_.value.int64Rep );
        break;
    case ctPropFloat:
        value = static_cast<plot_data_type>( infoData.plotValue_.value.doubleRep );
        break;
    default:
        // we don't support other component types for this feature
        return;
    }

    m_dataType = infoData.plotValue_.type;
    const PlotHashTable::const_iterator it = m_PlotHashTable.find( infoData.settingUsed_ );
    const unsigned int index = ( it == m_PlotHashTable.end() ) ? 0 : it->second;
    m_ppPlotValues[index]->push_back( m_boPlotDifferences ? value - m_CurrentPlotValues[index] : value );
    m_CurrentPlotValues[index] = value;

    const bool boFullUpdate = RefreshPlotData();
    if( MustUpdate( boForceRefresh ) )
    {
        Refresh( boFullUpdate );
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasInfo::SetupPlotIdentifiers( const vector<pair<string, int> >& plotIdentifiers )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    const unsigned int plotCount = static_cast<unsigned int>( plotIdentifiers.size() );
    AllocateDataBuffer( plotCount );
    m_PlotHashTable.clear();
    for( unsigned int i = 0; i < plotCount; i++ )
    {
        m_PlotHashTable.insert( make_pair( plotIdentifiers[i].second, i ) );
        m_PlotIdentifiers[i] = ConvertedString( plotIdentifiers[i].first );
    }
    Refresh( true );
}

//=============================================================================
//============== Implementation PlotCanvasFeatureVsTime =======================
//=============================================================================
BEGIN_EVENT_TABLE( PlotCanvasFeatureVsTime, PlotCanvasInfoBase )
    EVT_TIMER( wxID_ANY, PlotCanvasFeatureVsTime::OnTimer )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
PlotCanvasFeatureVsTime::PlotCanvasFeatureVsTime( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */,
        const wxString& name /* = "PlotCanvasFeatureVsTime" */ )
    : PlotCanvasInfoBase( parent, id, pos, size, style, name, true ),
      m_SelectedComponent(), m_UpdateTimer( this, teUpdate )
//-----------------------------------------------------------------------------
{
    Init();
}

//-----------------------------------------------------------------------------
void PlotCanvasFeatureVsTime::Init( void )
//-----------------------------------------------------------------------------
{
    // this plot interprets these values as milliseconds not as images
    m_UpdateFrequencyMin = 10;
    m_UpdateFrequencyMax = 60000;
}

//-----------------------------------------------------------------------------
void PlotCanvasFeatureVsTime::OnTimer( wxTimerEvent& e )
//-----------------------------------------------------------------------------
{
    try
    {
        switch( e.GetId() )
        {
        case teUpdate:
            RefreshData();
            break;
        default:
            break;
        }
    }
    catch( const ImpactAcquireException& theException )
    {
        dynamic_cast<PropViewFrame*>( GetParent() )->WriteErrorMessage( wxString::Format( wxT( "%s: An exception was generated while updating the feature vs. time plot: %s(%s)\n" ), ConvertedString( __FUNCTION__ ).c_str(), ConvertedString( theException.getErrorString() ).c_str(), ConvertedString( theException.getErrorCodeAsString() ).c_str() ) );
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasFeatureVsTime::OnUpdateFrequencyChanged( void )
//-----------------------------------------------------------------------------
{
    if( m_UpdateTimer.IsRunning() )
    {
        m_UpdateTimer.Start( GetUpdateFrequency() );
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasFeatureVsTime::RefreshData( void )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    if( !IsActive() || !m_SelectedComponent.isValid() )
    {
        return;
    }

    plot_data_type value;
    switch( m_SelectedComponent.type() )
    {
    case ctPropInt:
        {
            PropertyI prop( m_SelectedComponent.hObj() );
            value = static_cast<plot_data_type>( prop.read() );
        }
        break;
    case ctPropInt64:
        {
            PropertyI64 prop( m_SelectedComponent.hObj() );
            value = static_cast<plot_data_type>( prop.read() );
        }
        break;
    case ctPropFloat:
        {
            PropertyF prop( m_SelectedComponent.hObj() );
            value = static_cast<plot_data_type>( prop.read() );
        }
        break;
    default:
        // we don't support other component types for this feature
        return;
    }

    m_ppPlotValues[0]->push_back( m_boPlotDifferences ? value - m_CurrentPlotValues[0] : value );
    m_CurrentPlotValues[0] = value;
    Refresh( RefreshPlotData() );
}

//-----------------------------------------------------------------------------
void PlotCanvasFeatureVsTime::SetActive( bool boActive )
//-----------------------------------------------------------------------------
{
    if( boActive )
    {
        m_UpdateTimer.Start( GetUpdateFrequency() );
    }
    else
    {
        m_UpdateTimer.Stop();
    }
    Refresh( !boActive );
}

//-----------------------------------------------------------------------------
void PlotCanvasFeatureVsTime::SetComponentToPlot( mvIMPACT::acquire::Component comp, const wxString& fullPath /* = wxEmptyString */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_SelectedComponent = comp;
    m_PlotIdentifiers[0] = fullPath.AfterLast( wxT( '.' ) );
    if( comp.isValid() )
    {
        const TComponentType type = comp.type();
        switch( type )
        {
        case ctPropFloat:
        case ctPropInt:
        case ctPropInt64:
            m_dataType = type;
            break;
        default:
            // we don't support other component types for this feature
            break;
        }
    }
    else
    {
        m_dataType = ctPropInt64;
    }
    ClearCache();
}
