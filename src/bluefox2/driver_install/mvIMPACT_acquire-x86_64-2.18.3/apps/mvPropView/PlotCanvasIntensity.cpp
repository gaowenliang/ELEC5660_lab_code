//-----------------------------------------------------------------------------
#include <algorithm>
#include "PlotCanvasIntensity.h"
#include <limits>

using namespace std;

//=============================================================================
//================= Implementation PlotCanvasIntensity ========================
//=============================================================================
//-----------------------------------------------------------------------------
PlotCanvasIntensity::PlotCanvasIntensity( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "Intensity Plot" */, bool boActive /* = true */ )
    : HistogramCanvasPixel( parent, wxT( "IntensityPlot" ), id, pos, size, style, name, boActive ), m_SelectedPlot( psAverageIntensity ),
      m_ppPlotValues( 0 ), m_CurrentMaxPlotValues(), m_CurrentMinPlotValues(),
      m_CurrentMaxPlotValue( numeric_limits<plot_data_type>::min() ), m_CurrentMinPlotValue( numeric_limits<plot_data_type>::max() )
//-----------------------------------------------------------------------------
{
    m_plotFeatures.insert( pfHistoryDepth );
    m_plotFeatures.erase( pfStepWidth );
    CustomAlloc( 1 );
    m_PlotNames.resize( psLAST );
    m_PlotNames[psAverageIntensity] = wxString( wxT( "Average Intensity" ) );
    m_PlotNames[psMostFrequentValue] = wxString( wxT( "Most Frequent Value" ) );
    SetGridValueFormatString( wxT( "%.3f" ) );
}

//-----------------------------------------------------------------------------
PlotCanvasIntensity::~PlotCanvasIntensity()
//-----------------------------------------------------------------------------
{
    CustomDealloc();
}

//-----------------------------------------------------------------------------
void PlotCanvasIntensity::CustomAlloc( int channelCount )
//-----------------------------------------------------------------------------
{
    m_PlotCount = channelCount;
    m_ppPlotValues = new deque<plot_data_type>* [m_PlotCount];
    m_CurrentMaxPlotValues.resize( m_PlotCount );
    m_CurrentMinPlotValues.resize( m_PlotCount );
    for( unsigned int i = 0; i < m_PlotCount; i++ )
    {
        m_ppPlotValues[i] = new deque<plot_data_type>();
        m_CurrentMaxPlotValues[i] = numeric_limits<plot_data_type>::min();
        m_CurrentMinPlotValues[i] = numeric_limits<plot_data_type>::max();
    }
}

//-----------------------------------------------------------------------------
void PlotCanvasIntensity::CustomDealloc( void )
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
        m_PlotCount = 0;
    }
}

//-----------------------------------------------------------------------------
bool PlotCanvasIntensity::CustomRefreshData( const RequestData& /*data*/, int /*x = -1*/, int /*y = -1*/, int /*w = -1*/, int /*h = -1*/ )
//-----------------------------------------------------------------------------
{
    switch( m_SelectedPlot )
    {
    case psAverageIntensity:
        for( size_t channel = 0; channel < m_PlotCount; channel++ )
        {
            m_ppPlotValues[channel]->push_back( m_pHistogramAverage[channel] );
        }
        break;
    case psMostFrequentValue:
        for( size_t channel = 0; channel < m_PlotCount; channel++ )
        {
            m_ppPlotValues[channel]->push_back( m_pHistogramMax[channel].val_ );
        }
        break;
    case psLAST:
        return false;
        // don't add a default here. If an enum is not handled here the compiler can tell us then
    }

    return RefreshPlotData();
}

//-----------------------------------------------------------------------------
wxString PlotCanvasIntensity::GetGridValue( int row, int col ) const
//-----------------------------------------------------------------------------
{
    if( col == 0 )
    {
        return ( row == 0 ) ? wxT( "Timeline Steps" ) : wxString::Format( wxT( "%d" ), row - 1 );
    }
    else if( m_ppPlotValues && ( col > 0 ) && ( row >= 0 ) && ( col <= m_ChannelCount ) && ( static_cast<deque<plot_data_type>::size_type>( row ) <= m_ppPlotValues[col - 1]->size() ) )
    {
        return ( row == 0 ) ? m_Pens[col - 1].description_ : wxString::Format( GetGridValueFormatString().c_str(), ( *m_ppPlotValues[col - 1] )[row - 1] );
    }
    return wxEmptyString;
}

//-----------------------------------------------------------------------------
int PlotCanvasIntensity::GetRowCountNeededForNumericalDisplay( void ) const
//-----------------------------------------------------------------------------
{
    return m_HistoryDepth + 1;
}

//-----------------------------------------------------------------------------
double PlotCanvasIntensity::GetScaleX( wxCoord w ) const
//-----------------------------------------------------------------------------
{
    return static_cast<double>( w - 2 * GetBorderWidth() ) / static_cast<double>( m_HistoryDepth );
}

//-----------------------------------------------------------------------------
double PlotCanvasIntensity::GetScaleY( wxCoord h ) const
//-----------------------------------------------------------------------------
{
    const int diagramHeight = h - 2 * GetBorderWidth();
    return static_cast<double>( diagramHeight ) / ( static_cast<double>( m_valCount ) * static_cast<double>( GetPercentageToDraw() ) / 100. );
}

//-----------------------------------------------------------------------------
unsigned int PlotCanvasIntensity::GetXMarkerParameters( unsigned int& from, unsigned int& to ) const
//-----------------------------------------------------------------------------
{
    from = 0;
    to = m_HistoryDepth;
    return m_HistoryDepth / 5;
}

//-----------------------------------------------------------------------------
void PlotCanvasIntensity::OnPaintCustom( wxPaintDC& dc )
//-----------------------------------------------------------------------------
{
    wxCoord xOffset( 1 ), w( 0 ), h( 0 );
    dc.GetSize( &w, &h );
    const double scaleX = GetScaleX( w );
    const double scaleY = GetScaleY( h );
    DrawMarkerLines( dc, w, h, scaleX );
    unsigned int from, to;
    GetDrawRange( &from, &to );
    for( unsigned int i = 0; i < m_PlotCount; i++ )
    {
        const unsigned int valCount = static_cast<unsigned int>( m_ppPlotValues[i]->size() );
        if( valCount > 1 )
        {
            int lowerStart = h - GetBorderWidth();
            dc.SetPen( *m_Pens[i].pColour_ );
            for( unsigned int j = 0; j < valCount - 1; j++ )
            {
                dc.DrawLine( static_cast<wxCoord>( GetBorderWidth() + ( j * scaleX ) ),
                             static_cast<wxCoord>( lowerStart - ( saveAssign( static_cast<wxCoord>( ( *m_ppPlotValues[i] )[j] - from ), 0, static_cast<wxCoord>( to - from ) ) * scaleY ) ),
                             static_cast<wxCoord>( GetBorderWidth() + ( ( j + 1 ) * scaleX ) ),
                             static_cast<wxCoord>( lowerStart - ( saveAssign( static_cast<wxCoord>( ( *m_ppPlotValues[i] )[j + 1] - from ), 0, static_cast<wxCoord>( to - from ) ) * scaleY ) ) );
            }
            dc.SetPen( wxNullPen );
        }
        // info in the top left corner
        DrawInfoString( dc, wxString::Format( wxT( "%s: max: %.2f, min: %.2f : " ), m_Pens[i].description_.c_str(), m_CurrentMaxPlotValues[i], ( m_CurrentMinPlotValues[i] == numeric_limits<plot_data_type>::max() ) ? 0. : m_CurrentMinPlotValues[i] ), xOffset, 1, *( m_Pens[i].pColour_ ) );
    }
}

//-----------------------------------------------------------------------------
bool PlotCanvasIntensity::RefreshPlotData( void )
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
void PlotCanvasIntensity::SetPlotSelection( const wxString& plotName )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );

    TPlotSelection plotSelection = psLAST;
    for( vector<wxString>::size_type i = 0; i < psLAST; i++ )
    {
        if( m_PlotNames[i] == plotName )
        {
            plotSelection = static_cast<TPlotSelection>( i );
            break;
        }
    }

    if( m_SelectedPlot == plotSelection )
    {
        return;
    }

    // ignore invalid values
    if( ( plotSelection >= psLAST ) || ( plotSelection < psFIRST ) )
    {
        return;
    }

    m_SelectedPlot = plotSelection;
    for( unsigned int i = 0; i < m_PlotCount; i++ )
    {
        m_ppPlotValues[i]->clear();
        m_CurrentMaxPlotValues[i] = numeric_limits<plot_data_type>::min();
        m_CurrentMinPlotValues[i] = numeric_limits<plot_data_type>::max();
    }
    m_CurrentMaxPlotValue = numeric_limits<plot_data_type>::min();
    m_CurrentMinPlotValue = numeric_limits<plot_data_type>::max();
    UpdateAnalysisOutput( true );
}

//-----------------------------------------------------------------------------
void PlotCanvasIntensity::UpdateInternalData( void )
//-----------------------------------------------------------------------------
{
    RefreshPlotData();
    SetupNumericalDisplay( m_ChannelCount, GetRowCountNeededForNumericalDisplay() );
}
