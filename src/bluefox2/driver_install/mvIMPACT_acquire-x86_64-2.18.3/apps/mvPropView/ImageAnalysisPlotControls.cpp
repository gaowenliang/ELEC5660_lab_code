#include "ImageAnalysisPlotControls.h"
#include "PlotCanvasImageAnalysis.h"
#include "HistogramCanvas.h"
#include <wx/config.h>
#include <wx/notebook.h>
#include <wx/spinctrl.h>
#include <wx/string.h>

using namespace std;

//=============================================================================
//================= Implementation ImageAnalysisPlotControls ==================
//=============================================================================
//-----------------------------------------------------------------------------
void ImageAnalysisPlotControls::Load( wxConfigBase* pConfig )
//-----------------------------------------------------------------------------
{
    bool boEnable = pConfig->Read( wxString::Format( wxT( "/Controls/%s/AOIFullMode" ), m_pPlotCanvas->GetConfigName().c_str() ), 0l ) != 0;
    m_pCBAOIFullMode->SetValue( boEnable );
    m_pPlotCanvas->SetAOIFullMode( boEnable );
    if( m_pCBProcessBayerParity )
    {
        boEnable = pConfig->Read( wxString::Format( wxT( "/Controls/%s/ProcessBayerParity" ), m_pPlotCanvas->GetConfigName().c_str() ), 1l ) != 0;
        m_pCBProcessBayerParity->SetValue( boEnable );
        m_pPlotCanvas->SetProcessBayerParity( boEnable );
    }
    m_pSCAOIh->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/AOIh" ), m_pPlotCanvas->GetConfigName().c_str() ), 480l ) );
    m_pSCAOIw->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/AOIw" ), m_pPlotCanvas->GetConfigName().c_str() ), 640l ) );
    m_pSCAOIx->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/AOIx" ), m_pPlotCanvas->GetConfigName().c_str() ), 0l ) );
    m_pSCAOIy->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/AOIy" ), m_pPlotCanvas->GetConfigName().c_str() ), 0l ) );
    if( m_pCoBPlotSelection )
    {
        wxString plotSelection( pConfig->Read( wxString::Format( wxT( "/Controls/%s/PlotSelection" ), m_pPlotCanvas->GetConfigName().c_str() ), wxT( "" ) ) );
        const vector<wxString>& v( m_pPlotCanvas->GetAvailablePlotSelections() );
        const vector<wxString>::size_type cnt = v.size();
        for( vector<wxString>::size_type i = 0; i < cnt; i++ )
        {
            if( v[i] == plotSelection )
            {
                m_pCoBPlotSelection->Select( static_cast<int>( i ) );
                m_pPlotCanvas->SetPlotSelection( plotSelection );
            }
        }
    }
    if( m_pSCHistoryDepth )
    {
        m_pSCHistoryDepth->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/HistoryDepth" ), m_pPlotCanvas->GetConfigName().c_str() ), 0l ) );
        m_pPlotCanvas->SetHistoryDepth( m_pSCHistoryDepth->GetValue() );
    }
    if( m_pSCDrawStart_percent )
    {
        m_pSCDrawStart_percent->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/DrawStartPercent" ), m_pPlotCanvas->GetConfigName().c_str() ), 0l ) );
        m_pPlotCanvas->SetDrawStartPercent( m_pSCDrawStart_percent->GetValue() );
    }
    if( m_pSCDrawWindow_percent )
    {
        m_pSCDrawWindow_percent->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/DrawWindowPercent" ), m_pPlotCanvas->GetConfigName().c_str() ), 100l ) );
        m_pPlotCanvas->SetDrawWindowWidthPercent( m_pSCDrawWindow_percent->GetValue() );
    }
    if( m_pSCDrawStepWidth )
    {
        m_pSCDrawStepWidth->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/DrawStepWidth" ), m_pPlotCanvas->GetConfigName().c_str() ), 0l ) );
        if( dynamic_cast<HistogramCanvas*>( m_pPlotCanvas ) != 0 )
        {
            dynamic_cast<HistogramCanvas*>( m_pPlotCanvas )->SetDrawStepWidth( m_pSCDrawStepWidth->GetValue() );
        }
    }
    if( m_pPlotCanvas->HasFeature( PlotCanvasImageAnalysis::pfGridSteps ) )
    {
        m_pSCGridStepsX->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/GridStepsX" ), m_pPlotCanvas->GetConfigName().c_str() ), 1l ) );
        m_pSCGridStepsY->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/GridStepsY" ), m_pPlotCanvas->GetConfigName().c_str() ), 1l ) );
        m_pPlotCanvas->SetGridSteps( m_pSCGridStepsX->GetValue(), m_pSCGridStepsY->GetValue() );
    }
    m_pSCUpdateSpeed->SetValue( pConfig->Read( wxString::Format( wxT( "/Controls/%s/UpdateSpeed" ), m_pPlotCanvas->GetConfigName().c_str() ), 3l ) );
    PlotCanvasImageAnalysis::TDisplayMethod displayMethod = static_cast<PlotCanvasImageAnalysis::TDisplayMethod>( pConfig->Read( wxString::Format( wxT( "/Controls/%s/DisplayMethod" ), m_pPlotCanvas->GetConfigName().c_str() ), 0l ) );
    m_pNBDisplayMethod->SetSelection( displayMethod );
    m_pPlotCanvas->SetDisplayMethod( displayMethod );
    UpdateControls();
}

//-----------------------------------------------------------------------------
void ImageAnalysisPlotControls::Save( wxConfigBase* pConfig )
//-----------------------------------------------------------------------------
{
    pConfig->Write( wxString::Format( wxT( "/Controls/%s/AOIFullMode" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pCBAOIFullMode->GetValue() );
    if( m_pCBProcessBayerParity )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/ProcessBayerParity" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pCBProcessBayerParity->GetValue() );
    }
    pConfig->Write( wxString::Format( wxT( "/Controls/%s/AOIh" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCAOIh->GetValue() );
    pConfig->Write( wxString::Format( wxT( "/Controls/%s/AOIw" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCAOIw->GetValue() );
    pConfig->Write( wxString::Format( wxT( "/Controls/%s/AOIx" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCAOIx->GetValue() );
    pConfig->Write( wxString::Format( wxT( "/Controls/%s/AOIy" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCAOIy->GetValue() );
    if( m_pCoBPlotSelection )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/PlotSelection" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pCoBPlotSelection->GetValue() );
    }
    if( m_pSCHistoryDepth )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/HistoryDepth" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCHistoryDepth->GetValue() );
    }
    if( m_pSCDrawStart_percent )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/DrawStartPercent" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCDrawStart_percent->GetValue() );
    }
    if( m_pSCDrawWindow_percent )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/DrawWindowPercent" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCDrawWindow_percent->GetValue() );
    }
    if( m_pSCDrawStepWidth )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/DrawStepWidth" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCDrawStepWidth->GetValue() );
    }
    if( m_pSCGridStepsX )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/GridStepsX" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCGridStepsX->GetValue() );
    }
    if( m_pSCGridStepsY )
    {
        pConfig->Write( wxString::Format( wxT( "/Controls/%s/GridStepsY" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCGridStepsY->GetValue() );
    }
    pConfig->Write( wxString::Format( wxT( "/Controls/%s/UpdateSpeed" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pSCUpdateSpeed->GetValue() );
    pConfig->Write( wxString::Format( wxT( "/Controls/%s/DisplayMethod" ), m_pPlotCanvas->GetConfigName().c_str() ), m_pNBDisplayMethod->GetSelection() );
}

//-----------------------------------------------------------------------------
void ImageAnalysisPlotControls::UpdateControls( void )
//-----------------------------------------------------------------------------
{
    bool boFullAOIMode = m_pCBAOIFullMode->IsChecked();
    m_pSCAOIx->Enable( !boFullAOIMode );
    m_pSCAOIy->Enable( !boFullAOIMode );
    m_pSCAOIw->Enable( !boFullAOIMode );
    m_pSCAOIh->Enable( !boFullAOIMode );
}
