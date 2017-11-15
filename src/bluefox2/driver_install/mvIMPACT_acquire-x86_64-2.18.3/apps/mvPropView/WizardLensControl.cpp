#include <algorithm>
#include <apps/Common/wxAbstraction.h>
#include "WizardLensControl.h"
#include <wx/slider.h>

using namespace std;

//=============================================================================
//============== Implementation WizardLensControl =============================
//=============================================================================
BEGIN_EVENT_TABLE( WizardLensControl, OkAndCancelDlg )
    EVT_SPIN_UP( widSBFocus, WizardLensControl::OnSpinUp )
    EVT_SPIN_DOWN( widSBFocus, WizardLensControl::OnSpinDown )
    EVT_SPIN_UP( widSBZoom, WizardLensControl::OnSpinUp )
    EVT_SPIN_DOWN( widSBZoom, WizardLensControl::OnSpinDown )
    EVT_SPIN_UP( widSBIris, WizardLensControl::OnSpinUp )
    EVT_SPIN_DOWN( widSBIris, WizardLensControl::OnSpinDown )
    EVT_TEXT( widCBIrisMode, WizardLensControl::OnIrisModeComboTextChanged )
    EVT_SPINCTRL( widSCDriveLevel, WizardLensControl::OnSCDriveLevelChanged )
#ifdef BUILD_WITH_TEXT_EVENTS_FOR_SPINCTRL // Unfortunately on Linux wxWidgets 2.6.x - ??? handling these messages will cause problems, while on Windows not doing so will not always update the GUI as desired :-(
    EVT_TEXT( widSCDriveLevel, WizardLensControl::OnSCDriveLevelTextChanged )
#endif // #ifdef BUILD_WITH_TEXT_EVENTS_FOR_SPINCTRL
    EVT_CLOSE( WizardLensControl::OnClose )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
WizardLensControl::WizardLensControl( wxWindow* pParent, const wxString& title, mvIMPACT::acquire::Device* pDev )
    : OkAndCancelDlg( pParent, widMainFrame, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxMINIMIZE_BOX ),
      pSTFocus_( 0 ), pSBFocus_( 0 ), pSTFocusDurationSlow_( 0 ), pSLFocusDuration_( 0 ), pSTFocusDurationFast_( 0 ),
      pSTZoom_( 0 ), pSBZoom_( 0 ), pSTZoomDurationSlow_( 0 ), pSLZoomDuration_( 0 ), pSTZoomDurationFast_( 0 ),
      pSTIris_( 0 ), pSBIris_( 0 ), pSTIrisDurationSlow_( 0 ), pSLIrisDuration_( 0 ), pSTIrisDurationFast_( 0 ),
      pSTIrisMode_( 0 ), pCBIrisMode_( 0 ), pSTDriveLevel_( 0 ), pSCDriveLevel_( 0 ), pSTDriveLevelUnit_( 0 ),
      boGUICreated_( false ), lc_( pDev )
//-----------------------------------------------------------------------------
{
    /*
        |-------------------------------------|
        | pTopDownSizer                       |
        |                spacer               |
        | |---------------------------------| |
        | | lens control controls           | |
        | |---------------------------------| |
        |                spacer               |
        | |---------------------------------| |
        | | pButtonSizer                    | |
        | |---------------------------------| |
        |-------------------------------------|
    */

    wxPanel* pPanel = new wxPanel( this );
    wxFlexGridSizer* pLensControlControlsSizer = new wxFlexGridSizer( 5 );
    pLensControlControlsSizer->AddGrowableCol( 3, 2 );

    CreateLensControl( pPanel, pLensControlControlsSizer, wxT( "Focus" ), &pSTFocus_, &pSBFocus_, widSBFocus, &pSTFocusDurationSlow_, &pSLFocusDuration_, &pSTFocusDurationFast_ );
    CreateLensControl( pPanel, pLensControlControlsSizer, wxT( "Zoom" ), &pSTZoom_, &pSBZoom_, widSBZoom, &pSTZoomDurationSlow_, &pSLZoomDuration_, &pSTZoomDurationFast_ );
    CreateLensControl( pPanel, pLensControlControlsSizer, wxT( "Iris" ), &pSTIris_, &pSBIris_, widSBIris, &pSTIrisDurationSlow_, &pSLIrisDuration_, &pSTIrisDurationFast_ );

    pSTDriveLevel_ = new wxStaticText( pPanel, wxID_ANY, wxT( " Drive Level: " ) );
    const double driveLevelValue = lc_.mvDriveLevel.isValid() ? static_cast<double>( lc_.mvDriveLevel.read() ) : 1.;
    const double driveLevelValue_Max = lc_.mvDriveLevel.isValid() ? static_cast<double>( lc_.mvDriveLevel.getMaxValue() ) : 10.;
    const double driveLevelValue_Min = lc_.mvDriveLevel.isValid() ? static_cast<double>( lc_.mvDriveLevel.getMinValue() ) : -10.;
    pSCDriveLevel_ = new wxSpinCtrlDbl();
    pSCDriveLevel_->Create( pPanel, widSCDriveLevel, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, driveLevelValue_Min, driveLevelValue_Max, driveLevelValue, 1., wxSPINCTRLDBL_AUTODIGITS, wxT( "%.1f" ) );
    pSCDriveLevel_->SetMode( mDouble );
    if( lc_.mvDriveLevel.isValid() )
    {
        pSCDriveLevel_->SetToolTip( ConvertedString( lc_.mvDriveLevel.docString() ) );
    }
    pSTDriveLevelUnit_ = new wxStaticText( pPanel, wxID_ANY, wxT( "  mV" ) );

    wxFlexGridSizer* pLensControlDriveLevelGridSizer = new wxFlexGridSizer( 3 );
    pLensControlDriveLevelGridSizer->AddGrowableCol( 1, 3 );
    pLensControlDriveLevelGridSizer->Add( pSTDriveLevel_ );
    pLensControlDriveLevelGridSizer->Add( pSCDriveLevel_, wxSizerFlags( 3 ).Expand() );
    pLensControlDriveLevelGridSizer->Add( pSTDriveLevelUnit_ );

    wxBoxSizer* pLensControlSizer = new wxStaticBoxSizer( wxVERTICAL, pPanel, wxT( "Lens Controls: " ) );
    pLensControlSizer->AddSpacer( 5 );
    pLensControlSizer->Add( pLensControlControlsSizer, 2, 0, 5 );
    pLensControlSizer->AddSpacer( 10 );
    pLensControlSizer->Add( pLensControlDriveLevelGridSizer, 0, 0, 5 );
    pLensControlSizer->AddSpacer( 5 );

    wxBoxSizer* pLensControlVideoIrisSizer = 0;
    wxString mvIrisMode;
    wxArrayString mvIrisModes;
    if( lc_.mvIrisMode.isValid() )
    {
        mvIrisMode = ConvertedString( lc_.mvIrisMode.readS() );
        vector<string> mvIrisModeStrings;
        lc_.mvIrisMode.getTranslationDictStrings( mvIrisModeStrings );
        const vector<string>::size_type mvIrisModeCount = mvIrisModeStrings.size();
        for( vector<string>::size_type i = 0; i < mvIrisModeCount; i++ )
        {
            mvIrisModes.Add( ConvertedString( mvIrisModeStrings[i] ) );
        }
    }

    pSTIrisMode_ = new wxStaticText( pPanel, wxID_ANY, wxT( " Iris Mode: " ) );
    pCBIrisMode_ = new wxComboBox( pPanel, widCBIrisMode, mvIrisMode, wxDefaultPosition, wxSize( 120, wxDefaultCoord ), mvIrisModes, wxCB_DROPDOWN | wxCB_READONLY );
    pCBIrisMode_->SetToolTip( ConvertedString( lc_.mvIrisMode.docString() ) );

    wxFlexGridSizer* pLensControlVideoIrisGridSizer = new wxFlexGridSizer( 2 );
    pLensControlVideoIrisGridSizer->AddGrowableCol( 1, 3 );
    pLensControlVideoIrisGridSizer->Add( pSTIrisMode_ );
    pLensControlVideoIrisGridSizer->Add( pCBIrisMode_, wxSizerFlags( 3 ).Expand() );

    pLensControlVideoIrisSizer = new wxStaticBoxSizer( wxVERTICAL, pPanel, wxT( "Video Iris: " ) );
    pLensControlVideoIrisSizer->AddSpacer( 5 );
    pLensControlVideoIrisSizer->Add( pLensControlVideoIrisGridSizer, wxSizerFlags().Expand() );
    pLensControlVideoIrisSizer->AddSpacer( 5 );

    wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    pTopDownSizer->AddSpacer( 10 );
    pTopDownSizer->Add( pLensControlSizer, wxSizerFlags( 2 ).Expand() );
    pTopDownSizer->AddSpacer( 5 );
    pTopDownSizer->Add( pLensControlVideoIrisSizer, wxSizerFlags().Expand() );
    pTopDownSizer->AddSpacer( 10 );
    AddButtons( pPanel, pTopDownSizer, false );

    wxBoxSizer* pOuterSizer = new wxBoxSizer( wxHORIZONTAL );
    pOuterSizer->AddSpacer( 5 );
    pOuterSizer->Add( pTopDownSizer, wxSizerFlags().Expand() );
    pOuterSizer->AddSpacer( 5 );

    FinalizeDlgCreation( pPanel, pOuterSizer );
    boGUICreated_ = true;

    SetupDlgControls();
}

//-----------------------------------------------------------------------------
void WizardLensControl::ApplyDriveLevel( void )
//-----------------------------------------------------------------------------
{
    try
    {
        lc_.mvDriveLevel.write( static_cast<int64_type>( pSCDriveLevel_->GetValue() ) );
    }
    catch( const ImpactAcquireException& e )
    {
        wxMessageBox( wxString::Format( wxT( "Failed to apply motor drive level(Error: %s(%s))" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION, this );
    }
}

//-----------------------------------------------------------------------------
void WizardLensControl::ApplyMotorMovement( const int eventID, Method& meth )
//-----------------------------------------------------------------------------
{
    try
    {
        lc_.mvDriveSelector.writeS( string( EventID2DriveSelectorValue( eventID ) ) );
        int64_type fullDuration = 0;
        switch( eventID )
        {
        case widSBFocus:
            fullDuration = static_cast<int64_type>( pSLFocusDuration_->GetValue() );
            break;
        case widSBZoom:
            fullDuration = static_cast<int64_type>( pSLZoomDuration_->GetValue() );
            break;
        case widSBIris:
            fullDuration = static_cast<int64_type>( pSLIrisDuration_->GetValue() );
            break;
        default:
            break;
        }
        if( fullDuration > 0 )
        {
            const int64_type maxDurationPerPulse = lc_.mvDriveDuration.getMaxValue();
            int64_type durationApplied = 0;
            while( durationApplied < fullDuration )
            {
                const int64_type duration = ( ( fullDuration - durationApplied ) >= maxDurationPerPulse ) ? maxDurationPerPulse : fullDuration - durationApplied;
                lc_.mvDriveDuration.write( duration );
                meth.call();
                durationApplied += duration;
            }
        }
    }
    catch( const ImpactAcquireException& e )
    {
        wxMessageBox( wxString::Format( wxT( "Failed to apply motor movement(Error: %s(%s))" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION, this );
    }
}

//-----------------------------------------------------------------------------
void WizardLensControl::CreateLensControl( wxPanel* pPanel, wxFlexGridSizer* pLensControlControlsSizer, const wxString& name, wxStaticText** ppSTControl, wxSpinButton** ppSBControl, wxWindowID SBControlID, wxStaticText** ppSTControlDurationSlow, wxSlider** ppSLControlDuration, wxStaticText** ppSTControlDurationFast )
//-----------------------------------------------------------------------------
{
    *ppSTControl = new wxStaticText( pPanel, wxID_ANY, wxString::Format( wxT( " %s: " ), name.c_str() ) );
    pLensControlControlsSizer->Add( *ppSTControl );
    *ppSBControl = new wxSpinButton( pPanel, SBControlID, wxPoint( 0, 0 ), wxDefaultSize, wxSP_ARROW_KEYS | wxSP_HORIZONTAL | wxSP_WRAP );
    pLensControlControlsSizer->Add( *ppSBControl );
    *ppSTControlDurationSlow = new wxStaticText( pPanel, wxID_ANY, wxT( "   Slow " ) );
    pLensControlControlsSizer->Add( *ppSTControlDurationSlow );
    *ppSLControlDuration = new wxSlider( pPanel, wxID_ANY, 50000, 0, 50000, wxDefaultPosition, wxSize( 150, -1 ), wxSL_HORIZONTAL );
    pLensControlControlsSizer->Add( *ppSLControlDuration, wxSizerFlags().Expand() );
    *ppSTControlDurationFast = new wxStaticText( pPanel, wxID_ANY, wxT( " Fast " ) );
    pLensControlControlsSizer->Add( *ppSTControlDurationFast );
}

//-----------------------------------------------------------------------------
const char* WizardLensControl::EventID2DriveSelectorValue( const int id )
//-----------------------------------------------------------------------------
{
    switch( id )
    {
    case widSBFocus:
        return "mvFocus";
    case widSBZoom:
        return "mvZoom";
    case widSBIris:
        return "mvIris";
    default:
        break;
    }
    return "INVALID_EVENT_ID";
}

//-----------------------------------------------------------------------------
void WizardLensControl::OnClose( wxCloseEvent& e )
//-----------------------------------------------------------------------------
{
    Hide();
    if( e.CanVeto() )
    {
        e.Veto();
    }
}

//-----------------------------------------------------------------------------
void WizardLensControl::SetupDlgControls( void )
//-----------------------------------------------------------------------------
{
    vector<string> mvDriveSelectorStrings;
    if( lc_.mvDriveSelector.isValid() )
    {
        lc_.mvDriveSelector.getTranslationDictStrings( mvDriveSelectorStrings );
    }

    SetupLensControl( mvDriveSelectorStrings, "mvFocus", pSTFocus_, pSBFocus_, pSTFocusDurationSlow_, pSLFocusDuration_, pSTFocusDurationFast_ );
    SetupLensControl( mvDriveSelectorStrings, "mvZoom", pSTZoom_, pSBZoom_, pSTZoomDurationSlow_, pSLZoomDuration_, pSTZoomDurationFast_ );
    SetupLensControl( mvDriveSelectorStrings, "mvIris", pSTIris_, pSBIris_, pSTIrisDurationSlow_, pSLIrisDuration_, pSTIrisDurationFast_ );

    const bool boDriveLevelAvailable = lc_.mvDriveLevel.isValid();
    pSTDriveLevel_->Enable( boDriveLevelAvailable );
    pSCDriveLevel_->Enable( boDriveLevelAvailable );
    pSTDriveLevelUnit_->Enable( boDriveLevelAvailable );

    const bool boIrisModeAvailable = lc_.mvIrisMode.isValid();
    pSTIrisMode_->Enable( boIrisModeAvailable );
    pCBIrisMode_->Enable( boIrisModeAvailable );
}

//-----------------------------------------------------------------------------
void WizardLensControl::SetupLensControl( const vector<string>& valid_mvDriveSelectorValues, const string& mvDriveSelectorValue, wxStaticText* pSTControl, wxSpinButton* pSBControl, wxStaticText* pSTControlDurationSlow, wxSlider* pSLControlDuration, wxStaticText* pSTControlDurationFast )
//-----------------------------------------------------------------------------
{
    const bool boControlSupported = find( valid_mvDriveSelectorValues.begin(), valid_mvDriveSelectorValues.end(), mvDriveSelectorValue ) != valid_mvDriveSelectorValues.end();
    pSTControl->Enable( boControlSupported );
    pSBControl->Enable( boControlSupported );
    pSTControlDurationSlow->Enable( boControlSupported );
    pSLControlDuration->Enable( boControlSupported );
    pSTControlDurationFast->Enable( boControlSupported );
}
