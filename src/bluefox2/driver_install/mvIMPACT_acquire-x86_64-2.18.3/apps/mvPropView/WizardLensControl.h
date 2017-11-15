//-----------------------------------------------------------------------------
#ifndef WizardLensControlH
#define WizardLensControlH WizardLensControlH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include "ValuesFromUserDlg.h"
#include "spinctld.h"

class wxCheckBox;
class wxSlider;

//-----------------------------------------------------------------------------
class WizardLensControl : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()

private:
    //-----------------------------------------------------------------------------
    enum TWidgetIDs_LensControl
    //-----------------------------------------------------------------------------
    {
        widMainFrame = widFirst,
        widSBFocus,
        widSBZoom,
        widSBIris,
        widCBIrisMode,
        widSCDriveLevel
    };
    wxStaticText*                               pSTFocus_;
    wxSpinButton*                               pSBFocus_;
    wxStaticText*                               pSTFocusDurationSlow_;
    wxSlider*                                   pSLFocusDuration_;
    wxStaticText*                               pSTFocusDurationFast_;
    wxStaticText*                               pSTZoom_;
    wxSpinButton*                               pSBZoom_;
    wxStaticText*                               pSTZoomDurationSlow_;
    wxSlider*                                   pSLZoomDuration_;
    wxStaticText*                               pSTZoomDurationFast_;
    wxStaticText*                               pSTIris_;
    wxSpinButton*                               pSBIris_;
    wxStaticText*                               pSTIrisDurationSlow_;
    wxSlider*                                   pSLIrisDuration_;
    wxStaticText*                               pSTIrisDurationFast_;
    wxStaticText*                               pSTIrisMode_;
    wxComboBox*                                 pCBIrisMode_;
    wxStaticText*                               pSTDriveLevel_;
    wxSpinCtrlDbl*                              pSCDriveLevel_;
    wxStaticText*                               pSTDriveLevelUnit_;
    bool                                        boGUICreated_;
    mvIMPACT::acquire::GenICam::mvLensControl   lc_;

    void ApplyDriveLevel( void );
    void ApplyMotorMovement( const int eventID, Method& meth );
    void CreateLensControl( wxPanel* pPanel, wxFlexGridSizer* pLensControlControlsSizer, const wxString& name, wxStaticText** ppSTControl, wxSpinButton** ppSBControl, wxWindowID SBControlID, wxStaticText** ppSTControlDurationSlow, wxSlider** ppSLControlDuration, wxStaticText** ppSTControlDurationFast );
    static const char* EventID2DriveSelectorValue( const int id );
    virtual void OnBtnCancel( wxCommandEvent& )
    {
        Hide();
    }
    virtual void OnBtnOk( wxCommandEvent& )
    {
        Hide();
    }
    void OnClose( wxCloseEvent& e );
    void OnSCDriveLevelChanged( wxSpinEvent& )
    {
        ApplyDriveLevel();
    }
    void OnSCDriveLevelTextChanged( wxCommandEvent& )
    {
        ApplyDriveLevel();
    }
    void OnIrisModeComboTextChanged( wxCommandEvent& )
    {
        try
        {
            lc_.mvIrisMode.writeS( std::string( pCBIrisMode_->GetValue().mb_str() ) );
        }
        catch( const ImpactAcquireException& e )
        {
            wxMessageBox( wxString::Format( wxT( "Failed to change iris mode(Error: %s(%s))" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION, this );
        }
    }
    void OnSpinDown( wxSpinEvent& e )
    {
        ApplyMotorMovement( e.GetId(), lc_.mvDriveBackward );
    }
    void OnSpinUp( wxSpinEvent& e )
    {
        ApplyMotorMovement( e.GetId(), lc_.mvDriveForward );
    }
    void SetupDlgControls( void );
    void SetupLensControl( const std::vector<std::string>& valid_mvDriveSelectorValues, const std::string& mvDriveSelectorValue, wxStaticText* pSTControl, wxSpinButton* pSBControl, wxStaticText* pSTControlDurationSlow, wxSlider* pSLControlDuration, wxStaticText* pSTControlDurationFast );
public:
    explicit WizardLensControl( wxWindow* pParent, const wxString& title, mvIMPACT::acquire::Device* pDev );
};

#endif // WizardLensControlH
