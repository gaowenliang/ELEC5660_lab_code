//-----------------------------------------------------------------------------
#ifndef WizardSaturationH
#define WizardSaturationH WizardSaturationH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include "ValuesFromUserDlg.h"
#include "spinctld.h"

class wxCheckBox;
class wxSlider;
class wxTextCtrl;
class wxToggleButton;

//-----------------------------------------------------------------------------
class WizardColorCorrection : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()

    mvIMPACT::acquire::ImageProcessing ip_;
    mvIMPACT::acquire::GenICam::ColorTransformationControl* pctc_;
private:
    //-----------------------------------------------------------------------------
    enum TWidgetIDs_Saturation
    //-----------------------------------------------------------------------------
    {
        widMainFrame = widFirst,
        widCBEnableInputCorrectionMatrix,
        widInputCorrectionCombo,
        widCBEnableOutputCorrectionMatrix,
        widOutputCorrectionCombo,
        widCBSaturationEnable,
        widSLSaturation,
        widSCSaturation,
        widTBSaturationDetails,
        widBtnWriteToDevice,
        widBtnWriteToDeviceAndSwitchOffHost,
        widCBEnableDeviceColorCorrection
    };
    wxCheckBox*                         pCBEnableInputCorrectionMatrix_;
    wxComboBox*                         pInputCorrectionCombo_;
    wxCheckBox*                         pCBEnableOutputCorrectionMatrix_;
    wxComboBox*                         pOutputCorrectionCombo_;
    wxCheckBox*                         pCBSaturationEnable_;
    wxSlider*                           pSLSaturation_;
    wxSpinCtrlDbl*                      pSCSaturation_;
    wxToggleButton*                     pTBSaturationDetails_;
    wxTextCtrl*                         pTCSaturationHints_;
    wxButton*                           pBtnWriteToDevice_;
    wxButton*                           pBtnWriteToDeviceAndSwitchOffHost_;
    wxCheckBox*                         pCBEnableDeviceColorCorrection_;
    wxBoxSizer*                         pTopDownSizer_;
    bool                                boGUICreated_;

    void ApplySaturation( void );
    void ApplySaturation( const bool boEnable, const double K );
    void CreateGUI( void );
    void EnableDeviceColorCorrection( const bool boEnable );
    virtual void OnBtnCancel( wxCommandEvent& )
    {
        Hide();
    }
    virtual void OnBtnOk( wxCommandEvent& )
    {
        Hide();
    }
    void OnBtnSaturationDetails( wxCommandEvent& e )
    {
        ToggleDetails( e.IsChecked() );
    }
    void OnBtnWriteToDevice( wxCommandEvent& )
    {
        WriteToDevice();
    }
    void OnBtnWriteToDeviceAndSwitchOffHost( wxCommandEvent& );
    void OnCBEnableDeviceColorCorrection( wxCommandEvent& e )
    {
        EnableDeviceColorCorrection( e.IsChecked() );
    }
    void OnCBEnableInputCorrectionMatrix( wxCommandEvent& e )
    {
        ip_.colorTwistInputCorrectionMatrixEnable.write( e.IsChecked() ? bTrue : bFalse );
        SetupDlgControls();
    }
    void OnCBEnableOutputCorrectionMatrix( wxCommandEvent& e )
    {
        ip_.colorTwistOutputCorrectionMatrixEnable.write( e.IsChecked() ? bTrue : bFalse );
        SetupDlgControls();
    }
    void OnCBSaturationEnable( wxCommandEvent& )
    {
        ApplySaturation();
        SetupDlgControls();
    }
    void OnInputCorrectionComboTextChanged( wxCommandEvent& )
    {
        ip_.colorTwistInputCorrectionMatrixMode.writeS( std::string( pInputCorrectionCombo_->GetValue().mb_str() ) );
    }
    void OnOutputCorrectionComboTextChanged( wxCommandEvent& )
    {
        ip_.colorTwistOutputCorrectionMatrixMode.writeS( std::string( pOutputCorrectionCombo_->GetValue().mb_str() ) );
    }
    void OnSCSaturationChanged( wxSpinEvent& )
    {
        pSLSaturation_->SetValue( static_cast<int>( pSCSaturation_->GetValue() * 1000. ) );
        ApplySaturation();
    }
    void OnSCSaturationTextChanged( wxCommandEvent& )
    {
        pSLSaturation_->SetValue( static_cast<int>( pSCSaturation_->GetValue() * 1000. ) );
        ApplySaturation();
    }
    void OnSLSaturation( wxScrollEvent& e )
    {
        pSCSaturation_->SetValue( static_cast<double>( e.GetPosition() ) / 1000. );
        ApplySaturation();
    }
    void SetupDlgControls();
    void ToggleDetails( bool boShow );
    bool WriteToDevice( void );
public:
    explicit WizardColorCorrection( wxWindow* pParent, const wxString& title, mvIMPACT::acquire::Device* pDev );
    ~WizardColorCorrection();
    void RefreshControls( void );
};

#endif // WizardSaturationH
