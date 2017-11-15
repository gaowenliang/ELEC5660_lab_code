//-----------------------------------------------------------------------------
#ifndef WizardQuickSetupH
#define WizardQuickSetupH WizardQuickSetupH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include "ValuesFromUserDlg.h"
#include "spinctld.h"
#include "PropViewFrame.h"

class wxCheckBox;
class wxSlider;
class wxTextCtrl;
class wxToggleButton;

//-----------------------------------------------------------------------------
class WizardQuickSetup : public OkAndCancelDlg
    //-----------------------------------------------------------------------------
{
protected:
    //-----------------------------------------------------------------------------
    enum TWhiteBalanceChannel
    //-----------------------------------------------------------------------------
    {
        wbcRed,
        wbcBlue
    };
    //-----------------------------------------------------------------------------
    enum TOptimizationLevel
    //-----------------------------------------------------------------------------
    {
        olBestQuality,
        olHighestSpeed
    };
    void                                UpdateFramerateWarningBitmap( bool boIsFrameRateMax );
public:
    explicit                            WizardQuickSetup( PropViewFrame* pParent, const wxString& title );
    void                                ReferToNewDevice( Device* pDev );
    void                                RestoreWizardConfiguration( void );
    void                                SaveWizardConfiguration( void );
    void                                ShowImageTimeoutPopup( void );
    void                                UpdateControlsData( void );
    bool                                IsGenICam( void )
    {
        return pDev_->interfaceLayout.read() == dilGenICam;
    }

    //-----------------------------------------------------------------------------
    struct DeviceSettings
            //-----------------------------------------------------------------------------
    {
        double                          exposureTime;
        bool                            boAutoExposureEnabled;
        double                          unifiedGain;
        bool                            boAutoGainEnabled;
        double                          unifiedBlackLevel;
        bool                            boGammaEnabled;
        double                          whiteBalanceRed;
        double                          whiteBalanceBlue;
        bool                            boAutoWhiteBalanceEnabled;
        double                          saturation;
        bool                            boCCMEnabled;
        double                          frameRate;
        double                          maxFrameRateAtCurrentAOI;
        double                          maxAllowedHQFrameRateAtCurrentAOI;
        double                          autoExposureUpperLimit;
        bool                            boMaximumFrameRateEnabled;

        double                          analogGainMin;
        double                          analogGainMax;
        double                          digitalGainMin;
        double                          digitalGainMax;
        double                          analogBlackLevelMin;
        double                          analogBlackLevelMax;
        double                          digitalBlackLevelMin;
        double                          digitalBlackLevelMax;

        bool                            boColorEnabled;
        bool                            boDeviceSpecificOverlappedExposure;
        bool                            boSettingsHaveNotYetBeenSavedEvenOnce;
        TOptimizationLevel              optimizationLevel;
        std::string                     imageFormatControlPixelFormat;
        TImageDestinationPixelFormat    imageDestinationPixelFormat;
        std::string                     pixelClock;
    };

    //-----------------------------------------------------------------------------
    struct SupportedWizardFeatures
            //-----------------------------------------------------------------------------
    {
        bool                            boGainSupport;
        bool                            boBlackLevelSupport;
        bool                            boAutoExposureSupport;
        bool                            boAutoGainSupport;
        bool                            boAutoWhiteBalanceSupport;
        bool                            boMaximumFrameRateSupport;
        bool                            boRegulateFrameRateSupport;
        bool                            boColorOptionsSupport;
        bool                            boPlatformIPPSuport;
    };

    std::string                         currentProductString_;
    std::string                         currentDeviceSerial_;
    unsigned int                        currentDeviceFWVersion_;
    std::map<std::string, DeviceSettings> currentSettings_;
    std::map<std::string, SupportedWizardFeatures> featuresSupported_;

    virtual void                        CreateInterfaceLayoutSpecificControls( Device* pDev ) = 0;
    virtual void                        DeleteInterfaceLayoutSpecificControls( void ) = 0;
    virtual double                      DetermineMaxFrameRateAtCurrentAOI( void ) = 0;
    virtual double                      DetermineOptimalHQFrameRateAtCurrentAOI( void ) = 0;
    virtual void                        DoConfigureFrameRateMaximum( bool /*boActive*/, double /*frameRateValue*/ ) {}
    virtual double                      DoReadUnifiedBlackLevel( void ) const = 0;
    virtual double                      DoReadUnifiedGain( void ) const = 0;
    virtual void                        DoSetAcquisitionFrameRateLimitMode( void ) {}
    virtual void                        DoWriteUnifiedGain( double value ) const = 0;
    virtual void                        DoWriteUnifiedBlackLevelData( double value ) const = 0;
    void                                DoSetupFrameRateControls( double frameRateRangeMin, double frameRateRangeMax, double frameRate );
    void                                DoSetupExposureControls( double exposureMin, double exposureMax, double exposure, bool boHasStepWidth, double increment );
    void                                DoSetupGainControls( double gainUnifiedRangeMin, double gainUnifiedRangeMax, double gain );
    void                                DoSetupBlackLevelControls( double blackLevelUnifiedRangeMin, double blackLevelUnifiedRangeMax, double blackLevel );
    void                                DoSetupWhiteBalanceControls( double whiteBalanceRMin, double whiteBalanceRMax, double whiteBalanceR, double whiteBalanceBMin, double whiteBalanceBMax, double whiteBalanceB );
    TOptimizationLevel                  GetOptimizationLevel( void ) const
    {
        return static_cast< TOptimizationLevel >( pSLOptimization_->GetValue() );
    }
    virtual double                      GetAutoExposureUpperLimit( void ) const = 0;
    virtual double                      GetExposureTime( void ) const = 0;
    virtual bool                        GetExposureOverlapped() const = 0;
    virtual bool                        GetFrameRateEnable( void ) const
    {
        return false;
    }
    std::string                         GetFullPropertyName( Property prop ) const;
    virtual std::string                 GetPixelClock( void ) const = 0;
    virtual std::string                 GetPixelFormat( void ) const = 0;
    virtual double                      GetWhiteBalance( TWhiteBalanceChannel channel ) = 0;
    virtual bool                        HasAEC( void ) const = 0;
    virtual bool                        HasAGC( void ) = 0;
    virtual bool                        HasAWB( void ) const = 0;
    virtual bool                        HasMaximumFrameRate( void ) const = 0;
    virtual bool                        HasBlackLevel( void ) const = 0;
    virtual bool                        HasColorFormat( void ) const = 0;
    virtual bool                        HasFactoryDefault( void ) const = 0;
    virtual bool                        HasFrameRateEnable( void ) const
    {
        return false;
    }
    virtual bool                        HasGain( void ) const = 0;
    virtual bool                        HasUnifiedGain( void ) const
    {
        return false;
    }
    bool                                HasPlatformIPPSupport( void ) const
    {
        return pIP_->colorTwistEnable.isValid();
    }

    virtual void                        InitializeExposureParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features ) = 0;
    virtual void                        InitializeGainParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features ) = 0;
    virtual void                        InitializeBlackLevelParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features ) = 0;
    virtual void                        InitializeWhiteBalanceParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features ) = 0;
    virtual void                        QueryInterfaceLayoutSpecificSettings( DeviceSettings& devSettings ) = 0;
    virtual void                        RestoreFactoryDefault( void ) = 0;
    virtual void                        SelectOptimalPixelFormatColor( bool boHighQuality ) = 0;
    virtual void                        SelectOptimalPixelFormatGray( bool boHighQuality ) = 0;
    virtual void                        SelectOptimalAutoExposureSettings( bool boHighQuality ) = 0;
    virtual void                        SelectOptimalPixelClock( bool boHighQuality ) = 0;
    virtual void                        SetExposureTime( double value ) = 0;
    virtual void                        SetExposureOverlapped( bool /* boEnable */ ) {}
    virtual void                        SetAutoExposure( bool boEnable ) = 0;
    virtual void                        SetAutoExposureUpperLimit( double exposureMax ) = 0;
    virtual void                        SetAutoGain( bool boEnable ) = 0;
    virtual void                        SetAutoWhiteBalance( bool /* boEnable */ ) {}
    virtual void                        SetupExposureControls( void ) = 0;
    virtual void                        SetupGainControls( void ) = 0;
    virtual void                        SetupBlackLevelControls( void ) = 0;
    virtual void                        SetupWhiteBalanceControls( void ) = 0;
    virtual void                        SetupUnifiedGainData( void ) = 0;
    virtual void                        SetupUnifiedBlackLevelData( void ) = 0;
    virtual void                        SetupFrameRateControls( void ) {}
    virtual void                        SetFrameRateEnable( bool /* boOn */ ) {}
    virtual void                        SetFrameRate( double /* value */ ) {}
    virtual void                        SetPixelClock( const std::string& frequency ) = 0;
    virtual void                        SetPixelFormat( const std::string& format ) = 0;
    virtual void                        SetWhiteBalance( TWhiteBalanceChannel channel, double value ) = 0;
    virtual void                        TryToReadFrameRate( double& /*value*/ ) = 0;
    virtual void                        WriteExposureFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features ) = 0;
    virtual void                        WriteGainFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features ) = 0;
    virtual void                        WriteWhiteBalanceFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features ) = 0;
    void                                WriteQuickSetupWizardErrorMessage( const wxString& msg ) const
    {
        pParentPropViewFrame_->WriteErrorMessage( wxString( wxT( "Quick Setup Wizard: " ) + msg + wxT( "\n" ) ) );
    }
    void                                WriteQuickSetupWizardLogMessage( const wxString& msg ) const
    {
        pParentPropViewFrame_->WriteLogMessage( wxString( wxT( "Quick Setup Wizard: " ) + msg + wxT( "\n" ) ), wxTextAttr( *wxBLACK ) );
    }
private:
    //-----------------------------------------------------------------------------
    enum TWidgetIDs_QuickSetup
    //-----------------------------------------------------------------------------
    {
        widMainFrame = wxID_HIGHEST,
        widBtnPresetColor,
        widBtnPresetGray,
        widStaticBitmapQuality,
        widStaticBitmapSpeed,
        widStaticBitmapWarning,
        widSLOptimization,
        widBtnPresetFactory,
        widSLExposure,
        widSCExposure,
        widBtnExposureAuto,
        widSLGain,
        widSCGain,
        widBtnGainAuto,
        widSLBlackLevel,
        widSCBlackLevel,
        widBtnGamma,
        widSLSaturation,
        widSCSaturation,
        widBtnCCM,
        widSLWhiteBalanceR,
        widSCWhiteBalanceR,
        widSLWhiteBalanceB,
        widSCWhiteBalanceB,
        widBtnWhiteBalanceAuto,
        widSLFrameRate,
        widSCFrameRate,
        widBtnFrameRateMaximum,
        widCBShowDialogAtStartup,
        widBtnHistory,
        widBtnClearHistory,
        widTCHistory
    };

    //----------------------------------STATICS------------------------------------
    static const double                 EXPOSURE_SLIDER_GAMMA_;
    static const double                 SLIDER_GRANULARITY_;
    static const double                 GAMMA_CORRECTION_VALUE_;

    //----------------------------------GUI ELEMENTS-------------------------------
    wxBoxSizer*                         pTopDownSizer_;
    wxBitmapButton*                     pBtnPresetColor_;
    wxBitmapButton*                     pBtnPresetFactory_;
    wxBitmapButton*                     pBtnPresetGray_;
    wxSlider*                           pSLOptimization_;
    wxStaticBitmap*                     pStaticBitmapQuality_;
    wxStaticBitmap*                     pStaticBitmapSpeed_;
    wxStaticBitmap*                     pStaticBitmapWarning_;
    wxSlider*                           pSLExposure_;
    wxSpinCtrlDbl*                      pSCExposure_;
    wxToggleButton*                     pBtnExposureAuto_;
    wxSlider*                           pSLGain_;
    wxSpinCtrlDbl*                      pSCGain_;
    wxToggleButton*                     pBtnGainAuto_;
    wxSlider*                           pSLBlackLevel_;
    wxSpinCtrlDbl*                      pSCBlackLevel_;
    wxToggleButton*                     pBtnGamma_;
    wxSlider*                           pSLWhiteBalanceR_;
    wxSpinCtrlDbl*                      pSCWhiteBalanceR_;
    wxSlider*                           pSLWhiteBalanceB_;
    wxSpinCtrlDbl*                      pSCWhiteBalanceB_;
    wxToggleButton*                     pBtnWhiteBalanceAuto_;
    wxSlider*                           pSLSaturation_;
    wxSpinCtrlDbl*                      pSCSaturation_;
    wxToggleButton*                     pBtnCCM_;
    wxSlider*                           pSLFrameRate_;
    wxSpinCtrlDbl*                      pSCFrameRate_;
    wxToggleButton*                     pBtnFrameRateMaximum_;
    wxStaticText*                       pFrameRateControlStaticText_;
    wxCheckBox*                         pCBShowDialogAtStartup_;
    wxToggleButton*                     pBtnHistory_;
    wxButton*                           pBtnClearHistory_;
    wxTextCtrl*                         pTCHistory_;
    wxBoxSizer*                         pInfoSizer_;
    bool                                boGUILocked_;

protected:
    Device*                             pDev_;
    ImageProcessing*                    pIP_;
    double                              analogGainMin_;
    double                              analogGainMax_;
    double                              digitalGainMin_;
    double                              digitalGainMax_;
    double                              analogBlackLevelMin_;
    double                              analogBlackLevelMax_;
    double                              digitalBlackLevelMin_;
    double                              digitalBlackLevelMax_;
    TWhiteBalanceChannel                boLastWBChannelRead_;

    template<class _Ty, typename _Tx>
    void                                LoggedWriteProperty( _Ty& prop, const _Tx& value ) const
    {
        prop.write( value );
        std::ostringstream oss;
        oss << "Property " << GetFullPropertyName( prop ) << " set to " << prop.readS() << std::endl;
        propertyChangeHistory_ += oss.str();
    }
    template<class _Ty>
    void                                LoggedWriteProperty( _Ty& prop, const std::string& value ) const
    {
        prop.writeS( value );
        std::ostringstream oss;
        oss << "Property " << GetFullPropertyName( prop ) << " set to " << prop.readS() << std::endl;
        propertyChangeHistory_ += oss.str();
    }

private:
    std::map<std::string, DeviceSettings> propGridSettings_;
    PropViewFrame*                      pParentPropViewFrame_;
    mutable std::string                 propertyChangeHistory_;
    ImageDestination*                   pID_;

    //----------------------------------GENERAL------------------------------------
    void                                AnalyzeDeviceAndGatherInformation( SupportedWizardFeatures& wizardSupportedFeatures );
    void                                ApplyPreset( bool boColor, TOptimizationLevel optimizationLevel );
    void                                CleanUp( void );
    void                                CloseDlg( void );
    void                                HandlePresetChanges( void );
    void                                OnCBShowOnUseDevice( wxCommandEvent& e );
    virtual void                        OnClose( wxCloseEvent& );
    void                                QueryInitialDeviceSettings( DeviceSettings& settings );
    void                                PresetHiddenAdjustments( bool boColor, TOptimizationLevel optimizationLevel );
    void                                RefreshControls( void );
    void                                SelectLUTDependingOnPixelFormat( void );
    void                                SetAcquisitionFrameRateLimitMode( void );
    void                                SetupControls( void );
    void                                SetupDevice( void );
    void                                SetupDriverSettings( void );
    void                                SetupUnifiedData( bool newDevice );
    bool                                ShowFactoryResetPopup( void );
    void                                ToggleDetails( bool boShow );
    void                                UpdateDetailsTextControl( void );
    void                                UpdateExposureControlsFromCamera( void );
    void                                UpdateGainControlsFromCamera( void );
    void                                UpdateWhiteBalanceControlsFromCamera( void );

    //----------------------------------EXPOSURE-----------------------------------
    void                                ApplyExposure( void );
    void                                ConfigureExposureAuto( bool boActive );
    double                              ExposureFromSliderValue( void ) const;
    int                                 ExposureToSliderValue( const double exposure ) const;
    void                                HandleExposureSpinControlChanges( void );

    //----------------------------------GAIN---------------------------------------
    void                                ApplyGain( void );
    void                                ConfigureGainAuto( bool boActive );
    void                                HandleGainSpinControlChanges( void );
    double                              ReadUnifiedGainData( void );
    void                                WriteUnifiedGainData( double unifiedGain );

    //----------------------------------BLACKLEVEL---------------------------------
    void                                ApplyBlackLevel( void );
    void                                ConfigureGamma( bool boActive );
    void                                HandleBlackLevelSpinControlChanges( void );
    double                              ReadUnifiedBlackLevelData( void );
    void                                WriteUnifiedBlackLevelData( double unifiedBlackLevel );

    //----------------------------------WHITEBALANCE-------------------------------
    void                                ApplyWhiteBalance( TWhiteBalanceChannel channel );
    void                                ConfigureWhiteBalanceAuto( bool boActive );
    void                                HandleWhiteBalanceRSpinControlChanges( void );
    void                                HandleWhiteBalanceBSpinControlChanges( void );

    //----------------------------------SATURATION--------------------------------
    void                                ApplySaturation( void );
    void                                ConfigureCCM( bool boActive );
    void                                HandleSaturationSpinControlChanges( void );
    double                              ReadSaturationData( void );
    void                                WriteSaturationData( double saturation );

    //----------------------------------FRAMERATE---------------------------------
    void                                ApplyFrameRate( void );
    void                                ConfigureFrameRateMaximum( bool boActive );
    void                                HandleFrameRateSpinControlChanges( void );

    //----------------------------------BUTTONS------------------------------------
    void                                OnBtnPresetColor( wxCommandEvent& e );
    void                                OnBtnPresetGray( wxCommandEvent& e );
    void                                OnBtnPresetCustom( wxCommandEvent& e );
    void                                OnBtnPresetFactory( wxCommandEvent& e );

    void                                OnBtnExposureAuto( wxCommandEvent& e )
    {
        ConfigureExposureAuto( e.IsChecked() );
        RefreshControls();
    }
    void                                OnBtnGainAuto( wxCommandEvent& e )
    {
        ConfigureGainAuto( e.IsChecked() );
        RefreshControls();
    }
    void                                OnBtnGamma( wxCommandEvent& e )
    {
        ConfigureGamma( e.IsChecked() );
        RefreshControls();
    }
    void                                OnBtnCCM( wxCommandEvent& e )
    {
        ConfigureCCM( e.IsChecked() );
        RefreshControls();
    }
    void                                OnBtnWhiteBalanceAuto( wxCommandEvent& e )
    {
        ConfigureWhiteBalanceAuto( e.IsChecked() );
        RefreshControls();
    }
    void                                OnBtnFrameRateMaximum( wxCommandEvent& e )
    {
        ConfigureFrameRateMaximum( e.IsChecked() );
        RefreshControls();
    }
    void                                OnBtnHistory( wxCommandEvent& e )
    {
        ToggleDetails( e.IsChecked() );
    }
    void                                OnBtnClearHistory( wxCommandEvent& )
    {
        pTCHistory_->Clear();
    }

    virtual void                        OnBtnCancel( wxCommandEvent& );
    virtual void                        OnBtnOk( wxCommandEvent& );

    //----------------------------------SPINCONTROLS--------------------------------
    void                                OnSCExposureChanged( wxSpinEvent& )
    {
        HandleExposureSpinControlChanges();
    }
    void                                OnSCGainChanged( wxSpinEvent& )
    {
        HandleGainSpinControlChanges();
    }
    void                                OnSCBlackLevelChanged( wxSpinEvent& )
    {
        HandleBlackLevelSpinControlChanges();
    }
    void                                OnSCWhiteBalanceRChanged( wxSpinEvent& )
    {
        HandleWhiteBalanceRSpinControlChanges();
    }
    void                                OnSCWhiteBalanceBChanged( wxSpinEvent& )
    {
        HandleWhiteBalanceBSpinControlChanges();
    }
    void                                OnSCSaturationChanged( wxSpinEvent& )
    {
        HandleSaturationSpinControlChanges();
    }
    void                                OnSCFrameRateChanged( wxSpinEvent& )
    {
        HandleFrameRateSpinControlChanges();
    }

    //----------------------------------TEXTCONTROLS--------------------------------
    void                                OnSCExposureTextChanged( wxCommandEvent& )
    {
        HandleExposureSpinControlChanges();
    }
    void                                OnSCGainTextChanged( wxCommandEvent& )
    {
        HandleGainSpinControlChanges();
    }
    void                                OnSCBlackLevelTextChanged( wxCommandEvent& )
    {
        HandleBlackLevelSpinControlChanges();
    }
    void                                OnSCWhiteBalanceRTextChanged( wxCommandEvent& )
    {
        HandleWhiteBalanceRSpinControlChanges();
    }
    void                                OnSCWhiteBalanceBTextChanged( wxCommandEvent& )
    {
        HandleWhiteBalanceBSpinControlChanges();
    }
    void                                OnSCSaturationTextChanged( wxCommandEvent& )
    {
        HandleSaturationSpinControlChanges();
    }
    void                                OnSCFrameRateTextChanged( wxCommandEvent& )
    {
        HandleFrameRateSpinControlChanges();
    }

    //----------------------------------SLIDERS-------------------------------------
    void                                OnSLExposure( wxScrollEvent& e );
    void                                OnSLGain( wxScrollEvent& e );
    void                                OnSLBlackLevel( wxScrollEvent& e );
    void                                OnSLWhiteBalanceR( wxScrollEvent& e );
    void                                OnSLWhiteBalanceB( wxScrollEvent& e );
    void                                OnSLSaturation( wxScrollEvent& e );
    void                                OnSLFrameRate( wxScrollEvent& e );
    void                                OnSLOptimisation( wxScrollEvent& e );
    DECLARE_EVENT_TABLE()
};

//-----------------------------------------------------------------------------
class WizardQuickSetupGenICam : public WizardQuickSetup
//-----------------------------------------------------------------------------
{
public:
    explicit                            WizardQuickSetupGenICam( PropViewFrame* pParent, const wxString& title );
protected:
    virtual void                        CreateInterfaceLayoutSpecificControls( Device* pDev );
    virtual void                        DeleteInterfaceLayoutSpecificControls( void );
    virtual double                      DetermineMaxFrameRateAtCurrentAOI( void );
    virtual double                      DetermineOptimalHQFrameRateAtCurrentAOI( void );
    virtual void                        DoConfigureFrameRateMaximum( bool boActive, double frameRateValue );
    virtual double                      DoReadUnifiedBlackLevel( void ) const;
    virtual double                      DoReadUnifiedGain( void ) const;
    virtual void                        DoSetAcquisitionFrameRateLimitMode( void );
    virtual void                        DoWriteUnifiedGain( double value ) const;
    virtual void                        DoWriteUnifiedBlackLevelData( double value ) const;
    virtual double                      GetAutoExposureUpperLimit( void ) const;
    virtual bool                        GetExposureOverlapped() const
    {
        return false;
    }
    virtual std::string                 GetPixelClock( void ) const;
    virtual std::string                 GetPixelFormat( void ) const;
    virtual double                      GetExposureTime( void ) const
    {
        return pAcC_->exposureTime.read();
    }
    virtual bool                        GetFrameRateEnable( void ) const
    {
        return pAcC_->mvAcquisitionFrameRateEnable.read() == bTrue;
    }
    virtual double                      GetWhiteBalance( TWhiteBalanceChannel channel );
    virtual bool                        HasAEC( void ) const;
    virtual bool                        HasAGC( void );
    virtual bool                        HasAWB( void ) const;
    virtual bool                        HasMaximumFrameRate( void ) const;
    virtual bool                        HasBlackLevel( void ) const
    {
        return pAnC_->blackLevelSelector.isValid();
    }
    virtual bool                        HasColorFormat( void ) const;
    virtual bool                        HasFactoryDefault( void ) const;
    virtual bool                        HasFrameRateEnable( void ) const
    {
        return pAcC_->mvAcquisitionFrameRateEnable.isValid();
    }
    virtual bool                        HasGain( void ) const
    {
        return pAnC_->gainSelector.isValid();
    }
    virtual bool                        HasUnifiedGain( void ) const
    {
        return pAnC_->gainSelector.isValid();
    }
    virtual void                        InitializeExposureParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        InitializeGainParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        InitializeBlackLevelParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        InitializeWhiteBalanceParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        QueryInterfaceLayoutSpecificSettings( DeviceSettings& devSettings );
    virtual void                        RestoreFactoryDefault( void );
    virtual void                        SelectOptimalPixelFormatColor( bool boHighQuality );
    virtual void                        SelectOptimalPixelFormatGray( bool boHighQuality );
    virtual void                        SelectOptimalAutoExposureSettings( bool boHighQuality );
    virtual void                        SelectOptimalPixelClock( bool boHighQuality );
    virtual void                        SetExposureTime( double value )
    {
        LoggedWriteProperty( pAcC_->exposureTime, value );
    }
    virtual void                        SetAutoExposure( bool boEnable )
    {
        LoggedWriteProperty( pAcC_->exposureAuto, std::string( boEnable ? "Continuous" : "Off" ) );
    }
    virtual void                        SetAutoExposureUpperLimit( double exposureMax );
    virtual void                        SetAutoGain( bool boEnable );
    virtual void                        SetAutoWhiteBalance( bool boEnable )
    {
        LoggedWriteProperty( pAnC_->balanceWhiteAuto, std::string( boEnable ? "Continuous" : "Off" ) );
    }
    virtual void                        SetFrameRateEnable( bool boOn )
    {
        LoggedWriteProperty( pAcC_->mvAcquisitionFrameRateEnable, std::string( boOn ? "On" : "Off" ) );
    }
    virtual void                        SetPixelClock( const std::string& frequency )
    {
        LoggedWriteProperty( pDeC_->mvDeviceClockFrequency, frequency );
    }
    virtual void                        SetPixelFormat( const std::string& format )
    {
        LoggedWriteProperty( pIFC_->pixelFormat, format );
    }
    virtual void                        SetupExposureControls( void );
    virtual void                        SetupGainControls( void );
    virtual void                        SetupBlackLevelControls( void );
    virtual void                        SetupWhiteBalanceControls( void );
    virtual void                        SetupFrameRateControls( void );
    virtual void                        SetupUnifiedGainData( void );
    virtual void                        SetupUnifiedBlackLevelData( void );
    virtual void                        SetWhiteBalance( TWhiteBalanceChannel channel, double value );
    virtual void                        TryToReadFrameRate( double& value );
    virtual void                        WriteExposureFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        WriteGainFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        WriteWhiteBalanceFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        SetFrameRate( double value );
private:
    GenICam::DeviceControl*             pDeC_;
    GenICam::AcquisitionControl*        pAcC_;
    GenICam::AnalogControl*             pAnC_;
    GenICam::ImageFormatControl*        pIFC_;
    GenICam::UserSetControl*            pUSC_;
};

//-----------------------------------------------------------------------------
class WizardQuickSetupDeviceSpecific : public WizardQuickSetup
//-----------------------------------------------------------------------------
{
public:
    explicit                            WizardQuickSetupDeviceSpecific( PropViewFrame* pParent, const wxString& title );
protected:
    virtual void                        CreateInterfaceLayoutSpecificControls( Device* pDev );
    virtual void                        DeleteInterfaceLayoutSpecificControls( void );
    virtual double                      DetermineMaxFrameRateAtCurrentAOI( void );
    virtual double                      DetermineOptimalHQFrameRateAtCurrentAOI( void );
    virtual double                      DoReadUnifiedBlackLevel( void ) const;
    virtual double                      DoReadUnifiedGain( void ) const;
    virtual void                        DoWriteUnifiedGain( double value ) const;
    virtual void                        DoWriteUnifiedBlackLevelData( double value ) const;
    virtual double                      GetAutoExposureUpperLimit( void ) const;
    virtual bool                        GetExposureOverlapped() const
    {
        return pCSBF_->exposeMode.read() == cemOverlapped;
    }
    virtual std::string                 GetPixelClock( void ) const;
    virtual std::string                 GetPixelFormat( void ) const;
    virtual double                      GetExposureTime( void ) const
    {
        return pCSBF_->expose_us.read();
    }
    virtual double                      GetWhiteBalance( TWhiteBalanceChannel channel );
    virtual bool                        HasAEC( void ) const
    {
        return pCSBF_->autoExposeControl.isValid();
    }
    virtual bool                        HasAGC( void )
    {
        return pCSBF_->autoGainControl.isValid();
    }
    virtual bool                        HasAWB( void ) const
    {
        return false;
    }
    virtual bool                        HasMaximumFrameRate( void ) const
    {
        return false;
    }
    virtual bool                        HasBlackLevel( void ) const
    {
        return pIP_->gainOffsetKneeEnable.isValid();
    }
    virtual bool                        HasColorFormat( void ) const
    {
        return currentProductString_[currentProductString_.length() - 1] == 'C';
    }
    virtual bool                        HasFactoryDefault( void ) const
    {
        return true;
    }
    virtual bool                        HasFrameRateEnable( void ) const
    {
        return false;
    }
    virtual bool                        HasGain( void ) const
    {
        return pCSBF_->gain_dB.isValid();
    }
    virtual bool                        HasUnifiedGain( void ) const
    {
        return false;
    }
    virtual void                        InitializeExposureParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        InitializeGainParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        InitializeBlackLevelParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        InitializeWhiteBalanceParameters( DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        QueryInterfaceLayoutSpecificSettings( DeviceSettings& devSettings );
    virtual void                        RestoreFactoryDefault( void );
    virtual void                        SelectOptimalPixelFormatColor( bool boHighQuality );
    virtual void                        SelectOptimalPixelFormatGray( bool boHighQuality );
    virtual void                        SelectOptimalAutoExposureSettings( bool boHighQuality );
    virtual void                        SelectOptimalPixelClock( bool boHighQuality );
    virtual void                        SetExposureTime( double value )
    {
        LoggedWriteProperty( pCSBF_->expose_us, value );
    }
    virtual void                        SetAutoExposure( bool boEnable )
    {
        LoggedWriteProperty( pCSBF_->autoExposeControl, std::string( boEnable ? "On" : "Off" ) );
    }
    virtual void                        SetAutoExposureUpperLimit( double exposureMax );
    virtual void                        SetAutoGain( bool boEnable );
    virtual void                        SetPixelClock( const std::string& frequency )
    {
        LoggedWriteProperty( pCSBF_->pixelClock_KHz, frequency );
    }
    virtual void                        SetPixelFormat( const std::string& format )
    {
        LoggedWriteProperty( pCSBF_->pixelFormat, format );
    }
    virtual void                        SetupExposureControls( void );
    virtual void                        SetExposureOverlapped( bool boEnable );
    virtual void                        SetupGainControls( void );
    virtual void                        SetupBlackLevelControls( void );
    virtual void                        SetupWhiteBalanceControls( void );
    virtual void                        SetupUnifiedGainData( void );
    virtual void                        SetupUnifiedBlackLevelData( void );
    virtual void                        SetWhiteBalance( TWhiteBalanceChannel channel, double value );
    virtual void                        TryToReadFrameRate( double& /*value*/ )
    {
        //No auto-framerate for devices with deviceSpecific interface!
        UpdateFramerateWarningBitmap( false );
    }
    virtual void                        WriteExposureFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        WriteGainFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features );
    virtual void                        WriteWhiteBalanceFeatures( const DeviceSettings& devSettings, const SupportedWizardFeatures& features );
private:
    CameraSettingsBlueFOX*              pCSBF_;
};

#endif // WizardQuickSetupH
