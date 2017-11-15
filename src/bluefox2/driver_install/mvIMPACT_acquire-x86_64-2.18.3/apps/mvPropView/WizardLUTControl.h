//-----------------------------------------------------------------------------
#ifndef WizardLUTControlH
#define WizardLUTControlH WizardLUTControlH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include "PlotCanvas.h"
#include "spinctld.h"
#include "ValuesFromUserDlg.h"
#include <wx/notebook.h>

class DataGridWithClipboardFeature;
class wxNotebook;

wxDECLARE_EVENT( GammaParameterChangeEvent, wxCommandEvent );

//-----------------------------------------------------------------------------
class WizardLUTCanvas : public PlotCanvas
//-----------------------------------------------------------------------------
{
    std::vector<int>        lut_;
    int                     maxOutputValue_;
protected:
    virtual double          GetScaleX( wxCoord w ) const;
    virtual double          GetScaleY( wxCoord h ) const;
    virtual unsigned int    GetXMarkerParameters( unsigned int& from, unsigned int& to ) const;
    virtual void            OnPaintCustom( wxPaintDC& dc );
public:
    explicit                WizardLUTCanvas() {}
    explicit                WizardLUTCanvas( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "LUT" ), bool boActive = false );
    ~WizardLUTCanvas();
    int                     GetPercentageToDraw( void ) const
    {
        return 100;
    }
    void                    RefreshData( const std::vector<int>& lut, const int maxOutputValue );
};

//-----------------------------------------------------------------------------
struct GammaData
//-----------------------------------------------------------------------------
{
    double gamma_;
    double gammaAlpha_;
    TLUTGammaMode gammaMode_;
    int gammaStartThreshold_;
    explicit GammaData() : gamma_( 1.0 ), gammaAlpha_( 0.0 ), gammaMode_( LUTgmStandard ), gammaStartThreshold_( 12 ) {}
};

//-----------------------------------------------------------------------------
class WizardLUTControl : public OkAndCancelDlg, public DataGridDataProvider
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
    //-----------------------------------------------------------------------------
    enum TWidgetIDs_LUTControl
    //-----------------------------------------------------------------------------
    {
        widMainFrame = widFirst,
        widNBDisplayMethod,
        widGraphicalDisplay,
        widNumericalDisplay,
        widCBLUTSelection,
        widBtnSynchronize,
        widBtnEnableAll,
        widBtnDisableAll,
        widCBEnable,
        widBtnInvert,
        widBtnGamma,
        widBtnInterpolate,
        widBtnCopyTo,
        widBtnImport,
        widBtnExport
    };
    //-----------------------------------------------------------------------------
    enum TDisplayMethod
    //-----------------------------------------------------------------------------
    {
        dmGraphical,
        dmNumerical
    };
    //-----------------------------------------------------------------------------
    struct LUTEntry
            //-----------------------------------------------------------------------------
    {
        std::vector<int> lut_;
        bool boDirty_;
        explicit LUTEntry( const std::vector<int>& lut, bool boDirty ) : lut_( lut ), boDirty_( boDirty ) {}
    };

    wxTextAttr errorStyle_;
    wxCheckBox* pCBEnable_;
    wxComboBox* pCBLUTSelection_;
    wxNotebook* pNBDisplayMethod_;
    WizardLUTCanvas* pGraphicalDisplay_;
    DataGridWithClipboardFeature* pNumericalDisplay_;
    wxButton* pBtnEnableAll_;
    wxButton* pBtnDisableAll_;
    wxButton* pBtnSynchronize_;
    wxButton* pBtnInvert_;
    wxButton* pBtnGamma_;
    wxButton* pBtnInterpolate_;
    wxButton* pBtnCopyTo_;
    wxButton* pBtnImport_;
    wxButton* pBtnExport_;
    wxTextCtrl* pLogWindow_;
    TDisplayMethod displayMethod_;
    mvIMPACT::acquire::GenICam::LUTControl lc_;
    const wxArrayString lutSelectorEntries_;
    const size_t lutCnt_;
    typedef std::map<wxString, LUTEntry> LUTMap;
    LUTMap lutMap_;
    std::vector<int> activeLUT_;
    GammaData gammaData_;

    void AddLUTToCache( const wxString& lutIdentifier, std::vector<int>& lut, bool boDirty );
    void CalculateGammaLUT( std::vector<int>& lut, const GammaData& data );
    void ConfigureAllLUTs( bool boEnable );
    void ReadLUTFromDevice( const wxString& LUTSelectorValue, std::vector<int>& lut );
    void SetupLUTEnable( void );
    void SynchronizeLUT( const bool boCacheToDevice, const wxString& LUTSelectorValue, std::vector<int>& lut );
    void UpdateDisplay( bool boEraseBackground = true );
    void WriteErrorMessage( const wxString& msg )
    {
        WriteLogMessage( msg, errorStyle_ );
    }
    void WriteLogMessage( const wxString& msg, const wxTextAttr& style = wxTextAttr( *wxBLACK ) )
    {
        WriteToTextCtrl( pLogWindow_, msg, style );
    }
    void WriteLUTToDevice( const wxString& LUTSelectorValue, std::vector<int>& lut );
private:
    virtual void OnBtnApply( wxCommandEvent& );
    virtual void OnBtnCancel( wxCommandEvent& )
    {
        Hide();
    }
    void OnBtnCopyTo( wxCommandEvent& );
    void OnBtnDisableAll( wxCommandEvent& )
    {
        ConfigureAllLUTs( false );
    }
    void OnBtnEnableAll( wxCommandEvent& )
    {
        ConfigureAllLUTs( true );
    }
    void OnBtnExport( wxCommandEvent& );
    void OnBtnGamma( wxCommandEvent& );
    void OnBtnImport( wxCommandEvent& );
    void OnBtnInterpolate( wxCommandEvent& );
    void OnBtnInvert( wxCommandEvent& );
    virtual void OnBtnOk( wxCommandEvent& )
    {
        Hide();
    }
    void OnBtnSynchronize( wxCommandEvent& );
    void OnCBEnable( wxCommandEvent& e );
    void OnCBLUTSelectionChanged( wxCommandEvent& );
    void OnClose( wxCloseEvent& e );
    void OnGammaParameterChanged( wxCommandEvent& );
    void OnNBDisplayMethodPageChanged( wxNotebookEvent& e );
public:
    explicit            WizardLUTControl( wxWindow* pParent, const wxString& title, mvIMPACT::acquire::GenICam::LUTControl& lc, const wxArrayString& lutSelectorEntries );
    virtual wxString    GetGridValue( int row, int col ) const;
    virtual void        SetGridValue( int row, int col, const wxString& value );
    virtual void        SetGridValueFormatString( const wxString& gridValueFormatString );
    void                UpdateDialog( void );
};

//-----------------------------------------------------------------------------
class WizardLUTGammaDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
    //-----------------------------------------------------------------------------
    enum TWidgetIDs_LUTControl
    //-----------------------------------------------------------------------------
    {
        widSCGamma = widFirst,
        widSCGammaAlpha,
        widCBGammaMode,
        widSCGammaStartThreshold
    };
    GammaData& data_;
    wxSpinCtrlDbl* pSCGamma_;
    wxSpinCtrlDbl* pSCGammaAlpha_;
    wxComboBox* pCBGammaMode_;
    wxStaticText* pSTGammaStartThreshold_;
    wxSpinCtrl* pSCGammaStartThreshold_;

    void OnCBGammaModeChanged( wxCommandEvent& )
    {
        data_.gammaMode_ = ( GetGammaMode() == wxT( "Standard" ) ) ? LUTgmStandard : LUTgmLinearStart;
        SetupControls();
        InformParentAboutChanges();
    }
    void OnGammaChanged( wxSpinEvent& )
    {
        data_.gamma_ = GetGamma();
        InformParentAboutChanges();
    }
    void OnGammaTextChanged( wxCommandEvent& )
    {
        data_.gamma_ = GetGamma();
        InformParentAboutChanges();
    }
    void OnGammaAlphaChanged( wxSpinEvent& )
    {
        data_.gammaAlpha_ = GetGammaAlpha();
        InformParentAboutChanges();
    }
    void OnGammaAlphaTextChanged( wxCommandEvent& )
    {
        data_.gammaAlpha_ = GetGammaAlpha();
        InformParentAboutChanges();
    }
    void OnGammaStartThresholdChanged( wxSpinEvent& )
    {
        data_.gammaStartThreshold_ = GetGammaStartThreshold();
        InformParentAboutChanges();
    }
    void OnGammaStartThresholdTextChanged( wxCommandEvent& )
    {
        data_.gammaStartThreshold_ = GetGammaStartThreshold();
        InformParentAboutChanges();
    }
    void SetupControls( void );
    void InformParentAboutChanges( void ) const
    {
        wxCommandEvent e( GammaParameterChangeEvent, GetParent()->GetId() );
        ::wxPostEvent( GetParent()->GetEventHandler(), e );
    }
public:
    explicit WizardLUTGammaDlg( wxWindow* pParent, const wxString& title, const int maxLUTIndex, GammaData& data );
    double GetGamma( void ) const
    {
        return pSCGamma_->GetValue();
    }
    double GetGammaAlpha( void ) const
    {
        return pSCGammaAlpha_->GetValue();
    }
    wxString GetGammaMode( void ) const
    {
        return pCBGammaMode_->GetValue();
    }
    int GetGammaStartThreshold( void ) const
    {
        return pSCGammaStartThreshold_->GetValue();
    }
};

#endif // WizardLUTControlH
