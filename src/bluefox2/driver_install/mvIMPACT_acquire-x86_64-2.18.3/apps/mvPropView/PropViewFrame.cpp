#include <algorithm>
#include <apps/Common/exampleHelper.h>
#include <apps/Common/mvIcon.xpm>
#include <apps/Common/wxAbstraction.h>
#include "CaptureThread.h"
#include <common/STLHelper.h>
#include "DataConversion.h"
#include "HistogramCanvasSpatialNoise.h"
#include "HistogramCanvasTemporalNoise.h"
#include "icons.h"
#include "ImageCanvas.h"
#include "PlotCanvasIntensity.h"
#include <limits>
#include "LineProfileCanvasHorizontal.h"
#include "LineProfileCanvasVertical.h"
#include <memory>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam_FileStream.h>
#include "PropData.h"
#include "PropViewFrame.h"
#include "SpinEditorDouble.h"
#include <sstream>
#include "VectorScopeCanvas.h"
#include "WizardLensControl.h"
#include "WizardLUTControl.h"
#include "WizardQuickSetup.h"
#include "WizardSaturation.h"
#include "WizardSequencerControl.h"
#include <wx/choicdlg.h>
#include <wx/config.h>
#include <wx/dnd.h>
#include <wx/file.h>
#include <wx/listctrl.h>
#include <wx/numdlg.h>
#include <wx/platinfo.h>
#include <wx/progdlg.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/splitter.h>
#include <wx/tooltip.h>
#include <wx/utils.h>
#ifdef _WIN32
#   include <wx/dir.h>
#   include <wx/docview.h>
#   include <wx/filefn.h>
#   include <wx/wfstream.h>
#   include <wx/zipstrm.h>
#   include <wx/textfile.h>
#   include <wx/txtstrm.h>
#endif // _WIN32
#if defined(linux) || defined(__linux) || defined(__linux__)
#   include <sys/sysctl.h>
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

using namespace mvIMPACT::acquire;
using namespace std;

wxDEFINE_EVENT( imageReadyEvent, wxCommandEvent );
wxDEFINE_EVENT( imageSkippedEvent, wxCommandEvent );
wxDEFINE_EVENT( toggleDisplayArea, wxCommandEvent );
wxDEFINE_EVENT( liveModeAborted, wxCommandEvent );
wxDEFINE_EVENT( sequenceReadyEvent, wxCommandEvent );
wxDEFINE_EVENT( imageInfoEvent, wxCommandEvent );
wxDEFINE_EVENT( refreshAOIControls, wxCommandEvent );
wxDEFINE_EVENT( refreshCurrentPixelData, wxCommandEvent );
wxDEFINE_EVENT( imageCanvasSelected, wxCommandEvent );
wxDEFINE_EVENT( imageCanvasFullScreen, wxCommandEvent );
wxDEFINE_EVENT( monitorImageClosed, wxCommandEvent );
wxDEFINE_EVENT( featureChangedCallbackReceived, wxCommandEvent );
wxDEFINE_EVENT( imageTimeoutEvent, wxCommandEvent );

//-----------------------------------------------------------------------------
void CollectFeatureNamesForInfoPlot( ComponentIterator it, wxArrayString& v, const wxString& path = wxEmptyString )
//-----------------------------------------------------------------------------
{
    while( it.isValid() )
    {
        wxString fullName( path );
        if( fullName != wxT( "" ) )
        {
            fullName += wxT( "/" );
        }
        fullName += ConvertedString( it.name() );
        switch( it.type() )
        {
        case ctList:
            CollectFeatureNamesForInfoPlot( it.firstChild(), v, fullName );
            break;
        case ctPropInt:
        case ctPropInt64:
        case ctPropFloat:
            v.Add( fullName );
            break;
        default:
            break;
        }
        ++it;
    }
}

//-----------------------------------------------------------------------------
struct SettingData
//-----------------------------------------------------------------------------
{
    TScope scope_;
    string name_;
    explicit SettingData( TScope scope, const string& name ) : scope_( scope ), name_( name ) {}
};

//=============================================================================
//================= Implementation DnDCaptureSetting ==========================
//=============================================================================
//-----------------------------------------------------------------------------
class DnDCaptureSetting : public wxFileDropTarget
//-----------------------------------------------------------------------------
{
public:
    DnDCaptureSetting( PropViewFrame* pOwner ) : pOwner_( pOwner ) {}
    virtual bool OnDropFiles( wxCoord, wxCoord, const wxArrayString& fileNames )
    {
        return ( fileNames.IsEmpty() ) ? false : pOwner_->LoadActiveDeviceFromFile( wxFileName( fileNames.Item( 0 ) ).GetFullPath() );
    }
private:
    PropViewFrame* pOwner_;
};

//=============================================================================
//================= Implementation DnDImage ===================================
//=============================================================================
//-----------------------------------------------------------------------------
class DnDImage : public wxFileDropTarget
//-----------------------------------------------------------------------------
{
public:
    DnDImage( PropViewFrame* pOwner, ImageCanvas* pImageCanvas ) : pOwner_( pOwner ), pImageCanvas_( pImageCanvas ) {}
    virtual bool OnDropFiles( wxCoord, wxCoord, const wxArrayString& fileNames )
    {
        return ( fileNames.IsEmpty() ) ? false : pOwner_->SetCurrentImage( wxFileName( fileNames.Item( 0 ) ), pImageCanvas_ );
    }
private:
    PropViewFrame* pOwner_;
    ImageCanvas* pImageCanvas_;
};

//=============================================================================
//================= Implementation MonitorDisplay =============================
//=============================================================================
//-----------------------------------------------------------------------------
class MonitorDisplay : public wxFrame
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
    ImageCanvas* m_pDisplayArea;
public:
    explicit MonitorDisplay( wxWindow* pParent ) : wxFrame( pParent, wxID_ANY, wxT( "Monitor Display" ), wxDefaultPosition, wxDefaultSize, wxCAPTION | wxRESIZE_BORDER | wxFRAME_TOOL_WINDOW | wxFRAME_FLOAT_ON_PARENT )
    {
        wxPanel* pPanel = new wxPanel( this );
        m_pDisplayArea = new ImageCanvas( this, pPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE, wxEmptyString );
        m_pDisplayArea->SetScaling( true );
        m_pDisplayArea->HandleMouseAndKeyboardEvents( false );
        wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxHORIZONTAL );
        pTopDownSizer->Add( m_pDisplayArea, wxSizerFlags( 1 ).Expand() );
        pPanel->SetSizer( pTopDownSizer );
        pTopDownSizer->SetSizeHints( this );
        SetSizeHints( 100, 66 );
        bool boMaximized = false;
        SetSize( FramePositionStorage::Load( wxRect( 0, 0, 150, 100 ), boMaximized, wxString( wxT( "MonitorDisplay" ) ) ) );
    }
    ~MonitorDisplay()
    {
        FramePositionStorage::Save( this, wxString( wxT( "MonitorDisplay" ) ) );
    }
    ImageCanvas* GetDisplayArea( void )
    {
        return m_pDisplayArea;
    }
    void OnClose( wxCloseEvent& )
    {
        m_pDisplayArea->SetActive( false );
        Hide();
        wxCommandEvent e( monitorImageClosed, GetParent()->GetId() );
        ::wxPostEvent( GetParent()->GetEventHandler(), e );
    }
};

BEGIN_EVENT_TABLE( MonitorDisplay, wxFrame )
    EVT_CLOSE( MonitorDisplay::OnClose )
END_EVENT_TABLE()

//=============================================================================
//================= Implementation MyApp ======================================
//=============================================================================
//-----------------------------------------------------------------------------
class MyApp : public wxApp
//-----------------------------------------------------------------------------
{
    PropViewFrame* pFrame_;
public:
    virtual int FilterEvent( wxEvent& e )
    {
        if( ( e.GetEventType() == wxEVT_KEY_DOWN ) && ( ( wxKeyEvent& )e ).GetKeyCode() == WXK_ESCAPE )
        {
            pFrame_->EndFullScreenMode();
            return true;
        }
        return -1;
    }
    virtual bool OnInit( void )
    {
        wxImage::AddHandler( new wxGIFHandler );
        wxImage::AddHandler( new wxJPEGHandler );
        wxImage::AddHandler( new wxPNGHandler );
        wxImage::AddHandler( new wxTIFFHandler );
        SplashScreenScope splashScreenScope( wxBitmap::NewFromPNGData( wxPropView_png, sizeof( wxPropView_png ) ) );
        pFrame_ = new PropViewFrame( wxString::Format( wxT( "wxPropView(%s)" ), VERSION_STRING ), wxDefaultPosition, wxDefaultSize, argc, argv );
        pFrame_->Show( true );
        SetTopWindow( pFrame_ );
        //next line's purpose is to free the Image canvas of any buggy remains e.g. buttons of an invisible property grid etc.
        pFrame_->Refresh();
        return true;
    }
};

IMPLEMENT_APP( MyApp )

//=============================================================================
//================= Implementation PropViewFrame ==============================
//=============================================================================
BEGIN_EVENT_TABLE( PropViewFrame, PropGridFrameBase )
    EVT_MENU( miAction_DefaultDeviceInterface_DeviceSpecific, PropViewFrame::OnAction_DefaultDeviceInterface_Changed )
    EVT_MENU( miAction_DefaultDeviceInterface_GenICam, PropViewFrame::OnAction_DefaultDeviceInterface_Changed )
    EVT_MENU( miAction_DisplayConnectedDevicesOnly, PropViewFrame::OnAction_DisplayConnectedDevicesOnly )
    EVT_MENU( miAction_Exit, PropViewFrame::OnAction_Exit )
    EVT_MENU( miAction_CaptureSettings_Save_ToDefault, PropViewFrame::OnSaveToDefault )
    EVT_MENU( miAction_CaptureSettings_Save_CurrentProduct, PropViewFrame::OnSaveCurrentProduct )
    EVT_MENU( miAction_CaptureSettings_Save_ActiveDevice, PropViewFrame::OnSaveActiveDevice )
    EVT_MENU( miAction_CaptureSettings_Save_ExportActiveDevice, PropViewFrame::OnExportActiveDevice )
    EVT_MENU( miAction_CaptureSettings_Load_FromDefault, PropViewFrame::OnLoadFromDefault )
    EVT_MENU( miAction_CaptureSettings_Load_CurrentProduct, PropViewFrame::OnLoadCurrentProduct )
    EVT_MENU( miAction_CaptureSettings_Load_ActiveDevice, PropViewFrame::OnLoadActiveDevice )
    EVT_MENU( miAction_CaptureSettings_Load_ActiveDeviceFromFile, PropViewFrame::OnLoadActiveDeviceFromFile )
    EVT_MENU( miAction_CaptureSettings_Manage, PropViewFrame::OnManageSettings )
    EVT_MENU( miAction_LoadImage, PropViewFrame::OnLoadImage )
    EVT_MENU( miAction_SaveImage, PropViewFrame::OnSaveImage )
    EVT_MENU( miAction_SaveImageSequenceToFiles, PropViewFrame::OnSaveImageSequenceToFiles )
    EVT_MENU( miAction_SaveImageSequenceToStream, PropViewFrame::OnSaveImageSequenceToStream )
    EVT_MENU( miAction_UseDevice, PropViewFrame::OnUseDevice )
    EVT_MENU( miAction_UpdateDeviceList, PropViewFrame::OnUpdateDeviceList )
    EVT_MENU( miCapture_Acquire, PropViewFrame::OnBtnAcquire )
    EVT_MENU( miCapture_Abort, PropViewFrame::OnBtnAbort )
    EVT_MENU( miCapture_Unlock, PropViewFrame::OnBtnUnlock )
    EVT_MENU( miCapture_DefaultImageProcessingMode_ProcessAll, PropViewFrame::OnCapture_DefaultImageProcessingMode_Changed )
    EVT_MENU( miCapture_DefaultImageProcessingMode_ProcessLatestOnly, PropViewFrame::OnCapture_DefaultImageProcessingMode_Changed )
    EVT_MENU( miCapture_Record, PropViewFrame::OnBtnRecord )
    EVT_MENU( miCapture_Backward, PropViewFrame::OnIncDecRecordDisplay )
    EVT_MENU( miCapture_Forward, PropViewFrame::OnIncDecRecordDisplay )
    EVT_MENU( miCapture_Recording_Continuous, PropViewFrame::OnContinuousRecording )
    EVT_MENU( miCapture_Recording_SetupSequenceSize, PropViewFrame::OnSetupRecordSequenceSize )
    EVT_MENU( miCapture_Recording_SetupHardDiscRecording, PropViewFrame::OnSetupHardDiscRecording )
    EVT_MENU( miCapture_CaptureSettings_CreateCaptureSetting, PropViewFrame::OnCapture_CaptureSettings_CreateCaptureSetting )
    EVT_MENU( miCapture_CaptureSettings_CaptureSettingHierarchy, PropViewFrame::OnCapture_CaptureSettings_CaptureSettingHierarchy )
    EVT_MENU( miCapture_CaptureSettings_AssignToDisplays, PropViewFrame::OnCapture_CaptureSettings_AssignToDisplays )
    EVT_MENU( miCapture_CaptureSettings_UsageMode_Manual, PropViewFrame::OnCapture_CaptureSettings_UsageMode_Changed )
    EVT_MENU( miCapture_CaptureSettings_UsageMode_Automatic, PropViewFrame::OnCapture_CaptureSettings_UsageMode_Changed )
    EVT_MENU( miCapture_SetupCaptureQueueDepth, PropViewFrame::OnSetupCaptureQueueDepth )
    EVT_MENU( miCapture_DetailedRequestInformation, PropViewFrame::OnDetailedRequestInformation )
    EVT_MENU( miSettings_Display_ConfigureImageDisplayCount, PropViewFrame::OnConfigureImageDisplayCount )
    EVT_MENU( miSettings_Display_Active, PropViewFrame::OnActivateDisplay )
    EVT_MENU( miSettings_Display_ShowMonitorImage, PropViewFrame::OnShowMonitorDisplay )
    EVT_MENU( miSettings_Display_ShowIncompleteFrames, PropViewFrame::OnShowIncompleteFrames )
    EVT_MENU( miSettings_ShowStatusBar, PropViewFrame::OnShowStatusBar )
    EVT_MENU( miSettings_SetUpdateFrequency, PropViewFrame::OnSettings_SetUpdateFrequency )
    EVT_MENU( miSettings_ToggleFullScreenMode, PropViewFrame::OnToggleFullScreenMode )
    EVT_MENU( miSettings_AllowFastSingleFrameAcquisition, PropViewFrame::OnSettings_AllowFastSingleFrameAcquisition )
    EVT_MENU( miSettings_ShowLeftToolBar, PropViewFrame::OnShowLeftToolBar )
    EVT_MENU( miSettings_ShowUpperToolBar, PropViewFrame::OnShowUpperToolBar )
    EVT_MENU( miSettings_Analysis_ShowControls, PropViewFrame::OnSettings_Analysis_ShowControls )
    EVT_MENU( miSettings_Analysis_SynchronizeAOIs, PropViewFrame::OnSettings_Analysis_SynchronizeAOIs )
    EVT_MENU( miSettings_PropGrid_CreateEditorsWithSlider, PropViewFrame::OnSettings_PropGrid_CreateEditorsWithSlider )
    EVT_MENU( miSettings_PropGrid_Show, PropViewFrame::OnSettings_PropGrid_Show )
    EVT_MENU( miSettings_PropGrid_DisplayPropIndicesAsHex, PropViewFrame::OnSettings_PropGrid_UpdateViewMode )
    EVT_MENU( miSettings_PropGrid_DisplayToolTips, PropViewFrame::OnSettings_PropGrid_DisplayToolTips )
    EVT_MENU( miSettings_PropGrid_UseDisplayNameIfAvailable, PropViewFrame::OnSettings_PropGrid_UpdateViewMode )
    EVT_MENU( miSettings_PropGrid_UseSelectorGrouping, PropViewFrame::OnSettings_PropGrid_UseSelectorGrouping )
    EVT_MENU( miSettings_PropGrid_ViewMode_StandardView, PropViewFrame::OnSettings_PropGrid_ViewModeChanged )
    EVT_MENU( miSettings_PropGrid_ViewMode_DevelopersView, PropViewFrame::OnSettings_PropGrid_ViewModeChanged )
    EVT_MENU( miWizard_Open, PropViewFrame::OnWizard_Open )
    EVT_MENU( miWizards_ColorCorrection, PropViewFrame::OnWizards_ColorCorrection )
    EVT_MENU( miWizards_FileAccessControl_UploadFile, PropViewFrame::OnWizards_FileAccessControl_UploadFile )
    EVT_MENU( miWizards_FileAccessControl_DownloadFile, PropViewFrame::OnWizards_FileAccessControl_DownloadFile )
    EVT_MENU( miWizards_LensControl, PropViewFrame::OnWizards_LensControl )
    EVT_MENU( miWizards_LUTControl, PropViewFrame::OnWizards_LUTControl )
    EVT_MENU( miWizards_SequencerControl, PropViewFrame::OnWizards_SequencerControl )
    EVT_MENU( miWizards_QuickSetup, PropViewFrame::OnWizards_QuickSetup )
    EVT_MENU( miSettings_ShowQuickSetupOnUse, PropViewFrame::OnSettings_ShowQuickSetupOnUse )
    EVT_MENU( miHelp_About, PropViewFrame::OnHelp_About )
    EVT_MENU( miHelp_DriverInformation, PropViewFrame::OnHelp_DriverInformation )
    EVT_MENU( miHelp_FindFeature, PropViewFrame::OnHelp_FindFeature )
    EVT_MENU( miHelp_OnlineDocumentation, PropViewFrame::OnHelp_OnlineDocumentation )
#ifdef _WIN32
    EVT_MENU( miHelp_EmailLogFilesZip, PropViewFrame::OnHelp_EmailLogFilesZip )
    EVT_MENU( miHelp_OpenLogFilesFolder, PropViewFrame::OnHelp_OpenLogFilesFolder )
    EVT_MENU( miHelp_SaveLogFilesAsZip, PropViewFrame::OnHelp_SaveLogFilesAsZip )
#endif // _WIN32
    EVT_COMMAND_SCROLL_THUMBTRACK( widSLRecordDisplay, PropViewFrame::OnSLRecordDisplay )
    EVT_SIZE( PropViewFrame::OnSize )
    EVT_BUTTON( miAction_UseDevice, PropViewFrame::OnUseDevice )
    EVT_BUTTON( miAction_UpdateDeviceList, PropViewFrame::OnUpdateDeviceList )
    EVT_BUTTON( miCapture_Acquire, PropViewFrame::OnBtnAcquire )
    EVT_BUTTON( miCapture_Abort, PropViewFrame::OnBtnAbort )
    EVT_BUTTON( miCapture_Unlock, PropViewFrame::OnBtnUnlock )
    EVT_COMMAND( widMainFrame, imageReadyEvent, PropViewFrame::OnImageReady )
    EVT_COMMAND( widMainFrame, sequenceReadyEvent, PropViewFrame::OnSequenceReady )
    EVT_COMMAND( widMainFrame, imageSkippedEvent, PropViewFrame::OnImageSkipped )
    EVT_COMMAND( widMainFrame, toggleDisplayArea, PropViewFrame::OnToggleDisplayWindowSize )
    EVT_COMMAND( widMainFrame, liveModeAborted, PropViewFrame::OnLiveModeAborted )
    EVT_COMMAND( widMainFrame, imageTimeoutEvent, PropViewFrame::OnImageTimeoutEvent )
    EVT_COMMAND( widMainFrame, imageInfoEvent, PropViewFrame::OnImageInfo )
    EVT_COMMAND( widMainFrame, refreshAOIControls, PropViewFrame::OnRefreshAOIControls )
    EVT_COMMAND( widMainFrame, refreshCurrentPixelData, PropViewFrame::OnRefreshCurrentPixelData )
    EVT_COMMAND( widMainFrame, imageCanvasSelected, PropViewFrame::OnImageCanvasSelected )
    EVT_COMMAND( widMainFrame, imageCanvasFullScreen, PropViewFrame::OnImageCanvasFullScreen )
    EVT_COMMAND( widMainFrame, monitorImageClosed, PropViewFrame::OnMonitorImageClosed )
    EVT_COMMAND( widMainFrame, featureChangedCallbackReceived, PropViewFrame::OnFeatureChangedCallbackReceived )
    EVT_NOTEBOOK_PAGE_CHANGED( widNotebook, PropViewFrame::OnNotebookPageChanged )
    EVT_TEXT( widDevCombo, PropViewFrame::OnDevComboTextChanged )
    EVT_TEXT( widUserExperienceCombo, PropViewFrame::OnUserExperienceComboTextChanged )
    EVT_CLOSE( PropViewFrame::OnClose )
    EVT_SPLITTER_SASH_POS_CHANGED( widHorSplitter, PropViewFrame::OnSplitterSashPosChanged )
    EVT_SPLITTER_SASH_POS_CHANGED( widVerSplitter, PropViewFrame::OnSplitterSashPosChanged )

    EVT_CHECKBOX( widPixelHistogram | iapidCBAOIFullMode, PropViewFrame::OnAnalysisPlotCBAOIFullMode )
    EVT_CHECKBOX( widPixelHistogram | iapidCBProcessBayerParity, PropViewFrame::OnAnalysisPlotCBProcessBayerParity )
    EVT_SPINCTRL( widPixelHistogram | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxChanged )
    EVT_SPINCTRL( widPixelHistogram | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyChanged )
    EVT_SPINCTRL( widPixelHistogram | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwChanged )
    EVT_SPINCTRL( widPixelHistogram | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhChanged )
    EVT_SPINCTRL( widPixelHistogram | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartChanged )
    EVT_SPINCTRL( widPixelHistogram | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowChanged )
    EVT_SPINCTRL( widPixelHistogram | iapidSCDrawStepWidth, PropViewFrame::OnAnalysisPlotDrawStepWidthChanged )
    EVT_SPINCTRL( widPixelHistogram | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widPixelHistogram | iapidNBDisplayMethod, PropViewFrame::OnNBDisplayMethodPageChanged )

    EVT_CHECKBOX( widSpatialNoiseHistogram | iapidCBAOIFullMode, PropViewFrame::OnAnalysisPlotCBAOIFullMode )
    EVT_CHECKBOX( widSpatialNoiseHistogram | iapidCBProcessBayerParity, PropViewFrame::OnAnalysisPlotCBProcessBayerParity )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxChanged )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyChanged )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwChanged )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhChanged )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartChanged )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowChanged )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCDrawStepWidth, PropViewFrame::OnAnalysisPlotDrawStepWidthChanged )
    EVT_SPINCTRL( widSpatialNoiseHistogram | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widSpatialNoiseHistogram | iapidNBDisplayMethod, PropViewFrame::OnNBDisplayMethodPageChanged )

    EVT_CHECKBOX( widTemporalNoiseHistogram | iapidCBAOIFullMode, PropViewFrame::OnAnalysisPlotCBAOIFullMode )
    EVT_CHECKBOX( widTemporalNoiseHistogram | iapidCBProcessBayerParity, PropViewFrame::OnAnalysisPlotCBProcessBayerParity )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxChanged )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyChanged )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwChanged )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhChanged )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartChanged )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowChanged )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCDrawStepWidth, PropViewFrame::OnAnalysisPlotDrawStepWidthChanged )
    EVT_SPINCTRL( widTemporalNoiseHistogram | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widTemporalNoiseHistogram | iapidNBDisplayMethod, PropViewFrame::OnNBDisplayMethodPageChanged )

    EVT_CHECKBOX( widLineProfileHorizontal | iapidCBAOIFullMode, PropViewFrame::OnAnalysisPlotCBAOIFullMode )
    EVT_CHECKBOX( widLineProfileHorizontal | iapidCBProcessBayerParity, PropViewFrame::OnAnalysisPlotCBProcessBayerParity )
    EVT_SPINCTRL( widLineProfileHorizontal | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxChanged )
    EVT_SPINCTRL( widLineProfileHorizontal | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyChanged )
    EVT_SPINCTRL( widLineProfileHorizontal | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwChanged )
    EVT_SPINCTRL( widLineProfileHorizontal | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhChanged )
    EVT_SPINCTRL( widLineProfileHorizontal | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartChanged )
    EVT_SPINCTRL( widLineProfileHorizontal | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowChanged )
    EVT_SPINCTRL( widLineProfileHorizontal | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widLineProfileHorizontal | iapidNBDisplayMethod, PropViewFrame::OnNBDisplayMethodPageChanged )

    EVT_CHECKBOX( widLineProfileVertical | iapidCBAOIFullMode, PropViewFrame::OnAnalysisPlotCBAOIFullMode )
    EVT_CHECKBOX( widLineProfileVertical | iapidCBProcessBayerParity, PropViewFrame::OnAnalysisPlotCBProcessBayerParity )
    EVT_SPINCTRL( widLineProfileVertical | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxChanged )
    EVT_SPINCTRL( widLineProfileVertical | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyChanged )
    EVT_SPINCTRL( widLineProfileVertical | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwChanged )
    EVT_SPINCTRL( widLineProfileVertical | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhChanged )
    EVT_SPINCTRL( widLineProfileVertical | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartChanged )
    EVT_SPINCTRL( widLineProfileVertical | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowChanged )
    EVT_SPINCTRL( widLineProfileVertical | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widLineProfileVertical | iapidNBDisplayMethod, PropViewFrame::OnNBDisplayMethodPageChanged )

    EVT_CHECKBOX( widIntensityPlot | iapidCBAOIFullMode, PropViewFrame::OnAnalysisPlotCBAOIFullMode )
    EVT_CHECKBOX( widIntensityPlot | iapidCBProcessBayerParity, PropViewFrame::OnAnalysisPlotCBProcessBayerParity )
    EVT_SPINCTRL( widIntensityPlot | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxChanged )
    EVT_SPINCTRL( widIntensityPlot | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyChanged )
    EVT_SPINCTRL( widIntensityPlot | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwChanged )
    EVT_SPINCTRL( widIntensityPlot | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhChanged )
    EVT_TEXT( widIntensityPlot | iapidCoBPlotSelection , PropViewFrame::OnAnalysisPlot_PlotSelectionChanged )
    EVT_SPINCTRL( widIntensityPlot | iapidSCHistoryDepth, PropViewFrame::OnAnalysisPlotHistoryDepthChanged )
    EVT_SPINCTRL( widIntensityPlot | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartChanged )
    EVT_SPINCTRL( widIntensityPlot | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowChanged )
    EVT_SPINCTRL( widIntensityPlot | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widIntensityPlot | iapidNBDisplayMethod, PropViewFrame::OnNBDisplayMethodPageChanged )

    EVT_CHECKBOX( widVectorScope | iapidCBAOIFullMode, PropViewFrame::OnAnalysisPlotCBAOIFullMode )
    EVT_SPINCTRL( widVectorScope | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxChanged )
    EVT_SPINCTRL( widVectorScope | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyChanged )
    EVT_SPINCTRL( widVectorScope | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwChanged )
    EVT_SPINCTRL( widVectorScope | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhChanged )
    EVT_SPINCTRL( widVectorScope | iapidSCGridStepsX, PropViewFrame::OnAnalysisPlotGridXStepsChanged )
    EVT_SPINCTRL( widVectorScope | iapidSCGridStepsY, PropViewFrame::OnAnalysisPlotGridYStepsChanged )
    EVT_SPINCTRL( widVectorScope | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widVectorScope | iapidNBDisplayMethod, PropViewFrame::OnNBDisplayMethodPageChanged )

    EVT_TEXT( widInfoPlot_PlotSelectionCombo, PropViewFrame::OnInfoPlot_PlotSelectionComboTextChanged )
    EVT_CHECKBOX( widCBEnableInfoPlot, PropViewFrame::OnCBEnableInfoPlot )
    EVT_SPINCTRL( widSCInfoPlotHistoryDepth, PropViewFrame::OnInfoPlotUpdateHistoryDepthChanged )
    EVT_SPINCTRL( widSCInfoPlotUpdateSpeed, PropViewFrame::OnInfoPlotUpdateSpeedChanged )
    EVT_CHECKBOX( widCBInfoPlotDifferences, PropViewFrame::OnCBInfoPlotDifferences )
    EVT_CHECKBOX( widCBInfoPlotAutoScale, PropViewFrame::OnCBInfoPlotAutoScale )

    EVT_CHECKBOX( widCBEnableFeatureValueVsTimePlot, PropViewFrame::OnCBEnableFeatureValueVsTimePlot )
    EVT_SPINCTRL( widSCFeatureValueVsTimePlotHistoryDepth, PropViewFrame::OnFeatureValueVsTimePlotUpdateHistoryDepthChanged )
    EVT_SPINCTRL( widSCFeatureValueVsTimePlotUpdateSpeed, PropViewFrame::OnFeatureValueVsTimePlotUpdateSpeedChanged )
    EVT_CHECKBOX( widCBFeatureValueVsTimePlotDifferences, PropViewFrame::OnCBFeatureValueVsTimePlotDifferences )
    EVT_CHECKBOX( widCBFeatureValueVsTimePlotAutoScale, PropViewFrame::OnCBFeatureValueVsTimePlotAutoScale )

#ifdef BUILD_WITH_TEXT_EVENTS_FOR_SPINCTRL // Unfortunately on Linux wxWidgets 2.6.x - ??? handling these messages will cause problems, while on Windows not doing so will not always update the GUI as desired :-(
    EVT_TEXT( widPixelHistogram | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxTextChanged )
    EVT_TEXT( widPixelHistogram | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyTextChanged )
    EVT_TEXT( widPixelHistogram | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwTextChanged )
    EVT_TEXT( widPixelHistogram | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhTextChanged )
    EVT_TEXT( widPixelHistogram | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartTextChanged )
    EVT_TEXT( widPixelHistogram | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowTextChanged )
    EVT_TEXT( widPixelHistogram | iapidSCDrawStepWidth, PropViewFrame::OnAnalysisPlotDrawStepWidthTextChanged )
    EVT_TEXT( widPixelHistogram | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedTextChanged )

    EVT_TEXT( widSpatialNoiseHistogram | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxTextChanged )
    EVT_TEXT( widSpatialNoiseHistogram | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyTextChanged )
    EVT_TEXT( widSpatialNoiseHistogram | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwTextChanged )
    EVT_TEXT( widSpatialNoiseHistogram | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhTextChanged )
    EVT_TEXT( widSpatialNoiseHistogram | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartTextChanged )
    EVT_TEXT( widSpatialNoiseHistogram | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowTextChanged )
    EVT_TEXT( widSpatialNoiseHistogram | iapidSCDrawStepWidth, PropViewFrame::OnAnalysisPlotDrawStepWidthTextChanged )
    EVT_TEXT( widSpatialNoiseHistogram | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedTextChanged )

    EVT_TEXT( widTemporalNoiseHistogram | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxTextChanged )
    EVT_TEXT( widTemporalNoiseHistogram | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyTextChanged )
    EVT_TEXT( widTemporalNoiseHistogram | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwTextChanged )
    EVT_TEXT( widTemporalNoiseHistogram | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhTextChanged )
    EVT_TEXT( widTemporalNoiseHistogram | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartTextChanged )
    EVT_TEXT( widTemporalNoiseHistogram | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowTextChanged )
    EVT_TEXT( widTemporalNoiseHistogram | iapidSCDrawStepWidth, PropViewFrame::OnAnalysisPlotDrawStepWidthTextChanged )
    EVT_TEXT( widTemporalNoiseHistogram | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedTextChanged )

    EVT_TEXT( widLineProfileHorizontal | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxTextChanged )
    EVT_TEXT( widLineProfileHorizontal | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyTextChanged )
    EVT_TEXT( widLineProfileHorizontal | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwTextChanged )
    EVT_TEXT( widLineProfileHorizontal | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhTextChanged )
    EVT_TEXT( widLineProfileHorizontal | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartTextChanged )
    EVT_TEXT( widLineProfileHorizontal | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowTextChanged )
    EVT_TEXT( widLineProfileHorizontal | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedTextChanged )

    EVT_TEXT( widLineProfileVertical | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxTextChanged )
    EVT_TEXT( widLineProfileVertical | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyTextChanged )
    EVT_TEXT( widLineProfileVertical | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwTextChanged )
    EVT_TEXT( widLineProfileVertical | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhTextChanged )
    EVT_TEXT( widLineProfileVertical | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartTextChanged )
    EVT_TEXT( widLineProfileVertical | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowTextChanged )
    EVT_TEXT( widLineProfileVertical | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedTextChanged )

    EVT_TEXT( widIntensityPlot | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxTextChanged )
    EVT_TEXT( widIntensityPlot | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyTextChanged )
    EVT_TEXT( widIntensityPlot | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwTextChanged )
    EVT_TEXT( widIntensityPlot | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhTextChanged )
    EVT_TEXT( widIntensityPlot | iapidSCHistoryDepth, PropViewFrame::OnAnalysisPlotHistoryDepthTextChanged )
    EVT_TEXT( widIntensityPlot | iapidSCDrawStart_percent, PropViewFrame::OnAnalysisPlotDrawStartTextChanged )
    EVT_TEXT( widIntensityPlot | iapidSCDrawWindow_percent, PropViewFrame::OnAnalysisPlotDrawWindowTextChanged )
    EVT_TEXT( widIntensityPlot | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedTextChanged )

    EVT_TEXT( widVectorScope | iapidSCAOIx, PropViewFrame::OnAnalysisPlotAOIxTextChanged )
    EVT_TEXT( widVectorScope | iapidSCAOIy, PropViewFrame::OnAnalysisPlotAOIyTextChanged )
    EVT_TEXT( widVectorScope | iapidSCAOIw, PropViewFrame::OnAnalysisPlotAOIwTextChanged )
    EVT_TEXT( widVectorScope | iapidSCAOIh, PropViewFrame::OnAnalysisPlotAOIhTextChanged )
    EVT_TEXT( widVectorScope | iapidSCGridStepsX, PropViewFrame::OnAnalysisPlotGridXStepsTextChanged )
    EVT_TEXT( widVectorScope | iapidSCGridStepsY, PropViewFrame::OnAnalysisPlotGridYStepsTextChanged )
    EVT_TEXT( widVectorScope | iapidSCUpdateSpeed, PropViewFrame::OnAnalysisPlotUpdateSpeedTextChanged )

    EVT_TEXT( widSCInfoPlotHistoryDepth, PropViewFrame::OnInfoPlotUpdateHistoryDepthTextChanged )
    EVT_TEXT( widSCInfoPlotUpdateSpeed, PropViewFrame::OnInfoPlotUpdateSpeedTextChanged )

    EVT_TEXT( widSCFeatureValueVsTimePlotHistoryDepth, PropViewFrame::OnFeatureValueVsTimePlotUpdateHistoryDepthTextChanged )
    EVT_TEXT( widSCFeatureValueVsTimePlotUpdateSpeed, PropViewFrame::OnFeatureValueVsTimePlotUpdateSpeedTextChanged )
#endif // #ifdef BUILD_WITH_TEXT_EVENTS_FOR_SPINCTRL
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
PropViewFrame::PropViewFrame( const wxString& title, const wxPoint& pos, const wxSize& size, int argc, wxChar** argv )
    : PropGridFrameBase( widMainFrame, title, pos, size ), m_boAllowFullDeviceFileAccess( false ), m_boCloseDeviceInProgress( false ), m_boFindFeatureMatchCaseActive( false ),
      m_boHandleImageTimeoutEvents( true ), m_boDisplayWindowMaximized( false ), m_boShutdownInProgress( false ), m_boSingleCaptureInProgess( false ),
      m_boUpdateAOIInProgress( false ), m_boCurrentImageIsFromFile( false ), m_boNewImageDataAvailable( false ), m_boImageCanvasInFullScreenMode( false ), m_errorStyle( *wxRED ),
      m_boSelectedDeviceSupportsMultiFrame( false ), m_boSelectedDeviceSupportsSingleFrame( false ), m_CurrentImageAnalysisControlIndex( -1 ),
      m_DeviceListChangedCounter( numeric_limits<unsigned int>::max() ),
      m_DeviceCount( 0 ), m_ProductFirmwareTable(), m_NoDevStr( wxT( "No Device" ) ), m_SingleFrameStr( wxT( "SingleFrame" ) ), m_MultiFrameStr( wxT( "MultiFrame" ) ),
      m_ContinuousStr( wxT( "Continuous" ) ), m_pDevPropHandler( 0 ), m_pLastMouseHooverDisplay( 0 ), m_pCurrentAnalysisDisplay( 0 ),
      m_CurrentRequestDataIndex( 0 ), m_pDisplayPanel( 0 ), m_pDisplayGridSizer( 0 ), m_pUserExperienceCombo( 0 ), m_pMonitorImage( 0 ), m_pLUTControlDlg( 0 ),
      m_pColorCorrectionDlg( 0 ), m_pLensControlDlg( 0 ), m_pQuickSetupDlgCurrent( 0 ), m_pQuickSetupDlgGenICam( 0 ), m_pQuickSetupDlgDeviceSpecific( 0 ),
      m_boShowQuickSetupWizardCurrentProcess( false ), m_QuickSetupWizardEnforce( qswiDefaultBehaviour ) , m_pLogWindow( 0 ), m_pWindowDisabler( 0 ),
      m_pPropViewCallback( new PropViewCallback() ), m_currentWizard( wNone ), m_defaultDeviceInterfaceLayout( wxT( "GenICam" ) ),
      m_defaultImageProcessingMode( ipmProcessLatestOnly ), m_HardDiscRecordingParameters(), m_GUIBeforeQuickSetupWizard(),
      m_DisplayUpdateTimer( this, teUpdateDisplay ), m_displayUpdateFrequency( 1000 / DEFAULT_DISPLAY_UPDATE_PERIOD ),
      m_UseSmallIcons( true /* ( wxSystemSettings::GetMetric( wxSYS_SCREEN_X ) < 1280 ) || ( wxSystemSettings::GetMetric( wxSYS_SCREEN_Y ) < 1024 ) */ ),
      m_displayCountX( 1 ), m_displayCountY( 1 )
//-----------------------------------------------------------------------------
{
    // sub menu 'Action -> Default Device Interface'
    wxMenu* pMenuActionDefaultDeviceInterface = new wxMenu;
    wxMenuItem* pMIAction_DefaultDeviceInterface_DeviceSpecific = pMenuActionDefaultDeviceInterface->Append( miAction_DefaultDeviceInterface_DeviceSpecific, wxT( "Device Specific\tF2" ), wxT( "" ), wxITEM_RADIO );
    wxMenuItem* pMIAction_DefaultDeviceInterface_GenICam = pMenuActionDefaultDeviceInterface->Append( miAction_DefaultDeviceInterface_GenICam, wxT( "GenICam\tF3" ), wxT( "" ), wxITEM_RADIO );

    // sub menu 'Action -> Settings -> Save'
    wxMenu* pMenuActionCaptureSettingsSave = new wxMenu;
    m_pMIAction_CaptureSettings_Save_ToDefault = pMenuActionCaptureSettingsSave->Append( miAction_CaptureSettings_Save_ToDefault, wxT( "As Default Settings For All Devices Belonging To The Same Family(Per User Only)" ) );
    m_pMIAction_CaptureSettings_Save_CurrentProduct = pMenuActionCaptureSettingsSave->Append( miAction_CaptureSettings_Save_CurrentProduct, wxT( "As Default Settings For All Devices Belonging To The Same Family And Product Type" ) );
    m_pMIAction_CaptureSettings_Save_ActiveDevice = pMenuActionCaptureSettingsSave->Append( miAction_CaptureSettings_Save_ActiveDevice, wxT( "As Default Settings For This Device(Serial Number)\tCTRL+S" ) );
    m_pMIAction_CaptureSettings_Save_ExportActiveDevice = pMenuActionCaptureSettingsSave->Append( miAction_CaptureSettings_Save_ExportActiveDevice, wxT( "To A File\tALT+CTRL+S" ), wxT( "Stores the current settings for this device in a platform independent XML file" ) );

    // sub menu 'Action -> Settings -> Load'
    wxMenu* pMenuActionCaptureSettingsLoad = new wxMenu;
    m_pMIAction_CaptureSettings_Load_FromDefault = pMenuActionCaptureSettingsLoad->Append( miAction_CaptureSettings_Load_FromDefault, wxT( "From The Default Settings Location For This Devices Family(Per User Only)" ) );
    m_pMIAction_CaptureSettings_Load_CurrentProduct = pMenuActionCaptureSettingsLoad->Append( miAction_CaptureSettings_Load_CurrentProduct, wxT( "From The Default Settings Location For This Devices Family And Product Type" ) );
    m_pMIAction_CaptureSettings_Load_ActiveDevice = pMenuActionCaptureSettingsLoad->Append( miAction_CaptureSettings_Load_ActiveDevice, wxT( "From The Default Settings Location For This Device(Serial Number)\tCTRL+O" ) );
    m_pMIAction_CaptureSettings_Load_ActiveDeviceFromFile = pMenuActionCaptureSettingsLoad->Append( miAction_CaptureSettings_Load_ActiveDeviceFromFile, wxT( "From A File\tALT+CTRL+O" ), wxT( "Tries to load settings for this device from a platform independent XML file" ) );

    // sub menu 'Action -> Settings'
    wxMenu* pMenuActionCaptureSettings = new wxMenu;
    pMenuActionCaptureSettings->Append( wxID_ANY, wxT( "Save Active Device Settings" ), pMenuActionCaptureSettingsSave );
    pMenuActionCaptureSettings->Append( wxID_ANY, wxT( "Load Active Device Settings" ), pMenuActionCaptureSettingsLoad );
    m_pMIAction_CaptureSettings_Manage = pMenuActionCaptureSettings->Append( miAction_CaptureSettings_Manage, wxT( "Manage..." ) );

    // sub menu 'Action -> Save All Recorded Images'
    wxMenu* pMenuAction_SaveAllRecordedImages = new wxMenu;
    m_pMIAction_SaveImageSequenceToFiles = pMenuAction_SaveAllRecordedImages->Append( miAction_SaveImageSequenceToFiles, wxT( "Save Images Into Separate Files..." ) );
    m_pMIAction_SaveImageSequenceToStream = pMenuAction_SaveAllRecordedImages->Append( miAction_SaveImageSequenceToStream, wxT( "Save Images Into Stream..." ) );

    // menu 'Action'
    wxMenu* pMenuAction = new wxMenu;
    m_pMIAction_UpdateDeviceList = pMenuAction->Append( miAction_UpdateDeviceList, wxT( "Update Device List\tF5" ) );
    m_pMIAction_DisplayConnectedDevicesOnly = pMenuAction->Append( miAction_DisplayConnectedDevicesOnly, wxT( "Display &Connected Devices Only\tALT+CTRL+D" ), wxT( "" ), wxITEM_CHECK );
    pMenuAction->Append( wxID_ANY, wxT( "Default Device Interface Layout" ), pMenuActionDefaultDeviceInterface );
    m_pMIAction_AutomaticallyReconnectToUnusedDevices = pMenuAction->Append( miAction_AutomaticallyReconnectToUnusedDevices, wxT( "Automatically Reconnect To Unused Devices" ), wxT( "" ), wxITEM_CHECK );
    pMenuAction->AppendSeparator();
    m_pMIAction_Use = pMenuAction->Append( miAction_UseDevice, wxT( "&Use Device\tCTRL+U" ), wxT( "Opens or closes the selected device" ), wxITEM_CHECK );
    pMenuAction->Append( wxID_ANY, wxT( "Capture Settings" ), pMenuActionCaptureSettings );
    pMenuAction->AppendSeparator();
    m_pMIAction_LoadImage = pMenuAction->Append( miAction_LoadImage, wxT( "Load Image..." ) );
    m_pMIAction_SaveImage = pMenuAction->Append( miAction_SaveImage, wxT( "Save Image..." ) );
    pMenuAction->Append( wxID_ANY, wxT( "Save All Recorded Images" ), pMenuAction_SaveAllRecordedImages );
    pMenuAction->AppendSeparator();
    pMenuAction->Append( miAction_Exit, wxT( "E&xit\tALT+X" ) );

    // sub menu 'Capture -> Default Image Processing Mode'
    wxMenu* pMenuActionDefaultImageProcessingMode = new wxMenu;
    m_pMICapture_DefaultImageProcessingMode_ProcessAll = pMenuActionDefaultImageProcessingMode->Append( miCapture_DefaultImageProcessingMode_ProcessAll, wxT( "Process All" ), wxT( "" ), wxITEM_RADIO );
    m_pMICapture_DefaultImageProcessingMode_ProcessLatestOnly = pMenuActionDefaultImageProcessingMode->Append( miCapture_DefaultImageProcessingMode_ProcessLatestOnly, wxT( "Process Latest Only" ), wxT( "" ), wxITEM_RADIO );

    // sub menu 'Capture -> Recording'
    wxMenu* pMenuSettingsRecording = new wxMenu;
    m_pMICapture_Recording_SlientMode = pMenuSettingsRecording->AppendCheckItem( miCapture_Recording_SilentMode, wxT( "Silent Mode" ), wxT( "Check this to disable all recording related dialogs" ) );
    m_pMICapture_Recording_Continuous = pMenuSettingsRecording->AppendCheckItem( miCapture_Recording_Continuous, wxT( "Continuous" ), wxT( "Check this to enable continuous recording" ) );
    m_pMICapture_Recording_SetupSequenceSize = pMenuSettingsRecording->Append( miCapture_Recording_SetupSequenceSize, wxT( "Setup Sequence Size" ), wxT( "Sets the number of buffers that will be recorded during a recording session that stops automatically" ) );
    m_pMICapture_Recording_SetupHardDiscRecording = pMenuSettingsRecording->Append( miCapture_Recording_SetupHardDiscRecording, wxT( "Setup Hard Disc Recording" ), wxT( "Configures automatic hard disc recording" ) );

    // sub menu 'Capture -> Capture Settings -> Usage Mode'
    wxMenu* pMenuCaptureCaptureSettingsUsageMode = new wxMenu;
    m_pMICapture_CaptureSettings_UsageMode_Manual = pMenuCaptureCaptureSettingsUsageMode->Append( miCapture_CaptureSettings_UsageMode_Manual, wxT( "Manual\tF9" ), wxT( "" ), wxITEM_RADIO );
    m_pMICapture_CaptureSettings_UsageMode_Automatic = pMenuCaptureCaptureSettingsUsageMode->Append( miCapture_CaptureSettings_UsageMode_Automatic, wxT( "Automatic\tF10" ), wxT( "" ), wxITEM_RADIO );

    // sub menu 'Capture -> Capture Settings'
    wxMenu* pMenuCaptureCaptureSettings = new wxMenu;
    m_pMICapture_CaptureSettings_CreateCaptureSetting = pMenuCaptureCaptureSettings->Append( miCapture_CaptureSettings_CreateCaptureSetting, wxT( "Create Capture Setting...\tCTRL+C" ), wxT( "Creates a new capture setting to configure for acquisition" ) );
    m_pMICapture_CaptureSettings_CaptureSettingHierarchy = pMenuCaptureCaptureSettings->Append( miCapture_CaptureSettings_CaptureSettingHierarchy, wxT( "Capture Setting Hierarchy...\tCTRL+H" ), wxT( "Displays the current capture setting parent <-> child relationships" ) );
    m_pMICapture_CaptureSettings_AssignToDisplays = pMenuCaptureCaptureSettings->Append( miCapture_CaptureSettings_AssignToDisplays, wxT( "Assign To Display(s)...\tCTRL+D" ), wxT( "Opens a dialog to configure the display <-> setting relationships if there is more than one display" ) );
    pMenuCaptureCaptureSettings->Append( wxID_ANY, wxT( "Usage Mode" ), pMenuCaptureCaptureSettingsUsageMode );

    // menu 'Capture'
    wxMenu* pMenuCapture = new wxMenu;
    m_pMICapture_Acquire = pMenuCapture->Append( miCapture_Acquire, wxT( "A&cquire\tALT+CTRL+C" ), wxT( "Starts or stops an acquisition" ), wxITEM_CHECK );
    m_pMICapture_Abort = pMenuCapture->Append( miCapture_Abort, wxT( "&Abort\tCTRL+A" ), wxT( "Clears the request queue, but does NOT stop a running continuous acquisition" ) );
    m_pMICapture_Unlock = pMenuCapture->Append( miCapture_Unlock, wxT( "Un&lock\tALT+CTRL+L" ), wxT( "Unlocks ALL requests locked by the application and deep copies all images currently displayed to preserve the current state of displays and analysis plots. Afterwards the request count can be modified even if one or more images are currently being displayed" ) );
    pMenuCapture->Append( wxID_ANY, wxT( "Default Image Processing Mode" ), pMenuActionDefaultImageProcessingMode );
    pMenuCapture->AppendSeparator();
    m_pMICapture_Record = pMenuCapture->Append( miCapture_Record, wxT( "&Record\tCTRL+R" ), wxT( "Records the next 'RequestCount'(System Settings) images" ), wxITEM_CHECK );
    m_pMICapture_Forward = pMenuCapture->Append( miCapture_Forward, wxT( "&Forward\tALT+CTRL+F" ), wxT( "Displays the next recorded image" ) );
    m_pMICapture_Backward = pMenuCapture->Append( miCapture_Backward, wxT( "&Backward\tALT+CTRL+B" ), wxT( "Displays the previous recorded image" ) );
    pMenuCapture->Append( wxID_ANY, wxT( "&Recording" ), pMenuSettingsRecording );
    pMenuCapture->AppendSeparator();
    pMenuCapture->Append( wxID_ANY, wxT( "Capture Settings" ), pMenuCaptureCaptureSettings );
    m_pMICapture_SetupCaptureQueueDepth = pMenuCapture->Append( miCapture_SetupCaptureQueueDepth, wxT( "Setup Capture Queue Depth\tCTRL+Q" ), wxT( "Sets the number of buffers that will be used to pre-fill the capture queue" ) );
    m_pMICapture_DetailedRequestInformation = pMenuCapture->Append( miCapture_DetailedRequestInformation, wxT( "Detailed Request Information..." ), wxT( "Opens a dialog that displays detailed information about each capture buffer" ) );

    // sub menu 'Settings -> Property Grid -> View Mode'
    wxMenu* pMenuSettingPropertyGridViewMode = new wxMenu;
    m_pMISettings_PropGrid_ViewMode_StandardView = pMenuSettingPropertyGridViewMode->Append( miSettings_PropGrid_ViewMode_StandardView, wxT( "Standard View\tF6" ), wxT( "" ), wxITEM_RADIO );
    m_pMISettings_PropGrid_ViewMode_DevelopersView = pMenuSettingPropertyGridViewMode->Append( miSettings_PropGrid_ViewMode_DevelopersView, wxT( "Developers View\tF7" ), wxT( "" ), wxITEM_RADIO );

    // sub menu 'Settings -> Property Grid'
    wxMenu* pMenuSettingsPropertyGrid = new wxMenu;
    m_pMISettings_PropGrid_Show = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_Show, wxT( "Show &Property Grid\tALT+CTRL+P" ), wxT( "" ), wxITEM_CHECK );
    pMenuSettingsPropertyGrid->Append( wxID_ANY, wxT( "View Mode" ), pMenuSettingPropertyGridViewMode );
    pMenuSettingsPropertyGrid->AppendSeparator();
    m_pMISettings_PropGrid_DisplayPropIndicesAsHex = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_DisplayPropIndicesAsHex, wxT( "Display Property Indices As &Hex\tALT+CTRL+H" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_PropGrid_DisplayToolTips = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_DisplayToolTips, wxT( "Display Tool Tips" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_PropGrid_UseDisplayNameIfAvailable = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_UseDisplayNameIfAvailable, wxT( "Use Dis&play Names In Favour Of Feature Names\tCTRL+P" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_PropGrid_UseSelectorGrouping = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_UseSelectorGrouping, wxT( "Use Selector Grouping\tCTRL+G" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_PropGrid_CreateEditorsWithSlider = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_CreateEditorsWithSlider, wxT( "Create Editors With Slider\tALT+CTRL+R" ), wxT( "" ), wxITEM_CHECK );
    pMenuSettingsPropertyGrid->AppendSeparator();
    m_pMISettings_PropGrid_ShowMethodExecutionErrors = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_ShowMethodExecutionErrors, wxT( "Show Method Execution Errors\tALT+CTRL+E" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_PropGrid_ShowFeatureChangeTimeConsumption = pMenuSettingsPropertyGrid->Append( miSettings_PropGrid_ShowFeatureChangeTimeConsumption, wxT( "Show Feature Change Time Consumption" ), wxT( "" ), wxITEM_CHECK );

    // sub menu 'Settings -> Display'
    wxMenu* pMenuSettingsDisplay = new wxMenu;
    m_pMISettings_Display_ConfigureImageDisplayCount = pMenuSettingsDisplay->Append( miSettings_Display_ConfigureImageDisplayCount, wxT( "Configure Image Display Count" ), wxT( "Allows to specify the number of display windows in X- and Y-direction" ) );
    m_pMISettings_Display_Active = pMenuSettingsDisplay->Append( miSettings_Display_Active, wxT( "Show Image Display(&Blit)\tCTRL+B" ), wxT( "If not active images will be captured, but NOT displayed" ), wxITEM_CHECK );
    pMenuSettingsDisplay->AppendSeparator();
    m_pMISettings_Display_ShowIncompleteFrames = pMenuSettingsDisplay->Append( miSettings_Display_ShowIncompleteFrames, wxT( "Show &Incomplete Frames\tALT+CTRL+I" ), wxT( "If active, incomplete frames are displayed as well" ), wxITEM_CHECK );
    m_pMISettings_Display_ShowMonitorImage = pMenuSettingsDisplay->Append( miSettings_Display_ShowMonitorImage, wxT( "Show &Monitor Image\tCTRL+M" ), wxT( "" ) , wxITEM_CHECK );

    // sub menu 'Settings -> Analysis'
    wxMenu* pMenuSettingsAnalysis = new wxMenu;
    m_pMISettings_Analysis_ShowControls = pMenuSettingsAnalysis->Append( miSettings_Analysis_ShowControls, wxT( "Show &Analysis Tabs\tALT+CTRL+A" ), wxT( "" ), wxITEM_CHECK );
    pMenuSettingsAnalysis->AppendSeparator();
    m_pMISettings_Analysis_SynchroniseAOIs = pMenuSettingsAnalysis->Append( miSettings_Analysis_SynchronizeAOIs, wxT( "Synchronise AOIs" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_Analysis_DetailedInfoOnCallback = pMenuSettingsAnalysis->Append( wxID_ANY, wxT( "Detailed Info On Callback" ), wxT( "" ), wxITEM_CHECK );

    // menu 'Settings'
    wxMenu* pMenuSettings = new wxMenu;
    pMenuSettings->Append( wxID_ANY, wxT( "Property Grid" ), pMenuSettingsPropertyGrid );
    pMenuSettings->Append( wxID_ANY, wxT( "Image Display" ), pMenuSettingsDisplay );
    pMenuSettings->Append( wxID_ANY, wxT( "Analysis" ), pMenuSettingsAnalysis );
    pMenuSettings->AppendSeparator();
    m_pMISettings_AllowFastSingleFrameAcquisition = pMenuSettings->Append( miSettings_AllowFastSingleFrameAcquisition, wxT( "Allow Fast Single Frame Acquisition" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_ShowLeftToolBar = pMenuSettings->Append( miSettings_ShowLeftToolBar, wxT( "Show L&eft Tool Bar\tCTRL+E" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_ShowUpperToolBar = pMenuSettings->Append( miSettings_ShowUpperToolBar, wxT( "Show Upper &Tool Bar\tCTRL+T" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_ShowStatusBar = pMenuSettings->Append( miSettings_ShowStatusBar, wxT( "Show S&tatus Bar\tALT+CTRL+T" ), wxT( "" ), wxITEM_CHECK );
    pMenuSettings->AppendSeparator();
    m_pMISettings_ShowQuickSetupOnUse = pMenuSettings->Append( miSettings_ShowQuickSetupOnUse, wxT( "Show Quick Setup On Device Open" ), wxT( "" ), wxITEM_CHECK );
    pMenuSettings->AppendSeparator();
    m_pMISettings_WarnOnOutdatedFirmware = pMenuSettings->Append( wxID_ANY, wxT( "Warn On Outdated Firmware" ), wxT( "" ), wxITEM_CHECK );
    m_pMISettings_WarnOnReducedDriverPerformance = pMenuSettings->Append( wxID_ANY, wxT( "Warn On Reduced Driver Performance" ), wxT( "" ), wxITEM_CHECK );
#if defined(linux) || defined(__linux) || defined(__linux__)
    m_pMISettings_WarnOnPotentialBufferIssues = pMenuSettings->Append( wxID_ANY, wxT( "Warn On Potential Network/USB Buffer Issues" ), wxT( "" ), wxITEM_CHECK );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    m_pMISettings_WarnOnPotentialFirewallIssues = pMenuSettings->Append( wxID_ANY, wxT( "Warn On Potential Firewall Issues" ), wxT( "" ), wxITEM_CHECK );
    pMenuSettings->AppendSeparator();
    pMenuSettings->Append( miSettings_SetUpdateFrequency, wxT( "Set Update Frequency..." ) );
    m_pMISettings_ToggleFullScreenMode = pMenuSettings->Append( miSettings_ToggleFullScreenMode, wxT( "Full Screen Mode\tF11" ), wxT( "" ), wxITEM_CHECK );

    // sub menu 'Wizards -> File Access Control'
    wxMenu* pMenuWizards_FileAccessControl = new wxMenu;
    m_pMIWizards_FileAccessControl_UploadFile = pMenuWizards_FileAccessControl->Append( miWizards_FileAccessControl_UploadFile, wxT( "Upload File..." ) );
    m_pMIWizards_FileAccessControl_DownloadFile = pMenuWizards_FileAccessControl->Append( miWizards_FileAccessControl_DownloadFile, wxT( "Download File..." ) );

    // menu 'Wizards'
    wxMenu* pMenuWizards = new wxMenu;
    m_pMIWizards_ColorCorrection = pMenuWizards->Append( miWizards_ColorCorrection, wxT( "Color Correction..." ) );
    pMenuWizards->Append( wxID_ANY, wxT( "File Access Control" ), pMenuWizards_FileAccessControl );
    m_pMIWizards_LensControl = pMenuWizards->Append( miWizards_LensControl, wxT( "Lens Control..." ) );
    m_pMIWizards_LUTControl = pMenuWizards->Append( miWizards_LUTControl, wxT( "LUT Control..." ) );
    m_pMIWizards_QuickSetup = pMenuWizards->Append( miWizards_QuickSetup, wxT( "Quick Setup..." ) );
    m_pMIWizards_SequencerControl = pMenuWizards->Append( miWizards_SequencerControl, wxT( "Sequencer Control..." ) );

    // menu 'Help'
    wxMenu* pMenuHelp = new wxMenu;
    pMenuHelp->Append( miHelp_FindFeature, wxT( "&Find Feature...\tCTRL+F" ) );
    pMenuHelp->Append( miHelp_DriverInformation, wxT( "Driver Information...\tF8" ) );
    pMenuHelp->AppendSeparator();
#ifdef _WIN32
    pMenuHelp->Append( miHelp_OpenLogFilesFolder, wxT( "Open Logfiles Folder...\tCTRL+W" ) );
    pMenuHelp->Append( miHelp_SaveLogFilesAsZip, wxT( "Save Logfiles As Zip...\tALT+CTRL+U" ) );
    pMenuHelp->Append( miHelp_EmailLogFilesZip, wxT( "Email Logfiles to MV Support...\tALT+CTRL+W" ) );
    pMenuHelp->AppendSeparator();
#endif // _WIN32
    pMenuHelp->Append( miHelp_OnlineDocumentation, wxT( "Online Documentation...\tF12" ) );
    pMenuHelp->Append( miHelp_About, wxT( "About...\tF1" ) );

    // add all menus to the menu bar
    m_pMenuBar = new wxMenuBar;
    m_pMenuBar->Append( pMenuAction, wxT( "&Action" ) );
    m_pMenuBar->Append( pMenuCapture, wxT( "&Capture" ) );
    m_pMenuBar->Append( pMenuWizards, wxT( "&Wizards" ) );
    m_pMenuBar->Append( pMenuSettings, wxT( "&Settings" ) );
    m_pMenuBar->Append( pMenuHelp, wxT( "&Help" ) );
    // ... and attach this menu bar to the frame
    SetMenuBar( m_pMenuBar );

    // define the applications icon
    wxIcon icon( mvIcon_xpm );
    SetIcon( icon );

    wxConfigBase* pConfig = wxConfigBase::Get();
    m_displayCountX = pConfig->Read( wxT( "/MainFrame/displayCountX" ), 1l );
    m_displayCountY = pConfig->Read( wxT( "/MainFrame/displayCountY" ), 1l );

    // scan command line
    bool boDisplayDebugInfo = false;
    bool boDisplayFullTree = false;
    bool boDisplayInvisibleComponents = false;
    bool boOpenDeviceInLiveMode = false;
    wxString processedParameters;
    wxString serialToOpen;
    wxString fileToOpen;
    wxRect cmdLineRect( -1, -1, -1, -1 );
    wxString parserErrors;
    for( int i = 1; i < argc; i++ )
    {
        const wxString param( argv[i] );
        const wxString key = param.BeforeFirst( wxT( '=' ) );
        const wxString value = param.AfterFirst( wxT( '=' ) );
        if( key.IsEmpty() )
        {
            parserErrors.Append( wxString::Format( wxT( "Invalid command line parameter: '%s'. Ignored.\n" ), param.c_str() ) );
        }
        else
        {
            if( value.IsEmpty() )
            {
                fileToOpen = param;
            }
            else
            {
                if( ( key == wxT( "width" ) ) || ( key == wxT( "w" ) ) )
                {
                    cmdLineRect.width = atoi( value.mb_str() );
                }
                else if( ( key == wxT( "height" ) ) || ( key == wxT( "h" ) ) )
                {
                    cmdLineRect.height = atoi( value.mb_str() );
                }
                else if( ( key == wxT( "xpos" ) ) || ( key == wxT( "x" ) ) )
                {
                    cmdLineRect.x = atoi( value.mb_str() );
                }
                else if( ( key == wxT( "ypos" ) ) || ( key == wxT( "y" ) ) )
                {
                    cmdLineRect.y = atoi( value.mb_str() );
                }
                else if( ( key == wxT( "propgridwidth" ) ) || ( key == wxT( "pgw" ) ) )
                {
                    m_VerticalSplitterPos = atoi( value.mb_str() );
                }
                else if( ( key == wxT( "debuginfo" ) ) || ( key == wxT( "di" ) ) )
                {
                    boDisplayDebugInfo = ( atoi( value.mb_str() ) != 0 );
                }
                else if( ( key == wxT( "dic" ) ) )
                {
                    boDisplayInvisibleComponents = ( atoi( value.mb_str() ) != 0 );
                }
                else if( ( key == wxT( "fulltree" ) ) || ( key == wxT( "ft" ) ) )
                {
                    boDisplayFullTree = ( atoi( value.mb_str() ) != 0 );
                }
                else if( ( key == wxT( "device" ) ) || ( key == wxT( "d" ) ) )
                {
                    serialToOpen = value;
                }
                else if( key == wxT( "live" ) )
                {
                    boOpenDeviceInLiveMode = ( atoi( value.mb_str() ) != 0 );
                }
                else if( key == wxT( "qsw" ) )
                {
                    m_QuickSetupWizardEnforce = ( atoi( value.mb_str() ) != 0 ) ? qswiForceShow : qswiForceHide;
                }
                else if( ( key == wxT( "displayCountX" ) ) || ( key == wxT( "dcx" ) ) )
                {
                    m_displayCountX = atoi( value.mb_str() );
                    if( m_displayCountX > DISPLAY_COUNT_MAX )
                    {
                        m_displayCountX = DISPLAY_COUNT_MAX;
                        parserErrors.Append( wxString::Format( wxT( "Too many displays in X-direction: '%s'. %d is the current maximum.\n" ), param.c_str(), m_displayCountX ) );
                    }
                }
                else if( ( key == wxT( "displayCountY" ) ) || ( key == wxT( "dcy" ) ) )
                {
                    m_displayCountY = atoi( value.mb_str() );
                    if( m_displayCountY > DISPLAY_COUNT_MAX )
                    {
                        m_displayCountY = DISPLAY_COUNT_MAX;
                        parserErrors.Append( wxString::Format( wxT( "Too many displays in Y-direction: '%s'. %d is the current maximum.\n" ), param.c_str(), m_displayCountY ) );
                    }
                }
                else if( ( key == wxT( "allowFullDeviceFileAccess" ) ) )
                {
                    m_boAllowFullDeviceFileAccess = ( atoi( value.mb_str() ) != 0 );
                }
                else
                {
                    parserErrors.Append( wxString::Format( wxT( "Invalid command line parameter: '%s'. Ignored.\n" ), param.c_str() ) );
                }
            }
            processedParameters += param;
            processedParameters.Append( wxT( ' ' ) );
        }
    }

    m_pPanel = new wxPanel( this );

    CreateUpperToolBar();
    CreateLeftToolBar();

    // splitter for property grid and the windows on the right
    m_pVerticalSplitter = new wxSplitterWindow( m_pPanel, widVerSplitter, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );

    m_pSettingsPanel = new wxPanel( m_pVerticalSplitter );
    m_pSettingsPanel->SetWindowStyle( wxTAB_TRAVERSAL );
    // property grid on the left
    const int propGridSplitterPosition = wxConfigBase::Get()->Read( wxT( "/MainFrame/Settings/propgrid_SplitterPosisiton" ), 0l );
    long propertyGridStyle = wxPG_BOLD_MODIFIED | wxPG_SPLITTER_AUTO_CENTER | wxPG_DESCRIPTION | wxPG_TOOLTIPS | wxTAB_TRAVERSAL | wxPGMAN_DEFAULT_STYLE;
    if( propGridSplitterPosition == 0 )
    {
        propertyGridStyle |= wxPG_SPLITTER_AUTO_CENTER;
    }
    m_pPropGridManager = new wxPropertyGridManager( m_pSettingsPanel, widPropertyGridManager, wxDefaultPosition, wxDefaultSize, propertyGridStyle );
    m_pPropGridManager->SetDropTarget( new DnDCaptureSetting( this ) ); // this will allow to drop capture settings into the property grid
    wxPropertyGridPage* pPGDevice = m_pPropGridManager->AddPage( "Device And Bound Driver Properties" );
    GlobalDataStorage::Instance()->SetListBackgroundColour( pPGDevice->GetGrid()->GetCaptionBackgroundColour() );

    wxBoxSizer* pSettingsSizer = new wxBoxSizer( wxVERTICAL );
    try
    {
        wxArrayString userExperiences;
        userExperiences.Add( wxString( ConvertedString( Component::visibilityAsString( cvBeginner ) ) ) );
        userExperiences.Add( wxString( ConvertedString( Component::visibilityAsString( cvExpert ) ) ) );
        userExperiences.Add( wxString( ConvertedString( Component::visibilityAsString( cvGuru ) ) ) );
        //userExperiences.Add( wxString(ConvertedString(Component::visibilityAsString( cvInvisible ))) );
        // controls to adjust the level of user experience
        wxBoxSizer* pSettingsControlsSizer = new wxBoxSizer( wxHORIZONTAL );
        wxStaticText* pSTUserExperience = new wxStaticText( m_pSettingsPanel, wxID_ANY, wxT( " User Experience: " ) );
        pSTUserExperience->SetToolTip( wxT( "Allows to select the appropriate level of visibility for the user. More experienced users may select a higher level of visibility to get access to more advanced features" ) );
        pSettingsControlsSizer->Add( pSTUserExperience, wxSizerFlags().Center() );
        m_pUserExperienceCombo = new wxComboBox( m_pSettingsPanel, widUserExperienceCombo, userExperiences[0], wxDefaultPosition, wxSize( 120, wxDefaultCoord ), userExperiences, wxCB_DROPDOWN | wxCB_READONLY );
        m_pUserExperienceCombo->SetToolTip( wxT( "Allows to select the appropriate level of visibility for the user. More experienced users may select a higher level of visibility to get access to more advanced features" ) );
        pSettingsControlsSizer->Add( m_pUserExperienceCombo );
        pSettingsSizer->AddSpacer( 5 );
        pSettingsSizer->Add( pSettingsControlsSizer );
        GlobalDataStorage::Instance()->ConfigureComponentVisibilitySupport( true );
    }
    catch( const ImpactAcquireException& ) {}

    pSettingsSizer->AddSpacer( 5 );
    pSettingsSizer->Add( m_pPropGridManager, wxSizerFlags( 12 ).Expand() );
    pSettingsSizer->AddSpacer( 5 );
    m_pSettingsPanel->SetSizer( pSettingsSizer );

    // splitter between the image display window and the notebook window in the lower right corner of the window
    m_pHorizontalSplitter = new wxSplitterWindow( m_pVerticalSplitter, widHorSplitter, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );

    m_pDisplayPanel = new wxPanel( m_pHorizontalSplitter );
    m_pDisplayPanel->SetBackgroundColour( wxColour( 200, 200, 200 ) );
    CreateDisplayWindows();

    // A separate window that displays the full, scaled image with an AOI on top showing the part of the image currently visible in the main display window
    m_pMonitorImage = new MonitorDisplay( this );

    // the window for the controls in the lower right corner of the application
    m_pLowerRightWindow = new wxNotebook( m_pHorizontalSplitter, widNotebook, wxDefaultPosition, wxDefaultSize );

    // log window (page of lower right notebook)
    m_pLogWindow = new wxTextCtrl( m_pLowerRightWindow, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxBORDER_NONE | wxTE_RICH | wxTE_READONLY );
    m_pLowerRightWindow->AddPage( m_pLogWindow, wxT( "Output" ), true );

    wxScrolledWindow* pAnalysisPanel = 0;
    for( int i = 0; i < iapLAST; i++ )
    {
        pAnalysisPanel = CreateImageAnalysisPlotControls( m_pLowerRightWindow, BuildPlotWindowStartID( i ) );
        m_pLowerRightWindow->AddPage( pAnalysisPanel, m_ImageAnalysisPlots[GetAnalysisControlIndex( BuildPlotWindowStartID( i ) )].m_pPlotCanvas->GetName(), false );
    }

    // info plot window (page of lower right notebook)
    wxPanel* pPanelInfoPlot = new wxPanel( m_pLowerRightWindow );
    m_pCBEnableInfoPlot = new wxCheckBox( pPanelInfoPlot, widCBEnableInfoPlot, wxT( "Enable" ) );
    m_pCBInfoPlotDifferences = new wxCheckBox( pPanelInfoPlot, widCBInfoPlotDifferences, wxT( "Plot Differences" ) );
    m_pCBInfoPlotDifferences->SetToolTip( wxT( "When active not the actual values but the difference between 2 consecutive values will be plotted" ) );
    m_pCBInfoPlotAutoScale = new wxCheckBox( pPanelInfoPlot, widCBInfoPlotAutoScale, wxT( "Scale Automatically" ) );
    m_pCBInfoPlotAutoScale->SetToolTip( wxT( "When active the plot will be scaled from 'current min value' to 'current max value'" ) );
    m_pInfoPlotSelectionCombo = new wxComboBox( pPanelInfoPlot, widInfoPlot_PlotSelectionCombo, wxEmptyString, wxDefaultPosition, wxSize( 400, wxDefaultCoord ), 0, NULL, wxCB_DROPDOWN | wxCB_READONLY );
    m_pInfoPlotSelectionCombo->SetToolTip( wxT( "Contains a list of possible plots that can be displayed on this page" ) );
    m_pInfoPlotArea = new PlotCanvasInfo( pPanelInfoPlot, wxID_ANY, wxDefaultPosition, wxDefaultSize );

    wxFlexGridSizer* pInfoPlotControlsGridSizer = new wxFlexGridSizer( 2 );
    pInfoPlotControlsGridSizer->Add( new wxStaticText( pPanelInfoPlot, wxID_ANY, wxT( "Update Interval:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_pSCInfoPlotUpdateSpeed = new wxSpinCtrl( pPanelInfoPlot, widSCInfoPlotUpdateSpeed, wxString::Format( wxT( "%d" ), m_pInfoPlotArea->GetUpdateFrequency() ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, m_pInfoPlotArea->GetUpdateFrequencyMin(), m_pInfoPlotArea->GetUpdateFrequencyMax(), m_pInfoPlotArea->GetUpdateFrequency() );
    pInfoPlotControlsGridSizer->Add( m_pSCInfoPlotUpdateSpeed, wxSizerFlags().Expand() );
    pInfoPlotControlsGridSizer->Add( new wxStaticText( pPanelInfoPlot, wxID_ANY, wxT( "History Depth:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_pSCInfoPlotHistoryDepth = new wxSpinCtrl( pPanelInfoPlot, widSCInfoPlotHistoryDepth, wxT( "0" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 5, 10000, 20 );
    pInfoPlotControlsGridSizer->Add( m_pSCInfoPlotHistoryDepth, wxSizerFlags().Expand() );
    wxStaticText* pSTTimestampAsTime = new wxStaticText( pPanelInfoPlot, wxID_ANY, wxT( "Timestamp As Time:" ) );
    pSTTimestampAsTime->SetToolTip( wxT( "This is the device timestamp converted into local time. This value only makes sense for devices that can create a timestamp in us since 01.01.1970 GMT(thus a timestamp of 0 in Berlin will result an absolute time of 1 in the night on the first of January in 1970)" ) );
    pInfoPlotControlsGridSizer->Add( pSTTimestampAsTime, wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_pTCTimestampAsTime = new wxTextCtrl( pPanelInfoPlot, wxID_ANY, wxT( "00:00:00:000" ) );
    m_pTCTimestampAsTime->SetToolTip( wxT( "This is the device timestamp converted into local time. This value only makes sense for devices that can create a timestamp in us since 01.01.1970 GMT(thus a timestamp of 0 in Berlin will result an absolute time of 1 in the night on the first of January in 1970)" ) );
    m_pTCTimestampAsTime->Enable( false );
    pInfoPlotControlsGridSizer->Add( m_pTCTimestampAsTime, wxSizerFlags().Expand() );

    wxBoxSizer* pInfoPlotControlsSizer = new wxBoxSizer( wxVERTICAL );
    pInfoPlotControlsSizer->Add( m_pCBEnableInfoPlot, wxSizerFlags().Expand() );
    pInfoPlotControlsSizer->AddSpacer( 5 );
    pInfoPlotControlsSizer->Add( m_pCBInfoPlotDifferences, wxSizerFlags().Expand() );
    pInfoPlotControlsSizer->AddSpacer( 5 );
    pInfoPlotControlsSizer->Add( m_pCBInfoPlotAutoScale, wxSizerFlags().Expand() );
    pInfoPlotControlsSizer->AddSpacer( 5 );
    pInfoPlotControlsSizer->Add( pInfoPlotControlsGridSizer );

    wxBoxSizer* pInfoPlotFeatureSelectionSizer = new wxBoxSizer( wxHORIZONTAL );
    pInfoPlotFeatureSelectionSizer->Add( new wxStaticText( pPanelInfoPlot, wxID_ANY, wxT( "Feature To Plot: " ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    pInfoPlotFeatureSelectionSizer->Add( m_pInfoPlotSelectionCombo, wxSizerFlags( 1 ).Expand() );

    wxBoxSizer* pInfoPlotRightControlsSizer = new wxBoxSizer( wxVERTICAL );
    pInfoPlotRightControlsSizer->Add( pInfoPlotFeatureSelectionSizer );
    pInfoPlotRightControlsSizer->AddSpacer( 5 );
    pInfoPlotRightControlsSizer->Add( m_pInfoPlotArea, wxSizerFlags( 1 ).Expand() );

    wxBoxSizer* pInfoPlotPanelSizer = new wxBoxSizer( wxHORIZONTAL );
    pInfoPlotPanelSizer->Add( pInfoPlotControlsSizer, wxSizerFlags().Left() );
    pInfoPlotPanelSizer->AddSpacer( 5 );
    pInfoPlotPanelSizer->Add( pInfoPlotRightControlsSizer, wxSizerFlags( 1 ).Expand() );

    pPanelInfoPlot->SetSizer( pInfoPlotPanelSizer );
    m_pLowerRightWindow->AddPage( pPanelInfoPlot, wxT( "Info Plot" ), false );

    // feature value vs. time plot window (page of lower right notebook)
    wxPanel* pPanelFeatureValueVsTimePlot = new wxPanel( m_pLowerRightWindow );
    m_pCBEnableFeatureValueVsTimePlot = new wxCheckBox( pPanelFeatureValueVsTimePlot, widCBEnableFeatureValueVsTimePlot, wxT( "Enable" ) );
    m_pCBFeatureValueVsTimePlotDifferences = new wxCheckBox( pPanelFeatureValueVsTimePlot, widCBFeatureValueVsTimePlotDifferences, wxT( "Plot Differences" ) );
    m_pCBFeatureValueVsTimePlotDifferences->SetToolTip( wxT( "When active not the actual values but the difference between 2 consecutive values will be plotted" ) );
    m_pCBFeatureValueVsTimePlotAutoScale = new wxCheckBox( pPanelFeatureValueVsTimePlot, widCBFeatureValueVsTimePlotAutoScale, wxT( "Scale Automatically" ) );
    m_pCBFeatureValueVsTimePlotAutoScale->SetToolTip( wxT( "When active the plot will be scaled from 'current min value' to 'current max value'" ) );
    m_pFeatureValueVsTimePlotArea = new PlotCanvasFeatureVsTime( pPanelFeatureValueVsTimePlot, wxID_ANY, wxDefaultPosition, wxDefaultSize );

    wxFlexGridSizer* pFeatureValueVsTimePlotControlsGridSizer = new wxFlexGridSizer( 2 );
    pFeatureValueVsTimePlotControlsGridSizer->Add( new wxStaticText( pPanelFeatureValueVsTimePlot, wxID_ANY, wxT( "Update Interval(ms):" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_pSCFeatureValueVsTimePlotUpdateSpeed = new wxSpinCtrl( pPanelFeatureValueVsTimePlot, widSCFeatureValueVsTimePlotUpdateSpeed,
            wxString::Format( wxT( "%d" ), m_pFeatureValueVsTimePlotArea->GetUpdateFrequency() ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS,
            m_pFeatureValueVsTimePlotArea->GetUpdateFrequencyMin(),
            m_pFeatureValueVsTimePlotArea->GetUpdateFrequencyMax(),
            m_pFeatureValueVsTimePlotArea->GetUpdateFrequency() );
    pFeatureValueVsTimePlotControlsGridSizer->Add( m_pSCFeatureValueVsTimePlotUpdateSpeed, wxSizerFlags().Expand() );
    pFeatureValueVsTimePlotControlsGridSizer->Add( new wxStaticText( pPanelFeatureValueVsTimePlot, wxID_ANY, wxT( "History Depth:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_pSCFeatureValueVsTimePlotHistoryDepth = new wxSpinCtrl( pPanelFeatureValueVsTimePlot, widSCFeatureValueVsTimePlotHistoryDepth, wxT( "0" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 5, 10000, 20 );
    pFeatureValueVsTimePlotControlsGridSizer->Add( m_pSCFeatureValueVsTimePlotHistoryDepth, wxSizerFlags().Expand() );

    wxBoxSizer* pFeatureValueVsTimePlotLeftControlsSizer = new wxBoxSizer( wxVERTICAL );
    pFeatureValueVsTimePlotLeftControlsSizer->Add( m_pCBEnableFeatureValueVsTimePlot );
    pFeatureValueVsTimePlotLeftControlsSizer->AddSpacer( 5 );
    pFeatureValueVsTimePlotLeftControlsSizer->Add( m_pCBFeatureValueVsTimePlotDifferences, wxSizerFlags().Expand() );
    pFeatureValueVsTimePlotLeftControlsSizer->AddSpacer( 5 );
    pFeatureValueVsTimePlotLeftControlsSizer->Add( m_pCBFeatureValueVsTimePlotAutoScale, wxSizerFlags().Expand() );
    pFeatureValueVsTimePlotLeftControlsSizer->AddSpacer( 5 );
    pFeatureValueVsTimePlotLeftControlsSizer->Add( pFeatureValueVsTimePlotControlsGridSizer );

    wxBoxSizer* pFeatureValueVsTimePlotPanelSizer = new wxBoxSizer( wxHORIZONTAL );
    pFeatureValueVsTimePlotPanelSizer->Add( pFeatureValueVsTimePlotLeftControlsSizer, wxSizerFlags().Left() );
    pFeatureValueVsTimePlotPanelSizer->AddSpacer( 5 );
    pFeatureValueVsTimePlotPanelSizer->Add( m_pFeatureValueVsTimePlotArea, wxSizerFlags( 1 ).Expand() );

    wxBoxSizer* pFeatureValueVsTimePlotSizer = new wxBoxSizer( wxVERTICAL );
    pFeatureValueVsTimePlotSizer->AddSpacer( 5 );
    pFeatureValueVsTimePlotSizer->Add( new wxStaticText( pPanelFeatureValueVsTimePlot, wxID_ANY, wxT( "Right click on any int, int64 or double property you want to display here in the property tree and select 'Plot In Feature vs. Time Plot'" ) ) );
    pFeatureValueVsTimePlotSizer->AddSpacer( 5 );
    pFeatureValueVsTimePlotSizer->Add( pFeatureValueVsTimePlotPanelSizer, wxSizerFlags( 1 ).Expand() );

    pPanelFeatureValueVsTimePlot->SetSizer( pFeatureValueVsTimePlotSizer );
    m_pLowerRightWindow->AddPage( pPanelFeatureValueVsTimePlot, wxT( "Feature vs. Time Plot" ), false );

    // configure splitters
    m_pHorizontalSplitter->SetMinimumPaneSize( 175 );
    m_pHorizontalSplitter->SetSashGravity( 0.9 );
    m_pVerticalSplitter->SetMinimumPaneSize( 50 );

    m_pStatusBar = new wxStatusBar( m_pPanel, widStatusBar );

    // layout for GUI elements
    /*
        |---------------------|
        |menu bar             |
        |---------------------|
        |toolbar              |
        |---------------------|
        |t|prop-  | display   |
        |o|grid   |           |
        |o|       |           |
        |l|       |-----------|
        |b|       | output    |
        |a|       |           |
        |r|       |           |
        |---------------------|
        |status bar           |
        |---------------------|
    */

    wxBoxSizer* pCentralControlsSizer = new wxBoxSizer( wxHORIZONTAL );
    pCentralControlsSizer->Add( m_pLeftToolBar );
    pCentralControlsSizer->Add( m_pVerticalSplitter, wxSizerFlags( 6 ).Expand() );

    wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    pTopDownSizer->Add( pCentralControlsSizer, wxSizerFlags( 6 ).Expand() );
    pTopDownSizer->Add( m_pStatusBar, wxSizerFlags().Expand() );
    m_pPanel->SetSizer( pTopDownSizer );
    pTopDownSizer->SetSizeHints( this );

    bool boMaximized = false;
    wxRect rect = RestoreConfiguration( m_displayCountX * m_displayCountY, boMaximized );

    const wxTextAttr boldStyle( GetBoldStyle( m_pLogWindow ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "Press 'F1' for help.\n" ), boldStyle );
    WriteLogMessage( wxT( "\n" ) );
    const wxString none( wxT( "none" ) );
    WriteLogMessage( wxString::Format( wxT( "Processed command line parameters: %s\n" ), ( processedParameters.length() > 0 ) ? processedParameters.c_str() : none.c_str() ), boldStyle );
    WriteLogMessage( wxT( "\n" ) );
    if( !parserErrors.IsEmpty() )
    {
        WriteErrorMessage( parserErrors );
        WriteLogMessage( wxT( "\n" ) );
    }

    m_pDevPropHandler = new DevicePropertyHandler( pPGDevice, boDisplayDebugInfo, boDisplayFullTree, boDisplayInvisibleComponents );
    switch( m_ViewMode )
    {
    case DevicePropertyHandler::vmDeveloper:
        m_pMISettings_PropGrid_ViewMode_DevelopersView->Check();
        break;
    default:
        m_pMISettings_PropGrid_ViewMode_StandardView->Check();
        break;
    }
    UpdatePropGridViewMode();
    UpdateInfoPlotCombo();

    if( m_defaultDeviceInterfaceLayout == wxT( "GenICam" ) )
    {
        pMIAction_DefaultDeviceInterface_GenICam->Check();
    }
    else
    {
        pMIAction_DefaultDeviceInterface_DeviceSpecific->Check();
    }

    m_pMonitorImage->GetDisplayArea()->SetActive( m_pMISettings_Display_ShowMonitorImage->IsChecked() );
    m_pMonitorImage->Show( m_pMISettings_Display_ShowMonitorImage->IsChecked() && m_pCurrentAnalysisDisplay->IsActive() );
    m_DisplayAreas[0]->RegisterMonitorDisplay( m_pMISettings_Display_ShowMonitorImage->IsChecked() ? m_pMonitorImage->GetDisplayArea() : 0 );

    SetSizeHints( 720, 400 );
    if( cmdLineRect.width != -1 )
    {
        rect.width = cmdLineRect.width;
    }
    if( cmdLineRect.height != -1 )
    {
        rect.height = cmdLineRect.height;
    }
    if( cmdLineRect.x != -1 )
    {
        rect.x = cmdLineRect.x;
    }
    if( cmdLineRect.y != -1 )
    {
        rect.y = cmdLineRect.y;
    }
    SetSize( rect );
    Maximize( boMaximized );

    m_pHorizontalSplitter->SetSashPosition( ( m_HorizontalSplitterPos != -1 ) ? m_HorizontalSplitterPos : 3 * rect.height / 4, true );
    m_pVerticalSplitter->SetSashPosition( ( m_VerticalSplitterPos != -1 ) ? m_VerticalSplitterPos : rect.width / 3, true );

    if( m_VerticalSplitterPos == -1 )
    {
        m_VerticalSplitterPos = m_pVerticalSplitter->GetSashPosition();
    }

    SetupDlgControls();
    UpdateTitle();
    if( propGridSplitterPosition != 0 )
    {
        // this must be done after the dialog has been created completely as otherwise the result might not be as expected.
        // See wxWidgets documentation for details.
        m_pPropGridManager->GetGrid()->SetSplitterPosition( propGridSplitterPosition );
    }
    wxToolTip::Enable( true );
    StartPropertyGridUpdateTimer();

    if( serialToOpen.length() > 0 )
    {
        int selection = GetDesiredDeviceIndex( serialToOpen );
        if( selection != wxNOT_FOUND )
        {
            m_pDevCombo->Select( selection );
            UpdateDeviceFromComboBox();
            ToggleCurrentDevice();
            if( boOpenDeviceInLiveMode )
            {
                m_pAcquisitionModeCombo->SetValue( m_ContinuousStr );
                EnsureAcquisitionState( true );
            }
        }
        else
        {
            wxString msg( wxT( "\nCan't directly open device " ) );
            msg.Append( ConvertedString( serialToOpen ) );
            msg.Append( wxT( " as it doesn't seem to be present or the serial number is incorrect\n" ) );
            WriteErrorMessage( msg );
        }
    }
    else
    {
        SetCurrentImage( wxFileName( fileToOpen ), m_pCurrentAnalysisDisplay );
    }

    if( !GlobalDataStorage::Instance()->IsComponentVisibilitySupported() )
    {
        WriteErrorMessage( wxT( "WARNING: This version of wxPropView tries to make use of a user access level feature that currently doesn't seem to be supported by the underlying framework! Please update all your device drivers.\n" ) );
    }

    m_pPropViewCallback->attachApplication( this );
    UpdateDeviceList();
    CheckForPotentialFirewallIssues();
    StartDisplayUpdateTimer();
}

//-----------------------------------------------------------------------------
PropViewFrame::~PropViewFrame()
//-----------------------------------------------------------------------------
{
    StopDisplayUpdateTimer();
    Deinit();

    {
        // main frame
        FramePositionStorage::Save( this );
        // when we e.g. try to write config stuff on a read-only file system the result can
        // be an annoying message box. Therefore we switch off logging during the storage operation.
        wxLogNull logSuspendScope;
        // store the current state of the application
        wxConfigBase* pConfig = wxConfigBase::Get();
        pConfig->Write( wxT( "/MainFrame/verticalSplitter" ), ( m_pVerticalSplitter->IsSplit() ) ? m_pVerticalSplitter->GetSashPosition() : m_VerticalSplitterPos );
        pConfig->Write( wxT( "/MainFrame/horizontalSplitter" ), ( m_pHorizontalSplitter->IsSplit() ) ? m_pHorizontalSplitter->GetSashPosition() : m_HorizontalSplitterPos );
        pConfig->Write( wxT( "/MainFrame/selectedPage" ), m_pLowerRightWindow->GetSelection() );

        pConfig->Write( wxT( "/MainFrame/displayCountX" ), m_displayCountX );
        pConfig->Write( wxT( "/MainFrame/displayCountY" ), m_displayCountY );
        const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
        for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
        {
            ostringstream oss;
            oss << "/MainFrame/Settings/Display/" << i << "/";
            wxString displayToken( ConvertedString( oss.str() ) );
            pConfig->Write( displayToken + wxString( wxT( "FitToScreen" ) ), m_DisplayAreas[i]->IsScaled() );
            if( m_DisplayAreas[i]->SupportsDifferentScalingModes() )
            {
                pConfig->Write( displayToken + wxString( wxT( "ScalingMode" ) ), static_cast<int>( m_DisplayAreas[i]->GetScalingMode() ) );
            }
            pConfig->Write( displayToken + wxString( wxT( "ShowPerformanceWarnings" ) ), m_DisplayAreas[i]->GetPerformanceWarningOutput() );
            pConfig->Write( displayToken + wxString( wxT( "ShowImageModificationWarning" ) ), m_DisplayAreas[i]->GetImageModificationWarningOutput() );
            pConfig->Write( displayToken + wxString( wxT( "ShowRequestInfos" ) ), m_DisplayAreas[i]->InfoOverlayActive() );
        }

        pConfig->Write( wxT( "/MainFrame/Settings/findFeatureMatchCaseActive" ), m_boFindFeatureMatchCaseActive );
        pConfig->Write( wxT( "/MainFrame/Settings/displayUpdateFrequency" ), m_displayUpdateFrequency );
        pConfig->Write( wxT( "/MainFrame/Settings/monitorDisplay" ), m_pMISettings_Display_ShowMonitorImage->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/displayActive" ), m_pMISettings_Display_Active->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/displayIncompleteFrames" ), m_pMISettings_Display_ShowIncompleteFrames->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/silentRecording" ), m_pMICapture_Recording_SlientMode->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/continuousRecording" ), m_pMICapture_Recording_Continuous->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/activeDevicesOnly" ), m_pMIAction_DisplayConnectedDevicesOnly->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/displayLogWindow" ), m_pMISettings_Analysis_ShowControls->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/synchroniseAOIs" ), m_pMISettings_Analysis_SynchroniseAOIs->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/detailedInfosOnCallback" ), m_pMISettings_Analysis_DetailedInfoOnCallback->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/displayPropGrid" ), m_pMISettings_PropGrid_Show->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/showToolBar" ), m_pMISettings_ShowUpperToolBar->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/allowFastSingleFrameAcquisition" ), m_pMISettings_AllowFastSingleFrameAcquisition->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/showLeftToolBar" ), m_pMISettings_ShowLeftToolBar->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/showStatusBar" ), m_pMISettings_ShowStatusBar->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/warnOnOutdatedFirmware" ), m_pMISettings_WarnOnOutdatedFirmware->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/warnOnReducedDriverPerformance" ), m_pMISettings_WarnOnReducedDriverPerformance->IsChecked() );
#if defined(linux) || defined(__linux) || defined(__linux__)
        pConfig->Write( wxT( "/MainFrame/Settings/warnOnPotentialBufferIssues" ), m_pMISettings_WarnOnPotentialBufferIssues->IsChecked() );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
        pConfig->Write( wxT( "/MainFrame/Settings/warnOnPotentialFirewallIssues" ), m_pMISettings_WarnOnPotentialFirewallIssues->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/propgrid_showMethodExecutionErrors" ), m_pMISettings_PropGrid_ShowMethodExecutionErrors->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/propgrid_showFeatureChangeTimeConsumption" ), m_pMISettings_PropGrid_ShowFeatureChangeTimeConsumption->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/propgrid_displayToolTips" ), m_pMISettings_PropGrid_DisplayToolTips->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/propgrid_SplitterPosisiton" ), m_pPropGridManager->GetGrid()->GetSplitterPosition() );
        pConfig->Write( wxT( "/MainFrame/Settings/automaticallyReconnectToUnusedDevices" ), m_pMIAction_AutomaticallyReconnectToUnusedDevices->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/displayPropsWithHexIndices" ), m_pMISettings_PropGrid_DisplayPropIndicesAsHex->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/useDisplayNameIfAvailable" ), m_pMISettings_PropGrid_UseDisplayNameIfAvailable->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/useSelectorGrouping" ), m_pMISettings_PropGrid_UseSelectorGrouping->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/createEditorsWithSlider" ), m_pMISettings_PropGrid_CreateEditorsWithSlider->IsChecked() );
        pConfig->Write( wxT( "/MainFrame/Settings/propGridViewMode" ), int( m_ViewMode ) );
        pConfig->Write( wxT( "/MainFrame/Settings/defaultDeviceInterfaceLayoutString" ), m_defaultDeviceInterfaceLayout );
        if( m_pUserExperienceCombo )
        {
            pConfig->Write( wxT( "/MainFrame/Settings/UserExperience" ), m_pUserExperienceCombo->GetValue() );
        }
        pConfig->Write( wxT( "/MainFrame/Settings/Capture_DefaultImageProcessingMode" ), int( m_defaultImageProcessingMode ) );
        if( m_pMICapture_CaptureSettings_UsageMode_Manual->IsChecked() )
        {
            pConfig->Write( wxT( "/MainFrame/Settings/CaptureSettings_UsageMode" ), wxT( "Manual" ) );
        }
        else if( m_pMICapture_CaptureSettings_UsageMode_Automatic->IsChecked() )
        {
            pConfig->Write( wxT( "/MainFrame/Settings/CaptureSettings_UsageMode" ), wxT( "Automatic" ) );
        }

        for( unsigned int i = 0; i < iapLAST; i++ )
        {
            m_ImageAnalysisPlots[i].Save( pConfig );
        }

        // info plot
        pConfig->Write( wxT( "/Controls/InfoPlot/Enable" ), m_pCBEnableInfoPlot->GetValue() );
        pConfig->Write( wxT( "/Controls/InfoPlot/HistoryDepth" ), m_pSCInfoPlotHistoryDepth->GetValue() );
        pConfig->Write( wxT( "/Controls/InfoPlot/UpdateSpeed" ), m_pSCInfoPlotUpdateSpeed->GetValue() );
        pConfig->Write( wxT( "/Controls/InfoPlot/PlotDifferences" ), m_pCBInfoPlotDifferences->GetValue() );
        pConfig->Write( wxT( "/Controls/InfoPlot/AutoScale" ), m_pCBInfoPlotAutoScale->GetValue() );

        // feature vs. time plot
        pConfig->Write( wxT( "/Controls/FeatureValueVsTimePlot/Enable" ), m_pCBEnableFeatureValueVsTimePlot->GetValue() );
        pConfig->Write( wxT( "/Controls/FeatureValueVsTimePlot/HistoryDepth" ), m_pSCFeatureValueVsTimePlotHistoryDepth->GetValue() );
        pConfig->Write( wxT( "/Controls/FeatureValueVsTimePlot/UpdateSpeed" ), m_pSCFeatureValueVsTimePlotUpdateSpeed->GetValue() );
        pConfig->Write( wxT( "/Controls/FeatureValueVsTimePlot/PlotDifferences" ), m_pCBFeatureValueVsTimePlotDifferences->GetValue() );
        pConfig->Write( wxT( "/Controls/FeatureValueVsTimePlot/AutoScale" ), m_pCBFeatureValueVsTimePlotAutoScale->GetValue() );

        pConfig->Flush();

        // wizards
        pConfig->Write( wxT( "/Wizards/QuickSetup/Show" ), m_pMISettings_ShowQuickSetupOnUse->IsChecked() );
    }

    delete GlobalDataStorage::Instance();
    delete m_pPropViewCallback;
    m_pMonitorImage->Destroy();
    DestroyDialog( &m_pQuickSetupDlgCurrent );
    DestroyAdditionalDialogs();
    // when we e.g. try to write config stuff on a read-only file system the result can
    // be an annoying message box. Therefore we switch off logging now, as otherwise higher level
    // clean up code might produce error messages
    wxLog::EnableLogging( false );
}

//-----------------------------------------------------------------------------
void PropViewFrame::Abort( void )
//-----------------------------------------------------------------------------
{
    ReEnableSingleCapture();
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
    if( pThread )
    {
        int result = pThread->RequestReset();
        if( result != DMR_NO_ERROR )
        {
            WriteErrorMessage( wxString::Format( wxT( "FunctionInterface::imageRequestReset( 0, 0 ) returned an error: %s.\n" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::Acquire( void )
//-----------------------------------------------------------------------------
{
    bool boAcquisitionModeDiffers = false;
    Property acquisitionMode( m_pDevPropHandler->GetActiveDeviceAcquisitionMode() );
    try
    {
        if( m_lastAcquisitionMode != m_pAcquisitionModeCombo->GetValue() )
        {
            boAcquisitionModeDiffers = true;
            m_lastAcquisitionMode = m_pAcquisitionModeCombo->GetValue();
        }
        const string selectedAcquisitionMode( m_pAcquisitionModeCombo->GetValue().mb_str() );
        if( acquisitionMode.isValid() && acquisitionMode.isWriteable() && ( acquisitionMode.readS() != selectedAcquisitionMode ) )
        {
            acquisitionMode.writeS( selectedAcquisitionMode );
        }
    }
    catch( const EInvalidValue& )
    {
        if( boAcquisitionModeDiffers )
        {
            WriteErrorMessage( wxString::Format( wxT( "Acquisition mode '%s' is not supported by device %s!\n" ), m_pAcquisitionModeCombo->GetValue().c_str(), m_pDevCombo->GetValue().c_str() ) );
        }
        return;
    }
    catch( const ImpactAcquireException& e )
    {
        wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Internal problem during acquisition mode selection for device %s: %s" ), m_pDevCombo->GetValue().c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        return;
    }

    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
    if( pThread )
    {
        if( m_pAcquisitionModeCombo->GetValue() == m_ContinuousStr )
        {
            pThread->SetMultiFrameSequenceSize( 0 );
            ConfigureLive();
        }
        else if( m_pAcquisitionModeCombo->GetValue() == m_MultiFrameStr )
        {
            PropertyI64 acquisitionFrameCount( m_pDevPropHandler->GetActiveDeviceAcquisitionFrameCount() );
            const size_t frameCount( static_cast<size_t>( acquisitionFrameCount.isValid() ? acquisitionFrameCount.read() : 0 ) );
            if( ( frameCount == 0 ) && !pThread->GetLiveMode() )
            {
                WriteErrorMessage( wxString::Format( wxT( "Device %s does not support 'AcquisitionFrameCount' property while supporting '%s' acquisition. Strange behaviour is possible now.\n" ), m_pDevCombo->GetValue().c_str(), m_MultiFrameStr.c_str() ) );
            }
            pThread->SetMultiFrameSequenceSize( frameCount );
            ConfigureLive();
        }
        else if( m_pAcquisitionModeCombo->GetValue() == m_SingleFrameStr )
        {
            SetupUpdateFrequencies( false );
            m_boSingleCaptureInProgess = true;
            pThread->SetMultiFrameSequenceSize( 0 );
            int result = pThread->RequestSingle( !m_pMICapture_Recording_SlientMode->IsChecked() );
            if( result != DMR_NO_ERROR )
            {
                m_boSingleCaptureInProgess = false;
                WriteErrorMessage( wxString::Format( wxT( "Image request failed(Reason: %s)! More information can be found in the *.log-file or the debug output.\n" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
            }
            SetupDlgControls();
        }
        else
        {
            wxMessageDialog errorDlg( NULL, wxT( "Internal problem: Invalid/Unsupported acquisition mode selected." ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
            errorDlg.ShowModal();
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::AddListControlToAboutNotebook( wxNotebook* pNotebook, const wxString& pageTitle, bool boSelectPage, const wxString& col0, const wxString& col1, const vector<pair<wxString, wxString> >& v )
//-----------------------------------------------------------------------------
{
    wxListCtrl* pListCtrl = new wxListCtrl( pNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT | wxLC_SINGLE_SEL | wxBORDER_NONE );
    pListCtrl->InsertColumn( 0, col0 );
    pListCtrl->InsertColumn( 1, col1 );
    const unsigned long cnt = static_cast<unsigned long>( v.size() );
    for( unsigned long i = 0; i < cnt; i++ )
    {
        long index = pListCtrl->InsertItem( i, v[i].first, i );
        pListCtrl->SetItem( index, 1, v[i].second );
    }
    for( unsigned int i = 0; i < 2; i++ )
    {
        pListCtrl->SetColumnWidth( i, wxLIST_AUTOSIZE );
    }
    pNotebook->AddPage( pListCtrl, pageTitle, boSelectPage );
}

//-----------------------------------------------------------------------------
void PropViewFrame::AnalysisPlotUpdateSpeedChanged( void )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    if( m_pDevPropHandler )
    {
        m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
        if( pThread )
        {
            if( pThread->GetLiveMode() )
            {
                SetupUpdateFrequencies( true );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::AppendCustomPropGridExecutionErrorMessage( wxString& msg ) const
//-----------------------------------------------------------------------------
{
    msg.Append( wxT( "\n\nTo get rid of these message boxes disable the 'Show Method Execution Errors' option\nunder 'Settings -> Property Grid'." ) );
}

//-----------------------------------------------------------------------------
void PropViewFrame::AssignDefaultSettingToDisplayRelationship( void )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev )
    {
        try
        {
            ImageRequestControl irc( pDev );
            vector<pair<string, int> > settings;
            irc.setting.getTranslationDict( settings );
            wxCriticalSectionLocker locker( m_critSect );
            m_settingToDisplayDict.clear();
            const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
            const vector<pair<string, int> >::size_type settingCount = settings.size();
            for( vector<pair<string, int> >::size_type i = 0; i < settingCount; i++ )
            {
                m_settingToDisplayDict.insert( make_pair( settings[i].second, i % displayCount ) );
            }
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to create setting hierarchy: %s(%s)\n" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
mvIMPACT::acquire::TBayerMosaicParity PropViewFrame::BayerParityFromString( const wxString& value )
//-----------------------------------------------------------------------------
{
    if( value == wxT( "Red-green" ) )
    {
        return bmpRG;
    }
    else if( value == wxT( "Green-red" ) )
    {
        return bmpGR;
    }
    else if( value == wxT( "Blue-green" ) )
    {
        return bmpBG;
    }
    else if( value == wxT( "Green-blue" ) )
    {
        return bmpGB;
    }
    else if( value == wxT( "Undefined" ) )
    {
        return bmpUndefined;
    }

    WriteErrorMessage( wxString::Format( wxT( "Unsupported bayer parity(%s).\n" ), value.c_str() ) );
    return bmpUndefined;
}

//-----------------------------------------------------------------------------
template<typename _Ty, typename _Tx>
size_t PropViewFrame::BuildStringArrayFromPropertyDict( wxArrayString& choices, _Tx prop ) const
//-----------------------------------------------------------------------------
{
    choices.Clear();
    if( prop.isValid() && prop.hasDict() )
    {
        vector<pair<string, _Ty> > dict;
        prop.getTranslationDict( dict );
        typename vector<pair<string, _Ty> >::size_type dictSize = dict.size();
        for( typename vector<pair<string, _Ty> >::size_type i = 0; i < dictSize; i++ )
        {
            choices.Add( ConvertedString( dict[i].first ) );
        }
    }
    return choices.Count();
}

//-----------------------------------------------------------------------------
void PropViewFrame::CheckForDriverPerformanceIssues( Device* pDev )
//-----------------------------------------------------------------------------
{
    if( m_pMISettings_WarnOnReducedDriverPerformance->IsChecked() )
    {
        wxPlatformInfo platformInfo;
        if( ( platformInfo.GetOperatingSystemId() & wxOS_WINDOWS ) != 0 )
        {
            wxString msg;
            const wxString product( ConvertedString( pDev->product.read() ) );
            if( product.Contains( wxT( "mvBlueFOX" ) ) )
            {
                ComponentLocator locator( pDev->hDev() );
                PropertyI kernelDriver;
                locator.bindComponent( kernelDriver, "KernelDriver" );
                if( kernelDriver.isValid() &&
                    ( static_cast<TBlueFOXKernelDriver>( kernelDriver.read() ) == bfkdmvBlueFOX ) )
                {
                    msg = wxString::Format( wxT( "Device '%s' is bound to an outdated kernel driver. It's highly recommended to update to the latest driver version which offers a much better performance. This update can be performed using 'mvDeviceConfigure'." ), ConvertedString( pDev->serial.read() ).c_str() );
                }
            }
            else
            {
                ComponentLocator locator( pDev->hDrv() );
                PropertyS mvStreamDriverTechnology;
                locator.bindComponent( mvStreamDriverTechnology, "mvStreamDriverTechnology" );
                if( mvStreamDriverTechnology.isValid() )
                {
                    const wxString streamTechnology( ConvertedString( mvStreamDriverTechnology.read() ) );
                    if( streamTechnology.Contains( wxT( "Socket" ) ) )
                    {
                        msg = wxString::Format( wxT( "Device '%s' runs with the socket API based GEV capture driver. It's highly recommended to work with NDIS based filter driver instead which offers a much better performance. The filter driver can be installed and/or activated using 'mvGigEConfigure'." ), ConvertedString( pDev->serial.read() ).c_str() );
                    }
                }
            }

            if( !msg.IsEmpty() )
            {
                WriteErrorMessage( wxString::Format( wxT( "WARNING while opening device '%s': %s\n" ), ConvertedString( pDev->serial.read() ).c_str(), msg.c_str() ) );
                switch( wxMessageBox( wxString::Format( wxT( "Potential problem detected:\n\n%s\n\nPress 'Yes' to continue anyway.\n\nPress 'No' to end this application and resolve the issue.\n\nPress 'Cancel' to continue anyway and never see this message again. You can later re-enable this message box under 'Settings -> Warn On Reduced Driver Performance'." ), msg.c_str() ), wxT( "Potential Performance Issue Detected" ), wxYES_NO | wxCANCEL | wxICON_EXCLAMATION, this ) )
                {
                case wxNO:
                    Close( true );
                    break;
                case wxCANCEL:
                    m_pMISettings_WarnOnReducedDriverPerformance->Check( false );
                    break;
                }
            }
        }
    }
}

#if defined(linux) || defined(__linux) || defined(__linux__)
//-----------------------------------------------------------------------------
int PropViewFrame::CheckForPotentialBufferIssuesGetValueHelper( const wxString& path )
//-----------------------------------------------------------------------------
{
    int systemValue = -1;
    wxTextFile tfile;
    if( tfile.Open( path ) )
    {
        const wxString valueString = tfile.GetFirstLine();
        tfile.Close();
        systemValue = wxAtoi( valueString );
    }
    return systemValue;
}

//-----------------------------------------------------------------------------
void PropViewFrame::CheckForPotentialBufferIssues( Device* pDev )
//-----------------------------------------------------------------------------
{
    //buffer checks make only sense with GenICam devices
    if( pDev->interfaceLayout.read() != dilGenICam )
    {
        return;
    }
    const int desiredNetworkBufSize = 12582912;
    const int desiredNetworkQueueLen = 5000;
    const int desiredUsbBufSizeMB = 256 ;
    wxString buffersInfo = wxT( "\n" );
    bool hasRXNetworkBufferIssue = false;
    bool hasTXNetworkBufferIssue = false;
    bool hasQueueLengthIssue = false;
    bool usbBufferIssue = false;
    static const wxString rxBuff = wxT( "/proc/sys/net/core/rmem_max" );
    static const wxString txBuff = wxT( "/proc/sys/net/core/wmem_max" );
    static const wxString queueLen = wxT( "/proc/sys/net/core/netdev_max_backlog" );
    static const wxString usbcore = wxT( "/sys/module/usbcore/parameters/usbfs_memory_mb" );
    int rxBuffSize, txBuffSize, queueLenSize, usbcoreBuffSize;

    GenICam::DeviceModule devMod( pDev );
    if( devMod.deviceType.readS() == "GEV" )
    {
        if( ( rxBuffSize = CheckForPotentialBufferIssuesGetValueHelper( rxBuff ) ) == -1 )
        {
            WriteLogMessage( wxString::Format( wxT( "Error reading network receive buffer size.\n" ) ) );
            return;
        }
        else
        {
            hasRXNetworkBufferIssue = ( rxBuffSize < desiredNetworkBufSize ) ? true : false;
            buffersInfo += wxString::Format( wxT( "   Network Receive Buffer: \t%.2fMB  \t%s \n" ), ( static_cast<double>( rxBuffSize ) / static_cast<double>( 1048576 ) ), ( rxBuffSize < desiredNetworkBufSize ) ? wxT( "LOW!" ) : wxT( "OK!" ) );
        }
        if( ( txBuffSize = CheckForPotentialBufferIssuesGetValueHelper( txBuff ) ) == -1 )
        {
            WriteLogMessage( wxString::Format( wxT( "Error reading network send buffer size.\n" ) ) );
            return;
        }
        else
        {
            hasTXNetworkBufferIssue = ( txBuffSize < desiredNetworkBufSize ) ? true : false;
            buffersInfo += wxString::Format( wxT( "   Network Transmit Buffer: \t%.2fMB  \t%s \n" ), ( static_cast<double>( txBuffSize ) / static_cast<double>( 1048576 ) ), ( txBuffSize < desiredNetworkBufSize ) ? wxT( "LOW!" ) : wxT( "OK!" ) );
        }
        if( ( queueLenSize = CheckForPotentialBufferIssuesGetValueHelper( queueLen ) ) == -1 )
        {
            WriteLogMessage( wxString::Format( wxT( "Error reading network queue length.\n" ) ) );
            return;
        }
        else
        {
            hasQueueLengthIssue = ( queueLenSize < desiredNetworkQueueLen ) ? true : false;
            buffersInfo += wxString::Format( wxT( "   Network Queue Length: \t%d    \t%s \n" ), queueLenSize, ( queueLenSize < desiredNetworkQueueLen ) ? wxT( "LOW!" ) : wxT( "OK!" ) );
        }
    }
    else if( devMod.deviceType.readS() == "U3V" )
    {
        if( ( usbcoreBuffSize = CheckForPotentialBufferIssuesGetValueHelper( usbcore ) ) == -1 )
        {
            WriteLogMessage( wxString::Format( wxT( "Error reading usbcore buffer size.\n" ) ) );
            return;
        }
        else
        {
            usbBufferIssue = ( usbcoreBuffSize < desiredUsbBufSizeMB ) ? true : false;
            buffersInfo += wxString::Format( wxT( "   Usbcore Usbfs Buffer:     \t%dMB\t\t%s \n" ), usbcoreBuffSize, ( usbcoreBuffSize < desiredUsbBufSizeMB  ) ? wxT( "LOW!" ) : wxT( "OK!" ) );
        }
    }
    if( m_pMISettings_WarnOnPotentialBufferIssues->IsChecked() &&
        ( GetGenTLDeviceCount() != 0 ) &&
        ( hasRXNetworkBufferIssue || hasTXNetworkBufferIssue || hasQueueLengthIssue || usbBufferIssue ) )
    {
        switch( wxMessageBox( wxT( "Potential problem detected:\n\nEven though the GenICam/GenTL capture driver is installed, network receive buffers and/or usbcore(usbfs) buffers are not configured accordingly. Small buffer size can lead to incomplete frames during acquisition, or other serious problems. Please configure your system's buffer settings as described in the documentation of your MATRIX VISION camera (Quickstart section).\n" ) + buffersInfo + wxT( "\nPress 'Yes' to continue anyway.\n\nPress 'No' to end this application and resolve the issue.\n\nPress 'Cancel' to continue anyway and never see this message again. You can later re-enable this message box under 'Settings -> Warn On Potential Network/USB Buffer Issues'." ), wxT( "Low GEV/U3V Buffer Settings Detected" ), wxYES_NO | wxCANCEL | wxICON_EXCLAMATION, this ) )
        {
        case wxNO:
            Close( true );
            break;
        case wxCANCEL:
            m_pMISettings_WarnOnPotentialBufferIssues->Check( false );
            break;
        }
    }
}
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

//-----------------------------------------------------------------------------
void PropViewFrame::CheckForPotentialFirewallIssues( void )
//-----------------------------------------------------------------------------
{
    if( m_pMISettings_WarnOnPotentialFirewallIssues->IsChecked() &&
        IsGenTLDriverInstalled() &&
        ( GetGenTLDeviceCount() == 0 ) )
    {
        switch( wxMessageBox( wxT( "Potential problem detected:\n\nEven though the GenICam/GenTL capture driver is installed no compatible device could be detected. If you where expecting to see one or more GEV(GigE Vision) compliant device(s) please check your systems firewall and IP configuration as well as the network configuration on the devices you want to access (run 'mvIPConfigure' to do this).\n\nPress 'Yes' to continue anyway.\n\nPress 'No' to end this application and resolve the issue.\n\nPress 'Cancel' to continue anyway and never see this message again. You can later re-enable this message box under 'Settings -> Warn On Potential Firewall Issues'." ), wxT( "No GenICam/GenTL Devices Detected" ), wxYES_NO | wxCANCEL | wxICON_EXCLAMATION, this ) )
        {
        case wxNO:
            Close( true );
            break;
        case wxCANCEL:
            m_pMISettings_WarnOnPotentialFirewallIssues->Check( false );
            break;
        }
    }
}

//-----------------------------------------------------------------------------
int PropViewFrame::CheckIfDeviceIsReachable( Device* pDev, bool& boRunningIPConfigureMightHelp )
//-----------------------------------------------------------------------------
{
    boRunningIPConfigureMightHelp = false;
    wxString msg;
    if( pDev->state.read() == dsUnreachable )
    {
        msg.Append( wxString::Format( wxT( "WARNING: Device '%s(%s)' is currently reported as 'unreachable'. This indicates a compliant device has been detected but for some reasons cannot be used at the moment. For GEV devices this might be a result of an incorrect network configuration (run 'mvIPConfigure' with 'Advanced Device Discovery' enabled to get more information on this). For U3V devices this might be a result of the device being bound to a different (third party) U3V device driver. Refer to the documentation in section 'Troubleshooting' for additional information regarding this issue.\n\n" ),
                                      ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str() ) );
    }

    if( supportsValue( pDev->interfaceLayout, dilGenICam ) )
    {
        try
        {
            // find the GenTL interface this device claims to be connected to
            PropertyI64 interfaceID;
            PropertyI64 deviceMACAddress;
            DeviceComponentLocator locator( pDev->hDev() );
            locator.bindComponent( interfaceID, "InterfaceID" );
            locator.bindComponent( deviceMACAddress, "DeviceMACAddress" );
            if( interfaceID.isValid() && deviceMACAddress.isValid() )
            {
                const string interfaceDeviceHasBeenFoundOn( interfaceID.readS() );
                const int64_type devMACAddress( deviceMACAddress.read() );
                // always run this code as new interfaces might appear at runtime e.g. when plugging in a network cable...
                const int64_type interfaceCount( m_pDevPropHandler->GetGenTLInterfaceCount() );
                for( int64_type interfaceIndex = 0; interfaceIndex < interfaceCount; interfaceIndex++ )
                {
                    mvIMPACT::acquire::GenICam::InterfaceModule im( interfaceIndex );
                    if( interfaceID.readS() != im.interfaceID.readS() )
                    {
                        continue;
                    }
                    if( im.interfaceType.readS() != "GEV" )
                    {
                        continue;
                    }

                    boRunningIPConfigureMightHelp = true;
                    // this is the interface we are looking for
                    const int64_type deviceCount( im.deviceSelector.getMaxValue() + 1 );
                    const int64_type interfaceSubnetCount( im.gevInterfaceSubnetSelector.getMaxValue() + 1 );
                    for( int64_type deviceIndex = 0; deviceIndex < deviceCount; deviceIndex++ )
                    {
                        im.deviceSelector.write( deviceIndex );
                        if( im.gevDeviceMACAddress.read() != devMACAddress )
                        {
                            continue;
                        }

                        // this is the device we are looking for
                        bool boDeviceCorrectlyConfigured = false;
                        const int64_type netMask1 = im.gevDeviceSubnetMask.read();
                        const int64_type net1 = im.gevDeviceIPAddress.read();
                        for( int64_type interfaceSubnetIndex = 0; interfaceSubnetIndex < interfaceSubnetCount; interfaceSubnetIndex++ )
                        {
                            im.gevInterfaceSubnetSelector.write( interfaceSubnetIndex );
                            const int64_type netMask2 = im.gevInterfaceSubnetMask.read();
                            const int64_type net2 = im.gevInterfaceSubnetIPAddress.read();
                            if( ( netMask1 == netMask2 ) &&
                                ( ( net1 & netMask1 ) == ( net2 & netMask2 ) ) )
                            {
                                boDeviceCorrectlyConfigured = true;
                                break;
                            }
                        }
                        if( !boDeviceCorrectlyConfigured )
                        {
                            msg.Append( wxString::Format( wxT( "WARNING: GigE Vision device '%s' will currently not work properly as its network configuration is invalid in the current setup. Run 'mvIPConfigure' with 'Advanced Device Discovery' enabled to get more information on this).\n\n" ),
                                                          ConvertedString( im.deviceID.readS() ).c_str() ) );
                        }
                        break;
                    }
                    break;
                }
            }
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "Internal error. Failed to configure GEV interfaces for 'advanced device discovery'. %s(numerical error representation: %d (%s))." ), ConvertedString( e.getErrorString() ).c_str(), e.getErrorCode(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
    }

    if( msg.IsEmpty() )
    {
        return wxOK;
    }

    // The current device is in a bad state report it to the user
    WriteErrorMessage( msg );
    return wxMessageBox( wxString::Format( wxT( "Potential problem detected:\n\n%sPress 'Yes' to continue anyway.\n\nPress 'No' to end this application and resolve the issue.\n\nPress 'Cancel' to stop attempting to open the device." ), msg.c_str() ), wxT( "Unreachable Device Detected" ), wxYES_NO | wxCANCEL | wxCANCEL_DEFAULT | wxICON_EXCLAMATION, this );
}

//-----------------------------------------------------------------------------
void PropViewFrame::ClearDisplayInProgressStates( void )
//-----------------------------------------------------------------------------
{
    const RequestInfoContainer::size_type cnt = m_CurrentRequestDataContainer.size();
    for( RequestInfoContainer::size_type i = 0; i < cnt; i++ )
    {
        m_DisplayAreas[i]->ResetRequestInProgressFlag();
        if( static_cast<int>( i ) == m_CurrentRequestDataIndex )
        {
            m_pMonitorImage->GetDisplayArea()->ResetRequestInProgressFlag();
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::CloneAllRequests( bool boClone /* = true */ )
//-----------------------------------------------------------------------------
{
    const RequestInfoContainer::size_type cnt = m_CurrentRequestDataContainer.size();
    for( RequestInfoContainer::size_type i = 0; i < cnt; i++ )
    {
        if( ( m_CurrentRequestDataContainer[i].image_.getBuffer()->vpData ) &&
            ( m_CurrentRequestDataContainer[i].requestNr_ != INVALID_ID ) )
        {
            m_CurrentRequestDataContainer[i].image_ = boClone ? m_CurrentRequestDataContainer[i].image_.clone() : mvIMPACT::acquire::ImageBufferDesc( 1 );
            m_DisplayAreas[i]->SetImage( m_CurrentRequestDataContainer[i].image_.getBuffer(), !boClone );
            if( static_cast<int>( i ) == m_CurrentRequestDataIndex )
            {
                m_pMonitorImage->GetDisplayArea()->SetImage( m_CurrentRequestDataContainer[i].image_.getBuffer(), !boClone );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::CloneAndUnlockAllRequestsAndFreeRecordedSequence( bool boClone /* = true */ )
//-----------------------------------------------------------------------------
{
    CloneAllRequests( boClone );
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
    if( pThread )
    {
        const RequestInfoContainer::size_type cnt = m_CurrentRequestDataContainer.size();
        for( RequestInfoContainer::size_type i = 0; i < cnt; i++ )
        {
            pThread->UnlockRequest( m_CurrentRequestDataContainer[i].requestNr_, true );
            m_CurrentRequestDataContainer[i].requestNr_ = INVALID_ID;
        }
        pThread->FreeSequence();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureAnalysisPlot( const wxRect* pAOI /* = 0 */ )
//-----------------------------------------------------------------------------
{
    if( m_CurrentImageAnalysisControlIndex >= 0 )
    {
        if( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->GetImageCanvas() != m_pCurrentAnalysisDisplay )
        {
            m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->RegisterImageCanvas( m_pCurrentAnalysisDisplay );
        }
        m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->SetActive( true );
        m_pCurrentAnalysisDisplay->SetActiveAnalysisPlot( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas );
        ConfigureAOIControlLimits( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex] );
        if( pAOI )
        {
            ConfigureAOI( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex], *pAOI );
        }

        m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex],
                m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIx->GetValue(),
                m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIy->GetValue(),
                m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIw->GetValue(),
                m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIh->GetValue(),
                true );
    }
    else
    {
        m_pCurrentAnalysisDisplay->SetActiveAnalysisPlot( 0 );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureAOI( ImageAnalysisPlotControls& controls, const wxRect& rect )
//-----------------------------------------------------------------------------
{
    m_boUpdateAOIInProgress = true;
    controls.m_pSCAOIx->SetValue( rect.GetLeft() );
    controls.m_pSCAOIy->SetValue( rect.GetTop() );
    controls.m_pSCAOIw->SetValue( rect.GetWidth() );
    controls.m_pSCAOIh->SetValue( rect.GetHeight() );
    m_boUpdateAOIInProgress = false;
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureAOIControlLimits( ImageAnalysisPlotControls& controls )
//-----------------------------------------------------------------------------
{
    const ImageBuffer* p = m_pCurrentAnalysisDisplay->GetImage();
    if( p && p->vpData )
    {
        m_boUpdateAOIInProgress = true;
        if( ( p->iHeight > 0 ) && ( p->iHeight != controls.m_pSCAOIh->GetMax() ) )
        {
            controls.m_pSCAOIh->SetRange( 1, p->iHeight );
            controls.m_pSCAOIy->SetRange( 0, p->iHeight - 1 );
            // otherwise the limits are correct, but the displayed value might be incorrect
            controls.m_pSCAOIh->SetValue( controls.m_pSCAOIh->GetValue() );
            controls.m_pSCAOIy->SetValue( controls.m_pSCAOIy->GetValue() );
        }
        if( ( p->iWidth > 0 ) && ( p->iWidth != controls.m_pSCAOIw->GetMax() ) )
        {
            controls.m_pSCAOIw->SetRange( 1, p->iWidth );
            controls.m_pSCAOIx->SetRange( 0, p->iWidth - 1 );
            // otherwise the limits are correct, but the displayed value might be incorrect
            controls.m_pSCAOIw->SetValue( controls.m_pSCAOIw->GetValue() );
            controls.m_pSCAOIx->SetValue( controls.m_pSCAOIx->GetValue() );
        }
        m_boUpdateAOIInProgress = false;
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureLive( void )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        if( !pThread->GetLiveMode() )
        {
            CloneAllRequests();
            const RequestInfoContainer::size_type cnt = m_CurrentRequestDataContainer.size();
            for( RequestInfoContainer::size_type i = 0; i < cnt; i++ )
            {
                if( ( m_CurrentRequestDataContainer[i].image_.getBuffer()->vpData ) &&
                    ( m_CurrentRequestDataContainer[i].requestNr_ != INVALID_ID ) )
                {
                    pThread->UnlockRequest( m_CurrentRequestDataContainer[i].requestNr_, !m_pMICapture_Record->IsChecked() );
                }
                m_CurrentRequestDataContainer[i].requestNr_ = INVALID_ID;
            }
            if( !m_pMICapture_Record->IsChecked() )
            {
                pThread->FreeSequence();
            }
            SetupUpdateFrequencies( true );
            pThread->SetContinuousRecording( m_pMICapture_Recording_Continuous->IsChecked() );
            pThread->SetLiveMode( true, !m_pMICapture_Recording_SlientMode->IsChecked() );
        }
        else
        {
            pThread->SetLiveMode( false );
            // this increases the chance that the last image will actually be displayed
            ClearDisplayInProgressStates();
            const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
            for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
            {
                m_DisplayAreas[i]->ResetSkippedImagesCounter();
            }
        }
        SetupDlgControls();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureMonitorImage( bool boVisible )
//-----------------------------------------------------------------------------
{
    m_pMISettings_Display_ShowMonitorImage->Check( boVisible );
    m_pLeftToolBar->ToggleTool( miSettings_Display_ShowMonitorImage, boVisible );
    m_pCurrentAnalysisDisplay->RegisterMonitorDisplay( boVisible ? m_pMonitorImage->GetDisplayArea() : 0 );
    m_pMonitorImage->GetDisplayArea()->SetImage( boVisible ? m_pCurrentAnalysisDisplay->GetImage() : 0 );
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureRequestCount( void )
//-----------------------------------------------------------------------------
{
    if( m_pDevPropHandler )
    {
        Device* pDev = m_pDevPropHandler->GetActiveDevice();
        if( pDev )
        {
            SystemSettings ss( pDev );
            const int minRequestCountNeeded = 3 * GetDisplayCount();
            const int currentRequestCount = ss.requestCount.read();
            try
            {
                // When using multiple displays, it is wise to increase the request count for the device.
                if( currentRequestCount < minRequestCountNeeded )
                {
                    ss.requestCount.write( minRequestCountNeeded );
                }
            }
            catch( ImpactAcquireException& e )
            {
                WriteErrorMessage( wxString::Format( wxT( "%s(%d): Failed to increase request count from %d to %d(%s(%s))!\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__,
                                                     currentRequestCount, minRequestCountNeeded, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureSplitter( wxSplitterWindow* pSplitter, wxWindow* pWindowToRemove, wxWindow* pWindowToShow, bool boHorizontally /* = true */ )
//-----------------------------------------------------------------------------
{
    if( pSplitter )
    {
        if( pSplitter->IsSplit() )
        {
            pSplitter->Unsplit( pWindowToRemove );
        }
        else if( pSplitter->GetWindow1() != pWindowToShow )
        {
            if( pSplitter->GetWindow1() == pWindowToRemove )
            {
                // switch current window. Therefore we first need to split the window again and then
                // the 'old' window can be removed
                if( boHorizontally )
                {
                    pSplitter->SplitHorizontally( pWindowToShow, pWindowToRemove, 0 );
                }
                else
                {
                    pSplitter->SplitVertically( pWindowToShow, pWindowToRemove, 0 );
                }
                pSplitter->Unsplit( pWindowToRemove );
            }
            else
            {
                pSplitter->Initialize( pWindowToShow );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::ConfigureStatusBar( bool boShow )
//-----------------------------------------------------------------------------
{
    if( boShow )
    {
        m_pPanel->GetSizer()->Show( m_pStatusBar );
        UpdateStatusBar();
    }
    else
    {
        m_pPanel->GetSizer()->Hide( m_pStatusBar );
    }
    m_pPanel->GetSizer()->Layout();
}

//-----------------------------------------------------------------------------
void PropViewFrame::CreateDisplayWindows( void )
//-----------------------------------------------------------------------------
{
    const unsigned int currentDisplayCount = static_cast<unsigned int>( GetDisplayCount() );
    const unsigned int newDisplayCount = m_displayCountX * m_displayCountY;

    if( m_pDisplayGridSizer )
    {
        if( newDisplayCount != currentDisplayCount )
        {
            CloneAndUnlockAllRequestsAndFreeRecordedSequence();
        }
        m_pDisplayGridSizer->Clear();
    }

    if( newDisplayCount < currentDisplayCount )
    {
        // delete obsolete display windows (the new count is smaller than the current count)
        while( GetDisplayCount() > newDisplayCount )
        {
            ImageCanvas* pDisplayArea = m_DisplayAreas.back();
            if( m_pLastMouseHooverDisplay == pDisplayArea )
            {
                m_pLastMouseHooverDisplay = 0;
            }
            if( m_pCurrentAnalysisDisplay == pDisplayArea )
            {
                m_pCurrentAnalysisDisplay = 0;
            }
            pDisplayArea->Destroy();
            m_DisplayAreas.pop_back();
            m_CurrentRequestDataContainer.pop_back();
        }
        if( m_CurrentRequestDataIndex > static_cast<int>( m_CurrentRequestDataContainer.size() ) )
        {
            m_CurrentRequestDataIndex = 0;
        }
    }
    else if( newDisplayCount > currentDisplayCount )
    {
        // create missing display windows (the new count is larger than the current count)
        while( GetDisplayCount() < newDisplayCount )
        {
            const DisplayWindowContainer::size_type index = GetDisplayCount();
            ImageCanvas* pDisplayArea = new ImageCanvas( this, m_pDisplayPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE | wxHSCROLL | wxVSCROLL, wxEmptyString );
            pDisplayArea->SetUserData( index );
            pDisplayArea->EnableScrolling( true, true );
            pDisplayArea->SetDropTarget( new DnDImage( this, pDisplayArea ) ); // this will allow to drop image files into the display area
            m_DisplayAreas.push_back( pDisplayArea );
            m_CurrentRequestDataContainer.push_back( RequestData() );
            m_DisplayAreas[index]->SetImage( m_CurrentRequestDataContainer[index].image_.getBuffer() );
        }
        ConfigureRequestCount();
    }

    // create the grid sizer or update the existing one to reflect the new window configuration
    if( m_pDisplayGridSizer )
    {
        m_pDisplayGridSizer->SetCols( m_displayCountX );
        m_pDisplayGridSizer->SetRows( m_displayCountY );
    }
    else
    {
        m_pDisplayGridSizer = new wxGridSizer( m_displayCountY, m_displayCountX, 3, 3 );
        m_pDisplayPanel->SetSizer( m_pDisplayGridSizer );
    }
    // attach all display windows to the grid sizer
    for( unsigned int i = 0; i < newDisplayCount; i++ )
    {
        m_pDisplayGridSizer->Add( m_DisplayAreas[i], wxSizerFlags().Expand() );
    }
    // force a redraw of the sizer and all its children
    m_pDisplayPanel->Layout();

    if( !m_pLastMouseHooverDisplay )
    {
        m_pLastMouseHooverDisplay = m_DisplayAreas[0];
    }

    if( !m_pCurrentAnalysisDisplay )
    {
        m_pCurrentAnalysisDisplay = m_DisplayAreas[0];
    }

    if( newDisplayCount != currentDisplayCount )
    {
        RefreshPendingImageQueueDepthForCurrentDevice();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetDisplayWindowCount( unsigned int xCount, unsigned int yCount )
//-----------------------------------------------------------------------------
{
    if( xCount > DISPLAY_COUNT_MAX )
    {
        xCount = DISPLAY_COUNT_MAX;
    }
    if( yCount > DISPLAY_COUNT_MAX )
    {
        yCount = DISPLAY_COUNT_MAX;
    }
    m_displayCountX = xCount;
    m_displayCountY = yCount;
    CreateDisplayWindows();
}

//-----------------------------------------------------------------------------
void PropViewFrame::CreateLeftToolBar( void )
//-----------------------------------------------------------------------------
{
    m_pLeftToolBar = new wxToolBar( m_pPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER | wxTB_VERTICAL | wxTB_FLAT | wxTB_TEXT );
    SetCommonToolBarProperties( m_pLeftToolBar );
    m_pLeftToolBar->AddTool( miCapture_CaptureSettings_CaptureSettingHierarchy, wxT( "Hierarchy" ),
                             PNGToIconImage( flowChart_png, sizeof( flowChart_png ) ),
                             PNGToIconImage( flowChart_png, sizeof( flowChart_png ) ).ConvertToDisabled(),
                             wxITEM_NORMAL,
                             wxT( "Displays the current capture setting hierarchy" ),
                             wxT( "Displays the current capture setting hierarchy" ) );
    m_pLeftToolBar->AddSeparator();
    m_pLeftToolBar->AddTool( miSettings_PropGrid_Show, wxT( "Grid" ),
                             PNGToIconImage( documentAdd_png, sizeof( documentAdd_png ) ),
                             PNGToIconImage( documentAdd_png, sizeof( documentAdd_png ) ).ConvertToDisabled(),
                             wxITEM_CHECK,
                             wxT( "Displays or hides the device property grid" ),
                             wxT( "Displays or hides the device property grid" ) );
    m_pLeftToolBar->AddTool( miSettings_PropGrid_UseSelectorGrouping, wxT( "Selector" ),
                             PNGToIconImage( selector_png, sizeof( selector_png ) ),
                             PNGToIconImage( selector_png, sizeof( selector_png ) ).ConvertToDisabled(),
                             wxITEM_CHECK,
                             wxT( "If active, selector <-> selected feature relationships will be reflected by the property grid" ),
                             wxT( "If active, selector <-> selected feature relationships will be reflected by the property grid" ) );
    m_pLeftToolBar->AddSeparator();
    m_pLeftToolBar->AddTool( miSettings_Display_Active, wxT( "Display" ),
                             PNGToIconImage( frame_png, sizeof( frame_png ) ),
                             PNGToIconImage( frame_png, sizeof( frame_png ) ).ConvertToDisabled(),
                             wxITEM_CHECK,
                             wxT( "Turns the main display on or off" ),
                             wxT( "Turns the main display on or off" ) );
    m_pLeftToolBar->AddTool( miSettings_Display_ShowIncompleteFrames, wxT( "Incomp." ),
                             PNGToIconImage( lightning_png, sizeof( lightning_png ) ),
                             PNGToIconImage( lightning_png, sizeof( lightning_png ) ).ConvertToDisabled(),
                             wxITEM_CHECK,
                             wxT( "If active, incomplete frames are displayed as well" ),
                             wxT( "If active, incomplete frames are displayed as well" ) );
    m_pLeftToolBar->AddTool( miSettings_Display_ShowMonitorImage, wxT( "Monitor" ),
                             PNGToIconImage( frame_png, sizeof( frame_png ) ),
                             PNGToIconImage( frame_png, sizeof( frame_png ) ).ConvertToDisabled(),
                             wxITEM_CHECK,
                             wxT( "Displays or hides the scaled image to monitor the visible part of the main display(NOTE: This feature can only be used when the main display is active!)" ),
                             wxT( "Displays or hides the scaled image to monitor the visible part of the main display(NOTE: This feature can only be used when the main display is active!)" ) );
    m_pLeftToolBar->AddSeparator();
    m_pLeftToolBar->AddTool( miSettings_Analysis_ShowControls, wxT( "Analysis" ),
                             PNGToIconImage( heartMonitor_png, sizeof( heartMonitor_png ) ),
                             PNGToIconImage( heartMonitor_png, sizeof( heartMonitor_png ) ).ConvertToDisabled(),
                             wxITEM_CHECK,
                             wxT( "Displays or hides the analysis tabs" ),
                             wxT( "Displays or hides the analysis tabs" ) );
    m_pLeftToolBar->AddTool( miSettings_Analysis_SynchronizeAOIs, wxT( "Sync. AOI" ),
                             PNGToIconImage( selectingTool_png, sizeof( selectingTool_png ) ),
                             PNGToIconImage( selectingTool_png, sizeof( selectingTool_png ) ).ConvertToDisabled(),
                             wxITEM_CHECK,
                             wxT( "When active, this will automatically use the same AOI for all analysis plots(the 'Full AOI' option for individual plots will remain unaffected by this)" ),
                             wxT( "When active, this will automatically use the same AOI for all analysis plots(the 'Full AOI' option for individual plots will remain unaffected by this)" ) );
    m_pLeftToolBar->AddSeparator();
    m_pLeftToolBar->AddTool( miWizard_Open, wxT( "Wizard" ),
                             PNGToIconImage( magicWand_png, sizeof( magicWand_png ) ),
                             PNGToIconImage( magicWand_png, sizeof( magicWand_png ) ).ConvertToDisabled(),
                             wxITEM_NORMAL,
                             wxT( "Opens a wizard for the configuration of certain properties(Will become enabled if a property supporting a wizard gets selected)" ),
                             wxT( "Opens a wizard for the configuration of certain properties(Will become enabled if a property supporting a wizard gets selected)" ) );
    m_pLeftToolBar->AddSeparator();
    m_pLeftToolBar->AddTool( miHelp_FindFeature, wxT( "Find" ),
                             PNGToIconImage( find_png, sizeof( find_png ) ),
                             PNGToIconImage( find_png, sizeof( find_png ) ).ConvertToDisabled(),
                             wxITEM_NORMAL,
                             wxT( "Opens a dialog to locate a certain feature in the property grid" ),
                             wxT( "Opens a dialog to locate a certain feature in the property grid" ) );
    m_pLeftToolBar->AddTool( miHelp_DriverInformation, wxT( "Info" ),
                             PNGToIconImage( info_png, sizeof( info_png ) ),
                             PNGToIconImage( info_png, sizeof( info_png ) ).ConvertToDisabled(),
                             wxITEM_NORMAL,
                             wxT( "Displays detailed information about detected drivers" ),
                             wxT( "Displays detailed information about detected drivers" ) );
    m_pLeftToolBar->AddTool( miHelp_OnlineDocumentation, wxT( "Help" ),
                             PNGToIconImage( help_png, sizeof( help_png ) ),
                             PNGToIconImage( help_png, sizeof( help_png ) ).ConvertToDisabled(),
                             wxITEM_NORMAL,
                             wxT( "Launches a browser and navigates to the online documentation" ),
                             wxT( "Launches a browser and navigates to the online documentation" ) );
    m_pLeftToolBar->AddTool( miHelp_About, wxT( "About" ),
                             PNGToIconImage( help_png, sizeof( help_png ) ),
                             PNGToIconImage( help_png, sizeof( help_png ) ).ConvertToDisabled(),
                             wxITEM_NORMAL,
                             wxT( "Displays the about dialog(including some usage hints)" ),
                             wxT( "Displays the about dialog(including some usage hints)" ) );
#ifdef _WIN32
    m_pLeftToolBar->AddSeparator();
    m_pLeftToolBar->AddTool( miHelp_OpenLogFilesFolder, wxT( "Logs" ),
                             PNGToIconImage( logFile_png, sizeof( logFile_png ) ),
                             PNGToIconImage( logFile_png, sizeof( logFile_png ) ).ConvertToDisabled(),
                             wxITEM_NORMAL,
                             wxT( "Opens the folder which contains the log files" ),
                             wxT( "Opens the folder which contains the log files" ) );
#endif // _WIN32
    m_pLeftToolBar->Realize();
}

//-----------------------------------------------------------------------------
void PropViewFrame::CreateUpperToolBar( void )
//-----------------------------------------------------------------------------
{
    m_pUpperToolBar = CreateToolBar( wxNO_BORDER | wxHORIZONTAL | wxTB_FLAT | wxTB_TEXT );
    SetCommonToolBarProperties( m_pUpperToolBar );
    m_pDevCombo = new wxComboBox( m_pUpperToolBar, widDevCombo, m_NoDevStr, wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_DROPDOWN | wxCB_READONLY );
    m_pDevCombo->SetSize( wxSize( 350, wxDefaultCoord ) ); // at least with wxWidgets 3.x on Linux passing the size as a constructor argument in the line above results in the control being drawn incorrectly!
    m_pDevCombo->Select( 0 );
    m_pDevCombo->SetToolTip( wxT( "Allows to select which device to work with. If devices seem to miss make sure these are connected AND a driver for the corresponding device family has been installed" ) );
    m_pUpperToolBar->AddControl( m_pDevCombo );
    m_pUpperToolBar->AddControl( new wxStaticText( m_pUpperToolBar, wxID_ANY, wxT( "  " ) ) );
    m_pUpperToolBar->AddTool( miAction_UseDevice, wxT( "Use" ),
                              PNGToIconImage( power_png, sizeof( power_png ) ),
                              PNGToIconImage( power_png, sizeof( power_png ) ).ConvertToDisabled(),
                              wxITEM_CHECK,
                              wxT( "Opens or closes the selected device" ),
                              wxT( "Opens or closes the selected device" ) );
    m_pUpperToolBar->AddTool( miAction_UpdateDeviceList, wxT( "Update" ),
                              PNGToIconImage( refresh_png, sizeof( refresh_png ) ),
                              PNGToIconImage( refresh_png, sizeof( refresh_png ) ).ConvertToDisabled(),
                              wxITEM_NORMAL,
                              wxT( "Updates the device list" ),
                              wxT( "Updates the device list" ) );
    m_pUpperToolBar->AddSeparator();
    m_pUpperToolBar->AddTool( miWizards_QuickSetup, wxT( "Quick Setup" ),
                              PNGToIconImage( wizardHead_png, sizeof( wizardHead_png ) ),
                              PNGToIconImage( wizardHead_png, sizeof( wizardHead_png ) ).ConvertToDisabled(),
                              wxITEM_CHECK,
                              wxT( "Opens the Quick Setup Dialog" ),
                              wxT( "Opens the Quick Setup Dialog" ) );
    m_pUpperToolBar->AddSeparator();
    m_pAcquisitionModeCombo = new wxComboBox( m_pUpperToolBar, wxID_ANY, m_ContinuousStr, wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_DROPDOWN | wxCB_READONLY );
    m_pAcquisitionModeCombo->SetSize( wxSize( 100, wxDefaultCoord ) ); // at least with wxWidgets 3.x on Linux passing the size as a constructor argument in the line above results in the control being drawn incorrectly!
    m_pAcquisitionModeCombo->Select( 0 );
    m_pAcquisitionModeCombo->SetToolTip( wxT( "Allows to select one of the acquisition modes supported by this device/driver combination. Different combinations might support different sets of acquisition modes" ) );
    m_pUpperToolBar->AddControl( m_pAcquisitionModeCombo );
    m_pUpperToolBar->AddControl( new wxStaticText( m_pUpperToolBar, wxID_ANY, wxT( "  " ) ) );
    m_pUpperToolBar->AddTool( miCapture_Acquire, wxT( "Acquire" ),
                              PNGToIconImage( play_png, sizeof( play_png ) ),
                              PNGToIconImage( play_png, sizeof( play_png ) ).ConvertToDisabled(),
                              wxITEM_CHECK,
                              wxT( "Starts or stops an acquisition from the selected device in the selected acquisition mode" ),
                              wxT( "Starts or stops an acquisition from the selected device in the selected acquisition mode" ) );
    m_pUpperToolBar->AddTool( miCapture_Abort, wxT( "Abort" ),
                              PNGToIconImage( abort_png, sizeof( abort_png ) ),
                              PNGToIconImage( abort_png, sizeof( abort_png ) ).ConvertToDisabled(),
                              wxITEM_NORMAL,
                              wxT( "Empties the image request queue for the selected device but does NOT stop a running continuous acquisition" ),
                              wxT( "Empties the image request queue for the selected device but does NOT stop a running continuous acquisition" ) );
    m_pUpperToolBar->AddTool( miCapture_Unlock, wxT( "Unlock" ),
                              PNGToIconImage( unlock_png, sizeof( unlock_png ) ),
                              PNGToIconImage( unlock_png, sizeof( unlock_png ) ).ConvertToDisabled(),
                              wxITEM_NORMAL,
                              wxT( "Unlocks ALL requests locked by the application and deep copies all images currently displayed to preserve the current state of displays and analysis plots" ),
                              wxT( "Unlocks ALL requests locked by the application and deep copies all images currently displayed to preserve the current state of displays and analysis plots" ) );
    m_pUpperToolBar->AddSeparator();
    m_pUpperToolBar->AddTool( miCapture_Record, wxT( "Rec." ),
                              PNGToIconImage( record_png, sizeof( record_png ) ),
                              PNGToIconImage( record_png, sizeof( record_png ) ).ConvertToDisabled(),
                              wxITEM_CHECK,
                              wxT( "Records the next 'RequestCount'(System Settings) images" ),
                              wxT( "Records the next 'RequestCount'(System Settings) images" ) );
    m_pUpperToolBar->AddTool( miCapture_Backward, wxT( "Prev." ),
                              PNGToIconImage( backward_png, sizeof( backward_png ) ),
                              PNGToIconImage( backward_png, sizeof( backward_png ) ).ConvertToDisabled(),
                              wxITEM_NORMAL,
                              wxT( "Displays the previous recorded image" ),
                              wxT( "Displays the previous recorded image" ) );
    m_pUpperToolBar->AddTool( miCapture_Forward, wxT( "Next" ),
                              PNGToIconImage( forward_png, sizeof( forward_png ) ),
                              PNGToIconImage( forward_png, sizeof( forward_png ) ).ConvertToDisabled(),
                              wxITEM_NORMAL,
                              wxT( "Displays the next recorded image" ),
                              wxT( "Displays the next recorded image" ) );
    m_pRecordDisplaySlider = new wxSlider( m_pUpperToolBar, widSLRecordDisplay, 0, 0, 1000, wxDefaultPosition, wxSize( 150, -1 ), wxSL_HORIZONTAL | wxSL_LABELS );
    m_pRecordDisplaySlider->SetToolTip( wxT( "Can be used to quickly move within a recorded sequence of images" ) );
    m_pUpperToolBar->AddControl( m_pRecordDisplaySlider );
    m_pUpperToolBar->Realize();
}

//-----------------------------------------------------------------------------
void PropViewFrame::ChangeActiveDevice( const wxString& newSerial )
//-----------------------------------------------------------------------------
{
    m_pDevPropHandler->SetActiveDeviceSettingToDisplayDict( m_settingToDisplayDict );
    m_pDevPropHandler->SetActiveDeviceSequencerSetToDisplayMap( m_sequencerSetToDisplayMap );
    m_pDevPropHandler->SetActiveDeviceFeatureVsTimePlotInfo( m_pFeatureValueVsTimePlotArea->GetComponentToPlot(), m_pFeatureValueVsTimePlotArea->GetComponentToPlotFullPath() );
    m_pDevPropHandler->SetActiveDevice( newSerial );
    m_sequencerSetToDisplayMap = m_pDevPropHandler->GetActiveDeviceSequencerSetToDisplayMap();
    m_settingToDisplayDict = m_pDevPropHandler->GetActiveDeviceSettingToDisplayDict();
    UpdateFeatureVsTimePlotFeature();
}

//-----------------------------------------------------------------------------
void PropViewFrame::ChangeGUIForQuickSetupWizard( void )
//-----------------------------------------------------------------------------
{
    //following two lines influence the appearance of the tool bars when switching to the QuickSetupWizard
    ConfigureToolBar( m_pLeftToolBar, false );
    ConfigureToolBar( m_pUpperToolBar, false );
    m_pPanel->GetSizer()->Layout();
    SetMenuBar( 0 );
    m_pMISettings_PropGrid_Show->Check( false );
    m_pLeftToolBar->ToggleTool( miSettings_PropGrid_Show, false );
    SetupVerSplitter();
    m_DisplayAreas[0]->SetScaling( true );
    m_DisplayAreas[0]->SetActive( true );
    m_pMISettings_Analysis_ShowControls->Check( false );
    m_pLeftToolBar->ToggleTool( miSettings_Analysis_ShowControls, false );
    RefreshDisplays( true );
    SetupDisplayLogSplitter();
}

//-----------------------------------------------------------------------------
wxScrolledWindow* PropViewFrame::CreateImageAnalysisPlotControls( wxWindow* pParent, int windowIDOffset )
//-----------------------------------------------------------------------------
{
    int index = GetAnalysisControlIndex( windowIDOffset );
    wxScrolledWindow* pPanel = new wxScrolledWindow( pParent );
    pPanel->SetScrollRate( 10, 10 );
    wxBoxSizer* pSizer = new wxBoxSizer( wxHORIZONTAL );
    wxBoxSizer* pControlsSizer = new wxBoxSizer( wxVERTICAL );
    m_ImageAnalysisPlots[index].m_pNBDisplayMethod = new wxNotebook( pPanel, windowIDOffset + iapidNBDisplayMethod, wxDefaultPosition, wxDefaultSize );
    switch( windowIDOffset )
    {
    case widPixelHistogram:
        RegisterAnalysisPlot<HistogramCanvasPixel>( index, widPixelHistogram, widPixelHistogram | iapidGrid, wxColour( 255, 0, 128 ) );
        break;
    case widSpatialNoiseHistogram:
        RegisterAnalysisPlot<HistogramCanvasSpatialNoise>( index, widSpatialNoiseHistogram, widSpatialNoiseHistogram | iapidGrid, wxColour( 255, 0, 255 ) );
        break;
    case widTemporalNoiseHistogram:
        RegisterAnalysisPlot<HistogramCanvasTemporalNoise>( index, widTemporalNoiseHistogram, widTemporalNoiseHistogram | iapidGrid, *wxGREEN );
        break;
    case widLineProfileHorizontal:
        RegisterAnalysisPlot<LineProfileCanvasHorizontal>( index, widLineProfileHorizontal, widLineProfileHorizontal | iapidGrid, *wxBLUE );
        break;
    case widLineProfileVertical:
        RegisterAnalysisPlot<LineProfileCanvasVertical>( index, widLineProfileVertical, widLineProfileVertical | iapidGrid, wxColour( 255, 128, 0 ) );
        break;
    case widIntensityPlot:
        RegisterAnalysisPlot<PlotCanvasIntensity>( index, widIntensityPlot, widIntensityPlot | iapidGrid, *wxRED );
        break;
    case widVectorScope:
        RegisterAnalysisPlot<VectorScopeCanvas>( index, widVectorScope, widVectorScope | iapidGrid, wxColour( 255, 128, 128 ) );
        break;
    default:
        wxASSERT( false && "Invalid window ID offset for this function" );
        return 0;
    }
    m_ImageAnalysisPlots[index].m_pNBDisplayMethod->InsertPage( 0, m_ImageAnalysisPlots[index].m_pPlotCanvas, wxT( "Graphical" ), true );
    m_ImageAnalysisPlots[index].m_pCBAOIFullMode = new wxCheckBox( pPanel, windowIDOffset + iapidCBAOIFullMode, wxT( "Full AOI Mode" ) );
    pControlsSizer->Add( m_ImageAnalysisPlots[index].m_pCBAOIFullMode, wxSizerFlags().Left() );
    pControlsSizer->AddSpacer( 5 );

    if( m_ImageAnalysisPlots[index].m_pPlotCanvas->HasFeature( PlotCanvasImageAnalysis::pfProcessBayerParity ) )
    {
        m_ImageAnalysisPlots[index].m_pCBProcessBayerParity = new wxCheckBox( pPanel, windowIDOffset + iapidCBProcessBayerParity, wxT( "Process Bayer Parity" ) );
        m_ImageAnalysisPlots[index].m_pCBProcessBayerParity->SetToolTip( wxT( "When enabled, Bayer raw images will be treated as 4 channel images" ) );
        pControlsSizer->Add( m_ImageAnalysisPlots[index].m_pCBProcessBayerParity, wxSizerFlags().Left() );
        pControlsSizer->AddSpacer( 5 );
    }

    wxFlexGridSizer* pGridSizer = new wxFlexGridSizer( 2 );
    pGridSizer->AddGrowableCol( 1, 2 );
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "AOI X-Offset:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_ImageAnalysisPlots[index].m_pSCAOIx = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCAOIx, wxT( "0" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 639, 0 );
    pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCAOIx, wxSizerFlags().Expand() );
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "AOI Y-Offset:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_ImageAnalysisPlots[index].m_pSCAOIy = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCAOIy, wxT( "0" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 479, 0 );
    pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCAOIy, wxSizerFlags().Expand() );
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "AOI Width:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_ImageAnalysisPlots[index].m_pSCAOIw = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCAOIw, wxT( "640" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 640, 640 );
    pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCAOIw, wxSizerFlags().Expand() );
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "AOI Height:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_ImageAnalysisPlots[index].m_pSCAOIh = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCAOIh, wxT( "480" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 480, 480 );
    pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCAOIh, wxSizerFlags().Expand() );
    if( m_ImageAnalysisPlots[index].m_pPlotCanvas->HasPlotSelection() )
    {
        pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "Plot Selection:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
        m_ImageAnalysisPlots[index].m_pCoBPlotSelection = new wxComboBox( pPanel, windowIDOffset + iapidCoBPlotSelection, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_DROPDOWN | wxCB_READONLY );
        const vector<wxString>& v( m_ImageAnalysisPlots[index].m_pPlotCanvas->GetAvailablePlotSelections() );
        const vector<wxString>::size_type cnt = v.size();
        for( vector<wxString>::size_type i = 0; i < cnt; i++ )
        {
            int item = m_ImageAnalysisPlots[index].m_pCoBPlotSelection->Append( v[i] );
            m_ImageAnalysisPlots[index].m_pCoBPlotSelection->SetClientData( item, reinterpret_cast<void*>( i ) );
        }
        m_ImageAnalysisPlots[index].m_pCoBPlotSelection->Select( 0 );
        pGridSizer->Add( m_ImageAnalysisPlots[index].m_pCoBPlotSelection, wxSizerFlags().Expand() );
    }
    if( m_ImageAnalysisPlots[index].m_pPlotCanvas->HasFeature( PlotCanvasImageAnalysis::pfHistoryDepth ) )
    {
        pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "History Depth:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
        m_ImageAnalysisPlots[index].m_pSCHistoryDepth = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCHistoryDepth, wxT( "0" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 5, 10000, 20 );
        pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCHistoryDepth, wxSizerFlags().Expand() );
    }
    if( m_ImageAnalysisPlots[index].m_pPlotCanvas->HasFeature( PlotCanvasImageAnalysis::pfPercentageWindow ) )
    {
        pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "Draw Start(%):" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
        m_ImageAnalysisPlots[index].m_pSCDrawStart_percent = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCDrawStart_percent, wxT( "0" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 99, 0 );
        pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCDrawStart_percent, wxSizerFlags().Expand() );
        pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "Draw Window(%):" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
        m_ImageAnalysisPlots[index].m_pSCDrawWindow_percent = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCDrawWindow_percent, wxT( "100" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 100, 100 );
        pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCDrawWindow_percent, wxSizerFlags().Expand() );
    }
    if( m_ImageAnalysisPlots[index].m_pPlotCanvas->HasFeature( PlotCanvasImageAnalysis::pfStepWidth ) )
    {
        wxStaticText* pSTDrawStepWidth = new wxStaticText( pPanel, wxID_ANY, wxT( "Draw Step Width:" ) );
        pSTDrawStepWidth->SetToolTip( wxT( "Defines the number of points of a profile or histogram line that will be drawn. 1 will draw every value(slow), 0 will not draw more than 256 points. Values different from 1 might result in strange plots because of rounding" ) );
        pGridSizer->Add( pSTDrawStepWidth, wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
        m_ImageAnalysisPlots[index].m_pSCDrawStepWidth = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCDrawStepWidth, wxT( "0" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, HistogramCanvas::GetDrawStepWidthMax(), 0 );
        m_ImageAnalysisPlots[index].m_pSCDrawStepWidth->SetToolTip( wxT( "Defines the number of points of a profile or histogram line that will be drawn. 1 will draw every value(slow), 0 will not draw more than 256 points. Values different from 1 might result in strange plots because of rounding" ) );
        pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCDrawStepWidth, wxSizerFlags().Expand() );
    }
    if( m_ImageAnalysisPlots[index].m_pPlotCanvas->HasFeature( PlotCanvasImageAnalysis::pfGridSteps ) )
    {
        pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "Grid Steps X:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
        m_ImageAnalysisPlots[index].m_pSCGridStepsX = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCGridStepsX, wxT( "1" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 100, 1 );
        pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCGridStepsX, wxSizerFlags().Expand() );
        pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "Grid Steps Y:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
        m_ImageAnalysisPlots[index].m_pSCGridStepsY = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCGridStepsY, wxT( "1" ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 100, 1 );
        pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCGridStepsY, wxSizerFlags().Expand() );
    }
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( "Update Interval:" ) ), wxSizerFlags().Align( wxALIGN_CENTER_VERTICAL ) );
    m_ImageAnalysisPlots[index].m_pSCUpdateSpeed = new wxSpinCtrl( pPanel, windowIDOffset + iapidSCUpdateSpeed,
            wxString::Format( wxT( "%d" ), m_ImageAnalysisPlots[index].m_pPlotCanvas->GetUpdateFrequency() ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS,
            m_ImageAnalysisPlots[index].m_pPlotCanvas->GetUpdateFrequencyMin(),
            m_ImageAnalysisPlots[index].m_pPlotCanvas->GetUpdateFrequencyMax(),
            m_ImageAnalysisPlots[index].m_pPlotCanvas->GetUpdateFrequency() );
    pGridSizer->Add( m_ImageAnalysisPlots[index].m_pSCUpdateSpeed, wxSizerFlags().Expand() );
    pControlsSizer->Add( pGridSizer );
    pSizer->Add( pControlsSizer, wxSizerFlags().Left() );
    m_ImageAnalysisPlots[index].m_pPlotCanvas->RegisterImageCanvas( m_pCurrentAnalysisDisplay );
    m_ImageAnalysisPlots[index].m_pNBDisplayMethod->SetMinSize( wxSize( 100, 50 ) );
    pSizer->Add( m_ImageAnalysisPlots[index].m_pNBDisplayMethod, wxSizerFlags( 1 ).Expand() );
    pPanel->SetSizer( pSizer );
    return pPanel;
}

//-----------------------------------------------------------------------------
void PropViewFrame::Deinit( void )
//-----------------------------------------------------------------------------
{
    StopPropertyGridUpdateTimer();
    m_boShutdownInProgress = true;
    const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        m_DisplayAreas[i]->SetActive( false );
        m_CurrentRequestDataContainer[i] = RequestData();
    }
    DeleteElement( m_pDevPropHandler );
    DeleteElement( m_pWindowDisabler );
}

//-----------------------------------------------------------------------------
PlotCanvasImageAnalysis* PropViewFrame::DeselectAnalysisPlot( void )
//-----------------------------------------------------------------------------
{
    PlotCanvasImageAnalysis* pCurrentAnalysisPlot = const_cast<PlotCanvasImageAnalysis*>( m_pCurrentAnalysisDisplay->GetActiveAnalysisPlot() );
    if( pCurrentAnalysisPlot )
    {
        pCurrentAnalysisPlot->SetActive( false );
    }
    return pCurrentAnalysisPlot;
}

//-----------------------------------------------------------------------------
void PropViewFrame::DestroyAdditionalDialogs( void )
//-----------------------------------------------------------------------------
{
    DestroyDialog( &m_pColorCorrectionDlg );
    DestroyDialog( &m_pLensControlDlg );
    DestroyDialog( &m_pLUTControlDlg );
    if( m_pQuickSetupDlgCurrent != NULL )
    {
        m_pQuickSetupDlgCurrent->Hide();
    }
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void PropViewFrame::DestroyDialog( _Ty** ppDialog )
//-----------------------------------------------------------------------------
{
    if( ppDialog && *ppDialog )
    {
        ( *ppDialog )->Destroy();
        ( *ppDialog ) = 0;
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::DisplaySettingLoadSaveDeleteErrorMessage( const wxString& msgPrefix, int originalErrorCode )
//-----------------------------------------------------------------------------
{
    wxString msg( msgPrefix );
    int reasonIndex = 1;
    if( ( originalErrorCode != DMR_FEATURE_NOT_AVAILABLE ) &&
        ( originalErrorCode != PROPHANDLING_INCOMPATIBLE_COMPONENTS ) )
    {
        wxPlatformInfo platformInfo;
        if( ( platformInfo.GetOperatingSystemId() == wxOS_WINDOWS_NT ) &&
            ( platformInfo.GetOSMajorVersion() >= 6 ) )
        {
            msg.append( wxString::Format( wxT( "\nPossible reason %d: You are running %s version %d.%d. With active UAC you must start wxPropView 'as administrator' even if you are logged on as an administrator in order to load/save/delete settings globally.\n" ), reasonIndex++, platformInfo.GetOperatingSystemIdName().c_str(), platformInfo.GetOSMajorVersion(), platformInfo.GetOSMinorVersion() ) );
        }
    }
    if( originalErrorCode == PROPHANDLING_INCOMPATIBLE_COMPONENTS )
    {
        msg.append( wxString::Format( wxT( "\nPossible reason %d:\nThe selected setting is not compatible with the selected device. It might have been created using a different device family or product. For devices operated in 'GenICam' interface layout a setting must have been created with a device of the same product type while in 'DeviceSpecific' interface layout also the same family would be sufficient.\n" ), reasonIndex++ ) );
        msg.append( wxString::Format( wxT( "\nPossible reason %d:\nThe selected setting is not compatible with the current driver version. It might have been created using a newer driver version that uses a different setting format.\n" ), reasonIndex++ ) );
    }
    if( originalErrorCode == DMR_EXECUTION_PROHIBITED )
    {
        msg.Append( wxString::Format( wxT( "\nPossible reason %d:\nSome devices e.g. those operated in interface layout 'GenICam' do no support import/export of settings while they stream image/chunk data, so stopping the acquisition might fix the problem.\n" ), reasonIndex++ ) );
    }
    WriteErrorMessage( wxString::Format( wxT( "%s\n" ), msg.c_str() ) );
    wxMessageBox( msg, wxT( "Failed to load/save setting" ), wxOK | wxICON_INFORMATION, this );
}

//-----------------------------------------------------------------------------
void PropViewFrame::EndFullScreenMode( void )
//-----------------------------------------------------------------------------
{
    if( m_boImageCanvasInFullScreenMode )
    {
        const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
        for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
        {
            if( m_DisplayAreas[i]->IsFullScreen() )
            {
                m_DisplayAreas[i]->SetFullScreenMode( false );
            }
        }
    }
    else if( ( m_pMISettings_ToggleFullScreenMode->IsChecked() ) || IsFullScreen() )
    {
        m_pMISettings_ToggleFullScreenMode->Check( false );
        ShowFullScreen( false );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::EnsureAcquisitionState( bool boAcquire )
//-----------------------------------------------------------------------------
{
    mvIMPACT::acquire::Device* pDev = m_pDevPropHandler->GetActiveDevice();
    CaptureThread* pCT = 0;
    m_pDevPropHandler->GetDeviceData( pDev, 0, 0, &pCT );
    if( ( boAcquire != pCT->GetLiveMode() ) &&
        pDev->isOpen() &&
        ( pDev->state.read() == dsPresent ) )
    {
        Acquire();
    }
}

//-----------------------------------------------------------------------------
int PropViewFrame::GetAnalysisControlIndex( void ) const
//-----------------------------------------------------------------------------
{
    wxString selectedPageName = m_pLowerRightWindow->GetPageText( m_pLowerRightWindow->GetSelection() );
    for( int i = 0; i < iapLAST; i++ )
    {
        if( m_ImageAnalysisPlots[i].m_pPlotCanvas && ( m_ImageAnalysisPlots[i].m_pPlotCanvas->GetName() == selectedPageName ) )
        {
            return i;
        }
    }
    return -1;
}

//-----------------------------------------------------------------------------
int PropViewFrame::GetDesiredDeviceIndex( wxString& serial ) const
//-----------------------------------------------------------------------------
{
    string serialANSI( serial.mb_str() );
    Device* pDev = 0;
    if( m_pDevPropHandler )
    {
        pDev = m_pDevPropHandler->GetDevMgr().getDeviceBySerial( serialANSI );
    }
    const wxString serialFromDeviceManager( pDev ? ConvertedString( pDev->serial.read().c_str() ) : wxString( wxT( "" ) ) );
    const int cnt = m_pDevCombo->GetCount();
    for( int i = 0; i < cnt; i++ )
    {
        const wxString serialFromComboBox( m_pDevCombo->GetString( i ).BeforeFirst( wxT( ' ' ) ) );
        if( ( serialFromComboBox == serial ) || ( serialFromComboBox == serialFromDeviceManager ) )
        {
            return i;
        }
    }
    return wxNOT_FOUND;
}

//-----------------------------------------------------------------------------
ComponentIterator PropViewFrame::GetDriversIterator( void ) const
//-----------------------------------------------------------------------------
{
    const DeviceManager& devMgr = m_pDevPropHandler->GetDevMgr();
    ComponentIterator itDrivers( devMgr.getInternalHandle() );
    ComponentLocator locator( itDrivers.parent() );
    return ComponentIterator( locator.findComponent( "Drivers" ) );
}

//-----------------------------------------------------------------------------
unsigned int PropViewFrame::GetGenTLDeviceCount( void ) const
//-----------------------------------------------------------------------------
{
    if( IsGenTLDriverInstalled() == false )
    {
        return 0;
    }

    ComponentLocator locator( GetDriversIterator() );
    locator.bindSearchBase( locator.findComponent( "mvGenTLConsumer" ) );
    if( locator.findComponent( "Devices" ) == INVALID_ID )
    {
        return 0;
    }
    return ComponentList( locator.findComponent( "Devices" ) ).size();
}

//-----------------------------------------------------------------------------
bool PropViewFrame::IsGenTLDriverInstalled( void ) const
//-----------------------------------------------------------------------------
{
    ComponentLocator locator( GetDriversIterator() );
    if( locator.findComponent( "mvGenTLConsumer" ) == INVALID_ID )
    {
        return false;
    }

    locator.bindSearchBase( locator.findComponent( "mvGenTLConsumer" ) );
    return locator.findComponent( "FullPath" ) != INVALID_ID;
}

//-----------------------------------------------------------------------------
bool PropViewFrame::LoadActiveDeviceFromFile( const wxString& path )
//-----------------------------------------------------------------------------
{
    FunctionInterface* p = 0;
    m_pDevPropHandler->GetActiveDevice( &p );
    if( p )
    {
        const string pathANSI( path.mb_str() );
        const int result = LoadDeviceSetting( p, pathANSI, sfDefault, sGlobal );
        if( result == DMR_NO_ERROR )
        {
            UpdateSettingTable();
            WriteLogMessage( wxString::Format( wxT( "Successfully imported data from %s.\n" ), path.c_str() ) );
            return true;
        }
        else
        {
            DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Importing data from %s failed(error code: %d(%s)).\n" ), path.c_str(), result, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
        }
    }
    else
    {
        wxMessageBox( wxT( "You can only load a setting if the device currently selected has been initialised before." ), wxT( "Failed to load setting" ), wxOK | wxICON_INFORMATION, this );
    }
    return false;
}

//-----------------------------------------------------------------------------
int PropViewFrame::LoadDeviceSetting( FunctionInterface* pFI, const string& name, const TStorageFlag flags, const TScope scope )
//-----------------------------------------------------------------------------
{
    wxBusyCursor busyCursorScope;
    m_stopWatch.Start();
    const int result = pFI->loadSetting( name, flags, scope );
    UpdateLUTWizardAfterLoadSetting( result );
    WriteLogMessage( wxString::Format( wxT( "Loading a setting for device '%s' took %ld ms.\n" ), GetSelectedDeviceSerial().c_str(), m_stopWatch.Time() ) );
    return result;
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnAction_DefaultDeviceInterface_Changed( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    switch( e.GetId() )
    {
    case miAction_DefaultDeviceInterface_DeviceSpecific:
        m_defaultDeviceInterfaceLayout = wxT( "DeviceSpecific" );
        break;
    case miAction_DefaultDeviceInterface_GenICam:
        m_defaultDeviceInterfaceLayout = wxT( "GenICam" );
        break;
    default:
        return;
    }
    UpdateDeviceInterfaceLayouts();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnActivateDisplay( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        m_DisplayAreas[i]->SetActive( e.IsChecked() );
    }
    m_pMISettings_Display_Active->Check( e.IsChecked() );
    bool boShowMonitorDisplay = e.IsChecked() && m_pMISettings_Display_ShowMonitorImage->IsChecked();
    m_pMonitorImage->GetDisplayArea()->SetActive( boShowMonitorDisplay );
    m_pMonitorImage->Show( boShowMonitorDisplay );
    m_pLeftToolBar->ToggleTool( miSettings_Display_Active, e.IsChecked() );
    SetupDisplayLogSplitter();
    SetupDlgControls();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnConfigureImageDisplayCount( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    const long newDisplayCountX = ::wxGetNumberFromUser( wxT( "Enter the new number of display windows in horizontal direction" ),
                                  wxT( "New horizontal display window count" ),
                                  wxT( "Horizontal Display Window Count" ), m_displayCountX, 1, DISPLAY_COUNT_MAX, this );
    if( newDisplayCountX <= 0 )
    {
        WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
        return;
    }
    const long newDisplayCountY = ::wxGetNumberFromUser( wxT( "Enter the new number of display windows in vertical direction" ),
                                  wxT( "New vertical display window count" ),
                                  wxT( "Vertical Display Window Count" ), m_displayCountY, 1, DISPLAY_COUNT_MAX, this );
    if( newDisplayCountY <= 0 )
    {
        WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
        return;
    }

    m_displayCountX = static_cast<unsigned int>( newDisplayCountX );
    m_displayCountY = static_cast<unsigned int>( newDisplayCountY );
    CreateDisplayWindows();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnAnalysisPlotCBAOIFullMode( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    int plotIndex = GetAnalysisControlIndex( e.GetId() );
    m_ImageAnalysisPlots[plotIndex].m_pPlotCanvas->SetAOIFullMode( e.IsChecked() );
    if( m_ImageAnalysisPlots[plotIndex].m_pPlotCanvas->IsActive() )
    {
        m_ImageAnalysisPlots[plotIndex].m_pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex],
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIx->GetValue(),
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIy->GetValue(),
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIw->GetValue(),
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIh->GetValue(),
                true );
        m_pCurrentAnalysisDisplay->Refresh( false );
    }
    m_ImageAnalysisPlots[plotIndex].UpdateControls();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnAnalysisPlotCBProcessBayerParity( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    int plotIndex = GetAnalysisControlIndex( e.GetId() );
    m_ImageAnalysisPlots[plotIndex].m_pPlotCanvas->SetProcessBayerParity( e.IsChecked() );
    if( m_ImageAnalysisPlots[plotIndex].m_pPlotCanvas->IsActive() )
    {
        m_ImageAnalysisPlots[plotIndex].m_pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex],
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIx->GetValue(),
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIy->GetValue(),
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIw->GetValue(),
                m_ImageAnalysisPlots[plotIndex].m_pSCAOIh->GetValue(),
                true );
        m_pCurrentAnalysisDisplay->Refresh( false );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnBtnRecord( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        pThread->SetRecordMode( e.IsChecked() );
        m_pMICapture_Record->Check( e.IsChecked() );
        GetToolBar()->ToggleTool( miCapture_Record, e.IsChecked() );
        SetupUpdateFrequencies( true );
    }
    SetupDlgControls();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnBtnUnlock( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    CloneAndUnlockAllRequestsAndFreeRecordedSequence();
    SetupDlgControls();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCapture_CaptureSettings_CreateCaptureSetting( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    FunctionInterface* pFI = 0;
    CaptureThread* pCT = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( &pFI, 0, &pCT );
    if( pDev && pFI && pCT )
    {
        const vector<string>& settings( pFI->getAvailableSettings() );
        wxString settingName = ::wxGetTextFromUser( wxT( "Enter the name of the setting that shall be created(no white-spaces). A setting must have a unique name.\nPlease note that newly created settings can only be accessed via the property\ngrid is in 'Multiple Settings' or 'Developers' view mode." ),
                               wxT( "Name of the setting" ),
                               wxString::Format( wxT( "NewSetting%u" ), static_cast<unsigned int>( settings.size() ) ),
                               this );

        if( settingName.IsEmpty() )
        {
            WriteLogMessage( wxT( "Operation canceled by the user OR empty setting name.\n" ) );
            return;
        }

        wxArrayString choices;
        vector<string>::const_iterator it = settings.begin();
        const vector<string>::const_iterator itEND = settings.end();
        while( it != itEND )
        {
            choices.Add( ConvertedString( *it ) );
            ++it;
        }
        wxSingleChoiceDialog dlg( this, wxT( "Please select the setting that shall be used as a parent for the setting to create.\nUntouched parameters in the new setting will automatically change\nwhen the parent settings parameter is modified.\nFor detailed information about the behaviour of settings please refer to the manual." ), wxT( "Select the parent setting" ), choices );
        if( dlg.ShowModal() == wxID_OK )
        {
            wxString parentSetting( dlg.GetStringSelection() );
            if( parentSetting.IsEmpty() )
            {
                WriteErrorMessage( wxT( "No parent setting selected.\n" ) );
            }

            const string settingNameANSI( settingName.mb_str() );
            const string parentSettingANSI( parentSetting.mb_str() );
            ComponentList newSetting;
            int result = pFI->createSetting( settingNameANSI, parentSettingANSI, &newSetting );
            if( result == DMR_NO_ERROR )
            {
                UpdateSettingTable();
                ImageRequestControl irc( pDev );
                wxCriticalSectionLocker locker( m_critSect );
                m_settingToDisplayDict.insert( make_pair( newSetting.hObj(), ( irc.setting.dictSize() - 1 ) % GetDisplayCount() ) );
            }
            else
            {
                WriteErrorMessage( wxString::Format( wxT( "Creation of setting %s based on %s failed. Result: %s.\n" ), settingName.c_str(), parentSetting.c_str(), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
            }
        }
        else
        {
            WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCapture_CaptureSettings_CaptureSettingHierarchy( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    FunctionInterface* pFI = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( &pFI );
    if( pDev && pFI )
    {
        const vector<string>& settings( pFI->getAvailableSettings() );
        StringToStringMap m;
        try
        {
            vector<string>::size_type cnt = settings.size();
            for( vector<string>::size_type i = 0; i < cnt; i++ )
            {
                DeviceComponentLocator locator( pDev, dltSetting, settings[i] );
                PropertyS basedOn( locator.findComponent( "BasedOn" ) );
                m.insert( make_pair( ConvertedString( settings[i] ), ConvertedString( basedOn.read() ) ) );
            }
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to create setting hierarchy: %s(%s)\n" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            return;
        }

        SettingHierarchyDlg dlg( this, wxString( wxT( "Current Capture Setting Hierarchy" ) ), m );
        dlg.ShowModal();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCapture_CaptureSettings_AssignToDisplays( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev )
    {
        try
        {
            ImageRequestControl irc( pDev );
            vector<pair<string, int> > settings;
            irc.setting.getTranslationDict( settings );
            AssignSettingsToDisplaysDlg dlg( this, wxString( wxT( "Settings To Display Setup" ) ), settings, m_settingToDisplayDict, GetDisplayCount() );
            if( dlg.ShowModal() == wxID_OK )
            {
                const vector<wxControl*>& ctrls = dlg.GetUserInputControls();
                wxCriticalSectionLocker locker( m_critSect );
                m_settingToDisplayDict.clear();
                const vector<pair<string, int> >::size_type settingCount = settings.size();
                for( vector<pair<string, int> >::size_type i = 0; i < settingCount; i++ )
                {
                    wxString selection( dynamic_cast<wxComboBox*>( ctrls[i] )->GetValue().AfterLast( wxT( ' ' ) ) );
                    long index = 0;
                    if( selection.ToLong( &index ) )
                    {
                        m_settingToDisplayDict.insert( make_pair( settings[i].second, index ) );
                    }
                    else
                    {
                        WriteErrorMessage( wxString::Format( wxT( "Failed to obtain display index from selection '%s'.\n" ), selection.c_str() ) );
                    }
                }
            }
            else
            {
                WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
            }
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to create setting hierarchy: %s(%s)\n" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCapture_CaptureSettings_UsageMode_Changed( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    switch( e.GetId() )
    {
    case miCapture_CaptureSettings_UsageMode_Manual:
        SetupCaptureSettingsUsageMode( csumManual );
        break;
    case miCapture_CaptureSettings_UsageMode_Automatic:
        SetupCaptureSettingsUsageMode( csumAutomatic );
        break;
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCapture_DefaultImageProcessingMode_Changed( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    switch( e.GetId() )
    {
    case miCapture_DefaultImageProcessingMode_ProcessAll:
        m_defaultImageProcessingMode = ipmDefault;
        break;
    case miCapture_DefaultImageProcessingMode_ProcessLatestOnly:
        m_defaultImageProcessingMode = ipmProcessLatestOnly;
        break;
    }
    UpdateUserControlledImageProcessingEnableProperties();
    SetupImageProcessingMode();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCBEnableInfoPlot( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pInfoPlotArea->SetActive( m_pCBEnableInfoPlot->IsChecked() );
    if( m_pCBEnableInfoPlot->IsChecked() )
    {
        m_pInfoPlotArea->ClearCache();
        m_pInfoPlotArea->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].requestInfo_, true );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCBInfoPlotDifferences( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pInfoPlotArea->SetPlotDifferences( m_pCBInfoPlotDifferences->IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCBInfoPlotAutoScale( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pInfoPlotArea->SetAutoScale( m_pCBInfoPlotAutoScale->IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCBEnableFeatureValueVsTimePlot( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pFeatureValueVsTimePlotArea->SetActive( m_pCBEnableFeatureValueVsTimePlot->IsChecked() );
    if( m_pCBEnableFeatureValueVsTimePlot->IsChecked() )
    {
        m_pFeatureValueVsTimePlotArea->ClearCache();
        m_pFeatureValueVsTimePlotArea->RefreshData();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCBFeatureValueVsTimePlotDifferences( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pFeatureValueVsTimePlotArea->SetPlotDifferences( m_pCBFeatureValueVsTimePlotDifferences->IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnCBFeatureValueVsTimePlotAutoScale( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pFeatureValueVsTimePlotArea->SetAutoScale( m_pCBFeatureValueVsTimePlotAutoScale->IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnClose( wxCloseEvent& )
//-----------------------------------------------------------------------------
{
    Deinit();
    Destroy();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnContinuousRecording( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    if( m_pDevPropHandler )
    {
        m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
        if( pThread )
        {
            pThread->SetContinuousRecording( e.IsChecked() );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnDetailedRequestInformation( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    FunctionInterface* pFI = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( &pFI );
    if( pDev && pFI )
    {
        try
        {
            DetailedRequestInformationDlg dlg( this, wxString::Format( wxT( "Detailed Request Information [%s]" ), ConvertedString( pDev->serial.read() ).c_str() ), pFI );
            dlg.ShowModal();
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with the detailed request information dialog for device '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str(), ConvertedString( pDev->serial.read() ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnExportActiveDevice( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxFileDialog fileDlg( this, wxT( "Select a filename" ), wxT( "" ), wxT( "" ), wxT( "mv-settings Files (*.xml)|*.xml" ), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        wxString pathName = fileDlg.GetPath();
        FunctionInterface* p = 0;
        m_pDevPropHandler->GetActiveDevice( &p );
        if( p )
        {
            const string pathANSI( pathName.mb_str() );
            const int result = SaveDeviceSetting( p, pathANSI, sfDefault, sGlobal );
            if( result == DMR_NO_ERROR )
            {
                WriteLogMessage( wxString::Format( wxT( "Successfully stored data to %s.\n" ), pathName.c_str() ) );
            }
            else
            {
                DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Storing of data to %s failed(error code: %d(%s))." ), pathName.c_str(), result, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnHelp_About( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxBoxSizer* pTopDownSizer;
    wxDialog dlg( this, wxID_ANY, wxString( wxT( "About wxPropView" ) ), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER | wxMAXIMIZE_BOX | wxMINIMIZE_BOX );
    wxIcon icon( mvIcon_xpm );
    dlg.SetIcon( icon );

    pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    pTopDownSizer->Add( new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT( "Configuration tool for %s devices" ), COMPANY_NAME ) ), 0, wxALL | wxALIGN_CENTER, 5 );
    pTopDownSizer->Add( new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT( "(C) 2005 - %s by %s" ), CURRENT_YEAR, COMPANY_NAME ) ), 0, wxALL | wxALIGN_CENTER, 5 );
    pTopDownSizer->Add( new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT( "Version %s" ), VERSION_STRING ) ), 0, wxALL | wxALIGN_CENTER, 5 );
    AddSupportInfo( &dlg, pTopDownSizer );
    AddwxWidgetsInfo( &dlg, pTopDownSizer );
    AddSourceInfo( &dlg, pTopDownSizer );
    AddIconInfo( &dlg, pTopDownSizer );

    wxNotebook* pNotebook = new wxNotebook( &dlg, wxID_ANY, wxDefaultPosition, wxDefaultSize );

    wxTextCtrl* pUsageHints = new wxTextCtrl( pNotebook, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxBORDER_NONE | wxTE_RICH | wxTE_READONLY );
    pNotebook->AddPage( pUsageHints, wxT( "Usage Hints" ), true );

    const wxTextAttr defaultStyle( *wxBLUE );
    const wxTextAttr boldStyle( GetBoldStyle( m_pLogWindow ) );
    WriteToTextCtrl( pUsageHints, wxT( "Using the property grid:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Right-click on any feature to get a menu with additional options. This includes restoring the default value for a property or a group of features. When restoring the default for a list, all sub-lists and properties within that list will be restored to their default values as well.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Drag'n'Drop any supported capture setting for the device currently selected onto the property grid to load an apply this setting. The device must be initialised before.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Properties:\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Some properties may contain more than a single value. These are called 'vector-properties' and will be displayed as a list of values by the grid. The number of elements stored by the property will be displayed in brackets along with the property's name, while the index of a specific value will be displayed in brackets after the property's name within the list that contains the property's data.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Methods:\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Parameters must be separated by white-spaces, empty strings can be passed as a single underscore.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "To find out what kind of parameters a function expects please have a look in the manuals interface reference. Expected parameter types and return values will be displayed in a C-defaultStyle syntax:\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\tvoid: An 'empty' parameter, thus this method either does not return a value, does not expect parameters or both\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\tvoid*: A pointer to an arbitrary data type or memory location\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\tint: A 32-bit integer value\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\tint64: A 64-bit integer value\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\tfloat: A floating point value(double precision)\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\tchar*: A C-defaultStyle string\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Using the display:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Left-click onto a display area to select it for configuration and to assign this display to the analysis plots and the monitor display. This only applies when working with multiple displays(see 'Command Line Options' how to start the application using multiple displays).\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Double-left-click on any of the image displays to toggle full screen display mode.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Press '+' or '-' or use the mouse wheel for zooming in and out of the image for the selected display(this will only be active when the 'fit to screen' mode is switched off).\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Left-click into an AOI for a selected analysis page to drag it to a new location(this will only be possible if 'Full AOI Mode' for that analysis page is switched off).\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Left-click anywhere outside an AOI for a selected analysis page to drag the center of the visible display area to a new location(this will only be possible if scrollbars are currently visible for the selected display area).\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Right-click onto a display to get a pop up menu with additional options.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Press the left and right arrow keys on the keyboard or the numpad for defining a custom shift factor for the pixel data. This can be used to define which bits of a multi-byte image shall be displayed. When 'Performance Warning Overlay' from the pop up menu of the display is active, the current shift(this is the value defined by the user) value and the applied shift value(this is the shift value actually applied to the pixel data before displaying) will be displayed. E.g. for a 12 bit mono image, when 'current shift value' is has been defined as 1 by the user, then 'applied shift value' will be 3 because normally a 12 bit mono image would be displayed by shifting the data by 4 bits to the right in order to display the 8 msbs of the image. With a user defined shift value of 1, now instead of displaying bits 11 - 4, bits 10 - 3 will be displayed. The applied shift value is thus calculated from the values needed to display the 8 msbs MINUS the value defined by the user.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Right-click onto a display and drag the mouse to select a new AOI for the selected analysis page.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Drag'n'Drop any supported image file type onto the display area to open that file for analysis in the selected display area.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Importing *.raw image data:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Any file with a *.raw extension can be imported as a raw image. However you need to specify the pixel format and image dimensions in order to allow wxPropView to properly display *.raw images. " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "wxPropView can also extract this information from the file name if the filename is in the format of\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "<arbitrary prefix>.<width>x<height>.<pixel format>(BayerPattern=<pattern>).raw.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "The Bayer pattern information can be omitted.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "EXAMPLES:\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "example.640x480.Mono8(BayerPattern=Green-red).raw\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "example.640x480.RGBx888Planar.raw\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "The 'Output' window:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "This window will receive messages and additional status information by the application and driver. Whenever something is not working as expected it might be worth checking if any error message is displayed here.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Using the analysis plots:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Right-click in one of the cells of the numerical display of any of the analysis plot for a menu of additional options(Copy to Clipboard, define a custom format string for the numerical display, ...). The 'Process Bayer Parity' option will create analysis plots for all the four color components of a Bayer image, when switched off, Bayer data will be interpreted as single channel grey data.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "'Draw Start(%%)' and 'Draw Window(%%)' can be used to define a window within the analysis data thus effectively can be used to 'zoom' into certain regions of the plot. EXAMPLE: A start of 10%% and a window of 50%% would only plot values between 10 and 60 percent of overall value range.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "'Update Interval' can be used reduce the redraw frequency of the selected plot. This will save some CPU time but will not update a plot whenever a new image is displayed.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Pixel Histogram:\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Read: Channel (Mean value, most frequent value count / most frequent value)\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Spatial Noise Histogram:\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Read: Channel#Direction (Mean difference, most frequent value count/  value, Standard deviation)\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "EXAMPLE: For a single channel(Mono) image the output of 'C0Hor(3.43, 5086/  0, 9.25), C0Ver(3.26, 4840/  0, 7.30) will " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "indicate that the mean difference between pixels in horizontal direction is 3.43, the most frequent difference is 0 and this difference is present 5086 times " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "in the current AOI. The standard deviation in horizontal direction is 9.25. The C0Ver value list contains the same data but in vertical direction.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Temporal Noise Histogram:\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Read: Channel# (Mean difference, most frequent value count/  value, Standard deviation)\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "EXAMPLE: For a single channel(Mono) image the output of 'C0(3.43, 5086/  0, 9.25) will " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "indicate that the mean difference between pixels in 2 consecutive images is 3.43, the most frequent difference is 0 and this difference is present 5086 times " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "in the current AOI. The standard deviation between pixels in these 2 images is 9.25.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Please note the impact of the 'Update Interval' in this plot: It can be used to define a gap between 2 images to compare. E.g. if the update interval is set to 2,\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "the differences between image 1 and 3, 3 and 5, 5 and 7 etc. will be calculated. In order to get the difference between 2 consecutive images the update interval\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "must be set to 1!\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Using multiple settings/displays:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "This application is capable of dealing with multiple capture settings for a single device and in addition to that it can be configured to deal with " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "multiple image displays. For frame grabbers with multiple input channels this e.g. can be used to display live images from all input channels " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "simultaneously. This even works if each input channel is connected to a different video signal in terms of resolution and timing.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "The amount of image displays can be configured via the command line parameters 'dcx' and 'dcy'(see above) and can't be changed at runtime.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Additional capture settings can be created via 'Capture -> Capture Settings -> Create Capture Settings' and the property grid will display these " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "capture settings either in 'Developers' or in 'Multiple Settings' view. A step by step guide for an example setup can be found in the GUI section " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "of the device manual.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Please note that in 'GenICam' interface layout multiple capture settings are NOT supported.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Using the wizards:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Some features(e.g. LUT) are not easily configured using the property grid only. Therefore this application includes some wizards " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "in order to speed up the configuration of certain things. Wizards can be accessed either using the desired entry from the 'Wizards' " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "menu or by pressing on the 'Wizard' button in the tool bar on the left side of the screen(if there is no tool bar, it can be " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "enabled through the 'Settings' menu). In the 'Wizards' menu only those entries will be enabled that are actually supported by " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "the device currently selected(you might need to open the device to get access to all supported Wizards). If an entry here stays " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "disabled when the device has been successfully opened, this device does not support all the features required by the Wizard. The " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "'Wizard' button on the tool bar will automatically be enabled whenever a feature in the property grid gets selected that can also be " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "configured using a Wizard dialog. Pressing this button will automatically start the corresponding Wizard or will display a list of " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Wizards accessing this property for selection.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "Using the recording option:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "To record a sequence of images in RAM the 'Record' option needs to be enabled.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "The maximum size of images that can be recorded is limited by the amount of memory available to the process the application is running in " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "however when the amount of images that shall be recorded is larger than the number of request objects allocated by the driver each image " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "and the associated data will be copied by the application which might have negative influence on the overall performance. " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Thus in order to achieve maximum performance it is recommended to increase the value of the property 'RequestCount' to match the length of the sequence you plan to record. " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "The maximum value of 'RequestCount' again depends on the hardware or (more likely) is limited by the amount of memory available to the process the application " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "is running in. The acquisition will stop automatically when the desired amount of images have been recorded and each image belonging " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "to the sequence can be analyzed by using the slider and buttons in the upper tool bar. The 'Output' window will display some buffer " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "specific information whenever a new image is displayed.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "To get rid of the dialogs that pop up while recording, enable the 'Silent Mode' option in 'Capture -> Recording'.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "In order to constantly record the last 'n' images select the 'Continuous' option in 'Capture -> Recording'. With this option enabled, the " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "application will not stop acquiring data automatically but only 'RequestCount / 2' frames are kept in memory in order to have enough buffers " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "to keep a constant continuous acquisition possible. To overwrite the 'RequestCount / 2' in continuous mode use the 'Setup Capture Queue Depth' " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "option in 'Capture'. Then 'RequestCount - CaptureQueueDepth' frames will be kept in memory. Please note that at least 2 buffers should stay " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "available for the capture queue. Higher frame rates will require more buffers. EXAMPLE: 'RequestCount' is 512, continuous recording is active " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "and the capture queue depth has been set to 12. Now the last 500(512 - 12) frames will be kept in memory and 12 buffers will be used to keep the driver busy. " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Whenever there are more than 500 frames in the recorded queue, the oldest images will be discarded and new requests will be sent to the driver.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "When not using the 'Continuous' option for recording use the 'Setup Sequence Size' option in 'Capture -> Recording' " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "to define how many of the available buffers will be kept in memory.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "To directly store all displayed images to disc enable the 'Hard Disk Recording' option in 'Capture -> Recording'. Note that this will probably NOT " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "store every image captured, but every image that actually is send to the display only. When a still image is constantly redrawn(e.g. because of paint events " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "caused by moving around an AOI this will also store the image to disc multiple times, thus this option can NOT be used as a reliable capture data to " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "disc function.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "The 'Setup Capture Queue' depth can be used to define a maximum custom number of buffers used for the acquisition engine.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "'Automatically Reconnect To Unused Devices' option:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Certain devices(due to their nature) can be removed from a system without the need to e.g. switch off the system or shutdown the current " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "process. For performance reasons not all the drivers constantly check if all the devices discovered the last time the device list was updated " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "are still present or have changed their connection specific parameters(e.g. the IP address). Therefore an attempt to open such a device might " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "fail when the device has been unplugged but is present again at the time of the open device request. When the 'Automatically Reconnect To Unused " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Devices' option is enabled the application will try to open a device as usual and in case of a failure will try to reconnect to the device by updating " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "the internal device list and will then try to open the device a second time. This might take slightly longer then but may be useful when e.g." ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "playing around with different power supplies etc..\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "'Allow Fast Single Frame Acquisition' option:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "With this option disable (default) the acquisition button will be disabled while the request image has not been delivered. When enabling this option " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "multiple single frame acquisition commands can be sent to the device (depending on the speed the user clicks the button). This can result in one or " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "more single frame requests can reach the device while it is still busy dealing with the current request for a frame. Not every device does support " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "'queuing' of requests, thus some devices might simply silently discard these requests, which will result on the request returning with a 'rrTimeout' " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "error on the host side. This option is mainly there for testing purposes.\n" ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "\n" ) );
    WriteToTextCtrl( pUsageHints, wxT( "'Default Image Processing Mode' option:\n" ), boldStyle );
    WriteToTextCtrl( pUsageHints, wxT( "Several processing algorithms can be applied to a captured image by mvIMPACT Acquire on the host system AFTER the data has been captured. " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "All processing however consumes CPU resources and therefore the overall processing time needed on the host can be larger than the time between 2 consecutive " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "frames as transferred by a device. This typically results in delayed image display if the device offers a frame buffer or in lost frames if a device " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "either offers no frame buffer or the data transmission from the device to the host is not managed by requests send from the host to the device but by " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "the device pumping out data. For an application such as wxPropView it therefore can be desirable to capture at full frame rate but to display only a fraction " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "of post-processed images. This can be achieved by configuring the driver to process only the latest images from the queue of captured images and by forwarding " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "all additional data 'as captured' to an application. This can be configured by setting up the 'Default Image Processing Mode' accordingly. If images are not " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "passed through the processing pipeline they will be counted as 'skipped' giving the user a hint about the current lack of CPU resources. Please note that for " ), defaultStyle );
    WriteToTextCtrl( pUsageHints, wxT( "performance reasons, this must be set up before opening the device.\n" ), defaultStyle );
    pUsageHints->ScrollLines( -( 256 * 256 ) ); // make sure the text control always shows the beginning of the help text

    vector<pair<wxString, wxString> > v;
    v.push_back( make_pair( wxT( "CTRL+A" ), wxT( "imageRequestReset(Abort)" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+A" ), wxT( "Analysis tab on/off" ) ) );
    v.push_back( make_pair( wxT( "CTRL+B" ), wxT( "Image display(blit) on/off" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+B" ), wxT( "Backward in captured sequence" ) ) );
    v.push_back( make_pair( wxT( "CTRL+C" ), wxT( "Create capture setting" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+C" ), wxT( "Acquire" ) ) );
    v.push_back( make_pair( wxT( "CTRL+D" ), wxT( "Assign capture settings to display(s)" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+D" ), wxT( "Show connected devices only on/off" ) ) );
    v.push_back( make_pair( wxT( "CTRL+E" ), wxT( "Show left tool bar on/off" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+E" ), wxT( "Show method execution errors" ) ) );
    v.push_back( make_pair( wxT( "CTRL+F" ), wxT( "Find feature" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+F" ), wxT( "Forward in captured sequence" ) ) );
    v.push_back( make_pair( wxT( "CTRL+G" ), wxT( "Switch on/off selector grouping" ) ) );
    v.push_back( make_pair( wxT( "CTRL+H" ), wxT( "Display capture setting hierarchy" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+H" ), wxT( "Array property indices will be displayed as HEX on/off" ) ) );
    v.push_back( make_pair( wxT( "CTRL+I" ), wxT( "Single snap(Capture image)" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+I" ), wxT( "Display incomplete image on/off" ) ) );
    v.push_back( make_pair( wxT( "CTRL+L" ), wxT( "Live" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+L" ), wxT( "Unlock" ) ) );
    v.push_back( make_pair( wxT( "CTRL+M" ), wxT( "Show Monitor Image" ) ) );
    v.push_back( make_pair( wxT( "CTRL+O" ), wxT( "Load device settings from default location" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+O" ), wxT( "Load device settings from file" ) ) );
    v.push_back( make_pair( wxT( "CTRL+P" ), wxT( "Use Display names for grid features if available (More user friendly strings)" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+P" ), wxT( "Property grid on/off" ) ) );
    v.push_back( make_pair( wxT( "CTRL+R" ), wxT( "Record mode on/off" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+R" ), wxT( "Create editors with slider" ) ) );
    v.push_back( make_pair( wxT( "CTRL+S" ), wxT( "Save active device settings to default location" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+S" ), wxT( "Save active device settings to file" ) ) );
    v.push_back( make_pair( wxT( "CTRL+T" ), wxT( "Show upper tool bar on/off" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+T" ), wxT( "Show status bar on/off" ) ) );
    v.push_back( make_pair( wxT( "CTRL+U" ), wxT( "Open/Close Device" ) ) );
#ifdef _WIN32
    v.push_back( make_pair( wxT( "ALT+CTRL+U" ), wxT( "Save Logs As Zip" ) ) );
    v.push_back( make_pair( wxT( "CTRL+W" ), wxT( "Open Logfiles Folder" ) ) );
    v.push_back( make_pair( wxT( "ALT+CTRL+W" ), wxT( "Email Logfiles To MV Support" ) ) );
#endif // _WIN32
    v.push_back( make_pair( wxT( "CTRL+Q" ), wxT( "Set capture queue depth" ) ) );
    v.push_back( make_pair( wxT( "ALT+X" ), wxT( "Exit" ) ) );
    v.push_back( make_pair( wxT( "F1" ), wxT( "Display 'About' dialog" ) ) );
    v.push_back( make_pair( wxT( "F2" ), wxT( "Default Device Interface Layout 'Device Specific'" ) ) );
    v.push_back( make_pair( wxT( "F3" ), wxT( "Default Device Interface Layout 'GenICam'" ) ) );
    v.push_back( make_pair( wxT( "F5" ), wxT( "Update device list" ) ) );
    v.push_back( make_pair( wxT( "F6" ), wxT( "Grid view mode 'Standard'" ) ) );
    v.push_back( make_pair( wxT( "F7" ), wxT( "Grid view mode 'Developer'" ) ) );
    v.push_back( make_pair( wxT( "F8" ), wxT( "Display driver information dialog" ) ) );
    v.push_back( make_pair( wxT( "F9" ), wxT( "Capture Settings Usage Mode 'Manual'" ) ) );
    v.push_back( make_pair( wxT( "F10" ), wxT( "Capture Settings Usage Mode 'Automatic'" ) ) );
    v.push_back( make_pair( wxT( "F11" ), wxT( "Full Screen mode on/off" ) ) );
    v.push_back( make_pair( wxT( "F12" ), wxT( "Online Documentation" ) ) );
    AddListControlToAboutNotebook( pNotebook, wxT( "Keyboard Shortcuts" ), false, wxT( "Shortcut" ), wxT( "Command" ), v );

    v.clear();
    v.push_back( make_pair( wxT( "'width' or 'w'" ), wxT( "Defines the startup width of the application" ) ) );
    v.push_back( make_pair( wxT( "'height' or 'h'" ), wxT( "Defines the startup height of the application" ) ) );
    v.push_back( make_pair( wxT( "'xpos' or 'x'" ), wxT( "Defines the startup x position of the application" ) ) );
    v.push_back( make_pair( wxT( "'ypos' or 'y'" ), wxT( "Defines the startup y position of the application" ) ) );
    v.push_back( make_pair( wxT( "'propgridwidth' or 'pgw'" ), wxT( "Defines the startup width of the property grid" ) ) );
    v.push_back( make_pair( wxT( "'debuginfo' or 'di'" ), wxT( "Will display debug information in the property grid" ) ) );
    v.push_back( make_pair( wxT( "'dic'" ), wxT( "Will display invisible(currently shadowed) components in the property grid" ) ) );
    v.push_back( make_pair( wxT( "'displayCountX' or 'dcx'" ), wxT( "Defines the number of image displays in horizontal direction" ) ) );
    v.push_back( make_pair( wxT( "'displayCountY' or 'dcy'" ), wxT( "Defines the number of image displays in vertical direction" ) ) );
    v.push_back( make_pair( wxT( "'fulltree' or 'ft'" ), wxT( "Will display the complete property tree(including the data not meant to be accessed by the user) in the property grid" ) ) );
    v.push_back( make_pair( wxT( "'device' or 'd'" ), wxT( "Will directly open a device with a particular serial number" ) ) );
    v.push_back( make_pair( wxT( "'qsw'" ), wxT( "Will forcefully hide or show the quick setup window, regardless of the default settings" ) ) );
    v.push_back( make_pair( wxT( "'live'" ), wxT( "Will directly start live acquisition from the device opened via 'device' or 'd' directly" ) ) );
    v.push_back( make_pair( wxT( "'allowFullDeviceFileAccess'" ), wxT( "Allows to access files which are hidden by default (such as the device firmware) through the file up- and download wizard. Be sure you know what you are doing when using this parameter. You might damage your device!" ) ) );
    AddListControlToAboutNotebook( pNotebook, wxT( "Available Command Line Options" ), false, wxT( "Command" ), wxT( "Description" ), v );

    pTopDownSizer->AddSpacer( 10 );
    pTopDownSizer->Add( pNotebook, wxSizerFlags( 5 ).Expand() );
    pTopDownSizer->AddSpacer( 10 );
    wxButton* pBtnOK = new wxButton( &dlg, wxID_OK, wxT( "OK" ) );
    pBtnOK->SetDefault();
    pTopDownSizer->Add( pBtnOK, 0, wxALL | wxALIGN_RIGHT, 15 );
    dlg.SetSizer( pTopDownSizer );
    dlg.SetSizeHints( 720, 500 );
    dlg.SetSize( -1, -1, 720, 500 );
    dlg.ShowModal();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnHelp_DriverInformation( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    try
    {
        DriverInformationDlg dlg( this, wxString( wxT( "Current Driver Information" ) ), GetDriversIterator(), m_pDevPropHandler->GetDevMgr() );
        dlg.ShowModal();
    }
    catch( const ImpactAcquireException& e )
    {
        wxMessageBox( wxString::Format( wxT( "Error while trying to build list of drivers and devices: %s(%s)." ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxString( wxT( "INTERNAL ERROR" ) ), wxOK | wxICON_INFORMATION, this );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnHelp_FindFeature( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPropertyGrid* pPropGrid = GetPropertyGrid();
    if( pPropGrid )
    {
        NameToFeatureMap name2feature;
        wxPropertyGridConstIterator it = pPropGrid->GetIterator();
        while( !it.AtEnd() )
        {
            name2feature.insert( make_pair( ( *it )->GetName(), static_cast<PropData*>( ( *it )->GetClientData() ) ) );
            ++it;
        }
        FindFeatureDlg dlg( this, name2feature, m_boFindFeatureMatchCaseActive );
        dlg.ShowModal();
        m_boFindFeatureMatchCaseActive = dlg.GetMatchCase();
    }
}

#ifdef _WIN32
//-----------------------------------------------------------------------------
void PropViewFrame::CreateLogZipFile( const wxString& logFileDirectoryPath, const wxString& targetFullPath )
//-----------------------------------------------------------------------------
{
    // store the settings of all active devices
    if( m_pDevPropHandler )
    {
        const DeviceManager& devMgr = m_pDevPropHandler->GetDevMgr();
        const unsigned int devCnt = devMgr.deviceCount();
        for( unsigned int i = 0; i < devCnt; i++ )
        {
            Device* pDev = devMgr[i];
            if( pDev->isOpen() )
            {
                FunctionInterface fi( pDev );
                const wxString settingPath = wxString::Format( wxT( "%s/%s.xml" ), logFileDirectoryPath.c_str(), ConvertedString( pDev->serial.readS() ).c_str() );
                const std::string settingPathANSI( settingPath.mb_str() );
                const int result = fi.saveSetting( settingPathANSI, sfFile );
                if( result != DMR_NO_ERROR )
                {
                    DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Storing settings for device %s failed. Result: %s.\n" ), m_pDevCombo->GetValue().c_str(), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
                    return;
                }
            }
        }
    }

    // now create the ZIP container with all log-files AND all device settings
    wxArrayString logFilesArray;
    wxDir::GetAllFiles( logFileDirectoryPath, &logFilesArray, wxString( "*.*" ) );
    wxFFileOutputStream fileOutStream( targetFullPath );
    if( fileOutStream.IsOk() )
    {
        wxZipOutputStream zipOutStream( fileOutStream );
        wxTextOutputStream txtOutStream( zipOutStream );
        for( size_t i = 0; i < logFilesArray.GetCount(); i++ )
        {
            zipOutStream.PutNextEntry( logFilesArray[i].Mid( logFileDirectoryPath.length() ) );
            wxTextFile mvLogFile;
            wxString mvLogFileContents;
            mvLogFile.Open( logFilesArray[i] );
            mvLogFileContents = mvLogFile.GetFirstLine();
            txtOutStream << mvLogFileContents.c_str();
            while( mvLogFile.Eof() != true )
            {
                mvLogFileContents = mvLogFile.GetNextLine();
                txtOutStream << endl << mvLogFileContents.c_str();
            }
        }
        zipOutStream.Sync();
        zipOutStream.Close();
        fileOutStream.Close();
    }
    else
    {
        wxMessageBox( wxT( "Failed create ZIP-file with logs on Desktop.\nYou have to create the ZIP-file manually(Menu->Help->Save Logfiles As Zip)" ), wxT( "Cannot Create ZIP-file!" ), wxOK | wxICON_EXCLAMATION, this );
    }

    // clean up again (delete all the setting files just created)
    if( m_pDevPropHandler )
    {
        const DeviceManager& devMgr = m_pDevPropHandler->GetDevMgr();
        const unsigned int devCnt = devMgr.deviceCount();
        for( unsigned int i = 0; i < devCnt; i++ )
        {
            Device* pDev = devMgr[i];
            if( pDev->isOpen() )
            {
                FunctionInterface fi( pDev );
                const wxString settingPath = wxString::Format( wxT( "%s/%s.xml" ), logFileDirectoryPath.c_str(), ConvertedString( pDev->serial.readS() ).c_str() );
                ::wxRemoveFile( settingPath );
            }
        }
    }

}

//-----------------------------------------------------------------------------
wxString PropViewFrame::GetDesktopPath( void )
//-----------------------------------------------------------------------------
{
    wxString userProfile;
    if( wxGetEnv( wxString( "USERPROFILE" ), &userProfile ) == false )
    {
        wxMessageBox( wxT( "Env. variable 'USERPROFILE' is missing." ), wxT( "Missing System Environment Variable!" ), wxOK | wxICON_EXCLAMATION, this );
    }

    if( wxDir().Exists( userProfile + wxString( "\\Desktop" ) ) )
    {
        return userProfile;
    }

    wxString homeDrive;
    if( wxGetEnv( wxString( "HOMEDRIVE" ), &homeDrive ) == false )
    {
        wxMessageBox( wxT( "Env. variable 'HOMEDRIVE' is missing." ), wxT( "Missing System Environment Variable!" ), wxOK | wxICON_EXCLAMATION, this );
        return wxEmptyString;
    }
    wxString homePath;
    if( wxGetEnv( wxString( "HOMEPATH" ), &homePath ) == false )
    {
        wxMessageBox( wxT( "Env. variable 'HOMEPATH' is missing." ), wxT( "Missing System Environment Variable!" ), wxOK | wxICON_EXCLAMATION, this );
        return wxEmptyString;
    }
    return homeDrive + homePath;
}

//-----------------------------------------------------------------------------
bool PropViewFrame::CheckedGetMVIADataDir( wxString& mvDataDirectoryPath )
//-----------------------------------------------------------------------------
{
    if( !wxGetEnv( wxString( "MVIMPACT_ACQUIRE_DATA_DIR" ), &mvDataDirectoryPath ) )
    {
        wxMessageBox( wxT( "Env. variable 'MVIMPACT_ACQUIRE_DATA_DIR' is missing.\nThe Folder containing the log files cannot be located. " ), wxT( "Cannot Find MATRIX VISION Logs Folder!" ), wxOK | wxICON_EXCLAMATION, this );
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnHelp_EmailLogFilesZip( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxString mvDataDirectoryPath;
    if( CheckedGetMVIADataDir( mvDataDirectoryPath ) )
    {
        const wxString logFileDirectoryPath = mvDataDirectoryPath + wxString( "Logs\\" );
        const wxString targetFullPath = GetDesktopPath() + wxString( "\\Desktop\\mvAllLogs.zip" );
        CreateLogZipFile( logFileDirectoryPath, targetFullPath );

        wxString sendMailShellCommand = wxString( wxT( "mailto:support@matrix-vision.com?subject=MATRIX VISION GmbH log files&body=Please attach file 'mvLogsAll.zip' located on your Desktop and provide us with a description of your problem." ) );
        ShellExecute( NULL, wxT( "open" ), sendMailShellCommand, NULL, NULL, SW_SHOWNORMAL );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnHelp_OpenLogFilesFolder( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxString mvDataDirectoryPath;
    if( CheckedGetMVIADataDir( mvDataDirectoryPath ) )
    {
        const wxString executeString = wxString( "explorer " ) + mvDataDirectoryPath + wxString( "Logs" );
        wxExecute( executeString, wxEXEC_ASYNC, NULL );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnHelp_SaveLogFilesAsZip( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxString mvDataDirectoryPath;
    if( CheckedGetMVIADataDir( mvDataDirectoryPath ) )
    {
        const wxString logFileDirectoryPath = mvDataDirectoryPath + wxString( "Logs\\" );
        const wxString defaultPath = GetDesktopPath() + wxString( "\\Desktop\\" );
        wxFileDialog zipfileDlg( this, wxT( "Select a location" ), defaultPath, wxString( "mvAllLogs.zip" ), wxT( "ZIP files (*.zip)|*.zip" ), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
        if( zipfileDlg.ShowModal() == wxID_OK )
        {
            const wxString targetFullPath = zipfileDlg.GetPath();
            if( !targetFullPath.IsEmpty() )
            {
                CreateLogZipFile( logFileDirectoryPath, targetFullPath );
            }
        }
    }
}
#endif // _WIN32

//-----------------------------------------------------------------------------
void PropViewFrame::OnIncDecRecordDisplay( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        SetupUpdateFrequencies( false );
        unsigned long int currentImage = static_cast<unsigned long int>( ( e.GetId() == miCapture_Forward ) ? pThread->DisplaySequenceRequestNext() : pThread->DisplaySequenceRequestPrev() );
        WriteLogMessage( wxString::Format( wxT( "Recorded image %lu: " ), currentImage ) );
        m_pRecordDisplaySlider->SetValue( currentImage );
        m_boCurrentImageIsFromFile = false;
        UpdateData( true, true, true );
    }
    SetupDlgControls();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnImageCanvasFullScreen( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_boImageCanvasInFullScreenMode = ( e.GetInt() != 0 );
    if( m_boImageCanvasInFullScreenMode )
    {
        m_pWindowDisabler = new wxWindowDisabler();
    }
    else
    {
        delete m_pWindowDisabler;
        m_pWindowDisabler = 0;
    }
    GetPropertyGrid()->ClearSelection();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnImageInfo( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    ReEnableSingleCapture();
    auto_ptr<RequestInfoData> p( reinterpret_cast<RequestInfoData*>( e.GetClientData() ) );
    if( ( ( p->requestResult_ == rrOK ) || ( ( p->requestResult_ == rrFrameIncomplete ) && m_pMISettings_Display_ShowIncompleteFrames->IsChecked() ) ) && m_pInfoPlotArea )
    {
        m_pInfoPlotArea->RefreshData( *p );
        if( !m_boImageCanvasInFullScreenMode && m_pInfoPlotArea->IsActive() )
        {
            wxDateTime dt( static_cast<time_t>( p->timeStamp_us_ / 1000000 ) );
            dt.SetMillisecond( static_cast<wxDateTime::wxDateTime_t>( ( p->timeStamp_us_ % 1000000 ) / 1000 ) );
            m_pTCTimestampAsTime->SetValue( wxString::Format( wxT( "%02d:%02d:%02d:%03d" ), dt.GetHour(), dt.GetMinute(), dt.GetSecond(), dt.GetMillisecond() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnImageReady( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    if( m_boShutdownInProgress )
    {
        return;
    }
    // If the active device has changed after this event was generated but before it gets processed here
    // do nothing. Otherwise requests might get unlocked for the current device based on data valid for
    // the previous one!
    if( reinterpret_cast<Device*>( e.GetClientData() ) == m_pDevPropHandler->GetActiveDevice() )
    {
        m_boNewImageDataAvailable = true;
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnImageSkipped( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    if( m_boShutdownInProgress )
    {
        return;
    }

    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
    size_t cnt = 1;
    if( pThread )
    {
        cnt = pThread->GetSkippedImageCount();
    }

    int index = 0;
    {
        wxCriticalSectionLocker locker( m_critSect );
        /// \todo handle 'sequencerSetActive -> Display' here as well?
        SettingToDisplayDict::const_iterator it = m_settingToDisplayDict.find( e.GetInt() );
        index = static_cast<int>( ( it != m_settingToDisplayDict.end() ) ? it->second : 0 );
    }
    if( m_pLogWindow && !m_DisplayAreas.empty() && m_DisplayAreas[index]->IsActive() )
    {
        m_DisplayAreas[index]->IncreaseSkippedCounter( cnt );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnInfoPlot_PlotSelectionComboTextChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        const string pathANSI( m_pInfoPlotSelectionCombo->GetValue().mb_str() );
        pThread->SetCurrentPlotInfoPath( pathANSI );
    }
    m_pInfoPlotArea->ClearCache();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnLiveModeAborted( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    wxString msg( wxString::Format( wxT( "The live mode has been aborted because of the following error: '%s'.\n" ), e.GetString().c_str() ) );
    wxMessageDialog dlg( NULL, msg, wxT( "Information" ), wxOK | wxICON_INFORMATION );
    dlg.ShowModal();
    WriteErrorMessage( msg );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnImageTimeoutEvent( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pQuickSetupDlgCurrent && m_pQuickSetupDlgCurrent->IsVisible() && m_boHandleImageTimeoutEvents )
    {
        m_boHandleImageTimeoutEvents = false;
        m_pQuickSetupDlgCurrent->ShowImageTimeoutPopup();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnLoadActiveDevice( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pDevPropHandler )
    {
        FunctionInterface* p = 0;
        m_pDevPropHandler->GetActiveDevice( &p );
        if( p )
        {
            wxArrayString choices;
            choices.Add( wxString( wxT( "user specific(default)" ) ) );
            choices.Add( wxString( wxT( "system wide" ) ) );
            wxSingleChoiceDialog dlg( this, wxT( "Please select the scope from where for this setting shall be imported.\nSystem wide settings can be used by every user logged on to this system,\nbut only certain users might be able to modify them." ), wxT( "Select the settings scope" ), choices );
            if( dlg.ShowModal() == wxID_OK )
            {
                const string serialANSI( GetSelectedDeviceSerial().mb_str() );
                const int result = LoadDeviceSetting( p, serialANSI, sfNative, ( dlg.GetSelection() == 0 ) ? sUser : sGlobal );
                if( result == DMR_NO_ERROR )
                {
                    UpdateSettingTable();
                    WriteLogMessage( wxString::Format( wxT( "Successfully loaded current settings with %s scope for device %s.\n" ), ( dlg.GetSelection() == 0 ) ? wxT( "user specific" ) : wxT( "global" ), m_pDevCombo->GetValue().c_str() ) );
                }
                else
                {
                    DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Loading settings for the current device failed. Result: %s.\n" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnLoadActiveDeviceFromFile( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxFileDialog fileDlg( this, wxT( "Select a filename" ), wxT( "" ), wxT( "" ), wxT( "mv-settings Files (*.xml)|*.xml" ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        LoadActiveDeviceFromFile( fileDlg.GetPath() );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnLoadCurrentProduct( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pDevPropHandler )
    {
        FunctionInterface* p = 0;
        Device* pDev = m_pDevPropHandler->GetActiveDevice( &p );
        if( p && pDev )
        {
            wxArrayString choices;
            choices.Add( wxString( wxT( "user specific(default)" ) ) );
            choices.Add( wxString( wxT( "system wide" ) ) );
            wxSingleChoiceDialog dlg( this, wxT( "Please select the scope from where for this setting shall be imported.\nSystem wide settings can be used by every user logged on to this system,\nbut only certain users might be able to modify them." ), wxT( "Select the settings scope" ), choices );
            if( dlg.ShowModal() == wxID_OK )
            {
                const int result = LoadDeviceSetting( p, pDev->product.read(), sfNative, ( dlg.GetSelection() == 0 ) ? sUser : sGlobal );
                if( result == DMR_NO_ERROR )
                {
                    UpdateSettingTable();
                    WriteLogMessage( wxString::Format( wxT( "Successfully loaded current settings with %s scope for product range %s.\n" ), ( dlg.GetSelection() == 0 ) ? wxT( "user specific" ) : wxT( "global" ), ConvertedString( pDev->product.read() ).c_str() ) );
                }
                else
                {
                    DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Loading settings for the current product type failed. Result: %s.\n" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnLoadFromDefault( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pDevPropHandler )
    {
        FunctionInterface* p = 0;
        m_pDevPropHandler->GetActiveDevice( &p );
        if( p )
        {
            wxBusyCursor busyCursorScope;
            m_stopWatch.Start();
            const int result = p->loadSettingFromDefault();
            UpdateLUTWizardAfterLoadSetting( result );
            WriteLogMessage( wxString::Format( wxT( "Loading a setting for device '%s' took %ld ms.\n" ), GetSelectedDeviceSerial().c_str(), m_stopWatch.Time() ) );
            if( result == DMR_NO_ERROR )
            {
                UpdateSettingTable();
                WriteLogMessage( wxT( "Successfully loaded current settings for the current device from the default location.\n" ) );
            }
            else
            {
                DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Loading settings for the current device from the default location failed. Result: %s.\n" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnLoadImage( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxFileDialog fileDlg( this, wxT( "Select a file" ), wxT( "" ), wxT( "" ), wxT( "Supported image files (*.bmp;*.gif;*.jpeg;*.jpg;*.png;*.tif;*.raw)|*.bmp;*.gif;*.jpeg;*.jpg;*.png;*.tif;*.raw" ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        SetCurrentImage( wxFileName( fileDlg.GetPath() ), m_pCurrentAnalysisDisplay );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnManageSettings( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pDevPropHandler )
    {
        Device* pDev = m_pDevPropHandler->GetActiveDevice();
        if( pDev && pDev->autoLoadSettingOrder.isValid() )
        {
            map<size_t, SettingData> data;
            wxArrayString choices;
            const unsigned int valCount = pDev->autoLoadSettingOrder.valCount();
            for( unsigned int i = 0; i < valCount; i++ )
            {
                const string setting( pDev->autoLoadSettingOrder.read( i ) );
                if( OBJ_IsSettingAvailable( setting.c_str(), sfNative, sUser ) == PROPHANDLING_NO_ERROR )
                {
                    data.insert( make_pair( choices.GetCount(), SettingData( sUser, setting ) ) );
                    choices.Add( ConvertedString( setting ) );
                }
                if( OBJ_IsSettingAvailable( setting.c_str(), sfNative, sGlobal ) == PROPHANDLING_NO_ERROR )
                {
                    data.insert( make_pair( choices.GetCount(), SettingData( sGlobal, setting ) ) );
                    choices.Add( ConvertedString( setting ) + wxString( wxT( "(global storage)" ) ) );
                }
            }
            if( choices.IsEmpty() )
            {
                wxMessageDialog dlg( this, wxT( "There are no settings available for configuration for this device!" ) );
                dlg.ShowModal();
            }
            else
            {
                wxMultiChoiceDialog dlg( this, wxT( "Please select the settings that shall be deleted.\nPlease note that settings that have a name different from the serial\nnumber of the current device might affect the behaviour of other devices as well." ), wxT( "Select settings" ), choices );
                if( dlg.ShowModal() == wxID_OK )
                {
                    wxArrayInt selections = dlg.GetSelections();
                    const size_t selectionCount = selections.GetCount();
                    for( size_t j = 0; j < selectionCount; j++ )
                    {
                        map<size_t, SettingData>::const_iterator it = data.find( selections[j] );
                        if( it == data.end() )
                        {
                            WriteErrorMessage( wxT( "INTERNAL ERROR! Setting can't be deleted as it can't be located in the internal hash table.\n" ) );
                            continue;
                        }
                        TPROPHANDLING_ERROR result = OBJ_DeleteSetting( it->second.name_.c_str(), sfNative, it->second.scope_ );
                        if( result != PROPHANDLING_NO_ERROR )
                        {
                            DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Failed to delete setting %s(scope: %d). Result: %s.\n" ),
                                    ConvertedString( it->second.name_ ).c_str(),
                                    it->second.scope_,
                                    ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
                        }
                    }
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnNBDisplayMethodPageChanged( wxNotebookEvent& )
//-----------------------------------------------------------------------------
{
    if( m_CurrentImageAnalysisControlIndex >= 0 )
    {
        m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->SetDisplayMethod( static_cast<PlotCanvasImageAnalysis::TDisplayMethod>( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pNBDisplayMethod->GetSelection() ) );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnNotebookPageChanged( wxNotebookEvent& )
//-----------------------------------------------------------------------------
{
    PlotCanvasImageAnalysis* p = DeselectAnalysisPlot();
    m_CurrentImageAnalysisControlIndex = GetAnalysisControlIndex();
    if( p && m_pMISettings_Analysis_SynchroniseAOIs->IsChecked() )
    {
        wxRect activeAOI = p->GetAOIRect();
        ConfigureAnalysisPlot( &activeAOI );
    }
    else
    {
        ConfigureAnalysisPlot();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnPopUpPropAttachCallback( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp && GetPropertyGrid()->GetPropertyClientData( pProp ) )
    {
        m_pPropViewCallback->registerComponent( static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) )->GetComponent() );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnPopUpPropDetachCallback( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp && GetPropertyGrid()->GetPropertyClientData( pProp ) )
    {
        m_pPropViewCallback->unregisterComponent( static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) )->GetComponent() );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnPopUpPropPlotInFeatureValueVsTimePlot( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp && GetPropertyGrid()->GetPropertyClientData( pProp ) )
    {
        Component comp( static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) )->GetComponent() );
        m_pDevPropHandler->SetActiveDeviceFeatureVsTimePlotInfo( comp, pProp->GetName() );
        UpdateFeatureVsTimePlotFeature();
        WriteLogMessage( wxString::Format( wxT( "'%s' has been selected for plotting every %d ms." ),
                                           ConvertedString( comp.name().c_str() ).c_str(),
                                           m_pFeatureValueVsTimePlotArea->GetUpdateFrequency() ) );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnPopUpPropReadFromFile( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp && GetPropertyGrid()->GetPropertyClientData( pProp ) )
    {
        Component comp = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) )->GetComponent();
        if( ( comp.type() == ctPropString ) && ( comp.flags() & cfContainsBinaryData ) )
        {
            // The 'GetComponent().type()' check is only needed because some drivers with versions < 1.12.33
            // did incorrectly specify the 'cfContainsBinaryData' flag even though the data type was not 'ctPropString'...
            PropertyS prop( comp );
            const unsigned int valCount = prop.valCount();
            int index = 0;
            if( valCount > 1 )
            {
                index = static_cast<int>( ::wxGetNumberFromUser( wxT( "Select the property's value index to load a file into" ), wxT( "Value Index" ), wxT( "Property Value Index Selection" ), 0, 0, valCount - 1, this ) );
            }
            wxFileDialog fileDlg( this, wxString::Format( wxT( "Select a file to load into the property's value %d" ), index ), wxT( "" ), wxT( "" ), wxT( "All types (*.*)|*.*" ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
            if( fileDlg.ShowModal() == wxID_OK )
            {
                wxFile file( fileDlg.GetPath().c_str(), wxFile::read );
                if( !file.IsOpened() )
                {
                    WriteErrorMessage( wxString::Format( wxT( "Failed to open file %s.\n" ), fileDlg.GetPath().c_str() ) );
                    return;
                }

                const wxFileOffset maxFileSize = 32 * 1024;
                if( file.Length() > maxFileSize )
                {
                    wxString msg;
                    msg << wxT( "File '" ) << fileDlg.GetPath() << wxT( "' is too large(" ) << file.Length() << wxT( " bytes). The editor only supports files up to " ) << maxFileSize << wxT( " bytes.\n" );
                    WriteErrorMessage( msg );
                    wxMessageBox( msg, wxT( "Reading File Into Property Failed" ) );
                    return;
                }
                char* pBuf = new char[file.Length()];
                if( file.Read( pBuf, file.Length() ) == file.Length() )
                {
                    string data( pBuf, file.Length() );
                    try
                    {
                        prop.writeBinary( data, index );
                    }
                    catch( const ImpactAcquireException& e )
                    {
                        WriteErrorMessage( wxString::Format( wxT( "Failed to write data to value %d of property '%s': %s(%s)\n" ), index, ConvertedString( comp.name() ).c_str(), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
                    }
                }
                else
                {
                    WriteErrorMessage( wxString::Format( wxT( "Failed to read file %s completely.\n" ), fileDlg.GetPath().c_str() ) );
                }
                delete [] pBuf;
            }
            else
            {
                WriteLogMessage( wxString::Format( wxT( "Operation canceled by user while processing value %d of property '%s'.\n" ), index, ConvertedString( comp.name() ).c_str() ) );
            }
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "Feature '%s' does not support binary data storage.\n" ), ConvertedString( comp.name() ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnPopUpPropWriteToFile( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp && GetPropertyGrid()->GetPropertyClientData( pProp ) )
    {
        Component comp = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) )->GetComponent();
        if( ( comp.type() == ctPropString ) && ( comp.flags() & cfContainsBinaryData ) )
        {
            // The 'GetComponent().type()' check is only needed because some drivers with versions < 1.12.33
            // did incorrectly specify the 'cfContainsBinaryData' flag even though the data type was not 'ctPropString'...
            PropertyS prop( comp );
            const unsigned int valCount = prop.valCount();
            int index = 0;
            if( valCount > 1 )
            {
                index = static_cast<int>( ::wxGetNumberFromUser( wxT( "Select the property's value index to store into a file" ), wxT( "Value Index" ), wxT( "Property Value Index Selection" ), 0, 0, valCount - 1, this ) );
            }
            wxFileDialog fileDlg( this, wxString::Format( wxT( "Select a filename to store the property's value %d to" ), index ), wxT( "" ), wxT( "" ), wxT( "All types (*.*)|*.*" ), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
            if( fileDlg.ShowModal() == wxID_OK )
            {
                string data( prop.readBinary( index ) );
                WriteFile( data.c_str(), data.length(), fileDlg.GetPath(), m_pLogWindow );
            }
            else
            {
                WriteLogMessage( wxString::Format( wxT( "Operation canceled by user while processing value %d of property '%s'.\n" ), index, ConvertedString( comp.name() ).c_str() ) );
            }
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "Feature '%s' does not support binary data storage.\n" ), ConvertedString( comp.name() ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnPropertyGridTimer( void )
//-----------------------------------------------------------------------------
{
    if( !m_boImageCanvasInFullScreenMode )
    {
        if( GetPropertyGrid()->IsShown() )
        {
            // while the property grid is not displayed we don't need to update it
            m_pDevPropHandler->ValidateTrees();
        }
        SetupDlgControls();
        UpdateStatusBar();
        if( m_pQuickSetupDlgCurrent && m_pQuickSetupDlgCurrent->IsVisible() )
        {
            const Device* pDev = m_pDevPropHandler->GetActiveDevice();
            if( pDev && pDev->isOpen() && ( pDev->state.read() == dsPresent ) )
            {
                m_pQuickSetupDlgCurrent->UpdateControlsData();
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnRefreshAOIControls( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_CurrentImageAnalysisControlIndex < 0 )
    {
        return;
    }

    // this will reconstruct an AOI for the selected image analysis page as a result of rectangle being dragged with the right mouse button pressed down
    const AOI* const pAOI = m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->GetAOI();
    if( pAOI )
    {
        ConfigureAOI( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex], pAOI->m_rect );
        m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex], pAOI->m_rect.GetLeft(), pAOI->m_rect.GetTop(), pAOI->m_rect.GetWidth(), pAOI->m_rect.GetHeight() );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnRefreshCurrentPixelData( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pLastMouseHooverDisplay = m_DisplayAreas[e.GetInt()];
    RefreshCurrentPixelData();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSaveActiveDevice( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    FunctionInterface* p = 0;
    m_pDevPropHandler->GetActiveDevice( &p );
    if( p )
    {
        wxArrayString choices;
        choices.Add( wxString( wxT( "user specific(default)" ) ) );
        choices.Add( wxString( wxT( "system wide" ) ) );
        wxSingleChoiceDialog dlg( this, wxT( "Please select the scope to which this setting shall be exported.\nPlease note that storing a setting for system wide usage might\nrequire certain user privileges not available to every user.\nSystem wide settings can be used by every user logged on to this system." ), wxT( "Select the settings scope" ), choices );
        if( dlg.ShowModal() == wxID_OK )
        {
            const string pathANSI( GetSelectedDeviceSerial().mb_str() );
            const int result = SaveDeviceSetting( p, pathANSI, sfNative, ( dlg.GetSelection() == 0 ) ? sUser : sGlobal );
            if( result == DMR_NO_ERROR )
            {
                WriteLogMessage( wxString::Format( wxT( "Successfully stored current settings with %s scope for device %s.\n" ), ( dlg.GetSelection() == 0 ) ? wxT( "user specific" ) : wxT( "global" ), m_pDevCombo->GetValue().c_str() ) );
            }
            else
            {
                DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Storing settings for device %s failed. Result: %s.\n" ), m_pDevCombo->GetValue().c_str(), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSaveCurrentProduct( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    FunctionInterface* p = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( &p );
    if( p && pDev )
    {
        wxArrayString choices;
        choices.Add( wxString( wxT( "user specific(default)" ) ) );
        choices.Add( wxString( wxT( "system wide" ) ) );
        wxSingleChoiceDialog dlg( this, wxT( "Please select the scope to which this setting shall be exported.\nPlease note that storing a setting for system wide usage might\nrequire certain user privileges not available to every user.\nSystem wide settings can be used by every user logged on to this system." ), wxT( "Select the settings scope" ), choices );
        if( dlg.ShowModal() == wxID_OK )
        {
            const int result = SaveDeviceSetting( p, pDev->product.read(), sfNative, ( dlg.GetSelection() == 0 ) ? sUser : sGlobal );
            if( result == DMR_NO_ERROR )
            {
                WriteLogMessage( wxString::Format( wxT( "Successfully stored current settings with %s scope for product range %s.\n" ), ( dlg.GetSelection() == 0 ) ? wxT( "user specific" ) : wxT( "global" ), ConvertedString( pDev->product.read() ).c_str() ) );
            }
            else
            {
                DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Storing settings for product range %s failed. Result: %s.\n" ), ConvertedString( pDev->product.read() ).c_str(), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSaveImage( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxFileDialog fileDlg( this, wxT( "Select a filename" ), wxT( "" ), wxT( "" ), wxT( "BMP files (*.bmp)|*.bmp|PNG files (*.png)|*.png|RAW files (*.raw)|*.raw" ), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        wxString fileName = fileDlg.GetPath();
        if( fileName.IsEmpty() )
        {
            return;
        }

        wxFileName::SplitPath( fileName, 0, 0, 0 );
        SaveImage( fileName, static_cast<TFileFormat>( fileDlg.GetFilterIndex() ) );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSaveImageSequenceToFiles( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        wxFileDialog fileDlg( this, wxT( "Select a filename" ), wxT( "" ), wxT( "" ), wxT( "BMP files (*.bmp)|*.bmp|PNG files (*.png)|*.png|RAW files (*.raw)|*.raw" ), wxFD_SAVE );
        if( fileDlg.ShowModal() == wxID_OK )
        {
            wxString fileName = fileDlg.GetPath();
            if( fileName.IsEmpty() )
            {
                return;
            }
            wxFileName tmpFilename( fileName );
            wxString name;
            wxFileName::SplitPath( fileName, 0, &name, 0 );

            const RequestContainer::size_type size = pThread->GetRecordedSequenceSize();
            // get the number of digits for the size of the sequence to build a proper format string padded with just enough zeros
            RequestContainer::size_type tmpSize = size;
            streamsize digitCount = 0;
            while( tmpSize )
            {
                ++digitCount;
                tmpSize /= 10;
            }

            m_boCurrentImageIsFromFile = false;
            for( RequestContainer::size_type i = 0; i < size; i++ )
            {
                // make sure ALL images will be written to files
                ClearDisplayInProgressStates();

                pThread->DisplaySequenceRequest( i );
                UpdateData( false, true, true );
                ostringstream oss;
                oss << setw( digitCount ) << setfill( '0' ) << i;
                tmpFilename.SetName( wxString::Format( wxT( "%s.%s" ), name.c_str(), ConvertedString( oss.str() ).c_str() ) );
                SaveImage( tmpFilename.GetFullPath(), static_cast<TFileFormat>( fileDlg.GetFilterIndex() ) );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSaveImageSequenceToStream( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        wxFileDialog fileDlg( this, wxT( "Select a filename" ), wxT( "" ), wxT( "" ), wxT( "RAW Stream (*.rawstream)|*.rawstream" ), wxFD_SAVE );
        if( fileDlg.ShowModal() == wxID_OK )
        {
            if( fileDlg.GetPath().IsEmpty() )
            {
                return;
            }

            const RequestContainer::size_type size = pThread->GetRecordedSequenceSize();
            m_boCurrentImageIsFromFile = false;

            wxFile file( fileDlg.GetPath().c_str(), wxFile::write );
            if( !file.IsOpened() )
            {
                WriteErrorMessage( wxString::Format( wxT( "Storing of %s failed. Could not open file.\n" ), fileDlg.GetPath().c_str() ) );
                return;
            }

            for( RequestContainer::size_type i = 0; i < size; i++ )
            {
                pThread->DisplaySequenceRequest( i );
                UpdateData( false, true, true );
                size_t bytesWritten = file.Write( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->vpData, m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iSize );
                if( bytesWritten < static_cast<size_t>( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iSize ) )
                {
                    WriteErrorMessage( wxString::Format( wxT( "Storing of frame %d into %s failed. Could not write all data(bytes written: %d, frame size: %d).\n" ), i, fileDlg.GetPath().c_str(), bytesWritten, m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iSize ) );
                    return;
                }
            }
            WriteLogMessage( wxString::Format( wxT( "Successfully wrote %d frames into %s.\n" ), size, fileDlg.GetPath().c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSaveToDefault( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    FunctionInterface* p = 0;
    m_pDevPropHandler->GetActiveDevice( &p );
    if( p )
    {
        wxBusyCursor busyCursorScope;
        m_stopWatch.Start();
        const int result = p->saveSettingToDefault();
        WriteLogMessage( wxString::Format( wxT( "Saving a setting for device '%s' took %ld ms.\n" ), GetSelectedDeviceSerial().c_str(), m_stopWatch.Time() ) );
        if( result == DMR_NO_ERROR )
        {
            WriteLogMessage( wxT( "Successfully stored current settings for the current device to the default location.\n" ) );
        }
        else
        {
            DisplaySettingLoadSaveDeleteErrorMessage( wxString::Format( wxT( "Storing settings for the current device to the default location failed. Result: %s.\n" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), result );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSequenceReady( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        wxString msg;
        switch( e.GetInt() )
        {
        case eosrRecordingDone:
        case eosrRecordingDone_OutOfRAM:
            // in case there is also a pending image that has been reported
            // stop this from being display as now that our sequence has been
            // recorded completely we want to display the last image of the
            // sequence instead!
            m_boNewImageDataAvailable = false;
            SetupDlgControls();
            UpdateRecordedImage( m_pRecordDisplaySlider->GetMax() );
            msg = wxString::Format( wxT( "All %lu ('RequestCount') images recorded.\n" ), static_cast<unsigned long int>( pThread->GetRecordedSequenceSize() ) );
            if( e.GetInt() == eosrRecordingDone_OutOfRAM )
            {
                msg.Append( wxString::Format( wxT( "\nRecording stopped early (%lu images desired). Note that a 32-bit process\nmight not be able to acquire more memory, for a 64-bit process there might\nnot be enough RAM.\n" ), static_cast<unsigned long int>( pThread->GetRecordSequenceSize() ) ) );
            }
            if( !m_pMICapture_Recording_SlientMode->IsChecked() )
            {
                wxMessageDialog dlg( NULL, msg, wxT( "Info" ), wxOK | wxICON_WARNING );
                dlg.ShowModal();
            }
            break;
        case eosrMultiFrameDone:
            msg = wxString::Format( wxT( "MultiFrame sequence ready. %lu requests processed.\n" ), static_cast<unsigned long int>( pThread->GetMultiFrameSequenceSize() ) );
            break;
        }
        if( m_pLogWindow && !msg.empty() )
        {
            WriteLogMessage( msg );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSettings_Analysis_ShowControls( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pMISettings_Analysis_ShowControls->Check( e.IsChecked() );
    m_pLeftToolBar->ToggleTool( miSettings_Analysis_ShowControls, e.IsChecked() );
    RefreshDisplays( true );
    SetupDisplayLogSplitter();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSettings_Analysis_SynchronizeAOIs( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pMISettings_Analysis_SynchroniseAOIs->Check( e.IsChecked() );
    m_pLeftToolBar->ToggleTool( miSettings_Analysis_SynchronizeAOIs, e.IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSettings_PropGrid_CreateEditorsWithSlider( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pMISettings_PropGrid_CreateEditorsWithSlider->Check( e.IsChecked() );
    GetPropertyGrid()->ClearSelection();
    wxPGCustomSpinCtrlEditor_PropertyObject::Instance()->ConfigureControlsCreation( e.IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSettings_PropGrid_Show( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pMISettings_PropGrid_Show->Check( e.IsChecked() );
    m_pLeftToolBar->ToggleTool( miSettings_PropGrid_Show, e.IsChecked() );
    SetupVerSplitter();
    RefreshDisplays( true );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSettings_PropGrid_UseSelectorGrouping( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pMISettings_PropGrid_UseSelectorGrouping->Check( e.IsChecked() );
    m_pLeftToolBar->ToggleTool( miSettings_PropGrid_UseSelectorGrouping, e.IsChecked() );
    UpdatePropGridViewMode();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSettings_PropGrid_ViewModeChanged( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    switch( e.GetId() )
    {
    case miSettings_PropGrid_ViewMode_StandardView:
        m_ViewMode = DevicePropertyHandler::vmStandard;
        break;
    case miSettings_PropGrid_ViewMode_DevelopersView:
        m_ViewMode = DevicePropertyHandler::vmDeveloper;
        break;
    }
    UpdatePropGridViewMode();
    m_ViewMode = m_pDevPropHandler->GetViewMode();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSettings_SetUpdateFrequency( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    const long newValue = ::wxGetNumberFromUser( wxT( "Enter the new update frequency in Hz to use for display and analysis plots." ),
                          wxT( "New update frequency(Hz)" ),
                          wxT( "Set Update Frequency" ),
                          m_displayUpdateFrequency,
                          5,
                          200,
                          this );
    if( newValue >= 0 )
    {
        m_displayUpdateFrequency = newValue;
        StartDisplayUpdateTimer();
    }
    else
    {
        WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSetupCaptureQueueDepth( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    FunctionInterface* pFI = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( &pFI, 0, &pThread );

    if( pDev && pFI && pThread )
    {
        const int maxCaptureQueueDepth = pFI->requestCount() - 1;
        long newValue = ::wxGetNumberFromUser( wxString::Format( wxT( "Enter the new capture queue depth for device %s.\nNote that this can't be larger than the number of request buffers currently\nallocated - 1 buffer that is currently displayed and/or analysed.\n'0' will restore the internal default behaviour." ), ConvertedString( pDev->serial.read().c_str() ).c_str() ),
                                               wxT( "New capture queue depth" ),
                                               wxT( "Setup Capture Queue Depth" ),
                                               pThread->GetCaptureQueueDepth(),
                                               0,
                                               maxCaptureQueueDepth,
                                               this );
        if( newValue >= 0 )
        {
            pThread->SetCaptureQueueDepth( newValue );
        }
        else
        {
            WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSetupHardDiscRecording( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxMessageDialog dlg( NULL,
                         wxT( "Enable hard disc recording?" ),
                         wxT( "Configure Hard Disc Recording" ),
                         wxNO_DEFAULT | wxYES_NO );

    m_HardDiscRecordingParameters.boActive_ = dlg.ShowModal() == wxID_YES;
    if( !m_HardDiscRecordingParameters.boActive_ )
    {
        WriteLogMessage( wxT( "Hard disc recording disabled.\n" ) );
        return;
    }

    m_HardDiscRecordingParameters.targetDirectory_ = ::wxDirSelector( wxT( "Choose a target folder for the images" ) );
    if( m_HardDiscRecordingParameters.targetDirectory_.empty() )
    {
        WriteLogMessage( wxT( "Invalid target folder selected.\n" ) );
        m_HardDiscRecordingParameters.boActive_ = false;
        return;
    }

    wxArrayString choices;
    choices.Add( wxString( wxT( "*.bmp(slowest)" ) ) );
    choices.Add( wxString( wxT( "*.png" ) ) );
    choices.Add( wxString( wxT( "*.raw(fastest)" ) ) );
    wxSingleChoiceDialog dlgFileFormat( this, wxT( "Please select the file format for the recorded images. Please note that\nrecording needs an active display right now and once the storage of a frame needs\nmore time then the acquisition of the next image, the next image will neither\nbe displayed nor stored but is skipped.\nThe application might react reeeaaally slow when in BMP or PNG mode." ), wxT( "Configure Hard Disc Recording" ), choices );
    if( dlgFileFormat.ShowModal() != wxID_OK )
    {
        WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
        m_HardDiscRecordingParameters.boActive_ = false;
        return;
    }
    WriteLogMessage( wxString::Format( wxT( "Hard disc recording enabled with file format '%s'.\n" ), dlgFileFormat.GetStringSelection().c_str() ) );
    m_HardDiscRecordingParameters.fileFormat_ = static_cast<TFileFormat>( dlgFileFormat.GetSelection() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSetupRecordSequenceSize( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    FunctionInterface* pFI = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( &pFI, 0, &pThread );

    if( pDev && pFI && pThread )
    {
        long newValue = ::wxGetNumberFromUser( wxString::Format( wxT( "Enter the new record sequence length for device %s.\n\nNOTE:\nIf this value is larger than the number of request buffers currently allocated\n(see 'SystemSettings/RequestCount' in the device property tree) each buffer will\nbe stored by the application. This might result in reduced overall system performance.\nHowever some hardware drivers due to internal limitations might not allow to create a\nbig number of requests so here a copy of each buffer might be acceptable.\n\nNOTE:\nWhen recording into RAM on a 32-bit platform up to %d MB of data can be recorded,\non 64-bit platforms recording will stop when there is less than 1 GB of free RAM.\nIn continuous recording mode the oldest image will be freed automatically\nwhen these limits apply.\n\n'0' will restore the internal default behaviour." ), ConvertedString( pDev->serial.read().c_str() ).c_str(), CaptureThread::MAX_MEMORY_FOR_RECORDING_ON_32_BIT / CaptureThread::MB ),
                                               wxT( "New record sequence length" ),
                                               wxT( "Setup Record Sequence Length" ),
                                               static_cast<long>( pThread->GetRecordSequenceSize() ),
                                               0,
                                               numeric_limits<long>::max(),
                                               this );
        if( newValue >= 0 )
        {
            pThread->SetRecordSequenceSize( newValue );
        }
        else
        {
            WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnShowIncompleteFrames( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pMISettings_Display_ShowIncompleteFrames->Check( e.IsChecked() );
    m_pLeftToolBar->ToggleTool( miSettings_Display_ShowIncompleteFrames, e.IsChecked() );
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
    if( pThread )
    {
        pThread->SetCaptureMode( e.IsChecked() );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnShowMonitorDisplay( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    m_pMonitorImage->GetDisplayArea()->SetActive( e.IsChecked() );
    m_pMonitorImage->Show( e.IsChecked() );
    ConfigureMonitorImage( e.IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSize( wxSizeEvent& e )
//-----------------------------------------------------------------------------
{
    RefreshDisplays( false );
    for( unsigned int i = 0; i < iapLAST; i++ )
    {
        if( m_ImageAnalysisPlots[i].m_pPlotCanvas )
        {
            m_ImageAnalysisPlots[i].m_pPlotCanvas->Refresh( false );
        }
    }
    e.Skip();
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnSplitterSashPosChanged( wxSplitterEvent& )
//-----------------------------------------------------------------------------
{
    RefreshDisplays( true );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnTimerCustom( wxTimerEvent& e )
//-----------------------------------------------------------------------------
{
    switch( e.GetId() )
    {
    case teUpdateDisplay:
        if( m_boNewImageDataAvailable == true )
        {
            m_boCurrentImageIsFromFile = false;
            UpdateData( false, true );
            m_boNewImageDataAvailable = false;
        }
        break;
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnToggleDisplayWindowSize( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( !m_pHorizontalSplitter->IsSplit() && !m_pVerticalSplitter->IsSplit() )
    {
        // if all hideable windows are hidden already restore them
        m_boDisplayWindowMaximized = false;
    }
    else
    {
        // else toggle current status
        m_boDisplayWindowMaximized = !m_boDisplayWindowMaximized;
    }

    m_pMISettings_PropGrid_Show->Check( !m_boDisplayWindowMaximized );
    m_pMISettings_Analysis_ShowControls->Check( !m_boDisplayWindowMaximized );
    m_pLeftToolBar->ToggleTool( miSettings_Analysis_ShowControls, !m_boDisplayWindowMaximized );
    m_pLeftToolBar->ToggleTool( miSettings_PropGrid_Show, !m_boDisplayWindowMaximized );
    SetupDisplayLogSplitter( !m_boDisplayWindowMaximized );
    SetupVerSplitter();
    const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        m_DisplayAreas[i]->RefreshScrollbars();
    }
    RefreshDisplays( true );
}

//-----------------------------------------------------------------------------
void PropViewFrame::OnWizard_Open( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    switch( m_currentWizard )
    {
    case wColorCorrection:
        Wizard_ColorCorrection();
        break;
    case wFileAccessControl:
        {
            wxArrayString choices;
            choices.Add( wxT( "Upload" ) );
            choices.Add( wxT( "Download" ) );
            const wxString operation = ::wxGetSingleChoice( wxT( "Please select which file operation you want to execute" ), wxT( "File Access Control" ), choices, this );
            if( operation == wxT( "Upload" ) )
            {
                Wizard_FileAccessControl( true );
            }
            else if( operation == wxT( "Download" ) )
            {
                Wizard_FileAccessControl( false );
            }
        }
        break;
    case wLensControl:
        Wizard_LensControl();
        break;
    case wLUTControl:
        Wizard_LUTControl();
        break;
    case wSequencerControl:
        Wizard_SequencerControl();
        break;
    case wNone:
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::OpenDriverWithTimeMeassurement( Device* pDev )
//-----------------------------------------------------------------------------
{
    const wxString deviceSerial( GetSelectedDeviceSerial() );
    m_stopWatch.Start();
    const wxString timingDetails = m_pDevPropHandler->OpenDriver( deviceSerial, this, static_cast<unsigned int>( GetDisplayCount() ) );
    WriteLogMessage( wxString::Format( wxT( "Opening device '%s' took %ld ms.\n%s" ), ConvertedString( pDev->serial.read() ).c_str(), m_stopWatch.Time(), timingDetails.c_str() ) );
}

#define CHECKED_STRING_TO_PIXELFORMAT(S) \
    if( value == wxT(#S) ) { return ibpf##S; }

//-----------------------------------------------------------------------------
TImageBufferPixelFormat PropViewFrame::PixelFormatFromString( const wxString& value )
//-----------------------------------------------------------------------------
{
    CHECKED_STRING_TO_PIXELFORMAT( Mono8 )
    CHECKED_STRING_TO_PIXELFORMAT( Mono16 )
    CHECKED_STRING_TO_PIXELFORMAT( RGBx888Packed )
    CHECKED_STRING_TO_PIXELFORMAT( YUV422Packed )
    CHECKED_STRING_TO_PIXELFORMAT( RGB888Planar )
    CHECKED_STRING_TO_PIXELFORMAT( RGBx888Planar )
    CHECKED_STRING_TO_PIXELFORMAT( Mono10 )
    CHECKED_STRING_TO_PIXELFORMAT( Mono12 )
    CHECKED_STRING_TO_PIXELFORMAT( Mono14 )
    CHECKED_STRING_TO_PIXELFORMAT( RGB888Packed )
    CHECKED_STRING_TO_PIXELFORMAT( YUV444Planar )
    CHECKED_STRING_TO_PIXELFORMAT( Mono32 )
    CHECKED_STRING_TO_PIXELFORMAT( YUV422Planar )
    CHECKED_STRING_TO_PIXELFORMAT( RGB101010Packed )
    CHECKED_STRING_TO_PIXELFORMAT( RGB121212Packed )
    CHECKED_STRING_TO_PIXELFORMAT( RGB141414Packed )
    CHECKED_STRING_TO_PIXELFORMAT( RGB161616Packed )
    CHECKED_STRING_TO_PIXELFORMAT( YUV422_UYVYPacked )
    CHECKED_STRING_TO_PIXELFORMAT( Mono12Packed_V2 )
    CHECKED_STRING_TO_PIXELFORMAT( YUV422_10Packed )
    CHECKED_STRING_TO_PIXELFORMAT( YUV422_UYVY_10Packed )
    CHECKED_STRING_TO_PIXELFORMAT( BGR888Packed )
    CHECKED_STRING_TO_PIXELFORMAT( BGR101010Packed_V2 )
    CHECKED_STRING_TO_PIXELFORMAT( YUV444_UYVPacked )
    CHECKED_STRING_TO_PIXELFORMAT( YUV444_UYV_10Packed )
    CHECKED_STRING_TO_PIXELFORMAT( YUV444Packed )
    CHECKED_STRING_TO_PIXELFORMAT( YUV444_10Packed )
    CHECKED_STRING_TO_PIXELFORMAT( Mono12Packed_V1 )
    CHECKED_STRING_TO_PIXELFORMAT( YUV411_UYYVYY_Packed )

    WriteErrorMessage( wxString::Format( wxT( "Unsupported pixel format(%s).\n" ), value.c_str() ) );
    return ibpfMono8;
}

//-----------------------------------------------------------------------------
void PropViewFrame::ReEnableSingleCapture( void )
//-----------------------------------------------------------------------------
{
    if( m_boSingleCaptureInProgess )
    {
        m_boSingleCaptureInProgess = false;
        CaptureThread* pThread = 0;
        const Device* pDev = m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
        bool boDevOpen = false;
        bool boDevPresent = false;
        bool boDevLive = false;
        if( pDev )
        {
            boDevOpen = pDev->isOpen();
            boDevPresent = ( pDev->state.read() == dsPresent );
            if( pThread )
            {
                boDevLive = pThread->GetLiveMode();
            }
        }
        SetupAcquisitionControls( boDevOpen, boDevPresent, boDevLive );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::RefreshCurrentPixelData( void )
//-----------------------------------------------------------------------------
{
    if( ( m_pStatusBar->GetFieldsCount() != NO_OF_STATUSBAR_FIELDS ) || m_boShutdownInProgress || m_boImageCanvasInFullScreenMode )
    {
        return;
    }

    m_pStatusBar->SetStatusText( m_pLastMouseHooverDisplay->GetCurrentPixelDataAsString(), sbfPixelData );
}

//-----------------------------------------------------------------------------
void PropViewFrame::RefreshDisplays( bool boEraseBackground )
//-----------------------------------------------------------------------------
{
    const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        m_DisplayAreas[i]->Refresh( boEraseBackground );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::RefreshPendingImageQueueDepthForCurrentDevice( void )
//-----------------------------------------------------------------------------
{
    if( m_pDevPropHandler )
    {
        CaptureThread* pThread = 0;
        m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
        if( pThread )
        {
            pThread->SetPendingImageQueueDepth( GetDisplayCount() );
        }
    }
}

//-----------------------------------------------------------------------------
template<class _Ty>
void PropViewFrame::RegisterAnalysisPlot( int index, wxWindowID id, wxWindowID gridID, const wxColour& AOIColour )
//-----------------------------------------------------------------------------
{
    m_ImageAnalysisPlots[index].m_pPlotCanvas = new _Ty( m_ImageAnalysisPlots[index].m_pNBDisplayMethod, id, wxDefaultPosition, wxDefaultSize );
    m_ImageAnalysisPlots[index].m_pPlotCanvas->SetAOIColour( AOIColour );
    m_ImageAnalysisPlots[index].m_pPlotCanvas->CreateGrid( m_ImageAnalysisPlots[index].m_pNBDisplayMethod, gridID );
    m_ImageAnalysisPlots[index].m_pNBDisplayMethod->AddPage( m_ImageAnalysisPlots[index].m_pPlotCanvas->GetGrid(), wxT( "Numerical" ) );
}

//-----------------------------------------------------------------------------
wxRect PropViewFrame::RestoreConfiguration( const unsigned int displayCount, bool& boMaximized )
//-----------------------------------------------------------------------------
{
    // restore previous state
    wxRect rect = FramePositionStorage::Load( wxRect( 0, 0, 1024, 700 ), boMaximized );
    wxConfigBase* pConfig = wxConfigBase::Get();
    m_VerticalSplitterPos = pConfig->Read( wxT( "/MainFrame/verticalSplitter" ), -1l );
    m_HorizontalSplitterPos = pConfig->Read( wxT( "/MainFrame/horizontalSplitter" ), -1l );

    for( unsigned int i = 0; i < displayCount; i++ )
    {
        ostringstream oss;
        oss << "/MainFrame/Settings/Display/" << i << "/";
        wxString displayToken( ConvertedString( oss.str() ) );
        m_DisplayAreas[i]->SetScaling( pConfig->Read( displayToken + wxString( wxT( "FitToScreen" ) ), 0l ) != 0 );
        if( m_DisplayAreas[i]->SupportsDifferentScalingModes() )
        {
            m_DisplayAreas[i]->SetScalingMode( static_cast<ImageCanvas::TScalingMode>( pConfig->Read( displayToken + wxString( wxT( "ScalingMode" ) ), 0l ) ) );
        }
        m_DisplayAreas[i]->SetPerformanceWarningOutput( pConfig->Read( displayToken + wxString( wxT( "ShowPerformanceWarnings" ) ), 0l ) != 0 );
        m_DisplayAreas[i]->SetImageModificationWarningOutput( pConfig->Read( displayToken + wxString( wxT( "ShowImageModificationWarning" ) ), 1l ) != 0 );
        m_DisplayAreas[i]->SetInfoOverlayMode( pConfig->Read( displayToken + wxString( wxT( "ShowRequestInfos" ) ), 0l ) != 0 );
    }
    bool boActive = pConfig->Read( wxT( "/MainFrame/Settings/monitorDisplay" ), 0l ) != 0;
    m_pMISettings_Display_ShowMonitorImage->Check( boActive );
    m_pLeftToolBar->ToggleTool( miSettings_Display_ShowMonitorImage, boActive );
    bool boLowerRightControlsActive = pConfig->Read( wxT( "/MainFrame/Settings/displayLogWindow" ), 0l ) != 0;
    bool boDisplayActive = pConfig->Read( wxT( "/MainFrame/Settings/displayActive" ), 1l ) != 0;
    // either the display or the controls MUST be active!
    m_pMISettings_Analysis_ShowControls->Check( boLowerRightControlsActive );
    m_pLeftToolBar->ToggleTool( miSettings_Analysis_ShowControls, boLowerRightControlsActive );
    m_pMISettings_Display_Active->Check( boDisplayActive );
    m_pLeftToolBar->ToggleTool( miSettings_Display_Active, boDisplayActive );
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        m_DisplayAreas[i]->SetActive( boDisplayActive );
    }
    SetupDisplayLogSplitter( true ); // needed as otherwise the initial view might not be correct when display and propGrid are switched off!!!
    SetupDisplayLogSplitter();
    m_boFindFeatureMatchCaseActive = pConfig->Read( wxT( "/MainFrame/Settings/findFeatureMatchCaseActive" ), 0l ) != 0;
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/displayIncompleteFrames" ), 0l ) != 0;
    m_pMISettings_Display_ShowIncompleteFrames->Check( boActive );
    m_pLeftToolBar->ToggleTool( miSettings_Display_ShowIncompleteFrames, boActive );
    m_displayUpdateFrequency = pConfig->Read( wxT( "/MainFrame/Settings/displayUpdateFrequency" ), m_displayUpdateFrequency );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/silentRecording" ), 0l ) != 0;
    m_pMICapture_Recording_SlientMode->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/continuousRecording" ), 0l ) != 0;
    m_pMICapture_Recording_Continuous->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/activeDevicesOnly" ), 0l ) != 0;
    m_pMIAction_DisplayConnectedDevicesOnly->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/synchroniseAOIs" ), 0l ) != 0;
    m_pMISettings_Analysis_SynchroniseAOIs->Check( boActive );
    m_pLeftToolBar->ToggleTool( miSettings_Analysis_SynchronizeAOIs, boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/detailedInfosOnCallback" ), 0l ) != 0;
    m_pMISettings_Analysis_DetailedInfoOnCallback->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/displayPropGrid" ), 0l ) != 0;
    m_pMISettings_PropGrid_Show->Check( boActive );
    m_pLeftToolBar->ToggleTool( miSettings_PropGrid_Show, boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/showToolBar" ), 1l ) != 0;
    m_pMISettings_ShowUpperToolBar->Check( boActive );
    ConfigureToolBar( m_pUpperToolBar, boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/allowFastSingleFrameAcquisition" ), 0l ) != 0;
    m_pMISettings_AllowFastSingleFrameAcquisition->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/showLeftToolBar" ), 0l ) != 0;
    m_pMISettings_ShowLeftToolBar->Check( boActive );
    ConfigureToolBar( m_pLeftToolBar, boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/showStatusBar" ), 1l ) != 0;
    m_pMISettings_ShowStatusBar->Check( boActive );
    ConfigureStatusBar( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/warnOnOutdatedFirmware" ), 1l ) != 0;
    m_pMISettings_WarnOnOutdatedFirmware->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/warnOnReducedDriverPerformance" ), 1l ) != 0;
    m_pMISettings_WarnOnReducedDriverPerformance->Check( boActive );
#if defined(linux) || defined(__linux) || defined(__linux__)
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/warnOnPotentialBufferIssues" ), 1l ) != 0;
    m_pMISettings_WarnOnPotentialBufferIssues->Check( boActive );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/warnOnPotentialFirewallIssues" ), 1l ) != 0;
    m_pMISettings_WarnOnPotentialFirewallIssues->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/propgrid_showMethodExecutionErrors" ), 1l ) != 0;
    m_pMISettings_PropGrid_ShowMethodExecutionErrors->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/propgrid_showFeatureChangeTimeConsumption" ), 0l ) != 0;
    m_pMISettings_PropGrid_ShowFeatureChangeTimeConsumption->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/automaticallyReconnectToUnusedDevices" ), 1l ) != 0;
    m_pMIAction_AutomaticallyReconnectToUnusedDevices->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/displayPropsWithHexIndices" ), 0l ) != 0;
    m_pMISettings_PropGrid_DisplayPropIndicesAsHex->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/propgrid_displayToolTips" ), 1l ) != 0;
    m_pMISettings_PropGrid_DisplayToolTips->Check( boActive );
    ConfigureToolTipsForPropertyGrids( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/useDisplayNameIfAvailable" ), 1l ) != 0;
    m_pMISettings_PropGrid_UseDisplayNameIfAvailable->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/createEditorsWithSlider" ), 1l ) != 0;
    m_pMISettings_PropGrid_CreateEditorsWithSlider->Check( boActive );
    boActive = pConfig->Read( wxT( "/MainFrame/Settings/useSelectorGrouping" ), 1l ) != 0;
    m_pMISettings_PropGrid_UseSelectorGrouping->Check( boActive );
    m_pLeftToolBar->ToggleTool( miSettings_PropGrid_UseSelectorGrouping, boActive );
    int selectedPage = pConfig->Read( wxT( "/MainFrame/selectedPage" ), -1l );
    if( selectedPage != -1 )
    {
        m_pLowerRightWindow->SetSelection( selectedPage );
        m_CurrentImageAnalysisControlIndex = GetAnalysisControlIndex();
        if( m_CurrentImageAnalysisControlIndex >= 0 )
        {
            m_pCurrentAnalysisDisplay->SetActiveAnalysisPlot( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas );
            m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->SetActive( true );
        }
    }

    // previous view mode
    m_ViewMode = static_cast<DevicePropertyHandler::TViewMode>( pConfig->Read( wxT( "/MainFrame/Settings/propGridViewMode" ), DevicePropertyHandler::vmStandard ) );
    if( m_ViewMode >= DevicePropertyHandler::vmLAST )
    {
        m_ViewMode = DevicePropertyHandler::vmStandard; // for backward compatibility
    }

    m_defaultDeviceInterfaceLayout = pConfig->Read( wxT( "/MainFrame/Settings/defaultDeviceInterfaceLayoutString" ), wxT( "Unknown" ) );
    if( m_defaultDeviceInterfaceLayout != wxT( "DeviceSpecific" ) )
    {
        m_defaultDeviceInterfaceLayout = wxT( "GenICam" );
    }
    m_pAcquisitionModeCombo->SetValue( m_ContinuousStr );

    if( GlobalDataStorage::Instance()->IsComponentVisibilitySupported() )
    {
        wxString userExperience( pConfig->Read( wxT( "/MainFrame/Settings/UserExperience" ), wxString( ConvertedString( Component::visibilityAsString( cvBeginner ) ) ) ) );
        m_pUserExperienceCombo->SetValue( userExperience );
        UpdateUserExperience( userExperience );
    }
    m_defaultImageProcessingMode = static_cast<TImageProcessingMode>( pConfig->Read( wxT( "/MainFrame/Settings/Capture_DefaultImageProcessingMode" ), ipmProcessLatestOnly ) );
    switch( m_defaultImageProcessingMode )
    {
    case ipmDefault:
        m_pMICapture_DefaultImageProcessingMode_ProcessAll->Check();
        break;
    case ipmProcessLatestOnly:
        m_pMICapture_DefaultImageProcessingMode_ProcessLatestOnly->Check();
        break;
    }
    const wxString captureSettings_UsageMode( pConfig->Read( wxT( "/MainFrame/Settings/CaptureSettings_UsageMode" ), wxString( wxT( "Manual" ) ) ) );
    if( captureSettings_UsageMode == wxT( "Manual" ) )
    {
        m_pMICapture_CaptureSettings_UsageMode_Manual->Check();
    }
    else if( captureSettings_UsageMode == wxT( "Automatic" ) )
    {
        m_pMICapture_CaptureSettings_UsageMode_Automatic->Check();
    }

    for( unsigned int i = 0; i < iapLAST; i++ )
    {
        m_ImageAnalysisPlots[i].Load( pConfig );
    }

    // info plot
    const bool boInfoPlotActive = pConfig->Read( wxT( "/Controls/InfoPlot/Enable" ), 0l ) != 0;
    m_pCBEnableInfoPlot->SetValue( boInfoPlotActive );
    m_pInfoPlotArea->SetActive( boInfoPlotActive );
    m_pSCInfoPlotHistoryDepth->SetValue( pConfig->Read( wxT( "/Controls/InfoPlot/HistoryDepth" ), 20l ) );
    m_pInfoPlotArea->SetHistoryDepth( m_pSCInfoPlotHistoryDepth->GetValue() );
    m_pSCInfoPlotUpdateSpeed->SetValue( pConfig->Read( wxT( "/Controls/InfoPlot/UpdateSpeed" ), 3l ) );
    m_pInfoPlotArea->SetUpdateFrequency( m_pSCInfoPlotUpdateSpeed->GetValue() );
    const bool boInfoPlot_PlotDifferences = pConfig->Read( wxT( "/Controls/InfoPlot/PlotDifferences" ), 0l ) != 0;
    m_pCBInfoPlotDifferences->SetValue( boInfoPlot_PlotDifferences );
    m_pInfoPlotArea->SetPlotDifferences( boInfoPlot_PlotDifferences );
    const bool boInfoPlot_AutoScale = pConfig->Read( wxT( "/Controls/InfoPlot/AutoScale" ), 0l ) != 0;
    m_pCBInfoPlotAutoScale->SetValue( boInfoPlot_AutoScale );
    m_pInfoPlotArea->SetAutoScale( boInfoPlot_AutoScale );

    // feature vs. time plot
    const bool boFeatureValueVsTimePlotActive = pConfig->Read( wxT( "/Controls/FeatureValueVsTimePlot/Enable" ), 0l ) != 0;
    m_pCBEnableFeatureValueVsTimePlot->SetValue( boFeatureValueVsTimePlotActive );
    m_pFeatureValueVsTimePlotArea->SetActive( boFeatureValueVsTimePlotActive );
    m_pSCFeatureValueVsTimePlotHistoryDepth->SetValue( pConfig->Read( wxT( "/Controls/FeatureValueVsTimePlot/HistoryDepth" ), 20l ) );
    m_pFeatureValueVsTimePlotArea->SetHistoryDepth( m_pSCFeatureValueVsTimePlotHistoryDepth->GetValue() );
    m_pSCFeatureValueVsTimePlotUpdateSpeed->SetValue( pConfig->Read( wxT( "/Controls/FeatureValueVsTimePlot/UpdateSpeed" ), 1000l ) );
    m_pFeatureValueVsTimePlotArea->SetUpdateFrequency( m_pSCFeatureValueVsTimePlotUpdateSpeed->GetValue() );
    const bool boFeatureValueVsTimePlot_PlotDifferences = pConfig->Read( wxT( "/Controls/FeatureValueVsTimePlot/PlotDifferences" ), 0l ) != 0;
    m_pCBFeatureValueVsTimePlotDifferences->SetValue( boFeatureValueVsTimePlot_PlotDifferences );
    m_pFeatureValueVsTimePlotArea->SetPlotDifferences( boFeatureValueVsTimePlot_PlotDifferences );
    const bool boFeatureValueVsTimePlot_AutoScale = pConfig->Read( wxT( "/Controls/FeatureValueVsTimePlot/AutoScale" ), 0l ) != 0;
    m_pCBFeatureValueVsTimePlotAutoScale->SetValue( boFeatureValueVsTimePlot_AutoScale );
    m_pInfoPlotArea->SetAutoScale( boFeatureValueVsTimePlot_AutoScale );
    SetupVerSplitter();

    // wizards
    m_pMISettings_ShowQuickSetupOnUse->Check( pConfig->Read( wxT( "/Wizards/QuickSetup/Show" ), 1l ) != 0 );
    m_boShowQuickSetupWizardCurrentProcess = m_pMISettings_ShowQuickSetupOnUse->IsChecked();

    // Command-line argument for QuickSetupWizard should override registry settings without changing them
    if( m_QuickSetupWizardEnforce == qswiForceShow )
    {
        m_boShowQuickSetupWizardCurrentProcess = true;
    }
    else if( m_QuickSetupWizardEnforce == qswiForceHide )
    {
        m_boShowQuickSetupWizardCurrentProcess = false;
        // If it is the very first time and the QSW has never been shown, we should make the GUI
        // richer by showing the left tool bar and the property grid.
        SetupGUIOnFirstRun();
    }

    return rect;
}

//-----------------------------------------------------------------------------
wxImage PropViewFrame::PNGToIconImage( const void* pData, const size_t dataSize ) const
//-----------------------------------------------------------------------------
{
    wxImage image( wxBitmap::NewFromPNGData( pData, dataSize ).ConvertToImage() );
    return m_UseSmallIcons ? image.Scale( 16, 16 ) : image;
}

//-----------------------------------------------------------------------------
void PropViewFrame::PostQuickSetupWizardSettings( void )
//-----------------------------------------------------------------------------
{
    m_boShowQuickSetupWizardCurrentProcess = m_pMISettings_ShowQuickSetupOnUse->IsChecked();
    m_DisplayAreas[0]->DisableDoubleClickAndPrunePopupMenu( false );
    m_pUpperToolBar->ToggleTool( miWizards_QuickSetup, false );
}

//-----------------------------------------------------------------------------
void PropViewFrame::RestoreGUIStateAfterQuickSetupWizard( bool boFirstRun /*=false*/ )
//-----------------------------------------------------------------------------
{
    PostQuickSetupWizardSettings();
    if( boFirstRun )
    {
        // On the very first OK-press of the QuickSetup Wizard, the left ToolBar must appear, regardless of its' previous state
        ConfigureToolBar( m_pLeftToolBar, true );
        ConfigureToolBar( m_pUpperToolBar, true );
        m_pMISettings_PropGrid_Show->Check( true );
        m_pMISettings_ShowLeftToolBar->Check();
    }
    else
    {
        ConfigureToolBar( m_pLeftToolBar, m_GUIBeforeQuickSetupWizard.boLeftToolBarShown_ );
        ConfigureToolBar( m_pUpperToolBar, m_GUIBeforeQuickSetupWizard.boUpperToolbarShown_ );
        m_pMISettings_PropGrid_Show->Check( m_GUIBeforeQuickSetupWizard.boPropertyGridShown_ );
    }

    SetMenuBar( m_pMenuBar );
    m_pVerticalSplitter->SetSashPosition( m_GUIBeforeQuickSetupWizard.verticalSplitterPosition_ );
    m_pHorizontalSplitter->SetSashPosition( m_GUIBeforeQuickSetupWizard.horizontalSplitterPosition_ );
    m_pMISettings_Analysis_ShowControls->Check( m_GUIBeforeQuickSetupWizard.boAnalysisTabsShown_ );
    m_pLeftToolBar->ToggleTool( miSettings_Analysis_ShowControls, m_GUIBeforeQuickSetupWizard.boAnalysisTabsShown_ );
    m_pLeftToolBar->ToggleTool( miSettings_PropGrid_Show, m_GUIBeforeQuickSetupWizard.boPropertyGridShown_ );
    m_pPanel->GetSizer()->Layout();
    SetupVerSplitter();
    SetupDisplayLogSplitter();
    m_DisplayAreas[0]->SetScaling( boFirstRun ? false : m_GUIBeforeQuickSetupWizard.boFitToCanvas_ );
    m_DisplayAreas[0]->SetActive( boFirstRun ? true : m_GUIBeforeQuickSetupWizard.boDisplayShown_ );
    RefreshDisplays( m_GUIBeforeQuickSetupWizard.boPropertyGridShown_ );
}

//-----------------------------------------------------------------------------
void PropViewFrame::SaveGUIStateBeforeQuickSetupWizard( void )
//-----------------------------------------------------------------------------
{
    m_GUIBeforeQuickSetupWizard.boLeftToolBarShown_ = m_pLeftToolBar->IsShown();
    m_GUIBeforeQuickSetupWizard.boPropertyGridShown_ = m_pSettingsPanel->IsShown();
    m_GUIBeforeQuickSetupWizard.boUpperToolbarShown_ = m_pUpperToolBar->IsShown();
    m_GUIBeforeQuickSetupWizard.boStatusBarShown_ = m_pStatusBar->IsShown();
    m_GUIBeforeQuickSetupWizard.boAnalysisTabsShown_ = m_pMISettings_Analysis_ShowControls->IsChecked();

    // Canvas scaling data will be saved only for one display...
    m_GUIBeforeQuickSetupWizard.boFitToCanvas_ =  m_DisplayAreas[0]->IsScaled();
    m_GUIBeforeQuickSetupWizard.boDisplayShown_ =  m_DisplayAreas[0]->IsActive();
    m_GUIBeforeQuickSetupWizard.verticalSplitterPosition_ = m_pVerticalSplitter->GetSashPosition();
    m_GUIBeforeQuickSetupWizard.horizontalSplitterPosition_ = m_pHorizontalSplitter->GetSashPosition();
}

//-----------------------------------------------------------------------------
int PropViewFrame::SaveDeviceSetting( FunctionInterface* pFI, const string& name, const TStorageFlag flags, const TScope scope )
//-----------------------------------------------------------------------------
{
    wxBusyCursor busyCursorScope;
    m_stopWatch.Start();
    const int result = pFI->saveSetting( name, flags, scope );
    WriteLogMessage( wxString::Format( wxT( "Saving a setting for device '%s' took %ld ms.\n" ), GetSelectedDeviceSerial().c_str(), m_stopWatch.Time() ) );
    return result;
}

//-----------------------------------------------------------------------------
void PropViewFrame::SaveImage( const wxString& filenameAndPath, TFileFormat fileFilterIndex )
//-----------------------------------------------------------------------------
{
    wxString extension;
    switch( fileFilterIndex )
    {
    case 0:
        extension = wxT( "bmp" );
        break;
    case 1:
        extension = wxT( "png" );
        break;
    case 2:
    default:
        extension = wxT( "raw" );
        break;
    }

    if( extension == wxT( "raw" ) )
    {
        wxFileName tmpFilename( filenameAndPath );
        const wxString fileNameSuffix( wxString::Format( wxT( ".%dx%d.%s" ), m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iWidth, m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iHeight, m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].pixelFormat_.c_str() ) );
        wxString name( tmpFilename.GetName() );
        if( !name.EndsWith( fileNameSuffix ) )
        {
            tmpFilename.SetName( wxString::Format( wxT( "%s%s" ), name.c_str(), fileNameSuffix.c_str() ) );
        }
        tmpFilename.SetExt( extension );
        WriteFile( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->vpData, m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iSize, tmpFilename.GetFullPath(), m_pLogWindow );
    }
    else
    {
        wxFileName tmp( filenameAndPath );
        tmp.SetExt( extension );
        ImageCanvas::TSaveResult result = m_pCurrentAnalysisDisplay->SaveCurrentImage( tmp.GetFullPath(), extension );
        switch( result )
        {
        case ImageCanvas::srOK:
            WriteLogMessage( wxString::Format( wxT( "Storing of %s was successful.\n" ), tmp.GetFullPath().c_str() ) );
            break;
        case ImageCanvas::srNoImage:
            WriteErrorMessage( wxT( "There is no valid image to store!\n" ) );
            break;
        case ImageCanvas::srFailedToSave:
            WriteErrorMessage( wxString::Format( wxT( "Storing of %s failed.\n" ), tmp.GetFullPath().c_str() ) );
            break;
        default:
            WriteLogMessage( wxString::Format( wxT( "Storing of %s return unexpected value: %d.\n" ), tmp.GetFullPath().c_str(), result ) );
            break;
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::SearchAndSelectImageCanvas( ImageCanvas* pImageCanvas )
//-----------------------------------------------------------------------------
{
    int index = 0;
    const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        if( pImageCanvas == m_DisplayAreas[i] )
        {
            index = static_cast<int>( i );
            break;
        }
    }
    SelectImageCanvas( index );
}

//-----------------------------------------------------------------------------
void PropViewFrame::SelectImageCanvas( int index )
//-----------------------------------------------------------------------------
{
    if( m_pCurrentAnalysisDisplay != m_DisplayAreas[index] )
    {
        m_CurrentRequestDataIndex = index;
        m_pCurrentAnalysisDisplay->RegisterMonitorDisplay( 0 );
        PlotCanvasImageAnalysis* p = DeselectAnalysisPlot();
        if( p )
        {
            p->RegisterImageCanvas( 0 );
        }
        m_pCurrentAnalysisDisplay->Refresh();
        m_pCurrentAnalysisDisplay = m_DisplayAreas[index];
        m_pCurrentAnalysisDisplay->RegisterMonitorDisplay( m_pMISettings_Display_ShowMonitorImage->IsChecked() ? m_pMonitorImage->GetDisplayArea() : 0 );
        m_pMonitorImage->GetDisplayArea()->SetImage( m_pCurrentAnalysisDisplay->GetImage() );
        if( p && m_pMISettings_Analysis_SynchroniseAOIs->IsChecked() )
        {
            wxRect activeAOI = p->GetAOIRect();
            ConfigureAnalysisPlot( &activeAOI );
        }
        else
        {
            ConfigureAnalysisPlot();
        }
    }
}

//-----------------------------------------------------------------------------
bool PropViewFrame::SetCurrentImage( const wxFileName& fileName, ImageCanvas* pImageCanvas )
//-----------------------------------------------------------------------------
{
    if( fileName.GetFullPath().IsEmpty() )
    {
        return false;
    }

    if( fileName.GetExt().CmpNoCase( wxT( "raw" ) ) == 0 )
    {
        wxFile file( fileName.GetFullPath().c_str(), wxFile::read );
        if( !file.IsOpened() )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to open file '%s'.\n" ), fileName.GetFullPath().c_str() ) );
            return false;
        }

        RawImageImportDlg dlg( this, wxT( "Raw Image Import" ), fileName );
        if( dlg.ShowModal() != wxID_OK )
        {
            WriteLogMessage( wxString::Format( wxT( "Import of file '%s' canceled.\n" ), fileName.GetFullPath().c_str() ) );
            return false;
        }

        if( dlg.GetFormat().IsEmpty() || dlg.GetBayerParity().IsEmpty() )
        {
            WriteErrorMessage( wxString::Format( wxT( "Invalid pixel format selection while importing file '%s'.\n" ), fileName.GetFullPath().c_str() ) );
            return false;
        }

        if( ( dlg.GetWidth() < 1 ) || ( dlg.GetHeight() < 1 ) )
        {
            WriteErrorMessage( wxString::Format( wxT( "Invalid image dimensions(%ldx%ld) selected while importing file '%s'.\n" ), dlg.GetWidth(), dlg.GetHeight(), fileName.GetFullPath().c_str() ) );
            return false;
        }

        SearchAndSelectImageCanvas( pImageCanvas );
        m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_ = mvIMPACT::acquire::ImageBufferDesc( PixelFormatFromString( dlg.GetPixelFormat() ) , dlg.GetWidth(), dlg.GetHeight() );
        m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].pixelFormat_ = dlg.GetFormat();
        m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].bayerParity_ = BayerParityFromString( dlg.GetBayerParity() );
        const size_t imageSize = m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iSize;
        const size_t toRead = ( static_cast<size_t>( file.Length() ) < imageSize ) ? static_cast<size_t>( file.Length() ) : imageSize;
        file.Read( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->vpData, toRead );
    }
    else
    {
        wxImage image;
        bool boLoadedSuccessfully = image.LoadFile( fileName.GetFullPath() );
        if( !boLoadedSuccessfully )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to load file %s.\n" ), fileName.GetFullPath().c_str() ) );
            return false;
        }

        SearchAndSelectImageCanvas( pImageCanvas );
        m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_ = mvIMPACT::acquire::ImageBufferDesc( ibpfRGB888Packed, image.GetWidth(), image.GetHeight() );
        m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].pixelFormat_ = wxString( wxT( "RGB888Packed" ) );
        m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].bayerParity_ = bmpUndefined;
        unsigned char* pDst = static_cast<unsigned char*>( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->vpData );
        const int W = image.GetWidth();
        const int H = image.GetHeight();
        for( int y = 0; y < H; y++ )
        {
            for( int x = 0; x < W; x++ )
            {
                *pDst++ = image.GetBlue( x, y );
                *pDst++ = image.GetGreen( x, y );
                *pDst++ = image.GetRed( x, y );
            }
        }
    }
    WriteLogMessage( wxString::Format( wxT( "file %s successfully imported.\n" ), fileName.GetFullPath().c_str() ) );
    m_boCurrentImageIsFromFile = true;
    UpdateData();
    return true;
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetCommonToolBarProperties( wxToolBar* pToolBar )
//-----------------------------------------------------------------------------
{
    pToolBar->SetMargins( 5, 5 );
    pToolBar->SetToolBitmapSize( m_UseSmallIcons ? wxSize( 16, 16 ) : wxSize( 32, 32 ) );
    pToolBar->SetToolSeparation( 10 );
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupGUIOnFirstRun( void )
//-----------------------------------------------------------------------------
{
    wxConfigBase* pConfig = wxConfigBase::Get();
    if( !pConfig->Exists( wxT( "/Wizards/QuickSetup/Show" ) ) )
    {
        RestoreGUIStateAfterQuickSetupWizard( true );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupAcquisitionControls( const bool boDevOpen, const bool boDevPresent, const bool boDevLive )
//-----------------------------------------------------------------------------
{
    m_pMICapture_Acquire->Enable( boDevOpen && boDevPresent && ( m_pMISettings_AllowFastSingleFrameAcquisition->IsChecked() || !m_boSingleCaptureInProgess ) );
    m_pMICapture_Acquire->Check( boDevLive );
    if( m_pUpperToolBar )
    {
        m_pUpperToolBar->ToggleTool( miCapture_Acquire, boDevLive );
        m_pUpperToolBar->EnableTool( miCapture_Acquire, m_pMICapture_Acquire->IsEnabled() );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupCaptureSettingsUsageMode( TCaptureSettingUsageMode mode )
//-----------------------------------------------------------------------------
{
    CaptureThread* pCT = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pCT );
    if( pCT )
    {
        pCT->SetCaptureSettingUsageMode( mode );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupDisplayLogSplitter( bool boForceSplit /* = false */ )
//-----------------------------------------------------------------------------
{
    if( ( m_pMISettings_Display_Active->IsChecked() && m_pMISettings_Analysis_ShowControls->IsChecked() ) || boForceSplit )
    {
        m_pHorizontalSplitter->SplitHorizontally( m_pDisplayPanel, m_pLowerRightWindow, 0 );
        m_pHorizontalSplitter->SetSashPosition( m_HorizontalSplitterPos );
        ConfigureAnalysisPlot();
    }
    else
    {
        m_HorizontalSplitterPos = m_pHorizontalSplitter->GetSashPosition();
        if( m_pMISettings_Analysis_ShowControls->IsChecked() )
        {
            ConfigureSplitter( m_pHorizontalSplitter, m_pDisplayPanel, m_pLowerRightWindow );
            ConfigureAnalysisPlot();
        }
        else
        {
            ConfigureSplitter( m_pHorizontalSplitter, m_pLowerRightWindow, m_pDisplayPanel );
            DeselectAnalysisPlot();
        }
        SetupVerSplitter();
    }
    UpdateData();
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupDlgControls( void )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    const Device* pDev = m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );
    bool boDevOpen = false;
    bool boDevPresent = false;
    bool boDevLive = false;
    bool boSequenceBegin = true;
    bool boSequenceEnd = true;
    bool boDevRecord = false;
    bool boAllowDefaultLoadSave = true;
    TDeviceInterfaceLayout interfaceLayout = dilDeviceSpecific;
    RequestContainer::size_type recordedSequenceLength = 0;
    if( pDev )
    {
        boDevOpen = pDev->isOpen();
        boDevPresent = ( pDev->state.read() == dsPresent );
        if( pThread )
        {
            boDevRecord = pThread->GetRecordMode();
            boDevLive = pThread->GetLiveMode();
            pThread->SetCaptureMode( m_pMISettings_Display_ShowIncompleteFrames->IsChecked() );
            recordedSequenceLength = pThread->GetRecordedSequenceSize();
            if( recordedSequenceLength > 1 )
            {
                boSequenceBegin = pThread->GetCurrentSequenceIndex() == 0;
                boSequenceEnd = pThread->GetCurrentSequenceIndex() == recordedSequenceLength - 1;
            }
        }
        if( pDev->interfaceLayout.isValid() )
        {
            interfaceLayout = pDev->interfaceLayout.read();
        }
        boAllowDefaultLoadSave = interfaceLayout == dilDeviceSpecific;
    }
    const bool boMultipleCaptureSettingsSupported = interfaceLayout == dilDeviceSpecific;

    const DeviceManager& devMgr = m_pDevPropHandler->GetDevMgr();
    if( m_DeviceListChangedCounter != devMgr.changedCount() )
    {
        UpdateDeviceComboBox();
    }

    if( m_DeviceCount != devMgr.deviceCount() )
    {
        UpdateDeviceInterfaceLayouts();
        UpdateUserControlledImageProcessingEnableProperties();
    }

    m_pInfoPlotSelectionCombo->Enable( boDevOpen );
    bool isDeviceSelected = m_pDevCombo->GetValue() != m_NoDevStr;
    m_pMIAction_Use->Enable( isDeviceSelected );
    m_pMIAction_Use->Check( boDevOpen );
    m_pMICapture_Record->Check( boDevRecord );

    m_pMIAction_CaptureSettings_Save_ToDefault->Enable( boDevOpen && boAllowDefaultLoadSave );
    m_pMIAction_CaptureSettings_Save_CurrentProduct->Enable( boDevOpen );
    m_pMIAction_CaptureSettings_Save_ActiveDevice->Enable( boDevOpen );
    m_pMIAction_CaptureSettings_Save_ExportActiveDevice->Enable( boDevOpen );
    m_pMIAction_CaptureSettings_Load_FromDefault->Enable( boDevOpen && boAllowDefaultLoadSave );
    m_pMIAction_CaptureSettings_Load_CurrentProduct->Enable( boDevOpen );
    m_pMIAction_CaptureSettings_Load_ActiveDevice->Enable( boDevOpen );
    m_pMIAction_CaptureSettings_Load_ActiveDeviceFromFile->Enable( boDevOpen );
    m_pMIAction_CaptureSettings_Manage->Enable( isDeviceSelected && pDev && pDev->autoLoadSettingOrder.isValid() );
    m_pMIAction_LoadImage->Enable( !boDevLive );
    const bool boOpen_NotLive = boDevOpen && !boDevLive;
    m_pMIAction_SaveImage->Enable( boOpen_NotLive );
    m_pMIAction_SaveImageSequenceToFiles->Enable( boOpen_NotLive && ( !boSequenceEnd || !boSequenceBegin ) );
    m_pMIAction_SaveImageSequenceToStream->Enable( boOpen_NotLive && ( !boSequenceEnd || !boSequenceBegin ) );
    SetupAcquisitionControls( boDevOpen, boDevPresent, boDevLive );
    const bool boOpen_Present_NotLive = boDevOpen && boDevPresent && !boDevLive;
    bool boAcquisitionModeWriteable = boOpen_Present_NotLive;
    if( boOpen_Present_NotLive )
    {
        Property acquisitionMode( m_pDevPropHandler->GetActiveDeviceAcquisitionMode() );
        if( !acquisitionMode.isValid() )
        {
            boAcquisitionModeWriteable = true;
        }
        else if( acquisitionMode.isWriteable() )
        {
            boAcquisitionModeWriteable = true;
        }
        else
        {
            boAcquisitionModeWriteable = false;
        }
    }
    m_pMIWizards_QuickSetup->Enable( boDevOpen && boDevPresent );
    m_pAcquisitionModeCombo->Enable( boAcquisitionModeWriteable );
    m_pMICapture_Abort->Enable( boDevOpen && boDevPresent );
    m_pMICapture_Unlock->Enable( boOpen_Present_NotLive );
    m_pMICapture_Record->Enable( boOpen_Present_NotLive );
    m_pMICapture_Forward->Enable( boOpen_Present_NotLive && !boSequenceEnd );
    m_pMICapture_Backward->Enable( boOpen_Present_NotLive && !boSequenceBegin );
    m_pMICapture_SetupCaptureQueueDepth->Enable( boOpen_NotLive );
    m_pMICapture_DetailedRequestInformation->Enable( boOpen_NotLive );
    m_pMICapture_Recording_Continuous->Enable( boDevOpen );
    m_pMICapture_Recording_SlientMode->Enable( boOpen_NotLive );
    m_pMICapture_Recording_SetupSequenceSize->Enable( boOpen_NotLive );
    m_pRecordDisplaySlider->Enable( boOpen_Present_NotLive && ( recordedSequenceLength > 0 ) );
    if( m_pRecordDisplaySlider->IsEnabled() )
    {
        if( static_cast<size_t>( m_pRecordDisplaySlider->GetMax() ) != recordedSequenceLength - 1 )
        {
            m_pRecordDisplaySlider->SetRange( 0, static_cast<int>( recordedSequenceLength - 1 ) );
        }
    }
    else
    {
        m_pRecordDisplaySlider->SetRange( 0, 0 );
    }

    if( m_pUpperToolBar )
    {
        m_pUpperToolBar->EnableTool( miAction_UseDevice, isDeviceSelected );
        m_pUpperToolBar->ToggleTool( miAction_UseDevice, boDevOpen );
        m_pUpperToolBar->EnableTool( miWizards_QuickSetup, m_pDevPropHandler->DoesActiveDeviceSupportWizard( wQuickSetup ) && boDevOpen && boDevPresent );
        m_pUpperToolBar->EnableTool( miCapture_Abort, m_pMICapture_Abort->IsEnabled() );
        m_pUpperToolBar->EnableTool( miCapture_Unlock, m_pMICapture_Unlock->IsEnabled() );
        m_pUpperToolBar->EnableTool( miCapture_Record, m_pMICapture_Record->IsEnabled() );
        m_pUpperToolBar->ToggleTool( miCapture_Record, boDevRecord );
        m_pUpperToolBar->EnableTool( miCapture_Forward, m_pMICapture_Forward->IsEnabled() );
        m_pUpperToolBar->EnableTool( miCapture_Backward, m_pMICapture_Backward->IsEnabled() );
    }
    if( m_pLeftToolBar )
    {
        m_pLeftToolBar->EnableTool( miCapture_CaptureSettings_CaptureSettingHierarchy, boDevOpen && boMultipleCaptureSettingsSupported );
        m_pLeftToolBar->EnableTool( miSettings_Display_ShowMonitorImage, m_pMISettings_Display_Active->IsChecked() );
        switch( m_currentWizard )
        {
        case wColorCorrection:
        case wFileAccessControl:
        case wLensControl:
        case wLUTControl:
        case wQuickSetup:
            m_pLeftToolBar->EnableTool( miWizard_Open, true );
            break;
        case wSequencerControl:
            m_pLeftToolBar->EnableTool( miWizard_Open, !boDevLive );
            break;
        case wNone:
            m_pLeftToolBar->EnableTool( miWizard_Open, false );
            break;
        }
    }

    m_pMICapture_CaptureSettings_CreateCaptureSetting->Enable( boDevOpen && boMultipleCaptureSettingsSupported );
    m_pMICapture_CaptureSettings_CaptureSettingHierarchy->Enable( boDevOpen && boMultipleCaptureSettingsSupported );
    m_pMICapture_CaptureSettings_AssignToDisplays->Enable( boDevOpen && boMultipleCaptureSettingsSupported );
    m_pMICapture_CaptureSettings_UsageMode_Manual->Enable( boOpen_NotLive && boMultipleCaptureSettingsSupported );
    m_pMICapture_CaptureSettings_UsageMode_Automatic->Enable( boOpen_NotLive && boMultipleCaptureSettingsSupported );

    m_pMISettings_Display_ConfigureImageDisplayCount->Enable( !boDevLive );

    // setup wizard menu
    WizardFeatureMap supportedWizards( m_pDevPropHandler->GetActiveDeviceSupportedWizards() );
    const WizardFeatureMap::const_iterator itEND = supportedWizards.end();
    const bool boFileAccessControlWizardsAvailable = ( supportedWizards.find( wFileAccessControl ) != itEND );
    m_pMIWizards_ColorCorrection->Enable( supportedWizards.find( wColorCorrection ) != itEND );
    m_pMIWizards_FileAccessControl_UploadFile->Enable( boFileAccessControlWizardsAvailable );
    m_pMIWizards_FileAccessControl_DownloadFile->Enable( boFileAccessControlWizardsAvailable );
    m_pMIWizards_LensControl->Enable( supportedWizards.find( wLensControl ) != itEND );
    m_pMIWizards_LUTControl->Enable( supportedWizards.find( wLUTControl ) != itEND );
    m_pMIWizards_QuickSetup->Enable( ( supportedWizards.find( wQuickSetup ) != itEND ) && boDevOpen && boDevPresent );
    m_pMIWizards_SequencerControl->Enable( ( supportedWizards.find( wSequencerControl ) != itEND ) && !boDevLive );
    m_pMISettings_ShowQuickSetupOnUse->Enable( !HasEnforcedQSWBehavior() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupImageProcessingMode( void )
//-----------------------------------------------------------------------------
{
    CaptureThread* pCT = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pCT );
    if( pCT )
    {
        pCT->SetImageProcessingMode( m_defaultImageProcessingMode );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupUpdateFrequencies( bool boCurrentValue )
//-----------------------------------------------------------------------------
{
    for( unsigned int i = 0; i < iapLAST; i++ )
    {
        if( m_ImageAnalysisPlots[i].m_pPlotCanvas )
        {
            m_ImageAnalysisPlots[i].m_pPlotCanvas->SetUpdateFrequency( ( boCurrentValue ) ? m_ImageAnalysisPlots[i].m_pSCUpdateSpeed->GetValue() : 1 );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::SetupVerSplitter( void )
//-----------------------------------------------------------------------------
{
    if( m_pMISettings_PropGrid_Show->IsChecked() &&
        ( m_pMISettings_Analysis_ShowControls->IsChecked() || m_pMISettings_Display_Active->IsChecked() ) )
    {
        m_pVerticalSplitter->SplitVertically( m_pSettingsPanel, m_pHorizontalSplitter, 0 );
        m_pVerticalSplitter->SetSashPosition( m_VerticalSplitterPos );
    }
    else if( !m_pMISettings_PropGrid_Show->IsChecked() ||
             m_pMISettings_Display_Active->IsChecked() ||
             m_pMISettings_Analysis_ShowControls->IsChecked() )
    {
        ConfigureSplitter( m_pVerticalSplitter, m_pSettingsPanel, m_pHorizontalSplitter, false );
    }
    else if( m_pMISettings_PropGrid_Show->IsChecked() && !m_pMISettings_Display_Active->IsChecked() && !m_pMISettings_Analysis_ShowControls->IsChecked() )
    {
        ConfigureSplitter( m_pVerticalSplitter, m_pHorizontalSplitter, m_pSettingsPanel, false );
    }
    else if( !m_pVerticalSplitter->IsSplit() )
    {
        m_pVerticalSplitter->SplitVertically( m_pSettingsPanel, m_pHorizontalSplitter, 0 );
        m_pVerticalSplitter->SetSashPosition( m_VerticalSplitterPos );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::ToggleCurrentDevice( void )
//-----------------------------------------------------------------------------
{
    int devIndex = m_pDevCombo->GetSelection();
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev )
    {
        wxBusyCursor busyCursorScope;
        const bool isOpen = pDev->isOpen();
        WriteLogMessage( wxString::Format( wxT( "Trying to %s device %d (%s)...\n" ), ( isOpen ? wxT( "close" ) : wxT( "open" ) ), devIndex, ConvertedString( pDev->serial.read() ).c_str() ) );
        try
        {
            if( isOpen )
            {
                if( !m_boCloseDeviceInProgress )
                {
                    m_boCloseDeviceInProgress = true;
                    if( !m_boCurrentImageIsFromFile )
                    {
                        const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
                        for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
                        {
                            CaptureThread* pCT = 0;
                            m_pDevPropHandler->GetActiveDevice( 0, 0, &pCT );
                            if( pCT && ( m_CurrentRequestDataContainer[i].requestNr_ != INVALID_ID ) )
                            {
                                pCT->UnlockRequest( m_CurrentRequestDataContainer[i].requestNr_ );
                            }
                            m_CurrentRequestDataContainer[i] = RequestData();
                            m_DisplayAreas[i]->ResetSkippedImagesCounter();
                            m_DisplayAreas[i]->SetImage( m_CurrentRequestDataContainer[i].image_.getBuffer() );
                            if( m_DisplayAreas[i]->GetMonitorDisplay() == m_pMonitorImage->GetDisplayArea() )
                            {
                                m_pMonitorImage->GetDisplayArea()->SetImage( m_DisplayAreas[i]->GetImage() );
                            }
                        }
                    }
                    {
                        wxCriticalSectionLocker locker( m_critSect );
                        m_settingToDisplayDict.clear();
                        m_sequencerSetToDisplayMap.clear();
                    }
                    DestroyAdditionalDialogs();
                    m_stopWatch.Start();
                    m_pDevPropHandler->CloseDriver( GetSelectedDeviceSerial() );
                    WriteLogMessage( wxString::Format( wxT( "Closing device '%s' took %ld ms.\n" ), ConvertedString( pDev->serial.read() ).c_str(), m_stopWatch.Time() ) );
                    UpdateFeatureVsTimePlotFeature();
                    UpdateInfoPlotCombo();
                    m_pInfoPlotArea->ClearCache();
                    UpdateData();
                    UpdateWizardStatus( GetPropertyGrid()->GetSelectedProperty() );
                    m_boCloseDeviceInProgress = false;
                    m_boSingleCaptureInProgess = false;
                }
            }
            else
            {
                if( pDev->interfaceLayout.isValid() )
                {
                    vector<TDeviceInterfaceLayout> interfaceLayouts;
                    pDev->interfaceLayout.getTranslationDictValues( interfaceLayouts );
                    const vector<TDeviceInterfaceLayout>::const_iterator itGenICamInterfaceLayout = find( interfaceLayouts.begin(), interfaceLayouts.end(), dilGenICam );
                    if( ( pDev->interfaceLayout.read() != dilGenICam ) &&
                        ( itGenICamInterfaceLayout != interfaceLayouts.end() ) )
                    {
                        if( wxMessageBox( wxT( "Deprecated Interface Layout detected:\n\nThis interface layout has been declared deprecated. Please think about porting an application using it to the interface layout 'GenICam' instead. Do you wish to proceed?" ), wxT( "Deprecated interface layout detected" ), wxYES_NO | wxICON_EXCLAMATION, this ) != wxYES )
                        {
                            WriteLogMessage( wxString::Format( wxT( "Opening of device '%s' in '%s' interface layout canceled by user.\n" ), ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->interfaceLayout.readS() ).c_str() ) );
                            return;
                        }
                    }
                }

                if( m_pMISettings_WarnOnOutdatedFirmware->IsChecked() )
                {
                    ProductFirmwareTable::const_iterator it = m_ProductFirmwareTable.find( pDev->product.read() );
                    if( it != m_ProductFirmwareTable.end() )
                    {
                        const int firmwareVersion( pDev->firmwareVersion.read() );
                        if( firmwareVersion < it->second )
                        {
                            WriteErrorMessage( wxString::Format( wxT( "WARNING: Device '%s' uses an outdated firmware version. It's recommended to update the firmware with mvDeviceConfigure.\n" ), ConvertedString( pDev->serial.read() ).c_str() ) );
                            switch( wxMessageBox( wxString::Format( wxT( "Device '%s' uses an outdated firmware version.\n\nPress 'Yes' to continue without updating the firmware.\n\nPress 'No' to launch mvDeviceConfigure and update the firmware of the device(WARNING: Under some versions of Windows this will only work if the current application has been started with the 'Run as administrator' option.\n\nPress 'Cancel' to continue without updating the firmware and never see this message again. You can later re-enable this message box under 'Settings -> Warn On Outdated Firmware'." ), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "Firmware Update Available" ), wxYES_NO | wxCANCEL | wxICON_EXCLAMATION, this ) )
                            {
                            case wxNO:
                                {
                                    wxString commandString( wxT( "mvDeviceConfigure update_fw=" ) );
                                    commandString.Append( ConvertedString( pDev->serial.read() ) );
                                    ::wxExecute( commandString );
                                    UpdateDeviceList();
                                }
                                return;
                            case wxCANCEL:
                                m_pMISettings_WarnOnOutdatedFirmware->Check( false );
                                break;
                            }
                        }
                    }
                }

                const wxString deviceSerial( GetSelectedDeviceSerial() );
                // see 'Usage hints' to understand this section
                bool boUpdateDeviceListAndRetry = false;
                bool boRunningIPConfigureMightHelp = false;
                try
                {
                    switch( CheckIfDeviceIsReachable( pDev, boRunningIPConfigureMightHelp ) )
                    {
                    case wxNO:
                        if( boRunningIPConfigureMightHelp )
                        {
                            switch( wxMessageBox( wxT( "Do you want to run mvIPConfigure now?" ), wxT( "Start mvIPConfigure" ), wxYES_NO, this ) )
                            {
                            case wxYES:
                                ::wxExecute( wxT( "mvIPConfigure" ) );
                                break;
                            }
                        }
                        Close( true );
                        return;
                    case wxCANCEL:
                        WriteLogMessage( wxString::Format( wxT( "Opening of device '%s' in '%s' interface layout canceled by user.\n" ), ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->interfaceLayout.readS() ).c_str() ) );
                        return;
                    default:
                        break;
                    }
                    OpenDriverWithTimeMeassurement( pDev );
                }
                catch( const ImpactAcquireException& )
                {
                    if( m_pMIAction_AutomaticallyReconnectToUnusedDevices->IsChecked() )
                    {
                        boUpdateDeviceListAndRetry = true;
                    }
                    else
                    {
                        throw;
                    }
                }
                if( boUpdateDeviceListAndRetry )
                {
                    UpdateDeviceList();
                    OpenDriverWithTimeMeassurement( pDev );
                }
                if( !pDev->isOpen() )
                {
                    ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, DMR_DEV_CANNOT_OPEN, INVALID_ID, "Open device failed(skipped by user request)" );
                }
                UpdateInfoPlotCombo();
                SetFocus();//workaround for a wxWidgets bug (not focusing on first selected property).
                ExpandPropertyTreeToDeviceSettings();
                m_pInfoPlotArea->ClearCache();
                UpdateAcquisitionModes();
                SetupCaptureSettingsUsageMode( m_pMICapture_CaptureSettings_UsageMode_Manual->IsChecked() ? csumManual : csumAutomatic );
                SetupImageProcessingMode();
                UpdateSettingTable();
                AssignDefaultSettingToDisplayRelationship();
                ConfigureRequestCount();
                if( ( m_boShowQuickSetupWizardCurrentProcess == true ) &&
                    ( m_pDevPropHandler->DoesActiveDeviceSupportWizard( wQuickSetup ) == true ) )
                {
                    Wizard_QuickSetup();
                }
                else
                {
                    // If it is the very first time and the QSW has never been shown, we should make the GUI
                    // richer by showing the left tool bar and the property grid.
                    SetupGUIOnFirstRun();
                }
            }
            WriteLogMessage( wxString::Format( wxT( "Device %d (%s) %s\n" ), devIndex, ConvertedString( pDev->serial.read() ).c_str(), ( isOpen ? wxT( "closed" ) : wxT( "opened" ) ) ) );
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "\n%s(%s)\n" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            WriteErrorMessage( wxString::Format( wxT( "Error origin: %s.\n" ), ConvertedString( e.getErrorOrigin() ).c_str() ) );
        }
        SetupDlgControls();
        if( pDev->isOpen() )
        {
            CheckForDriverPerformanceIssues( pDev );
#if defined(linux) || defined(__linux) || defined(__linux__)
            CheckForPotentialBufferIssues( pDev );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
        }
    }
}

//-----------------------------------------------------------------------------
bool PropViewFrame::UpdateAcquisitionModes( void )
//-----------------------------------------------------------------------------
{
    m_boSelectedDeviceSupportsMultiFrame = false;
    m_boSelectedDeviceSupportsSingleFrame = false;
    Property acquisitionMode( m_pDevPropHandler->GetActiveDeviceAcquisitionMode() );
    if( !acquisitionMode.isValid() )
    {
        // devices that do not support the acquisition mode property will most likely be our own an all of them support
        // single frame acquisition
        m_boSelectedDeviceSupportsSingleFrame = true;
    }
    if( acquisitionMode.isValid() )
    {
        // this unfortunately is a design mistake we made in the past. Depending on the
        // interface layout the data type of 'AcquisitionMode' can differ.
        switch( acquisitionMode.type() )
        {
        case ctPropInt:
            {
                vector<pair<string, int> > dict;
                PropertyI prop( acquisitionMode );
                prop.getTranslationDict( dict );
                const vector<pair<string, int> >::size_type cnt = dict.size();
                for( vector<pair<string, int> >::size_type i = 0; i < cnt; i++ )
                {
                    if( dict[i].first == string( m_MultiFrameStr.mb_str() ) )
                    {
                        if( m_pDevPropHandler->GetActiveDeviceAcquisitionFrameCount().isValid() )
                        {
                            m_boSelectedDeviceSupportsMultiFrame = true;
                        }
                    }
                    else if( dict[i].first == string( m_SingleFrameStr.mb_str() ) )
                    {
                        m_boSelectedDeviceSupportsSingleFrame = true;
                    }
                }
            }
            break;
        case ctPropInt64:
            {
                vector<pair<string, int64_type> > dict;
                PropertyI64 prop( acquisitionMode );
                prop.getTranslationDict( dict );
                const vector<pair<string, int64_type> >::size_type cnt = dict.size();
                for( vector<pair<string, int64_type> >::size_type i = 0; i < cnt; i++ )
                {
                    if( dict[i].first == string( m_MultiFrameStr.mb_str() ) )
                    {
                        if( m_pDevPropHandler->GetActiveDeviceAcquisitionFrameCount().isValid() )
                        {
                            m_boSelectedDeviceSupportsMultiFrame = true;
                        }
                    }
                    else if( dict[i].first == string( m_SingleFrameStr.mb_str() ) )
                    {
                        m_boSelectedDeviceSupportsSingleFrame = true;
                    }
                }
            }
            break;
        default:
            break;
        }
    }

    m_pAcquisitionModeCombo->Clear();
    m_pAcquisitionModeCombo->Append( m_ContinuousStr );
    if( m_boSelectedDeviceSupportsMultiFrame )
    {
        m_pAcquisitionModeCombo->Append( m_MultiFrameStr );
    }
    if( m_boSelectedDeviceSupportsSingleFrame )
    {
        m_pAcquisitionModeCombo->Append( m_SingleFrameStr );
    }

    const wxString am( acquisitionMode.isValid() ? ConvertedString( acquisitionMode.readS() ) : m_ContinuousStr );
    if( am == m_ContinuousStr )
    {
        m_pAcquisitionModeCombo->SetValue( m_ContinuousStr );
    }
    else if( am == m_MultiFrameStr )
    {
        m_pAcquisitionModeCombo->SetValue( m_MultiFrameStr );
    }
    else if( am == m_SingleFrameStr )
    {
        m_pAcquisitionModeCombo->SetValue( m_SingleFrameStr );
    }
    return m_boSelectedDeviceSupportsMultiFrame;
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateAnalysisPlotAOIParameter( PlotCanvasImageAnalysis* pPlotCanvas, int x /* = -1 */, int y /* = -1 */, int w /* = -1 */, int h /* = -1 */ )
//-----------------------------------------------------------------------------
{
    if( m_boUpdateAOIInProgress )
    {
        return;
    }
    pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex], x, y, w, h );
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateAnalysisPlotGridSteps( PlotCanvasImageAnalysis* pPlotCanvas, int XSteps, int YSteps )
//-----------------------------------------------------------------------------
{
    pPlotCanvas->SetGridSteps( XSteps, YSteps );
    pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex], -1, -1, -1, -1, true );
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateData( const bool boShowInfo /* = false */, const bool boForceUnlock /* = false */, const bool boFromRecordedStream /* = false */ )
//-----------------------------------------------------------------------------
{
    // To prevent a crash when the handler e.g. OnImageReady() gets called and 'm_pDevPropHandler' has already been deleted or when
    // reaching this code and 'm_pDevPropHandler' has not been created.
    if( !m_pDevPropHandler )
    {
        return;
    }

    TImageBufferPixelFormat lastPixelFormat = m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->pixelFormat;
    Device* pDev = 0;
    CaptureThread* pCT = 0;
    RequestData data;
    int lastRequestNr = INVALID_ID;
    int index = 0;

    pDev = m_pDevPropHandler->GetActiveDevice( 0, 0, &pCT );
    if( !m_boCurrentImageIsFromFile && boForceUnlock )
    {
        if( pCT && pDev && !m_boCloseDeviceInProgress && pDev->isOpen() )
        {
            pCT->GetImageData( data, boFromRecordedStream );
        }
    }
    else
    {
        lastRequestNr = m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].requestNr_;
        index = m_CurrentRequestDataIndex;
    }

    if( data.requestNr_ != INVALID_ID )
    {
        wxCriticalSectionLocker locker( m_critSect );
        if( !m_sequencerSetToDisplayMap.empty() )
        {
            SequencerSetToDisplayMap::const_iterator it = m_sequencerSetToDisplayMap.find( data.requestInfo_.chunkSequencerSetActive_ );
            index = static_cast< int >( ( it != m_sequencerSetToDisplayMap.end() ) ? it->second : 0 );
        }
        else
        {
            SettingToDisplayDict::const_iterator it = m_settingToDisplayDict.find( data.requestInfo_.settingUsed_ );
            index = static_cast<int>( ( it != m_settingToDisplayDict.end() ) ? it->second : 0 );
        }
        index = index % m_CurrentRequestDataContainer.size(); // make sure that even after someone reduced the display count everything works as expected!
        lastRequestNr = m_CurrentRequestDataContainer[index].requestNr_;
        lastPixelFormat = m_CurrentRequestDataContainer[index].image_.getBuffer()->pixelFormat;
        m_CurrentRequestDataContainer[index] = data;
    }

    if( !m_boCurrentImageIsFromFile )
    {
        if( m_DisplayAreas[index]->InfoOverlayActive() || boShowInfo )
        {
            if( pCT && pDev && !m_boCloseDeviceInProgress && pDev->isOpen() )
            {
                if( data.requestInfoStrings_.empty() )
                {
                    pCT->GetImageInfo( data.requestInfoStrings_ );
                }
                if( boShowInfo )
                {
                    wxString msg;
                    vector<wxString>::size_type vSize = data.requestInfoStrings_.size();
                    for( vector<wxString>::size_type i = 0; i < vSize; i++ )
                    {
                        msg.append( data.requestInfoStrings_[i] );
                        msg.append( ( i < vSize - 1 ) ? wxT( ", " ) : wxT( "\n" ) );
                    }
                    WriteLogMessage( msg );
                }
            }
            if( m_DisplayAreas[index]->InfoOverlayActive() )
            {
                m_DisplayAreas[index]->SetInfoOverlay( data.requestInfoStrings_ );
            }
        }
    }

    ImageBuffer* p = m_CurrentRequestDataContainer[index].image_.getBuffer();
    if( !m_boImageCanvasInFullScreenMode && ( m_DisplayAreas[index]->GetMonitorDisplay() == m_pMonitorImage->GetDisplayArea() ) )
    {
        m_pMonitorImage->GetDisplayArea()->SetImage( p );
    }
    if( !m_boImageCanvasInFullScreenMode || m_DisplayAreas[index]->IsFullScreen() )
    {
        if( !m_DisplayAreas[index]->SetImage( p ) && ( lastPixelFormat != p->pixelFormat ) )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s: ERROR!!! Don't know how to display format %s.\n" ), ConvertedString( __FUNCTION__ ).c_str(), m_CurrentRequestDataContainer[index].pixelFormat_.c_str() ) );
        }
    }
    if( p && p->vpData )
    {
        if( m_HardDiscRecordingParameters.boActive_ )
        {
            wxString name( wxT( "IMG" ) );
            name.Append( wxString::Format( wxT( MY_FMT_I64_0_PADDED ), m_CurrentRequestDataContainer[index].requestInfo_.frameNr_ ) );
            wxFileName filename( m_HardDiscRecordingParameters.targetDirectory_, name );
            SaveImage( filename.GetFullPath(), m_HardDiscRecordingParameters.fileFormat_ );
        }
        if( !m_boImageCanvasInFullScreenMode && ( m_CurrentImageAnalysisControlIndex >= 0 ) && ( index == m_CurrentRequestDataIndex ) )
        {
            ConfigureAOIControlLimits( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex] );
            m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[index],
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIx->GetValue(),
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIy->GetValue(),
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIw->GetValue(),
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIh->GetValue() );
        }
        if( lastPixelFormat != p->pixelFormat )
        {
            switch( p->pixelFormat )
            {
            case ibpfMono10:
            case ibpfMono12:
            case ibpfMono12Packed_V1:
            case ibpfMono12Packed_V2:
            case ibpfMono14:
            case ibpfMono16:
            case ibpfBGR888Packed:
            case ibpfBGR101010Packed_V2:
            case ibpfRGB101010Packed:
            case ibpfRGB121212Packed:
            case ibpfRGB141414Packed:
            case ibpfRGB161616Packed:
            case ibpfRGB888Planar:
            case ibpfRGBx888Planar:
            case ibpfYUV411_UYYVYY_Packed:
            case ibpfYUV422Packed:
            case ibpfYUV422_10Packed:
            case ibpfYUV422_UYVYPacked:
            case ibpfYUV422_UYVY_10Packed:
            case ibpfYUV444_UYVPacked:
            case ibpfYUV444_UYV_10Packed:
            case ibpfYUV444Packed:
            case ibpfYUV444_10Packed:
            case ibpfYUV422Planar:
                WriteLogMessage( wxString::Format( wxT( "%s: Please note that pixel format %s will require additional CPU time as it needs to be converted for displaying.\n" ), ConvertedString( __FUNCTION__ ).c_str(), m_CurrentRequestDataContainer[index].pixelFormat_.c_str() ) );
                break;
            default:
                break;
            }
        }
    }

    if( pCT && ( lastRequestNr != INVALID_ID ) &&
        ( ( lastRequestNr != m_CurrentRequestDataContainer[index].requestNr_ ) || m_boCurrentImageIsFromFile || boForceUnlock ) )
    {
        pCT->UnlockRequest( lastRequestNr );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateDeviceComboBox( void )
//-----------------------------------------------------------------------------
{
    const DeviceManager& devMgr = m_pDevPropHandler->GetDevMgr();
    bool boDisplayConnectedDevicesOnly = m_pMIAction_DisplayConnectedDevicesOnly->IsChecked();
    int oldIndex = m_pDevCombo->GetSelection();
    wxString oldSelection = ( oldIndex == wxNOT_FOUND ) ? m_NoDevStr : m_pDevCombo->GetString( oldIndex );
    // delete all entries
    m_pDevCombo->Clear();
    m_ProductFirmwareTable.clear();
    // refill the combo box with up to date data
    const unsigned int devCnt = devMgr.deviceCount();
    if( devCnt > 0 )
    {
        for( unsigned int i = 0; i < devCnt; i++ )
        {
            if( !boDisplayConnectedDevicesOnly ||
                ( devMgr[i]->state.read() == dsPresent ) )
            {
                m_pDevCombo->Append( wxString::Format( wxT( "%s (%s, DeviceID: '%s')" ), ConvertedString( devMgr[i]->serial.read() ).c_str(), ConvertedString( devMgr[i]->product.read() ).c_str(), ConvertedString( devMgr[i]->deviceID.readS() ).c_str() ) );
            }
            const string product( devMgr[i]->product.read() );
            const int firmwareVersion( devMgr[i]->firmwareVersion.read() );
            ProductFirmwareTable::iterator it = m_ProductFirmwareTable.find( product );
            if( it == m_ProductFirmwareTable.end() )
            {
                m_ProductFirmwareTable.insert( make_pair( product, firmwareVersion ) );
            }
            else if( it->second < firmwareVersion )
            {
                it->second = firmwareVersion;
            }
        }
        if( IsListOfChoicesEmpty( m_pDevCombo ) )
        {
            m_pDevCombo->Append( m_NoDevStr );
        }
        if( oldSelection == m_NoDevStr )
        {
            m_pDevCombo->Select( 0 );
        }
        else
        {
            int selection = m_pDevCombo->FindString( oldSelection );
            m_pDevCombo->Select( ( selection == wxNOT_FOUND ) ? 0 : selection );
        }
    }
    else
    {
        m_pDevCombo->Append( m_NoDevStr );
        m_pDevCombo->Select( 0 );
    }

    m_DeviceListChangedCounter = devMgr.changedCount();
    if( m_pDevCombo->GetString( 0 ) != m_NoDevStr )
    {
        ChangeActiveDevice( GetSelectedDeviceSerial() );
    }
    UpdateAcquisitionModes();
    UpdateTitle();
    UpdateData();
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateDeviceFromComboBox( void )
//-----------------------------------------------------------------------------
{
    ChangeActiveDevice( GetSelectedDeviceSerial() );
    UpdateAcquisitionModes();
    RefreshPendingImageQueueDepthForCurrentDevice();
    UpdateInfoPlotCombo();
    m_pInfoPlotArea->ClearCache();
    UpdateTitle();
    UpdateData();

    const DisplayWindowContainer::size_type displayCount = GetDisplayCount();
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        m_DisplayAreas[i]->ResetSkippedImagesCounter();
        m_DisplayAreas[i]->SetImage( m_CurrentRequestDataContainer[i].image_.getBuffer() );
    }

    ImageBuffer* p = m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer();
    m_pMonitorImage->GetDisplayArea()->SetImage( p );
    if( p && p->vpData )
    {
        if( m_CurrentImageAnalysisControlIndex >= 0 )
        {
            ConfigureAOIControlLimits( m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex] );
            m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pPlotCanvas->RefreshData( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex],
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIx->GetValue(),
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIy->GetValue(),
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIw->GetValue(),
                    m_ImageAnalysisPlots[m_CurrentImageAnalysisControlIndex].m_pSCAOIh->GetValue() );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateDeviceInterfaceLayouts( void )
//-----------------------------------------------------------------------------
{
    const DeviceManager& devMgr = m_pDevPropHandler->GetDevMgr();
    m_DeviceCount = devMgr.deviceCount();
    for( unsigned int i = 0; i < m_DeviceCount; i++ )
    {
        try
        {
            if( devMgr[i]->interfaceLayout.isValid() &&
                devMgr[i]->interfaceLayout.isWriteable() &&
                ( devMgr[i]->interfaceLayout.readS() != string( m_defaultDeviceInterfaceLayout.mb_str() ) ) )
            {
                vector<pair<string, TDeviceInterfaceLayout> > dict;
                devMgr[i]->interfaceLayout.getTranslationDict( dict );
                vector<pair<string, TDeviceInterfaceLayout> >::const_iterator it = find_if( dict.begin(), dict.end(), FirstMatches<const string, TDeviceInterfaceLayout>( make_pair( string( m_defaultDeviceInterfaceLayout.mb_str() ), dilGenICam ) ) );
                if( it != dict.end() )
                {
                    devMgr[i]->interfaceLayout.writeS( string( m_defaultDeviceInterfaceLayout.mb_str() ) );
                    WriteLogMessage( wxString::Format( wxT( "Interface layout for device %d (%s) switched to '%s'.\n" ), i, ConvertedString( devMgr[i]->serial.read() ).c_str(), ConvertedString( devMgr[i]->interfaceLayout.readS() ).c_str() ) );
                }
            }
        }
        catch( const ImpactAcquireException& )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to update interface layout for device %d (%s).\n" ), i, ConvertedString( devMgr[i]->serial.read() ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateDeviceList( void )
//-----------------------------------------------------------------------------
{
    wxBusyCursor busyCursorScope;
    WriteLogMessage( wxT( "Updating device list...\n" ) );
    m_stopWatch.Start();
    try
    {
        // always run this code as new interfaces might appear at runtime e.g. when plugging in a network cable...
        const int64_type interfaceCount( m_pDevPropHandler->GetGenTLInterfaceCount() );
        for( int64_type interfaceIndex = 0; interfaceIndex < interfaceCount; interfaceIndex++ )
        {
            mvIMPACT::acquire::GenICam::InterfaceModule im( interfaceIndex );
            if( im.mvGevAdvancedDeviceDiscoveryEnable.isValid() &&
                im.mvGevAdvancedDeviceDiscoveryEnable.isWriteable() )
            {
                im.mvGevAdvancedDeviceDiscoveryEnable.write( bTrue );
            }
        }
    }
    catch( const ImpactAcquireException& e )
    {
        WriteErrorMessage( wxString::Format( wxT( "Internal error. Failed to configure GEV interfaces for 'advanced device discovery'. %s(numerical error representation: %d (%s))." ), ConvertedString( e.getErrorString() ).c_str(), e.getErrorCode(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
    }
    m_pDevPropHandler->GetDevMgr().updateDeviceList();
    m_pDevPropHandler->ValidateTrees( true );
    WriteLogMessage( wxString::Format( wxT( "Updating device list took %ld ms.\n" ), m_stopWatch.Time() ) );
    CheckForPotentialFirewallIssues();
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateFeatureVsTimePlotFeature( void )
//-----------------------------------------------------------------------------
{
    Component comp;
    wxString fullPath;
    m_pDevPropHandler->GetActiveDeviceFeatureVsTimePlotInfo( comp, fullPath );
    m_pFeatureValueVsTimePlotArea->SetComponentToPlot( comp, fullPath );
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateInfoPlotCombo( void )
//-----------------------------------------------------------------------------
{
    const wxString value = m_pInfoPlotSelectionCombo->GetValue();
    m_pInfoPlotSelectionCombo->Clear();

    FunctionInterface* pFI = 0;
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( &pFI, 0, &pThread );
    wxArrayString infoPlotChoices;
    if( pFI )
    {
        CollectFeatureNamesForInfoPlot( ComponentIterator( pFI->getRequest( 0 )->getComponentLocator() ).firstChild(), infoPlotChoices );
    }
    else
    {
        infoPlotChoices.Add( wxT( "None" ) );
    }
    m_pInfoPlotSelectionCombo->Append( infoPlotChoices );

    // try to restore the previous value
    if( m_pInfoPlotSelectionCombo->FindString( value ) != wxNOT_FOUND )
    {
        m_pInfoPlotSelectionCombo->SetValue( value );
    }
    else
    {
        m_pInfoPlotSelectionCombo->Select( 0 );
    }

    if( pThread )
    {
        const string pathANSI( m_pInfoPlotSelectionCombo->GetValue().mb_str() );
        pThread->SetCurrentPlotInfoPath( pFI ? pathANSI : "" );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateLUTWizardAfterLoadSetting( const int result )
//-----------------------------------------------------------------------------
{
    if( ( m_pLUTControlDlg != 0 ) && ( result == DMR_NO_ERROR ) )
    {
        m_pLUTControlDlg->UpdateDialog();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdatePropGridViewMode( void )
//-----------------------------------------------------------------------------
{
    m_pDevPropHandler->SetViewMode( m_ViewMode,
                                    m_pMISettings_PropGrid_DisplayPropIndicesAsHex->IsChecked(),
                                    m_pMISettings_PropGrid_UseDisplayNameIfAvailable->IsChecked(),
                                    m_pMISettings_PropGrid_UseSelectorGrouping->IsChecked() );
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateRecordedImage( size_t index )
//-----------------------------------------------------------------------------
{
    CaptureThread* pThread = 0;
    m_pDevPropHandler->GetActiveDevice( 0, 0, &pThread );

    if( pThread )
    {
        SetupUpdateFrequencies( false );
        pThread->DisplaySequenceRequest( index );
        WriteLogMessage( wxString::Format( wxT( "Recorded image %lu: " ), static_cast<unsigned long int>( index ) ) );
        m_pRecordDisplaySlider->SetValue( static_cast<int>( index ) );
        m_boCurrentImageIsFromFile = false;
        UpdateData( true, true, true );
    }
    SetupDlgControls();
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateSettingTable( void )
//-----------------------------------------------------------------------------
{
    CaptureThread* pCT = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( 0, 0, &pCT );
    if( pDev && pCT )
    {
        pCT->UpdateSettingTable();
        ImageRequestControl irc( pDev );
        vector<pair<string, int> > settings;
        irc.setting.getTranslationDict( settings );
        if( m_pInfoPlotArea )
        {
            m_pInfoPlotArea->SetupPlotIdentifiers( settings );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateStatusBar( void )
//-----------------------------------------------------------------------------
{
    if( !m_pStatusBar->IsShown() || !m_pDevPropHandler )
    {
        return;
    }

    const Statistics* pS = 0;
    FunctionInterface* pFI = 0;
    CaptureThread* pCT = 0;
    Device* pDev = m_pDevPropHandler->GetActiveDevice( &pFI, &pS, &pCT );
    bool boValidDevice = pFI && pS && pCT;
    bool boValidImage = m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer() && m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->vpData;
    if( boValidDevice || boValidImage )
    {
        if( m_pStatusBar->GetFieldsCount() != NO_OF_STATUSBAR_FIELDS )
        {
            int statusBarFieldWidths[NO_OF_STATUSBAR_FIELDS];
            statusBarFieldWidths[sbfFramesPerSecond] = -10;
            statusBarFieldWidths[sbfBandwidthConsumed] = -11;
            statusBarFieldWidths[sbfFramesDelivered] = -6;
            statusBarFieldWidths[sbfStatistics] = -18;
            statusBarFieldWidths[sbfImageFormat] = -18;
            statusBarFieldWidths[sbfPixelData] = -10;
            m_pStatusBar->SetFieldsCount( NO_OF_STATUSBAR_FIELDS, &statusBarFieldWidths[0] );
        }

        bool boSlowDisplay = ( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->pixelFormat != ibpfMono8 ) &&
                             ( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->pixelFormat != ibpfRGBx888Packed ) &&
                             ( m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].pixelFormat_ != RequestData::UNKNOWN_PIXEL_FORMAT_STRING_ );
        if( pS && pCT )
        {
            const double fps = pS->framesPerSecond.read();
            m_pStatusBar->SetStatusText( wxString::Format( wxT( "FPS: %.1f, Display Frame Rate: %.1f" ), fps, fps * ( pCT->GetPercentageOfImagesSentToDisplay() / 100. ) ), sbfFramesPerSecond );
            wxString bandwidth;
            if( pS->bandwidthConsumed.isValid() )
            {
                bandwidth.Append( wxString::Format( wxT( "%.2f MB/s" ), pS->bandwidthConsumed.read() / 1000. ) );
            }
            else
            {
                bandwidth.Append( wxT( "-" ) );
            }

            if( pDev->state.read() == dsPresent )
            {
                PropertyI64 acquisitionMemoryFrameCount( m_pDevPropHandler->GetActiveDeviceAcquisitionMemoryFrameCount() );
                if( acquisitionMemoryFrameCount.isValid() )
                {
                    const double frameBufferUsage_pc = 100. * ( static_cast<double>( acquisitionMemoryFrameCount.read() ) / static_cast<double>( acquisitionMemoryFrameCount.getMaxValue() ) );
                    bandwidth.Append( wxString::Format( wxT( ", Frame Buffer Usage: %.2f%%" ), frameBufferUsage_pc ) );
                }
            }
            m_pStatusBar->SetStatusText( bandwidth, sbfBandwidthConsumed );
            m_pStatusBar->SetStatusText( wxString::Format( wxT( "Frames: %d" ), pS->frameCount.read() ), sbfFramesDelivered );
            m_pStatusBar->SetStatusText( wxString::Format( wxT( "Errors: %d Timeouts: %d Aborted: %d Lost: %d Incomplete: %d" ), pS->errorCount.read(),
                                         pS->timedOutRequestsCount.read(), pS->abortedRequestsCount.read(), pS->lostImagesCount.read(), pS->framesIncompleteCount.read() ), sbfStatistics );
        }
        else
        {
            m_pStatusBar->SetStatusText( wxT( "FPS: -, Display Frame Rate: -" ), sbfFramesPerSecond );
            m_pStatusBar->SetStatusText( wxT( "-" ), sbfBandwidthConsumed );
            m_pStatusBar->SetStatusText( wxT( "Frames: -" ), sbfFramesDelivered );
            m_pStatusBar->SetStatusText( wxT( "Errors: -, Timeouts: -, Aborted: -, Lost: -, Incomplete: -" ), sbfStatistics );
        }
        wxString imageFormatMsg( wxString::Format( wxT( "Image: %dx%d, %s%s" ),
                                 m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iWidth,
                                 m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].image_.getBuffer()->iHeight,
                                 m_CurrentRequestDataContainer[m_CurrentRequestDataIndex].pixelFormat_.c_str(),
                                 ( boSlowDisplay ? wxT( "(slow display: Conversion needed)" ) : wxT( "" ) ) ) );
        m_pStatusBar->SetStatusText( imageFormatMsg, sbfImageFormat );
        RefreshCurrentPixelData();
    }
    else
    {
        if( m_pStatusBar->GetFieldsCount() != 1 )
        {
            m_pStatusBar->SetFieldsCount( 1, 0 );
        }
        m_pStatusBar->SetStatusText( wxT( "no statistics available" ), 0 );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateTitle( void )
//-----------------------------------------------------------------------------
{
    wxString title( wxString::Format( wxT( "wxPropView(%s) [%s]" ), VERSION_STRING, m_pDevCombo->GetValue().c_str() ) );
    if( m_pDevPropHandler )
    {
        string serialANSI( GetSelectedDeviceSerial().mb_str() );
        Device* p = m_pDevPropHandler->GetDevMgr().getDeviceBySerial( serialANSI );
        if( p && p->isOpen() )
        {
            mvIMPACT::acquire::Info info( p );
            title.append( wxString::Format( wxT( "(driver version: %s)" ), ConvertedString( info.driverVersion.readS() ).c_str() ) );
        }
    }
    SetTitle( title );
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateWizardStatus( wxPGProperty* pProp )
//-----------------------------------------------------------------------------
{
    m_currentWizard = wNone;
    if( pProp && m_pDevPropHandler )
    {
        PropData* prop_data = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( prop_data )
        {
            const HOBJ hObj( prop_data->GetComponent().hObj() );
            WizardFeatureMap supportedWizards( m_pDevPropHandler->GetActiveDeviceSupportedWizards() );
            WizardFeatureMap::const_iterator it = supportedWizards.begin();
            const WizardFeatureMap::const_iterator itEND = supportedWizards.end();
            while( it != itEND )
            {
                if( it->second.find( hObj ) != it->second.end() )
                {
                    m_currentWizard = it->first;
                    break;
                }
                ++it;
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateUserControlledImageProcessingEnableProperties( void )
//-----------------------------------------------------------------------------
{
    const DeviceManager& devMgr = m_pDevPropHandler->GetDevMgr();
    m_DeviceCount = devMgr.deviceCount();
    const TBoolean desiredValue = ( m_defaultImageProcessingMode != ipmDefault ) ? bTrue : bFalse;
    for( unsigned int i = 0; i < m_DeviceCount; i++ )
    {
        try
        {
            if( devMgr[i]->userControlledImageProcessingEnable.isValid() &&
                devMgr[i]->userControlledImageProcessingEnable.isWriteable() &&
                ( devMgr[i]->userControlledImageProcessingEnable.read() != desiredValue ) )
            {
                devMgr[i]->userControlledImageProcessingEnable.write( desiredValue );
                WriteLogMessage( wxString::Format( wxT( "'UserControlledImageProcessingEnable' for device %d (%s) switched to '%s'.\n" ), i, ConvertedString( devMgr[i]->serial.read() ).c_str(), ConvertedString( devMgr[i]->userControlledImageProcessingEnable.readS() ).c_str() ) );
            }
        }
        catch( const ImpactAcquireException& )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to update 'UserControlledImageProcessingEnable' for device %d (%s).\n" ), i, ConvertedString( devMgr[i]->serial.read() ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::UpdateUserExperience( const wxString& userExperience )
//-----------------------------------------------------------------------------
{
    if( userExperience == wxString( ConvertedString( Component::visibilityAsString( cvBeginner ) ) ) )
    {
        GlobalDataStorage::Instance()->SetComponentVisibility( cvBeginner );
    }
    else if( userExperience == wxString( ConvertedString( Component::visibilityAsString( cvExpert ) ) ) )
    {
        GlobalDataStorage::Instance()->SetComponentVisibility( cvExpert );
    }
    else if( userExperience == wxString( ConvertedString( Component::visibilityAsString( cvGuru ) ) ) )
    {
        GlobalDataStorage::Instance()->SetComponentVisibility( cvGuru );
    }
    else if( userExperience == wxString( ConvertedString( Component::visibilityAsString( cvInvisible ) ) ) )
    {
        GlobalDataStorage::Instance()->SetComponentVisibility( cvInvisible );
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::Wizard_ColorCorrection( void )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev && !m_pColorCorrectionDlg )
    {
        try
        {
            m_pColorCorrectionDlg = new WizardColorCorrection( this, wxString::Format( wxT( "Color Correction For Device %s" ), ConvertedString( pDev->serial.readS() ).c_str() ), pDev );
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with the color twist filter on device '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str(), ConvertedString( pDev->serial.read() ).c_str() ) );
        }
    }
    if( m_pColorCorrectionDlg )
    {
        m_pColorCorrectionDlg->RefreshControls();
        m_pColorCorrectionDlg->ShowModal();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::Wizard_FileAccessControl( bool boUpload )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev )
    {
        char* pBuf = 0;
        try
        {
            mvIMPACT::acquire::GenICam::FileAccessControl fac( pDev );
            wxArrayString choices;
            size_t fileSelectorCnt = BuildStringArrayFromPropertyDict<int64_type, PropertyI64>( choices, fac.fileSelector );
            if( !m_boAllowFullDeviceFileAccess )
            {
                for( size_t i = 0; i < fileSelectorCnt; i++ )
                {
                    // do not present firmware related files to the user when working with the file access wizard
                    if( choices[i].Lower().Contains( wxT( "firmware" ) ) == true )
                    {
                        choices.RemoveAt( i );
                        i = 0;
                        fileSelectorCnt = choices.GetCount();
                    }
                }
            }
            if( choices.IsEmpty() == false )
            {
                const wxString fileNameDevice = ::wxGetSingleChoice( wxString::Format( wxT( "Please select the file you want to %s device '%s'" ), boUpload ? wxT( "upload to" ) : wxT( "download from" ), ConvertedString( pDev->serial.read() ).c_str() ),
                                                wxString::Format( wxT( "File Access Control: %s" ), boUpload ? wxT( "File Upload" ) : wxT( "File Download" ) ),
                                                choices,
                                                this );
                if( fileNameDevice != wxEmptyString )
                {
                    WriteLogMessage( wxString::Format( wxT( "File selected for %s: '%s'.\n" ), boUpload ? wxT( "upload" ) : wxT( "download" ), fileNameDevice.c_str() ) );
                    static const wxFileOffset fileOperationBlockSize = 4096;
                    if( boUpload )
                    {
                        wxFileDialog fileDlg( this, wxString::Format( wxT( "Select a file to upload into file '%s' of device '%s'" ), ConvertedString( pDev->serial.read() ).c_str(), fileNameDevice.c_str() ), wxT( "" ), wxT( "" ), wxT( "All types (*.*)|*.*" ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
                        if( fileDlg.ShowModal() == wxID_OK )
                        {
                            wxFile fileLocal( fileDlg.GetPath().c_str(), wxFile::read );
                            if( fileLocal.IsOpened() )
                            {
                                pBuf = new char[fileLocal.Length()];
                                if( fileLocal.Read( pBuf, fileLocal.Length() ) == fileLocal.Length() )
                                {
                                    WriteLogMessage( wxString::Format( wxT( "File '%s' successfully copied into RAM. Trying to upload the file to the device now.\n" ), fileDlg.GetPath().c_str() ) );
                                    wxBusyCursor busyCursorScope;
                                    mvIMPACT::acquire::GenICam::ODevFileStream file;
                                    file.open( pDev, fileNameDevice.mb_str() );
                                    if( !file.fail() )
                                    {
                                        wxFileOffset bytesWritten = 0;
                                        wxFileOffset bytesToWriteTotal = fileLocal.Length();
                                        ostringstream oss;
                                        oss << setw( 8 ) << setfill( '0' ) << bytesWritten << '/'
                                            << setw( 8 ) << setfill( '0' ) << bytesToWriteTotal;
                                        wxProgressDialog dlg( wxString::Format( wxT( "Uploading file '%s' to file selector entry '%s' on device" ), fileDlg.GetPath().c_str(), fileNameDevice.c_str() ),
                                                              wxString::Format( wxT( "Uploading file '%s' to file selector entry '%s' on device(%s bytes written)" ), fileDlg.GetPath().c_str(), fileNameDevice.c_str(), ConvertedString( oss.str() ).c_str() ),
                                                              bytesToWriteTotal / fileOperationBlockSize, // range
                                                              this,              // parent
                                                              wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME );
                                        while( bytesWritten < bytesToWriteTotal )
                                        {
                                            const wxFileOffset bytesToWrite = ( ( bytesWritten + fileOperationBlockSize ) <= bytesToWriteTotal ) ? fileOperationBlockSize : bytesToWriteTotal - bytesWritten;
                                            file.write( pBuf + bytesWritten, bytesToWrite );
                                            bytesWritten += bytesToWrite;
                                            ostringstream progress;
                                            progress << setw( 8 ) << setfill( '0' ) << bytesWritten << '/'
                                                     << setw( 8 ) << setfill( '0' ) << bytesToWriteTotal;
                                            dlg.Update( bytesWritten / fileOperationBlockSize, wxString::Format( wxT( "Uploading file '%s' to file selector entry '%s' on device(%s bytes written)" ), fileDlg.GetPath().c_str(), fileNameDevice.c_str(), ConvertedString( progress.str() ).c_str() ) );
                                        }
                                        if( file.good() )
                                        {
                                            wxMessageDialog dlgUploadSuccessful( NULL, wxString::Format( wxT( "File '%s' successfully uploaded into file '%s' of device '%s'.\n" ), fileDlg.GetFilename().c_str(), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "File Upload Succeeded" ), wxOK );
                                            dlgUploadSuccessful.ShowModal();
                                        }
                                        else
                                        {
                                            wxMessageDialog dlgUploadFailed( NULL, wxString::Format( wxT( "Could not upload file '%s' into file '%s' of device '%s'.\n" ), fileDlg.GetFilename().c_str(), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "File Upload Failed" ), wxOK | wxICON_INFORMATION );
                                            dlgUploadFailed.ShowModal();
                                        }
                                    }
                                    else
                                    {
                                        wxMessageDialog dlg( NULL, wxString::Format( wxT( "Could not open file '%s' with write access on device '%s'.\n" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "File Upload Failed" ), wxOK | wxICON_INFORMATION );
                                        dlg.ShowModal();
                                    }
                                }
                                else
                                {
                                    WriteErrorMessage( wxString::Format( wxT( "Failed to read file '%s' completely.\n" ), fileDlg.GetPath().c_str() ) );
                                }
                            }
                            else
                            {
                                WriteErrorMessage( wxString::Format( wxT( "Failed to open file '%s'.\n" ), fileDlg.GetPath().c_str() ) );
                            }
                        }
                        else
                        {
                            WriteLogMessage( wxT( "File upload canceled by user.\n" ) );
                        }
                    }
                    else
                    {
                        wxBusyCursor busyCursorScope;
                        mvIMPACT::acquire::GenICam::IDevFileStream file;
                        file.open( pDev, fileNameDevice.mb_str() );
                        if( !file.fail() )
                        {
                            const streamsize fileSize = file.size();
                            if( fileSize > 0 )
                            {
                                pBuf = new char[fileSize];
                                memset( pBuf, 0, fileSize );
                                wxFileOffset bytesRead = 0;
                                wxProgressDialog dlgProgress( wxString::Format( wxT( "Downloading file '%s' from device '%s' into RAM" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ),
                                                              wxString::Format( wxT( "Downloading file '%s' from device '%s' into RAM(0/%08u bytes read)" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str(), static_cast<unsigned int>( fileSize ) ),
                                                              fileSize / fileOperationBlockSize, // range
                                                              this,     // parent
                                                              wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME );
                                while( bytesRead < fileSize )
                                {
                                    const wxFileOffset bytesToRead = ( ( bytesRead + fileOperationBlockSize ) <= fileSize ) ? fileOperationBlockSize : fileSize - bytesRead;
                                    file.read( pBuf + bytesRead, bytesToRead );
                                    bytesRead += bytesToRead;
                                    ostringstream progress;
                                    progress << setw( 8 ) << setfill( '0' ) << bytesRead << '/'
                                             << setw( 8 ) << setfill( '0' ) << fileSize;
                                    dlgProgress.Update( bytesRead / fileOperationBlockSize, wxString::Format( wxT( "Downloading file '%s' from device '%s' into RAM(%s bytes read)" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( progress.str() ).c_str() ) );
                                }
                                if( file.good() )
                                {
                                    wxMessageDialog dlg( NULL, wxString::Format( wxT( "File '%s' successfully downloaded from device '%s' into RAM.\n" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "File Download Succeeded" ), wxOK );
                                    dlg.ShowModal();
                                    wxFileDialog fileDlg( this, wxString::Format( wxT( "Select a filename to store file '%s' of device '%s' to" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "" ), wxT( "" ), wxT( "All types (*.*)|*.*" ), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
                                    if( fileDlg.ShowModal() == wxID_OK )
                                    {
                                        WriteFile( pBuf, fileSize, fileDlg.GetPath(), m_pLogWindow );
                                    }
                                    else
                                    {
                                        WriteLogMessage( wxT( "File download canceled by user.\n" ) );
                                    }
                                }
                                else
                                {
                                    wxMessageDialog dlg( NULL, wxString::Format( wxT( "Could not download file '%s' from device '%s'.\n" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "File Download Failed" ), wxOK | wxICON_INFORMATION );
                                    dlg.ShowModal();
                                }
                            }
                            else
                            {
                                wxMessageDialog dlg( NULL, wxString::Format( wxT( "Can't read file size of file '%s' on device '%s'.\n" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "File Download Failed" ), wxOK | wxICON_INFORMATION );
                                dlg.ShowModal();
                            }
                        }
                        else
                        {
                            wxMessageDialog dlg( NULL, wxString::Format( wxT( "Could not open file '%s' with read access on device '%s'.\n" ), fileNameDevice.c_str(), ConvertedString( pDev->serial.read() ).c_str() ), wxT( "File Download Failed" ), wxOK | wxICON_INFORMATION );
                            dlg.ShowModal();
                        }
                    }
                }
                else
                {
                    WriteLogMessage( wxString::Format( wxT( "File %s canceled by user.\n" ), boUpload ? wxT( "upload" ) : wxT( "download" ) ) );
                }
            }
            else
            {
                WriteErrorMessage( wxString::Format( wxT( "%s(%d): 'FileSelector' of device '%s' does not allow the selection of any file.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( pDev->serial.read() ).c_str() ) );
            }
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with file data on device '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str(), ConvertedString( pDev->serial.read() ).c_str() ) );
        }
        delete [] pBuf;
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::Wizard_LensControl( void )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev && !m_pLensControlDlg )
    {
        try
        {
            m_pLensControlDlg = new WizardLensControl( this, wxString::Format( wxT( "Lens Control For Device %s" ), ConvertedString( pDev->serial.readS() ).c_str() ), pDev );
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to create a lens control wizard for device '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str(), ConvertedString( pDev->serial.read() ).c_str() ) );
        }
    }
    if( m_pLensControlDlg )
    {
        m_pLensControlDlg->Show();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::Wizard_LUTControl( void )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev && !m_pLUTControlDlg )
    {
        try
        {
            mvIMPACT::acquire::GenICam::LUTControl lc( pDev );
            wxArrayString choices;
            if( lc.LUTSelector.isValid() )
            {
                BuildStringArrayFromPropertyDict<int64_type, PropertyI64>( choices, lc.LUTSelector );
            }
            if( choices.IsEmpty() ) // a device might support a LUT but no LUT selector!
            {
                choices.Add( wxT( "LUT-0" ) );
            }
            m_pLUTControlDlg = new WizardLUTControl( this, wxString::Format( wxT( "LUT Control For Device %s" ), ConvertedString( pDev->serial.readS() ).c_str() ), lc, choices );
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with LUT data on device '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str(), ConvertedString( pDev->serial.read() ).c_str() ) );
        }
    }
    if( m_pLUTControlDlg )
    {
        m_pLUTControlDlg->Show();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::Wizard_QuickSetup( void )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev->desiredAccess.isValid() && pDev->desiredAccess.read() == damRead )
    {
        wxMessageDialog infoDlg( NULL, wxString::Format( wxT( "Current device has been opened with DesiredAccess:'Read'\nQuickSetup wizard does not support this mode!" ) ), wxT( "Unsupported access mode" ), wxOK | wxICON_INFORMATION );
        infoDlg.ShowModal();
        m_pUpperToolBar->ToggleTool( miWizards_QuickSetup, false );
        return;
    }
    EnsureAcquisitionState( false );
    m_pAcquisitionModeCombo->SetValue( m_ContinuousStr );
    m_pUpperToolBar->ToggleTool( miWizards_QuickSetup, true );
    const TDeviceInterfaceLayout currentDeviceInterfaceLayout = pDev->interfaceLayout.read();
    if( pDev->isOpen() )
    {
        SaveGUIStateBeforeQuickSetupWizard();
        ChangeGUIForQuickSetupWizard();
    }
    if( ( pDev && !m_pQuickSetupDlgCurrent ) ||
        ( pDev && m_pQuickSetupDlgCurrent->IsGenICam() && currentDeviceInterfaceLayout == dilDeviceSpecific && !m_pQuickSetupDlgDeviceSpecific ) ||
        ( pDev && !m_pQuickSetupDlgCurrent->IsGenICam() && currentDeviceInterfaceLayout == dilGenICam && !m_pQuickSetupDlgGenICam ) )
    {
        try
        {
            const wxString dialogTitle( wxString::Format( wxT( "Quick Setup [%s - %s] (%s)" ), ConvertedString( pDev->product.readS() ).c_str(), ConvertedString( pDev->serial.readS() ).c_str(), ConvertedString( pDev->interfaceLayout.readS() ).c_str() ) );
            switch( currentDeviceInterfaceLayout )
            {
            case dilGenICam:
                m_pQuickSetupDlgGenICam = new WizardQuickSetupGenICam( this, dialogTitle );
                m_pQuickSetupDlgCurrent = m_pQuickSetupDlgGenICam;
                break;
            case dilDeviceSpecific:
                m_pQuickSetupDlgDeviceSpecific = new WizardQuickSetupDeviceSpecific( this, dialogTitle );
                m_pQuickSetupDlgCurrent = m_pQuickSetupDlgDeviceSpecific;
                break;
            default:
                break;
            }
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with the quick setup dialog on device '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str(), ConvertedString( pDev->serial.read() ).c_str() ) );
            m_pUpperToolBar->ToggleTool( miWizards_QuickSetup, false );
            RestoreGUIStateAfterQuickSetupWizard();
            return;
        }
    }
    if( m_pQuickSetupDlgCurrent )
    {
        m_pQuickSetupDlgCurrent = ( currentDeviceInterfaceLayout == dilGenICam ) ? m_pQuickSetupDlgGenICam : m_pQuickSetupDlgDeviceSpecific;
        m_DisplayAreas[0]->DisableDoubleClickAndPrunePopupMenu( true );
        m_boHandleImageTimeoutEvents = true;
        m_pQuickSetupDlgCurrent->ReferToNewDevice( pDev );
        EnsureAcquisitionState( true );
        m_pQuickSetupDlgCurrent->Show();
    }
}

//-----------------------------------------------------------------------------
void PropViewFrame::Wizard_SequencerControl( void )
//-----------------------------------------------------------------------------
{
    Device* pDev = m_pDevPropHandler->GetActiveDevice();
    if( pDev )
    {
        try
        {
            GenICam::SequencerControl sc( pDev );
            const string sequencerMode( sc.sequencerMode.readS() );
            if( sequencerMode != "Off" )
            {
                sc.sequencerMode.writeS( "Off" );
            }
            if( sc.sequencerConfigurationMode.isValid() )
            {
                sc.sequencerConfigurationMode.writeS( "On" );
            }
            WizardSequencerControl dlg( this, wxString::Format( wxT( "Sequencer Control For Device %s" ), ConvertedString( pDev->serial.readS() ).c_str() ), pDev, GetDisplayCount(), m_sequencerSetToDisplayMap );
            int result = dlg.ShowModal();
            if( result == wxID_OK )
            {
                m_sequencerSetToDisplayMap = dlg.GetSetToDisplayTable();
                if( GetDisplayCount() > 1 )
                {
                    // Enable ChunkData in order to use multiple displays
                    GenICam::ChunkDataControl cdc( pDev );
                    if( cdc.chunkModeActive.isValid() )
                    {
                        cdc.chunkModeActive.write( bTrue );
                    }
                    // Also boost update frequency in case of multiple displays
                    {
                        m_displayUpdateFrequency = std::min( static_cast< long >( 50 * GetDisplayCount() ), static_cast< long >( 200 ) );
                        StartDisplayUpdateTimer();
                    }
                }
                if( sc.sequencerConfigurationMode.isValid() )
                {
                    sc.sequencerConfigurationMode.writeS( "Off" );
                }
                sc.sequencerMode.writeS( "On" );
                CloneAndUnlockAllRequestsAndFreeRecordedSequence( false );
                m_pAcquisitionModeCombo->SetValue( m_ContinuousStr );
                EnsureAcquisitionState( true );
            }
            else
            {
                m_sequencerSetToDisplayMap = dlg.GetSetToDisplayTable();
                if( sc.sequencerConfigurationMode.isValid() )
                {
                    sc.sequencerConfigurationMode.writeS( "Off" );
                }
                sc.sequencerMode.writeS( "Off" );
            }
        }
        catch( const ImpactAcquireException& e )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with the sequencer on device '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str(), ConvertedString( pDev->serial.read() ).c_str() ) );
        }
    }
}
