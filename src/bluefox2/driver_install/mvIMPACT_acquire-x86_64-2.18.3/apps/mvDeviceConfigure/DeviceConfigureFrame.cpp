#include <apps/Common/exampleHelper.h>
#include <apps/Common/mvIcon.xpm>
#include <apps/Common/wxAbstraction.h>
#include "DeviceConfigureFrame.h"
#include "DeviceHandlerBlueDevice.h"
#include "DeviceHandlerBlueFOX.h"
#include "DeviceHandlerHYPERION.h"
#include "DeviceListCtrl.h"
#include <fstream>
#include <limits>
#include "LogOutputHandlerDlg.h"
#include <memory>
#include <string>
#include <wx/config.h>
#include <wx/ffile.h>
#include <wx/image.h>
#if wxCHECK_VERSION(2, 7, 1)
#   include <wx/platinfo.h>
#endif // #if wxCHECK_VERSION(2, 7, 1)
#include <wx/splitter.h>
#include <wx/tokenzr.h>
#include <wx/utils.h>

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
class MyApp : public wxApp
//-----------------------------------------------------------------------------
{
public:
    virtual bool OnInit();
    virtual int  OnRun();
};

// ----------------------------------------------------------------------------
// event tables and other macros for wxWidgets
// ----------------------------------------------------------------------------

// the event tables connect the wxWidgets events with the functions (event
// handlers) which process them. It can be also done at run-time, but for the
// simple menu events like this the static method is much simpler.
BEGIN_EVENT_TABLE( DeviceConfigureFrame, wxFrame )
    EVT_MENU( miHelp_About, DeviceConfigureFrame::OnHelp_About )
    EVT_MENU( miHelp_OnlineDocumentation, DeviceConfigureFrame::OnHelp_OnlineDocumentation )
    EVT_MENU( miAction_Quit, DeviceConfigureFrame::OnQuit )
    EVT_MENU( miAction_SetID, DeviceConfigureFrame::OnSetID )
    EVT_MENU( miAction_UpdateFW, DeviceConfigureFrame::OnUpdateFirmware )
    EVT_MENU( miAction_UpdateKernelDriver, DeviceConfigureFrame::OnUpdateKernelDriver )
    EVT_MENU( miAction_ConfigureLogOutput, DeviceConfigureFrame::OnConfigureLogOutput )
    EVT_MENU( miAction_UpdateDeviceList, DeviceConfigureFrame::OnUpdateDeviceList )
    EVT_MENU( miAction_UpdateDMABufferSize, DeviceConfigureFrame::OnUpdateDMABufferSize )
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    EVT_MENU( miDirectShow_RegisterAllDevices, DeviceConfigureFrame::OnRegisterAllDevicesForDirectShow )
    EVT_MENU( miDirectShow_UnregisterAllDevices, DeviceConfigureFrame::OnUnregisterAllDevicesForDirectShow )
    EVT_MENU( miDirectShow_SetFriendlyName, DeviceConfigureFrame::OnSetFriendlyNameForDirectShow )
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    EVT_MENU( miSettings_CPUIdleStatesEnabled, DeviceConfigureFrame::OnSettings_CPUIdleStatesEnabled )
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    EVT_MENU( miSettings_KeepUserSetSettingsAfterFirmwareUpdate, DeviceConfigureFrame::OnSettings_KeepUserSetSettingsAfterFirmwareUpdate )
    EVT_TIMER( wxID_ANY, DeviceConfigureFrame::OnTimer )
END_EVENT_TABLE()

// Create a new application object: this macro will allow wxWidgets to create
// the application object during program execution (it's better than using a
// static object for many reasons) and also declares the access function
// wxGetApp() which will return the reference of the right type (i.e. MyApp and
// not wxApp)
IMPLEMENT_APP( MyApp )

//-----------------------------------------------------------------------------
// 'Main program' equivalent: the program execution "starts" here
bool MyApp::OnInit()
//-----------------------------------------------------------------------------
{
    wxImage::AddHandler( new wxPNGHandler );
    SplashScreenScope splashScreenScope( wxBitmap::NewFromPNGData( mvDeviceConfigure_png, sizeof( mvDeviceConfigure_png ) ) );
    // Create the main application window
    DeviceConfigureFrame* pFrame = new DeviceConfigureFrame( wxString::Format( wxT( "mvDeviceConfigure - Tool For %s Devices(Firmware Updates, Log-Output Configuration, etc.)(%s)" ), COMPANY_NAME, VERSION_STRING ), wxDefaultPosition, wxDefaultSize, argc, argv );
    pFrame->Show( true );
    SetTopWindow( pFrame );
    // success: wxApp::OnRun() will be called which will enter the main message
    // loop and the application will run. If we returned false here, the
    // application would exit immediately.
    // Workaround for the refreshing of the Log wxTextCtrl
    pFrame->WriteLogMessage( wxT( "" ) );
    return true;
}

//-----------------------------------------------------------------------------
int MyApp::OnRun()
//-----------------------------------------------------------------------------
{
    wxApp::OnRun();
    return DeviceConfigureFrame::GetUpdateResult();
}

//=============================================================================
//================= Implementation DeviceConfigureFrame =======================
//=============================================================================

int DeviceConfigureFrame::m_updateResult = 0;

//-----------------------------------------------------------------------------
DeviceConfigureFrame::DeviceConfigureFrame( const wxString& title, const wxPoint& pos, const wxSize& size, int argc, wxChar** argv )
    : wxFrame( ( wxFrame* )NULL, wxID_ANY, title, pos, size ), m_pSystemModule( 0 ),
      m_pDevListCtrl( 0 ), m_pLogWindow( 0 ), m_logFileName(), m_lastDevMgrChangedCount( numeric_limits<unsigned int>::max() ),
      m_customFirmwareFile(), m_customFirmwarePath(), m_customGenICamFile(), m_IPv4Masks(), m_boPendingQuit( false ), m_boForceFirmwareUpdate( false )
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
      , m_boChangeProcessorIdleStates( false ), m_boEnableIdleStates( false ), m_pMISettings_CPUIdleStatesEnabled( 0 )
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
//-----------------------------------------------------------------------------
{
    try
    {
        m_pSystemModule = new mvIMPACT::acquire::GenICam::SystemModule();
    }
    catch( const ImpactAcquireException& )
    {
        // The GenICam/GenTL driver might not be installed on every system and then creating any of the objects above will raise an exception.
    }

    m_deviceHandlerFactory.Register( wxString( wxT( "GigEVisionDevice" ) ), DeviceHandler3rdParty::Create );
    m_deviceHandlerFactory.Register( wxString( wxT( "USB3VisionDevice" ) ), DeviceHandler3rdParty::Create );
    m_deviceHandlerFactory.Register( wxString( wxT( "mvBlueCOUGAR" ) ), DeviceHandlerBlueDevice::Create );
    m_deviceHandlerFactory.Register( wxString( wxT( "mvBlueLYNX" ) ), DeviceHandlerBlueDevice::Create );
    m_deviceHandlerFactory.Register( wxString( wxT( "mvBlueFOX" ) ), DeviceHandlerBlueFOX::Create );
    m_deviceHandlerFactory.Register( wxString( wxT( "mvBlueFOX3" ) ), DeviceHandlerBlueDevice::Create );
    m_deviceHandlerFactory.Register( wxString( wxT( "mvHYPERION" ) ), DeviceHandlerHYPERION::Create );

    // create the menu
    /* Keyboard shortcuts:
            CTRL+C : Configure Log Output
            CTRL+F : Update Firmware
        ALT+CTRL+F : Set Friendly Name
            CTRL+I : Enable/Disable CPU Idle states
            CTRL+P : Enable/Disable Persistent UserSet Settings
            CTRL+K : Update Kernel Driver
            CTRL+D : Update Permanent DMA Buffer
            CTRL+R : Register All Devices(for DirectShow)
            CTRL+S : Set ID
            CTRL+U : Unregister All Devices(for DirectShow)
                F5 : Update Device List
               F12 : Online Documentation
        ALT+     X : Exit
    */

    wxMenu* pMenuAction = new wxMenu;
    m_pMIActionSetID = pMenuAction->Append( miAction_SetID, wxT( "&Set ID\tCTRL+S" ) );
    m_pMIActionUpdateFW = pMenuAction->Append( miAction_UpdateFW, wxT( "Update &Firmware\tCTRL+F" ) );
    m_pMIActionUpdateKernelDriver = pMenuAction->Append( miAction_UpdateKernelDriver, wxT( "Update &Kernel Driver\tCTRL+K" ) );
    m_pMIActionUpdateDMABufferSize = pMenuAction->Append( miAction_UpdateDMABufferSize, wxT( "Update Permanent &DMA Buffer Size\tCTRL+D" ) );

    pMenuAction->AppendSeparator();
    pMenuAction->Append( miAction_ConfigureLogOutput, wxT( "&Configure Log Output\tCTRL+C" ) );
    m_pMIActionUpdateDeviceList = pMenuAction->Append( miAction_UpdateDeviceList, wxT( "&Update Device List\tF5" ) );
    pMenuAction->AppendSeparator();

    pMenuAction->Append( miAction_Quit, wxT( "E&xit\tALT+X" ) );

    wxMenu* pMenuHelp = new wxMenu;
    pMenuHelp->Append( miHelp_About, wxT( "About mvDeviceConfigure\tF1" ) );
    pMenuHelp->Append( miHelp_OnlineDocumentation, wxT( "Online Documentation...\tF12" ) );

    // create a menu bar...
    wxMenuBar* pMenuBar = new wxMenuBar;
    // ... add all the menu items...
    pMenuBar->Append( pMenuAction, wxT( "&Action" ) );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    wxMenu* menuDirectShow = new wxMenu;
    menuDirectShow->Append( miDirectShow_RegisterAllDevices, wxT( "&Register All Devices\tCTRL+R" ) );
    menuDirectShow->Append( miDirectShow_UnregisterAllDevices, wxT( "&Unregister All Devices\tCTRL+U" ) );
    menuDirectShow->AppendSeparator();
    m_pMIDirectShow_SetFriendlyName = menuDirectShow->Append( miDirectShow_SetFriendlyName, wxT( "Set Friendly Name\tALT+CTRL+F" ) );
    pMenuBar->Append( menuDirectShow, wxT( "&DirectShow" ) );
    m_DSDevMgr.create( this );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    wxMenu* pMenuSettings = new wxMenu;
    m_pMISettings_KeepUserSetSettingsAfterFirmwareUpdate = pMenuSettings->Append( miSettings_KeepUserSetSettingsAfterFirmwareUpdate, wxT( "&Persistent UserSet Settings\tCTRL+P" ), wxT( "" ), wxITEM_CHECK );
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    bool boAllowCPUIdleStateConfiguration = true;
#   if wxCHECK_VERSION(2, 7, 1)
    wxPlatformInfo platformInfo;
    if( platformInfo.GetOperatingSystemId() == wxOS_WINDOWS_NT )
    {
        if( ( platformInfo.GetOSMajorVersion() > 6 ) ||
            ( ( platformInfo.GetOSMajorVersion() == 6 ) && ( platformInfo.GetOSMinorVersion() >= 2 ) ) )
        {
            boAllowCPUIdleStateConfiguration = false;
        }
    }
#   endif // #if wxCHECK_VERSION(2, 7, 1)
    if( boAllowCPUIdleStateConfiguration )
    {
        m_pMISettings_CPUIdleStatesEnabled = pMenuSettings->Append( miSettings_CPUIdleStatesEnabled, wxT( "CPU &Idle States Enabled\tCTRL+I" ), wxT( "" ), wxITEM_CHECK );
        bool boValue = false;
        GetPowerState( boValue );
        m_pMISettings_CPUIdleStatesEnabled->Check( boValue );
    }
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    pMenuBar->Append( pMenuSettings, wxT( "&Settings" ) );
    pMenuBar->Append( pMenuHelp, wxT( "&Help" ) );
    // ... and attach this menu bar to the frame
    SetMenuBar( pMenuBar );

    // define the applications icon
    wxIcon icon( mvIcon_xpm );
    SetIcon( icon );

    wxPanel* pPanel = new wxPanel( this, wxID_ANY );
    wxSplitterWindow* pHorizontalSplitter = new wxSplitterWindow( pPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );
    wxBoxSizer* pSizer = new wxBoxSizer( wxVERTICAL );
    pSizer->Add( pHorizontalSplitter, wxSizerFlags( 5 ).Expand() );

    m_pDevListCtrl = new DeviceListCtrl( pHorizontalSplitter, LIST_CTRL, wxDefaultPosition, wxDefaultSize, wxLC_REPORT | wxLC_SINGLE_SEL | wxBORDER_NONE, this );
    m_pDevListCtrl->InsertColumn( lcFamily, wxT( "Family" ) );
    m_pDevListCtrl->InsertColumn( lcProduct, wxT( "Product" ) );
    m_pDevListCtrl->InsertColumn( lcSerial, wxT( "Serial" ) );
    m_pDevListCtrl->InsertColumn( lcState, wxT( "State" ) );
    m_pDevListCtrl->InsertColumn( lcFWVersion, wxT( "Firmware Version" ) );
    m_pDevListCtrl->InsertColumn( lcKernelDriver, wxT( "Kernel Driver" ) );
    m_pDevListCtrl->InsertColumn( lcDeviceID, wxT( "Device ID" ) );
    m_pDevListCtrl->InsertColumn( lcAllocatedDMABuffer, wxT( "Allocated DMA Buffer(KB)" ) );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    m_pDevListCtrl->InsertColumn( lcDSRegistered, wxT( "Registered For DirectShow" ) );
    m_pDevListCtrl->InsertColumn( lcDSFriendlyName, wxT( "DirectShow Friendly Name" ) );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

    m_pLogWindow = new wxTextCtrl( pHorizontalSplitter, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxBORDER_NONE | wxTE_RICH | wxTE_READONLY );

    // splitter...
    pHorizontalSplitter->SetSashGravity( 0.8 );
    pHorizontalSplitter->SetMinimumPaneSize( 50 );

    BuildList();

    // restore previous state
    const wxRect defaultRect( 0, 0, 640, 480 );
    bool boMaximized = false;
    const wxRect rect = FramePositionStorage::Load( defaultRect, boMaximized );
    wxConfigBase* pConfig = wxConfigBase::Get();
    bool boUserSetPersistence = pConfig->Read( wxT( "/MainFrame/Settings/PersistentUserSetSettings" ), 1l ) != 0;

    wxString processedParameters;
    std::vector<wxString> commandLineErrors;
    for( int i = 1; i < argc; i++ )
    {
        const wxString param( argv[i] );
        const wxString key = param.BeforeFirst( wxT( '=' ) );
        const wxString value = param.AfterFirst( wxT( '=' ) );
        if( key.IsEmpty() )
        {
            commandLineErrors.push_back( wxString::Format( wxT( "%s: Invalid command line parameter: '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), param.c_str() ) );
        }
        else
        {
            if( ( key == wxT( "update_fw" ) ) || ( key == wxT( "ufw" ) ) )
            {
                GetConfigurationEntry( value )->second.boUpdateFW_ = true;
            }
            else if( ( key == wxT( "update_fw_file" ) ) || ( key == wxT( "ufwf" ) ) )
            {
                GetConfigurationEntriesFromFile( value );
            }
            else if( ( key == wxT( "custom_genicam_file" ) ) || ( key == wxT( "cgf" ) ) )
            {
                m_customGenICamFile = value;
            }
            else if( key == wxT( "fw_path" ) )
            {
                m_customFirmwarePath = value;
            }
            else if( key == wxT( "fw_file" ) )
            {
                m_customFirmwareFile = value;
            }
            else if( ( key == wxT( "update_kd" ) ) || ( key == wxT( "ukd" ) ) )
            {
                GetConfigurationEntry( value )->second.boUpdateKernelDriver_ = true;
            }
            else if( key == wxT( "ipv4_mask" ) )
            {
                wxStringTokenizer tokenizer( value, wxString( wxT( ";" ) ), wxTOKEN_STRTOK );
                while( tokenizer.HasMoreTokens() )
                {
                    std::string token( tokenizer.GetNextToken().mb_str() );
                    m_IPv4Masks.push_back( token );
                }
            }
            else if( ( key == wxT( "log_file" ) ) || ( key == wxT( "lf" ) ) )
            {
                m_logFileName = value;
            }
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
            else if( ( key == wxT( "set_processor_idle_states" ) ) || ( key == wxT( "spis" ) ) )
            {
                unsigned long conversionResult = 0;
                if( value.ToULong( &conversionResult ) )
                {
                    m_boChangeProcessorIdleStates = true;
                    m_boEnableIdleStates = conversionResult != 0;
                }
            }
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
            else if( ( key == wxT( "setid" ) ) || ( key == wxT( "id" ) ) )
            {
                wxString serial( value.BeforeFirst( wxT( '.' ) ) );
                wxString idString( value.AfterFirst( wxT( '.' ) ) );
                if( serial.IsEmpty() || idString.IsEmpty() )
                {
                    commandLineErrors.push_back( wxString::Format( wxT( "%s: Invalid command line parameter: '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), param.c_str() ) );
                }
                else
                {
                    long id;
                    if( !idString.ToLong( &id ) )
                    {
                        commandLineErrors.push_back( wxString::Format( wxT( "%s: Invalid command line parameter: '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), param.c_str() ) );
                    }
                    else
                    {
                        GetConfigurationEntry( serial )->second.boSetDeviceID_ = true;
                        GetConfigurationEntry( serial )->second.deviceID_ = static_cast<int>( id );
                    }
                }
            }
            else if( ( key == wxT( "quit" ) ) || ( key == wxT( "q" ) ) )
            {
                m_boPendingQuit = true;
            }
            else if( ( key == wxT( "force" ) ) || ( key == wxT( "f" ) ) )
            {
                m_boForceFirmwareUpdate = true;
            }
            else if( key == wxT( "set_userset_persistence" ) || key == wxT( "sup" ) )
            {
                unsigned long conversionResult = 0;
                if( value.ToULong( &conversionResult ) )
                {
                    // overwrite the value stored by the application (Registry on Windows, a .<nameOfTheApplication> file on other platforms
                    boUserSetPersistence = ( conversionResult != 0 ) ? true : false;
                }
            }
            else
            {
                commandLineErrors.push_back( wxString::Format( wxT( "%s: Invalid command line parameter: '%s'.\n" ), ConvertedString( __FUNCTION__ ).c_str(), param.c_str() ) );
            }
        }
        processedParameters += param;
        processedParameters.Append( wxT( ' ' ) );
    }

    const wxTextAttr boldStyle( GetBoldStyle( m_pLogWindow ) );
    WriteLogMessage( wxT( "Available command line options:\n" ), boldStyle );
    WriteLogMessage( wxT( "'setid'                     or 'id'   to update the firmware of one or many devices(syntax: 'id=<serial>.id' or 'id=<product>.id').\n" ) );
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    WriteLogMessage( wxT( "'set_processor_idle_states' or 'spis' to change the C1, C2 and C3 states for ALL processors in the current system(syntax: 'spis=1' or 'spis=0').\n" ) );
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    WriteLogMessage( wxT( "'set_userset_persistence'   or 'sup'  to set the persistency of UserSet settings during firmware updates (syntax: 'sup=1' or 'sup=0').\n" ) );
    WriteLogMessage( wxT( "'update_fw'                 or 'ufw'  to update the firmware of one or many devices.\n" ) );
    WriteLogMessage( wxT( "'update_fw_file'            or 'ufwf' to update the firmware of one or many devices. Pass a full path to a text file that contains a serial number or a product type per line.\n" ) );
    WriteLogMessage( wxT( "'custom_genicam_file'       or 'cgf'  to specify a custom GenICam file to be used to open devices for firmware updates. This can be useful when the actual XML on the device is damaged/invalid.\n" ) );
    WriteLogMessage( wxT( "'update_kd'                 or 'ukd'  to update the kernel driver of one or many devices.\n" ) );
    WriteLogMessage( wxT( "'ipv4_mask'                           to specify an IPv4 address mask to use as a filter for the selected update operations. Multiple masks can be passed here separated by semicolons.\n" ) );
    WriteLogMessage( wxT( "'fw_file'                             to specify a custom name for the firmware file to use.\n" ) );
    WriteLogMessage( wxT( "'fw_path'                             to specify a custom path for the firmware files.\n" ) );
    WriteLogMessage( wxT( "'log_file'                  or 'lf'   to specify a log file storing the content of this text control upon application shutdown.\n" ) );
    WriteLogMessage( wxT( "'quit'                      or 'q'    to end the application automatically after all updates have been applied.\n" ) );
    WriteLogMessage( wxT( "'force'                     or 'f'    to force a firmware update in unattended mode, even if it isn't a newer version.\n" ) );
    WriteLogMessage( wxT( "'*' can be used as a wildcard, devices will be searched by serial number AND by product. The application will first try to locate a device with a serial number matching the specified string and then (if no suitable device is found) a device with a matching product string.\n" ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "The number of commands that can be passed to the application is not limited.\n" ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "Usage examples:\n" ) );
    WriteLogMessage( wxT( "mvDeviceConfigure ufw=BF000666 (will update the firmware of a mvBlueFOX with the serial number BF000666)\n" ) );
    WriteLogMessage( wxT( "mvDeviceConfigure update_fw=BF* (will update the firmware of ALL mvBlueFOX devices in the current system)\n" ) );
    WriteLogMessage( wxT( "mvDeviceConfigure update_fw=mvBlueFOX-2* lf=output.txt quit (will update the firmware of ALL mvBlueFOX-2 devices in the current system, then will store a log file of the executed operations and afterwards will terminate the application.)\n" ) );
    WriteLogMessage( wxT( "mvDeviceConfigure setid=BF000666.5 (will assign the device ID '5' to a mvBlueFOX with the serial number BF000666)\n" ) );
    WriteLogMessage( wxT( "mvDeviceConfigure ufw=* (will update the firmware of every device in the system)\n" ) );
    WriteLogMessage( wxT( "mvDeviceConfigure ufw=BF000666 ufw=BF000667 (will update the firmware of 2 mvBlueFOX cameras)\n" ) );
    WriteLogMessage( wxT( "mvDeviceConfigure ipv4_mask=169.254.*;192.168.100* update_fw=GX* (will update the firmware of all mvBlueCOUGAR-X devices with a valid IPv4 address that starts with '169.254.' or '192.168.100.')\n" ) );
    WriteLogMessage( wxT( "\n" ) );
    if( argc > 1 )
    {
        WriteLogMessage( wxString::Format( wxT( "Processed command line: %s.\n" ), processedParameters.c_str() ), boldStyle );
        WriteLogMessage( wxT( "\n" ) );
    }
    if( commandLineErrors.empty() == false )
    {
        vector<wxString>::size_type cnt = commandLineErrors.size();
        for( vector<wxString>::size_type i = 0; i < cnt; i++ )
        {
            WriteErrorMessage( commandLineErrors[i] );
        }
        WriteLogMessage( wxT( "\n" ) );
        m_updateResult = DeviceHandler::urInvalidParameter;
    }
    WriteLogMessage( wxT( "Usage:\n" ), boldStyle );
    wxTextAttr blueStyle( wxColour( 80, 100, 255 ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "Right click on a device entry to get a menu with available options for the selected device\n" ), blueStyle );
    WriteLogMessage( wxT( "If a menu entry is disabled the underlying feature is not available for the selected device.\n" ), blueStyle );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "Double click on a list entry to start live acquisition from this device in a new instance of wxPropView (wxPropView must be locatable via " ), blueStyle );
    WriteLogMessage( wxT( "the systems path or must reside in the same directory as mvDeviceConfigure!)\n" ), blueStyle );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "Menu entries under 'Action' will be enabled and disabled whenever the currently selected device changes.\n" ), blueStyle );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "To modify the way log outputs are created select 'Action -> Configure Log Output'.\n" ), blueStyle );
    WriteLogMessage( wxT( "\n" ) );
    if( !m_devicesToConfigure.empty() )
    {
        WriteLogMessage( wxT( "Devices that await configuration:\n" ), boldStyle );
        std::map<wxString, DeviceConfigurationData>::const_iterator it = m_devicesToConfigure.begin();
        const std::map<wxString, DeviceConfigurationData>::const_iterator itEND = m_devicesToConfigure.end();
        while( it != itEND )
        {
            const wxString IDString( it->second.boSetDeviceID_ ? wxString::Format( wxT( "yes(%d)" ), it->second.deviceID_ ) : wxT( "no" ) );
            WriteLogMessage( wxString::Format( wxT( "%s(kernel driver update: %s, firmware update: %s, assigning device ID: %s).\n" ),
                                               it->first.c_str(),
                                               it->second.boUpdateKernelDriver_ ? wxT( "yes" ) : wxT( "no" ),
                                               it->second.boUpdateFW_ ? wxT( "yes" ) : wxT( "no" ),
                                               IDString.c_str() ) );
            ++it;
        }
        WriteLogMessage( wxT( "\n" ) );
    }

    m_pMISettings_KeepUserSetSettingsAfterFirmwareUpdate->Check( boUserSetPersistence );

    if( !m_devicesToConfigure.empty() ||
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
        m_boChangeProcessorIdleStates ||
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
        m_boPendingQuit )
    {
        m_timer.SetOwner( this, teTimer );
        m_timer.Start( TIMER_PERIOD );
    }

    SetSizeHints( defaultRect.GetWidth(), defaultRect.GetHeight() );
    pPanel->SetSizer( pSizer );

    SetSize( rect );
    Maximize( boMaximized );
    UpdateMenu( -1 );
    UpdateDeviceList();

    pHorizontalSplitter->SplitHorizontally( m_pDevListCtrl, m_pLogWindow );

    m_listUpdateTimer.SetOwner( this, teListUpdate );
    m_listUpdateTimer.Start( TIMER_PERIOD );
}

//-----------------------------------------------------------------------------
DeviceConfigureFrame::~DeviceConfigureFrame()
//-----------------------------------------------------------------------------
{
    // store the current state of the application
    FramePositionStorage::Save( this );
    if( m_listUpdateTimer.IsRunning() )
    {
        m_listUpdateTimer.Stop();
    }
    if( m_timer.IsRunning() )
    {
        m_timer.Stop();
    }

    if( !m_logFileName.IsEmpty() )
    {
        wxFFile file( m_logFileName, wxT( "w" ) );
        if( file.IsOpened() )
        {
            file.Write( m_pLogWindow->GetValue() );
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "ERROR!!! Could not create log file '%s'.\n" ), m_logFileName.c_str() ) );
        }
    }
    // when we e.g. try to write config stuff on a read-only file system the result can
    // be an annoying message box. Therefore we switch off logging now, as otherwise higher level
    // clean up code might produce error messages
    wxLog::EnableLogging( false );
    wxConfigBase* pConfig = wxConfigBase::Get();
    pConfig->Write( wxT( "/MainFrame/Settings/PersistentUserSetSettings" ), m_pMISettings_KeepUserSetSettingsAfterFirmwareUpdate->IsChecked() );
    delete m_pSystemModule;
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::ActivateDeviceIn_wxPropView( int deviceIndex )
//-----------------------------------------------------------------------------
{
    wxString commandString( wxT( "wxPropView device=" ) );
    commandString.Append( ConvertedString( m_devMgr.getDevice( deviceIndex )->serial.read() ) );
    commandString.Append( wxT( " live=1" ) );
    ::wxExecute( commandString );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::BuildList( void )
//-----------------------------------------------------------------------------
{
    typedef map<int, int> IntIntMap;
    typedef pair<int, int> IntIntPair;
    typedef map<string, IntIntMap> String_IntIntMap_Map;
    typedef pair<string, IntIntMap> String_IntIntMap_Pair;
    String_IntIntMap_Map devMap;

    m_lastDevMgrChangedCount = m_devMgr.changedCount();
    m_pDevListCtrl->DeleteAllItems();

#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    DirectShowDeviceManager::DSDeviceList registeredDSDevices;
    m_DSDevMgr.getDeviceList( registeredDSDevices );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

    unsigned int devCnt = m_devMgr.deviceCount();
    map<string, int> productFirmwareTable;
    for( unsigned int i = 0; i < devCnt; i++ )
    {
        const string product( m_devMgr.getDevice( i )->product.read() );
        const int firmwareVersion( m_devMgr.getDevice( i )->firmwareVersion.read() );
        map<string, int>::iterator it = productFirmwareTable.find( product );
        if( it == productFirmwareTable.end() )
        {
            productFirmwareTable.insert( make_pair( product, firmwareVersion ) );
        }
        else if( it->second < firmwareVersion )
        {
            it->second = firmwareVersion;
        }
    }
    for( unsigned int i = 0; i < devCnt; i++ )
    {
        Device* pDev = m_devMgr[i];
        long index = m_pDevListCtrl->InsertItem( i, ConvertedString( pDev->family.read() ) );
        pair<String_IntIntMap_Map::iterator, bool> p = devMap.insert( String_IntIntMap_Pair( pDev->family.read(), IntIntMap() ) );
        if( !p.second )
        {
            IntIntMap::iterator it = p.first->second.find( pDev->deviceID.read() );
            if( it != p.first->second.end() )
            {
                // mark items that have a conflicting device ID
                m_pDevListCtrl->SetItemTextColour( index, *wxRED );
                m_pDevListCtrl->SetItemTextColour( it->second, *wxRED );
                WriteErrorMessage( wxString::Format( wxT( "WARNING: The device in row %ld(%s) has the same ID as the device in row %d(%s) belonging to the same device family. This should be resolved.\n" ), index, ConvertedString( pDev->serial.read() ).c_str(), it->second, ConvertedString( m_devMgr[it->second]->serial.read() ).c_str() ) );
            }
            else
            {
                p.first->second.insert( IntIntPair( pDev->deviceID.read(), index ) );
            }
        }
        else
        {
            p.first->second.insert( IntIntPair( pDev->deviceID.read(), index ) );
        }
        const string product( pDev->product.read() );
        m_pDevListCtrl->SetItem( index, lcProduct, ConvertedString( product ) );
        m_pDevListCtrl->SetItem( index, lcSerial, ConvertedString( pDev->serial.read() ) );
        m_pDevListCtrl->SetItem( index, lcState, ConvertedString( pDev->state.readS() ) );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
        {
            DirectShowDeviceManager::DSDeviceList::const_iterator it = registeredDSDevices.find( pDev->serial.read() );
            if( it == registeredDSDevices.end() )
            {
                m_pDevListCtrl->SetItem( index, lcDSRegistered, wxString( wxT( "no" ) ) );
                m_pDevListCtrl->SetItem( index, lcDSFriendlyName, wxEmptyString );
            }
            else
            {
                m_pDevListCtrl->SetItem( index, lcDSRegistered, wxString( wxT( "yes" ) ) );
                m_pDevListCtrl->SetItem( index, lcDSFriendlyName, ConvertedString( it->second ) );
            }
        }
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
        m_pDevListCtrl->SetItemData( index, i );
        if( pDev->state.read() != dsPresent )
        {
            wxColour col( wxT( "LIGHT GREY" ) );
            m_pDevListCtrl->SetItemBackgroundColour( index, col );
        }

        auto_ptr<DeviceHandler> pHandler( m_deviceHandlerFactory.CreateObject( ConvertedString( pDev->family.read() ), pDev ) );

        bool boUpdateAvailable = false;
        Version latestFirmware;
        wxString latestFirmwareS;
        if( pHandler.get() && ( pHandler->GetLatestFirmwareVersion( latestFirmware ) == DeviceHandler::urOperationSuccessful ) )
        {
            latestFirmwareS = wxString::Format( wxT( "(Version %ld.%ld.%ld.%ld)" ), latestFirmware.major_, latestFirmware.minor_, latestFirmware.subMinor_, latestFirmware.release_ );
            boUpdateAvailable = true;
        }
        else
        {
            map<string, int>::const_iterator it = productFirmwareTable.find( product );
            if( it != productFirmwareTable.end() )
            {
                const int firmwareVersion( pDev->firmwareVersion.read() );
                if( firmwareVersion < it->second )
                {
                    m_pDevListCtrl->SetItemTextColour( index, *wxRED );
                    boUpdateAvailable = true;
                }
            }
        }

        const wxString updateNotification( boUpdateAvailable ? wxString::Format( wxT( "(UPDATE AVAILABLE%s)" ), latestFirmwareS.c_str() ) : wxT( "" ) );
        m_pDevListCtrl->SetItem( index, lcFWVersion, wxString::Format( wxT( "%s%s" ), ConvertedString( pDev->firmwareVersion.readS() ).c_str(), updateNotification.c_str() ) );
        if( boUpdateAvailable )
        {
            wxTextAttr blueStyle( wxColour( 80, 100, 255 ) );
            WriteLogMessage( wxString::Format( wxT( "NOTE: The device in row %ld(%s, %s) uses an outdated firmware version.\n" ), index, ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str() ), blueStyle );
            m_pDevListCtrl->SetItemBackgroundColour( index, wxColour( 200, 220, 255 ) );
        }

        string kernelDriverName;
        bool boNewerDriverAvailable = false;
        bool boFeatureSupported = false;
        if( pHandler.get() )
        {
            boFeatureSupported = pHandler->SupportsKernelDriverUpdate( boNewerDriverAvailable, kernelDriverName );
        }
        wxString wxKernelDriverName;
        if( kernelDriverName.empty() )
        {
            wxKernelDriverName = wxString( wxT( "unsupported" ) );
        }
        else
        {
            wxKernelDriverName = ConvertedString( kernelDriverName );
        }
        m_pDevListCtrl->SetItem( index, lcKernelDriver, wxKernelDriverName );
        int currentDMABufferSize_kB = 0;
        if( pHandler.get() && pHandler->SupportsDMABufferSizeUpdate( &currentDMABufferSize_kB ) )
        {
            m_pDevListCtrl->SetItem( index, lcAllocatedDMABuffer, wxString::Format( wxT( "%d kB" ), currentDMABufferSize_kB ) );
        }
        else
        {
            m_pDevListCtrl->SetItem( index, lcAllocatedDMABuffer, wxString( wxT( "unsupported" ) ) );
        }
        if( boFeatureSupported && boNewerDriverAvailable )
        {
            wxColour col( 150, 80, 150 );
            wxTextAttr oldKernelDriverStyle( col );
            WriteLogMessage( wxString::Format( wxT( "WARNING: The device in row %ld(%s, %s) is not running with the latest kernel driver. To update the kernel driver for each device of that family currently present right click on the device.\n" ), index, ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str() ), oldKernelDriverStyle );
            m_pDevListCtrl->SetItemBackgroundColour( index, wxColour( 255, 200, 255 ) );
        }
        if( pDev->state.read() == dsUnreachable )
        {
            m_pDevListCtrl->SetItemTextColour( index, *wxRED );
            WriteErrorMessage( wxString::Format( wxT( "WARNING: device in row %ld(%s, %s)' is currently reported as 'unreachable'. This indicates a compliant device has been detected but for some reasons cannot be used at the moment. For GEV devices this might be a result of an incorrect network configuration (run 'mvIPConfigure' with 'Advanced Device Discovery' enabled to get more information on this). For U3V devices this might be a result of the device being bound to a different (third party) U3V device driver. Refer to the documentation in section 'Troubleshooting' for additional information regarding this issue.\n" ),
                                                 index, ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str() ) );
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
                    const int64_type interfaceCount( GetGenTLInterfaceCount() );
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
                                m_pDevListCtrl->SetItemTextColour( index, *wxRED );
                                WriteErrorMessage( wxString::Format( wxT( "WARNING: GigE Vision device '%s' in row %ld will currently not work properly as its network configuration is invalid in the current setup. Run 'mvIPConfigure' with 'Advanced Device Discovery' enabled to get more information on this).\n" ),
                                                                     ConvertedString( im.deviceID.readS() ).c_str(), index ) );
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





        m_pDevListCtrl->SetItem( index, lcDeviceID, ConvertedString( pDev->deviceID.readS() ) );
    }
    m_pDevListCtrl->SetColumnWidth( lcFamily, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
    m_pDevListCtrl->SetColumnWidth( lcProduct, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
    m_pDevListCtrl->SetColumnWidth( lcSerial, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
    m_pDevListCtrl->SetColumnWidth( lcState, ( ( devCnt > 0 ) ? wxLIST_AUTOSIZE : wxLIST_AUTOSIZE_USEHEADER ) );
    m_pDevListCtrl->SetColumnWidth( lcFWVersion, wxLIST_AUTOSIZE_USEHEADER );
    m_pDevListCtrl->SetColumnWidth( lcKernelDriver, wxLIST_AUTOSIZE_USEHEADER );
    m_pDevListCtrl->SetColumnWidth( lcDeviceID, wxLIST_AUTOSIZE_USEHEADER );
    m_pDevListCtrl->SetColumnWidth( lcAllocatedDMABuffer, wxLIST_AUTOSIZE_USEHEADER );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    m_pDevListCtrl->SetColumnWidth( lcDSRegistered, wxLIST_AUTOSIZE_USEHEADER );
    m_pDevListCtrl->SetColumnWidth( lcDSFriendlyName, wxLIST_AUTOSIZE_USEHEADER );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    WriteLogMessage( wxT( "\n" ) );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::GetConfigurationEntriesFromFile( const wxString& fileName )
//-----------------------------------------------------------------------------
{
    std::ifstream file( fileName.mb_str() );
    if( !file.good() )
    {
        WriteErrorMessage( wxString::Format( wxT( "Could not open file '%s' for obtaining device update information." ), fileName.c_str() ) );
        return;
    }

    while( !file.eof() )
    {
        std::string l;
        getline( file, l );
        // remove all white-spaces, TABs and newlines and also just consider everything until the first ',' to get the product name as the file
        // format allows to specify additional parameters after a ',' (e.g. 'mvBlueCOUGAR-XD126aC, serial=GX200376, linkspeed=1000000000, firmwareFile=mvBlueCOUGAR-X.mvu')
        const wxString line = ConvertedString( l ).Strip( wxString::both );
        const wxString product = line.BeforeFirst( wxT( ',' ) );
        if( product.IsEmpty() == false )
        {
            if( product.Contains( wxT( "mvBlueCOUGAR-S" ) ) == false )
            {
                std::map<wxString, DeviceConfigurationData>::iterator it = GetConfigurationEntry( product );
                it->second.boUpdateFW_ = true;
                wxStringTokenizer tokenizer( line.AfterFirst( wxT( ',' ) ), wxString( wxT( "," ) ), wxTOKEN_STRTOK );
                while( tokenizer.HasMoreTokens() )
                {
                    const wxString token( tokenizer.GetNextToken().Strip( wxString::both ) );
                    const wxString key = token.BeforeFirst( wxT( '=' ) ).MakeLower();
                    const wxString value = token.AfterFirst( wxT( '=' ) );
                    if( key == wxT( "firmwarefile" ) )
                    {
                        it->second.customFirmwareFileName_ = value;
                        break;
                    }
                }
            }
            else if( product.StartsWith( wxT( "mvBlueCOUGAR-S" ) ) == true )
            {
                WriteErrorMessage( wxString::Format( wxT( "Devices of type '%s' do not support this update mechanism as there is no way to check if the firmware on the device is the same as the one in the update archive and this could result in a lot of redundant update cycles in an automatic environment.\n" ), product.c_str() ) );
            }
        }
    }
}

//-----------------------------------------------------------------------------
std::map<wxString, DeviceConfigureFrame::DeviceConfigurationData>::iterator DeviceConfigureFrame::GetConfigurationEntry( const wxString& value )
//-----------------------------------------------------------------------------
{
    std::map<wxString, DeviceConfigurationData>::iterator it = m_devicesToConfigure.find( value );
    if( it == m_devicesToConfigure.end() )
    {
        string valueANSI( value.mb_str() );
        it = m_devicesToConfigure.insert( make_pair( value, DeviceConfigurationData( valueANSI ) ) ).first;
    }
    return it;
}

//-----------------------------------------------------------------------------
int64_type DeviceConfigureFrame::GetGenTLInterfaceCount( void ) const
//-----------------------------------------------------------------------------
{
    if( !m_pSystemModule )
    {
        return 0;
    }

    const int64_type interfaceSelectorMax( m_pSystemModule->interfaceSelector.getMaxValue() );
    if( ( interfaceSelectorMax == 0 ) &&
        m_pSystemModule->interfaceID.readS().empty() )
    {
        // a system module without an interface will report a max. 'interfaceSelector' value of 0 and an empty string for 'interfaceID'
        return 0;
    }

    return interfaceSelectorMax + 1;
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnHelp_About( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxBoxSizer* pTopDownSizer;
    wxDialog dlg( this, wxID_ANY, wxString( _( "About mvDeviceConfigure" ) ) );
    wxIcon icon( mvIcon_xpm );
    dlg.SetIcon( icon );
    pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    wxStaticText* pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT( "mvDeviceConfigure - Tool For %s Devices(Firmware Updates, Log-Output Configuration, etc.)(%s)" ), COMPANY_NAME, VERSION_STRING ) );
    pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
    pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT( "(C) 2005 - %s by %s" ), CURRENT_YEAR, COMPANY_NAME ) );
    pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
    pText = new wxStaticText( &dlg, wxID_ANY, wxString::Format( wxT( "Version %s" ), VERSION_STRING ) );
    pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
    AddSupportInfo( &dlg, pTopDownSizer );
    AddwxWidgetsInfo( &dlg, pTopDownSizer );
    pText = new wxStaticText( &dlg, wxID_ANY, wxT( "The expat wrapper class used internally has been written by Descartes Systems Sciences, Inc." ) );
    pTopDownSizer->Add( pText, 0, wxALL | wxALIGN_CENTER, 5 );
    AddSourceInfo( &dlg, pTopDownSizer );
    wxButton* pBtnOK = new wxButton( &dlg, wxID_OK, wxT( "OK" ) );
    pBtnOK->SetDefault();
    pTopDownSizer->Add( pBtnOK, 0, wxALL | wxALIGN_RIGHT, 15 );
    dlg.SetSizer( pTopDownSizer );
    pTopDownSizer->Fit( &dlg );
    dlg.ShowModal();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnConfigureLogOutput( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    LogOutputHandlerDlg dlg( this, m_devMgr, m_debugData );
    dlg.ShowModal();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnQuit( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    // true is to force the frame to close
    Close( true );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnSetID( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    SetID( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnTimer( wxTimerEvent& e )
//-----------------------------------------------------------------------------
{
    switch( e.GetId() )
    {
    case teListUpdate:
        if( m_lastDevMgrChangedCount == m_devMgr.changedCount() )
        {
            return;
        }
        BuildList();
        break;
    case teTimer:
        m_timer.Stop();
        if( !m_devicesToConfigure.empty() )
        {
            std::map<wxString, DeviceConfigurationData>::const_iterator it = m_devicesToConfigure.begin();
            const std::map<wxString, DeviceConfigurationData>::const_iterator itEND = m_devicesToConfigure.end();
            const vector<string>::size_type IPv4MaskCnt = m_IPv4Masks.size();
            while( it != itEND )
            {
                int i = 0;
                Device* pDev = 0;
                while( ( ( pDev = m_devMgr.getDeviceBySerial( it->second.searchToken_, i ) ) != 0 ) ||
                       ( ( pDev = m_devMgr.getDeviceByProduct( it->second.searchToken_, i ) ) != 0 ) )
                {
                    if( !m_IPv4Masks.empty() )
                    {
                        DeviceComponentLocator locator( pDev->hDev() );
                        PropertyI deviceIPAddress;
                        locator.bindComponent( deviceIPAddress, "DeviceIPAddress" );
                        if( deviceIPAddress.isValid() )
                        {
                            const string IPAddress( deviceIPAddress.readS() );
                            if( IPAddress != string( "Unavailable" ) )
                            {
                                bool boIgnore = true;
                                for( vector<string>::size_type j = 0; j < IPv4MaskCnt; j++ )
                                {
                                    if( match( IPAddress, m_IPv4Masks[j], '*' ) == 0 )
                                    {
                                        boIgnore = false;
                                        break;
                                    }
                                }
                                if( boIgnore == true )
                                {
                                    WriteLogMessage( wxString::Format( wxT( "Device '%s'(%s) will not be configured as it IPv4 address(%s) does not match any of the specified masks.\n" ), ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str(), ConvertedString( IPAddress ).c_str() ) );
                                    ++i;
                                    continue;
                                }
                            }
                        }
                    }
                    if( it->second.boSetDeviceID_ )
                    {
                        RefreshApplicationExitCode( SetID( pDev, it->second.deviceID_ ) );
                    }
                    if( it->second.boUpdateKernelDriver_ )
                    {
                        RefreshApplicationExitCode( UpdateKernelDriver( pDev, true ) );
                    }
                    if( it->second.boUpdateFW_ )
                    {
                        RefreshApplicationExitCode( UpdateFirmware( pDev, true, m_pMISettings_KeepUserSetSettingsAfterFirmwareUpdate->IsChecked(), it->second.customFirmwareFileName_ ) );
                    }
                    ++i;
                }
                if( i == 0 )
                {
                    WriteErrorMessage( wxString::Format( wxT( "No device found, that matches the search criterion %s.\n" ), it->first.c_str() ) );
                }
                ++it;
            }
        }
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
        if( m_boChangeProcessorIdleStates )
        {
            if( m_pMISettings_CPUIdleStatesEnabled )
            {
                if( SetPowerState( m_boEnableIdleStates ) )
                {
                    m_pMISettings_CPUIdleStatesEnabled->Check( m_boEnableIdleStates );
                }
            }
            else
            {
                WriteErrorMessage( wxT( "Changing the processor idle state configuration is not supported on this platform.\n" ) );
            }
        }
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
        if( m_boPendingQuit )
        {
            Close( true );
        }
        break;
    default:
        break;
    }
    e.Skip();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateFirmware( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    UpdateFirmware( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateKernelDriver( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    UpdateKernelDriver( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::SetID( Device* pDev, int newID )
//-----------------------------------------------------------------------------
{
    int result = 0;
    if( newID < didlMin )
    {
        result = -4;
        WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );

    }
    else if( newID > didlMax )
    {
        result = -5;
        WriteErrorMessage( wxString::Format( wxT( "Invalid ID. Valid IDs range from %d to %d.\n" ), didlMin, didlMax ) );
    }
    else
    {
        Device* pDevByFamily = 0;
        string family = pDev->family.read();
        int devIndex = 0;
        while( ( pDevByFamily = m_devMgr.getDeviceByFamily( family, devIndex++ ) ) != 0 )
        {
            if( ( pDev != pDevByFamily ) && ( pDevByFamily->deviceID.read() == newID ) )
            {
                WriteErrorMessage( wxString::Format( wxT( "WARNING: The new ID(%d) is already assigned to at least one other device belonging to the same family(%s)(%s)????.\n" ), newID, ConvertedString( pDevByFamily->serial.read() ).c_str(), ConvertedString( family ).c_str() ) );
                break;
            }
        }
    }

    if( result == 0 )
    {
        WriteLogMessage( wxString::Format( wxT( "Trying to assign ID %d to %s(%s).\n" ), newID, ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str() ) );
        switch( pDev->setID( newID ) )
        {
        case DMR_FEATURE_NOT_AVAILABLE:
            WriteErrorMessage( wxString::Format( wxT( "Device %s(%s) doesn't support setting the device ID.\n" ), ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str() ) );
            break;
        case DMR_NO_ERROR:
            WriteLogMessage( wxString::Format( wxT( "%s.\n" ), ConvertedString( pDev->HWUpdateResult.readS() ).c_str() ) );
            break;
        default:
            WriteErrorMessage( wxString::Format( wxT( "An error occurred while setting the device ID for device %s(%s): %s(please refer to the manual for this error code).\n" ), ConvertedString( pDev->serial.read() ).c_str(), ConvertedString( pDev->product.read() ).c_str(), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
            break;
        }
    }
    return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::SetID( int deviceIndex )
//-----------------------------------------------------------------------------
{
    int result = 0;
    if( deviceIndex >= 0 )
    {
        Device* pDev = m_devMgr[deviceIndex];
        if( pDev )
        {
            auto_ptr<DeviceHandler> pHandler( m_deviceHandlerFactory.CreateObject( ConvertedString( pDev->family.read() ), pDev ) );
            if( pHandler.get() )
            {
                pHandler->AttachParent( this );
                long newID = 0;
                if( pHandler->GetIDFromUser( newID, didlMin, didlMax ) )
                {
                    result = SetID( pDev, static_cast<int>( newID ) );
                }
            }
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "Invalid item selection(index: %d).\n" ), deviceIndex ) );
            result = -2;
        }
    }
    else
    {
        wxMessageDialog selectDlg( NULL, wxT( "Select a device." ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        selectDlg.ShowModal();
        result = -1;
    }
    return result;
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateDeviceList( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    UpdateDeviceList();
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::UpdateDeviceList( void )
//-----------------------------------------------------------------------------
{
    wxBusyCursor busyCursorScope;
    m_pLogWindow->SetInsertionPointEnd();
    WriteLogMessage( wxString( wxT( "Updating device list...\n" ) ) );
    try
    {
        // always run this code as new interfaces might appear at runtime e.g. when plugging in a network cable...
        const int64_type interfaceCount( GetGenTLInterfaceCount() );
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
    m_devMgr.updateDeviceList();
    BuildList();
    WriteLogMessage( wxString( wxT( "Done.\n" ) ) );
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateFirmware( Device* pDev, bool boSilentMode, bool boPersistentUserSets, const wxString& customFirmwareFileName /* = wxEmptyString */ )
//-----------------------------------------------------------------------------
{
    int result = 0;
    auto_ptr<DeviceHandler> pHandler( m_deviceHandlerFactory.CreateObject( ConvertedString( pDev->family.read() ), pDev ) );
    if( pHandler.get() )
    {
        wxBusyCursor busyCursorScope;
        m_listUpdateTimer.Stop();
        pHandler->AttachParent( this );
        // Prefer using the more specific custom firmware file name
        if( !customFirmwareFileName.IsEmpty() )
        {
            pHandler->SetCustomFirmwareFile( customFirmwareFileName );
        }
        else
        {
            pHandler->SetCustomFirmwareFile( m_customFirmwareFile );
        }
        pHandler->SetCustomFirmwarePath( m_customFirmwarePath );
        pHandler->SetCustomGenICamFile( m_customGenICamFile );
        result = pHandler->UpdateFirmware( boSilentMode, boPersistentUserSets );
        m_listUpdateTimer.Start( TIMER_PERIOD );
        UpdateDeviceList();
    }
    else
    {
        WriteErrorMessage( wxString::Format( wxT( "Device %s doesn't support firmware updates.\n" ), ConvertedString( pDev->serial.read() ).c_str() ) );
        result = -3;
    }
    return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateFirmware( int deviceIndex )
//-----------------------------------------------------------------------------
{
    int result = 0;
    if( deviceIndex >= 0 )
    {
        Device* pDev = m_devMgr[deviceIndex];
        if( pDev )
        {
            //Deselect the list item (so that the background color is visible) and color the background
            //light yellow to highlight the device on which a firmware update is performed.
            m_pDevListCtrl->SetItemState( deviceIndex, 0, wxLIST_STATE_SELECTED | wxLIST_STATE_FOCUSED );
            m_pDevListCtrl->SetItemBackgroundColour( deviceIndex, wxColour( 200, 255, 200 ) );
            result = UpdateFirmware( pDev, false, m_pMISettings_KeepUserSetSettingsAfterFirmwareUpdate->IsChecked() );
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "Invalid item selection(index: %d).\n" ), deviceIndex ) );
            result = -2;
        }
    }
    else
    {
        wxMessageDialog selectDlg( NULL, wxT( "Select a device." ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        selectDlg.ShowModal();
        result = -1;
    }
    return result;
}


//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnUpdateDMABufferSize( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    UpdateDMABufferSize( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::RefreshApplicationExitCode( const int result )
//-----------------------------------------------------------------------------
{
    if( ( result != DeviceHandler::urOperationSuccessful ) &&
        ( result != DeviceHandler::urFeatureUnsupported ) ) // this might be returned e.g. from 3rd party devices but during a script or command line based updates an should not be treated as an error
    {
        m_updateResult = result;
    }
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateDMABufferSize( int deviceIndex )
//-----------------------------------------------------------------------------
{
    int result = 0;
    if( deviceIndex >= 0 )
    {
        Device* pDev = m_devMgr[deviceIndex];
        if( pDev )
        {
            auto_ptr<DeviceHandler> pHandler( m_deviceHandlerFactory.CreateObject( ConvertedString( pDev->family.read() ), pDev ) );
            if( pHandler.get() )
            {
                wxBusyCursor busyCursorScope;
                pHandler->AttachParent( this );
                result = pHandler->UpdatePermanentDMABufferSize( false );
            }
            else
            {
                WriteErrorMessage( wxString::Format( wxT( "Device %s doesn't support firmware updates.\n" ), ConvertedString( pDev->serial.read() ).c_str() ) );
                result = -3;
            }
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "Invalid item selection(index: %d).\n" ), deviceIndex ) );
            result = -2;
        }
    }
    else
    {
        wxMessageDialog selectDlg( NULL, wxT( "Select a device." ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        selectDlg.ShowModal();
        result = -1;
    }
    return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateKernelDriver( Device* pDev, bool boSilentMode )
//-----------------------------------------------------------------------------
{
    int result = 0;
    auto_ptr<DeviceHandler> pHandler( m_deviceHandlerFactory.CreateObject( ConvertedString( pDev->family.read() ), pDev ) );
    if( pHandler.get() )
    {
        wxBusyCursor busyCursorScope;
        if( m_listUpdateTimer.IsRunning() )
        {
            m_listUpdateTimer.Stop();
        }
        pHandler->AttachParent( this );
        result = pHandler->UpdateKernelDriver( boSilentMode );
        m_listUpdateTimer.Start( TIMER_PERIOD );
    }
    else
    {
        WriteErrorMessage( wxString::Format( wxT( "Device %s doesn't support kernel driver updates.\n" ), ConvertedString( pDev->serial.read() ).c_str() ) );
        result = -3;
    }
    return result;
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::UpdateKernelDriver( int deviceIndex )
//-----------------------------------------------------------------------------
{
    int result = 0;
    if( deviceIndex >= 0 )
    {
        Device* pDev = m_devMgr[deviceIndex];
        if( pDev )
        {
            result = UpdateKernelDriver( pDev, false );
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "Invalid item selection(index: %d).\n" ), deviceIndex ) );
            result = -2;
        }
    }
    else
    {
        wxMessageDialog selectDlg( NULL, wxT( "Select a device." ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        selectDlg.ShowModal();
        result = -1;
    }
    return result;
}

#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
//-----------------------------------------------------------------------------
void DeviceConfigureFrame::OnSetFriendlyNameForDirectShow( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    SetDSFriendlyName( m_pDevListCtrl->GetCurrentItemIndex() );
}

//-----------------------------------------------------------------------------
int DeviceConfigureFrame::SetDSFriendlyName( int deviceIndex )
//-----------------------------------------------------------------------------
{
    int result = 0;
    if( deviceIndex >= 0 )
    {
        Device* pDev = m_devMgr[deviceIndex];
        if( pDev )
        {
            wxListItem info;
            info.m_itemId = deviceIndex;
            info.m_col = lcDSFriendlyName;
            info.m_mask = wxLIST_MASK_TEXT;
            if( !m_pDevListCtrl->GetItem( info ) )
            {
                WriteErrorMessage( wxT( "Failed to obtain item info.\n" ) );
            }

            string devSerial = pDev->serial.read();
            string devProduct = pDev->product.read();
            WriteLogMessage( wxString::Format( wxT( "Trying to set new DirectShow friendly name for %s(%s). Current friendly name: %s\n" ), ConvertedString( devSerial ).c_str(), ConvertedString( devProduct ).c_str(), info.m_text.c_str() ) );
            wxString newFriendlyName = wxGetTextFromUser( wxString::Format( wxT( "Enter the new DirectShow friendly name for device %s.\nMake sure that no other device is using this name already.\n" ), ConvertedString( devSerial ).c_str() ),
                                       wxT( "New friendly name:" ),
                                       wxString::Format( wxT( "%s_%s" ), ConvertedString( devProduct ).c_str(), ConvertedString( devSerial ).c_str() ),
                                       this );
            if( newFriendlyName == wxEmptyString )
            {
                result = -4;
                WriteErrorMessage( wxT( "Operation canceled by the user or invalid (empty) input.\n" ) );
            }
            else
            {
                // check if this name is already in use
                int itemCount = m_pDevListCtrl->GetItemCount();
                for( int i = 0; i < itemCount; i++ )
                {
                    info.m_itemId = i;
                    info.m_col = lcDSFriendlyName;
                    info.m_mask = wxLIST_MASK_TEXT;
                    if( !m_pDevListCtrl->GetItem( info ) )
                    {
                        WriteErrorMessage( wxString::Format( wxT( "Failed to obtain item info for item %d.\n" ), i ) );
                        continue;
                    }
                    if( info.m_text == newFriendlyName )
                    {
                        if( i == deviceIndex )
                        {
                            WriteLogMessage( wxT( "The friendly name for this device has not been changed.\n" ) );
                            result = -5;
                        }
                        else
                        {
                            WriteErrorMessage( wxT( "WARNING: Another device is using this friendly name already. Operation skipped.\n" ) );
                            result = -6;
                        }
                    }
                }

                if( result == 0 )
                {
                    WriteLogMessage( wxString::Format( wxT( "Trying to assign %s as a friendly name to device %s.\n" ), newFriendlyName.c_str(), ConvertedString( devSerial ).c_str() ) );
                    string friendlyNameANSI( newFriendlyName.mb_str() );
                    result = m_DSDevMgr.registerDevice( pDev->deviceID.read(), pDev->product.read().c_str(), friendlyNameANSI.c_str(), pDev->serial.read().c_str(), pDev->family.read().c_str() );
                    BuildList();
                }
            }
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "Invalid item selection(index: %d).\n" ), deviceIndex ) );
            result = -2;
        }
    }
    else
    {
        wxMessageDialog selectDlg( NULL, wxT( "Select a device." ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        selectDlg.ShowModal();
        result = -1;
    }
    return result;
}
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::UpdateMenu( int deviceIndex )
//-----------------------------------------------------------------------------
{
    Device* pDev = 0;
    if( ( deviceIndex >= 0 ) && ( deviceIndex < static_cast<int>( m_devMgr.deviceCount() ) ) )
    {
        pDev = m_devMgr.getDevice( deviceIndex );
    }
    string kernelDriverName;
    bool boNewerDriverAvailable = false;
    bool boFeatureSupported = false;
    bool boFWUpdateSupported = false;
    bool boSetIDSupported = false;
    bool boDMABufferSizeSupported = false;
    if( pDev )
    {
        auto_ptr<DeviceHandler> pHandler( m_deviceHandlerFactory.CreateObject( ConvertedString( pDev->family.read() ), pDev ) );
        if( pHandler.get() )
        {
            boFeatureSupported = pHandler->SupportsKernelDriverUpdate( boNewerDriverAvailable, kernelDriverName );
            boFWUpdateSupported = pHandler->SupportsFirmwareUpdate();
            boSetIDSupported = pHandler->SupportsSetID();
            boDMABufferSizeSupported = pHandler->SupportsDMABufferSizeUpdate();
        }
    }
    m_pMIActionSetID->Enable( boSetIDSupported );
    m_pMIActionUpdateFW->Enable( boFWUpdateSupported );
    m_pMIActionUpdateKernelDriver->Enable( boFeatureSupported && boNewerDriverAvailable );
    m_pMIActionUpdateDMABufferSize->Enable( boDMABufferSizeSupported );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    m_pMIDirectShow_SetFriendlyName->Enable( pDev != 0 );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::WriteErrorMessage( const wxString& msg )
//-----------------------------------------------------------------------------
{
    wxTextAttr errorStyle( wxColour( 255, 0, 0 ) );
    WriteLogMessage( msg, errorStyle );
}

//-----------------------------------------------------------------------------
void DeviceConfigureFrame::WriteLogMessage( const wxString& msg, const wxTextAttr& style /* = wxTextAttr(wxColour(0, 0, 0)) */ )
//-----------------------------------------------------------------------------
{
    if( m_pLogWindow )
    {
        long posBefore = m_pLogWindow->GetLastPosition();
        m_pLogWindow->WriteText( msg );
        long posAfter = m_pLogWindow->GetLastPosition();
        m_pLogWindow->SetStyle( posBefore, posAfter, style );
        m_pLogWindow->ScrollLines( m_pLogWindow->GetNumberOfLines() );
    }
}
