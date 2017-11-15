//-----------------------------------------------------------------------------
#ifndef DeviceConfigureFrameH
#define DeviceConfigureFrameH DeviceConfigureFrameH
//-----------------------------------------------------------------------------
#include "wx/wx.h"
#include "DebugFileParser.h"
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include "ObjectFactory.h"
#include "DeviceHandler.h"
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#   include "win32/DirectShowSupport.h"
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
#   include <common/ProcessorPowerStateConfiguration/ProcessorPowerStateConfiguration.h>
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
#include <map>

class DeviceListCtrl;

//-----------------------------------------------------------------------------
class DeviceConfigureFrame : public wxFrame
//-----------------------------------------------------------------------------
{
public:
    explicit DeviceConfigureFrame( const wxString& title, const wxPoint& pos, const wxSize& size, int argc, wxChar** argv );
    ~DeviceConfigureFrame();
    typedef DeviceHandler* ( *CreateDeviceHandler )( mvIMPACT::acquire::Device* );
    typedef ObjectFactory<DeviceHandler, wxString, CreateDeviceHandler, mvIMPACT::acquire::Device*> DeviceHandlerFactory;
    void ActivateDeviceIn_wxPropView( int deviceIndex );
    int SetID( int deviceIndex );
    int UpdateFirmware( int deviceIndex );
    int UpdateKernelDriver( int deviceIndex );
    void UpdateMenu( int deviceIndex );
    void WriteErrorMessage( const wxString& msg );
    void WriteLogMessage( const wxString& msg, const wxTextAttr& style = wxTextAttr( wxColour( 0, 0, 0 ) ) );
    DeviceManager& GetDeviceManager( void )
    {
        return m_devMgr;
    }
    DeviceHandlerFactory& GetHandlerFactory( void )
    {
        return m_deviceHandlerFactory;
    }
    int UpdateDMABufferSize( int deviceIndex );
    static int GetUpdateResult( void )
    {
        return m_updateResult;
    }
    bool IsFirmwareUpdateForced( void )
    {
        return m_boForceFirmwareUpdate;
    }
protected:
    // event handlers (these functions should _not_ be virtual)
    void OnHelp_About( wxCommandEvent& e );
    void OnHelp_OnlineDocumentation( wxCommandEvent& )
    {
        ::wxLaunchDefaultBrowser( wxT( "http://www.matrix-vision.com/manuals/" ) );
    }
    void OnConfigureLogOutput( wxCommandEvent& e );
    void OnQuit( wxCommandEvent& e );
    void OnSetID( wxCommandEvent& e );
    void OnTimer( wxTimerEvent& e );
    void OnUpdateDeviceList( wxCommandEvent& e );
    void OnUpdateFirmware( wxCommandEvent& e );
    void OnUpdateKernelDriver( wxCommandEvent& e );
    void OnUpdateDMABufferSize( wxCommandEvent& e );
    //-----------------------------------------------------------------------------
    enum TTimerEvent
    //-----------------------------------------------------------------------------
    {
        teListUpdate,
        teTimer
    };
    //-----------------------------------------------------------------------------
    enum
    //-----------------------------------------------------------------------------
    {
        TIMER_PERIOD = 500
    };
private:
    void                                BuildList( void );
    int64_type                          GetGenTLInterfaceCount( void ) const;
    static void                         RefreshApplicationExitCode( const int result );
    int                                 SetID( Device* pDev, int newID );
    void                                UpdateDeviceList( void );
    int                                 UpdateFirmware( Device* pDev, bool boSilentMode, bool boPersistentUserSets, const wxString& customFirmwareFileName = wxEmptyString );
    int                                 UpdateKernelDriver( Device* pDev, bool boSilentMode );
    static int                          m_updateResult;
    mvIMPACT::acquire::DeviceManager    m_devMgr;
    mvIMPACT::acquire::GenICam::SystemModule* m_pSystemModule;
    DeviceListCtrl*                     m_pDevListCtrl;
    wxMenuItem*                         m_pMIActionSetID;
    wxMenuItem*                         m_pMIActionUpdateFW;
    wxMenuItem*                         m_pMIActionUpdateKernelDriver;
    wxMenuItem*                         m_pMIActionUpdateDMABufferSize;
    wxMenuItem*                         m_pMIActionUpdateDeviceList;
    wxTextCtrl*                         m_pLogWindow;
    wxString                            m_logFileName;
    wxTimer                             m_timer;
    wxTimer                             m_listUpdateTimer;
    unsigned int                        m_lastDevMgrChangedCount;
    LogConfigurationVector              m_debugData;
    DeviceHandlerFactory                m_deviceHandlerFactory;
    wxString                            m_customFirmwareFile;
    wxString                            m_customFirmwarePath;
    wxString                            m_customGenICamFile;
    std::vector<std::string>            m_IPv4Masks;
    //-----------------------------------------------------------------------------
    struct DeviceConfigurationData
            //-----------------------------------------------------------------------------
    {
        std::string searchToken_;
        bool boUpdateKernelDriver_;
        bool boUpdateFW_;
        wxString customFirmwareFileName_;
        bool boSetDeviceID_;
        int deviceID_;
        explicit DeviceConfigurationData( const std::string& searchToken ) : searchToken_( searchToken ),
            boUpdateKernelDriver_( false ), boUpdateFW_( false ), customFirmwareFileName_(),
            boSetDeviceID_( false ), deviceID_( -1 ) {}
    };
    void GetConfigurationEntriesFromFile( const wxString& fileName );
    std::map<wxString, DeviceConfigurationData>::iterator GetConfigurationEntry( const wxString& value );
    std::map<wxString, DeviceConfigurationData> m_devicesToConfigure;
    bool                                m_boPendingQuit;
    bool                                m_boForceFirmwareUpdate;
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    bool                                m_boChangeProcessorIdleStates;
    bool                                m_boEnableIdleStates;
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    // any class wishing to process wxWidgets events must use this macro
    DECLARE_EVENT_TABLE()
    //-----------------------------------------------------------------------------
    // IDs for the controls and the menu commands
    enum TMenuItem
    //-----------------------------------------------------------------------------
    {
        miAction_Quit = 1,
        miAction_SetID,
        miAction_UpdateFW,
        miAction_UpdateKernelDriver,
        miAction_ConfigureLogOutput,
        miAction_UpdateDeviceList,
        miAction_UpdateDMABufferSize,
        miHelp_About,
        miHelp_OnlineDocumentation,
        miCOMMON_LAST
    };
    //-----------------------------------------------------------------------------
    enum TDeviceIDLimits
    //-----------------------------------------------------------------------------
    {
        didlMin = 0,
        didlMax = 250
    };
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    DirectShowDeviceManager             m_DSDevMgr;
    //-----------------------------------------------------------------------------
    enum TMenuItem_DirectShow
    //-----------------------------------------------------------------------------
    {
        miDirectShow_RegisterAllDevices = miCOMMON_LAST,
        miDirectShow_UnregisterAllDevices,
        miDirectShow_SetFriendlyName
    };
    wxMenuItem*                         m_pMIDirectShow_SetFriendlyName;
    void OnRegisterAllDevicesForDirectShow( wxCommandEvent& )
    {
        m_DSDevMgr.registerAllDevices();
        BuildList();
    }
    void OnUnregisterAllDevicesForDirectShow( wxCommandEvent& )
    {
        m_DSDevMgr.unregisterAllDevices();
        BuildList();
    }
    void OnSetFriendlyNameForDirectShow( wxCommandEvent& e );
public:
    int SetDSFriendlyName( int deviceIndex );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

    //-----------------------------------------------------------------------------
    enum TMenuItem_Settings
    //-----------------------------------------------------------------------------
    {
        miSettings_KeepUserSetSettingsAfterFirmwareUpdate = miCOMMON_LAST + 100
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
                , miSettings_CPUIdleStatesEnabled
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    };

    wxMenuItem*                         m_pMISettings_KeepUserSetSettingsAfterFirmwareUpdate;
#ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    wxMenuItem*                         m_pMISettings_CPUIdleStatesEnabled;
    void OnSettings_CPUIdleStatesEnabled( wxCommandEvent& e )
    {
        if( wxMessageBox( wxString::Format( wxT( "Are you sure you want to %s the C1, C2 and C3 states for ALL processors in this system?" ), e.IsChecked() ? wxT( "enable" ) : wxT( "disable" ) ), wxT( "CPU Power State Configuration" ), wxYES_NO | wxNO_DEFAULT | wxICON_EXCLAMATION, this ) == wxYES )
        {
            if( !SetPowerState( e.IsChecked() ) )
            {
                wxMessageBox( wxString::Format( wxT( "Failed to %s the C1, C2 and C3 states for ALL processors." ), e.IsChecked() ? wxT( "enable" ) : wxT( "disable" ) ), wxT( "ERROR" ), wxOK | wxICON_ERROR, this );
            }
        }
        else
        {
            bool boValue = false;
            if( GetPowerState( boValue ) )
            {
                m_pMISettings_CPUIdleStatesEnabled->Check( boValue );
            }
            else
            {
                wxMessageBox( wxT( "Failed to query the C1, C2 and C3 states for ALL processors." ), wxT( "ERROR" ), wxOK | wxICON_ERROR, this );
            }
        }
    }
#endif // #ifdef BUILD_WITH_PROCESSOR_POWER_STATE_CONFIGURATION_SUPPORT
    void OnSettings_KeepUserSetSettingsAfterFirmwareUpdate( wxCommandEvent& e )
    {
        if( wxMessageBox( wxString::Format( wxT( "Are you sure you want to %s persistent UserSet settings?\n If you select YES, all UserSet settings of a GenICam device \n will be %s every time you update its' firmware!" ), e.IsChecked() ? wxT( "enable" ) : wxT( "disable" ), e.IsChecked() ? wxT( "preserved" ) : wxT( "deleted" ) ), wxT( "UserSet settings and Firmware Update" ), wxYES_NO | wxNO_DEFAULT | wxICON_EXCLAMATION, this ) != wxYES )
        {
            m_pMISettings_KeepUserSetSettingsAfterFirmwareUpdate->Check( !e.IsChecked() );
        }
    }
};

#endif // DeviceConfigureFrameH
