//-----------------------------------------------------------------------------
#include <apps/Common/wxAbstraction.h>
#include "DeviceConfigureFrame.h"
#include "DeviceHandlerBlueDevice.h"
#include <memory>
#include <sstream>
#include <wx/artprov.h>
#include <wx/choicdlg.h>
#include <wx/dir.h>
#include <wx/ffile.h>
#include <wx/filedlg.h>
#include <wx/mstream.h>
#include <wx/progdlg.h>
#include <wx/thread.h>
#include <wx/utils.h>
#include <wx/zipstrm.h>

//=============================================================================
//=================== internal helper functions, classes and structs ==========
//=============================================================================
// ----------------------------------------------------------------------------
class FirmwareUpdateConfirmationDialog : public wxDialog
//-----------------------------------------------------------------------------
{
public:
    FirmwareUpdateConfirmationDialog( wxWindow* parent, wxWindowID id,
                                      const wxString& title,
                                      const wxPoint& pos = wxDefaultPosition,
                                      const wxSize& size = wxDefaultSize,
                                      long style = wxDEFAULT_DIALOG_STYLE,
                                      const wxString& message = wxT( "" ),
                                      bool keepUserSets = true ) : wxDialog( parent, id, title, pos, size, style, wxT( "FirmwareUpdateConfirmationDialog" ) ), pCBKeepUserSetSettings_( 0 )
    {
        const wxIcon infoIcon = wxArtProvider::GetMessageBoxIcon( wxICON_INFORMATION );
        SetIcon( infoIcon );
        SetBackgroundColour( *wxWHITE );
        wxPanel* pPanel = new wxPanel( this, wxID_ANY );

        pCBKeepUserSetSettings_ = new wxCheckBox( pPanel, wxID_ANY, wxT( "Keep UserSet Settings(Takes longer but preserves UserSet settings stored on the device)" ) );
        wxButton* pBtnOk = new wxButton( pPanel, wxID_OK, wxT( "OK" ) );
        wxButton* pBtnCancel = new wxButton( pPanel, wxID_CANCEL, wxT( "Cancel" ) );

        wxBoxSizer* pTextSizer = new wxBoxSizer( wxHORIZONTAL );
        pTextSizer->AddSpacer( 10 );
        pTextSizer->Add( new wxStaticBitmap( pPanel, wxID_ANY, infoIcon ) );
        pTextSizer->AddSpacer( 10 );
        pTextSizer->Add( new wxStaticText( pPanel, wxID_ANY, message ) );
        pTextSizer->AddSpacer( 10 );

        wxBoxSizer* pCheckBoxSizer = new wxBoxSizer( wxHORIZONTAL );
        pCheckBoxSizer->AddSpacer( 10 );
        pCheckBoxSizer->Add( pCBKeepUserSetSettings_ );
        pCheckBoxSizer->AddSpacer( 10 );

        wxBoxSizer* pButtonSizer = new wxBoxSizer( wxHORIZONTAL );
        pButtonSizer->Add( pBtnOk, wxSizerFlags().Border( wxALL, 7 ) );
        pButtonSizer->Add( pBtnCancel, wxSizerFlags().Border( wxALL, 7 ) );

        wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxVERTICAL );
        pTopDownSizer->AddSpacer( 10 );
        pTopDownSizer->Add( pTextSizer );
        pTopDownSizer->AddSpacer( 20 );
        pTopDownSizer->Add( pCheckBoxSizer );
        pTopDownSizer->AddSpacer( 5 );
        pTopDownSizer->Add( pButtonSizer, wxSizerFlags().Right() );

        pPanel->SetSizer( pTopDownSizer );
        pTopDownSizer->SetSizeHints( this );

        pCBKeepUserSetSettings_->SetValue( keepUserSets ? true : false );
        Center();
    }
    bool shallKeepUserSets( void ) const
    {
        return pCBKeepUserSetSettings_->IsChecked();
    }
private:
    wxCheckBox* pCBKeepUserSetSettings_;
};

//-----------------------------------------------------------------------------
struct UpdateFeatures
//-----------------------------------------------------------------------------
{
    Device* pDev_;
    PropertyS sourceFileName_;
    PropertyS destinationFileName_;
    PropertyI transferMode_;
    PropertyS transferBuffer_;
    Method upload_;
    Method download_;
    Method install_;
    Method updateFirmware_;
    PropertyS lastResult_;
    void bindFeatures( void )
    {
        DeviceComponentLocator locator( pDev_, dltSystemSettings, "FileExchange" );
        locator.bindComponent( sourceFileName_, "SourceFileName" );
        locator.bindComponent( destinationFileName_, "DestinationFileName" );
        locator.bindComponent( transferMode_, "TransferMode" );
        locator.bindComponent( transferBuffer_, "TransferBuffer" );
        locator.bindComponent( upload_, "DoFileUpload@i" );
        locator.bindComponent( download_, "DoFileDownload@i" );
        locator.bindComponent( install_, "DoInstallFile@i" );
        locator.bindComponent( updateFirmware_, "DoFirmwareUpdate@i" );
        locator.bindComponent( lastResult_, "LastResult" );
    }
    explicit UpdateFeatures( Device* pDev ) : pDev_( pDev )
    {
        bindFeatures();
    }
};

//------------------------------------------------------------------------------
class UpdateThreadBlueCOUGAR_X : public wxThread
//------------------------------------------------------------------------------
{
    UpdateFeatures& uf_;
    int updateFirmwareCallResult_;
protected:
    void* Entry( void )
    {
        updateFirmwareCallResult_ = uf_.updateFirmware_.call();
        return 0;
    }
public:
    explicit UpdateThreadBlueCOUGAR_X( UpdateFeatures& uf ) : wxThread( wxTHREAD_JOINABLE ),
        uf_( uf ), updateFirmwareCallResult_( DMR_NO_ERROR ) {}
    int getUpdateResult( void ) const
    {
        return updateFirmwareCallResult_;
    }
};

//------------------------------------------------------------------------------
class EraseFlashThread : public wxThread
//------------------------------------------------------------------------------
{
    mvIMPACT::acquire::GenICam::FileAccessControl    fac_;
    int&                                             fileOperationExecuteResult_;
protected:
    void* Entry()
    {
        // erase the flash before downloading the new firmware
        fac_.fileSelector.writeS( "DeviceFirmware" );
        fac_.fileOperationSelector.writeS( "Delete" );
        fileOperationExecuteResult_ = fac_.fileOperationExecute.call();
        return 0;
    }
public:
    explicit EraseFlashThread( mvIMPACT::acquire::GenICam::FileAccessControl& fac, int& fileOperationExecuteResult_ ) : wxThread( wxTHREAD_JOINABLE ), fac_( fac ), fileOperationExecuteResult_( fileOperationExecuteResult_ ) {}
};

//------------------------------------------------------------------------------
class RebootDeviceThread : public wxThread
//------------------------------------------------------------------------------
{
    Device* pDev_;
    int maxSleepTime_s_;
    int napTime_s_;
protected:
    void* Entry()
    {
        int timeSlept_s = 0;
        DeviceManager devMgr;

        // Wait up to 'maxSleepTime_s_' minutes for the device to reboot. If it doesn't re-appear
        // until then something went wrong but we don't want to stay here
        // forever. An error will be generated further up the stack then.
        while( timeSlept_s < maxSleepTime_s_ )
        {
            wxSleep( napTime_s_ );
            timeSlept_s += napTime_s_;
            devMgr.updateDeviceList(); // this is needed in order to invalidate cached data that might no longer be valid(e.g. the URLs for the description files)
            if( pDev_->state.read() == dsPresent )
            {
                break;
            }
        }
        return 0;
    }
public:
    explicit RebootDeviceThread( Device* pDev, const int maxSleepTime_s, const int napTime_s ) : wxThread( wxTHREAD_JOINABLE ), pDev_( pDev ), maxSleepTime_s_( maxSleepTime_s ), napTime_s_( napTime_s ) {}
};

//------------------------------------------------------------------------------
class StoreToFlashThread : public wxThread
//------------------------------------------------------------------------------
{
    mvIMPACT::acquire::GenICam::FileAccessControl& fac_;
    int result_;
protected:
    void* Entry()
    {
        fac_.fileOperationSelector.writeS( "MvFlashWrite" );
        result_ = fac_.fileOperationExecute.call();
        return 0;
    }
public:
    explicit StoreToFlashThread( mvIMPACT::acquire::GenICam::FileAccessControl& fac ) : wxThread( wxTHREAD_JOINABLE ), fac_( fac ), result_( DMR_NO_ERROR ) {}
    int GetResult( void ) const
    {
        return result_;
    }
};

//-----------------------------------------------------------------------------
class SystemSettingsGenTL : public SystemSettings
//-----------------------------------------------------------------------------
{
public:
    explicit SystemSettingsGenTL( Device* pDev ) : SystemSettings( pDev ),
        featurePollingEnable()
    {
        DeviceComponentLocator locator( pDev, dltSystemSettings );
        locator.bindComponent( featurePollingEnable, "FeaturePollingEnable" );
    }
    PropertyIBoolean featurePollingEnable;
};

//------------------------------------------------------------------------------
class WaitForCloseThread : public wxThread
//------------------------------------------------------------------------------
{
    mvIMPACT::acquire::GenICam::ODevFileStream* pUploadFile_;
protected:
    void* Entry()
    {
        pUploadFile_->close();
        return 0;
    }
public:
    explicit WaitForCloseThread(  mvIMPACT::acquire::GenICam::ODevFileStream* pUploadFile ) : wxThread( wxTHREAD_JOINABLE ), pUploadFile_( pUploadFile ) {}
};

//-----------------------------------------------------------------------------
template<class _Elem, class _Traits, class _Ax, class _AVx>
typename std::vector<std::basic_string<_Elem, _Traits, _AVx> >::size_type split(
    /// The string to split
    const std::basic_string<_Elem, _Traits, _Ax>& str,
    /// A string defining a separator
    const std::basic_string<_Elem, _Traits, _Ax>& separator,
    /// A vector for the resulting strings
    std::vector<std::basic_string<_Elem, _Traits, _Ax>, _AVx >& v )
//-----------------------------------------------------------------------------
{
    v.clear();
    typename std::basic_string<_Elem, _Traits, _Ax>::size_type start = 0, end = 0;
    std::basic_string<_Elem, _Traits, _Ax> param, key, value;
    while( ( start = str.find_first_not_of( separator, start ) ) != std::basic_string<_Elem, _Traits, _Ax>::npos )
    {
        end = str.find_first_of( separator, start );
        v.push_back( ( end == std::basic_string<_Elem, _Traits, _Ax>::npos ) ? str.substr( start ) : str.substr( start, end - start ) );
        start = end;
    }
    return v.size();
}

//-----------------------------------------------------------------------------
/// Highest file number first!
int CompareFileVersion( const wxString& first, const wxString& second )
//-----------------------------------------------------------------------------
{
    Version firstVersion( VersionFromString( ExtractVersionNumber( first ) ) );
    Version secondVersion( VersionFromString( ExtractVersionNumber( second ) ) );
    if( firstVersion < secondVersion )
    {
        return 1;
    }
    else if( firstVersion > secondVersion )
    {
        return -1;
    }
    return 0;
}

//-----------------------------------------------------------------------------
wxString ExtractVersionNumber( const wxString& s )
//-----------------------------------------------------------------------------
{
    //%s(%s, file version: %d.%d)
    static const wxString TOKEN( wxT( "version: " ) );
    int pos = s.Find( TOKEN.c_str() );
    if( pos == wxNOT_FOUND )
    {
        return wxString();
    }
    wxString version( s.Mid( pos + TOKEN.Length() ) );
    int posVersionEND = version.Find( wxT( ')' ) );
    return ( ( posVersionEND == wxNOT_FOUND ) ? version : version.Mid( 0, posVersionEND ) );
}

//-----------------------------------------------------------------------------
bool GetNextNumber( wxString& str, long& number )
//-----------------------------------------------------------------------------
{
    wxString numberString = str.BeforeFirst( wxT( '_' ) );
    str = str.AfterFirst( wxT( '_' ) );
    return numberString.ToLong( &number );
}

//-----------------------------------------------------------------------------
int64_type MACAddressFromString( const std::string& MAC )
//-----------------------------------------------------------------------------
{
    int64_type result = 0;
    std::vector<std::string> v;
    std::vector<std::string>::size_type cnt = split( MAC, std::string( ":" ), v );
    if( cnt != 6 )
    {
        // invalid string format
        return 0;
    }
    for( std::vector<std::string>::size_type i = 0; i < cnt; i++ )
    {
        unsigned int val;
        sscanf( v[i].c_str(), "%x", &val );
        result = result | ( static_cast<int64_type>( val ) << ( 8 * ( cnt - 1 - i ) ) );
    }
    return result;
}

//=============================================================================
//=================== implementation DeviceHandlerBlueDevice ==================
//=============================================================================
//-----------------------------------------------------------------------------
DeviceHandlerBlueDevice::DeviceHandlerBlueDevice( mvIMPACT::acquire::Device* pDev ) : DeviceHandler( pDev, true ), product_( pgUnknown ),
    productStringForFirmwareUpdateCheck_(), firmwareUpdateFileName_(), firmwareUpdateFolder_(), firmwareUpdateFolderDevelopment_(),
    GenICamFile_(), temporaryFolder_(), userSetsToKeepDuringUpdate_()
//-----------------------------------------------------------------------------
{
    ConvertedString product( pDev_->product.read() );
    if( product.Find( wxT( "mvBlueCOUGAR-P" ) ) != wxNOT_FOUND )
    {
        product_ = pgBlueCOUGAR_P;
    }
    else if( product.Find( wxT( "mvBlueCOUGAR-S" ) ) != wxNOT_FOUND )
    {
        product_ = pgBlueCOUGAR_S;
    }
    else if( IsBlueCOUGAR_X() )
    {
        product_ = pgBlueCOUGAR_X;
    }
    else if( IsBlueCOUGAR_Y() )
    {
        product_ = pgBlueCOUGAR_Y;
    }
    else if( IsBlueFOX3() )
    {
        product_ = pgBlueFOX3;
    }
    else if( product.Find( wxT( "mvBlueLYNX-M7" ) ) != wxNOT_FOUND )
    {
        product_ = pgBlueLYNX_M7;
    }

    switch( product_ )
    {
    case pgBlueCOUGAR_P:
        firmwareUpdateFileName_ = wxString( wxT( "mvBlueCOUGAR-P_Update.tgz" ) );
        productStringForFirmwareUpdateCheck_ = wxString( wxT( "mvBlueCOUGAR-P" ) );
        break;
    case pgBlueCOUGAR_S:
        firmwareUpdateFileName_ = product;
        if( pDev->firmwareVersion.read() >= 0x10100 )
        {
            // this is HW revision 1, which means it needs a different firmware
            firmwareUpdateFileName_.Append( wxT( "_R001" ) );
        }
        firmwareUpdateFileName_.Append( wxT( "_Update.fpg" ) );
        productStringForFirmwareUpdateCheck_ = firmwareUpdateFileName_;
        break;
    case pgBlueCOUGAR_X:
        firmwareUpdateFileName_ = wxString( wxT( "mvBlueCOUGAR-X_Update.mvu" ) );
        productStringForFirmwareUpdateCheck_ = wxString( wxT( "mvBlueCOUGAR-X" ) );
        break;
    case pgBlueCOUGAR_Y:
        firmwareUpdateFileName_ = wxString( wxT( "mvBlueCOUGAR-Y_Update.mvu" ) );
        productStringForFirmwareUpdateCheck_ = wxString( wxT( "mvBlueCOUGAR-Y" ) );
        break;
    case pgBlueFOX3:
        firmwareUpdateFileName_ = wxString( wxT( "mvBlueFOX3_Update.mvu" ) );
        productStringForFirmwareUpdateCheck_ = wxString( wxT( "mvBlueFOX3" ) );
        break;
    case pgBlueLYNX_M7:
        firmwareUpdateFileName_ = wxString( wxT( "mvBlueLYNX-M7_Update.tgz" ) );
        productStringForFirmwareUpdateCheck_ = wxString( wxT( "mvBlueLYNX-M7" ) );
        break;
    default:
        break;
    }

    static const wxString MVIMPACT_ACQUIRE_DIR( wxT( "MVIMPACT_ACQUIRE_DIR" ) );
    if( !::wxGetEnv( MVIMPACT_ACQUIRE_DIR, &firmwareUpdateFolder_ ) )
    {
        if( pParent_ )
        {
            pParent_->WriteErrorMessage( wxT( "Can't install new firmware automatically as a crucial environment variable(MVIMPACT_ACQUIRE_DIR) is missing...\nPlease select the folder containing the files manually.\n" ) );
        }
    }
    else
    {
        AppendPathSeparatorIfNeeded( firmwareUpdateFolder_ );
        firmwareUpdateFolderDevelopment_ = firmwareUpdateFolder_;
        if( product_ == pgBlueFOX3 )
        {
            firmwareUpdateFolder_.append( wxT( "FirmwareUpdates/mvBlueFOX" ) );
        }
        else
        {
            firmwareUpdateFolder_.append( wxT( "FirmwareUpdates/mvBlueCOUGAR" ) );
        }
        firmwareUpdateFolderDevelopment_.append( wxT( "/mvBlueCOUGAR/Firmware" ) );
    }
    firmwareUpdateDefaultFolder_ = firmwareUpdateFolder_;
    std::replace( firmwareUpdateDefaultFolder_.begin(), firmwareUpdateDefaultFolder_.end(), '/', static_cast<char>( wxFileName::GetPathSeparator() ) );
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::CheckForIncompatibleFirmwareVersions_BlueCOUGAR_X( bool boSilentMode, const wxString& serial, const FileEntryContainer& fileEntries, const wxString& selection, const Version& currentFirmwareVersion )
//-----------------------------------------------------------------------------
{
    // check for updates between incompatible firmware versions.
    const FileEntryContainer::size_type cnt = fileEntries.size();
    static const Version firstVersionWithNewNames( 1, 1, 86, 0 );
    bool boCompatibleInterfaces = true;
    FileEntryContainer::size_type fileIndex = 0;
    for( fileIndex = 0; fileIndex < cnt; fileIndex++ )
    {
        if( fileEntries[fileIndex].name_ == selection )
        {
            if( ( currentFirmwareVersion < firstVersionWithNewNames ) &&
                ( fileEntries[fileIndex].version_ >= firstVersionWithNewNames ) )
            {
                boCompatibleInterfaces = false;
                break;
            }
            else if( ( currentFirmwareVersion >= firstVersionWithNewNames ) &&
                     ( fileEntries[fileIndex].version_ < firstVersionWithNewNames ) )
            {
                boCompatibleInterfaces = false;
                break;
            }
        }
    }

    if( !boCompatibleInterfaces )
    {
        if( !MessageToUser( wxT( "Firmware Update" ), wxString::Format( wxT( "There has been an incompatible interface change between the firmware version currently running on the device(%ld.%ld.%ld.%ld) and the version you are about to install(%ld.%ld.%ld.%ld).\nFor further information please refer to the mvBlueCOUGAR-X manual section 'C++ developer -> Porting existing code -> Notes about mvIMPACT Acquire version 1.11.32 or higher and/or using mvBlueCOUGAR-X with firmware 1.2.0 or higher'.\n\nAre you sure you want to continue updating the firmware of device %s?" ),
                            currentFirmwareVersion.major_, currentFirmwareVersion.minor_, currentFirmwareVersion.subMinor_, currentFirmwareVersion.release_,
                            fileEntries[fileIndex].version_.major_, fileEntries[fileIndex].version_.minor_, fileEntries[fileIndex].version_.subMinor_, fileEntries[fileIndex].version_.release_,
                            serial.c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), ConvertedString( serial ).c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
            }
            return urOperationCanceled;
        }
    }

    return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
DeviceHandler::TUpdateResult DeviceHandlerBlueDevice::DoFirmwareUpdate_BlueCOUGAR_XOrY( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize )
//-----------------------------------------------------------------------------
{
    pDev_->interfaceLayout.write( dilGenICam );
    {
        // switch off automatic update of GenICam features triggered by recommended polling times in the GenICam XML file
        SystemSettingsGenTL ss( pDev_ );
        ss.featurePollingEnable.write( bFalse );
    }

    mvIMPACT::acquire::GenICam::FileAccessControl fac( pDev_ );
    TUpdateResult result = urOperationSuccessful;

    static const wxFileOffset fileOperationBlockSize = 4096;
    {
        // download the firmware image
        mvIMPACT::acquire::GenICam::ODevFileStream uploadFile;
        const TUpdateResult uploadResult = UploadFirmwareFile( uploadFile, boSilentMode, serial, pBuf, bufSize, fileOperationBlockSize );
        if( uploadResult != urOperationSuccessful )
        {
            return uploadResult;
        }
    }
    {
        static const int MAX_UPDATE_TIME_MILLISECONDS = 40000;
        static const int UPDATE_THREAD_PROGRESS_INTERVAL_MILLISECONDS = 100;
        wxProgressDialog storeToFlashDlg( wxT( "Storing Firmware To Flash" ),
                                          wxT( "Please wait...                                " ),
                                          MAX_UPDATE_TIME_MILLISECONDS, // range
                                          pParent_,     // parent
                                          wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME );

        StoreToFlashThread storeThread( fac );
        storeThread.Create();
        storeThread.Run();
        while( storeThread.IsRunning() )
        {
            wxMilliSleep( UPDATE_THREAD_PROGRESS_INTERVAL_MILLISECONDS );
            storeToFlashDlg.Pulse();
        }
        storeThread.Wait();
        storeToFlashDlg.Update( MAX_UPDATE_TIME_MILLISECONDS );
        if( static_cast<TDMR_ERROR>( storeThread.GetResult() ) != DMR_NO_ERROR )
        {
            MessageToUser( wxT( "Warning" ), wxString::Format( wxT( "Failed to write firmware to non-volatile memory of device %s failed. Error reported from driver: %d(%s). Please power-cycle the device now, update the device list and re-try to update the firmware." ), ConvertedString( serial ).c_str(), storeThread.GetResult(), ConvertedString( ImpactAcquireException::getErrorCodeAsString( storeThread.GetResult() ) ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
            return urFileIOError;
        }
        // It would be correct to enable this next section but there was a bug in the firmware until version 1.6.129 that would return
        // an error here, thus lots of devices out there would suddenly report a problem during the update so we leave this code disabled!
        //if( fac.fileOperationStatus.readS() != "Success" )
        //{
        //    MessageToUser( wxT( "Warning" ), wxString::Format( wxT( "Failed to write firmware to non-volatile memory of device %s failed. Error reported from device (fileOperationStatus): '%s'. Please power-cycle the device now, update the device list and re-try to update the firmware." ), ConvertedString( serial ).c_str(), ConvertedString( fac.fileOperationStatus.readS() ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
        //    return urFileIOError;
        //}
    }
    RebootDevice( boSilentMode, serial );

    if( result == urOperationSuccessful )
    {
        // now check if the new firmware is actually active
        try
        {
            if( pDev_->isOpen() )
            {
                pDev_->close();
            }
            DeviceManager devMgr;
            devMgr.updateDeviceList(); // this is needed in order to invalidate cached data that might no longer be valid(e.g. the URLs for the description files)
            SelectCustomGenICamFile();
            pDev_->open();
        }
        catch( const ImpactAcquireException& e )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to close and re-open device %s(%s)(%s, %s).\n" ),
                                           ConvertedString( pDev_->serial.read() ).c_str(),
                                           ConvertedString( pDev_->product.read() ).c_str(),
                                           ConvertedString( e.getErrorString() ).c_str(),
                                           ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            }
        }

        GenICam::DeviceControl dc( pDev_ );
        if( dc.mvDeviceFirmwareSource.isValid() )
        {
            if( ConvertedString( dc.mvDeviceFirmwareSource.readS() ) != wxString( wxT( "ProgramSection" ) ) )
            {
                MessageToUser( wxT( "Warning" ), wxT( "The firmware update did not succeed. The device is running with a fall-back firmware version now as the update section is damaged. Please repeat the firmware update and at the end power-cycle the device when this message shows up again. If afterwards the property 'DeviceControl/mvDeviceFirmwareSource' does not show 'ProgramSection' please contact MATRIX VISION." ), boSilentMode, wxOK | wxICON_INFORMATION );
            }
        }
        else
        {
            MessageToUser( wxT( "Warning" ), wxT( "The firmware of this device could not be verified. Please Check if the firmware version displayed matches the expected version after power-cycling the device." ), boSilentMode, wxOK | wxICON_INFORMATION );
        }
    }
    pDev_->close();
    return result;
}

//-----------------------------------------------------------------------------
DeviceHandler::TUpdateResult DeviceHandlerBlueDevice::DoFirmwareUpdate_BlueFOX3( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize )
//-----------------------------------------------------------------------------
{
    pDev_->interfaceLayout.write( dilGenICam );
    {
        // switch off automatic update of GenICam features triggered by recommended polling times in the GenICam XML file
        SystemSettingsGenTL ss( pDev_ );
        ss.featurePollingEnable.write( bFalse );
    }

    mvIMPACT::acquire::GenICam::FileAccessControl fac( pDev_ );
    int fileOperationExecuteResult = DMR_NO_ERROR;

    static const int MAX_UPDATE_TIME_MILLISECONDS = 40000;
    static const int UPDATE_THREAD_PROGRESS_INTERVAL_MILLISECONDS = 100;

    wxProgressDialog dlg( wxT( "Flash Erase" ),
                          wxT( "Erasing flash memory..." ),
                          MAX_UPDATE_TIME_MILLISECONDS, // range
                          pParent_,     // parent
                          wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME );

    EraseFlashThread eraseThread( fac, fileOperationExecuteResult );
    eraseThread.Create();
    eraseThread.Run();
    while( eraseThread.IsRunning() )
    {
        wxMilliSleep( UPDATE_THREAD_PROGRESS_INTERVAL_MILLISECONDS );
        dlg.Pulse();
    }
    eraseThread.Wait();
    dlg.Update( MAX_UPDATE_TIME_MILLISECONDS );

    if( static_cast<TDMR_ERROR>( fileOperationExecuteResult ) != DMR_NO_ERROR )
    {
        MessageToUser( wxT( "Warning" ), wxString::Format( wxT( "Failed to erase flash memory for firmware image on device %s. Error reported from driver: %d(%s)." ), ConvertedString( serial ).c_str(), fileOperationExecuteResult, ConvertedString( ImpactAcquireException::getErrorCodeAsString( fileOperationExecuteResult ) ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
        return urFileIOError;
    }

    static const wxFileOffset fileOperationBlockSize = 4096;
    {
        // download the firmware image
        mvIMPACT::acquire::GenICam::ODevFileStream uploadFile;
        const TUpdateResult uploadResult = UploadFirmwareFile( uploadFile, boSilentMode, serial, pBuf, bufSize, fileOperationBlockSize );
        if( uploadResult != urOperationSuccessful )
        {
            return uploadResult;
        }

        wxProgressDialog closeDlg( wxT( "Verifying Firmware Update" ),
                                   wxT( "Please wait...                                " ),
                                   MAX_UPDATE_TIME_MILLISECONDS, // range
                                   pParent_,     // parent
                                   wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME );

        WaitForCloseThread closeThread( &uploadFile );
        closeThread.Create();
        closeThread.Run();
        while( closeThread.IsRunning() )
        {
            wxMilliSleep( UPDATE_THREAD_PROGRESS_INTERVAL_MILLISECONDS );
            closeDlg.Pulse();
        }
        closeThread.Wait();
        closeDlg.Update( MAX_UPDATE_TIME_MILLISECONDS );

        if ( !uploadFile.good() )
        {
            MessageToUser( wxT( "Warning" ), wxString::Format( wxT( "Firmware verification for device %s. Please power-cycle the device now, update the device list and re-try to update the firmware." ), ConvertedString( serial ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
            uploadFile.close();
            return urFileIOError;
        }
    }
    RebootDevice( boSilentMode, serial );
    return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::ExtractFileVersion( const wxString& fileName, Version& fileVersion ) const
//-----------------------------------------------------------------------------
{
    static const wxString TOKEN( wxT( "_GigE_" ) );
    int pos = fileName.Find( TOKEN.c_str() );
    if( pos == wxNOT_FOUND )
    {
        return false;
    }
    wxString versionString( fileName.Mid( pos + TOKEN.Length() ) );
    return ( GetNextNumber( versionString, fileVersion.major_ ) &&
             GetNextNumber( versionString, fileVersion.minor_ ) &&
             GetNextNumber( versionString, fileVersion.subMinor_ ) );
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::GetFileFromArchive( const wxString& firmwareFileAndPath, const char* pArchive, size_t archiveSize, const wxString& filename, auto_array_ptr<char>& data, DeviceConfigureFrame* pParent )
//-----------------------------------------------------------------------------
{
    wxMemoryInputStream zipData( pArchive, archiveSize );
    wxZipInputStream zipStream( zipData );
    wxZipEntry* pZipEntry = 0;
    while( ( pZipEntry = zipStream.GetNextEntry() ) != 0 )
    {
        /// GetNextEntry returns the ownership to the object!
        std::auto_ptr<wxZipEntry> ptrGuard( pZipEntry );
        if( !zipStream.OpenEntry( *pZipEntry ) )
        {
            if( pParent )
            {
                pParent->WriteErrorMessage( wxString::Format( wxT( "Failed to open zip entry of archive %s\n" ), firmwareFileAndPath.c_str() ) );
            }
            continue;
        }

        if( pZipEntry->IsDir() )
        {
            continue;
        }

        int pos = pZipEntry->GetInternalName().Find( filename.c_str() );
        if( pos == wxNOT_FOUND )
        {
            continue;
        }

        if( pZipEntry->GetInternalName().Mid( pos ) == filename )
        {
            data.realloc( pZipEntry->GetSize() );
            zipStream.Read( data.get(), data.parCnt() );
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::GetIDFromUser( long& /*newID*/, const long /*minValue*/, const long /*maxValue*/ )
//-----------------------------------------------------------------------------
{
    ConvertedString serial( pDev_->serial.read() );
    wxString tool;
    wxString standard;
    switch( product_ )
    {
    case pgUnknown:
        break;
    case pgBlueFOX3:
        tool = wxT( "wxPropView" );
        standard = wxT( "USB3 Vision" );
        break;
    case pgBlueCOUGAR_P:
    case pgBlueCOUGAR_S:
    case pgBlueCOUGAR_X:
    case pgBlueCOUGAR_Y:
    case pgBlueLYNX_M7:
        tool = wxT( "mvIPConfigure" );
        standard = wxT( "GigE Vision" );
        break;
    }

    if( tool.IsEmpty() || standard.IsEmpty() )
    {
        if( MessageToUser( wxT( "Set Device ID" ), wxString::Format( wxT( "No valid tool/standard could be located for setting the device ID of device %s" ), serial.c_str() ), false, wxOK | wxICON_ERROR ) )
        {
            return false;
        }
    }

    if( MessageToUser( wxT( "Set Device ID" ), wxString::Format( wxT( "Device %s does not support setting an integer device ID.\n\nAs this device is %s compliant it might however support setting a user defined string by modifying the property 'DeviceUserID' that can be used instead.\n\nThis string can be assigned using the tool '%s'. Do you want to launch this application now?" ), serial.c_str(), standard.c_str(), tool.c_str() ), false, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
    {
        ::wxExecute( tool );
    }
    return false;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::GetLatestFirmwareVersion( Version& latestFirmwareVersion ) const
//-----------------------------------------------------------------------------
{
    switch( product_ )
    {
    case pgBlueCOUGAR_X:
    case pgBlueCOUGAR_Y:
    case pgBlueFOX3:
        return GetLatestFirmwareVersionCOUGAR_XAndYOrFOX3Device( latestFirmwareVersion );
    default:
        break;
    }
    return urFeatureUnsupported;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::GetLatestFirmwareVersionCOUGAR_XAndYOrFOX3Device( Version& latestFirmwareVersion ) const
//-----------------------------------------------------------------------------
{
    ConvertedString serial( pDev_->serial.read() );
    wxString firmwareFileAndPath;
    wxString descriptionFileAndPath;

    firmwareFileAndPath = firmwareUpdateFolder_ + wxString( wxT( "/" ) ) + firmwareUpdateFileName_;
    if( !::wxFileExists( firmwareFileAndPath ) )
    {
        firmwareFileAndPath = firmwareUpdateFolderDevelopment_ + wxString( wxT( "/" ) ) + firmwareUpdateFileName_;
        if( !::wxFileExists( firmwareFileAndPath ) )
        {
            return urFileIOError;
        }
    }

    PackageDescriptionFileParser parser;
    auto_array_ptr<char> pBuffer( 0 );
    const TUpdateResult result = ParseUpdatePackageCOUGAR_XOrFOX3Device( parser, firmwareFileAndPath, 0, pBuffer );
    if( result != urOperationSuccessful )
    {
        return result;
    }

    const FileEntryContainer& fileEntries = parser.GetResults();
    const FileEntryContainer::size_type cnt = fileEntries.size();
    const wxString product( ConvertedString( pDev_->product.read() ) );
    const wxString productFromManufacturerInfo( GetProductFromManufacturerInfo() );
    const wxString deviceVersionAsString( pDev_->deviceVersion.isValid() ? ConvertedString( pDev_->deviceVersion.read() ) : wxString( wxT( "1.0" ) ) );
    const Version deviceVersion( VersionFromString( deviceVersionAsString ) );
    wxArrayString choices;
    for( FileEntryContainer::size_type i = 0; i < cnt; i++ )
    {
        std::map<wxString, SuitableProductKey>::const_iterator it = GetSuitableFirmwareIterator( fileEntries[i], product, productFromManufacturerInfo );
        if( ( it != fileEntries[i].suitableProductKeys_.end() ) && ( fileEntries[i].type_ == wxString( wxT( "Firmware" ) ) ) &&
            IsVersionWithinRange( deviceVersion, it->second.revisionMin_, it->second.revisionMax_ ) )
        {
            choices.Add( wxString::Format( wxT( "%s(%s, file version: %ld.%ld.%ld.%ld)" ), fileEntries[i].name_.c_str(), fileEntries[i].description_.c_str(), fileEntries[i].version_.major_, fileEntries[i].version_.minor_, fileEntries[i].version_.subMinor_, fileEntries[i].version_.release_ ) );
        }
    }
    choices.Sort( CompareFileVersion );

    wxString selection;
    if( choices.IsEmpty() )
    {
        return urInvalidFileSelection;
    }

    selection = choices[0].BeforeFirst( wxT( '(' ) );

    const Version currentFirmwareVersion = VersionFromString( ConvertedString( pDev_->firmwareVersion.readS() ) );
    for( FileEntryContainer::size_type i = 0; i < cnt; i++ )
    {
        if( fileEntries[i].name_ == selection )
        {
            if( currentFirmwareVersion >= fileEntries[i].version_ )
            {
                return urOperationCanceled;
            }
            else
            {
                latestFirmwareVersion = fileEntries[i].version_;
                break;
            }
        }
    }

    return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
wxString DeviceHandlerBlueDevice::GetProductFromManufacturerInfo( void ) const
//-----------------------------------------------------------------------------
{
    wxString product;
    if( pDev_->manufacturerSpecificInformation.isValid() )
    {
        std::vector<std::string> v;
        std::vector<std::string>::size_type cnt = split( pDev_->manufacturerSpecificInformation.readS(), std::string( ";" ), v );
        if( cnt > 0 )
        {
            // all parameters here use the <param>=<value> syntax (e.g. FW=3.5.67.3) except the product type
            // which is always the LAST token. Tokens are separated by a single ';'. Thus if a device specifies
            // a device type here this might look like this:
            // FW=2.17.360.0;DS1-024ZG-11111-00
            // So if the last token does NOT contain a '=' we assume this to be the product type!
            std::vector<std::string> keyValPair;
            if( split( v[cnt - 1], std::string( "=" ), keyValPair ) == 1 )
            {
                product = ConvertedString( v[cnt - 1] );
            }
        }
    }
    return product;
}

//-----------------------------------------------------------------------------
std::map<wxString, SuitableProductKey>::const_iterator DeviceHandlerBlueDevice::GetSuitableFirmwareIterator( const FileEntry& entry, const wxString& product, const wxString& productFromManufacturerInfo ) const
//-----------------------------------------------------------------------------
{
    std::map<wxString, SuitableProductKey>::const_iterator it = entry.suitableProductKeys_.find( product );
    if( ( it == entry.suitableProductKeys_.end() ) && !productFromManufacturerInfo.IsEmpty() )
    {
        it = entry.suitableProductKeys_.find( productFromManufacturerInfo );
    }
    return it;
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::IsBlueCOUGAR_X( void ) const
//-----------------------------------------------------------------------------
{
    if( ConvertedString( pDev_->product.read() ).Find( wxT( "mvBlueCOUGAR-X" ) ) != wxNOT_FOUND )
    {
        return true;
    }

    ComponentLocator locator( pDev_->hDev() );
    Property prop;
    locator.bindComponent( prop, "DeviceMACAddress" );
    if( !prop.isValid() )
    {
        return false;
    }

    const int64_type MAC = MACAddressFromString( prop.readS() );
    return ( ( ( MAC >= 0x0000000c8d600001LL ) && ( MAC <= 0x0000000c8d60ffffLL ) ) ||
             ( ( MAC >= 0x0000000c8d610001LL ) && ( MAC <= 0x0000000c8d61ffffLL ) ) ||
             ( ( MAC >= 0x0000000c8d700001LL ) && ( MAC <= 0x0000000c8d707fffLL ) ) ||
             ( ( MAC >= 0x0000000c8d708001LL ) && ( MAC <= 0x0000000c8d70bfffLL ) ) ||
             ( ( MAC >= 0x0000000c8d70c001LL ) && ( MAC <= 0x0000000c8d70cfffLL ) ) ||
             ( ( MAC >= 0x0000000c8d710001LL ) && ( MAC <= 0x0000000c8d717fffLL ) ) );
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::IsBlueCOUGAR_Y( void ) const
//-----------------------------------------------------------------------------
{
    if( ConvertedString( pDev_->product.read() ).Find( wxT( "mvBlueCOUGAR-Y" ) ) != wxNOT_FOUND )
    {
        return true;
    }

    ComponentLocator locator( pDev_->hDev() );
    Property prop;
    locator.bindComponent( prop, "DeviceMACAddress" );
    if( !prop.isValid() )
    {
        return false;
    }

    const int64_type MAC = MACAddressFromString( prop.readS() );
    return( ( MAC >= 0x0000000c8d620001LL ) && ( MAC <= 0x0000000c8d62ffffLL ) );
}

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueDevice::IsFirmwareUpdateMeaningless( bool boSilentMode, const Version& deviceFWVersion, const Version& selectedFWVersion, const wxString& defaultFWArchive, const Version& defaultFolderFWVersion ) const
//-----------------------------------------------------------------------------
{
    wxString issues;
    if( deviceFWVersion >= selectedFWVersion )
    {
        issues.append( wxString::Format( wxT( "You are about to %sgrade from firmware version %ld.%ld.%ld to %ld.%ld.%ld so the new version will be %s the one currently running on the device!\n\n" ),
                                         ( selectedFWVersion == deviceFWVersion ) ? wxT( "up" ) : wxT( "down" ),
                                         deviceFWVersion.major_, deviceFWVersion.minor_, deviceFWVersion.subMinor_,
                                         selectedFWVersion.major_, selectedFWVersion.minor_, selectedFWVersion.subMinor_,
                                         ( selectedFWVersion == deviceFWVersion ) ? wxT( "equal to" ) : wxT( "older than" ) ) );
    }
    if( defaultFolderFWVersion > selectedFWVersion )
    {
        issues.append( wxString::Format( wxT( "The firmware version in the MATRIX VISION default firmware folder (%s) is newer (%ld.%ld.%ld) than the one currently selected (%ld.%ld.%ld)!\n\n" ),
                                         defaultFWArchive.c_str(), defaultFolderFWVersion.major_, defaultFolderFWVersion.minor_, defaultFolderFWVersion.subMinor_, selectedFWVersion.major_, selectedFWVersion.minor_, selectedFWVersion.subMinor_ ) );
    }
    if( !issues.IsEmpty() )
    {
        issues.append( wxT( "Should the operation continue?\n" ) );
        if( !MessageToUser( wxT( "Firmware Update Issues Detected!" ), issues, boSilentMode, wxCANCEL | wxOK | wxICON_EXCLAMATION ) )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), ConvertedString( pDev_->serial.read() ).c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
            }
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
DeviceHandler::TUpdateResult DeviceHandlerBlueDevice::ParseUpdatePackageCOUGAR_XOrFOX3Device( PackageDescriptionFileParser& parser, const wxString& firmwareFileAndPath, DeviceConfigureFrame* pParent, auto_array_ptr<char>& pBuffer )
//-----------------------------------------------------------------------------
{
    // first check if the file actually exists as the 'wxFFile' constructor will produce a modal dialog box if not...
    if( !::wxFileExists( firmwareFileAndPath ) )
    {
        if( pParent )
        {
            pParent->WriteErrorMessage( wxString::Format( wxT( "Could not open update archive %s.\n" ), firmwareFileAndPath.c_str() ) );
        }
        return urFileIOError;
    }
    wxFFile archive( firmwareFileAndPath.c_str(), wxT( "rb" ) );
    if( !archive.IsOpened() )
    {
        if( pParent )
        {
            pParent->WriteErrorMessage( wxString::Format( wxT( "Could not open update archive %s.\n" ), firmwareFileAndPath.c_str() ) );
        }
        return urFileIOError;
    }

    pBuffer.realloc( archive.Length() );
    size_t bytesRead = archive.Read( pBuffer.get(), pBuffer.parCnt() );
    if( bytesRead != static_cast<size_t>( archive.Length() ) )
    {
        if( pParent )
        {
            wxString lengthS;
            lengthS << archive.Length();
            pParent->WriteErrorMessage( wxString::Format( wxT( "Could not read full content of archive '%s'(wanted: %s, bytes, got %d bytes).\n" ), firmwareFileAndPath.c_str(), lengthS.c_str(), bytesRead ) );
        }
        return urFileIOError;
    }

    wxMemoryInputStream zipData( pBuffer.get(), pBuffer.parCnt() );
    wxZipInputStream zipStream( zipData );
    int fileCount = zipStream.GetTotalEntries();
    if( fileCount < 2 )
    {
        if( pParent )
        {
            pParent->WriteErrorMessage( wxString::Format( wxT( "The archive %s claims to contain %d file(s) while at least 2 are needed\n" ), firmwareFileAndPath.c_str(), fileCount ) );
        }
        return urInvalidFileSelection;
    }

    auto_array_ptr<char> pPackageDescription;
    if( !GetFileFromArchive( firmwareFileAndPath, pBuffer.get(), pBuffer.parCnt(), wxString( wxT( "packageDescription.xml" ) ), pPackageDescription, pParent ) )
    {
        if( pParent )
        {
            pParent->WriteErrorMessage( wxString::Format( wxT( "Could not extract package description from archive %s.\n" ), firmwareFileAndPath.c_str() ) );
        }
        return urFileIOError;
    }

    parser.Create();
    const bool fSuccess = parser.Parse( pPackageDescription.get(), static_cast<int>( pPackageDescription.parCnt() ), true );
    if( !fSuccess || ( parser.GetErrorCode() != XML_ERROR_NONE ) )
    {
        if( pParent )
        {
            pParent->WriteErrorMessage( wxString::Format( wxT( "Package description file parser error(fSuccess: %d, XML error: %d(%s)).\n" ), fSuccess, parser.GetErrorCode(), parser.GetErrorString( parser.GetErrorCode() ) ) );
        }
        return urFileIOError;
    }

    if( !parser.GetLastError().empty() )
    {
        if( pParent )
        {
            pParent->WriteErrorMessage( wxString::Format( wxT( "Package description file parser error(last error: %s).\n" ), parser.GetLastError().c_str() ) );
        }
        return urFileIOError;
    }

    return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
void DeviceHandlerBlueDevice::RebootDevice( bool boSilentMode, const wxString& serial )
//-----------------------------------------------------------------------------
{
    mvIMPACT::acquire::GenICam::DeviceControl dc( pDev_ );
    if( dc.deviceReset.isValid() )
    {
        const int deviceResetResult = dc.deviceReset.call();
        if( static_cast<TDMR_ERROR>( deviceResetResult ) != DMR_NO_ERROR )
        {
            MessageToUser( wxT( "Update Result" ), wxString::Format( wxT( "Update successful but resetting device failed. Please disconnect and reconnect device %s now to activate the new firmware. Error reported from driver: %d(%s)." ), ConvertedString( serial ).c_str(), deviceResetResult, ConvertedString( ImpactAcquireException::getErrorCodeAsString( deviceResetResult ) ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
        }
    }
    else
    {
        MessageToUser( wxT( "Update Result" ), wxString::Format( wxT( "Update successful. Please disconnect and reconnect device %s now to activate the new firmware." ), ConvertedString( serial ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
    }

    pDev_->close();

    wxStopWatch stopWatch;
    static const int MAX_TIME_MILLISECONDS = 180000;
    static const int THREAD_INTERVAL_MILLISECONDS = 3000;
    wxProgressDialog dlg( wxT( "Rebooting device" ),
                          wxT( "Please wait...                                " ),
                          MAX_TIME_MILLISECONDS, // range
                          pParent_,     // parent
                          wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME );
    RebootDeviceThread rebootThread( pDev_, MAX_TIME_MILLISECONDS / 1000, THREAD_INTERVAL_MILLISECONDS / 1000 );
    rebootThread.Create();
    rebootThread.Run();
    while( rebootThread.IsRunning() )
    {
        wxMilliSleep( 100 );
        dlg.Pulse();
    }
    rebootThread.Wait();
    dlg.Update( MAX_TIME_MILLISECONDS );
    if( ( pDev_->state.read() == dsPresent ) && pParent_ )
    {
        pParent_->WriteLogMessage( wxString::Format( wxT( "Rebooting device '%s' took %ld ms.\n" ), serial.c_str(), stopWatch.Time() ) );
    }
}

//-----------------------------------------------------------------------------
void DeviceHandlerBlueDevice::SelectCustomGenICamFile( const wxString& descriptionFile/* = wxEmptyString */ )
//-----------------------------------------------------------------------------
{
    ComponentLocator locatorDevice( pDev_->hDev() );
    PropertyI descriptionToUse( locatorDevice.findComponent( "DescriptionToUse" ) );
    PropertyS customDescriptionFilename( locatorDevice.findComponent( "CustomDescriptionFileName" ) );
    if( descriptionFile.IsEmpty() == true )
    {
        descriptionToUse.writeS( "XMLLocation0" );
    }
    else
    {
        descriptionToUse.writeS( "CustomFile" );
        customDescriptionFilename.write( std::string( wxConvCurrent->cWX2MB( descriptionFile.c_str() ) ) );
    }
}

//-----------------------------------------------------------------------------
void DeviceHandlerBlueDevice::SetCustomFirmwareFile( const wxString& customFirmwareFile )
//-----------------------------------------------------------------------------
{
    if( !customFirmwareFile.IsEmpty() )
    {
        firmwareUpdateFileName_ = customFirmwareFile;
    }
}

//-----------------------------------------------------------------------------
void DeviceHandlerBlueDevice::SetCustomFirmwarePath( const wxString& customFirmwarePath )
//-----------------------------------------------------------------------------
{
    if( !customFirmwarePath.IsEmpty() )
    {
        firmwareUpdateFolder_ = customFirmwarePath;
    }
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateCOUGAR_SDevice( bool boSilentMode )
//-----------------------------------------------------------------------------
{
    ConvertedString serial( pDev_->serial.read() );
    wxString firmwareFileAndPath;
    wxString descriptionFileAndPath;
    bool boGotDescriptionFile = false;
    if( boSilentMode )
    {
        firmwareFileAndPath = firmwareUpdateFolder_ + wxString( wxT( "/" ) ) + firmwareUpdateFileName_;
        wxArrayString descriptionFiles;
        wxDir::GetAllFiles( firmwareUpdateFolder_, &descriptionFiles, wxT( "MATRIXVISION_mvBlueCOUGAR-S_GigE_*.zip" ) );
        const size_t cnt = descriptionFiles.GetCount();
        Version highestVersion;
        for( size_t i = 0; i < cnt; i++ )
        {
            Version currentVersion;
            wxFileName fileName( descriptionFiles[i] );
            if( ExtractFileVersion( fileName.GetName(), currentVersion ) )
            {
                if( highestVersion < currentVersion )
                {
                    descriptionFileAndPath = descriptionFiles[i];
                    boGotDescriptionFile = true;
                    highestVersion = currentVersion;
                }
            }
            else if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to parse filenames major version number(%s).\n" ), descriptionFiles[i].c_str() ) );
            }
        }
    }

    if( !boGotDescriptionFile )
    {
        wxFileDialog fileDlg( pParent_, wxT( "Please select the firmware file" ), firmwareUpdateFolder_, firmwareUpdateFileName_, wxString::Format( wxT( "%s firmware file (%s)|%s" ), ConvertedString( pDev_->product.read() ).c_str(), firmwareUpdateFileName_.c_str(), firmwareUpdateFileName_.c_str() ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
        if( fileDlg.ShowModal() != wxID_OK )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), ConvertedString( serial ).c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
            }
            return urOperationCanceled;
        }

        if( fileDlg.GetFilename().Find( productStringForFirmwareUpdateCheck_.c_str() ) != 0 )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "The file %s is not meant for updating device %s(%s).\n" ), fileDlg.GetFilename().c_str(), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ), wxTextAttr( wxColour( 255, 0, 0 ) ) );
            }
            return urInvalidFileSelection;
        }

        firmwareFileAndPath = fileDlg.GetPath();

        wxFileDialog fileDlgDescriptionFile( pParent_, wxT( "Please select the GenICam description file that came with the firmware" ), firmwareUpdateFolder_, wxT( "" ), wxString::Format( wxT( "%s GenICam description file (MATRIXVISION_mvBlueCOUGAR-S_GigE*.zip)|MATRIXVISION_mvBlueCOUGAR-S_GigE*.zip" ), ConvertedString( pDev_->product.read() ).c_str() ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
        if( fileDlgDescriptionFile.ShowModal() != wxID_OK )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), ConvertedString( serial ).c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
            }
            return urOperationCanceled;
        }

        if( fileDlgDescriptionFile.GetFilename().Find( wxT( "MATRIXVISION_mvBlueCOUGAR-S_GigE" ) ) != 0 )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "The file %s is not meant for updating device %s(%s).\n" ), fileDlgDescriptionFile.GetFilename().c_str(), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ), wxTextAttr( wxColour( 255, 0, 0 ) ) );
            }
            return urInvalidFileSelection;
        }

        descriptionFileAndPath = fileDlgDescriptionFile.GetPath();
    }

    if( !MessageToUser( wxT( "Firmware Update" ), wxString::Format( wxT( "Are you sure you want to update the firmware and description file of device %s?\nThis might take several minutes during which the application will not react.\n\nPlease be patient and do NOT disconnect the device from the power supply during this time." ), serial.c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), ConvertedString( serial ).c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
        }
        return urOperationCanceled;
    }

    int result = UploadFile( descriptionFileAndPath, descriptionFileAndPath );
    if( result != urOperationSuccessful )
    {
        return result;
    }
    result = UploadFile( firmwareFileAndPath, descriptionFileAndPath );
    if( result == urOperationSuccessful )
    {
        MessageToUser( wxT( "Information" ), wxT( "The Firmware has been updated. The new Firmware is active now and the device can be used again by other applications." ), boSilentMode, wxOK | wxICON_INFORMATION );
    }
    return result;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateCOUGAR_XOrYOrFOX3Device( bool boSilentMode, bool boPersistentUserSets )
//-----------------------------------------------------------------------------
{
    ConvertedString serial( pDev_->serial.read() );
    wxString firmwareFileAndPath;
    wxString descriptionFileAndPath;

    if( boSilentMode )
    {
        firmwareFileAndPath = firmwareUpdateFolder_ + wxString( wxT( "/" ) ) + firmwareUpdateFileName_;
    }
    else
    {
        wxFileDialog fileDlg( pParent_, wxT( "Please select the update archive containing the firmware" ), firmwareUpdateFolder_, firmwareUpdateFileName_, wxString::Format( wxT( "%s update archive (%s)|%s" ), ConvertedString( pDev_->product.read() ).c_str(), firmwareUpdateFileName_.c_str(), firmwareUpdateFileName_.c_str() ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
        if( fileDlg.ShowModal() != wxID_OK )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
            }
            return urOperationCanceled;
        }
        firmwareFileAndPath = fileDlg.GetPath();
    }

    PackageDescriptionFileParser parser;
    auto_array_ptr<char> pBuffer( 0 );
    TUpdateResult result = ParseUpdatePackageCOUGAR_XOrFOX3Device( parser, firmwareFileAndPath, pParent_, pBuffer );
    if( result != urOperationSuccessful )
    {
        return result;
    }

    const FileEntryContainer& fileEntries = parser.GetResults();
    const FileEntryContainer::size_type cnt = fileEntries.size();
    const wxString product( ConvertedString( pDev_->product.read() ) );
    const wxString productFromManufacturerInfo( GetProductFromManufacturerInfo() );
    const wxString deviceVersionAsString( pDev_->deviceVersion.isValid() ? ConvertedString( pDev_->deviceVersion.read() ) : wxString( wxT( "1.0" ) ) );
    const Version deviceVersion( VersionFromString( deviceVersionAsString ) );
    wxArrayString choices;
    wxArrayString unusableFiles;
    for( FileEntryContainer::size_type i = 0; i < cnt; i++ )
    {
        std::map<wxString, SuitableProductKey>::const_iterator it = GetSuitableFirmwareIterator( fileEntries[i], product, productFromManufacturerInfo );
        if( ( it != fileEntries[i].suitableProductKeys_.end() ) && ( fileEntries[i].type_ == wxString( wxT( "Firmware" ) ) ) &&
            IsVersionWithinRange( deviceVersion, it->second.revisionMin_, it->second.revisionMax_ ) )
        {
            choices.Add( wxString::Format( wxT( "%s(%s, file version: %ld.%ld.%ld.%ld)" ), fileEntries[i].name_.c_str(), fileEntries[i].description_.c_str(), fileEntries[i].version_.major_, fileEntries[i].version_.minor_, fileEntries[i].version_.subMinor_, fileEntries[i].version_.release_ ) );
        }
        else
        {
            wxString suitableProducts;
            it = fileEntries[i].suitableProductKeys_.begin();
            while( it != fileEntries[i].suitableProductKeys_.end() )
            {
                suitableProducts.Append( wxString::Format( wxT( "  %s(%s, rev. %ld.%ld.%ld - %ld.%ld.%ld)\n" ), it->first.c_str(), it->second.name_.c_str(),
                                         it->second.revisionMin_.major_, it->second.revisionMin_.minor_, it->second.revisionMin_.release_,
                                         it->second.revisionMax_.major_, it->second.revisionMax_.minor_, it->second.revisionMax_.release_ ) );
                ++it;
            }
            unusableFiles.Add( wxString::Format( wxT( "%s(%s, file version: %ld.%ld.%ld.%ld), suitable products:\n %s" ), fileEntries[i].name_.c_str(), fileEntries[i].description_.c_str(), fileEntries[i].version_.major_, fileEntries[i].version_.minor_, fileEntries[i].version_.subMinor_, fileEntries[i].version_.release_, suitableProducts.c_str() ) );
        }
    }
    choices.Sort( CompareFileVersion );

    wxString selection;
    if( choices.IsEmpty() )
    {
        if( pParent_ )
        {
            pParent_->WriteErrorMessage( wxString::Format( wxT( "The archive %s does not contain a suitable firmware file for device %s(%s, Rev. %s). Files detected:\n" ), firmwareFileAndPath.c_str(), serial.c_str(), product.c_str(), deviceVersionAsString.c_str() ) );
            const wxArrayString::size_type unusableFilesCount = unusableFiles.size();
            for( wxArrayString::size_type i = 0; i < unusableFilesCount; i++ )
            {
                pParent_->WriteErrorMessage( unusableFiles[i] );
            }
        }
        return urInvalidFileSelection;
    }
    else if( ( choices.size() == 1 ) || boSilentMode )
    {
        selection = choices[0].BeforeFirst( wxT( '(' ) );
    }
    else
    {
        wxSingleChoiceDialog dlg( pParent_, wxT( "This archive contains more than one firmware file.\nPlease select a file you want to program the device with." ), wxT( "Please select a file to upload and install on the device" ), choices );
        if( dlg.ShowModal() == wxID_OK )
        {
            selection = dlg.GetStringSelection().BeforeFirst( wxT( '(' ) );
        }
        else
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
            }
            return urOperationCanceled;
        }
    }

    const Version currentFirmwareVersion = VersionFromString( ConvertedString( pDev_->firmwareVersion.readS() ) );
    if( product_ == pgBlueCOUGAR_X )
    {
        const int incompatibleFirmwareCheckResult = CheckForIncompatibleFirmwareVersions_BlueCOUGAR_X( boSilentMode, serial, fileEntries, selection, currentFirmwareVersion );
        if( incompatibleFirmwareCheckResult != urOperationSuccessful )
        {
            return incompatibleFirmwareCheckResult;
        }
    }

    if( boSilentMode )
    {
        for( FileEntryContainer::size_type i = 0; i < cnt; i++ )
        {
            if( fileEntries[i].name_ == selection )
            {
                if( !pParent_->IsFirmwareUpdateForced() && currentFirmwareVersion == fileEntries[i].version_ )
                {
                    if( pParent_ )
                    {
                        pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s) as the current firmware version(%ld.%ld.%ld.%ld) is equal to the one provided in the archive(%ld.%ld.%ld.%ld).\n" ),
                                                   ConvertedString( serial ).c_str(),
                                                   ConvertedString( pDev_->product.read() ).c_str(),
                                                   currentFirmwareVersion.major_, currentFirmwareVersion.minor_, currentFirmwareVersion.subMinor_, currentFirmwareVersion.release_,
                                                   fileEntries[i].version_.major_, fileEntries[i].version_.minor_, fileEntries[i].version_.subMinor_, fileEntries[i].version_.release_ ) );
                    }
                    return urOperationSuccessful;
                }
                else
                {
                    // different version -> proceed
                    break;
                }
            }
        }
    }

    auto_array_ptr<char> pUploadFileBuffer;
    if( !GetFileFromArchive( firmwareFileAndPath, pBuffer.get(), pBuffer.parCnt(), selection, pUploadFileBuffer, pParent_ ) )
    {
        if( pParent_ )
        {
            pParent_->WriteErrorMessage( wxString::Format( wxT( "Could not extract file %s from archive %s.\n" ), selection.c_str(), firmwareFileAndPath.c_str() ) );
        }
        return urFileIOError;
    }

    Version latestVersion;
    GetLatestFirmwareVersionCOUGAR_XAndYOrFOX3Device( latestVersion );

    if( !boSilentMode && ( IsFirmwareUpdateMeaningless( boSilentMode, VersionFromString( ConvertedString( pDev_->firmwareVersion.readS() ) ), fileEntries[0].version_, firmwareUpdateDefaultFolder_, latestVersion ) ) )
    {
        return urOperationCanceled;
    }

    if( !pParent_->IsFirmwareUpdateForced() && boSilentMode && ( VersionFromString( ConvertedString( pDev_->firmwareVersion.readS() ) ) == fileEntries[0].version_ ) )
    {
        // In silent mode the firmware update will be done even if the "new" version is older than the "old" one. This is done
        // in order to allow a defined downgrade from a command line or script operation.
        // In case the versions are EXACTLY the same, no update action will take place, as this would be a definite waste of time.
        return urOperationCanceled;
    }

    if( !boSilentMode )
    {
        FirmwareUpdateConfirmationDialog dlg( pParent_,
                                              wxID_ANY,
                                              wxT( "Firmware Update" ),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              wxNO_DEFAULT | wxYES_NO,
                                              wxString::Format( wxT( "Are you sure you want to update the firmware of device %s?\nThis might take several minutes during which the application will not react.\n\nPlease be patient and do NOT disconnect the device from the\npower supply during this time." ), serial.c_str() ),
                                              boPersistentUserSets );
        switch( dlg.ShowModal() )
        {
        case wxID_OK:
            break;
        default:
            if ( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), ConvertedString( serial ).c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
            }
            return urOperationCanceled;
        }
        boPersistentUserSets = dlg.shallKeepUserSets();
    }

    // avoid keeping UserSets when the camera runs as Bootloader device
    static const Version VERSION_1_0_0_0( 1, 0, 0, 0 );
    if( ( boPersistentUserSets == true ) &&
        ( pDev_->family.readS() == "mvBlueFOX3" ) &&
        ( ( currentFirmwareVersion == VERSION_1_0_0_0 ) || ( currentFirmwareVersion.major_ == 0 ) ) ) // old bootloader code did use version 1.0.0.0, newer versions reflect the source they have been build from by reverting the firmware version of that time thus it is 0.minor.subminor.major
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "No attempt to keep UserSets will be done for device %s(%s), because it is currently a bootloader device and cannot have valid UserSet settings.\n" ), ConvertedString( serial ).c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
        }
        boPersistentUserSets = false;
    }

    try
    {
        // In order to make sure an inconsistent setting will not block the correct startup of the device after the update switch the setting
        // to use at startup back to 'Default'. If the user selects to keep the settings on the device restore the value later after checking
        // that it can actually be used.
        const wxString previousUserSetDefault = SetUserSetDefault( wxT( "Default" ) );
        if( boPersistentUserSets )
        {
            UserSetBackup();
        }

        SelectCustomGenICamFile( GenICamFile_ );
        switch( product_ )
        {
        case pgBlueCOUGAR_X:
        case pgBlueCOUGAR_Y:
            result = DoFirmwareUpdate_BlueCOUGAR_XOrY( boSilentMode, serial, pUploadFileBuffer.get(), pUploadFileBuffer.parCnt() );
            break;
        case pgBlueFOX3:
            result = DoFirmwareUpdate_BlueFOX3( boSilentMode, serial, pUploadFileBuffer.get(), pUploadFileBuffer.parCnt() );
            break;
        default:
            result = urFeatureUnsupported;
            break;
        }

        if( boPersistentUserSets )
        {
            // restore the previous value for 'UserSetDefault'. If the user did chose not to keep the settings we also do not need
            // to restore the value for 'UserSetDefault'. It wouldn't do anything useful on the device anyway.
            UserSetRestore( previousUserSetDefault );
        }
    }
    catch( const ImpactAcquireException& e )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to update device %s(%s)(%s, %s).\n" ),
                                       ConvertedString( serial ).c_str(),
                                       ConvertedString( pDev_->product.read() ).c_str(),
                                       ConvertedString( e.getErrorString() ).c_str(),
                                       ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
        result = urDeviceAccessError;
    }

    if( result == urOperationSuccessful )
    {
        MessageToUser( wxT( "Information" ), wxT( "The Firmware has been updated. The new Firmware is active now and the device can be used again by other applications." ), boSilentMode, wxOK | wxICON_INFORMATION );
    }

    if( pDev_->isOpen() )
    {
        pDev_->close();
    }

    return result;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateFirmware( bool boSilentMode, bool boPersistentUserSets )
//-----------------------------------------------------------------------------
{
    ConvertedString serial( pDev_->serial.read() );
    if( !MessageToUser( wxT( "Firmware Update" ), wxString::Format( wxT( "Please close all other applications using device %s before proceeding." ), serial.c_str() ), boSilentMode, wxCANCEL | wxOK | wxICON_INFORMATION ) )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
        }
        return urOperationCanceled;
    }

    if( product_ == pgBlueCOUGAR_S )
    {
        return UpdateCOUGAR_SDevice( boSilentMode );
    }
    else if( ( product_ == pgBlueCOUGAR_X ) ||
             ( product_ == pgBlueCOUGAR_Y ) ||
             ( product_ == pgBlueFOX3 ) )
    {
        return UpdateCOUGAR_XOrYOrFOX3Device( boSilentMode, boPersistentUserSets );
    }
    else
    {
        return UpdateLYNX_M7OrCOUGAR_PDevice( firmwareUpdateFileName_, wxString( wxT( ".tgz" ) ), boSilentMode );
    }
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UpdateLYNX_M7OrCOUGAR_PDevice( const wxString& updateFileName, const wxString& /* fileExtension */, bool boSilentMode )
//-----------------------------------------------------------------------------
{
    ConvertedString serial( pDev_->serial.read() );
    wxFileDialog fileDlg( pParent_, wxT( "Please select the firmware file" ), firmwareUpdateFolder_, updateFileName, wxString::Format( wxT( "%s update file (%s)|%s" ), ConvertedString( pDev_->product.read() ).c_str(), updateFileName.c_str(), updateFileName.c_str() ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if( fileDlg.ShowModal() != wxID_OK )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
        }
        return urOperationCanceled;
    }

    if( fileDlg.GetFilename().Find( productStringForFirmwareUpdateCheck_.c_str() ) != 0 )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "The file %s is not meant for updating device %s.\n" ), fileDlg.GetFilename().c_str(), serial.c_str() ), wxTextAttr( wxColour( 255, 0, 0 ) ) );
        }
        return urInvalidFileSelection;
    }

    if( !MessageToUser( wxT( "Firmware Update" ), wxString::Format( wxT( "Are you sure you want to update the firmware of device %s(%s)?\nThis might take several minutes during which the application will not react.\n\nPlease be patient and do NOT disconnect the device from the power supply during this time.\n\nWARNING: During the firmware update all data stored on the device by a user\nlike applications or installed ipks will be deleted!" ), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "firmware update canceled for device %s(%s).\n" ), serial.c_str(), ConvertedString( pDev_->product.read() ).c_str() ) );
        }
        return urOperationCanceled;
    }

    try
    {
        pDev_->open();
    }
    catch( const ImpactAcquireException& e )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to open device %s(%s)(%s, %s).\n" ),
                                       serial.c_str(),
                                       ConvertedString( pDev_->product.read() ).c_str(),
                                       ConvertedString( e.getErrorString() ).c_str(),
                                       ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
        return urDeviceAccessError;
    }

    try
    {
        UpdateFeatures uf( pDev_ );
        uf.transferMode_.writeS( "Binary" );

        wxString directory( fileDlg.GetDirectory() );
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxT( "Installing updates. This will take some minutes. During this time the application will not react.\n" ) );
        }

        // upload the file
        uf.sourceFileName_.write( std::string( wxConvCurrent->cWX2MB( fileDlg.GetPath().c_str() ) ) );
        uf.destinationFileName_.write( std::string( wxConvCurrent->cWX2MB( fileDlg.GetFilename().c_str() ) ) );
        int functionResult = DMR_NO_ERROR;
        if( ( functionResult = uf.upload_.call( std::string( wxConvCurrent->cWX2MB( wxT( "" ) ) ) ) ) == DMR_NO_ERROR )
        {
            bool boUpdateDone = false;
            bool boNewUpdateFeatureUsed = false;
            // install the file
            if( uf.updateFirmware_.isValid() ) // check if the new method is available
            {
                if( ( functionResult = uf.updateFirmware_.call( std::string( wxConvCurrent->cWX2MB( wxT( "" ) ) ) ) ) == DMR_NO_ERROR )
                {
                    boUpdateDone = true;
                    boNewUpdateFeatureUsed = true;
                }
                else
                {
                    if( pParent_ )
                    {
                        pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to install %s using the new firmware update procedure: %d(%s) %s.\n" ),
                                                   fileDlg.GetFilename().c_str(),
                                                   functionResult,
                                                   ConvertedString( ImpactAcquireException::getErrorCodeAsString( functionResult ) ).c_str(),
                                                   ConvertedString( uf.lastResult_.read() ).c_str() ) );
                    }
                }
            }

            if( !boUpdateDone )
            {
                if( pParent_ )
                {
                    pParent_->WriteLogMessage( wxT( "Trying deprecated firmware update procedure now.\n" ) );
                }
                if( ( functionResult = uf.install_.call( std::string( wxConvCurrent->cWX2MB( wxT( "" ) ) ) ) ) == DMR_NO_ERROR )
                {
                    boUpdateDone = true;
                }
                else if( pParent_ )
                {
                    pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to install %s: %d(%s) %s\n" ),
                                               fileDlg.GetFilename().c_str(),
                                               functionResult,
                                               ConvertedString( ImpactAcquireException::getErrorCodeAsString( functionResult ) ).c_str(),
                                               ConvertedString( uf.lastResult_.read() ).c_str() ) );
                }
            }
            if( boUpdateDone )
            {
                if( pParent_ )
                {
                    pParent_->WriteLogMessage( wxString::Format( wxT( "Result of the installation of %s: %s\n" ), fileDlg.GetPath().c_str(), ConvertedString( uf.lastResult_.read() ).c_str() ) );
                }
                // download the installation log file
                wxString logFileName;
                wxString fullLogFilePath;
                if( boNewUpdateFeatureUsed )
                {
                    // during the new method the device will reboot, thus we must re-establish the connection to it.
                    pDev_->close();
                    pDev_->open();
                    uf.bindFeatures();
                    DeviceComponentLocator locator( pDev_, dltInfo );
                    PropertyS firmwareLogFilePathProp( locator.findComponent( "DeviceFirmwareUpdateLogFilePath" ) );
                    if( firmwareLogFilePathProp.isValid() )
                    {
                        fullLogFilePath = ConvertedString( firmwareLogFilePathProp.read() );
                        size_t pos = fullLogFilePath.find_last_of( wxT( "/" ) );
                        if( pos != wxString::npos )
                        {
                            logFileName = fullLogFilePath.Mid( pos + 1 );
                        }
                        else if( pParent_ )
                        {
                            pParent_->WriteLogMessage( wxString::Format( wxT( "Can't obtain log file name from path %s\n" ), fullLogFilePath.c_str() ) );
                        }
                    }
                    else if( pParent_ )
                    {
                        pParent_->WriteLogMessage( wxT( "Can't obtain log file path\n" ) );
                    }
                }
                else
                {
                    logFileName = fileDlg.GetFilename();
                    logFileName.append( wxT( ".install.log" ) );
                    fullLogFilePath = wxT( "/tmp/" );
                    fullLogFilePath.append( logFileName );
                }
                uf.sourceFileName_.write( std::string( wxConvCurrent->cWX2MB( fullLogFilePath.c_str() ) ) );
                wxString logFileDestinationName( directory );
                logFileDestinationName.append( wxT( "/" ) );
                logFileDestinationName.append( logFileName );
                uf.destinationFileName_.write( std::string( wxConvCurrent->cWX2MB( logFileDestinationName.c_str() ) ) );
                uf.download_.call( std::string( wxConvCurrent->cWX2MB( wxT( "" ) ) ) );
                if( pParent_ )
                {
                    pParent_->WriteLogMessage( wxString::Format( wxT( "Result of the downloading the installation log file %s to %s: %s\n" ), fileDlg.GetFilename().c_str(), logFileDestinationName.c_str(), ConvertedString( uf.lastResult_.read() ).c_str() ) );
                }

                // try to display and remove the *.log-file
                {
                    wxFFile logFile( logFileDestinationName.c_str(), wxT( "r" ) );
                    if( logFile.IsOpened() )
                    {
                        wxString logData;
                        logFile.ReadAll( &logData );
                        if( pParent_ )
                        {
                            pParent_->WriteLogMessage( logData );
                        }
                    }
                    else if( pParent_ )
                    {
                        pParent_->WriteLogMessage( wxString::Format( wxT( "ERROR! Couldn't open temporary file %s for reading.\n" ), logFileDestinationName.c_str() ), wxTextAttr( wxColour( 255, 0, 0 ) ) );
                    }
                }

                if( !::wxRemoveFile( logFileDestinationName ) && pParent_ )
                {
                    pParent_->WriteLogMessage( wxString::Format( wxT( "ERROR! Couldn't delete temporary file %s. Must be deleted manually.\n" ), logFileDestinationName.c_str() ), wxTextAttr( wxColour( 255, 0, 0 ) ) );
                }

                MessageToUser( wxT( "Information" ), wxT( "The firmware has been updated. Please close this and all other applications\nusing this device and disconnect the device from the power supply.\nAfter rebooting the new firmware will be active.\n" ), boSilentMode, wxOK | wxICON_INFORMATION );
            }
        }
        else if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to upload %s: %d(%s) %s\n" ),
                                       fileDlg.GetFilename().c_str(),
                                       functionResult,
                                       ConvertedString( ImpactAcquireException::getErrorCodeAsString( functionResult ) ).c_str(),
                                       ConvertedString( uf.lastResult_.read() ).c_str() ) );
        }
    }
    catch( const ImpactAcquireException& e )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to update device %s(%s)(%s, %s).\n" ),
                                       serial.c_str(),
                                       ConvertedString( pDev_->product.read() ).c_str(),
                                       ConvertedString( e.getErrorString() ).c_str(),
                                       ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
        pDev_->close();
        return urDeviceAccessError;
    }
    pDev_->close();
    return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueDevice::UploadFile( const wxString& fullPath, const wxString& descriptionFile )
//-----------------------------------------------------------------------------
{
    int result = urOperationSuccessful;
    const wxString serial( ConvertedString( pDev_->serial.read() ) );
    try
    {
        SelectCustomGenICamFile( descriptionFile );
        pDev_->open();
    }
    catch( const ImpactAcquireException& e )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to open device %s(%s)(%s, %s).\n" ),
                                       serial.c_str(),
                                       ConvertedString( pDev_->product.read() ).c_str(),
                                       ConvertedString( e.getErrorString() ).c_str(),
                                       ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
        return urDeviceAccessError;
    }
    try
    {
        UpdateFeatures uf( pDev_ );
        uf.transferMode_.writeS( "Binary" );
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Installing %s. This will take some minutes. During this time the application will not react.\n" ), fullPath.c_str() ) );
        }

        // upload the file
        uf.sourceFileName_.write( std::string( wxConvCurrent->cWX2MB( fullPath.c_str() ) ) );
        uf.upload_.call( std::string( wxConvCurrent->cWX2MB( wxT( "" ) ) ) );
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Result of the installation of %s: %s\n" ), fullPath.c_str(), ConvertedString( uf.lastResult_.read() ).c_str() ) );
        }
    }
    catch( const ImpactAcquireException& e )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to update device %s(%s)(%s, %s).\n" ),
                                       serial.c_str(),
                                       ConvertedString( pDev_->product.read() ).c_str(),
                                       ConvertedString( e.getErrorString() ).c_str(),
                                       ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
        result = urDeviceAccessError;
    }
    pDev_->close();
    return result;
}

//-----------------------------------------------------------------------------
DeviceHandler::TUpdateResult DeviceHandlerBlueDevice::UploadFirmwareFile( mvIMPACT::acquire::GenICam::ODevFileStream& uploadFile, bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize, const wxFileOffset fileOperationBlockSize )
//-----------------------------------------------------------------------------
{
    uploadFile.open( pDev_, "DeviceFirmware" );
    if( !uploadFile.fail() )
    {
        wxFileOffset bytesWritten = 0;
        wxFileOffset bytesToWriteTotal = bufSize;
        std::ostringstream oss;
        oss << std::setw( 8 ) << std::setfill( '0' ) << bytesWritten << '/'
            << std::setw( 8 ) << std::setfill( '0' ) << bytesToWriteTotal;
        wxProgressDialog dlgFWUpdate( wxT( "Applying Firmware Update" ),
                                      wxString::Format( wxT( "Applying Firmware Update(%s bytes written)" ), ConvertedString( oss.str() ).c_str() ),
                                      bytesToWriteTotal / fileOperationBlockSize, // range
                                      pParent_,          // parent
                                      wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME );
        while( bytesWritten < bytesToWriteTotal )
        {
            const wxFileOffset bytesToWrite = ( ( bytesWritten + fileOperationBlockSize ) <= bytesToWriteTotal ) ? fileOperationBlockSize : bytesToWriteTotal - bytesWritten;
            uploadFile.write( pBuf + bytesWritten, bytesToWrite );
            bytesWritten += bytesToWrite;
            std::ostringstream progress;
            progress << std::setw( 8 ) << std::setfill( '0' ) << bytesWritten << '/'
                     << std::setw( 8 ) << std::setfill( '0' ) << bytesToWriteTotal;
            dlgFWUpdate.Update( bytesWritten / fileOperationBlockSize, wxString::Format( wxT( "Applying Firmware Update(%s bytes written)" ), ConvertedString( progress.str() ).c_str() ) );
        }
        if( !uploadFile.good() )
        {
            MessageToUser( wxT( "Warning" ), wxString::Format( wxT( "Failed to upload firmware file to device %s." ), ConvertedString( serial ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
            uploadFile.close();
            return urFileIOError;
        }
    }
    else
    {
        MessageToUser( wxT( "Warning" ), wxString::Format( wxT( "Failed to open firmware file on device %s." ), ConvertedString( serial ).c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
        return urFileIOError;
    }
    return urOperationSuccessful;
}

//-----------------------------------------------------------------------------
void DeviceHandlerBlueDevice::UserSetBackup( void )
//-----------------------------------------------------------------------------
{
#if defined(linux) || defined(__linux) || defined(__linux__)
    temporaryFolder_ = wxT( "/tmp" );
#elif defined __WIN32__
    wxGetEnv( "TEMP", &temporaryFolder_ );
#else
    temporaryFolder_ = firmwareUpdateFolder_;
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    userSetsToKeepDuringUpdate_.clear();
    if( pDev_->isOpen() )
    {
        pDev_->close();
    }
    pDev_->interfaceLayout.write( dilGenICam );
    SelectCustomGenICamFile( GenICamFile_ );
    pDev_->open();
    FunctionInterface fi( pDev_ );
    mvIMPACT::acquire::GenICam::UserSetControl usc( pDev_ );
    const unsigned int userSetCnt = usc.userSetSelector.dictSize() ;
    // As of now, the Default UserSet will not be touched, thus the number of UserSets to be saved will be dictSize-1.
    wxProgressDialog dlg( wxT( "Backing up UserSet settings" ),
                          wxT( "                                                       " ),
                          userSetCnt - 1 , // range
                          pParent_,     // parent
                          wxPD_AUTO_HIDE | wxPD_APP_MODAL );
    for( unsigned int i = 0; i < userSetCnt; i++ )
    {
        const wxString currentUserSetString( usc.userSetSelector.getTranslationDictString( i ).c_str(), wxConvUTF8 );
        if( currentUserSetString == wxT( "Default" ) )
        {
            continue;
        }
        usc.userSetSelector.writeS( std::string( currentUserSetString.mb_str() ) );
        const wxString FQPath = temporaryFolder_ + wxT( "/" ) + wxString( pDev_->serial.readS().c_str(), wxConvUTF8 ) + wxT( "-" ) + currentUserSetString;
        dlg.Update( i , wxString::Format( wxT( "Backing up '%s'..." ), currentUserSetString.c_str() ) );
        const TDMR_ERROR result = static_cast<TDMR_ERROR>( usc.userSetLoad.call() );
        if( result == DMR_NO_ERROR )
        {
            fi.saveSetting( std::string( FQPath.mb_str() ), sfFile, sUser );
            userSetsToKeepDuringUpdate_.push_back( std::string( currentUserSetString.mb_str() ) );
        }
        else if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to store '%s'. Error code: %d(%s). NOTE: This might only indicate that this device does not have a set stored at this position!\n" ), currentUserSetString.c_str(), result, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
        }
    }
    pDev_->close();
}

//-----------------------------------------------------------------------------
void DeviceHandlerBlueDevice::UserSetRestore( const wxString& previousUserSetDefaultValueToRestore )
//-----------------------------------------------------------------------------
{
    if( pDev_->isOpen() )
    {
        pDev_->close();
    }
    pDev_->interfaceLayout.write( dilGenICam );
    SelectCustomGenICamFile();
    pDev_->open();
    FunctionInterface fi( pDev_ );
    mvIMPACT::acquire::GenICam::UserSetControl usc( pDev_ );
    const std::vector<std::string>::size_type userSetCnt = userSetsToKeepDuringUpdate_.size();
    wxProgressDialog dlg( wxT( "Restoring UserSet settings" ),
                          wxT( "                                                       " ),
                          static_cast<int>( userSetCnt ), // range
                          pParent_,     // parent
                          wxPD_AUTO_HIDE | wxPD_APP_MODAL );

    int numberOfFilesRestored = 0;
    for( std::vector<std::string>::size_type i = 0; i < userSetCnt; i++ )
    {
        usc.userSetSelector.writeS( userSetsToKeepDuringUpdate_[i] );
        const wxString currentUserSetString( ConvertedString( userSetsToKeepDuringUpdate_[i].c_str() ) );
        const wxString FQPath = temporaryFolder_ + wxT( "/" ) + wxString( pDev_->serial.readS().c_str(), wxConvUTF8 ) + wxT( "-" ) + currentUserSetString + wxT( ".xml" );
        dlg.Update( numberOfFilesRestored + 1 , wxString::Format( wxT( "Restoring '%s'..." ), currentUserSetString.c_str() ) );
        TDMR_ERROR result = static_cast<TDMR_ERROR>( fi.loadSetting( std::string( FQPath.mb_str() ), sfFile, sUser ) );
        if( result == DMR_NO_ERROR )
        {
            result = static_cast<TDMR_ERROR>( usc.userSetSave.call() );
            if( pParent_ )
            {
                if( result != DMR_NO_ERROR )
                {
                    pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to restore '%s'! Error code: %d(%s).\n" ), currentUserSetString.c_str(), result, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
                }
                else
                {
                    pParent_->WriteLogMessage( wxString::Format( wxT( "Successfully restored '%s'.\n" ), currentUserSetString.c_str() ) );
                }
            }
            wxRemoveFile( FQPath );
            ++numberOfFilesRestored;
        }
        else if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to restore '%s'. Error code: %d(%s). NOTE: This might only indicate that this device does not have a set stored at this position!\n" ), currentUserSetString.c_str(), result, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
        }
    }
    try
    {
        const std::string previousUserSetDefaultValueToRestoreANSI( previousUserSetDefaultValueToRestore.mb_str() );
        usc.userSetSelector.writeS( previousUserSetDefaultValueToRestoreANSI );
        const TDMR_ERROR result = static_cast<TDMR_ERROR>( usc.userSetLoad.call() );
        if( result == DMR_NO_ERROR )
        {
            // The setting can still be loaded. Restore the old default value in non-volatile memory
            usc.userSetDefault.writeS( previousUserSetDefaultValueToRestoreANSI );
        }
        else if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to load user set '%s', will NOT restore the previous value of 'UserSetDefault'! Error: %d(%s).\n" ), previousUserSetDefaultValueToRestore.c_str(), result, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
        }
    }
    catch( const ImpactAcquireException& e )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to set 'UserSetSelector' to the UserSet specified in 'UserSetDefault'! Error: %d(%s).\n" ), e.getErrorCode(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
    }
    pDev_->close();
}

//-----------------------------------------------------------------------------
wxString DeviceHandlerBlueDevice::SetUserSetDefault( const wxString& userSetDefaultValue )
//-----------------------------------------------------------------------------
{
    if( pDev_->isOpen() )
    {
        pDev_->close();
    }
    pDev_->interfaceLayout.write( dilGenICam );
    SelectCustomGenICamFile( GenICamFile_ );
    pDev_->open();

    wxString previousValue( wxT( "Default" ) );
    try
    {
        const std::string userSetDefaultValueANSI( userSetDefaultValue.mb_str() );
        mvIMPACT::acquire::GenICam::UserSetControl usc( pDev_ );
        if( usc.userSetDefault.isValid() )
        {
            previousValue = ConvertedString( usc.userSetDefault.readS() );
            usc.userSetDefault.writeS( userSetDefaultValueANSI );
        }
    }
    catch( const ImpactAcquireException& e )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Failed to set 'UserSetDefault' to '%s'! Error: %d(%s).\n" ), userSetDefaultValue.c_str(), e.getErrorCode(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
        }
    }
    pDev_->close();
    return previousValue;
}
