//-----------------------------------------------------------------------------
#ifndef DeviceHandlerBlueDeviceH
#define DeviceHandlerBlueDeviceH DeviceHandlerBlueDeviceH
//-----------------------------------------------------------------------------
#include <common/auto_array_ptr.h>
#include "DeviceHandler.h"
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam_FileStream.h>
#include "PackageDescriptionParser.h"

//-----------------------------------------------------------------------------
class DeviceHandlerBlueDevice : public DeviceHandler
//-----------------------------------------------------------------------------
{
    enum TProductGroup
    {
        pgUnknown,
        pgBlueCOUGAR_P,
        pgBlueCOUGAR_S,
        pgBlueCOUGAR_X,
        pgBlueCOUGAR_Y,
        pgBlueFOX3,
        pgBlueLYNX_M7
    };
    TProductGroup product_;
    wxString productStringForFirmwareUpdateCheck_;
    wxString firmwareUpdateFileName_;
    wxString firmwareUpdateFolder_;
    wxString firmwareUpdateDefaultFolder_;
    wxString firmwareUpdateFolderDevelopment_;
    wxString GenICamFile_;
    wxString temporaryFolder_;
    std::vector<std::string> userSetsToKeepDuringUpdate_;
    int CheckForIncompatibleFirmwareVersions_BlueCOUGAR_X( bool boSilentMode, const wxString& serial, const FileEntryContainer& fileEntries, const wxString& selection, const Version& currentFirmwareVersion );
    TUpdateResult DoFirmwareUpdate_BlueCOUGAR_XOrY( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize );
    TUpdateResult DoFirmwareUpdate_BlueFOX3( bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize );
    bool ExtractFileVersion( const wxString& fileName, Version& fileVersion ) const;
    static bool GetFileFromArchive( const wxString& firmwareFileAndPath, const char* pArchive, size_t archiveSize, const wxString& filename, auto_array_ptr<char>& data, DeviceConfigureFrame* pParent );
    int GetLatestFirmwareVersionCOUGAR_XAndYOrFOX3Device( Version& latestFirmwareVersion ) const;
    wxString GetProductFromManufacturerInfo( void ) const;
    std::map<wxString, SuitableProductKey>::const_iterator GetSuitableFirmwareIterator( const FileEntry& entry, const wxString& product, const wxString& productFromManufacturerInfo ) const;
    bool IsBlueCOUGAR_X( void ) const;
    bool IsBlueCOUGAR_Y( void ) const;
    bool IsBlueFOX3( void ) const
    {
        return ( ConvertedString( pDev_->product.read() ).Find( wxT( "mvBlueFOX3" ) ) == wxNOT_FOUND ) ? false : true;
    }
    bool IsFirmwareUpdateMeaningless( bool boSilentMode, const Version& deviceFWVersion, const Version& selectedFWVersion, const wxString& defaultFWArchive, const Version& defaultFolderFWVersion ) const;
    static TUpdateResult ParseUpdatePackageCOUGAR_XOrFOX3Device( PackageDescriptionFileParser& fileParser, const wxString& firmwareFileAndPath, DeviceConfigureFrame* pParent, auto_array_ptr<char>& pBuffer );
    void RebootDevice( bool boSilentMode, const wxString& serial );
    void SelectCustomGenICamFile( const wxString& descriptionFile = wxEmptyString );
    wxString SetUserSetDefault( const wxString& userSetDefaultValue );
    int UpdateCOUGAR_SDevice( bool boSilentMode );
    int UpdateCOUGAR_XOrYOrFOX3Device( bool boSilentMode, bool boPersistentUserSets );
    int UpdateLYNX_M7OrCOUGAR_PDevice( const wxString& updateFileName, const wxString& fileExtension, bool boSilentMode );
    int UploadFile( const wxString& fullPath, const wxString& descriptionFile );
    TUpdateResult UploadFirmwareFile( mvIMPACT::acquire::GenICam::ODevFileStream& uploadFile, bool boSilentMode, const wxString& serial, const char* pBuf, const size_t bufSize, const wxFileOffset fileOperationBlockSize );
    void UserSetBackup( void );
    void UserSetRestore( const wxString& previousUserSetDefaultValueToRestore );
public:
    DeviceHandlerBlueDevice( mvIMPACT::acquire::Device* pDev );
    static DeviceHandler* Create( mvIMPACT::acquire::Device* pDev )
    {
        return new DeviceHandlerBlueDevice( pDev );
    }
    virtual bool GetIDFromUser( long& newID, const long minValue, const long maxValue );
    virtual int GetLatestFirmwareVersion( Version& latestFirmwareVersion ) const;
    virtual void SetCustomFirmwareFile( const wxString& customFirmwareFile );
    virtual void SetCustomFirmwarePath( const wxString& customFirmwarePath );
    virtual void SetCustomGenICamFile( const wxString& customGenICamFile )
    {
        GenICamFile_ = customGenICamFile;
    }
    virtual bool SupportsFirmwareUpdate( void ) const
    {
        return !firmwareUpdateFileName_.IsEmpty();
    }
    virtual int UpdateFirmware( bool boSilentMode, bool boPersistentUserSets );
};

int        CompareFileVersion( const wxString& first, const wxString& second );
wxString   ExtractVersionNumber( const wxString& s );
bool       GetNextNumber( wxString& str, long& number );
int64_type MACAddressFromString( const std::string& MAC );

#endif // DeviceHandlerBlueDeviceH
