//-----------------------------------------------------------------------------
#ifndef DeviceHandlerH
#define DeviceHandlerH DeviceHandlerH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "PackageDescriptionParser.h"

class DeviceConfigureFrame;

//-----------------------------------------------------------------------------
class DeviceHandler
//-----------------------------------------------------------------------------
{
    bool boSetIDSupported_;
protected:
    mvIMPACT::acquire::Device* pDev_;
    DeviceConfigureFrame* pParent_;
    wxString customFirmwarePath_;
    bool MessageToUser( const wxString& caption, const wxString& msg, bool boSilentMode, long style ) const;
public:
    DeviceHandler( mvIMPACT::acquire::Device* pDev, bool boSetIDSupported = false ) : boSetIDSupported_( boSetIDSupported ), pDev_( pDev ), pParent_( 0 ) {}
    virtual ~DeviceHandler() {}
    void AttachParent( DeviceConfigureFrame* pParent )
    {
        pParent_ = pParent;
    }
    virtual bool SupportsFirmwareUpdate( void ) const = 0;
    virtual int GetLatestFirmwareVersion( Version& /*latestFirmwareVersion*/ ) const
    {
        return urFeatureUnsupported;
    }
    virtual bool SupportsKernelDriverUpdate( bool& /*boNewerDriverAvailable*/, std::string& /*kernelDriverName*/ )
    {
        return false;
    }
    bool SupportsSetID( void ) const
    {
        return boSetIDSupported_;
    }
    virtual bool GetIDFromUser( long& newID, const long minValue, const long maxValue );
    virtual bool SupportsDMABufferSizeUpdate( int* /* pCurrentDMASize_kB */ = 0 )
    {
        return false;
    }
    virtual int UpdateFirmware( bool boSilentMode, bool boPersistentUserSets );
    virtual int UpdateKernelDriver( bool boSilentMode );
    virtual int UpdatePermanentDMABufferSize( bool /*boSilentMode*/ )
    {
        return urFeatureUnsupported;
    }
    virtual void SetCustomFirmwareFile( const wxString& /* customFirmwareFile */ ) {}
    virtual void SetCustomFirmwarePath( const wxString& /* customFirmwarePath */ ) {}
    virtual void SetCustomGenICamFile( const wxString& /* customGenICamFile */ ) {}
    //-----------------------------------------------------------------------------
    enum TUpdateResult
    //-----------------------------------------------------------------------------
    {
        urOperationSuccessful = 0,
        urFeatureUnsupported = 1,
        urOperationCanceled = 2,
        urInvalidFileSelection = 3,
        urFileIOError = 4,
        urDeviceAccessError = 5,
        urInvalidParameter = 6
    };
};

//-----------------------------------------------------------------------------
class DeviceHandler3rdParty : public DeviceHandler
//-----------------------------------------------------------------------------
{
public:
    DeviceHandler3rdParty( mvIMPACT::acquire::Device* pDev ) : DeviceHandler( pDev ) {}
    static DeviceHandler* Create( mvIMPACT::acquire::Device* pDev )
    {
        return new DeviceHandler3rdParty( pDev );
    }
    virtual bool SupportsFirmwareUpdate( void ) const
    {
        return false;
    }
};

#endif // DeviceHandlerH
