//-----------------------------------------------------------------------------
#ifndef DeviceHandlerBlueFOXH
#define DeviceHandlerBlueFOXH DeviceHandlerBlueFOXH
//-----------------------------------------------------------------------------
#include "DeviceHandler.h"

//-----------------------------------------------------------------------------
class DeviceHandlerBlueFOX : public DeviceHandler
//-----------------------------------------------------------------------------
{
public:
    DeviceHandlerBlueFOX( mvIMPACT::acquire::Device* pDev ) : DeviceHandler( pDev, true ) {}
    static DeviceHandler* Create( mvIMPACT::acquire::Device* pDev )
    {
        return new DeviceHandlerBlueFOX( pDev );
    }
    virtual bool SupportsFirmwareUpdate( void ) const
    {
        return true;
    }
    virtual bool SupportsKernelDriverUpdate( bool& boNewerDriverAvailable, std::string& kernelDriverName );
    virtual int UpdateFirmware( bool boSilentMode, bool boPersistentUserSets );
    virtual int UpdateKernelDriver( bool boSilentMode );
};

#endif // DeviceHandlerBlueFOXH
