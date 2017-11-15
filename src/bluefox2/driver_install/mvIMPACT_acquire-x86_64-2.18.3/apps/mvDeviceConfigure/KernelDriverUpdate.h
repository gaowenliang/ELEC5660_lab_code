//-----------------------------------------------------------------------------
#ifndef KernelDriverUpdateH
#define KernelDriverUpdateH KernelDriverUpdateH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <string>
#include <sstream>

//-----------------------------------------------------------------------------
enum TKernelDriverUpdateResult
//-----------------------------------------------------------------------------
{
    kdurOK,
    kdurUnsupported,
    kdurFailed
};

bool SupportsKernelDriverFeature( mvIMPACT::acquire::Device* pDev, std::string& kernelDriverName, bool& newerDriverAvailable );
TKernelDriverUpdateResult UpdateKernelDriver( const std::string& famliyName, std::ostringstream& resultMsg );
inline std::string GetUnavailableString( void )
{
    return "unavailable";
}

#endif // KernelDriverUpdateH
