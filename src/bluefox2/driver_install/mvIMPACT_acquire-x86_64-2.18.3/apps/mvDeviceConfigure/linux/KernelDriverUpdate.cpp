//-----------------------------------------------------------------------------
#include "../KernelDriverUpdate.h"

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
bool SupportsKernelDriverFeature( Device* /*pDev*/, string& kernelDriverName, bool& newerDriverAvailable )
//-----------------------------------------------------------------------------
{
    newerDriverAvailable = false;
    kernelDriverName = GetUnavailableString();
    return false;
}

//-----------------------------------------------------------------------------
TKernelDriverUpdateResult UpdateKernelDriver( const string& /*famliyName*/, ostringstream& /*resultMsg*/ )
//-----------------------------------------------------------------------------
{
    return kdurUnsupported;
}

