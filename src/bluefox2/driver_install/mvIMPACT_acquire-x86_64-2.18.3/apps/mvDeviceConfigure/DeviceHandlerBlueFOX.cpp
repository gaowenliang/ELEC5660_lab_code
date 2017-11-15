//-----------------------------------------------------------------------------
#include "DeviceConfigureFrame.h"
#include "DeviceHandlerBlueFOX.h"
#include "KernelDriverUpdate.h"
#include <sstream>
#include "wx/msgdlg.h"
#include <apps/Common/wxAbstraction.h>

using namespace std;

//-----------------------------------------------------------------------------
bool DeviceHandlerBlueFOX::SupportsKernelDriverUpdate( bool& boNewerDriverAvailable, std::string& kernelDriverName )
//-----------------------------------------------------------------------------
{
    return SupportsKernelDriverFeature( pDev_, kernelDriverName, boNewerDriverAvailable );
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueFOX::UpdateFirmware( bool boSilentMode, bool /*boPersistentUserSets*/ )
//-----------------------------------------------------------------------------
{
    const ConvertedString serial( pDev_->serial.read() );
    const ConvertedString product( pDev_->product.read() );
    if( MessageToUser( wxT( "Firmware Update" ), wxString::Format( wxT( "Are you sure you want to update the firmware of device %s?" ), serial.c_str() ), boSilentMode, wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION ) &&
        MessageToUser( wxT( "Information" ), wxT( "The firmware will now be updated. During this time(approx. 30 sec.) the application will not react. Please be patient." ), boSilentMode, wxOK | wxICON_INFORMATION ) )
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Updating firmware of device %s(%s). This might take some time. Please be patient.\n" ), serial.c_str(), product.c_str() ) );
        }
        int result = pDev_->updateFirmware();
        if( pParent_ )
        {
            if( result == DMR_FEATURE_NOT_AVAILABLE )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "Device %s(%s) doesn't support firmware updates.\n" ), serial.c_str(), product.c_str() ) );
            }
            else if( result != DMR_NO_ERROR )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "An error occurred: %s(please refer to the manual for this error code).\n" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
            }
            else
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "%s.\n" ), ConvertedString( pDev_->HWUpdateResult.readS() ).c_str() ) );
            }
        }
        if( pDev_->HWUpdateResult.read() == urUpdateFWOK )
        {
            MessageToUser( wxT( "Update Result" ), wxString::Format( wxT( "Update successful. Please disconnect and reconnect device %s(%s) now to activate the new firmware." ), serial.c_str(), product.c_str() ), boSilentMode, wxOK | wxICON_INFORMATION );
        }
        return urOperationSuccessful;
    }
    else
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Firmware update canceled for device %s(%s).\n" ), serial.c_str(), product.c_str() ) );
        }
        return urOperationCanceled;
    }
}

//-----------------------------------------------------------------------------
int DeviceHandlerBlueFOX::UpdateKernelDriver( bool boSilentMode )
//-----------------------------------------------------------------------------
{
    if( MessageToUser( wxT( "Kernel Driver Update" ), wxString::Format( wxT( "Are you sure you want to update the kernel driver for every device belonging to the\n%s family that is currently connected?" ), ConvertedString( pDev_->family.read() ).c_str() ), boSilentMode, wxCANCEL | wxOK | wxICON_INFORMATION ) )
    {
        ostringstream resultMsg;
        wxString str;

        if( ::UpdateKernelDriver( pDev_->family.read(), resultMsg ) != kdurOK )
        {
            wxTextAttr redStyle( wxColour( 255, 0, 0 ) );
            str = wxT( "Failed to update kernel driver. Message: " );
            str.append( ConvertedString( resultMsg.str() ) );
            if( pParent_ )
            {
                pParent_->WriteLogMessage( str, redStyle );
            }
        }
        else
        {
            str = wxT( "Update successful." );
        }
        MessageToUser( wxT( "Update Result" ), str, boSilentMode, wxOK | wxICON_INFORMATION );
        return urOperationSuccessful;
    }
    else
    {
        if( pParent_ )
        {
            pParent_->WriteLogMessage( wxString::Format( wxT( "Kernel driver update for %s devices canceled.\n" ), ConvertedString( pDev_->family.read() ).c_str() ) );
        }
        return urOperationCanceled;
    }
}
