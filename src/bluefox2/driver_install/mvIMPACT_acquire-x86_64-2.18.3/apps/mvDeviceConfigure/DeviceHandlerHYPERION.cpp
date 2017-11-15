//-----------------------------------------------------------------------------
#include <apps/Common/FirmwareUpdate_mvHYPERION/Epcs.h>
#include <apps/Common/wxAbstraction.h>
#include <common/auto_array_ptr.h>
#include "DeviceConfigureFrame.h"
#include "DeviceHandlerHYPERION.h"
#include "KernelDriverUpdate.h"
#include <sstream>
#include "wx/msgdlg.h"
#include "wx/ffile.h"
#include "wx/numdlg.h"

using namespace std;

//-----------------------------------------------------------------------------
bool DeviceHandlerHYPERION::SupportsDMABufferSizeUpdate( int* pCurrentDMASize_kB /* = 0 */ )
//-----------------------------------------------------------------------------
{
    if( pCurrentDMASize_kB )
    {
        try
        {
            ComponentLocator locator( pDev_->hDev() );
            Method readDMABufferSize( locator.findComponent( "ReadPermanentDMABufferSize@i" ) );
            *pCurrentDMASize_kB = readDMABufferSize.call() / 1024;
        }
        catch( const ImpactAcquireException& e )
        {
            if( pParent_ )
            {
                pParent_->WriteErrorMessage( wxString::Format( wxT( "Failed to query DMA memory size for %s(%s, %s).\n" ),
                                             ConvertedString( pDev_->serial.read() ).c_str(),
                                             ConvertedString( e.getErrorString() ).c_str(),
                                             ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            }
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
int DeviceHandlerHYPERION::UpdatePermanentDMABufferSize( bool /*boSilentMode*/ )
//-----------------------------------------------------------------------------
{
    static const long MB = 1024 * 1024;

    int result = urOperationSuccessful;
    string devSerial = pDev_->serial.read();
    if( pParent_ )
    {
        try
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxT( "Trying to update permanent DMA buffer size now. Please note that this change will not become effective until the system has been rebooted.\n" ) );
            }
            ComponentLocator locator( pDev_->hDev() );
            Method readDMABufferSize( locator.findComponent( "ReadPermanentDMABufferSize@i" ) );
            Method writeDMABufferSize( locator.findComponent( "WritePermanentDMABufferSize@ii" ) );
            const int currentDMASize = readDMABufferSize.call();
            long newSize_Megabytes = wxGetNumberFromUser( wxT( "Enter the new size for the permanent DMA buffer.\nPlease note that this change will not become effective until the system has been rebooted.\n" ),
                                     wxT( "New Size in Megabytes:" ),
                                     wxString::Format( wxT( "Set New DMA Buffer Size For %s(%s)" ), ConvertedString( devSerial ).c_str(), ConvertedString( pDev_->family.read() ).c_str() ),
                                     ( ( currentDMASize < 0 ? 0 : currentDMASize ) / MB ), // start
                                     0, // min
                                     0x7FFFFFFF, // max
                                     pParent_ );
            if( newSize_Megabytes >= 0 )
            {
                std::stringstream sstr;
                sstr.str( "" );
                sstr << ( newSize_Megabytes * MB ) << endl;
                const int result = writeDMABufferSize.call( sstr.str() );
                if( result != DMR_NO_ERROR )
                {
                    pParent_->WriteErrorMessage( wxString::Format( wxT( "Failed to update DMA memory: %s(%s).\n" ),
                                                 ConvertedString( devSerial ).c_str(),
                                                 ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ) );
                }
            }
            else
            {
                result = urOperationCanceled;
                pParent_->WriteLogMessage( wxT( "Operation canceled by the user.\n" ) );
            }
        }
        catch( const ImpactAcquireException& e )
        {
            if( pParent_ )
            {
                pParent_->WriteErrorMessage( wxString::Format( wxT( "Failed to update DMA memory: %s(%s, %s).\n" ),
                                             ConvertedString( devSerial ).c_str(),
                                             ConvertedString( e.getErrorString() ).c_str(),
                                             ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            }
            result = urDeviceAccessError;
        }
    }
    else
    {
        result = urDeviceAccessError;
    }
    return result;
}

//-----------------------------------------------------------------------------
int DeviceHandlerHYPERION::UpdateFirmware( bool boSilentMode, bool /*boPersistentUserSets*/ )
//-----------------------------------------------------------------------------
{
    int result = urOperationSuccessful;
    wxFileDialog fileDlg( NULL, wxT( "Load an existing raw programming data file" ), wxT( "" ), wxT( "" ), wxT( "raw-programming-data Files (*.rpd)|*.rpd" ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        MessageToUser( wxT( "Information" ), wxT( "The firmware will now be updated. During this time(approx. 90 sec.) the application will not react. Please be patient." ), boSilentMode, wxOK | wxICON_INFORMATION );
        wxFFile file( fileDlg.GetPath().c_str(), wxT( "rb" ) );
        if( !file.IsOpened() )
        {
            if( pParent_ )
            {
                pParent_->WriteLogMessage( wxString::Format( wxT( "ERROR! Could not open file %s.\n" ), fileDlg.GetPath().c_str() ), wxTextAttr( wxColour( 255, 0, 0 ) ) );
            }
            return urFileIOError;
        }
        auto_array_ptr<unsigned char> pFileBuffer( file.Length() );
        int bytesRead = static_cast<int>( file.Read( pFileBuffer.get(), file.Length() ) );
        file.Close();
        ostringstream logMsg;
        if( FlashUpdate_mvHYPERION( pDev_, pFileBuffer, static_cast<unsigned long>( bytesRead ), logMsg ) == false )
        {
            result = urDeviceAccessError;
        }
        if( pParent_ )
        {
            const wxString msg( wxString::Format( wxT( "%s" ), ConvertedString( logMsg.str() ).c_str() ) );
            if( result != urOperationSuccessful )
            {
                pParent_->WriteErrorMessage( msg );
            }
            else
            {
                pParent_->WriteLogMessage( msg );
            }
        }
    }
    return result;
}
