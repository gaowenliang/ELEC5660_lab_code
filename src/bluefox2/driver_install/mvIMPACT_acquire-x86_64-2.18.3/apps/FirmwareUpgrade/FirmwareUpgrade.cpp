//-----------------------------------------------------------------------------
#include <apps/Common/FirmwareUpdate_mvHYPERION/Epcs.h>
#include <apps/Common/exampleHelper.h>
#include <common/auto_array_ptr.h>
#include <cstdio>
#include <iostream>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
void printHelp( void )
//-----------------------------------------------------------------------------
{
    cout << " Available commands / usage:" << endl
         << " ------------------------------------" << endl
         << " no parameters: Silent mode (will update the firmware for every mvBlueFOX connected to the system" << endl
         << "                without asking any further questions." << endl
         << endl
         << " -d<serial>: Will update the firmware for a device with the serial number specified by" << endl
         << "             <serial>. Pass '*' as a wildcard." << endl
         << "             EXAMPLE: FirmwareUpgrade -dBF* // will update the firmware for every device with a serial number starting with 'BF'" << endl
         << endl
         << " -hff<path>: mvHYPERION Firmware File. Full path to a firmware file to update mvHYPERION devices." << endl
         << endl
         << " -sel: Will prompt the user to select a device to update" << endl
         << endl
         << " -help: Will print this help." << endl;
}

//-----------------------------------------------------------------------------
int getFileSize( FILE* fp )
//-----------------------------------------------------------------------------
{
    const int cur = ftell( fp );
    fseek( fp, 0, SEEK_END );
    const int size = ftell( fp );
    fseek( fp, cur, SEEK_SET );
    return size;
}

//-----------------------------------------------------------------------------
void updateFirmware( Device* pDev, const string& mvHYPERIONFirmwarePath = "" )
//-----------------------------------------------------------------------------
{
    if( pDev )
    {
        const string family( pDev->family.read() );
        if( family == "mvBlueFOX" )
        {
            cout << "The firmware of device " << pDev->serial.read() << " is currently " << pDev->firmwareVersion.readS() << "." << endl;
            cout << "It will now be updated. During this time(approx. 30 sec.) the application will not react. Please be patient." << endl;
            int result = pDev->updateFirmware();
            if( result == DMR_FEATURE_NOT_AVAILABLE )
            {
                cout << "This device doesn't support firmware updates." << endl;
            }
            else if( result != DMR_NO_ERROR )
            {
                cout << "An error occurred: " << ImpactAcquireException::getErrorCodeAsString( result ) << ". (please refer to the manual for this error code)." << endl;
            }
            else
            {
                cout << "Firmware update done. Result: " << pDev->HWUpdateResult.readS() << endl;
                if( pDev->HWUpdateResult.read() == urUpdateFWOK )
                {
                    cout << "Update successful." << endl;
                    if( pDev->family.read() == "mvBlueFOX" )
                    {
                        cout << "Please disconnect and reconnect the device now to activate the new firmware." << endl;
                    }
                }
            }
        }
        else if( family == "mvHYPERION" )
        {
            if( mvHYPERIONFirmwarePath.empty() != true )
            {
                const string extension( ".rpd" );
                if( mvHYPERIONFirmwarePath.find( extension ) == mvHYPERIONFirmwarePath.length() - extension.length() )
                {
                    FILE* fp = fopen( mvHYPERIONFirmwarePath.c_str(), "rb" );
                    if( fp != 0 )
                    {
                        auto_array_ptr<unsigned char> pFileBuffer( getFileSize( fp ) );
                        memset( pFileBuffer, 0, pFileBuffer.parCnt() );
                        if( ( fread( pFileBuffer, pFileBuffer.parCnt(), 1, fp ) != 1 ) && !feof( fp ) )
                        {
                            cout << "*** Error: Failed to read from file '" << mvHYPERIONFirmwarePath << "'" << endl;
                        }
                        else
                        {
                            cout << "The firmware of device " << pDev->serial.read() << " will now be updated. During this time(approx. 90 sec.) the application will not react. Please be patient." << endl;
                            ostringstream logMsg;
                            FlashUpdate_mvHYPERION( pDev, pFileBuffer, static_cast<unsigned long>( pFileBuffer.parCnt() ), logMsg );
                            cout << logMsg.str();
                        }
                        fclose( fp );
                    }
                    else
                    {
                        cout << "*** Error: Failed to open file '" << mvHYPERIONFirmwarePath << "'" << endl;
                    }
                }
                else
                {
                    cout << "*** Error: Invalid file type for updating mvHYPERION device " << pDev->serial.read() << " specified('" << mvHYPERIONFirmwarePath << "', extension expected: '" << extension << "')" << endl;
                }
            }
            else
            {
                cout << "*** Error: Empty path for updating mvHYPERION device " << pDev->serial.read() << " specified" << endl;
            }
        }
        else
        {
            cout << "*** Error: This application is meant for 'mvBlueFOX' and 'mvHYPERION' devices only. This is an '" << pDev->family.read() << "' device." << endl;
        }
    }
    else
    {
        cout << "*** Error: Invalid device pointer passed to update function." << endl;
    }
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    DeviceManager devMgr;
    const unsigned int devCnt = devMgr.deviceCount();
    if( devCnt == 0 )
    {
        cout << "*** Error: No MATRIX VISION device found! Unable to continue!" << endl;
        return 1;
    }

    cout << "Have found " << devCnt << " devices on this platform!" << endl
         << "Please note that this application will only work for 'mvBlueFOX' and 'mvHYPERION' devices" << endl
         << endl
         << "Detected devices:" << endl;
    for( unsigned int i = 0; i < devCnt; i++ )
    {
        cout << " [" << i << "] " << devMgr[i]->serial.read() << "(" << devMgr[i]->product.read() << ", Firmware version: " << devMgr[i]->firmwareVersion.readS() << ")" << endl;
    }
    cout << endl;

    if( argc == 1 )
    {
        int index = 0;
        Device* pDev = 0;
        while( ( pDev = devMgr.getDeviceByFamily( "mvBlueFOX", index ) ) != 0 )
        {
            updateFirmware( pDev );
            ++index;
        }
    }
    else
    {
        string mvHYPERIONFirmwarePath;
        for( int i = 1; i < argc; i++ )
        {
            const string arg( argv[i] );
            if( arg == "-sel" )
            {
                Device* pDev = getDeviceFromUserInput( devMgr );
                if( !pDev )
                {
                    return 1;
                }
                updateFirmware( pDev );
            }
            else if( arg.find( "-d" ) == 0 )
            {
                string serial( arg.substr( 2 ) );
                int index = 0;
                Device* pDev = 0;
                while( ( pDev = devMgr.getDeviceBySerial( serial, index ) ) != 0 )
                {
                    updateFirmware( pDev, mvHYPERIONFirmwarePath );
                    ++index;
                }
            }
            else if( arg.find( "-hff" ) == 0 )
            {
                mvHYPERIONFirmwarePath = string( arg.substr( 4 ) );
            }
            else if( arg == "-help" )
            {
                printHelp();
            }
            else
            {
                cout << "Invalid input parameter: '" << arg << "'" << endl
                     << endl;
                printHelp();
            }
        }
    }

    return 0;
}
