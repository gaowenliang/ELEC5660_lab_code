//-----------------------------------------------------------------------------
#include <stdio.h>
#include <iostream>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "Timer.h"
#include <common/PGMFileFunctions.h>
//-----------------------------------------------------------------------------

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    const char* pDevSerial = "BF*";

    cout << "\n ++ Start PowerDownTest sample: " << __DATE__ << "/" << __TIME__ << endl;

    if( argc > 1 )
    {
        pDevSerial = argv[1];
    }

    unsigned int uiDevCount = 0;
    DeviceManager DevMgr;
    if( ( uiDevCount = DevMgr.deviceCount() ) == 0 )
    {
        cout << "*** Error: No MATRIX VISION device found! Unable to continue!" << endl;
        return 0;
    }

    cout << "Have found " << uiDevCount << " devices on this platform!" << endl;

    for( unsigned i = 0; i < uiDevCount; i++ )
    {
        Device* pDevTmp = DevMgr.getDevice( i );
        cout << " " << i << " Serial: " << pDevTmp->serial.read() << endl;
    }

    cout << "Initialising the device: " << pDevSerial << ". This might take some time..." << endl;
    // create an interface to the first MATRIX VISION device with the serila number pDevSerial
    Device* pDev = DevMgr.getDeviceBySerial( pDevSerial );

    try
    {
        pDev->open();
    }
    catch( ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "*** Error: An error occurred while opening the device(error code: " << e.getErrorCode() << ")." << endl;
        return 0;
    }

    FunctionInterface fi( pDev );

    // only 8Bit/pixel destination image are supported by the \c writePGMFile() function
    ImageDestination imgDst( pDev );
    imgDst.pixelFormat.write( idpfMono8 );

    // get mvBF system settings
    SystemBlueFOX sbf( pDev );

    bool bPowerDown = false;
    do
    {
        cout << "Ready to snap. Press 'p'<return> to power down, 'q'<return> to quit or <return> to snap an image.." << endl;
        char ch = getchar();

        if( ch == 'p' )
        {
            // for mvBlueFOX only: test power down / up
            cout << "Power off!" << endl;
            sbf.powerMode.write( dpmOff );
            bPowerDown = true;
            // read and discard the <return>
            getchar();
        }
        else if( ch == 'q' )
        {
            // break out of loop to finish application
            // read and discard the <return>
            getchar();
            break;
        }
        else
        {
            if( bPowerDown )
            {
                // first we need to power up again
                CTime timer1;
                sbf.powerMode.write( dpmOn );
                bPowerDown = false;
                cout << "Power On!" << ". Power On took " << timer1.elapsed() << "s., " << endl;
            }

            CTime timer;

            // send a request to the default request queue of the device and wait for the result.
            fi.imageRequestSingle();
            const int iMaxWaitTime_ms = 5000;

            // wait for results from the default capture queue
            int iRequestNr = fi.imageRequestWaitFor( iMaxWaitTime_ms );

            cout << "Request Nr.: " << iRequestNr << ". Snap took " << timer.elapsed() << "s., " << endl;
            // check if the image has been captured without any problems
            if( !fi.isRequestNrValid( iRequestNr ) )
            {
                // this can not happen in this sample, but may happen if you wait for a request without
                // sending one to the driver before
                cout << "*** Error: No request has been sent to the driver." << endl;
                // unlock the buffer to let the driver know that you no longer need this buffer
                fi.imageRequestUnlock( iRequestNr );
                // free resources
                fi.imageRequestReset( 0, 0 );
                pDev->close();
                return 0;
            }

            const Request* pRequest = fi.getRequest( iRequestNr );
            if( !pRequest->isOK() )
            {
                cout << "*** " << pRequest->requestResult << endl;
                // unlock the buffer to let the driver know that you no longer need this buffer
                fi.imageRequestUnlock( iRequestNr );
                // free resources
                fi.imageRequestReset( 0, 0 );
                pDev->close();
                return 0;
            }

            // Everything went well. Save the result
            const char* pImageName = "SingCapt.pgm";
            cout << "Will save the file as:  " << pImageName << endl;
            if( writePGMFile( ( const unsigned char* )pRequest->imageData.read(), pRequest->imageWidth.read(), pRequest->imageHeight.read(),
                              pRequest->imageLinePitch.read(), pRequest->imagePixelPitch.read(), pImageName ) < 0 )
            {
                cout << "*** Error: File: " << pImageName << " could not be saved!!" << endl;
            }
            // unlock the buffer to let the driver know that you no longer need this buffer
            fi.imageRequestUnlock( iRequestNr );
        }
    }
    while( true );

    // free resources
    fi.imageRequestReset( 0, 0 );
    pDev->close();

    return 0;
}
