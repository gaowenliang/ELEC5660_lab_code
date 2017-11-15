//-----------------------------------------------------------------------------
#include <stdio.h>
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <common/PGMFileFunctions.h>
#ifdef MALLOC_TRACE
#   include <mcheck.h>
#endif  // MALLOC_TRACE

#if !defined(linux) && !defined(__linux) && !defined(__linux__)
#   define PRESS_A_KEY \
    cout << "Press RETURN to end the application..." << endl; \
    cin.get();
#else
#   define PRESS_A_KEY \
    cout << "Application will exit now." << endl;
#endif // #if !defined(linux) && !defined(__linux) && !defined(__linux__)

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
int main( int /*argc*/, char* /*argv*/[] )
//-----------------------------------------------------------------------------
{
#ifdef MALLOC_TRACE
    mtrace();
#endif  // MALLOC_TRACE

    cout << endl << " ++ Start sample: " << __DATE__ << "/" << __TIME__ << endl;

    DeviceManager devMgr;
    Device* pDev = getDeviceFromUserInput( devMgr );

    if( pDev )
    {
        try
        {
            pDev->open();
        }
        catch( ImpactAcquireException& e )
        {
            // this e.g. might happen if the same device is already opened in another process...
            cout << "*** Error: An error occurred while opening the device (error: " << e.getErrorString() << "! code: " << e.getErrorCode() << ")." << endl;
            PRESS_A_KEY
            return 0;
        }
    }
    else
    {
        PRESS_A_KEY
        return 0;
    }

    // only 8Bit/pixel destination image are supported by the writeFile() function
    ImageDestination imgDst( pDev );
    imgDst.pixelFormat.write( idpfMono8 );

    CameraSettingsBase cs( pDev );
    cs.imageRequestTimeout_ms.write( 10000 ); // e.g. USB 1.1 on an embedded system needs a large timeout for the first image

    FunctionInterface fi( pDev );
    // send a request to the default request queue of the device and wait for the result.
    fi.imageRequestSingle();

    // now that the driver has a buffer to capture data into we might need to tell the device
    // to start streaming data...
    const bool boManualAcquisitionEngineControl = pDev->acquisitionStartStopBehaviour.isValid() && ( pDev->acquisitionStartStopBehaviour.read() == assbUser );
    if( boManualAcquisitionEngineControl )
    {
        cout << "Manual start/stop of acquisition engine needed." << endl;
        const int startResult = fi.acquisitionStart();
        cout << "Result of start: " << startResult << "("
             << ImpactAcquireException::getErrorCodeAsString( startResult ) << ")" << endl;
    }

    // wait for results from the default capture queue
    int iRequestNr = fi.imageRequestWaitFor( -1 );

    // if the streaming of data was started manually we have to stop it again now since we are done with capturing
    if( boManualAcquisitionEngineControl )
    {
        const int stopResult = fi.acquisitionStop();
        cout << "Manually stopping acquisition engine. Result: " << stopResult << "("
             << ImpactAcquireException::getErrorCodeAsString( stopResult ) << ")" << endl;
    }

    cout << "Request Nr.: " << iRequestNr << endl;
    // check if the image has been captured without any problems
    if( !fi.isRequestNrValid( iRequestNr ) )
    {
        // this can not happen in this sample, but may happen if you wait for a request without
        // sending one to the driver before
        cout << "*** Error: " << ImpactAcquireException::getErrorCodeAsString( iRequestNr ) << "(" << iRequestNr << ")" << endl;
        PRESS_A_KEY
        return 0;
    }

    const Request* pRequest = fi.getRequest( iRequestNr );
    if( !pRequest->isOK() )
    {
        cout << "*** Error: " << pRequest->requestResult.readS() << endl;
        PRESS_A_KEY
        return 0;
    }

    // everything went well. Store the result into a file
    const string sImageName = string( "SingCapt.pgm" );
    cout << "Will save the file as:  " << sImageName << endl;
    if( writePGMFile( ( const unsigned char* )pRequest->imageData.read(), pRequest->imageWidth.read(), pRequest->imageHeight.read(), pRequest->imageLinePitch.read(), pRequest->imagePixelPitch.read(), sImageName.c_str() ) < 0 )
    {
        cout << "*** Error: File: " << sImageName << " could not be saved!!" << endl;
    }

    // unlock the buffer to let the driver know that you no longer need this buffer
    fi.imageRequestUnlock( iRequestNr );

    PRESS_A_KEY

    // free resources
    fi.imageRequestReset( 0, 0 );

    pDev->close();

    return 0;
}
