#ifdef _MSC_VER         // is Microsoft compiler?
#   if _MSC_VER < 1300  // is 'old' VC 6 compiler?
#       pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#   endif // #if _MSC_VER < 1300
#endif // #ifdef _MSC_VER
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#if defined(linux) || defined(__linux) || defined(__linux__)
#   include <stdio.h>
#   include <unistd.h>
#else
#   include <windows.h>
#   include <process.h>
#   include <mvDisplay/Include/mvIMPACT_acquire_display.h>
using namespace mvIMPACT::acquire::display;
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

using namespace std;
using namespace mvIMPACT::acquire;

static bool s_boTerminated = false;

//-----------------------------------------------------------------------------
struct ThreadParameter
//-----------------------------------------------------------------------------
{
    Device* pDev;
#if defined(linux) || defined(__linux) || defined(__linux__)
    explicit ThreadParameter( Device* p ) : pDev( p ) {}
#else
    ImageDisplayWindow  displayWindow;
    explicit ThreadParameter( Device* p, const std::string& windowTitle ) : pDev( p ), displayWindow( windowTitle ) {}
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
};

//-----------------------------------------------------------------------------
unsigned int DMR_CALL liveThread( void* pData )
//-----------------------------------------------------------------------------
{
    ThreadParameter* pThreadParameter = reinterpret_cast<ThreadParameter*>( pData );

    // establish access to the statistic properties
    Statistics statistics( pThreadParameter->pDev );
    // create an interface to the device found
    FunctionInterface fi( pThreadParameter->pDev );

    // Send all requests to the capture queue. There can be more than 1 queue for some device, but for this sample
    // we will work with the default capture queue. If a device supports more than one capture or result
    // queue, this will be stated in the manual. If nothing is mentioned about it, the device supports one
    // queue only. This loop will send all requests currently available to the driver. To modify the number of requests
    // use the property mvIMPACT::acquire::SystemSettings::requestCount at runtime or the property
    // mvIMPACT::acquire::Device::defaultRequestCount BEFORE opening the device.
    TDMR_ERROR result = DMR_NO_ERROR;
    while( ( result = static_cast<TDMR_ERROR>( fi.imageRequestSingle() ) ) == DMR_NO_ERROR ) {};
    if( result != DEV_NO_FREE_REQUEST_AVAILABLE )
    {
        cout << "'FunctionInterface.imageRequestSingle' returned with an unexpected result: " << result
             << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
    }

    manuallyStartAcquisitionIfNeeded( pThreadParameter->pDev, fi );
    // run thread loop
    int requestNr = INVALID_ID;
    unsigned int cnt = 0;
    const unsigned int timeout_ms = 500;
    // we always have to keep at least 2 images as the display module might want to repaint the image, thus we
    // cannot free it unless we have a assigned the display to a new buffer.
    int lastRequestNr = INVALID_ID;
    while( !s_boTerminated )
    {
        // wait for results from the default capture queue
        requestNr = fi.imageRequestWaitFor( timeout_ms );
        if( fi.isRequestNrValid( requestNr ) )
        {
            const Request* pRequest = fi.getRequest( requestNr );
            if( pRequest->isOK() )
            {
                // within this scope we have a valid buffer of data that can be an image or any other
                // chunk of data.
                ++cnt;
                // here we can display some statistical information every 100th image
                if( cnt % 100 == 0 )
                {
                    cout << "Info from " << pThreadParameter->pDev->serial.read()
                         << ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
                         << ", " << statistics.errorCount.name() << ": " << statistics.errorCount.readS()
                         << ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS() << endl;
                }
#if defined(linux) || defined(__linux) || defined(__linux__)
                cout << "Image captured(" << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << ")" << endl;
#else
                pThreadParameter->displayWindow.GetImageDisplay().SetImage( pRequest );
                pThreadParameter->displayWindow.GetImageDisplay().Update();
#endif  // #if defined(linux) || defined(__linux) || defined(__linux__)
            }
            else
            {
                cout << "Error: " << pRequest->requestResult.readS() << endl;
            }
            if( fi.isRequestNrValid( lastRequestNr ) )
            {
                // this image has been displayed thus the buffer is no longer needed...
                fi.imageRequestUnlock( lastRequestNr );
            }
            lastRequestNr = requestNr;
            // send a new image request into the capture queue
            fi.imageRequestSingle();
        }
        else
        {
            // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
            // additional information under TDMR_ERROR in the interface reference
            cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")"
                 << ", timeout value too small?" << endl;
        }
#if defined(linux) || defined(__linux) || defined(__linux__)
        s_boTerminated = waitForInput( 0, STDOUT_FILENO ) == 0 ? false : true; // break by STDIN
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    }
    manuallyStopAcquisitionIfNeeded( pThreadParameter->pDev, fi );

#if !defined(linux) && !defined(__linux) && !defined(__linux__)
    // stop the display from showing freed memory
    pThreadParameter->displayWindow.GetImageDisplay().SetImage( reinterpret_cast<Request*>( 0 ) );
#endif // #if !defined(linux) && !defined(__linux) && !defined(__linux__)

    // In this sample all the next lines are redundant as the device driver will be
    // closed now, but in a real world application a thread like this might be started
    // several times an then it becomes crucial to clean up correctly.

    // free the last potentially locked request
    if( fi.isRequestNrValid( requestNr ) )
    {
        fi.imageRequestUnlock( requestNr );
    }
    // clear the request queue
    fi.imageRequestReset( 0, 0 );
    // extract and unlock all requests that are now returned as 'aborted'
    while( ( requestNr = fi.imageRequestWaitFor( 0 ) ) >= 0 )
    {
        cout << "Request " << requestNr << " did return with status " << fi.getRequest( requestNr )->requestResult.readS() << endl;
        fi.imageRequestUnlock( requestNr );
    }
    return 0;
}

//-----------------------------------------------------------------------------
// This function will allow to select devices that support the GenICam interface
// layout(these are devices, that claim to be compliant with the GenICam standard)
// and that are bound to drivers that support the user controlled start and stop
// of the internal acquisition engine. Other devices will not be listed for
// selection as the code of the example relies on these features in the code.
bool isDeviceSupportedBySample( const Device* const pDev )
//-----------------------------------------------------------------------------
{
    if( !pDev->interfaceLayout.isValid() &&
        !pDev->acquisitionStartStopBehaviour.isValid() )
    {
        return false;
    }

    vector<TDeviceInterfaceLayout> availableInterfaceLayouts;
    pDev->interfaceLayout.getTranslationDictValues( availableInterfaceLayouts );
    return find( availableInterfaceLayouts.begin(), availableInterfaceLayouts.end(), dilGenICam ) != availableInterfaceLayouts.end();
}

//-----------------------------------------------------------------------------
int main( int /*argc*/, char* /*argv*/[] )
//-----------------------------------------------------------------------------
{
    DeviceManager devMgr;
    Device* pDev = getDeviceFromUserInput( devMgr, isDeviceSupportedBySample );
    if( !pDev )
    {
        cout << "Unable to continue!";
        cout << "Press [ENTER] to end the application" << endl;
        cin.get();
        return 1;
    }

    try
    {
        cout << "Initialising the device. This might take some time..." << endl << endl;
        pDev->open();
    }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while opening device " << pDev->serial.read()
             << "(error code: " << e.getErrorCodeAsString() << ")." << endl
             << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 1;
    }

    // now display some SFNC(Standard Feature Naming Convention) compliant features(see http://www.emva.org to find out more
    // about the standard and to download the latest SFNC document version)
    //
    // IMPORTANT:
    //
    // The SFNC unfortunately does NOT define numerical values for enumerations, thus a device independent piece of software
    // should use the enum-strings defined in the SFNC to ensure interoperability between devices. This is slightly slower
    // but should not cause problems in real world applications. When the device type AND GenICam XML file version is
    // guaranteed to be constant for a certain version of software, the driver internal code generator can be used to create
    // and interface header, that has numerical constants for enumerations as well. See device driver documentation under
    // 'Use Cases -> GenICam to mvIMPACT Acquire code generator' for details.
    mvIMPACT::acquire::GenICam::DeviceControl dc( pDev );
    displayPropertyDataWithValidation( dc.deviceVendorName, "DeviceVendorName" );
    cout << endl;
    displayPropertyDataWithValidation( dc.deviceModelName, "DeviceModelName" );
    cout << endl;

    // show the current exposure time allow the user to change it
    mvIMPACT::acquire::GenICam::AcquisitionControl ac( pDev );
    displayAndModifyPropertyDataWithValidation( ac.exposureTime, "ExposureTime" );

    // show the current pixel format, width and height and allow the user to change it
    mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );
    displayAndModifyPropertyDataWithValidation( ifc.pixelFormat, "PixelFormat" );
    displayAndModifyPropertyDataWithValidation( ifc.width, "Width" );
    displayAndModifyPropertyDataWithValidation( ifc.height, "Height" );

    // start the execution of the 'live' thread.
    cout << "Press [ENTER] to end the application" << endl;

#if defined(linux) || defined(__linux) || defined(__linux__)
    ThreadParameter threadParam( pDev );
    liveThread( &threadParam );
#else
    unsigned int dwThreadID;
    string windowTitle( "mvIMPACT_acquire sample, Device " + pDev->serial.read() );
    // initialise display window
    // IMPORTANT: It's NOT save to create multiple display windows in multiple threads!!!
    ThreadParameter threadParam( pDev, windowTitle );
    HANDLE hThread = ( HANDLE )_beginthreadex( 0, 0, liveThread, ( LPVOID )( &threadParam ), 0, &dwThreadID );
    cin.get();
    s_boTerminated = true;
    WaitForSingleObject( hThread, INFINITE );
    CloseHandle( hThread );
#endif  // #if defined(linux) || defined(__linux) || defined(__linux__)
    return 0;
}
