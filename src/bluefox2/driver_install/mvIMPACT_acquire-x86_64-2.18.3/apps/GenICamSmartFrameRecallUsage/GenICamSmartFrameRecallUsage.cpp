#ifdef _MSC_VER         // is Microsoft compiler?
#   if _MSC_VER < 1300  // is 'old' VC 6 compiler?
#       pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#   endif // #if _MSC_VER < 1300
#endif // #ifdef _MSC_VER
#include <algorithm>
#include <ctime>
#include <functional>
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam_CustomCommands.h>
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

//=============================================================================
//================= Data type definitions =====================================
//=============================================================================
//-----------------------------------------------------------------------------
struct ThreadParameter
//-----------------------------------------------------------------------------
{
    Device* pDev;
    GenICam::CustomCommandGenerator ccg;
    int width; // the width of the ROI to request for transmission
    int height; // the height of the ROI to request for transmission
    int requestRate; // the rate at which full resolution frames shall be requested. A 'requestRate' of 2 will request every 2nd image from the stream again in full resolution
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    ImageDisplayWindow displayWindowStream;
    // As the display might want to repaint the image we must keep it until the next one has been assigned to the display
    Request* pLastRequestForStream;
    ImageDisplayWindow displayWindowExplicitRequestedByHost;
    // As the display might want to repaint the image we must keep it until the next one has been assigned to the display
    Request* pLastRequestExplicitlyRequested;
#endif // #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    explicit ThreadParameter( Device* p, int w, int h, int rr ) : pDev( p ), ccg( pDev ), width( w ), height( h ), requestRate( rr )
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
        // initialise display window
        // IMPORTANT: It's NOT save to create multiple display windows in multiple threads!!!
        , displayWindowStream( "mvIMPACT_acquire sample, Device " + pDev->serial.read() + "(live stream)" )
        , displayWindowExplicitRequestedByHost( "mvIMPACT_acquire sample, Device " + pDev->serial.read() + "(explicit requests by the host)" )
        , pLastRequestForStream( 0 ), pLastRequestExplicitlyRequested( 0 )
#endif // #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    {}
};

//=============================================================================
//================= function declarations =====================================
//=============================================================================
static void                     configureDevice( Device* pDev );
static bool                     isDeviceSupportedBySample( const Device* const pDev );
static unsigned int DMR_CALL    liveThread( void* pData );
static void                     reportProblemAndExit( Device* pDev, const string& prologue, const string& epilogue = "" );
static void                     unlockAndRequestNext( FunctionInterface& fi, Request* pRequest );

//=============================================================================
//================= implementation ============================================
//=============================================================================
//-----------------------------------------------------------------------------
// Check if all features needed by this application are actually available
// and set up these features in a way that the 'smart frame recall' can be used.
// If a crucial feature is missing/not available this function will terminate
// with an error message.
void configureDevice( Device* pDev )
//-----------------------------------------------------------------------------
{
    //switch to Bayer pixel format if this is a color camera
    GenICam::DeviceControl dc( pDev );
    GenICam::ImageFormatControl ifc( pDev );
    if( ( dc.mvDeviceSensorColorMode.isValid() ) && ( dc.mvDeviceSensorColorMode.readS() == "BayerMosaic" ) )
    {
        if( ifc.pixelColorFilter.isValid() )
        {
            const string parity = ifc.pixelColorFilter.readS();
            if( parity == "BayerBG" )
            {
                ifc.pixelFormat.writeS( "BayerBG8" );
            }
            else if( parity == "BayerGB" )
            {
                ifc.pixelFormat.writeS( "BayerGB8" );
            }
            else if( parity == "BayerRG" )
            {
                ifc.pixelFormat.writeS( "BayerRG8" );
            }
            else if( parity == "BayerGR" )
            {
                ifc.pixelFormat.writeS( "BayerGR8" );
            }
            else
            {
                cout << "Undefined BayerMosaicParity! Terminating..." << endl;
                exit( 42 );
            }
        }
        else
        {
            cout << "Cannot determine BayerMosaicParity! Terminating..." << endl;
            exit( 42 );
        }
    }

    // Configure the device to use max. decimation in both directions in order to achieve minimum bandwidth
    // for the preview stream. A real world application could use this frame e.g. to do some blob analysis
    // on the reduced data saving both transmission bandwidth and processing time. Whenever anything of
    // interest is detected an application could then request the same image with full resolution or even
    // just a ROI of the image in full resolution.
    if( ifc.decimationHorizontal.isValid() )
    {
        ifc.decimationHorizontal.write( ifc.decimationHorizontal.getMaxValue() );
        displayPropertyData( ifc.decimationHorizontal );
    }
    if( ifc.decimationVertical.isValid() )
    {
        ifc.decimationVertical.write( ifc.decimationVertical.getMaxValue() );
        displayPropertyData( ifc.decimationVertical );
    }

    // Enable the chunk mode and switch on the 'mvCustomIdentifier' chunk that can be used to easily distinguish frames
    // belonging to the normal stream from the ones explicitly requested by the host application.
    GenICam::ChunkDataControl cdc( pDev );
    if( !cdc.chunkModeActive.isValid() )
    {
        reportProblemAndExit( pDev, "Chunk data is NOT supported" );
    }
    cdc.chunkModeActive.write( bTrue );

    if( !supportsEnumStringValue( cdc.chunkSelector, "mvCustomIdentifier" ) )
    {
        reportProblemAndExit( pDev, "'mvCustomIdentifier' chunk is NOT supported", " Can't distinguish requested frames from streamed ones..." );
    }

    // The image data itself of course is needed in any case!
    cdc.chunkSelector.writeS( "Image" );
    cdc.chunkEnable.write( bTrue );
    cdc.chunkSelector.writeS( "mvCustomIdentifier" );
    cdc.chunkEnable.write( bTrue );

    // Enable the smart frame recall feature itself. This will configure the devices internal memory
    // to store each frame that gets transmitted to the host in full resolution. These images (or ROIs of these images)
    // can be requested by an application when needed. However once the internal memory is full
    // the oldest frame will be removed from the memory whenever a new one becomes ready.
    GenICam::AcquisitionControl ac( pDev );
    if( !ac.mvSmartFrameRecallEnable.isValid() )
    {
        reportProblemAndExit( pDev, "The 'Smart Frame Recall' feature is NOT supported" );
    }
    ac.mvSmartFrameRecallEnable.write( bTrue );
    // after switching on the smart frame recall certain properties (e.g. everything related
    // to binning and decimation) will become read-only!
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
    if( find( availableInterfaceLayouts.begin(), availableInterfaceLayouts.end(), dilGenICam ) == availableInterfaceLayouts.end() )
    {
        return false;
    }

    string manufacturer( pDev->manufacturer.read() );
    transform( manufacturer.begin(), manufacturer.end(), manufacturer.begin(), ptr_fun<int, int>( tolower ) );
    if( manufacturer.find( "matrix vision" ) == string::npos )
    {
        cout << "Device '" << pDev->serial.read() << "(" << pDev->product.read() << ", " << pDev->manufacturer.read() << ")' cannot work with this example as features presented here are specific to MATRIX VISION GmbH device" << endl;
        return false;
    }
    return true;
}

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

    srand( static_cast<unsigned int>( time( 0 ) ) );
    while( !s_boTerminated )
    {
        // wait for results from the default capture queue
        requestNr = fi.imageRequestWaitFor( timeout_ms );
        if( fi.isRequestNrValid( requestNr ) )
        {
            Request* pRequest = fi.getRequest( requestNr );
            if( pRequest->isOK() )
            {
                // within this scope we have a valid buffer of data that can be an image or any other chunk of data.
                ++cnt;
                if( cnt == 0 )
                {
                    // make sure 'cnt' is never 0 no even when the variable overflows
                    cnt = 1;
                }
                // here we can display some statistical information every 100th image
                if( cnt % 100 == 0 )
                {
                    cout << "Info from " << pThreadParameter->pDev->serial.read()
                         << ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
                         << ", " << statistics.errorCount.name() << ": " << statistics.errorCount.readS()
                         << ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS() << endl;
                }
                const int64_type chunkmvCustomIdentifier = pRequest->chunkmvCustomIdentifier.read();
                // Depending on the 'chunkmvCustomIdentifier' an application can easily distinguish between frames belonging to the
                // default 'reduced' stream or if these images have been explicitly requested by the application. A typical application
                // would
                // - perform a quick analysis of the reduced image to check if this image is needed in full resolution. If so it would request this
                //   image or a ROI of this image from the device
                // - perform the actual analysis the application has been developed for on the images that have been requested based on the previous
                //   reduced analysis
                // This can reduce the bandwidth consumed by an application dramatically!
#if defined(linux) || defined(__linux) || defined(__linux__)
                cout << "Image captured(" << ( ( chunkmvCustomIdentifier == 0 ) ? string( "default stream" ) : string( "as requested by the application" ) ) << "): " << pRequest->imageOffsetX.read() << "x" << pRequest->imageOffsetY.read()
                     << "@" << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << endl;
#else
                ImageDisplay& display = ( chunkmvCustomIdentifier == 0 ) ? pThreadParameter->displayWindowStream.GetImageDisplay() : pThreadParameter->displayWindowExplicitRequestedByHost.GetImageDisplay();
                display.SetImage( pRequest );
                display.Update();
                Request** ppRequest = ( chunkmvCustomIdentifier == 0 ) ? &pThreadParameter->pLastRequestForStream : &pThreadParameter->pLastRequestExplicitlyRequested;
                swap( *ppRequest, pRequest );
#endif  // #if defined(linux) || defined(__linux) || defined(__linux__)

                // if 'chunkmvCustomIdentifier' is NOT 0 this was an image requested by the host application! Do not use this to trigger another transmission request!
                // if 'chunkmvCustomIdentifier' is 0 trigger transmission requests with the desired request rate
                if( ( cnt % pThreadParameter->requestRate == 0 ) && ( chunkmvCustomIdentifier == 0 ) )
                {
                    // here depending on the user supplied input parameters we either roll the dice or we use user supplied values for the ROI
                    // of the image transmission request.
                    int w = pThreadParameter->width;
                    int x = 0;
                    if( w == 0 )
                    {
                        // there is no user supplied 'width' so generate a random one
                        w = rand() % pRequest->imageWidth.read();
                        if( w == 0 )
                        {
                            w = 1;
                        }
                        x = rand() % ( pRequest->imageWidth.read() - w );
                    }
                    int h = pThreadParameter->height;
                    int y = 0;
                    if( h == 0 )
                    {
                        // there is no user supplied 'height' so generate a random one
                        h = rand() % pRequest->imageHeight.read();
                        if( h == 0 )
                        {
                            h = 1;
                        }
                        y = rand() % ( pRequest->imageHeight.read() - h );
                    }
                    cout << "Requesting the full resolution transmission of the image with timestamp " << pRequest->infoTimeStamp_us.read() << " and a user defined identifier of '" << cnt
                         << "'. ROI: " << x << "x" << y << "@" << w << "x" << h << endl;
                    // 'cnt' will be used as the custom identifier. To distinguish between normal stream an requested transmissions one could also simply set this value to '1'.
                    // When dealing with high frame rates one should consider to request the retransmission of frames from a separate thread in order not to block the acquisition
                    // loop while communicating with the device. Also when multiple AOIs shall be requested from a single image these requests should be send to the device in a
                    // single command if possible by using one of the overloads of 'queueTransmissionRequest' instead of 'requestTransmission'. Please note that there are versions
                    // NOT using the pointer to the request object but the timestamp of the request. So once the timestamp has been extracted the request can be unlock even
                    // if a retransmission command is constructed afterwards. When using multiple threads be sure to lock all resources appropriately. The more complex C# example
                    // 'SmartFrameRecall' makes use of the recommendations made here thus can be used by those looking for additional information.
                    pThreadParameter->ccg.requestTransmission( pRequest, x, y, w, h, rtmFullResolution, cnt );
                }
            }
            else
            {
                cout << "Error: " << pRequest->requestResult.readS() << endl;
            }
            unlockAndRequestNext( fi, pRequest );
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
    pThreadParameter->displayWindowExplicitRequestedByHost.GetImageDisplay().SetImage( reinterpret_cast<Request*>( 0 ) );
    pThreadParameter->displayWindowStream.GetImageDisplay().SetImage( reinterpret_cast<Request*>( 0 ) );
    // free the last potentially locked requests
    if( pThreadParameter->pLastRequestExplicitlyRequested )
    {
        pThreadParameter->pLastRequestExplicitlyRequested->unlock();
    }
    if( pThreadParameter->pLastRequestForStream )
    {
        pThreadParameter->pLastRequestForStream->unlock();
    }
#endif // #if !defined(linux) && !defined(__linux) && !defined(__linux__)

    // In this sample all the next lines are redundant as the device driver will be
    // closed now, but in a real world application a thread like this might be started
    // several times an then it becomes crucial to clean up correctly.

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
// When a crucial feature needed for this example application is not available
// this function will get called. It reports an error and then terminates the
// application.
void reportProblemAndExit( Device* pDev, const string& prologue, const string& epilogue /* = "" */ )
//-----------------------------------------------------------------------------
{
    cout << prologue << " by device " << pDev->serial.read() << "(" << pDev->product.read() << ", Firmware version: " << pDev->firmwareVersion.readS() << ")." << epilogue << endl
         << "Press [ENTER] to end the application..." << endl;
    cin.get();
    exit( 42 );
}

//-----------------------------------------------------------------------------
// Unlock the current request and send a new image request down to the driver
void unlockAndRequestNext( FunctionInterface& fi, Request* pRequest )
//-----------------------------------------------------------------------------
{
    if( pRequest )
    {
        pRequest->unlock();
        // send a new image request into the capture queue
        fi.imageRequestSingle();
    }
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    int width = 0;
    int height = 0;
    int requestRate = 3;
    string serial;
    // scan command line
    if( argc > 1 )
    {
        for( int i = 1; i < argc; i++ )
        {
            string param( argv[i] ), key, value;
            string::size_type keyEnd = param.find_first_of( "=" );
            if( ( keyEnd == string::npos ) || ( keyEnd == param.length() - 1 ) )
            {
                cout << "Invalid command line parameter: '" << param << "' (ignored)." << endl;
            }
            else
            {
                key = param.substr( 0, keyEnd );
                value = param.substr( keyEnd + 1 );
                if( ( key == "serial" ) || ( key == "s" ) )
                {
                    serial = value.c_str();
                }
                else if( ( key == "requestRate" ) || ( key == "rr" ) )
                {
                    requestRate = static_cast<int>( atoi( value.c_str() ) );
                }
                else if( ( key == "width" ) || ( key == "w" ) )
                {
                    width = static_cast<int>( atoi( value.c_str() ) );
                }
                else if( ( key == "height" ) || ( key == "h" ) )
                {
                    height = static_cast<int>( atoi( value.c_str() ) );
                }
                else
                {
                    cout << "Invalid command line parameter: '" << param << "' (ignored)." << endl;
                }
            }
        }
    }
    else
    {
        cout << "No command line parameters specified. Available parameters:" << endl
             << "  'serial' or 's' to specify the serial number of the device to use" << endl
             << "  'width' or 'w' to specify the width of the ROI that will be requested for transmission" << endl
             << "  'height' or 'h' to specify the height of the ROI that will be requested for transmission" << endl
             << "  'requestRate' or 'rr' to specify the request rate(default: 3, will request every third image)" << endl
             << "When either width or height is specified the parameter and the corresponding offset will no longer change randomly!"
             << endl
             << "USAGE EXAMPLE:" << endl
             << "  GenICamSmartFrameRecallUsage width=100 rr=2" << endl << endl;
    }
    DeviceManager devMgr;
    Device* pDev = 0;
    if( !serial.empty() )
    {
        pDev = devMgr.getDeviceBySerial( serial );
        if( pDev && pDev->interfaceLayout.isValid() )
        {
            // if this device offers the 'GenICam' interface switch it on, as this will
            // allow are better control over GenICam compliant devices
            conditionalSetProperty( pDev->interfaceLayout, dilGenICam, true );
        }
        std::cout << ", interface layout: " << pDev->interfaceLayout.readS();
    }
    if( !pDev )
    {
        pDev = getDeviceFromUserInput( devMgr, isDeviceSupportedBySample );
    }
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
        // load the default settings
        GenICam::UserSetControl uc( pDev );
        uc.userSetSelector.writeS( "Default" );
        uc.userSetLoad.call();
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

    try
    {
        configureDevice( pDev );
        // start the execution of the 'live' thread.
        cout << "Press [ENTER] to end the application" << endl;

        ThreadParameter threadParam( pDev, width, height, requestRate );
#if defined(linux) || defined(__linux) || defined(__linux__)
        liveThread( &threadParam );
#else
        unsigned int dwThreadID;
        HANDLE hThread = ( HANDLE )_beginthreadex( 0, 0, liveThread, ( LPVOID )( &threadParam ), 0, &dwThreadID );
        cin.get();
        s_boTerminated = true;
        WaitForSingleObject( hThread, INFINITE );
        CloseHandle( hThread );
#endif  // #if defined(linux) || defined(__linux) || defined(__linux__)
    }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while setting up device " << pDev->serial.read()
             << "(error code: " << e.getErrorCodeAsString() << ")." << endl
             << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 1;
    }
    return 0;
}
