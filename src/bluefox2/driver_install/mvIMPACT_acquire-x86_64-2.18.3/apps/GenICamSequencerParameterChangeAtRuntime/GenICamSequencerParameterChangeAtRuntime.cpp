#ifdef _MSC_VER         // is Microsoft compiler?
#   if _MSC_VER < 1300  // is 'old' VC 6 compiler?
#       pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#   endif // #if _MSC_VER < 1300
#endif // #ifdef _MSC_VER
#include <algorithm>
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

//=============================================================================
//================= Data type definitions =====================================
//=============================================================================
//-----------------------------------------------------------------------------
struct SequencerSetParameter
//-----------------------------------------------------------------------------
{
    const int64_type setNr_;
    const int64_type sequencerSetNext_;
    const double exposureTime_us_;
    explicit SequencerSetParameter( const int64_type setNr, const int64_type sequencerSetNext, const double exposureTime_us ) :
        setNr_( setNr ), sequencerSetNext_( sequencerSetNext ), exposureTime_us_( exposureTime_us ) {}
};

//-----------------------------------------------------------------------------
struct ThreadParameter
//-----------------------------------------------------------------------------
{
    Device* pDev;
    GenICam::AcquisitionControl ac;
    GenICam::SequencerControl sc;
    GenICam::CustomCommandGenerator ccg;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    ImageDisplayWindow displayWindow;
#endif // #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    explicit ThreadParameter( Device* p ) : pDev( p ), ac( p ), sc( p ), ccg( p )
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
        // initialise display window
        // IMPORTANT: It's NOT save to create multiple display windows in multiple threads!!!
        , displayWindow( "mvIMPACT_acquire sample, Device " + pDev->serial.read() )
#endif // #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    {}
};

//=============================================================================
//================= static variables ==========================================
//=============================================================================
static bool s_boTerminated = false;
static SequencerSetParameter s_SequencerData[] =
{
    SequencerSetParameter( 0, 1,  5000. ), // Set 0: Capture 1 frame with an exposure time of  5000 us, then jump to set 1
    SequencerSetParameter( 1, 2, 10000. ), // Set 1: Capture 1 frame with an exposure time of 10000 us, then jump to set 2
    SequencerSetParameter( 2, 3, 15000. ), // Set 2: Capture 1 frame with an exposure time of 15000 us, then jump to set 3
    SequencerSetParameter( 3, 4, 20000. ), // Set 3: Capture 1 frame with an exposure time of 20000 us, then jump to set 4
    SequencerSetParameter( 4, 5, 25000. ), // Set 4: Capture 1 frame with an exposure time of 25000 us, then jump to set 5
    SequencerSetParameter( 5, 0, 30000. )  // Set 5: Capture 1 frame with an exposure time of 30000 us, then jump back to set 0
};
static const double s_SequencerExposureDataForRuntimeChange[] =
{
    50000.,
    60000.,
    70000.,
    80000.,
    90000.,
    100000.
};

//=============================================================================
//================= function declarations =====================================
//=============================================================================
static void                     configureDevice( Device* pDev );
static bool                     isDeviceSupportedBySample( const Device* const pDev );
static unsigned int DMR_CALL    liveThread( void* pData );

//=============================================================================
//================= implementation ============================================
//=============================================================================
// Calls the function bound to an mvIMPACT::acquire::Method object and displays
// an error message if the function call did fail.
void checkedMethodCall( Device* pDev, Method& method )
//-----------------------------------------------------------------------------
{
    const TDMR_ERROR result = static_cast<TDMR_ERROR>( method.call() );
    if( result != DMR_NO_ERROR )
    {
        std::cout << "An error was returned while calling function '" << method.displayName() << "' on device " << pDev->serial.read()
                  << "(" << pDev->product.read() << "): " << ImpactAcquireException::getErrorCodeAsString( result ) << endl;
    }
}

//-----------------------------------------------------------------------------
// Configures all the stuff that needs to be done only once. All the stuff related
// to setting up the actual sequencer could be called multiple times whenever an
// application gets re-configured. This is not the case here, but the code has been
// split in order to logically group what belongs together.
//
// Whenever 'conditionalSetEnumPropertyByString' or 'conditionalSetProperty' is
// not used here the stuff MUST succeed as otherwise when the device doesn't allow
// this feature the whole example does not work!
void configureDevice( Device* pDev )
//-----------------------------------------------------------------------------
{
    try
    {
        // Restore the factory default first in order to make sure nothing is incorrectly configured
        GenICam::UserSetControl usc( pDev );
        conditionalSetEnumPropertyByString( usc.userSetSelector, "Default" );
        const TDMR_ERROR result = static_cast<TDMR_ERROR>( usc.userSetLoad.call() );
        if( result != DMR_NO_ERROR )
        {
            std::cout << "An error occurred while restoring the factory default for device " << pDev->serial.read()
                      << "(error code: " << ImpactAcquireException::getErrorCodeAsString( result ) << ")." << endl;
        }

        // Auto exposure or an open shutter will not be helpful for this example thus switch it off if possible.
        GenICam::AcquisitionControl acqc( pDev );
        conditionalSetEnumPropertyByString( acqc.exposureMode, "Timed" );
        conditionalSetEnumPropertyByString( acqc.exposureAuto, "Off" );

        // Auto gain will not be helpful for this example either thus switch it off if possible.
        GenICam::AnalogControl ac( pDev );
        if( ac.gainSelector.isValid() )
        {
            // There might be more than a single 'Gain' as a 'GainSelector' is present. Iterate over all
            // 'Gain's that can be configured and switch off every 'Auto' feature detected.
            vector<string> validGainSelectorValues;
            ac.gainSelector.getTranslationDictStrings( validGainSelectorValues );
            const vector<string>::size_type cnt = validGainSelectorValues.size();
            for( vector<string>::size_type i = 0; i < cnt; i++ )
            {
                conditionalSetEnumPropertyByString( ac.gainSelector, validGainSelectorValues[i] );
                conditionalSetEnumPropertyByString( ac.gainAuto, "Off" );
            }
        }
        else
        {
            // There is just a single 'Gain' turn off the 'Auto' feature if supported.
            conditionalSetEnumPropertyByString( ac.gainAuto, "Off" );
        }

        // Chunk mode is needed in order to get back all the information needed to properly check
        // if an image has been taken using the desired parameters.
        GenICam::ChunkDataControl cdc( pDev );
        cdc.chunkModeActive.write( bTrue );

        // We want to act fast, thus if e.g. Bayer-images arrive in the system do NOT convert them on the fly as depending
        // on the device speed the host system might be too slow deal with the amount of data
        ImageProcessing ip( pDev );
        ip.colorProcessing.write( cpmRaw );
        if( ip.tapSortEnable.isValid() )
        {
            ip.tapSortEnable.write( bFalse );
        }
    }
    catch( const ImpactAcquireException& e )
    {
        // This e.g. might happen if the same device is already opened in another process...
        std::cout << "An error occurred while configuring the device " << pDev->serial.read()
                  << "(error code: " << e.getErrorCodeAsString() << ")." << endl
                  << "Press [ENTER] to end the application..." << endl;
        cin.get();
        exit( 1 );
    }
}

//-----------------------------------------------------------------------------
// Configures a single 'SequencerSet' so that 'X' frames are captured using a
// certain exposure time and afterwards another sets will be used.
void configureSequencerSet( ThreadParameter* pTP, const SequencerSetParameter& ssp )
//-----------------------------------------------------------------------------
{
    pTP->sc.sequencerSetSelector.write( ssp.setNr_ );
    pTP->ac.exposureTime.write( ssp.exposureTime_us_ );
    pTP->sc.sequencerPathSelector.write( 0LL );
    pTP->sc.sequencerTriggerSource.writeS( "ExposureEnd" );
    pTP->sc.sequencerSetNext.write( ssp.sequencerSetNext_ );
    checkedMethodCall( pTP->pDev, pTP->sc.sequencerSetSave );
}

//-----------------------------------------------------------------------------
// This function will configure the sequencer on the device to take a sequence of
// 'X' images with different exposure times. To change the sequence edit the
// 's_SequencerData' data array and recompile the application.
void configureSequencer( ThreadParameter* pTP )
//-----------------------------------------------------------------------------
{
    try
    {
        pTP->sc.sequencerMode.writeS( "Off" );
        pTP->sc.sequencerConfigurationMode.writeS( "On" );
        pTP->sc.sequencerFeatureSelector.writeS( "ExposureTime" );
        pTP->sc.sequencerFeatureEnable.write( bTrue );
        const size_t cnt = sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] );
        for( size_t i = 0; i < cnt; i++ )
        {
            configureSequencerSet( pTP, s_SequencerData[i] );
        }
        pTP->sc.sequencerSetStart.write( 0 );
        pTP->sc.sequencerConfigurationMode.writeS( "Off" );
        pTP->sc.sequencerMode.writeS( "On" );
    }
    catch( const ImpactAcquireException& e )
    {
        std::cout << "An error occurred while setting up the sequencer for device " << pTP->pDev->serial.read()
                  << "(error code: " << e.getErrorCodeAsString() << ")." << endl;
        s_boTerminated = true;
    }
}

//-----------------------------------------------------------------------------
// This function will configure the sequencer on the device at runtime without
// stopping the acquisition of the sequencer program. This approach is much faster
// than the usual way.
void configureSequencerAtRuntime( ThreadParameter* pTP, const bool boApplyOriginalData )
//-----------------------------------------------------------------------------
{
    try
    {
        if( boApplyOriginalData )
        {
            const size_t cnt = sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] );
            for( size_t i = 0; i < cnt; i++ )
            {
                cout << "Will set the exposure time of sequencer set " << i << " to " << s_SequencerData[i].exposureTime_us_ << "us." << endl;
                pTP->ccg.queueSequencerSetValueModification( static_cast<int64_type>( i ), sspExposureTime, s_SequencerData[i].exposureTime_us_ );
                pTP->ccg.queueSequencerSetValueModification( static_cast<int64_type>( i ), sspExposureTime, s_SequencerData[i].exposureTime_us_ );
            }
        }
        else
        {
            const size_t cnt = sizeof( s_SequencerExposureDataForRuntimeChange ) / sizeof( s_SequencerExposureDataForRuntimeChange[0] );
            for( size_t i = 0; i < cnt; i++ )
            {
                cout << "Will set the exposure time of sequencer set " << i << " to " << s_SequencerExposureDataForRuntimeChange[i] << "us." << endl;
                pTP->ccg.queueSequencerSetValueModification( static_cast<int64_type>( i ), sspExposureTime, s_SequencerExposureDataForRuntimeChange[i] );
            }
        }
        pTP->ccg.sendCommandBuffer();
    }
    catch( const ImpactAcquireException& e )
    {
        std::cout << "An error occurred while setting up the sequencer for device " << pTP->pDev->serial.read()
                  << "(error code: " << e.getErrorCodeAsString() << ")." << endl;
        s_boTerminated = true;
    }
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
    ThreadParameter* pTP = reinterpret_cast<ThreadParameter*>( pData );
    unsigned int cnt = 0;
    // establish access to the statistic properties
    Statistics statistics( pTP->pDev );
    // create an interface to the device found
    FunctionInterface fi( pTP->pDev );
    configureSequencer( pTP );

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

    manuallyStartAcquisitionIfNeeded( pTP->pDev, fi );
    // run thread loop
    mvIMPACT::acquire::Request* pRequest = 0;
    // we always have to keep at least 2 images as the displayWindow module might want to repaint the image, thus we
    // can free it unless we have a assigned the displayWindow to a new buffer.
    mvIMPACT::acquire::Request* pPreviousRequest = 0;
    const unsigned int timeout_ms = 500;
    bool boApplyOriginalData = false;
    while( !s_boTerminated )
    {
        // wait for results from the default capture queue
        int requestNr = fi.imageRequestWaitFor( timeout_ms );
        pRequest = fi.isRequestNrValid( requestNr ) ? fi.getRequest( requestNr ) : 0;
        if( pRequest )
        {
            if( pRequest->isOK() )
            {
                // within this scope we have a valid buffer of data that can be an image or any other chunk of data.
                ++cnt;
                // here we can display some statistical information every 100th image
                if( cnt % 100 == 0 )
                {
                    cout << "Info from " << pTP->pDev->serial.read()
                         << ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
                         << ", " << statistics.errorCount.name() << ": " << statistics.errorCount.readS()
                         << ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS() << endl;
                    configureSequencerAtRuntime( pTP, boApplyOriginalData );
                    boApplyOriginalData = !boApplyOriginalData;
                }
                cout << "Image captured" << ": " << pRequest->imageOffsetX.read() << "x" << pRequest->imageOffsetY.read()
                     << "@" << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << ". SequencerSetActive: " << pRequest->chunkSequencerSetActive.read() << ", exposure time: " << pRequest->chunkExposureTime.read() << endl;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
                pTP->displayWindow.GetImageDisplay().SetImage( pRequest );
                pTP->displayWindow.GetImageDisplay().Update();
#endif // #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
            }
            if( pPreviousRequest )
            {
                // this image has been displayed thus the buffer is no longer needed...
                pPreviousRequest->unlock();
            }
            pPreviousRequest = pRequest;
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
    manuallyStopAcquisitionIfNeeded( pTP->pDev, fi );

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    // stop the display from showing freed memory
    pTP->displayWindow.GetImageDisplay().SetImage( reinterpret_cast<Request*>( 0 ) );
#endif // #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)

    // In this sample all the next lines are redundant as the device driver will be
    // closed now, but in a real world application a thread like this might be started
    // several times an then it becomes crucial to clean up correctly.

    // free the last potentially locked request
    if( pRequest )
    {
        pRequest->unlock();
    }
    // clear the request queue
    fi.imageRequestReset( 0, 0 );
    // extract and unlock all requests that are now returned as 'aborted'
    int requestNr = INVALID_ID;
    while( ( requestNr = fi.imageRequestWaitFor( 0 ) ) >= 0 )
    {
        pRequest = fi.getRequest( requestNr );
        cout << "Request " << requestNr << " did return with status " << pRequest->requestResult.readS() << endl;
        pRequest->unlock();
    }
    return 0;
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

    try
    {
        configureDevice( pDev );
        // start the execution of the 'live' thread.
        cout << "Press [ENTER] to end the application" << endl;

        ThreadParameter threadParam( pDev );
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
