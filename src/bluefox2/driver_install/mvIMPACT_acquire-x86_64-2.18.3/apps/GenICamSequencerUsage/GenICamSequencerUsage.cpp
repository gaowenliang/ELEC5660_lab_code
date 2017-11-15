#ifdef _MSC_VER         // is Microsoft compiler?
#   if _MSC_VER < 1300  // is 'old' VC 6 compiler?
#       pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#   endif // #if _MSC_VER < 1300
#endif // #ifdef _MSC_VER
#include <apps/Common/exampleHelper.h>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>
#if defined(linux) || defined(__linux) || defined(__linux__)
#   include <stdio.h>
#   include <sys/time.h>
#   include <unistd.h>
#else
#   include <windows.h>
#   include <process.h>
#   include <mvDisplay/Include/mvIMPACT_acquire_display.h>
using namespace mvIMPACT::acquire::display;
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

#undef min // Otherwise we can't work with the 'numeric_limits' template here as Windows defines a macro 'min'
#undef max // Otherwise we can't work with the 'numeric_limits' template here as Windows defines a macro 'max'

using namespace std;
using namespace mvIMPACT::acquire;

#define USE_EXTENDED_SEQUENCER

//=============================================================================
//================= Data type definitions =====================================
//=============================================================================
#if defined(linux) || defined(__linux) || defined(__linux__)
//-----------------------------------------------------------------------------
/// \brief Provides platform independent clock and time measurement functions
class CTime
//-----------------------------------------------------------------------------
{
    struct timespec m_tsStart;
    struct timespec m_tsEnd;
    clockid_t m_ClockId;
    // Returns a time difference in milli-seconds
    long diffTime( void )
    {
        clock_gettime( m_ClockId, &m_tsEnd );
        static struct timespec tsDiff;
        tsDiff.tv_sec  = m_tsEnd.tv_sec  - m_tsStart.tv_sec ;
        tsDiff.tv_nsec = m_tsEnd.tv_nsec - m_tsStart.tv_nsec ;

        if( tsDiff.tv_nsec < 0 ) // handle 1 sec borrow
        {
            tsDiff.tv_nsec += 1000000000 ; // usec / sec
            tsDiff.tv_sec -= 1;
        }
        return tsDiff.tv_sec * 1000 + tsDiff.tv_nsec / 1000000;
    }
public:
    explicit CTime() : m_tsStart(), m_tsEnd(), m_ClockId( CLOCK_REALTIME )
    {
#ifdef USE_MONOTONIC_CLOCK
        struct timespec ts;
        m_ClockId = ( clock_gettime( CLOCK_MONOTONIC, &ts ) == 0 ) ? CLOCK_MONOTONIC : CLOCK_REALTIME;
#endif // #ifdef USE_MONOTONIC_CLOCK
        start();
    }
    void start( void )
    {
        clock_gettime( m_ClockId, &m_tsStart );
    }
    double elapsed( void )
    {
        return static_cast<double>( diffTime() / 1000. ); // in sec
    }
    double restart( void )
    {
        double result = elapsed();
        m_tsStart = m_tsEnd;
        return result;
    }
};
#else
//-----------------------------------------------------------------------------
class CTime
//-----------------------------------------------------------------------------
{
    LARGE_INTEGER frequency_;
    LARGE_INTEGER start_;
    LARGE_INTEGER end_;
public:
    explicit CTime()
    {
        QueryPerformanceFrequency( &frequency_ );
        QueryPerformanceCounter( &start_ );
    }
    void start( void )
    {
        QueryPerformanceCounter( &( start_ ) );
    }
    double elapsed( void )
    {
        QueryPerformanceCounter( &end_ );
        return static_cast<double>( end_.QuadPart - start_.QuadPart ) / frequency_.QuadPart;
    }
    double restart( void )
    {
        QueryPerformanceCounter( &end_ );
        double result = static_cast<double>( end_.QuadPart - start_.QuadPart ) / frequency_.QuadPart;
        start_ = end_;
        return result;
    }
};
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

//-----------------------------------------------------------------------------
struct SequencerSetParameter
//-----------------------------------------------------------------------------
{
    const int64_type setNr_;
    const int64_type sequencerSetNext_;
    const double exposureTime_us_;
    const int64_type frameCount_;
    const int64_type horizontalBinning_;
    const int64_type verticalBinning_;
    double expectedFrameRate_;
    explicit SequencerSetParameter( const int64_type setNr, const int64_type sequencerSetNext, const double exposureTime_us, const int64_type frameCount, const int64_type horizontalBinning, const int64_type verticalBinning ) :
        setNr_( setNr ), sequencerSetNext_( sequencerSetNext ), exposureTime_us_( exposureTime_us ), frameCount_( frameCount ), horizontalBinning_( horizontalBinning ), verticalBinning_( verticalBinning ), expectedFrameRate_( 0.0 ) {}
};

//-----------------------------------------------------------------------------
struct ThreadParameter
//-----------------------------------------------------------------------------
{
    Device* pDev;
    FunctionInterface fi;
    Statistics statistics;
    GenICam::AcquisitionControl ac;
    GenICam::ImageFormatControl ifc;
    GenICam::ChunkDataControl cdc;
    GenICam::CounterAndTimerControl ctc;
    GenICam::SequencerControl sc;
    int64_type framesCaptured;
#if defined(linux) || defined(__linux) || defined(__linux__)
    explicit ThreadParameter( Device* p ) : pDev( p ), fi( pDev ), statistics( pDev ), ac( pDev ), ifc( pDev ), cdc( pDev ), ctc( pDev ), sc( pDev ), framesCaptured( 0 ) {}
#else
    ImageDisplayWindow  displayWindow;
    explicit ThreadParameter( Device* p, const std::string& windowTitle ) : pDev( p ), fi( pDev ), statistics( pDev ), ac( pDev ), ifc ( pDev ), cdc( pDev ), ctc( pDev ), sc( pDev ), framesCaptured( 0 ), displayWindow( windowTitle ) {}
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
};

//=============================================================================
//================= static variables ==========================================
//=============================================================================
static bool s_boTerminated = false;
static SequencerSetParameter s_SequencerData[] =
{
#ifdef USE_EXTENDED_SEQUENCER
    SequencerSetParameter( 0, 1,  1000.,  5, 2, 2 ), // Set 0: Capture 5 frames Exposure = 1000 us HBinning = 2 VBinning = 2, then jump to set 1
    SequencerSetParameter( 1, 2,  2000., 16, 2, 2 ), // Set 1: Capture 16 frames Exposure = 2000 us HBinning = 2 VBinning = 2, then jump to set 2
    SequencerSetParameter( 2, 3,  2000.,  8, 1, 1 ), // Set 2: Capture 8 frames Exposure = 2000 us HBinning = 1 VBinning = 1, then jump to set 3
    SequencerSetParameter( 3, 4, 10000., 16, 1, 1 ), // Set 3: Capture 16 frames Exposure = 10000 us HBinning = 1 VBinning = 1, then jump to set 4
    SequencerSetParameter( 4, 5,  5000.,  5, 1, 1 ), // Set 4: Capture 5 frames Exposure = 5000 us HBinning = 1 VBinning = 1, then jump to set 5
    SequencerSetParameter( 5, 6,  2000.,  5, 2, 2 ), // Set 5: Capture 5 frames Exposure = 2000 us HBinning = 2 VBinning = 2, then jump to set 6
    SequencerSetParameter( 6, 7,  1000., 16, 2, 2 ), // Set 6: Capture 16 frames Exposure = 1000 us HBinning = 2 VBinning = 2, then jump to set 7
    SequencerSetParameter( 7, 8,  5000.,  8, 1, 1 ), // Set 7: Capture 8 frames Exposure = 5000 us HBinning = 1 VBinning = 1, then jump to set 8
    SequencerSetParameter( 8, 9, 10000., 16, 1, 1 ), // Set 8: Capture 16 frames Exposure = 10000 us HBinning = 1 VBinning = 1, then jump to set 9
    SequencerSetParameter( 9, 0, 15000.,  5, 1, 1 )  // Set 9: Capture 5 frames Exposure = 15000 us HBinning = 1 VBinning = 1, then jump back to set 0*/
#else
    SequencerSetParameter( 0, 1,  1000.,  5, 1, 1 ), // Set 0: Capture  5 frames with an exposure time of 1000  us, then jump to set 1
    SequencerSetParameter( 1, 2, 15000., 40, 1, 1 ), // Set 1: Capture 40 frames with an exposure time of 15000 us, then jump to set 2
    SequencerSetParameter( 2, 3,  2000., 20, 1, 1 ), // Set 2: Capture 20 frames with an exposure time of 2000  us, then jump to set 3
    SequencerSetParameter( 3, 4, 10000., 40, 1, 1 ), // Set 3: Capture 40 frames with an exposure time of 10000 us, then jump to set 4
    SequencerSetParameter( 4, 0,  5000.,  5, 1, 1 )  // Set 4: Capture  5 frames with an exposure time of 5000  us, then jump back to set 0
#endif
};

//=============================================================================
//================= function declarations =====================================
//=============================================================================
static void                     checkedMethodCall( Device* pDev, Method& method );
static void                     configureDevice( Device* pDev );
static void                     configureSequencerSet( ThreadParameter* pTP, const SequencerSetParameter& ssp );
static void                     configureSequencer( ThreadParameter* pTP );
static size_t                   getExpectedSequencerSet( int64_type frameNr );
static double                   getMinimalExposureTime( void );
static int64_type               getOverallSequenceLength( void );
static double                   getPureAcquisitionTimeOfCapturedFrames( const int64_type framesCaptured );
static bool                     isDeviceSupportedBySample( const Device* const pDev );
static unsigned int DMR_CALL    liveThread( void* pData );
static void                     queueCaptureBuffers( ThreadParameter* pTP );
static void                     storeRawFrame( Request* pRequest );

//=============================================================================
//================= implementation ============================================
//=============================================================================
//-----------------------------------------------------------------------------
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

        // The sequencer program will jump from one set to the next after 'CounterDuration'
        // frames have been captured with the current set, thus we need to configure the counter
        // to count 'frames' and to reset itself once 'CounterDuration' has been reached.
        GenICam::CounterAndTimerControl ctc( pDev );
        ctc.counterSelector.writeS( "Counter1" );
        ctc.counterEventSource.writeS( "ExposureEnd" );
        ctc.counterTriggerSource.writeS( "Counter1End" );

        // In order to have at least some kind of external trigger we use a timer running with the
        // highest frequency defined by all sequencer set, thus the reciprocal value of the
        // smallest exposure time defined in the set array.
        ctc.timerSelector.writeS( "Timer1" );
        ctc.timerDuration.write( getMinimalExposureTime() );
        ctc.timerTriggerSource.writeS( "Timer1End" );
        acqc.triggerSelector.writeS( "FrameStart" );
        acqc.triggerMode.writeS( "On" );
        acqc.triggerSource.writeS( "Timer1End" );

        // This is needed to correctly calculate the expected capture time
        conditionalSetEnumPropertyByString( acqc.mvAcquisitionFrameRateLimitMode, "mvDeviceLinkThroughput" );
        conditionalSetEnumPropertyByString( acqc.mvAcquisitionFrameRateEnable, "Off" );

        // As we want to keep ALL images belonging to the full sequence in RAM we need as many requests as
        // there are frames defined by the sequence.
        SystemSettings ss( pDev );
        ss.requestCount.write( static_cast<int>( getOverallSequenceLength() ) );

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
    pTP->ifc.binningHorizontal.write( ssp.horizontalBinning_ );
    pTP->ifc.binningVertical.write( ssp.verticalBinning_ );
    pTP->ifc.height.write( pTP->ifc.heightMax.read() );
    pTP->ctc.counterDuration.write( ssp.frameCount_ );
    pTP->sc.sequencerPathSelector.write( 0LL );
    pTP->sc.sequencerTriggerSource.writeS( "Counter1End" );
    pTP->sc.sequencerSetNext.write( ssp.sequencerSetNext_ );
    checkedMethodCall( pTP->pDev, pTP->sc.sequencerSetSave );
}

//-----------------------------------------------------------------------------
// This function will configure the sequencer on the device to take a sequence of
// 'X' images where the sequence is split into parts of different length and each
// part of the sequence can use a different exposure time. Thus e.g.
// -  5 frames with 1000us
// - 40 frames with 15000us
// - 20 frames with 2000us
// - 10 frames with 10000us
// -  5 frames with 5000us
// can be captured. To change the sequence edit the 's_SequencerData' data array
// and recompile the application.
void configureSequencer( ThreadParameter* pTP )
//-----------------------------------------------------------------------------
{
    try
    {
        pTP->sc.sequencerMode.writeS( "Off" );
        pTP->sc.sequencerConfigurationMode.writeS( "On" );
        pTP->sc.sequencerFeatureSelector.writeS( "ExposureTime" );
        pTP->sc.sequencerFeatureEnable.write( bTrue );
        pTP->sc.sequencerFeatureSelector.writeS( "CounterDuration" );
        pTP->sc.sequencerFeatureEnable.write( bTrue );
        const size_t cnt = sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] );
        for( size_t i = 0; i < cnt; i++ )
        {
            configureSequencerSet( pTP, s_SequencerData[i] );
            s_SequencerData[i].expectedFrameRate_ = pTP->ac.mvResultingFrameRate.read();
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
// Returns the expected sequencer set for frame 'frameNr' based on the data
// defined by the entries of 's_SequencerData'.
size_t getExpectedSequencerSet( int64_type frameNr )
//-----------------------------------------------------------------------------
{
    const size_t cnt = sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] );
    int64_type framesUpToHere = 0LL;
    for( size_t i = 0; i < cnt; i++ )
    {
        framesUpToHere += s_SequencerData[i].frameCount_;
        if( frameNr < framesUpToHere )
        {
            return i;
        }
    }
    return 0xFFFFFFFF;
}

//-----------------------------------------------------------------------------
// Returns the minimal exposure time defined by the entries of 's_SequencerData'.
double getMinimalExposureTime( void )
//-----------------------------------------------------------------------------
{
    const size_t cnt = sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] );
    double minExposureTime_us = numeric_limits<double>::max();
    for( size_t i = 0; i < cnt; i++ )
    {
        if( minExposureTime_us > s_SequencerData[i].exposureTime_us_ )
        {
            minExposureTime_us = s_SequencerData[i].exposureTime_us_;
        }
    }
    // make sure we found at least one entry, that makes sense!
    assert( minExposureTime_us != numeric_limits<double>::max() );
    return minExposureTime_us;
}

//-----------------------------------------------------------------------------
// Calculates the overall number of frames that will form a complete sequence as
// defined by the entries of 's_SequencerData'.
int64_type getOverallSequenceLength( void )
//-----------------------------------------------------------------------------
{
    const size_t cnt = sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] );
    int64_type overallFrameCount = 0LL;
    for( size_t i = 0; i < cnt; i++ )
    {
        overallFrameCount += s_SequencerData[i].frameCount_;
    }
    return overallFrameCount;
}

//-----------------------------------------------------------------------------
// Returns the pure acquisition time in seconds. This is the sum of the defined exposure
// times or sensor read-out times (depending on which value is larger) of all frames
// that belong to the full sequence. This might be useful to
// e.g. calculate the difference between the overall capture time and the ideal
// capture time (when the sensor read-out is always faster than the exposure etc.).
double getPureAcquisitionTimeOfCapturedFrames( const int64_type framesCaptured )
//-----------------------------------------------------------------------------
{
    const size_t cnt = sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] );
    int64_type framesProcessed = 0;
    double pureAcquisitionTime_us = 0.0;
    for( size_t i = 0; i < cnt; i++ )
    {
        const int64_type framesToConsider = ( ( framesProcessed + s_SequencerData[i].frameCount_ ) > framesCaptured ) ? framesCaptured - framesProcessed : s_SequencerData[i].frameCount_;
        if( ( s_SequencerData[i].expectedFrameRate_ > 0.0 ) &&
            ( ( 1.0 / s_SequencerData[i].expectedFrameRate_ * 1000000. ) > s_SequencerData[i].exposureTime_us_ ) )
        {
            pureAcquisitionTime_us += ( 1.0 / s_SequencerData[i].expectedFrameRate_ * 1000000. ) * framesToConsider;
        }
        else
        {
            pureAcquisitionTime_us += s_SequencerData[i].exposureTime_us_ * framesToConsider;
        }
        framesProcessed += framesToConsider;
        if( framesProcessed >= framesCaptured )
        {
            break;
        }
    }
    return pureAcquisitionTime_us / 1000000.;
}

//-----------------------------------------------------------------------------
// Sends all requests to the capture queue. There can be more than 1 queue for some device, but for this sample
// we will work with the default capture queue. If a device supports more than one capture or result
// queue, this will be stated in the manual. If nothing is mentioned about it, the device supports one
// queue only. This loop will send all requests currently available to the driver. To modify the number of requests
// use the property mvIMPACT::acquire::SystemSettings::requestCount at runtime or the property
// mvIMPACT::acquire::Device::defaultRequestCount BEFORE opening the device.
void queueCaptureBuffers( ThreadParameter* pTP )
//-----------------------------------------------------------------------------
{
    TDMR_ERROR result = DMR_NO_ERROR;
    for( int i = 0; i < static_cast<int>( getOverallSequenceLength() ); i++ )
    {
        if( ( result = static_cast<TDMR_ERROR>( pTP->fi.imageRequestSingle() ) ) != DMR_NO_ERROR )
        {
            std::cout << "'FunctionInterface.imageRequestSingle' returned with an unexpected result: " << result
                      << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
            std::cout << "Failed to queue the desired number of buffer. Stopped at " << i << "/" << pTP->fi.requestCount() << "." << endl;
            s_boTerminated = true;
        }
    }
}

//-----------------------------------------------------------------------------
// Stores a frame in RAW format using a file name that contains all information
// needed to reconstruct the image later. This uses the same format that is understood
// by wxPropView, thus such a file can be displayed in wxPropView by simply
// dragging the file into the display area of the tool or by loading the image
// via the corresponding menu items.
void storeRawFrame( Request* pRequest )
//-----------------------------------------------------------------------------
{
    const void* pData = pRequest->imageData.read();
    if( pData )
    {
        ostringstream fileName;
        fileName << "image" << setw( 6 ) << setfill( '0' ) << pRequest->infoFrameID.read() << "_"
                 << "Set=" << pRequest->chunkSequencerSetActive.read() << "_"
                 << "Exposure=" << static_cast<unsigned int>( pRequest->chunkExposureTime.read() ) << "." // the 'cast' is just to get rid of the '.' in the 'double' value as this otherwise breaks the 'RAW' file import of wxPropView...
                 << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read()
                 << "." << pRequest->imagePixelFormat.readS();
        if( pRequest->imageBayerMosaicParity.read() != bmpUndefined )
        {
            fileName << "(BayerPattern=" << pRequest->imageBayerMosaicParity.readS() << ")";
        }
        fileName << ".raw";
        FILE* pFile = fopen( fileName.str().c_str(), "wb" );
        if( pFile )
        {
            if( fwrite( pData, pRequest->imageSize.read(), 1, pFile ) != 1 )
            {
                std::cout << "Failed to write file '" << fileName.str() << "'." << endl;
            }
            else
            {
                std::cout << "Successfully written file '" << fileName.str() << "'." << endl;
            }
            fclose( pFile );
        }
    }
}

//-----------------------------------------------------------------------------
unsigned int DMR_CALL liveThread( void* pData )
//-----------------------------------------------------------------------------
{
    ThreadParameter* pTP = reinterpret_cast<ThreadParameter*>( pData );

    BasicDeviceSettings bds( pTP->pDev );
    const unsigned int timeout_ms = bds.imageRequestTimeout_ms.read() + 500;
    const int framesToCapture = pTP->fi.requestCount();
    int requestNr = INVALID_ID;
    bool isFirstValidImage = true;

    // store width and height for checking correct image size
    const int64_type orgWidth = pTP->ifc.width.read();
    const int64_type orgHeight = pTP->ifc.height.read();
    std::cout << "OrgWidth = " << orgWidth << " OrgHeight = " << orgHeight << endl;
    CTime timer;
    configureSequencer( pTP );
    std::cout << "Setting up the sequencer took " << timer.restart() << " seconds." << endl;
    queueCaptureBuffers( pTP );
    std::cout << "Queuing capture buffers took " << timer.restart() << " seconds." << endl;
    manuallyStartAcquisitionIfNeeded( pTP->pDev, pTP->fi );
    std::cout << "Starting the acquisition took " << timer.restart() << " seconds." << endl;

    vector<string> information;

    // Run thread loop
    while( !s_boTerminated && ( pTP->framesCaptured < framesToCapture ) )
    {
        ostringstream oss;
        // Wait for results from the default capture queue
        requestNr = pTP->fi.imageRequestWaitFor( timeout_ms );
        if( pTP->fi.isRequestNrValid( requestNr ) )
        {
            const Request* pRequest = pTP->fi.getRequest( requestNr );
            if( pRequest->isOK() )
            {
                // Within this scope we have a valid buffer of data that can be an image or any other chunk of data.
                if( isFirstValidImage == true )
                {
                    oss << "The first frame arrived after " << timer.elapsed() << " seconds using the following format: "
                        << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << ", " << pRequest->imagePixelFormat.readS()
                        << endl;
                    isFirstValidImage = false;
                }
                oss << "Image captured: "
                    << "TimeStamp: " << setw( 16 ) << pRequest->infoTimeStamp_us.read() << ", "
                    << "ChunkExposureTime: " << setw( 10 ) << static_cast<int>( pRequest->chunkExposureTime.read() ) << ", "
                    << "ChunkSequencerSetActive: " << pRequest->chunkSequencerSetActive.read() << ", "
                    << "ChunkWidth: " << pRequest->chunkWidth.read() << ", "
                    << "ChunkHeight: " << pRequest->chunkHeight.read() << ", "
                    << "FrameID: " << setw( 16 ) << pRequest->infoFrameID.read() << ".";
                const size_t expectedSet = getExpectedSequencerSet( pTP->framesCaptured );
                if( expectedSet < ( sizeof( s_SequencerData ) / sizeof( s_SequencerData[0] ) ) )
                {
                    if( expectedSet != static_cast<size_t>( pRequest->chunkSequencerSetActive.read() ) )
                    {
                        oss << " ERROR! Expected set " << expectedSet << ", reported set " << pRequest->chunkSequencerSetActive.read();
                    }
                    // check exposure time
                    const double reportedExposureTime = pRequest->chunkExposureTime.read();
                    if( ( s_SequencerData[expectedSet].exposureTime_us_ * 0.95 > reportedExposureTime ) ||
                        ( s_SequencerData[expectedSet].exposureTime_us_ * 1.05 < reportedExposureTime ) )
                    {
                        oss << " ERROR! Expected exposure time " << s_SequencerData[expectedSet].exposureTime_us_ << ", reported exposure time " << static_cast<int>( reportedExposureTime );
                    }
                    // check image width
                    const int64_type reportedWidth = pRequest->chunkWidth.read();
                    if ( ( s_SequencerData[expectedSet].horizontalBinning_ * reportedWidth != orgWidth ) )
                    {
                        oss << " ERROR! Expected width " << orgWidth / s_SequencerData[expectedSet].horizontalBinning_ << ", reported width " << static_cast<int>( reportedWidth );
                    }
                    // check image height
                    const int64_type reportedHeight = pRequest->chunkHeight.read();
                    if ( ( s_SequencerData[expectedSet].verticalBinning_ * reportedHeight != orgHeight ) )
                    {
                        oss << " ERROR! Expected height " << orgHeight / s_SequencerData[expectedSet].verticalBinning_ << ", reported height " << static_cast<int>( reportedHeight );
                    }
                }
                else
                {
                    oss << "Internal error! Failed to locate matching sequencer set!";
                }
                oss << endl;
#if !defined(linux) && !defined(__linux) && !defined(__linux__)
                pTP->displayWindow.GetImageDisplay().SetImage( pRequest );
                pTP->displayWindow.GetImageDisplay().Update();
#endif  // #if !defined(linux) && !defined(__linux) && !defined(__linux__)
            }
            else
            {
                oss << "Error: " << pRequest->requestResult.readS() << endl;
            }
            ++pTP->framesCaptured; // in case of an error this is not correct, but if we don't count here the full sequence check will not work!
            // Do not unlock any request as we want to store the data later on!
            // Also do not request new requests here as the full sequence has been allocated and queued already in 'setupAndQueueCaptureBuffers'!
        }
        else
        {
            // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
            // additional information under TDMR_ERROR in the interface reference
            oss << "'imageRequestWaitFor' failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")." << endl
                << "In this sample this indicates a severe problem as this timeout passed to 'imageRequestWaitFor' was larger than the buffer timeout (BasicDeviceSettings::imageRequestTimeout_ms)" << endl
                << " thus something is really broken!" << endl;
            s_boTerminated = true;
        }
        information.push_back( oss.str() );
#if defined(linux) || defined(__linux) || defined(__linux__)
        s_boTerminated = waitForInput( 0, STDOUT_FILENO ) == 0 ? false : true; // break by STDIN
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    }

    const double captureTime = timer.elapsed();
    // Writing to stdout is very slow, thus we buffer the information first and output it after measuring the capture time
    const vector<string>::size_type informationCount = information.size();
    for( vector<string>::size_type i = 0; i < informationCount; i++ )
    {
        std::cout << information[i];
    }
    std::cout << "Capturing the sequence took " << captureTime << " seconds while the pure acquisition time of all frames would have been " << getPureAcquisitionTimeOfCapturedFrames( pTP->framesCaptured ) << " seconds." << endl;
    timer.restart();
    manuallyStopAcquisitionIfNeeded( pTP->pDev, pTP->fi );
    std::cout << "Stopping the acquisition took " << timer.restart() << " seconds." << endl;

#if !defined(linux) && !defined(__linux) && !defined(__linux__)
    // Stop the display from showing data we are about to process
    pTP->displayWindow.GetImageDisplay().SetImage( reinterpret_cast<Request*>( 0 ) );
    std::cout << "The capture process has finished. Press [ENTER] to end the thread!" << endl;
#endif // #if !defined(linux) && !defined(__linux) && !defined(__linux__)
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
        std::cout << "Unable to continue!";
        std::cout << "Press [ENTER] to end the application" << endl;
        cin.get();
        return 1;
    }

    try
    {
        std::cout << "Initialising the device. This might take some time..." << endl << endl;
        pDev->open();
    }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        std::cout << "An error occurred while opening the device " << pDev->serial.read()
                  << "(error code: " << e.getErrorCodeAsString() << ")." << endl
                  << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 1;
    }

    // Now configure SFNC(Standard Feature Naming Convention) compliant features(see http://www.emva.org to find out more
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
    CTime timer;
    configureDevice( pDev );
    std::cout << "Setting up the device took " << timer.restart() << " seconds." << endl;

    // start the execution of the 'live' thread.
#if defined(linux) || defined(__linux) || defined(__linux__)
    ThreadParameter tp( pDev );
    liveThread( &tp );
#else
    unsigned int dwThreadID;
    string windowTitle( "mvIMPACT_acquire sample, Device " + pDev->serial.read() );
    // Initialise display window
    // IMPORTANT: It's NOT save to create multiple display windows in multiple threads!!!
    ThreadParameter tp( pDev, windowTitle );
    HANDLE hThread = ( HANDLE )_beginthreadex( 0, 0, liveThread, ( LPVOID )( &tp ), 0, &dwThreadID );
    cin.get();
    s_boTerminated = true;
    WaitForSingleObject( hThread, INFINITE );
    CloseHandle( hThread );
#endif  // #if defined(linux) || defined(__linux) || defined(__linux__)

    std::cout << "If the " << tp.framesCaptured << " frames shall be stored to disc press 'y' [ENTER] now: ";
    string store;
    cin >> store;
    if( store == "y" )
    {
        std::cout << "Storing...." << endl;
        for( unsigned int i = 0; i < tp.framesCaptured; i++ )
        {
            storeRawFrame( tp.fi.getRequest( i ) );
        }
        std::cout << "\nAll files have been stored in RAW format. They can e.g. be watched by dragging them onto the display area of wxPropView!" << endl;
    }

    // In this sample all the next lines are redundant as the device driver will be
    // closed now, but in a real world application a thread like this might be started
    // several times an then it becomes crucial to clean up correctly.
    for( unsigned int i = 0; i < tp.framesCaptured; i++ )
    {
        const TDMR_ERROR result = static_cast<TDMR_ERROR>( tp.fi.imageRequestUnlock( i ) );
        if( result != DMR_NO_ERROR )
        {
            std::cout << "Failed to unlock request number " << i << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
        }
    }

    // Clear the request queue. In this example this should no do anything as we captured precisely the number of images we requested, thus
    // whenever another request is returned here, this is a severe malfunction!
    tp.fi.imageRequestReset( 0, 0 );
    // Extract and unlock all requests that are now returned as 'aborted'
    int requestNr = INVALID_ID;
    while( ( requestNr = tp.fi.imageRequestWaitFor( 0 ) ) >= 0 )
    {
        std::cout << "ERROR!!! Request " << requestNr << " did return with status " << tp.fi.getRequest( requestNr )->requestResult.readS()
                  << " even though it was not supposed to reside in the driver any more!" << endl;
        tp.fi.imageRequestUnlock( requestNr );
    }

    return 0;
}
