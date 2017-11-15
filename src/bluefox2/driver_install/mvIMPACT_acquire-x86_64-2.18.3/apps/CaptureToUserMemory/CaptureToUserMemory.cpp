#ifdef _MSC_VER // is Microsoft compiler?
#   if _MSC_VER < 1300  // is 'old' VC 6 compiler?
#       pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#   endif // #if _MSC_VER < 1300
#endif // #ifdef _MSC_VER
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <common/minmax.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#ifdef _WIN32
#   include <windows.h>
#   include <process.h>
#   include <mvDisplay/Include/mvIMPACT_acquire_display.h>
using namespace mvIMPACT::acquire::display;
#   define USE_DISPLAY
#elif defined(linux) || defined(__linux) || defined(__linux__)
#   if defined(__x86_64__) || defined(__aarch64__) || defined(__powerpc64__) // -m64 makes GCC define __powerpc64__
typedef uint64_t UINT_PTR;
#   elif defined(__i386__) || defined(__arm__) || defined(__powerpc__) // and -m32 __powerpc__
typedef uint32_t UINT_PTR;
#   endif
#endif // #ifdef _WIN32

#ifdef BUILD_WITH_OPENCV_SUPPORT
// In order to build this application with OpenCV support there are 2 options:
// a) Use Visual Studio 2013 or later with OpenCV 2.4.13. With this version this example project already contains
//    project configurations 'Debug OpenCV' and 'Release OpenCV'. The only thing left to do in that case is to define
//    an environment variable 'OPENCV_DIR' that points to the top-level folder of the OpenCV 2.4.13 package. Afterwards
//    when restarting Visual Studio the application can be build with a simple OpenCV example.
//    - Different OpenCV versions can be used as well, but you need to state the OpenCV lib names matching your
//      version in the linker section then as the libraries contain the version in their name
// b) Older Visual Studio versions or Linux/gcc approaches must make sure that the include and lib paths to
//    OpenCV are set up correctly. However information on how to efficiently create OpenCV buffers can be extracted
//    from the code without compiling this application as well. Simply look for every occurrence of 'BUILD_WITH_OPENCV_SUPPORT'
//    in this source file.
#   include <opencv2/core/core.hpp>
#   include <opencv2/highgui/highgui.hpp>
#   include <opencv2/imgproc/imgproc.hpp>
#endif // #ifdef BUILD_WITH_OPENCV_SUPPORT

using namespace std;
using namespace mvIMPACT::acquire;

static bool s_boTerminated = false;

//-----------------------------------------------------------------------------
// the buffer we pass to the device driver must be aligned according to its requirements
// As we can't allocate aligned heap memory we will align it 'by hand'
class UserSuppliedHeapBuffer
//-----------------------------------------------------------------------------
{
    char* pBuf_;
    int bufSize_;
    int alignment_;
    int pitch_;
public:
    explicit UserSuppliedHeapBuffer( int bufSize, int alignment, int pitch ) : pBuf_( 0 ), bufSize_( bufSize ), alignment_( alignment ), pitch_( pitch )
    {
        if( bufSize_ > 0 )
        {
            pBuf_ = new char[bufSize_ + alignment_];
        }
    }
    ~UserSuppliedHeapBuffer()
    {
        delete [] pBuf_;
    }
    char* getPtr( void )
    {
        if( alignment_ <= 1 )
        {
            return pBuf_;
        }
        return reinterpret_cast<char*>( align( reinterpret_cast<UINT_PTR>( pBuf_ ), static_cast<UINT_PTR>( alignment_ ) ) );
    }
    int getSize( void ) const
    {
        return bufSize_;
    }
};

typedef std::vector<UserSuppliedHeapBuffer*> CaptureBufferContainer;

//-----------------------------------------------------------------------------
struct CaptureParameter
//-----------------------------------------------------------------------------
{
    Device*                 pDev;
#ifdef USE_DISPLAY
    ImageDisplayWindow*     pDisplayWindow;
#endif // #ifdef USE_DISPLAY
#ifdef BUILD_WITH_OPENCV_SUPPORT
    std::string             openCVDisplayTitle;
    std::string             openCVResultDisplayTitle;
#endif // #ifdef BUILD_WITH_OPENCV_SUPPORT
    FunctionInterface       fi;
    ImageRequestControl     irc;
    Statistics              statistics;
    bool                    boUserSuppliedMemoryUsed;
    bool                    boAlwaysUseNewUserSuppliedBuffers;
    int                     bufferSize;
    int                     bufferAlignment;
    int                     bufferPitch;
    CaptureBufferContainer  buffers;
    CaptureParameter( Device* p ) : pDev( p ), fi( p ), irc( p ), statistics( p ), boUserSuppliedMemoryUsed( false ),
        boAlwaysUseNewUserSuppliedBuffers( false ), bufferSize( 0 ), bufferAlignment( 0 ), bufferPitch( 0 ), buffers()
    {
#ifdef USE_DISPLAY
        // IMPORTANT: It's NOT save to create multiple display windows in multiple threads!!!
        const string windowTitle( "mvIMPACT_acquire sample, Device " + pDev->serial.read() );
        pDisplayWindow = new ImageDisplayWindow( windowTitle );
#endif // #ifdef USE_DISPLAY
#ifdef BUILD_WITH_OPENCV_SUPPORT
        openCVDisplayTitle = string( "mvIMPACT_acquire sample, Device " + pDev->serial.read() + ", OpenCV display" );
        openCVResultDisplayTitle = openCVDisplayTitle + "(Result)";
#endif // #ifdef BUILD_WITH_OPENCV_SUPPORT
    }
    ~CaptureParameter()
    {
#ifdef USE_DISPLAY
        delete pDisplayWindow;
#endif // #ifdef USE_DISPLAY
    }
};

void checkCaptureBufferAddress( const Request* const pRequest, bool boShouldContainUserSuppliedMemory, const CaptureBufferContainer& buffers );
int  createCaptureBuffer( FunctionInterface& fi, CaptureBufferContainer& buffers, const int bufferSize, const int bufferAlignment, const int bufferPitch, unsigned int requestNr );
int  createCaptureBuffers( FunctionInterface& fi, CaptureBufferContainer& buffers, const int bufferSize, const int bufferAlignment, const int bufferPitch );
void freeCaptureBuffer( FunctionInterface& fi, CaptureBufferContainer& buffers, unsigned int requestNr );
void freeCaptureBuffers( FunctionInterface& fi, CaptureBufferContainer& buffers );
CaptureBufferContainer::iterator lookUpBuffer( CaptureBufferContainer& buffers, void* pAddr );
void runLiveLoop( CaptureParameter& captureParams );

//-----------------------------------------------------------------------------
/// \brief This function checks whether a buffer returned from an acquisition into a
/// request that has been assigned a user supplied buffer really contains a buffer
/// pointer that has been assigned by the user.
void checkCaptureBufferAddress( const Request* const pRequest, bool boShouldContainUserSuppliedMemory, const CaptureBufferContainer& buffers )
//-----------------------------------------------------------------------------
{
    if( boShouldContainUserSuppliedMemory && ( pRequest->imageMemoryMode.read() != rimmUser ) )
    {
        cout << "ERROR: Request number " << pRequest->getNumber() << " is supposed to contain user supplied memory, but claims that it doesn't." << endl;
        return;
    }
    else if( !boShouldContainUserSuppliedMemory )
    {
        if( pRequest->imageMemoryMode.read() == rimmUser )
        {
            cout << "ERROR: Request number " << pRequest->getNumber() << " is supposed NOT to contain user supplied memory, but claims that it does." << endl;
        }
        return;
    }

    void* pAddr = pRequest->imageData.read();
    const CaptureBufferContainer::size_type vSize = buffers.size();
    for( CaptureBufferContainer::size_type i = 0; i < vSize; i++ )
    {
        if( pAddr == buffers[i]->getPtr() )
        {
            // found the buffer that has been assigned by the user
            return;
        }
    }

    cout << "ERROR: A buffer has been returned, that doesn't match any of the buffers assigned as user memory in request number " << pRequest->getNumber() << "." << endl;
    cout << "Buffer got: 0x" << pAddr << endl;
    cout << "Buffers allocated:" << endl;
    for( CaptureBufferContainer::size_type j = 0; j < vSize; j++ )
    {
        cout << "[" << j << "]: 0x" << reinterpret_cast<void*>( buffers[j]->getPtr() ) << endl;
    }
}

//-----------------------------------------------------------------------------
int createCaptureBuffer( FunctionInterface& fi, CaptureBufferContainer& buffers, const int bufferSize, const int bufferAlignment, const int bufferPitch, unsigned int requestNr )
//-----------------------------------------------------------------------------
{
    int functionResult = DMR_NO_ERROR;
    Request* pRequest = fi.getRequest( requestNr );
    UserSuppliedHeapBuffer* pBuffer = new UserSuppliedHeapBuffer( bufferSize, bufferAlignment, bufferPitch );
    if( ( functionResult = pRequest->attachUserBuffer( pBuffer->getPtr(), pBuffer->getSize() ) ) != DMR_NO_ERROR )
    {
        cout << "An error occurred while attaching a buffer to request number " << requestNr << ": " << ImpactAcquireException::getErrorCodeAsString( functionResult ) << "." << endl;
        delete pBuffer;
        return -1;
    }
    buffers.push_back( pBuffer );
    return 0;
}

//-----------------------------------------------------------------------------
int createCaptureBuffers( FunctionInterface& fi, CaptureBufferContainer& buffers, const int bufferSize, const int bufferAlignment, const int bufferPitch )
//-----------------------------------------------------------------------------
{
    freeCaptureBuffers( fi, buffers );

    unsigned int requestCnt = fi.requestCount();
    for( unsigned int i = 0; i < requestCnt; i++ )
    {
        try
        {
            const int result = createCaptureBuffer( fi, buffers, bufferSize, bufferAlignment, bufferPitch, i );
            if( result != 0 )
            {
                freeCaptureBuffers( fi, buffers );
                return result;
            }
        }
        catch( const ImpactAcquireException& e )
        {
            freeCaptureBuffers( fi, buffers );
            return e.getErrorCode();
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------
void freeCaptureBuffer( FunctionInterface& fi, CaptureBufferContainer& buffers, unsigned int requestNr )
//-----------------------------------------------------------------------------
{
    try
    {
        int functionResult = DMR_NO_ERROR;
        Request* pRequest = fi.getRequest( requestNr );
        if( pRequest->imageMemoryMode.read() == rimmUser )
        {
            if( ( functionResult = pRequest->detachUserBuffer() ) != DMR_NO_ERROR )
            {
                cout << "An error occurred while detaching a buffer from request number " << requestNr << " : " << ImpactAcquireException::getErrorCodeAsString( functionResult ) << "." << endl;
            }
        }
        CaptureBufferContainer::iterator it = lookUpBuffer( buffers, pRequest->imageData.read() );
        if( it != buffers.end() )
        {
            delete ( *it );
            buffers.erase( it );
        }
    }
    catch( const ImpactAcquireException& e )
    {
        cout << "An error occurred while changing the mode of request number " << requestNr << ": " << e.getErrorCodeAsString() << "." << endl;
    }
}

//-----------------------------------------------------------------------------
void freeCaptureBuffers( FunctionInterface& fi, CaptureBufferContainer& buffers )
//-----------------------------------------------------------------------------
{
    const unsigned int requestCnt = fi.requestCount();
    for( unsigned int i = 0; i < requestCnt; i++ )
    {
        freeCaptureBuffer( fi, buffers, i );
    }

    if( buffers.empty() == false )
    {
        cout << "Error! The buffer container should be empty now but still contains " << buffers.size() << " elements!" << endl;
    }
}

//-----------------------------------------------------------------------------
CaptureBufferContainer::iterator lookUpBuffer( CaptureBufferContainer& buffers, void* pAddr )
//-----------------------------------------------------------------------------
{
    const CaptureBufferContainer::iterator itEND = buffers.end();
    CaptureBufferContainer::iterator it = buffers.begin();
    while( it != itEND )
    {
        if( pAddr == ( *it )->getPtr() )
        {
            // found the buffer that has been assigned by the user
            break;
        }
        ++it;
    }
    return it;
}

//-----------------------------------------------------------------------------
void displayImage( CaptureParameter* pCaptureParameter, Request* pRequest )
//-----------------------------------------------------------------------------
{
#if !defined(USE_DISPLAY) && !defined(BUILD_WITH_OPENCV_SUPPORT)
    // suppress compiler warnings
    pRequest = pRequest;
    pCaptureParameter = pCaptureParameter;
#endif // #if !defined(USE_DISPLAY) && !defined(BUILD_WITH_OPENCV_SUPPORT)
#ifdef USE_DISPLAY
    pCaptureParameter->pDisplayWindow->GetImageDisplay().SetImage( pRequest );
    pCaptureParameter->pDisplayWindow->GetImageDisplay().Update();
#endif // #ifdef USE_DISPLAY
#ifdef BUILD_WITH_OPENCV_SUPPORT
    int openCVDataType = CV_8UC1;
    switch( pRequest->imagePixelFormat.read() )
    {
    case ibpfMono8:
        openCVDataType = CV_8UC1;
        break;
    case ibpfMono10:
    case ibpfMono12:
    case ibpfMono14:
    case ibpfMono16:
        openCVDataType = CV_16UC1;
        break;
    case ibpfMono32:
        openCVDataType = CV_32SC1;
        break;
    case ibpfBGR888Packed:
    case ibpfRGB888Packed:
        openCVDataType = CV_8UC3;
        break;
    case ibpfRGBx888Packed:
        openCVDataType = CV_8UC4;
        break;
    case ibpfRGB101010Packed:
    case ibpfRGB121212Packed:
    case ibpfRGB141414Packed:
    case ibpfRGB161616Packed:
        openCVDataType = CV_16UC3;
        break;
    case ibpfMono12Packed_V1:
    case ibpfMono12Packed_V2:
    case ibpfBGR101010Packed_V2:
    case ibpfRGB888Planar:
    case ibpfRGBx888Planar:
    case ibpfYUV422Packed:
    case ibpfYUV422_10Packed:
    case ibpfYUV422_UYVYPacked:
    case ibpfYUV422_UYVY_10Packed:
    case ibpfYUV422Planar:
    case ibpfYUV444Packed:
    case ibpfYUV444_10Packed:
    case ibpfYUV444_UYVPacked:
    case ibpfYUV444_UYV_10Packed:
    case ibpfYUV444Planar:
    case ibpfYUV411_UYYVYY_Packed:
        cout << "ERROR! Don't know how to render this pixel format (" << pRequest->imagePixelFormat.readS() << ") in OpenCV! Select another one e.g. by writing to mvIMPACT::acquire::ImageDestination::pixelFormat!" << endl;
        exit( 42 );
        break;
    }
    cv::Mat openCVImage( cv::Size( pRequest->imageWidth.read(), pRequest->imageHeight.read() ), openCVDataType, pRequest->imageData.read(), pRequest->imageLinePitch.read() );
    cv::imshow( pCaptureParameter->openCVDisplayTitle, openCVImage );
    // OpenCV event handling: you need this!
    cv::waitKey( 5 );
    // apply Canny Edge detection and display the result too
    cv::Mat edgesMat;
    switch( openCVDataType )
    {
    case CV_16UC3:
        cout << "This format seems to crash the Canny Edge detector. Will display the original image instead!" << endl;
        edgesMat = openCVImage;
        break;
    default:
        cv::Canny( openCVImage, edgesMat, 35.0, 55.0 );
        break;
    }
    cv::imshow( pCaptureParameter->openCVResultDisplayTitle, edgesMat );
    // OpenCV event handling: you need this!
    cv::waitKey( 5 );
#endif // #ifdef BUILD_WITH_OPENCV_SUPPORT
}

//-----------------------------------------------------------------------------
unsigned int DMR_CALL liveLoop( void* pData )
//-----------------------------------------------------------------------------
{
    CaptureParameter* pThreadParameter = reinterpret_cast<CaptureParameter*>( pData );

    // Send all requests to the capture queue. There can be more than 1 queue for some device, but for this sample
    // we will work with the default capture queue. If a device supports more than one capture or result
    // queue, this will be stated in the manual. If nothing is mentioned about it, the device supports one
    // queue only. This loop will send all requests currently available to the driver. To modify the number of requests
    // use the property mvIMPACT::acquire::SystemSettings::requestCount at runtime or the property
    // mvIMPACT::acquire::Device::defaultRequestCount BEFORE opening the device.
    TDMR_ERROR result = DMR_NO_ERROR;
    while( ( result = static_cast<TDMR_ERROR>( pThreadParameter->fi.imageRequestSingle() ) ) == DMR_NO_ERROR ) {};
    if( result != DEV_NO_FREE_REQUEST_AVAILABLE )
    {
        cout << "'FunctionInterface.imageRequestSingle' returned with an unexpected result: " << result
             << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
    }

    manuallyStartAcquisitionIfNeeded( pThreadParameter->pDev, pThreadParameter->fi );
    // run thread loop
    unsigned int cnt = 0;
    const unsigned int timeout_ms = 500;
    Request* pRequest = 0;
    // we always have to keep at least 2 images as the display module might want to repaint the image, thus we
    // cannot free it unless we have a assigned the display to a new buffer.
    Request* pPreviousRequest = 0;
    while( !s_boTerminated )
    {
        // wait for results from the default capture queue
        int requestNr = pThreadParameter->fi.imageRequestWaitFor( timeout_ms );
        pRequest = pThreadParameter->fi.isRequestNrValid( requestNr ) ? pThreadParameter->fi.getRequest( requestNr ) : 0;
        if( pRequest )
        {
            if( pRequest->isOK() )
            {
                ++cnt;
                // here we can display some statistical information every 100th image
                if( cnt % 100 == 0 )
                {
                    cout << "Info from " << pThreadParameter->pDev->serial.read()
                         << ": " << pThreadParameter->statistics.framesPerSecond.name() << ": " << pThreadParameter->statistics.framesPerSecond.readS()
                         << ", " << pThreadParameter->statistics.errorCount.name() << ": " << pThreadParameter->statistics.errorCount.readS()
                         << ", " << pThreadParameter->statistics.captureTime_s.name() << ": " << pThreadParameter->statistics.captureTime_s.readS()
                         << ", CaptureDimension: " << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << "(" << pRequest->imagePixelFormat.readS() << ")" << endl;
                }
                displayImage( pThreadParameter, pRequest );
                checkCaptureBufferAddress( pRequest, pThreadParameter->boUserSuppliedMemoryUsed, pThreadParameter->buffers );
            }
            else
            {
                cout << "Error: " << pRequest->requestResult.readS() << endl;
            }
            if( pPreviousRequest )
            {
                // this image has been displayed thus the buffer is no longer needed...
                pPreviousRequest->unlock();
                if( pThreadParameter->boAlwaysUseNewUserSuppliedBuffers )
                {
                    // attach a fresh piece of memory
                    freeCaptureBuffer( pThreadParameter->fi, pThreadParameter->buffers, pPreviousRequest->getNumber() );
                    createCaptureBuffer( pThreadParameter->fi, pThreadParameter->buffers, pThreadParameter->bufferSize, pThreadParameter->bufferAlignment, pThreadParameter->bufferPitch, pPreviousRequest->getNumber() );
                }
            }
            pPreviousRequest = pRequest;
            // send a new image request into the capture queue
            pThreadParameter->fi.imageRequestSingle();
        }
        else
        {
            // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
            // additional information under TDMR_ERROR in the interface reference
            cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")"
                 << ", timeout value too small?" << endl;
        }
#if defined(linux) || defined(__linux) || defined(__linux__)
        s_boTerminated = checkKeyboardInput() == 0 ? false : true;
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    }
    manuallyStopAcquisitionIfNeeded( pThreadParameter->pDev, pThreadParameter->fi );
#ifdef USE_DISPLAY
    // stop the display from showing freed memory
    pThreadParameter->pDisplayWindow->GetImageDisplay().SetImage( reinterpret_cast<Request*>( 0 ) );
#endif // #ifdef USE_DISPLAY
#ifdef BUILD_WITH_OPENCV_SUPPORT
    cv::destroyAllWindows();
#endif // #ifdef BUILD_WITH_OPENCV_SUPPORT
    // In this sample all the next lines are redundant as the device driver will be
    // closed now, but in a real world application a thread like this might be started
    // several times an then it becomes crucial to clean up correctly.

    // free the last potentially locked request
    if( pRequest )
    {
        pRequest->unlock();
    }
    // clear the request queue
    pThreadParameter->fi.imageRequestReset( 0, 0 );
    // extract and unlock all requests that are now returned as 'aborted'
    int requestNr = INVALID_ID;
    while( ( requestNr = pThreadParameter->fi.imageRequestWaitFor( 0 ) ) >= 0 )
    {
        pThreadParameter->fi.imageRequestUnlock( requestNr );
    }
    return 0;
}

//-----------------------------------------------------------------------------
void runLiveLoop( CaptureParameter& captureParams )
//-----------------------------------------------------------------------------
{
    s_boTerminated = false;
#if defined(linux) || defined(__linux) || defined(__linux__)
    liveLoop( &captureParams );
#else
    // start the execution of the 'live' thread.
    unsigned int dwThreadID = 0;
    HANDLE hThread = ( HANDLE )_beginthreadex( 0, 0, liveLoop, ( LPVOID )( &captureParams ), 0, &dwThreadID );
    cin.get();
    s_boTerminated = true;
    WaitForSingleObject( hThread, INFINITE );
    CloseHandle( hThread );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
}

//-----------------------------------------------------------------------------
int main( int /*argc*/, char* /*argv*/[] )
//-----------------------------------------------------------------------------
{
    DeviceManager devMgr;
    Device* pDev = getDeviceFromUserInput( devMgr );
    if( !pDev )
    {
        cout << "Unable to continue!";
        cout << "Press [ENTER] to end the application" << endl;
        cin.get();
        return 0;
    }

    cout << "Initialising the device. This might take some time..." << endl;
    try
    {
        pDev->open();
    }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while opening device " << pDev->serial.read()
             << "(error code: " << e.getErrorCodeAsString() << ")." << endl
             << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 0;
    }

    CaptureParameter captureParams( pDev );

    //=============================================================================
    //========= Capture loop into memory managed by the driver (default) ==========
    //=============================================================================
    cout << "The device will try to capture continuously into memory automatically allocated be the device driver." << endl
         << "This is the default behaviour." << endl;
    cout << "Press [ENTER] to end the continuous acquisition." << endl;
    runLiveLoop( captureParams );

    //=============================================================================
    //========= Capture loop into memory managed by the user (advanced) ===========
    //=============================================================================
    cout << "The device will now try to capture continuously into user supplied memory." << endl;
    captureParams.boUserSuppliedMemoryUsed = true;
    // find out the size of the resulting buffer by requesting a dummy request
    int bufferAlignment = 0;
    Request* pCurrentCaptureBufferLayout = 0;
    captureParams.fi.getCurrentCaptureBufferLayout( captureParams.irc, &pCurrentCaptureBufferLayout, &bufferAlignment );
    int bufferSize = pCurrentCaptureBufferLayout->imageSize.read() + pCurrentCaptureBufferLayout->imageFooterSize.read();
    int bufferPitch = pCurrentCaptureBufferLayout->imageLinePitch.read();
    int result = createCaptureBuffers( captureParams.fi, captureParams.buffers, bufferSize, bufferAlignment, bufferPitch );
    if( result != 0 )
    {
        cout << "An error occurred while setting up the user supplied buffers for device " << captureParams.pDev->serial.read()
             << "(error code: " << ImpactAcquireException::getErrorCodeAsString( result ) << ")." << endl
             << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 0;
    }
    cout << "Press [ENTER] to end the continuous acquisition into user supplied memory." << endl;
    runLiveLoop( captureParams );

    //=============================================================================
    //========= unregister user supplied buffers again ============================
    //=============================================================================
    freeCaptureBuffers( captureParams.fi, captureParams.buffers );

    //=============================================================================
    //========= Capture loop into memory managed by the user using a fresh ========
    //========= buffer for each image(advanced) ===================================
    //=============================================================================
    cout << "The device will now try to capture continuously into user supplied memory using a new buffer for each image thus constantly re-allocating and freeing user memory." << endl;
    captureParams.boUserSuppliedMemoryUsed = true;
    captureParams.boAlwaysUseNewUserSuppliedBuffers = true;
    captureParams.bufferSize = bufferSize;
    captureParams.bufferAlignment = bufferAlignment;
    captureParams.bufferPitch = bufferPitch;
    result = createCaptureBuffers( captureParams.fi, captureParams.buffers, bufferSize, bufferAlignment, bufferPitch );
    if( result != 0 )
    {
        cout << "An error occurred while setting up the user supplied buffers for device " << captureParams.pDev->serial.read()
             << "(error code: " << ImpactAcquireException::getErrorCodeAsString( result ) << ")." << endl
             << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 0;
    }
    cout << "Press [ENTER] to end the continuous acquisition." << endl;
    runLiveLoop( captureParams );

    //=============================================================================
    //========= unregister user supplied buffers again ============================
    //=============================================================================
    freeCaptureBuffers( captureParams.fi, captureParams.buffers );

    //=============================================================================
    //========= Capture loop into memory managed by the driver again (default) ====
    //=============================================================================
    captureParams.boUserSuppliedMemoryUsed = false;
    captureParams.boAlwaysUseNewUserSuppliedBuffers = false;
    cout << "The device will try to capture continuously into memory automatically allocated be the device driver again." << endl
         << "This is the default behaviour." << endl;
    cout << "Press [ENTER] to end the continuous acquisition." << endl;
    runLiveLoop( captureParams );

    //=============================================================================
    //========= Capture into a specific buffer managed by the user (advanced) =====
    //=============================================================================
    // by default the driver will decide which request will be used for an acquisition
    // requested by the user. However sometimes it can be necessary to make sure that a
    // certain request object will be used...
    cout << "Now the device will try to capture one frame into a specific user supplied buffer" << endl;
    UserSuppliedHeapBuffer buffer( bufferSize, bufferAlignment, bufferPitch );
    const int REQUEST_TO_USE = 2;
    // we want to use request number 'REQUEST_TO_USE' (zero based) for this acquisition thus we have to make sure
    // that there are at least 'REQUEST_TO_USE + 1' requests
    SystemSettings ss( pDev );
    ss.requestCount.write( REQUEST_TO_USE + 1 );
    // associate a user supplied buffer with this request
    Request* pRequest = captureParams.fi.getRequest( REQUEST_TO_USE );
    try
    {
        int functionResult = pRequest->attachUserBuffer( buffer.getPtr(), buffer.getSize() );
        if( functionResult != DMR_NO_ERROR )
        {
            cout << "An error occurred while attaching a user buffer to request number " << REQUEST_TO_USE << ": " << ImpactAcquireException::getErrorCodeAsString( functionResult ) << "." << endl;
            cout << "Press [ENTER] to end the application." << endl;
            cin.get();
            return 0;
        }
    }
    catch( const ImpactAcquireException& e )
    {
        cout << "An error occurred while attaching a user buffer to request number " << REQUEST_TO_USE << ": " << ImpactAcquireException::getErrorCodeAsString( e.getErrorCode() ) << "." << endl;
        cout << "Press [ENTER] to end the application." << endl;
        cin.get();
        return 0;
    }
    // define that 'REQUEST_TO_USE' is used for the next acquisition
    captureParams.irc.requestToUse.write( REQUEST_TO_USE );
    // and capture the image
    int requestUsed = INVALID_ID;
    result = captureParams.fi.imageRequestSingle( &captureParams.irc, &requestUsed );
    if( result != DMR_NO_ERROR )
    {
        cout << "An error occurred while requesting an image for request number " << REQUEST_TO_USE << ": " << ImpactAcquireException::getErrorCodeAsString( result ) << "." << endl;
        cout << "Press [ENTER] to end the application." << endl;
        cin.get();
        return 0;
    }
    if( requestUsed != REQUEST_TO_USE )
    {
        cout << "ERROR! An acquisition into buffer " << REQUEST_TO_USE << " was requested, but the driver did use " << requestUsed << " for this acquisition." << endl;
    }

    manuallyStartAcquisitionIfNeeded( pDev, captureParams.fi );
    // Wait for the buffer to get filled
    int requestNr = captureParams.fi.imageRequestWaitFor( -1 );
    manuallyStopAcquisitionIfNeeded( pDev, captureParams.fi );

    pRequest = captureParams.fi.getRequest( requestNr );
    if( !pRequest->isOK() )
    {
        cout << "Error: " << pRequest->requestResult.readS() << endl;
        cout << "Press [ENTER] to end the application..." << endl;
        cin.get();
        return 0;
    }
    cout << "Capture into specific user supplied buffer done." << endl;
    displayImage( &captureParams, pRequest );

    // and end the application
    cout << "Press [ENTER] to end the application..." << endl;
    cin.get();
    captureParams.fi.imageRequestUnlock( requestNr );
    captureParams.fi.imageRequestReset( 0, 0 );
    return 0;
}
