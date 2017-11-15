#include <apps/Common/exampleHelper_C.h>
#include <assert.h>
#include <limits.h>
#include <mvDeviceManager/Include/mvDeviceManager.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#   include <conio.h>
#   include <mvDisplay/Include/mvDisplayExtensions.h>
#   include <mvDisplay/Include/mvDisplayWindow.h>
#   include <process.h>
#   include <windows.h>
#   define USE_MV_DISPLAY_LIB // only available for Windows
#   define LIVE_LOOP_CALL __stdcall
#elif defined(linux) || defined(__linux) || defined(__linux__)
#   include <unistd.h>
#   define LIVE_LOOP_CALL
#   if defined(__x86_64__) || defined(__powerpc64__) // -m64 makes GCC define __powerpc64__
typedef uint64_t UINT_PTR;
#   elif defined(__i386__) || defined(__arm__) || defined(__powerpc__) // and -m32 __powerpc__
typedef uint32_t UINT_PTR;
#   else
#       error unsupported target platform
#   endif
#else
#   error unsupported target platform
#endif // #ifdef _WIN32

#define BUF_SIZE (512) // should always be enough, but could be done nicer by checking if buffer too small is returned from a function and then perform a re-allocation...

static int s_boTerminated = 0;

//=============================================================================
//================= Data type definitions =====================================
//=============================================================================
//-----------------------------------------------------------------------------
// the buffer we pass to the device driver must be aligned according to its requirements
// As we can't allocate aligned heap memory we will align it 'by hand'
typedef struct _UserSuppliedHeapBuffer
//-----------------------------------------------------------------------------
{
    char* pBuf_;
    char* pBufAligned_;
    int bufSize_;
    int alignment_;
    void* pNext_;
    void* pPrev_;
} UserSuppliedHeapBuffer;

//-----------------------------------------------------------------------------
typedef struct _Request
//-----------------------------------------------------------------------------
{
    HOBJ hImageData_;
    HOBJ hImageFooterSize_;
    HOBJ hImageMemoryMode_;
    HOBJ hImageSize_;
    int  nr_;
} Request;

//-----------------------------------------------------------------------------
typedef struct _CaptureParameter
//-----------------------------------------------------------------------------
{
    HDRV                    hDrv;
#ifdef USE_MV_DISPLAY_LIB
    HDISP                   hDisp;
    TDisp*                  pDisp;
#endif // #ifdef USE_MV_DISPLAY_LIB
    Request**               ppRequests;
    size_t                  requestCount;
    UserSuppliedHeapBuffer* pFirstHeapBuffer;
    HOBJ                    hRequestCount;
    HOBJ                    hFramesPerSecond;
    HOBJ                    hCaptureBufferAlignment;
    HOBJ                    hRequestControl_Mode;
    HOBJ                    hRequestControl_RequestToUse;
    int                     boUserSuppliedMemoryUsed;
} CaptureParameter;

//=============================================================================
//================= function declarations =====================================
//=============================================================================

int isPowerOfTwo( int val );

// functions working with a 'UserSuppliedHeapBuffer' struct
UserSuppliedHeapBuffer* UserSuppliedHeapBuffer_Alloc( int bufSize, int alignment );
void                    UserSuppliedHeapBuffer_Free( UserSuppliedHeapBuffer* p );
void                    UserSuppliedHeapBuffer_Insert( UserSuppliedHeapBuffer* p, UserSuppliedHeapBuffer* pAfter );
void                    UserSuppliedHeapBuffer_Remove( UserSuppliedHeapBuffer* p );

// functions working with a 'Request' struct
Request* Request_Alloc( HDRV hDrv, int nr );
void     Request_Free( Request* p );
int      Request_IsValid( Request* p );

// functions for handling user supplied buffers attached to requests
void checkCaptureBufferAddress( const Request* const pRequest, int boShouldContainUserSuppliedMemory, const UserSuppliedHeapBuffer* pFirstBuffer );
int  createCaptureBuffers( CaptureParameter* pCaptureParameters, int bufferSize, int bufferAlignment );
void freeCaptureBuffers( CaptureParameter* pCaptureParameters );

// functions for handling requests
void allocateRequests( CaptureParameter* captureParams );
void freeRequests( CaptureParameter* captureParams );

// functions for controlling the acquisition
void captureLoop( CaptureParameter* pCaptureParams );

//---------------------------------------------------------------------------
int isPowerOfTwo( int val )
//---------------------------------------------------------------------------
{
    return ( ( ( val & ( val - 1 ) ) == 0 ) && ( val > 0 ) );
}

//-----------------------------------------------------------------------------
UserSuppliedHeapBuffer* UserSuppliedHeapBuffer_Alloc( int bufSize, int alignment )
//-----------------------------------------------------------------------------
{
    UserSuppliedHeapBuffer* p = ( UserSuppliedHeapBuffer* )calloc( 1, sizeof( UserSuppliedHeapBuffer ) );
    p->bufSize_ = bufSize;
    p->alignment_ = alignment;
    if( bufSize > 0 )
    {
        assert( isPowerOfTwo( alignment ) );
        p->pBuf_ = ( char* )calloc( 1, bufSize + alignment );
#if defined(_MSC_VER) && (_MSC_VER < 1400) // is older then Microsoft VS 2005 compiler?
        p->pBufAligned_ = ( char* )( ( ( ( UINT_PTR )( p->pBuf_ ) + alignment - 1 ) & ( UINT_MAX - ( alignment - 1 ) ) ) );
#elif defined(_WIN32) && defined(__BORLANDC__) // is Borland C++ Builder?
        p->pBufAligned_ = ( char* )( ( ( ( UINT_PTR )( p->pBuf_ ) + alignment - 1 ) & ( UINT_MAX - ( alignment - 1 ) ) ) );
#else
        p->pBufAligned_ = ( char* )( ( ( ( UINT_PTR )( p->pBuf_ ) + alignment - 1 ) & ( SIZE_MAX - ( alignment - 1 ) ) ) );
#endif // #if defined(_MSC_VER) && (_MSC_VER < 1300) // is 'old' Microsoft VC6 compiler?
    }
    else
    {
        p->pBuf_ = 0;
        p->pBufAligned_ = 0;
    }
    p->pNext_ = 0;
    p->pPrev_ = 0;
    return p;
}

//-----------------------------------------------------------------------------
void UserSuppliedHeapBuffer_Free( UserSuppliedHeapBuffer* p )
//-----------------------------------------------------------------------------
{
    if( p )
    {
        if( p->pBuf_ )
        {
            free( p->pBuf_ );
            p->pBuf_ = 0;
        }
        free( p );
    }
}

//-----------------------------------------------------------------------------
void UserSuppliedHeapBuffer_Insert( UserSuppliedHeapBuffer* p, UserSuppliedHeapBuffer* pAfter )
//-----------------------------------------------------------------------------
{
    if( p )
    {
        if( pAfter )
        {
            p->pNext_ = pAfter->pNext_;
            pAfter->pNext_ = p;
        }
        else
        {
            p->pNext_ = 0;
        }
        p->pPrev_ = pAfter;
    }
}

//-----------------------------------------------------------------------------
void UserSuppliedHeapBuffer_Remove( UserSuppliedHeapBuffer* p )
//-----------------------------------------------------------------------------
{
    if( p )
    {
        if( p->pNext_ )
        {
            ( ( UserSuppliedHeapBuffer* )( p->pNext_ ) )->pPrev_ = p->pPrev_;
        }
        if( p->pPrev_ )
        {
            ( ( UserSuppliedHeapBuffer* )( p->pPrev_ ) )->pNext_ = p->pNext_;
        }
    }
}

//-----------------------------------------------------------------------------
Request* Request_Alloc( HDRV hDrv, int nr )
//-----------------------------------------------------------------------------
{
    Request* p = ( Request* )calloc( 1, sizeof( Request ) );
    p->hImageData_ = getRequestProp( hDrv, nr, "Data" );
    p->hImageFooterSize_ = getRequestProp( hDrv, nr, "FooterSize" );
    p->hImageMemoryMode_ = getRequestProp( hDrv, nr, "MemoryMode" );
    p->hImageSize_ = getRequestProp( hDrv, nr, "Size" );
    p->nr_ = nr;
    return p;
}

//-----------------------------------------------------------------------------
void Request_Free( Request* p )
//-----------------------------------------------------------------------------
{
    if( p )
    {
        free( p );
    }
}

//-----------------------------------------------------------------------------
int Request_IsValid( Request* p )
//-----------------------------------------------------------------------------
{
    if( ( p->hImageData_ != INVALID_ID ) &&
        ( p->hImageMemoryMode_ != INVALID_ID ) &&
        ( p->hImageSize_ != INVALID_ID ) &&
        ( p->nr_ >= 0 ) )
    {
        return 1;
    }
    return 0;
}

//-----------------------------------------------------------------------------
/// \brief This function checks whether a buffer returned from an acquisition into a
/// request that has been assigned a user supplied buffer really contains a buffer
/// pointer that has been assigned by the user.
void checkCaptureBufferAddress( const Request* const pRequest, int boShouldContainUserSuppliedMemory, const UserSuppliedHeapBuffer* pFirstBuffer )
//-----------------------------------------------------------------------------
{
    TRequestImageMemoryMode memoryMode = rimmAuto;
    void* pAddr = 0;
    const UserSuppliedHeapBuffer* pBuffer = pFirstBuffer;
    int cnt = 0;

    memoryMode = ( TRequestImageMemoryMode )getPropI( pRequest->hImageMemoryMode_, 0 );
    if( boShouldContainUserSuppliedMemory && ( memoryMode != rimmUser ) )
    {
        printf( "ERROR: Request number %d is supposed to contain user supplied memory, but claims that it doesn't.\n", pRequest->nr_ );
        return;
    }
    else if( !boShouldContainUserSuppliedMemory )
    {
        if( memoryMode == rimmUser )
        {
            printf( "ERROR: Request number %d is supposed NOT to contain user supplied memory, but claims that it does.\n", pRequest->nr_ );
        }
        return;
    }

    pAddr = getPropP( pRequest->hImageData_, 0 );
    pBuffer = pFirstBuffer;
    while( pBuffer )
    {
        if( pAddr == pBuffer->pBufAligned_ )
        {
            // found the buffer that has been assigned by the user
            return;
        }
        pBuffer = pBuffer->pNext_;
    }

    printf( "ERROR: A buffer has been returned, that doesn't match any of the buffers assigned as user memory in request number %d.\n", pRequest->nr_ );
    printf( "Buffer got: %p.\n", pAddr );
    printf( "Buffers allocated:\n" );
    pBuffer = pFirstBuffer;
    while( pBuffer )
    {
        printf( "[%d]: %p.\n", cnt, pBuffer->pBufAligned_ );
        cnt = cnt + 1;
    }
}

//-----------------------------------------------------------------------------
int createCaptureBuffers( CaptureParameter* pCaptureParameters, int bufferSize, int bufferAlignment )
//-----------------------------------------------------------------------------
{
    size_t i = 0;
    UserSuppliedHeapBuffer* pBuffer = 0;

    freeCaptureBuffers( pCaptureParameters );

    for( i = 0; i < pCaptureParameters->requestCount; i++ )
    {
        int functionResult = DMR_NO_ERROR;
        if( ( functionResult = DMR_ImageRequestConfigure( pCaptureParameters->hDrv, pCaptureParameters->ppRequests[i]->nr_, 0, 0 ) ) != DMR_NO_ERROR )
        {
            printf( "An error occurred while setting request number %d in configuration mode: %s.\n", pCaptureParameters->ppRequests[i]->nr_, DMR_ErrorCodeToString( functionResult ) );
            freeCaptureBuffers( pCaptureParameters );
            return -1;
        }

        pBuffer = UserSuppliedHeapBuffer_Alloc( bufferSize, bufferAlignment );
        if( pCaptureParameters->pFirstHeapBuffer )
        {
            UserSuppliedHeapBuffer_Insert( pBuffer, pCaptureParameters->pFirstHeapBuffer );
        }
        else
        {
            pCaptureParameters->pFirstHeapBuffer = pBuffer;
        }

        setPropI( pCaptureParameters->ppRequests[i]->hImageMemoryMode_, rimmUser, 0 );
        setPropP( pCaptureParameters->ppRequests[i]->hImageData_, pBuffer->pBufAligned_, 0 );
        setPropI( pCaptureParameters->ppRequests[i]->hImageSize_, pBuffer->bufSize_, 0 );

        if( ( functionResult = DMR_ImageRequestUnlock( pCaptureParameters->hDrv, pCaptureParameters->ppRequests[i]->nr_ ) ) != DMR_NO_ERROR )
        {
            printf( "An error occurred while unlocking request number %d: %s.\n", pCaptureParameters->ppRequests[i]->nr_, DMR_ErrorCodeToString( functionResult ) );
            freeCaptureBuffers( pCaptureParameters );
            return -1;
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------
void freeCaptureBuffers( CaptureParameter* pCaptureParameters )
//-----------------------------------------------------------------------------
{
    size_t i = 0;
    TRequestImageMemoryMode memoryMode = rimmAuto;
    UserSuppliedHeapBuffer* pBuffer = pCaptureParameters->pFirstHeapBuffer;

    for( i = 0; i < pCaptureParameters->requestCount; i++ )
    {
        int functionResult = DMR_NO_ERROR;
        memoryMode = ( TRequestImageMemoryMode )getPropI( pCaptureParameters->ppRequests[i]->hImageMemoryMode_, 0 );
        if( memoryMode != rimmUser )
        {
            continue;
        }

        if( ( functionResult = DMR_ImageRequestConfigure( pCaptureParameters->hDrv, pCaptureParameters->ppRequests[i]->nr_, 0, 0 ) ) != DMR_NO_ERROR )
        {
            printf( "An error occurred while setting request number %d in configuration mode: %s.\n", pCaptureParameters->ppRequests[i]->nr_, DMR_ErrorCodeToString( functionResult ) );
            continue;
        }

        setPropI( pCaptureParameters->ppRequests[i]->hImageMemoryMode_, rimmAuto, 0 );

        if( ( functionResult = DMR_ImageRequestUnlock( pCaptureParameters->hDrv, pCaptureParameters->ppRequests[i]->nr_ ) ) != DMR_NO_ERROR )
        {
            printf( "An error occurred while unlocking request number %d: %s.\n", pCaptureParameters->ppRequests[i]->nr_, DMR_ErrorCodeToString( functionResult ) );
        }
    }

    while( pBuffer )
    {
        UserSuppliedHeapBuffer* p = pBuffer;
        pBuffer = pBuffer->pNext_;
        UserSuppliedHeapBuffer_Free( p );
    }
    pCaptureParameters->pFirstHeapBuffer = 0;
}

//-----------------------------------------------------------------------------
void allocateRequests( CaptureParameter* captureParams )
//-----------------------------------------------------------------------------
{
    size_t i = 0;
    freeRequests( captureParams );
    captureParams->requestCount = ( size_t )getPropI( captureParams->hRequestCount, 0 );
    captureParams->ppRequests = 0;
    if ( captureParams->requestCount > 0 )
    {
        captureParams->ppRequests = ( Request** )( calloc( captureParams->requestCount, sizeof( Request* ) ) );
        if ( captureParams->ppRequests == 0 )
        {
            captureParams->requestCount = 0;
            return;
        }
        for( i = 0; i < captureParams->requestCount; i++ )
        {
            captureParams->ppRequests[i] = Request_Alloc( captureParams->hDrv, ( int )i );
            if ( captureParams->ppRequests == 0 )
            {
                captureParams->requestCount = 0;
                return;
            }
        }
    }
}

//-----------------------------------------------------------------------------
void freeRequests( CaptureParameter* captureParams )
//-----------------------------------------------------------------------------
{
    size_t i = 0;
    if( captureParams->requestCount <= 0 )
    {
        return;
    }

    for( i = 0; i < captureParams->requestCount; i++ )
    {
        Request_Free( captureParams->ppRequests[i] );
    }
    free( captureParams->ppRequests );
    captureParams->ppRequests = 0;
    captureParams->requestCount = 0;
}

//-----------------------------------------------------------------------------
unsigned int LIVE_LOOP_CALL liveLoop( void* pData )
//-----------------------------------------------------------------------------
{
    HDRV              hDrv;
    TDMR_ERROR        result;
    ImageBuffer*      pIB;
    RequestResult     ReqRes;
    int               frameCount;
    double            fps;
    int               requestNr;
    int               lastRequestNr;
    CaptureParameter* pCaptureParameter;

    pCaptureParameter = ( ( CaptureParameter* )pData );
    hDrv = pCaptureParameter->hDrv;
    pIB = 0;
    frameCount = 0;
    fps = 0.0;
    requestNr = -1;
    // we always have to keep at least 2 images as the display module might want to repaint the image, thus we
    // cannot free it unless we have a assigned the display to a new buffer.
    lastRequestNr = -1;

    // pre-fill the default capture queue
    while( ( result = DMR_ImageRequestSingle( hDrv, 0, 0 ) ) == DMR_NO_ERROR );
    if( result != DEV_NO_FREE_REQUEST_AVAILABLE )
    {
        printf( "DMR_ImageRequestSingle: Unexpected error(code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
    }

    manuallyStartAcquisitionIfNeeded( hDrv );
    // run thread loop
    while( !s_boTerminated )
    {
        // please note that the value stored in the property 'ImageRequestTimeout_ms' specifies the
        // maximum time a request will remain in the queue. If no complete image has been taken until
        // then, RequestResult.result will contain 'rrTimeout', so to allow long wait times, this
        // property needs to be modified as well, as its default is 2000 ms.
        // In this sample this can be achieved by calling 'getSettingProp( hDrv, "Base", "ImageRequestTimeout_ms" )'
        result = DMR_ImageRequestWaitFor( hDrv, 500, 0, &requestNr );
        if( result == DMR_NO_ERROR )
        {
            // check if the request contains a valid image
            result = DMR_GetImageRequestResultEx( hDrv, requestNr, &ReqRes, sizeof( ReqRes ), 0, 0 );
            if( ( result == DMR_NO_ERROR ) && ( ReqRes.result == rrOK ) )
            {
                // display statistical information every 100th image
                frameCount = frameCount + 1;
                if( ( frameCount % 100 ) == 0 )
                {
                    OBJ_GetF( pCaptureParameter->hFramesPerSecond, &fps, 0 );
                    printf( "frames per second: %.5f.\n", fps );
                }
                if( ( result = DMR_GetImageRequestBuffer( hDrv, requestNr, &pIB ) ) == DMR_NO_ERROR )
                {
#ifdef USE_MV_DISPLAY_LIB
                    // display the captured image
                    mvDispSetImageFromImageBuffer( pCaptureParameter->pDisp, pIB );
                    mvDispUpdate( pCaptureParameter->pDisp );
#endif // #ifdef USE_MV_DISPLAY_LIB
                    checkCaptureBufferAddress( pCaptureParameter->ppRequests[requestNr], pCaptureParameter->boUserSuppliedMemoryUsed, pCaptureParameter->pFirstHeapBuffer );
                }
                else
                {
                    printf( "DMR_GetImageRequestBuffer: ERROR! Code %d\n", result );
                }
            }
            else
            {
                // this can happen e.g. when a triggered acquisition timed out (missing trigger signal)
                // A request does not remain in teh queue forever, but is removed after the max. queue time has elapsed. This timeout
                // is defined by the 'ImageRequestTimeout_ms' property. If this timeout has elapsed and no
                // image has been captured, the RequestResult.result parameter will not contain 'rrOK', but the
                // request still needs to be unlocked for the driver as it has been returned to the user.
                printf( "DMR_GetImageRequestResult: ERROR! Return value: %d, request result: %d.\n", result, ReqRes.result );
            }
            if( lastRequestNr >= 0 )
            {
                // this image has been displayed thus the buffer is no longer needed...
                DMR_ImageRequestUnlock( hDrv, lastRequestNr );
            }
            lastRequestNr = requestNr;
            DMR_ImageRequestSingle( hDrv, 0, 0 );
        }
        else
        {
            printf( "DMR_ImageRequestWaitFor: ERROR! Code %d\n", result );
        }
#if defined(linux) || defined(__linux) || defined(__linux__)
        s_boTerminated = waitForInput( 0, STDOUT_FILENO ) == 0 ? 0 : 1; // break by STDIN
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    }
    manuallyStopAcquisitionIfNeeded( hDrv );

#ifdef USE_MV_DISPLAY_LIB
    // remove the displays reference to memory that is about to be freed
    mvDispSetImage( pCaptureParameter->pDisp, 0, 0, 0, 0, 0 );
#endif // #ifdef USE_MV_DISPLAY_LIB

    // free the last potentially locked request
    if( requestNr >= 0 )
    {
        DMR_ImageRequestUnlock( hDrv, requestNr );
    }
    // clear the request queue
    if( ( result = DMR_ImageRequestReset( hDrv, 0, 0 ) ) != DMR_NO_ERROR )
    {
        printf( "Calling DMR_ImageRequestReset returned an error: %s.\n", DMR_ErrorCodeToString( result ) );
    }
    // extract and unlock all requests that are now returned as 'aborted'
    while( DMR_ImageRequestWaitFor( hDrv, 0, 0, &requestNr ) == DMR_NO_ERROR )
    {
        DMR_ImageRequestUnlock( hDrv, requestNr );
    }
    if( ( result = DMR_ReleaseImageRequestBufferDesc( &pIB ) ) != DMR_NO_ERROR )
    {
        printf( "Calling DMR_ReleaseImageRequestBufferDesc returned an error: %s.\n", DMR_ErrorCodeToString( result ) );
    }
    return 0;
}

//-----------------------------------------------------------------------------
void captureLoop( CaptureParameter* pCaptureParams )
//-----------------------------------------------------------------------------
{
#ifdef _WIN32
    HANDLE                     hThread = NULL;
    unsigned int               threadID = 0;
#endif // #ifdef _WIN32

    printf( "Press [ENTER] to end the continuous acquisition.\n" );
    s_boTerminated = 0;
#ifdef _WIN32
    hThread = ( HANDLE )_beginthreadex( 0, 0, liveLoop, ( LPVOID )( pCaptureParams ), 0, &threadID );
    _getch();
    s_boTerminated = 1;
    WaitForSingleObject( hThread, INFINITE );
    CloseHandle( hThread );
#else
    liveLoop( pCaptureParams );
#endif // #ifdef _WIN32
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    HDMR                       hDMR = INVALID_ID;
    HDRV                       hDrv = INVALID_ID;
    HDEV                       hDevice = INVALID_ID;
    TDMR_ERROR                 result = DMR_NO_ERROR;
    CaptureParameter           captureParams;
    unsigned int               i = 0;
    HOBJ                       hPropFamily = INVALID_ID;
    char*                      pStringBuffer = NULL;
    UserSuppliedHeapBuffer*    pUserSuppliedBuffer = 0;
    int                        requestNr = INVALID_ID;
    int                        requestUsed = INVALID_ID;
    int                        bufferSize = 0;
    RequestResult              ReqRes;
    ImageBuffer*               pIB = 0;
    const int                  REQUEST_TO_USE = 2;

    // get rid of warnings
    argc = argc;
    argv = argv;

    // try to initialise the library.
    if( ( result = DMR_Init( &hDMR ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_Init failed (code: %d)\n", result );
        END_APPLICATION;
    }

    getDeviceFromUserInput( &hDevice, 0, 1 );

    if( ( hPropFamily = getDeviceProp( hDevice, "Family" ) ) == INVALID_ID )
    {
        printf( "Failed to obtain device family property for device %d.\n", i );
        END_APPLICATION;
    }
    getStringValue( hPropFamily, &pStringBuffer, 0 );
    free( pStringBuffer );
    pStringBuffer = 0;

    // try to initialise this device
    if( ( result = DMR_OpenDevice( hDevice, &hDrv ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_OpenDevice failed (code: %d)\n", result );
        printf( "DMR_Close: %d\n", DMR_Close() );
        END_APPLICATION;
    }

#ifdef USE_MV_DISPLAY_LIB
    // create a window to display the captured images
    captureParams.hDisp = mvDispWindowCreate( "CaptureToUserMemory sample(plain 'C')" );
    captureParams.pDisp = mvDispWindowGetDisplayHandle( captureParams.hDisp );
    mvDispWindowShow( captureParams.hDisp );
#endif // #ifdef USE_MV_DISPLAY_LIB
    captureParams.hDrv = hDrv;
    captureParams.pFirstHeapBuffer = 0;
    captureParams.requestCount = 0;
    captureParams.ppRequests = 0;
    // try to locate the frames per second property
    if( ( captureParams.hFramesPerSecond = getStatisticProp( hDrv, "FramesPerSecond" ) ) == INVALID_ID )
    {
        printf( "Couldn't locate 'FramesPerSecond' property! Unable to continue!\n" );
        END_APPLICATION;
    }

    if( ( captureParams.hRequestCount = getSystemSettingProp( hDrv, "RequestCount" ) ) == INVALID_ID )
    {
        printf( "Couldn't locate 'RequestCount' property! Unable to continue!\n" );
        END_APPLICATION;
    }

    if( ( captureParams.hCaptureBufferAlignment = getInfoProp( hDrv, "CaptureBufferAlignment" ) ) == INVALID_ID )
    {
        printf( "Couldn't locate 'CaptureBufferAlignment' property! Unable to continue!\n" );
        END_APPLICATION;
    }

    if( ( captureParams.hRequestControl_Mode = getRequestCtrlProp( hDrv, "Base", "Mode" ) ) == INVALID_ID )
    {
        printf( "Couldn't locate request controls 'Mode' property! Unable to continue!\n" );
        END_APPLICATION;
    }

    if( ( captureParams.hRequestControl_RequestToUse = getRequestCtrlProp( hDrv, "Base", "RequestToUse" ) ) == INVALID_ID )
    {
        printf( "Couldn't locate request controls 'RequestToUse' property! Unable to continue!\n" );
        END_APPLICATION;
    }

    allocateRequests( &captureParams );
    if( captureParams.requestCount <= 0 )
    {
        printf( "Couldn't allocate requests! Unable to continue!\n" );
        END_APPLICATION;
    }

    //=============================================================================
    //========= Capture loop into memory managed by the driver (default) ==========
    //=============================================================================
    printf( "The device will try to capture continuously into memory automatically allocated be the device driver..\n" );
    printf( "This is the default behaviour.\n" );
    captureParams.boUserSuppliedMemoryUsed = 0;
    captureLoop( &captureParams );

    //=============================================================================
    //========= Capture loop into memory managed by the user (advanced) ===========
    //=============================================================================
    printf( "The device will now try to capture continuously into user supplied memory.\n" );
    captureParams.boUserSuppliedMemoryUsed = 1;
    // find out the size of the resulting buffer by requesting a dummy request
    setPropI( captureParams.hRequestControl_Mode, ircmTrial, 0 );
    DMR_ImageRequestSingle( captureParams.hDrv, 0, 0 );
    // waitFor will return as fast as possible. No 'real' image will be taken
    // but a request object that contains a dummy image with the format, dimensions
    // and other information will be returned, that is (apart from the pixel data)
    // similar to any 'real' image that would be captured with the current settings
    result = DMR_ImageRequestWaitFor( captureParams.hDrv, -1, 0, &requestNr );
    if( result == DMR_NO_ERROR )
    {
        // check if the request contains a valid image
        result = DMR_GetImageRequestResultEx( hDrv, requestNr, &ReqRes, sizeof( ReqRes ), 0, 0 );
        if( ( result == DMR_NO_ERROR ) && ( ReqRes.result == rrOK ) )
        {
            // obtain the buffer size needed in the current configuration
            bufferSize = getPropI( captureParams.ppRequests[requestNr]->hImageSize_, 0 ) + getPropI( captureParams.ppRequests[requestNr]->hImageFooterSize_, 0 );
            /// switch back to 'normal' capture mode
            setPropI( captureParams.hRequestControl_Mode, ircmManual, 0 );
            // unlock this request to make it usable for the driver again
            DMR_ImageRequestUnlock( captureParams.hDrv, requestNr );
            result = createCaptureBuffers( &captureParams, bufferSize, getPropI( captureParams.hCaptureBufferAlignment, 0 ) );
            if( result != 0 )
            {
                printf( "An error occurred while setting up the user supplied buffers(error code: %s).\n", DMR_ErrorCodeToString( result ) );
                freeRequests( &captureParams );
                END_APPLICATION;
            }
        }
        else
        {
            printf( "Internal error(Request result: %08x! This should not happen an is a driver fault! Unable to continue.\n", ReqRes.result );
            freeRequests( &captureParams );
            END_APPLICATION;
        }
    }
    else
    {
        printf( "Internal error! This should not happen an is a driver fault! Unable to continue.\n" );
        freeRequests( &captureParams );
        END_APPLICATION;
    }

    captureLoop( &captureParams );
    //=============================================================================
    //========= unregister user supplied buffers again ============================
    //=============================================================================
    freeCaptureBuffers( &captureParams );

    //=============================================================================
    //========= Capture loop into memory managed by the driver again (default) ====
    //=============================================================================
    captureParams.boUserSuppliedMemoryUsed = 0;
    printf( "The device will try to capture continuously into memory automatically allocated be the device driver again.\n" );
    printf( "This is the default behaviour.\n" );
    captureLoop( &captureParams );

    //=============================================================================
    //========= Capture into a specific buffer managed by the user (advanced) =====
    //=============================================================================
    // by default the driver will decide which request will be used for an acquisition
    // requested by the user. However sometimes it can be necessary to make sure that a
    // certain request object will be used...
    printf( "Now the device will try to capture one frame into a specific user supplied buffer.\n" );
    pUserSuppliedBuffer = UserSuppliedHeapBuffer_Alloc( bufferSize, getPropI( captureParams.hCaptureBufferAlignment, 0 ) );
    // we want to use request number 'REQUEST_TO_USE' (zero based) for this acquisition thus we have to make sure
    // that there are at least 'REQUEST_TO_USE + 1' requests
    if( getPropI( captureParams.hRequestCount, 0 ) <= REQUEST_TO_USE )
    {
        setPropI( captureParams.hRequestCount, REQUEST_TO_USE + 1, 0 );
        allocateRequests( &captureParams );
        if( captureParams.requestCount <= 0 )
        {
            printf( "Couldn't allocate requests! Unable to continue!\n" );
            UserSuppliedHeapBuffer_Free( pUserSuppliedBuffer );
            END_APPLICATION;
        }
    }
    // associate a user supplied buffer with this request
    result = DMR_ImageRequestConfigure( captureParams.hDrv, REQUEST_TO_USE, 0, 0 );
    if( result != DMR_NO_ERROR )
    {
        printf( "An error occurred while setting request number %d in configuration mode: %s.\n", REQUEST_TO_USE, DMR_ErrorCodeToString( result ) );
        printf( "Press [ENTER] to end the continuous acquisition.\n" );
        getchar();
    }
    if( captureParams.ppRequests[REQUEST_TO_USE] == 0 )
    {
        printf( "An error occurred when accessing the request to use" );
        freeRequests( &captureParams );
        freeCaptureBuffers( &captureParams );
        UserSuppliedHeapBuffer_Free( pUserSuppliedBuffer );
        END_APPLICATION;
    }
    setPropI( captureParams.ppRequests[REQUEST_TO_USE]->hImageMemoryMode_, rimmUser, 0 );
    setPropP( captureParams.ppRequests[REQUEST_TO_USE]->hImageData_, pUserSuppliedBuffer->pBufAligned_, 0 );
    setPropI( captureParams.ppRequests[REQUEST_TO_USE]->hImageSize_, pUserSuppliedBuffer->bufSize_, 0 );

    if( ( result = DMR_ImageRequestUnlock( captureParams.hDrv, captureParams.ppRequests[REQUEST_TO_USE]->nr_ ) ) != DMR_NO_ERROR )
    {
        printf( "An error occurred while unlocking request number %d: %s.\n", captureParams.ppRequests[REQUEST_TO_USE]->nr_, DMR_ErrorCodeToString( result ) );
        freeRequests( &captureParams );
        freeCaptureBuffers( &captureParams );
        UserSuppliedHeapBuffer_Free( pUserSuppliedBuffer );
        END_APPLICATION;
    }

    // define that 'REQUEST_TO_USE' is used for the next acquisition
    setPropI( captureParams.hRequestControl_RequestToUse, REQUEST_TO_USE, 0 );
    // and capture the image
    requestUsed = INVALID_ID;
    result = DMR_ImageRequestSingle( captureParams.hDrv, 0, &requestUsed );
    if( result != DMR_NO_ERROR )
    {
        printf( "An error occurred while requesting an image for request number %d: (%s).\n", REQUEST_TO_USE, DMR_ErrorCodeToString( result ) );
        printf( "Press [ENTER] to end the continuous acquisition.\n" );
        freeRequests( &captureParams );
        freeCaptureBuffers( &captureParams );
        UserSuppliedHeapBuffer_Free( pUserSuppliedBuffer );
        END_APPLICATION;
    }
    if( requestUsed != REQUEST_TO_USE )
    {
        printf( "ERROR! An acquisition into buffer %d was requested, but the driver did use %d for this acquisition.\n", REQUEST_TO_USE, requestUsed );
    }
    manuallyStartAcquisitionIfNeeded( hDrv );
    result = DMR_ImageRequestWaitFor( captureParams.hDrv, -1, 0, &requestNr );
    manuallyStopAcquisitionIfNeeded( hDrv );
    if( result == DMR_NO_ERROR )
    {
        // check if the request contains a valid image
        result = DMR_GetImageRequestResultEx( hDrv, requestNr, &ReqRes, sizeof( ReqRes ), 0, 0 );
        if( ( result == DMR_NO_ERROR ) && ( ReqRes.result == rrOK ) )
        {
            if( ( result = DMR_GetImageRequestBuffer( hDrv, requestNr, &pIB ) ) == DMR_NO_ERROR )
            {
#ifdef USE_MV_DISPLAY_LIB
                // display the captured image
                mvDispSetImageFromImageBuffer( captureParams.pDisp, pIB );
                mvDispUpdate( captureParams.pDisp );
#else
                printf( "Frame captured into request number %d(%dx%d).\n", requestNr, pIB->iWidth, pIB->iHeight );
#endif // #ifdef USE_MV_DISPLAY_LIB
            }
            else
            {
                printf( "DMR_GetImageRequestBuffer: ERROR! Code %d\n", result );
            }
        }
        else
        {
            printf( "Acquisition into a specific buffer was not successful. Request result: 0x%08x.\n", ReqRes.result );
        }
    }
    else
    {
        printf( "Waiting for a frame captured into a specific buffer failed: %s.\n", DMR_ErrorCodeToString( result ) );
    }
    UserSuppliedHeapBuffer_Free( pUserSuppliedBuffer );

#ifdef USE_MV_DISPLAY_LIB
    mvDispWindowDestroy( captureParams.hDisp );
#endif // #ifdef USE_MV_DISPLAY_LIB
    freeRequests( &captureParams );
    freeCaptureBuffers( &captureParams );
    printf( "DMR_ReleaseImageRequestBufferDesc: %s.\n", DMR_ErrorCodeToString( DMR_ReleaseImageRequestBufferDesc( &pIB ) ) );
    printf( "DMR_CloseDevice: %s\n", DMR_ErrorCodeToString( DMR_CloseDevice( hDrv, hDevice ) ) );
    printf( "DMR_Close: %s\n", DMR_ErrorCodeToString( DMR_Close() ) );
    END_APPLICATION;
}
