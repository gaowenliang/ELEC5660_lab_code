#include <apps/Common/exampleHelper_C.h>
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
#   include <string.h>
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

#define BUF_SIZE (512) // should always be enough, but could be done nicer by checking if buffer to small is returned from a function and then perform a re-allocation...

static int s_boTerminated = 0;

//-----------------------------------------------------------------------------
typedef struct CaptureParameter
//-----------------------------------------------------------------------------
{
    HDRV    hDrv;
#ifdef USE_MV_DISPLAY_LIB
    HDISP   hDisp;
#endif // #ifdef USE_MV_DISPLAY_LIB
} CaptureParameter;

//-----------------------------------------------------------------------------
unsigned int LIVE_LOOP_CALL liveLoop( void* pData )
//-----------------------------------------------------------------------------
{
#ifdef USE_MV_DISPLAY_LIB
    TDisp*          pDisp;
#endif // #ifdef USE_MV_DISPLAY_LIB
    HDRV            hDrv = INVALID_ID;
    TDMR_ERROR      result;
    ImageBuffer*    pIB;
    RequestResult   requestResult;
    int             frameCount;
    HOBJ            hPropFPS = INVALID_ID;
    double          fps;
    HOBJ            hErrorCount = INVALID_ID;
    int             errorCount;
    int             requestNr;
    int             lastRequestNr;

    hDrv = ( ( CaptureParameter* )pData )->hDrv;
#ifdef USE_MV_DISPLAY_LIB
    pDisp = mvDispWindowGetDisplayHandle( ( ( CaptureParameter* )pData )->hDisp );
#endif // #ifdef USE_MV_DISPLAY_LIB
    pIB = 0;
    frameCount = 0;
    fps = 0.0;
    requestNr = -1;
    // we always have to keep at least 2 images as the display module might want to repaint the image, thus we
    // cannot free it unless we have a assigned the display to a new buffer.
    lastRequestNr = -1;

    // try to locate the frames per second property
    if( ( ( hPropFPS = getStatisticProp( hDrv, "FramesPerSecond" ) ) == INVALID_ID ) ||
        ( ( hErrorCount = getStatisticProp( hDrv, "ErrorCount" ) ) == INVALID_ID ) )
    {
        printf( "Unable to continue!\n" );
        return 0;
    }

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
        // please note, that the value stored in the property 'ImageRequestTimeout_ms' specifies the
        // maximum time a request will remain in the queue. If no complete image has been taken until
        // then, RequestResult.result will contain 'rrTimeout', so to allow long wait times, this
        // property needs to be modified as well, as its default is 2000 ms.
        // In this sample this can be achieved by calling 'getSettingProp( hDrv, "Base", "ImageRequestTimeout_ms" )'
        result = DMR_ImageRequestWaitFor( hDrv, 500, 0, &requestNr );
        if( result == DMR_NO_ERROR )
        {
            // check if the request contains a valid image
            result = DMR_GetImageRequestResultEx( hDrv, requestNr, &requestResult, sizeof( requestResult ), 0, 0 );
            if( ( result == DMR_NO_ERROR ) && ( requestResult.result == rrOK ) )
            {
                // display statistical information every 100th image
                frameCount = frameCount + 1;
                if( ( result = DMR_GetImageRequestBuffer( hDrv, requestNr, &pIB ) ) == DMR_NO_ERROR )
                {
                    if( ( frameCount % 100 ) == 0 )
                    {
                        OBJ_GetF( hPropFPS, &fps, 0 );
                        OBJ_GetI( hErrorCount, &errorCount, 0 );
                        printf( "Frames per second: %.5f, Width: %d, Height: %d, Error count: %d.\n", fps, pIB->iWidth, pIB->iHeight, errorCount );
                    }
#ifdef USE_MV_DISPLAY_LIB
                    // display the captured image
                    mvDispSetImageFromImageBuffer( pDisp, pIB );
                    mvDispUpdate( pDisp );
#endif // #ifdef USE_MV_DISPLAY_LIB
                    // do your processing here
                }
                else
                {
                    printf( "DMR_GetImageRequestBuffer failed(code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
                }
            }
            else
            {
                // this can happen e.g. when a triggered acquisition timed out (missing trigger signal)
                // A request does not remain in the queue forever, but is removed after the max. queue time has elapsed. This timeout
                // is defined by the 'ImageRequestTimeout_ms' property. If this timeout has elapsed and no
                // image has been captured, the RequestResult.result parameter will not contain 'rrOK', but the
                // request still needs to be unlocked for the driver as it has been returned to the user.
                printf( "DMR_GetImageRequestResult: ERROR! Return value: %d(%s), request result: %d.\n", result, DMR_ErrorCodeToString( result ), requestResult.result );
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
            printf( "DMR_ImageRequestWaitFor failed(code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
        }
#ifdef linux
        s_boTerminated = waitForInput( 0, STDOUT_FILENO ) == 0 ? 0 : 1; // break by STDIN
        if( s_boTerminated == 1 )
        {
            // remove the '\n' from the stream
            fgetc( stdin );
        }
#endif // #ifdef linux
    }
    manuallyStopAcquisitionIfNeeded( hDrv );

#ifdef USE_MV_DISPLAY_LIB
    // stop the display from showing freed memory
    mvDispSetImage( pDisp, 0, 0, 0, 0, 0 );
#endif // #ifdef USE_MV_DISPLAY_LIB
    // free the last potentially locked request
    if( requestNr >= 0 )
    {
        DMR_ImageRequestUnlock( hDrv,  requestNr );
    }
    // clear the request queue
    if ( ( result = DMR_ImageRequestReset( hDrv, 0, 0 ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_ImageRequestReset(code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
    }
    // extract and unlock all requests that are now returned as 'aborted'
    while( DMR_ImageRequestWaitFor( hDrv, 0, 0, &requestNr ) == DMR_NO_ERROR )
    {
        DMR_ImageRequestUnlock( hDrv, requestNr );
    }
    if ( ( result = DMR_ReleaseImageRequestBufferDesc( &pIB ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_ReleaseImageRequestBufferDesc(code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
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
// This function will allow to select devices that support the GenICam interface
// layout(these are devices, that claim to be compliant with the GenICam standard)
// and that are bound to drivers that support the user controlled start and stop
// of the internal acquisition engine. Other devices will not be listed for
// selection as the code of the example relies on these features in the code.
unsigned int isDeviceSupportedBySample( const HDEV hDev )
//-----------------------------------------------------------------------------
{
    unsigned int dictValCount = 0;
    unsigned int i = 0;
    unsigned int result = 0;
    int* dictVals = NULL;
    HOBJ hPropInterfaceLayout = getDeviceProp( hDev, "InterfaceLayout" );

    if( ( hPropInterfaceLayout == INVALID_ID ) ||
        ( getDeviceProp( hDev, "AcquisitionStartStopBehaviour" ) == INVALID_ID ) )
    {
        return 0;
    }

    if( readTranslationDictValuesI( hPropInterfaceLayout, &dictVals, &dictValCount, 1 ) != PROPHANDLING_NO_ERROR )
    {
        return 0;
    }

    for( ; i < dictValCount; i++ )
    {
        if( dictVals[i] == dilGenICam )
        {
            result = 1;
            break;
        }
    }
    free( dictVals );
    return result;
}

//-----------------------------------------------------------------------------
void setToMaxAOI( HOBJ hPropWidth, HOBJ hPropHeight )
//-----------------------------------------------------------------------------
{
    if( isFeatureWriteable( hPropWidth ) )
    {
        setPropI64( hPropWidth, getPropI64( hPropWidth, PROP_MAX_VAL ), 0 );
    }
    if( isFeatureWriteable( hPropHeight ) )
    {
        setPropI64( hPropHeight, getPropI64( hPropHeight, PROP_MAX_VAL ), 0 );
    }
}

//-----------------------------------------------------------------------------
void setToQuarterAOI( HOBJ hPropWidth, HOBJ hPropHeight )
//-----------------------------------------------------------------------------
{
    if( isFeatureWriteable( hPropWidth ) )
    {
        setPropI64( hPropWidth, getPropI64( hPropWidth, PROP_MAX_VAL ) / 2LL , 0 );
    }
    if( isFeatureWriteable( hPropHeight ) )
    {
        setPropI64( hPropHeight, getPropI64( hPropHeight, PROP_MAX_VAL ) / 2LL, 0 );
    }
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    TDMR_ERROR       result = DMR_NO_ERROR;
    HDMR             hDMR = INVALID_ID;
    HDRV             hDrv = INVALID_ID;
    HDEV             hDevice = INVALID_ID;
    HOBJ             hPropWidth = INVALID_ID;
    HOBJ             hPropHeight = INVALID_ID;
    CaptureParameter captureParameter;

    // get rid of warnings
    argc = argc;
    argv = argv;

    // try to initialise the library.
    if( ( result = DMR_Init( &hDMR ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_Init failed (code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
        END_APPLICATION;
    }

    // select a GenICam device
    getDeviceFromUserInput( &hDevice, isDeviceSupportedBySample, 1 );

    // try to initialise this device
    if( ( result = DMR_OpenDevice( hDevice, &hDrv ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_OpenDevice failed (code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
        result = DMR_Close();
        printf( "DMR_Close: (code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
        END_APPLICATION;
    }

    // try to get access to the mandatory 'Width' and 'Height' properties
    hPropHeight = getSettingProp( hDrv, "Base", "Height" );
    hPropWidth = getSettingProp( hDrv, "Base", "Width" );

#ifdef USE_MV_DISPLAY_LIB
    // create a window to display the captured images
    captureParameter.hDisp = mvDispWindowCreate( "ContinuousCaptureGenICam sample(plain 'C')" );
    mvDispWindowShow( captureParameter.hDisp );
#endif // #ifdef USE_MV_DISPLAY_LIB
    captureParameter.hDrv = hDrv;

    setToQuarterAOI( hPropWidth, hPropHeight );
    captureLoop( &captureParameter );
    setToMaxAOI( hPropWidth, hPropHeight );
    captureLoop( &captureParameter );

#ifdef USE_MV_DISPLAY_LIB
    mvDispWindowDestroy( captureParameter.hDisp );
#endif // #ifdef USE_MV_DISPLAY_LIB

    if( ( result = DMR_CloseDevice( hDrv, hDevice ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_CloseDevice: (code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
    }
    if( ( result = DMR_Close() ) != DMR_NO_ERROR )
    {
        printf( "DMR_Close: (code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
    }
    END_APPLICATION;
}
