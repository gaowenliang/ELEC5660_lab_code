#if !defined(linux) && !defined(__linux) && !defined(__linux__)
#   error Sorry! Linux only code!
#endif // #if !defined(linux) && !defined(__linux) && !defined(__linux__)
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <errno.h>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

using namespace std;
using namespace mvIMPACT::acquire;


#define PRESS_A_KEY             \
    getchar();

//-----------------------------------------------------------------------------
class ThreadParameter
//-----------------------------------------------------------------------------
{
    Device*         m_pDev;
    volatile bool   m_boTerminateThread;
public:
    ThreadParameter( Device* pDev ) : m_pDev( pDev ), m_boTerminateThread( false ) {}
    Device* device( void ) const
    {
        return m_pDev;
    }
    bool    terminated( void ) const
    {
        return m_boTerminateThread;
    }
    void    terminateThread( void )
    {
        m_boTerminateThread = true;
    }
};

//-----------------------------------------------------------------------------
static unsigned int thread_func( void* pData )
//-----------------------------------------------------------------------------
{
    ThreadParameter* pThreadParameter = reinterpret_cast<ThreadParameter*>( pData );
    unsigned int cnt = 0;

    try
    {
        pThreadParameter->device()->open();
    }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while opening the device " << pThreadParameter->device()->serial.read()
             << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread." << endl
             << "Press [ENTER] to end the application..."
             << endl;
        PRESS_A_KEY
        return 0;
    }

    // establish access to the statistic properties
    Statistics statistics( pThreadParameter->device() );
    // create an interface to the device found
    FunctionInterface fi( pThreadParameter->device() );

    // pre-fill the capture queue. There can be more than 1 queue for some device, but for this sample
    // we will work with the default capture queue. If a device supports more than one capture or result
    // queue, this will be stated in the manual. If nothing is mentioned about it, the device supports one
    // queue only. Request as many images as possible. If there are no more free requests 'DEV_NO_FREE_REQUEST_AVAILABLE'
    // will be returned by the driver.
    int result = DMR_NO_ERROR;
    SystemSettings ss( pThreadParameter->device() );
    const int REQUEST_COUNT = ss.requestCount.read();
    for( int i = 0; i < REQUEST_COUNT; i++ )
    {
        result = fi.imageRequestSingle();
        if( result != DMR_NO_ERROR )
        {
            cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString( result ) << endl;
        }
    }

    // run thread loop
    const Request* pRequest = 0;
    const unsigned int timeout_ms = 8000;   // USB 1.1 on an embedded system needs a large timeout for the first image
    int requestNr = INVALID_ID;
    // This next comment is valid once we have a display:
    // we always have to keep at least 2 images as the display module might want to repaint the image, thus we
    // can't free it unless we have a assigned the display to a new buffer.
    int lastRequestNr = INVALID_ID;
    while( !pThreadParameter->terminated() )
    {
        // wait for results from the default capture queue
        requestNr = fi.imageRequestWaitFor( timeout_ms );
        if( fi.isRequestNrValid( requestNr ) )
        {
            pRequest = fi.getRequest( requestNr );
            if( pRequest->isOK() )
            {
                ++cnt;
                // here we can display some statistical information every 100th image
                if( cnt % 100 == 0 )
                {
                    cout << "Info from " << pThreadParameter->device()->serial.read()
                         << ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
                         << ", " << statistics.errorCount.name() << ": " << statistics.errorCount.readS()
                         << ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS() << endl;
                }
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
            // additional information under TDMR_ERROR in the interface reference (
            cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ", device " << pThreadParameter->device()->serial.read() << ")"
                 << ", timeout value too small?" << endl;
        }
    }

    // free the last potentially locked request
    if( fi.isRequestNrValid( requestNr ) )
    {
        fi.imageRequestUnlock( requestNr );
    }
    // clear the request queue
    fi.imageRequestReset( 0, 0 );
    return 0;
}

//-----------------------------------------------------------------------------
static void* liveThread( void* pData )
//-----------------------------------------------------------------------------
{
    thread_func( pData );
    return NULL;
}

//-----------------------------------------------------------------------------
int main( int /*argc*/, char* /*argv*/[] )
//-----------------------------------------------------------------------------
{
    DeviceManager devMgr;
    const unsigned int devCnt = devMgr.deviceCount();
    if( devCnt == 0 )
    {
        cout << "No MATRIX VISION device found! Unable to continue!" << endl;
        return 0;
    }
    pthread_t* pHandles = new pthread_t[devCnt];
    pthread_attr_t* pAttrs = new pthread_attr_t[devCnt];
    // store all device infos in a vector
    // and start the execution of a 'live' thread for each device.
    vector<ThreadParameter*> threadParams;
    for( unsigned int i = 0; i < devCnt; i++ )
    {
        threadParams.push_back( new ThreadParameter( devMgr[i] ) );
        cout << devMgr[i]->family.read() << "(" << devMgr[i]->serial.read() << ")" << endl;
    }

    // start live threads
    for( unsigned int j = 0; j < devCnt; j++ )
    {
        pthread_attr_init( &pAttrs[j] );
        // you can set the stack size like this: pthread_attr_setstacksize (&pAttrs[j], 1024*1024);
        pthread_create( &pHandles[j], &pAttrs[j], liveThread, ( void* )threadParams[j] );
    }

    // now all threads will start running...
    cout << "Press return to end the acquisition( the initialisation of the devices might take some time )" << endl;
    PRESS_A_KEY

    // stop all threads again
    cout << "Terminating live threads..." << endl;
    size_t vSize = threadParams.size();
    for( unsigned int k = 0; k < vSize; k++ )
    {
        cout << "Terminating thread " << k << "." << endl;
        threadParams[k]->terminateThread();
    }

    // wait until each live thread has terminated.
    for( unsigned int j = 0; j < devCnt; j++ )
    {
        cout << "Waiting for thread " << j << " to terminate." << endl;
        pthread_join ( pHandles[j], NULL );
    }
    cout << "All capture threads terminated." << endl;

    // free resources
    for( unsigned int l = 0; l < vSize; l++ )
    {
        delete threadParams[l];
        cout << "thread parameter " << l << " removed." << endl;
    }
    delete [] pHandles;
    cout << "All thread handles removed." << endl;

    delete [] pAttrs;
    cout << "All thread atributes removed." << endl;
    return 0;
}
