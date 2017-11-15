#if !defined(linux) && !defined(__linux) && !defined(__linux__)
#   error Sorry! Linux only code!
#endif // #if !defined(linux) && !defined(__linux) && !defined(__linux__)
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Overlay_Window.H>
#include <FL/fl_draw.H>
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

#ifdef MALLOC_TRACE
#   include <mcheck.h>
#endif  // MALLOC_TRACE

#define PRESS_A_KEY_AND_RETURN          \
    cout << "Press a key..." << endl;   \
    getchar(); \
    return 0;

#define AOI_WIDTH 640
#define AOI_HEIGHT 480

using namespace std;
using namespace mvIMPACT::acquire;

static bool s_boTerminated = false;
static bool s_boGreySensor = false;

//-----------------------------------------------------------------------------
class MyWindow : public Fl_Overlay_Window
//-----------------------------------------------------------------------------
{
    const Request* pRequest_;
    string msg_;
    virtual void draw()
    {
        if( pRequest_ )
        {
            fl_draw_image( ( unsigned char* ) pRequest_->imageData.read(), 0, 0,
                           pRequest_->imageWidth.read(), pRequest_->imageHeight.read(),
                           pRequest_->imageBytesPerPixel.read(),
                           pRequest_->imageLinePitch.read() );
        }
    }
    virtual void draw_overlay()
    {
        fl_color( FL_RED );
        fl_draw( msg_.c_str(), 8, h() - 20 );
    }
public:
    MyWindow( int W, int H ) : Fl_Overlay_Window( W, H ), pRequest_( 0 ), msg_( "" ) {}
    void NewRequest( const Request* pRequest )
    {
        pRequest_ = pRequest;
    }
    void setOverlayString( const string& msg )
    {
        msg_ = msg;
        redraw_overlay();
    }
};

//-----------------------------------------------------------------------------
MyWindow* createWindow( int width, int height )
//-----------------------------------------------------------------------------
{
    MyWindow* pWindow = new MyWindow( width, height );
    //pWindow->clear_border();
    fl_font( FL_TIMES, 12 );
    pWindow->end();
    Fl::visual( FL_RGB | FL_DOUBLE | FL_INDEX );
    return pWindow;
}

//-----------------------------------------------------------------------------
void windowCallback( Fl_Widget*, void* )
//-----------------------------------------------------------------------------
{
    printf( "Window was closed\n" );
    s_boTerminated = true;
}

//-----------------------------------------------------------------------------
void GenICamDeviceSetColorPixelFormat( Device* pDev )
//-----------------------------------------------------------------------------
{
    mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );

    std::string fmt_ = ifc.pixelColorFilter.readS();

    if( !fmt_.compare( "BayerRG" ) )
    {
        ifc.pixelFormat.writeS( "BayerRG8" );
    }
    else if( !fmt_.compare( "BayerGB" ) )
    {
        ifc.pixelFormat.writeS( "BayerGB8" );
    }
    else if( !fmt_.compare( "BayerGR" ) )
    {
        ifc.pixelFormat.writeS( "BayerGR8" );
    }
    else if( !fmt_.compare( "BayerBG" ) )
    {
        ifc.pixelFormat.writeS( "BayerBG8" );
    }
}

//-----------------------------------------------------------------------------
unsigned int liveLoop( Device* pDev, bool boStoreFrames, const string& settingName, int iWidth, int iHeight, bool boSingleShotMode )
//-----------------------------------------------------------------------------
{
    cout << " == " << __FUNCTION__ << " - establish access to the statistic properties...." << endl;
    // establish access to the statistic properties
    Statistics statistics( pDev );
    cout << " == " << __FUNCTION__ << " - create an interface to the device found...." << endl;
    // create an interface to the device found
    FunctionInterface fi( pDev );

    if( !settingName.empty() )
    {
        cout << "Trying to load setting " << settingName << "..." << endl;
        int result = fi.loadSetting( settingName );
        if( result != DMR_NO_ERROR )
        {
            cout << "loadSetting( \"" << settingName << "\" ); call failed: " << ImpactAcquireException::getErrorCodeAsString( result ) << endl;
        }
    }

    // depending on the device and its sensor, we set an appropriate output format for displaying
    mvIMPACT::acquire::ImageDestination id( pDev );

    if( !std::string( "mvBlueFOX" ).compare( pDev->family.readS() ) )
    {
        mvIMPACT::acquire::InfoBlueFOX ibf( pDev );
        if( !std::string( "BayerMosaic" ).compare( ibf.sensorColorMode.readS() ) )
        {
            id.pixelFormat.writeS( "BGR888Packed" );
        }
    }
    else if( !std::string( "mvBlueCOUGAR" ).compare( pDev->family.readS() ) )
    {
        mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );
        if( s_boGreySensor )
        {
            ifc.pixelFormat.writeS( "Mono8" );
        }
        else
        {
            GenICamDeviceSetColorPixelFormat( pDev );
            id.pixelFormat.writeS( "BGR888Packed" );
        }
    }
    else if( !std::string( "mvBlueLYNX" ).compare( pDev->family.readS() ) )
    {
        mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );
        if( s_boGreySensor )
        {
            ifc.pixelFormat.writeS( "Mono8" );
        }
        else
        {
            GenICamDeviceSetColorPixelFormat( pDev );
            id.pixelFormat.writeS( "BGR888Packed" );
        }
    }

    // Pre-fill the capture queue with ALL buffers currently available. In case the acquisition engine is operated
    // manually, buffers can only be queued when they have been queued before the acquisition engine is started as well.
    // Even though there can be more than 1, for this sample we will work with the default capture queue
    int requestResult = DMR_NO_ERROR;
    int requestCount = 0;

    if( boSingleShotMode )
    {
        fi.imageRequestSingle();
        ++requestCount;
    }
    else
    {
        while( ( requestResult = fi.imageRequestSingle() ) == DMR_NO_ERROR )
        {
            ++requestCount;
        }
    }

    if( requestResult != DEV_NO_FREE_REQUEST_AVAILABLE )
    {
        cout << "Last result: " << requestResult << "(" << ImpactAcquireException::getErrorCodeAsString( requestResult ) << "), ";
    }
    cout << requestCount << " buffers requested";
    SystemSettings ss( pDev );
    if( ss.requestCount.hasMaxValue() )
    {
        cout << ", max request count: " << ss.requestCount.getMaxValue();
    }
    cout << endl;

    cout << "Press <<ENTER>> to end the application!!" << endl;

    MyWindow* pWindow = createWindow( iWidth, iHeight );
    pWindow->show();
    pWindow->callback( windowCallback );

    manuallyStartAcquisitionIfNeeded( pDev, fi );
    // run thread loop
    const Request* pRequest = 0;
    const unsigned int timeout_ms = 8000;   // USB 1.1 on an embedded system needs a large timeout for the first image
    int requestNr = -1;
    bool boLoopRunning = true;
    unsigned int cnt = 0;
    while( boLoopRunning )
    {
        // wait for results from the default capture queue
        requestNr = fi.imageRequestWaitFor( timeout_ms );
        if( fi.isRequestNrValid( requestNr ) )
        {
            pRequest = fi.getRequest( requestNr );
            if( pRequest->isOK() )
            {
                ++cnt;
                // here we can display some statistical information every 20th image
                if( cnt % 20 == 0 )
                {

                    ostringstream out;
                    out << pDev->serial.read() << ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS();
                    pWindow->setOverlayString( out.str() );

                    cout << cnt << ": Info from " << pDev->serial.read()
                         << ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
                         << ", " << statistics.errorCount.name() << ": " << statistics.errorCount.readS()
                         << ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS()
                         << " Image count: " << cnt
                         << " (dimensions: " << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << ", format: " << pRequest->imagePixelFormat.readS();
                    if( pRequest->imageBayerMosaicParity.read() != bmpUndefined )
                    {
                        cout << ", " << pRequest->imageBayerMosaicParity.name() << ": " << pRequest->imageBayerMosaicParity.readS();
                    }
                    cout << ")" << endl;
                }
                // here we can store an image every 100th frame
                if( boStoreFrames && ( cnt % 100 == 0 ) )
                {
                    ostringstream oss;
                    oss << "Image" << cnt << "." << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << "." << pRequest->imagePixelFormat.readS();
                    if( pRequest->imageBayerMosaicParity.read() != bmpUndefined )
                    {
                        oss << "(BayerPattern=" << pRequest->imageBayerMosaicParity.readS() << ")";
                    }
                    oss << ".raw";
                    FILE* fp = fopen( oss.str().c_str(), "wb" );
                    if( fp )
                    {
                        unsigned char* pImageData = ( unsigned char* ) pRequest->imageData.read();
                        for( int h = 0; h < pRequest->imageHeight.read(); h++ )
                        {
                            // write one line
                            fwrite( pImageData, pRequest->imageWidth.read(), pRequest->imageBytesPerPixel.read(), fp );
                            // respect image line pitch
                            pImageData += pRequest->imageLinePitch.read();
                        }
                        fclose( fp );
                    }
                }

                // everything went well. Display the result
                pWindow->NewRequest( pRequest );
                pWindow->redraw();
                Fl::check();
            }
            else
            {
                cout << "*** Error: A request has been returned with the following result: " << pRequest->requestResult << endl;
            }

            // this image has been displayed thus the buffer is no longer needed...
            fi.imageRequestUnlock( requestNr );
            // send a new image request into the capture queue
            fi.imageRequestSingle();
            if( boSingleShotMode )
            {
                manuallyStartAcquisitionIfNeeded( pDev, fi );
            }
        }
        else
        {
            cout << "*** Error: Result of waiting for a finished request: " << requestNr << "("
                 << ImpactAcquireException::getErrorCodeAsString( requestNr ) << "). Timeout value too small?" << endl;
        }

        boLoopRunning = waitForInput( 0, STDOUT_FILENO ) == 0 ? true : false; // break by STDIN
        // Exit when window was cloesd
        if( s_boTerminated )
        {
            break;
        }
    }

    if( !boSingleShotMode )
    {
        manuallyStopAcquisitionIfNeeded( pDev, fi );
    }
    cout << " == " << __FUNCTION__ << " - free resources...." << endl;
    // free resources
    fi.imageRequestReset( 0, 0 );
    return 0;
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
#ifdef MALLOC_TRACE
    mtrace();
#endif  // MALLOC_TRACE

    cout << " ++ starting application...." << endl;

    bool boStoreFrames = false;
    string settingName;
    int width = -1;
    int height = -1;
    string pixelFormat;
    string acquisitionMode;
    string deviceSerial;
    int defaultRequestCount = -1;
    for( int i = 1; i < argc; i++ )
    {
        string arg( argv[i] );
        if( string( argv[i] ) == "-sf" )
        {
            boStoreFrames = true;
        }
        else if( arg.find( "-a" ) == 0 )
        {
            acquisitionMode = arg.substr( 2 );
        }
        else if( arg.find( "-drc" ) == 0 )
        {
            defaultRequestCount = atoi( arg.substr( 4 ).c_str() );
        }
        else if( arg.find( "-h" ) == 0 )
        {
            height = atoi( arg.substr( 2 ).c_str() );
        }
        else if( arg.find( "-s" ) == 0 )
        {
            deviceSerial = arg.substr( 2 );
        }
        else if( arg.find( "-w" ) == 0 )
        {
            width = atoi( arg.substr( 2 ).c_str() );
        }
        else
        {
            // try to load this setting later on...
            settingName = string( argv[1] );
        }
    }

    if( argc <= 1 )
    {
        cout << "Available command line parameters:" << endl
             << endl
             << "-sf to store every 100th frame in raw format" << endl
             << "-a<mode> to set the acquisition mode" << endl
             << "-h<height> to set the AOI width" << endl
             << "-s<serialNumber> to pre-select a certain device. If this device can be found no further user interaction is needed" << endl
             << "-w<width> to set the AOI width" << endl
             << "-drc<bufferCount> to specify the default request count" << endl
             << "any other string will be interpreted as a name of a setting to load" << endl;
    }

    DeviceManager devMgr;
    Device* pDev = 0;
    if( !deviceSerial.empty() )
    {
        pDev = devMgr.getDeviceBySerial( deviceSerial );
        if( pDev )
        {
            // if this device offers the 'GenICam' interface switch it on, as this will
            // allow are better control over GenICam compliant devices
            conditionalSetProperty( pDev->interfaceLayout, dilGenICam );
            // if this device offers a user defined acquisition start/stop behaviour
            // enable it as this allows finer control about the streaming behaviour
            conditionalSetProperty( pDev->acquisitionStartStopBehaviour, assbUser );
        }
    }
    if( !pDev )
    {
        // this will automatically set the interface layout etc. to the values from the branch above
        pDev = getDeviceFromUserInput( devMgr );
    }
    if( pDev == 0 )
    {
        cout << "Unable to continue!";
        PRESS_A_KEY_AND_RETURN
    }

    if( width <= 0 )
    {
        width = AOI_WIDTH;
    }

    if( height <= 0 )
    {
        height = AOI_HEIGHT;
    }

    // create an interface to the first MATRIX VISION device with the serial number sDevSerial
    if( pDev )
    {
        cout << "Initialising device: " << pDev->serial.read() << ". This might take some time..." << endl
             << "Using interface layout '" << pDev->interfaceLayout.readS() << "'." << endl;
        try
        {
            pDev->open();
            if( defaultRequestCount > 0 )
            {
                cout << "Setting default request count to " << defaultRequestCount << endl;
                pDev->defaultRequestCount.write( defaultRequestCount );
            }
            switch( pDev->interfaceLayout.read() )
            {
            case dilGenICam:
                {
                    mvIMPACT::acquire::GenICam::AcquisitionControl ac( pDev );
                    mvIMPACT::acquire::GenICam::ImageFormatControl ifc( pDev );

                    // calculations to crop to VGA size in the center of the sensor image
                    const int maxw = ifc.widthMax.read();
                    const int maxh = ifc.heightMax.read();
                    int xoff = ( maxw - width ) / 2;
                    int yoff = ( maxh - height ) / 2;

                    if( xoff < 0 )
                    {
                        xoff = 0;
                    }
                    if( yoff < 0 )
                    {
                        yoff = 0;
                    }

                    ifc.width.write( width );
                    ifc.height.write( height );
                    ifc.offsetX.write( xoff );
                    ifc.offsetY.write( yoff );

                    if( !acquisitionMode.empty() )
                    {
                        ac.acquisitionMode.writeS( acquisitionMode );
                    }

                    // check if it's a color sensor
                    mvIMPACT::acquire::GenICam::DeviceControl dc( pDev );
                    if( !std::string( "Grey" ).compare( dc.mvDeviceSensorColorMode.readS() ) )
                    {
                        s_boGreySensor = true;
                    }

                    acquisitionMode = ac.acquisitionMode.readS();
                    cout << "Device set up to " << ifc.width.read() << "x" << ifc.height.read() << endl;
                }
                break;
            case dilDeviceSpecific:
                {
                    mvIMPACT::acquire::CameraSettingsBase cs( pDev );

                    // calculations to crop to VGA size in the center of the sensor image
                    const int maxw = cs.aoiWidth.getMaxValue();
                    const int maxh = cs.aoiHeight.getMaxValue();
                    int xoff = ( maxw - width ) / 2;
                    int yoff = ( maxh - height ) / 2;

                    if( xoff < 0 )
                    {
                        xoff = 0;
                    }
                    if( yoff < 0 )
                    {
                        yoff = 0;
                    }

                    cs.aoiWidth.write( width );
                    cs.aoiHeight.write( height );
                    cs.aoiStartX.write( xoff );
                    cs.aoiStartY.write( yoff );

                    cout << "Device set up to " << cs.aoiWidth.read() << "x" << cs.aoiHeight.read() << endl;
                }
                break;
            default:
                break;
            }
        }
        catch( ImpactAcquireException& e )
        {
            // this e.g. might happen if the same device is already opened in another process...
            cout << "*** " << __FUNCTION__ << " - An error occurred while opening the device " << pDev->serial.read()
                 << "(error code: " << e.getErrorCode() << ", " << e.getErrorCodeAsString() << "). Press any key to end the application..." << endl;
            PRESS_A_KEY_AND_RETURN
        }

        // initialise display window
        // IMPORTANT: It's NOT safe to create multiple display windows in multiple threads!!!
        string windowTitle( "mvIMPACT_acquire sample, Device " + pDev->serial.read() );
        cout << "<<" << windowTitle << ">>" << endl;

        // start the execution of the 'live' loop.
        liveLoop( pDev, boStoreFrames, settingName, width, height, acquisitionMode == "SingleFrame" );
        cout << " == Will exit...." << endl;
        pDev->close();
    }
    cout << " -- ending application...." << endl;

    return 0;
}
