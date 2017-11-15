#ifdef _MSC_VER // is Microsoft compiler?
#   if _MSC_VER < 1300  // is 'old' VC 6 compiler?
#       pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#   endif // #if _MSC_VER < 1300
#endif // #ifdef _MSC_VER
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>
#ifdef _WIN32
#   include <windows.h> // for 'Sleep'
#elif defined(linux) || defined(__linux) || defined(__linux__)
#   include <unistd.h> // for 'usleep'
#else
#   error unsupported target platform
#endif // #ifdef _WIN32
#include <apps/Common/exampleHelper.h>
#include <common/minmax.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

using namespace std;
using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;

//-----------------------------------------------------------------------------
string getStringFromCIN( void )
//-----------------------------------------------------------------------------
{
    cout << endl << ">>> ";
    string cmd;
    cin >> cmd;
    // remove the '\n' from the stream
    std::cin.get();
    return cmd;
}

//-----------------------------------------------------------------------------
int getIntFromCIN( void )
//-----------------------------------------------------------------------------
{
    return atoi( getStringFromCIN().c_str() );
}

//-----------------------------------------------------------------------------
int getHEXFromCIN( void )
//-----------------------------------------------------------------------------
{
    int result = 0;
    sscanf( getStringFromCIN().c_str(), "%i", &result );
    return result;
}

//-----------------------------------------------------------------------------
template<typename _Ty>
void hexToCOUT( const _Ty& param )
//-----------------------------------------------------------------------------
{
    cout.setf( std::ios::hex, std::ios::basefield );
    cout << "0x" << param;
    cout.unsetf( std::ios::hex );
}

//-----------------------------------------------------------------------------
void modifySyncOutput( SyncOutput* p )
//-----------------------------------------------------------------------------
{
    displayPropertyData( p->frequency_Hz );
    modifyPropertyValue( p->frequency_Hz );
    displayPropertyData( p->lowPart_pc );
    modifyPropertyValue( p->lowPart_pc );
}

//-----------------------------------------------------------------------------
void displayCommonIOFeatures( IOSubSystem& ioss )
//-----------------------------------------------------------------------------
{
    // display available features
    cout << "This device has" << endl;

    const unsigned int inputCount = ioss.getInputCount();
    cout << "  " << inputCount << " digital input(s)" << endl;
    // display the state and name of each individual digital input
    for( unsigned int i = 0; i < inputCount; i++ )
    {
        cout << "   [" << i << "]: " << ioss.input( i )->getDescription() << "(current state: " << ioss.input( i )->get() << ")" << endl;
    }
    cout << endl;

    if( inputCount > 0 )
    {
        // read the state of all digital inputs in a single function call
        cout << "All input registers can be queried with a single function call: Calling 'readInputRegister' returned ";
        hexToCOUT( ioss.readInputRegister() );
        cout << endl;
        cout << "From the LSB to the MSB a '1' in this result indicates, that this input is currently connected to a signal" << endl
             << "that is interpreted as a logial '1'. E.g. 0x13 indicates that inputs 0, 1 and 4 are currently in 'high' state." << endl;
    }

    const unsigned int outputCount = ioss.getOutputCount();
    cout << "  " << outputCount << " digital output(s)" << endl;
    if( outputCount > 0 )
    {
        unsigned int readOnlyAccessMask = 0;
        bool boRun = true;
        while( boRun )
        {
            // display the state and name of each individual digital output
            for( unsigned int j = 0; j < outputCount; j++ )
            {
                DigitalOutput* pOutput = ioss.output( j );
                cout << "   [" << j << "]: " << pOutput->getDescription() << "(current state: " << pOutput->get() << ", " << ( pOutput->isWriteable() ? "" : "NOT " ) << "manually switchable)" << endl;
                if( !pOutput->isWriteable() )
                {
                    readOnlyAccessMask |= 1 << j;
                }
            }
            cout << endl;
            cout << "Enter the number of a digital output followed by [ENTER] to modify its state or 'c' followed by [ENTER] to continue." << endl;
            string cmd( getStringFromCIN() );
            if( cmd == "c" )
            {
                boRun = false;
                continue;
            }
            const unsigned int index = static_cast<unsigned int>( atoi( cmd.c_str() ) );
            if( ( index >= outputCount ) || !isdigit( cmd[0] ) )
            {
                cout << "Invalid selection" << endl;
                continue;
            }

            DigitalOutput* pOutput = ioss.output( index );
            if( !pOutput->isWriteable() )
            {
                cout << pOutput->getDescription() << " is not manually switchable." << endl;
                continue;
            }
            cout << "Please enter the number in front of the function that shall be called followed by [ENTER]:" << endl;
            cout << "  [0]: set" << endl
                 << "  [1]: reset" << endl
                 << "  [2]: flip" << endl;
            const int newMode = getIntFromCIN();
            switch( newMode )
            {
            case 0:
                pOutput->set();
                break;
            case 1:
                pOutput->reset();
                break;
            case 2:
                pOutput->flip();
                break;
            default:
                cout << "Invalid selection." << endl;
                break;
            }
        }

        // read the state of all digital outputs in a single function call
        cout << "All output registers can be queried with a single function call." << endl
             << endl
             << "From the LSB to the MSB a '1' in this result indicates, that this output is currently switched to 'high' or 'active' state" << endl
             << endl
             << "E.g. 0x22 indicates that outputs 1 and 5 (zero-based) are currently in 'high' state." << endl;
        const unsigned int fullOutputMask = bitMask( outputCount );
        boRun = true;
        while( boRun )
        {
            cout << "Calling 'readOutputRegister' returned ";
            hexToCOUT( ioss.readOutputRegister() );
            cout << endl;
            cout << "Please enter 'y' followed by [ENTER] to modify all digital outputs with a single function" << endl
                 << "call or anything else followed by [ENTER] to continue." << endl;
            if( getStringFromCIN() != "y" )
            {
                boRun = false;
                continue;
            }
            cout << "Please enter the bitmask in hex that contains the new values for the digital outputs followed by [ENTER]: ";
            unsigned int value = static_cast<unsigned int>( getHEXFromCIN() );
            if( value & ~fullOutputMask )
            {
                value &= fullOutputMask;
                cout << "WARNING: More bits than outputs specified. Bitmask truncated to ";
                hexToCOUT( value );
                cout << endl;
            }
            cout << "Please enter the bitmask in hex that contains '1's for outputs that shall be affected by this operation followed by [ENTER]: ";
            const unsigned int mask = static_cast<unsigned int>( getHEXFromCIN() );
            if( readOnlyAccessMask & mask )
            {
                cout << "WARNING: At least one selected output is not manually switchable: Mask: ";
                hexToCOUT( mask );
                cout << ", read-only access mask: ";
                hexToCOUT( readOnlyAccessMask );
                cout << endl;
                cout << "No digital outputs have been modified." << endl
                     << endl;
                continue;
            }
            ioss.writeOutputRegister( value, mask );
        }
    }

    cout << "This device also has" << endl;
    cout << "  " << ioss.RTCtrProgramCount() << " hardware real-time controller(s)." << endl
         << endl;
    if( ioss.RTCtrProgramCount() > 0 )
    {
        cout << "How to program the HRTC (Hardware RealTime Controller) is not part of this sample, but the manual will contain a separate chapter on this topic.";
    }

    cout << "  " << ioss.getPulseStartConfigurationCount() << " pulse start configuration(s)." << endl
         << endl;
}

//-----------------------------------------------------------------------------
void mvGenICamIOAccess( Device* pDev )
//-----------------------------------------------------------------------------
{
    try
    {
        DigitalIOControl dioc( pDev );

        //Count the number of Digital IOs
        const unsigned int IOCount = dioc.lineSelector.dictSize();
        bool boRun = true;
        while( boRun )
        {
            cout << endl;
            cout << "       This device has " << IOCount << " DigitalIOs" << endl;
            cout << " ------------------------------------------" << endl;
            // display information about each individual DigitalIO
            vector<int64_type> validLineSelectorValues;
            dioc.lineSelector.getTranslationDictValues( validLineSelectorValues );
            const vector<int64_type>::size_type cnt = validLineSelectorValues.size();
            for( vector<int64_type>::size_type i = 0; i < cnt; i++ )
            {
                dioc.lineSelector.write( validLineSelectorValues[i] );
                cout << " IO " << validLineSelectorValues[i] << ": \t Type: " << dioc.lineMode.readS() << " \t Current state: " << ( dioc.lineStatus.read() ? "ON" : "OFF" ) << endl;
            }
            cout << " ------------------------------------------" << endl;

            cout << endl;
            cout << "Please enter a valid line number followed by [ENTER]:" << endl;
            cout << "- if it is an output its value will be inverted" << endl;
            cout << "- if it is an input its value will be polled continuously for 10 seconds" << endl;
            cout << "- or enter 'c' followed by [ENTER] to continue:" << endl;
            string cmd( getStringFromCIN() );
            if( cmd == "c" )
            {
                boRun = false;
                continue;
            }
            const unsigned int index = static_cast<unsigned int>( atoi( cmd.c_str() ) );
            if( ( index >= IOCount ) || !isdigit( cmd[0] ) )
            {
                cout << "Invalid selection" << endl;
                continue;
            }

            //Define the IO we are interested in
            dioc.lineSelector.write( index );
            //check whether selected IO is an Output or an Input
            if( dioc.lineMode.readS() == "Output" )
            {
                dioc.lineInverter.write( dioc.lineInverter.read() ? bFalse : bTrue );
            }
            else if( dioc.lineMode.readS() == "Input" )
            {
                cout << endl
                     << endl;
                cout << " ------------------------------------------" << endl;
                cout << "           Polling Input '" << dioc.lineSelector.readS() << "'" << endl;
                cout << " ------------------------------------------" << endl;
                cout << endl;
                for( int i = 0; i < 100; i++ )
                {
                    cout << "\r    Value:" << ( dioc.lineStatus.read() ? "ON" : "OFF" );
#ifdef _WIN32
                    Sleep( 100 );
#else
                    usleep( 1000 * 100 );
#endif // #ifdef _WIN32
                    cout << "\t    Remaining Time:"
                         << fixed << setprecision( 1 )
                         << ( 10. - ( static_cast<double>( i + 1 ) / 10. ) );
                    cout.flush();
                }
                cout << endl
                     << endl;
            }
            else
            {
                cout << "IO " << index << " is a '" << dioc.lineMode.readS() << "'!" << endl;
            }
        }
    }
    catch( const ImpactAcquireException& e )
    {
        cout << endl;
        cout << " An mvIMPACT Acquire Exception occurred:" << e.getErrorCodeAsString() << endl;
        cout << endl;
    }
}

//-----------------------------------------------------------------------------
void mvBlueFOXIOAccess( Device* pDev )
//-----------------------------------------------------------------------------
{
    IOSubSystemBlueFOX ioss( pDev );
    displayCommonIOFeatures( ioss );
    if( ioss.digitalInputThreshold.isValid() )
    {
        cout << "This device also supports the '" << ioss.digitalInputThreshold.name() << "' property." << endl;
        displayPropertyData( ioss.digitalInputThreshold );
    }
    CameraSettingsBlueFOX cs( pDev );
    cout << "To use a digital output in 'expose active' mode, the property '" << cs.flashMode.name() << "' can be used." << endl
         << "If a delay between switching the output and starting the frame exposure is needed, this can be achieved by " << endl
         << "writing to the property '" << cs.flashToExposeDelay_us.name() << "'." << endl;
}

//-----------------------------------------------------------------------------
void frameGrabberIOAccess( Device* pDev )
//-----------------------------------------------------------------------------
{
    IOSubSystemFrameGrabber ioss( pDev );
    displayCommonIOFeatures( ioss );

    const unsigned int HDOutputCount = ioss.getHDOutputCount();
    const unsigned int VDOutputCount = ioss.getVDOutputCount();
    if( ( HDOutputCount > 0 ) || ( VDOutputCount > 0 ) )
    {
        cout << "This device also offers" << endl;
        bool boRun = true;
        while( boRun )
        {
            cout << "  " << HDOutputCount << " HD output(s)" << endl;
            for( unsigned int i = 0; i < HDOutputCount; i++ )
            {
                SyncOutput* p = ioss.HDOutput( i );
                cout << "   [" << i << "]: " << p->getDescription() << "(" << p->frequency_Hz << ", " << p->lowPart_pc << ")" << endl;
            }
            cout << endl;

            cout << "  " << VDOutputCount << " VD output(s)" << endl;
            for( unsigned int j = 0; j < VDOutputCount; j++ )
            {
                SyncOutput* p = ioss.VDOutput( j );
                cout << "   [" << j << "]: " << p->getDescription() << "(" << p->frequency_Hz << ", " << p->lowPart_pc << ")" << endl;
            }
            cout << "HD and VD output signals are currently switched to '" << ioss.syncOutputMode.readS() << "' mode." << endl;

            cout << "Please enter the number in front of the function that shall be called followed by [ENTER] or 'c' followed by [ENTER] to continue:" << endl;
            cout << "  [0]: modify the properties a HD output" << endl
                 << "  [1]: modify the properties a VD output" << endl
                 << "  [2]: modify the " << ioss.syncOutputMode.name() << " property." << endl;
            string cmd( getStringFromCIN() );
            if( cmd == "c" )
            {
                boRun = false;
                continue;
            }

            unsigned int cmdIndex = static_cast<unsigned int>( atoi( cmd.c_str() ) );
            if( ( cmdIndex > 2 ) || !isdigit( cmd[0] ) )
            {
                cout << "Invalid selection" << endl;
                continue;
            }
            switch( cmdIndex )
            {
            case 0:
                {
                    cout << "Please enter the index of the HD-output you want to modify followed by [ENTER]: ";
                    string cmd( getStringFromCIN() );
                    unsigned int index = static_cast<unsigned int>( atoi( cmd.c_str() ) );
                    if( ( index >= HDOutputCount ) || !isdigit( cmd[0] ) )
                    {
                        cout << "Invalid selection" << endl;
                        continue;
                    }
                    modifySyncOutput( ioss.HDOutput( index ) );
                }
                break;
            case 1:
                {
                    cout << "Please enter the index of the VD-output you want to modify followed by [ENTER]: ";
                    string cmd( getStringFromCIN() );
                    unsigned int index = static_cast<unsigned int>( atoi( cmd.c_str() ) );
                    if( ( index >= VDOutputCount ) || !isdigit( cmd[0] ) )
                    {
                        cout << "Invalid selection" << endl;
                        continue;
                    }
                    modifySyncOutput( ioss.VDOutput( index ) );
                }
                break;
            case 2:
                displayPropertyData( ioss.syncOutputMode );
                modifyPropertyValue( ioss.syncOutputMode );
            default:
                break;
            }
        }
    }
    else
    {
        cout << "HD and VD output signals are not supported by this device." << endl;
    }

    const unsigned int outputCount = ioss.getOutputCount();
    if( outputCount > 0 )
    {
        // show how to define and undefine certain output signals for frame grabber devices
        OutputSignalGeneratorFrameGrabber osg( pDev );
    }
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

    if( pDev->interfaceLayout.isValid() && ( pDev->interfaceLayout.read() == dilGenICam ) )
    {
        mvGenICamIOAccess( pDev );
    }
    else if( pDev->family.read() == "mvBlueFOX" )
    {
        mvBlueFOXIOAccess( pDev );
    }
    else if( pDev->deviceClass.read() == dcFrameGrabber )
    {
        frameGrabberIOAccess( pDev );
    }
    else
    {
        cout << "Device " << pDev->serial.read() << "(" << pDev->product << ") is not supported by this sample" << endl;
        cout << "Press [ENTER] to end the application" << endl;
        cin.get();
        return 0;
    }

    cout << "Press [ENTER] to end the application" << endl;
    cin.get();

    return 0;
}
