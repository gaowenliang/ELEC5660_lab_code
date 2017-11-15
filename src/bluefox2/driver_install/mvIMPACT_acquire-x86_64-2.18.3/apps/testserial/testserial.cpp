//---------------------------------------------------------------------------
/*!
\xmlonly
Network/Communication
\endxmlonly

\brief Simple test program for the serial port.
*/

//---------------------------------------------------------------------------
#if !defined(linux) && !defined(__linux) && !defined(__linux__)
#   error Sorry! Linux only code!
#endif // #if !defined(linux) && !defined(__linux) && !defined(__linux__)
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <string>
#include <sstream>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>

#define PROG_PREFIX "testserial: "
#ifndef _strarg
#   define _strarg( var ) do { if( argc-- > 1 )     var = *argv++; } while(0)
#endif  // _strarg
#ifndef _skiparg
#   define _skiparg() argv++
#endif  // _skiparg

using namespace std;

//---------------------------------------------------------------------------
/* returns 0 if timeout, else 1 */
unsigned waitForInput ( int maxwait_sec, int iFd )
//---------------------------------------------------------------------------
{
    fd_set rfds;
    struct timeval tv;

    FD_ZERO( &rfds );
    FD_SET( iFd, &rfds );

    tv.tv_sec = maxwait_sec ;
    tv.tv_usec = 0;

    return select( iFd + 1, &rfds, NULL, NULL, &tv );
}

//-----------------------------------------------------------------------------
int setSerial( int iFd, const unsigned long ulBaudrate = B115200, bool bBreakDetect = false, bool bHardwareHandshake = false )
//-----------------------------------------------------------------------------
{
    struct termios stOldTio, stNewTio;

    /// Save current serial port settings.
    tcgetattr( iFd, &stOldTio );

    /// Clear struct for new port settings.
    bzero( &stNewTio, sizeof( stNewTio ) );

    /// BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
    /// CS8     : 8n1 (8bit,no parity,1 stopbit)
    /// CLOCAL  : local connection, no modem contol
    /// CREAD   : enable receiving characters
    stNewTio.c_cflag = ulBaudrate | CS8 | CLOCAL | CREAD;
    //  stNewTio.c_cflag = ulBaudrate|CS8|CLOCAL|CSTOPB|CREAD;

    /// CRTSCTS : output hardware flow control (only used if the cable has
    ///           all necessary lines. See sect. 7 of Serial-HOWTO)
    if ( bHardwareHandshake )
    {
        stNewTio.c_cflag |= CRTSCTS;
    }

    /// IGNPAR  : ignore bytes with parity errors
    /// ICRNL   : map CR to NL (otherwise a CR input on the other computer
    ///           will not terminate input)
    /// otherwise make device raw (no other input processing)
    //  stNewTio.c_iflag = IGNPAR|ICRNL;    //&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|IXON);       //= 0;
    //  stNewTio.c_iflag = 0;   //&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|IXON);       //= 0;
    if ( bBreakDetect )
    {
        stNewTio.c_iflag |= PARMRK;
    }
    else
    {
        stNewTio.c_iflag = 0;
    }
    //  stNewTio.c_iflag |= IGNPAR;

    /// Raw output.
    stNewTio.c_oflag = 0;//~OPOST;      // = 0;

    /// ICANON  : enable canonical input
    ///          disable all echo functionality, and don't send signals to calling program
    stNewTio.c_lflag = 0;   //&= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN); // = 0;
    //  stNewTio.c_lflag = ICANON;

    /// Initialize all control characters
    /// default values can be found in /usr/include/termios.h, and are given
    /// in the comments, but we don't need them here.
    stNewTio.c_cc[VINTR]    = 0;     /// 003, ETX, 'Ctrl-C', or also 0177, DEL, rubout
    stNewTio.c_cc[VQUIT]    = 0;     /// 034, FS, 'Ctrl-\'
    stNewTio.c_cc[VERASE]   = 0;     /// 0177, DEL, rubout, or 010, BS, Ctrl-H, or also #
    stNewTio.c_cc[VKILL]    = 0;     /// 025, NAK, 'Ctrl-U', or 'Ctrl-X', or also @
    stNewTio.c_cc[VEOF]     = 4;     /// 004, EOT, 'Ctrl-D'
    stNewTio.c_cc[VTIME]    = 0;     /// inter-character timer unused
    stNewTio.c_cc[VMIN]     = 1;     /// blocking read until 1 character arrives
    //stNewTio.c_cc[VSWTC]    = 0;     /// '\0'
    stNewTio.c_cc[VSTART]   = 0;     /// 021, DC1, 'Ctrl-Q'
    stNewTio.c_cc[VSTOP]    = 0;     /// 023, DC3, 'Ctrl-S'
    stNewTio.c_cc[VSUSP]    = 0;     /// 032, SUB, 'Ctrl-Z'
    stNewTio.c_cc[VEOL]     = 0;     /// 0, NUL
    stNewTio.c_cc[VREPRINT] = 0;     /// 022, DC2, 'Ctrl-R'
    //stNewTio.c_cc[VDISCARD] = 0;     /// 'Ctrl-u'
    stNewTio.c_cc[VWERASE]  = 0;     /// 027, ETB, 'Ctrl-W'
    stNewTio.c_cc[VLNEXT]   = 0;     /// 026, SYN, 'Ctrl-V'
    stNewTio.c_cc[VEOL2]    = 0;     /// 0, NUL

    /// Now clean the modem line and activate the settings for the port.
    tcflush( iFd, TCIOFLUSH );
    tcsetattr( iFd, TCSANOW, &stNewTio );

    return 0;
}

//---------------------------------------------------------------------------
void writeToDevice( int fd, const void* pBuf, size_t bufSize, const char* pDeviceName )
//---------------------------------------------------------------------------
{
    ssize_t ret = write( fd, pBuf, bufSize );
    if( ret < 0 )
    {
        fprintf( stdout, " == Writing to device >%s< failed. Result: %d(%s).\n", pDeviceName, errno, strerror( errno ) );
    }
    fsync( fd );
    fprintf( stdout, "%d bytes written to %s ....\n", static_cast<int>( bufSize ), pDeviceName );
}

//---------------------------------------------------------------------------
int main( int argc, char* argv[] )
//---------------------------------------------------------------------------
{
    char* pTmp = 0;
    _skiparg();
    _strarg( pTmp );
    string sDevice = string( "/dev/ttyS0" );
    if( pTmp )
    {
        sDevice = string( pTmp );
    }

    fprintf( stdout, " ++ " PROG_PREFIX "start " PROG_PREFIX __DATE__ " / " __TIME__ "\n\n" );
    fprintf( stdout, " == Try open: >%s< ....\n", sDevice.c_str() );
    int iFd = open( sDevice.c_str(), O_RDWR );
    if( iFd == -1 )
    {
        perror( "serial" );
        return 1;
    }

    if( setSerial( iFd ) < 0 )
    {
        fprintf( stderr, "*** Error: Can not set up the serial communication!!\n" );
        return 0;
    }

    fprintf( stdout, " == Device >%s< opened! Please check the outputs in  serial terminal!\n", sDevice.c_str() );
    string sStrData = string( "\n\nOpening serial port.\nPlease enter text and newline...\n" );
    writeToDevice( iFd, sStrData.c_str(), sStrData.size(), sDevice.c_str() );

    const size_t LOOP_COUNT = 20;
    for( size_t c = 0; c < LOOP_COUNT; c++ )
    {
        if( waitForInput( 1, iFd ) > 0 )
        {
            const size_t BUFFER_SIZE = 100;
            char pInputBuffer[BUFFER_SIZE];
            int iInsize = read( iFd, pInputBuffer, BUFFER_SIZE );
            if( iInsize > 0 )
            {
                string sInput = string( pInputBuffer );
                string sOutput = string( "You entered: " ) + sInput + string( "\n\n" );
                fprintf( stdout, "%s", sOutput.c_str() );
                writeToDevice( iFd, sOutput.c_str(), sOutput.size(), sDevice.c_str() );
                break;
            }
            else
            {
                fprintf( stdout, "*** No data read!!\n" );
            }
        }
        else
        {
            ostringstream oss;
            oss << "Still waiting.... " << ( LOOP_COUNT - c ) << " sec\n";
            if( c == ( LOOP_COUNT - 2 ) )
            {
                oss.str( "" );
                oss << "Last chance!!\n";
            }
            writeToDevice( iFd, oss.str().c_str(), oss.str().size(), sDevice.c_str() );
        }
    }

    sStrData = string( "Closing serial port.\n\n" );
    writeToDevice( iFd, sStrData.c_str(), sStrData.size(), sDevice.c_str() );
    fprintf( stdout, " == Closing: >%s<\n", sDevice.c_str() );
    close( iFd );

    fprintf( stdout, " -- " PROG_PREFIX "exit " PROG_PREFIX __DATE__ " / " __TIME__ "\n" );
    return 0;
}
