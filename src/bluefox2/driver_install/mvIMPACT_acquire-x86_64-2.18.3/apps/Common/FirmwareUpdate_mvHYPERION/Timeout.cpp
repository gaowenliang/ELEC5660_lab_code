//-----------------------------------------------------------------------------
#include "Timeout.h"
#ifdef _WIN32
#   include <windows.h>
#else
#   include <unistd.h>
#endif // #ifdef _WIN32

//-----------------------------------------------------------------------------
CTimeout::CTimeout( unsigned long timeout_ms )
//-----------------------------------------------------------------------------
{
    Start( timeout_ms );
}

//-----------------------------------------------------------------------------
void CTimeout::Start( const unsigned long timeout_ms )
//-----------------------------------------------------------------------------
{
    start_ = clock();
    elapse_period_ = timeout_ms * CLOCKS_PER_SEC / 1000;
    if( elapse_period_ == 0 )
    {
        elapse_period_ = 1;
    }
    end_ = start_ + elapse_period_;
}

//-----------------------------------------------------------------------------
unsigned char CTimeout::Elapsed( void ) const
//-----------------------------------------------------------------------------
{
    return ( end_ < clock() );
}

//-----------------------------------------------------------------------------
void CTimeout::SleepMS( const unsigned long ms ) const
//-----------------------------------------------------------------------------
{
#ifdef _WIN32
    Sleep( ms );
#else
    usleep( ms * 1000 );
#endif // #ifdef _WIN32
}

//-----------------------------------------------------------------------------
unsigned long CTimeout::Remain( void ) const
//-----------------------------------------------------------------------------
{
    return Elapsed() ? 0 : ( ( clock() - end_ ) * 1000 ) / CLOCKS_PER_SEC;
}
