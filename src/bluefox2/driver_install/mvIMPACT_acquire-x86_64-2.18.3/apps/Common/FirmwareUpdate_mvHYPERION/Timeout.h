//-----------------------------------------------------------------------------
#ifndef TimeoutH
#define TimeoutH TimeoutH
//-----------------------------------------------------------------------------
#include <time.h>

//-----------------------------------------------------------------------------
class CTimeout
//-----------------------------------------------------------------------------
{
public:
    explicit        CTimeout( unsigned long timeout_ms );

    void            Start( const unsigned long timeout_ms );
    unsigned char   Elapsed( void ) const;
    void            SleepMS( const unsigned long ms ) const;
    unsigned long   Remain( void ) const;
private:
    clock_t elapse_period_;
    clock_t start_;
    clock_t end_;
};

#endif //TimeoutH
