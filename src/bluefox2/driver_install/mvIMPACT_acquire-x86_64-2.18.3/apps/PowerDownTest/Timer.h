#include <sys/time.h>

//-----------------------------------------------------------------------------
/// \brief Provides clock and time measurement functions
class CTime
//-----------------------------------------------------------------------------
{
private:
    struct  timeval tvStart;
    struct  timeval tvEnd;
    long    tvToMSec( const struct timeval tv );
    long    tvToUSec( const struct timeval tv );

public:
    /// \brief Constructs a new time measurement object
    ///
    /// This constructs the new object and starts a new measurement.
    ///
    /// \sa
    /// <b>CTime::start()</b>
    CTime();
    /// \brief Starts a new time measurement.
    ///
    /// Call <b>CTime::elapsed()</b> to get the timespan in sec. after the last call
    /// to <b>CTime::start()</b>.
    void    start( void );
    /// \brief Returns the elapsed time in sec.
    ///
    /// Returns the elapsed time in sec. after the last call to <b>CTime::start()</b>
    double  elapsed( void );
    /// \brief Returns the elapsed time in sec. and sets this time to the current time
    ///
    /// Returns the elapsed time in sec. after the last call to <b>CTime::start()</b>
    /// or <b>CTime::restart()</b> and sets this time to the current time.
    double  restart( void );
};

