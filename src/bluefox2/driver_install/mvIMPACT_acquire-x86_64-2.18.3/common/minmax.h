//-----------------------------------------------------------------------------
#ifndef minmaxH
#define minmaxH minmaxH
//-----------------------------------------------------------------------------
#include <cassert>
#include <limits>
#include <stdexcept>

#undef min // otherwise we can't work with the 'numeric_limits' template here as Windows defines a macro 'min'
#undef max // otherwise we can't work with the 'numeric_limits' template here as Windows defines a macro 'max'

//-----------------------------------------------------------------------------
template<typename _Ty>
_Ty bitMask( _Ty bitCount )
//-----------------------------------------------------------------------------
{
    if( bitCount >= static_cast<_Ty>( sizeof( _Ty ) * 8 ) )
    {
        return _Ty( -1 );
    }
    return ( _Ty( 1 ) << bitCount ) - _Ty( 1 );
}

//-----------------------------------------------------------------------------
template<typename _Ty>
_Ty bitMask_s( _Ty bitCount )
//-----------------------------------------------------------------------------
{
    if( bitCount > static_cast<_Ty>( sizeof( _Ty ) * 8 ) )
    {
        throw std::invalid_argument( "Requested bit mask cannot be stored by target type" );
    }
    return bitMask( bitCount );
}

//---------------------------------------------------------------------------
template<typename _Ty>
bool isPowerOfTwo( _Ty val )
//---------------------------------------------------------------------------
{
    return ( ( val > 0 ) && ( ( val & ( val - 1 ) ) == 0 ) );
}

//-----------------------------------------------------------------------------
template<typename _Ty>
_Ty align( _Ty val, _Ty alignment )
//-----------------------------------------------------------------------------
{
    if( ( alignment <= 0 ) || ( !isPowerOfTwo( alignment ) ) )
    {
        throw std::invalid_argument( "align: Invalid alignment" );
    }
    return static_cast<_Ty>( ( val + alignment - 1 ) & ( bitMask( static_cast<_Ty>( sizeof( val ) * 8 ) ) - ( alignment - 1 ) ) );
}

//-----------------------------------------------------------------------------
template<typename _Ty>
bool isAligned( _Ty val, _Ty alignment )
//-----------------------------------------------------------------------------
{
    if( ( alignment <= 0 ) || ( !isPowerOfTwo( alignment ) ) )
    {
        throw std::invalid_argument( "isAligned: Invalid alignment" );
    }
    return ( ( ( alignment - 1 ) & val ) == 0 );
}

//-----------------------------------------------------------------------------
template<typename _Ty, typename _Result>
inline _Result getBit( _Ty shift )
//-----------------------------------------------------------------------------
{
    if( shift > static_cast<_Result>( ( sizeof( _Result ) * 8 ) ) )
    {
        throw std::invalid_argument( "shift value too large for this data type" );
    }
    return static_cast<_Result>( _Result( 1 ) << _Result( shift ) );
}

//-----------------------------------------------------------------------------
template<typename _Ty>
bool isInRange( const _Ty& lowerLimit, const _Ty& upperLimit, const _Ty& start, const _Ty& end, _Ty* pOverlappingRangeStart = 0, _Ty* pOverlappingRangeEnd = 0 )
//-----------------------------------------------------------------------------
{
    if( end < lowerLimit )
    {
        return false;
    }

    if( start > upperLimit )
    {
        return false;
    }

    if( pOverlappingRangeStart )
    {
        *pOverlappingRangeStart = ( start > lowerLimit ) ? start : lowerLimit;
    }

    if( pOverlappingRangeEnd )
    {
        *pOverlappingRangeEnd = ( end > upperLimit ) ? upperLimit : end;
    }

    return true;
}

//-----------------------------------------------------------------------------
template<typename T>
#if (defined(_WIN32) || defined(WIN32) || defined(__WIN32__)) && (defined(_WIN64) || defined(WIN64) || defined(__WIN64__))
inline const T& saveAssign( const __unaligned T& val, const __unaligned T& min, const __unaligned T& max )
#else
inline const T& saveAssign( const T& val, const T& min, const T& max )
#endif
//-----------------------------------------------------------------------------
{
    return ( ( val > max ) ? max : ( val < min ) ? min : val );
}

//-----------------------------------------------------------------------------
template<typename _Result, typename _Ty>
_Result getClippedValue( _Ty val )
//-----------------------------------------------------------------------------
{
    if( val > static_cast<_Ty>( std::numeric_limits<_Result>::max() ) )
    {
        return std::numeric_limits<_Result>::max();
    }

    if( val < static_cast<_Ty>( std::numeric_limits<_Result>::min() ) )
    {
        return std::numeric_limits<_Result>::min();
    }

    return static_cast<_Result>( val );
}

#endif // minmaxH
