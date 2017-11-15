//-----------------------------------------------------------------------------
#ifndef STLHelperH
#define STLHelperH STLHelperH
//-----------------------------------------------------------------------------
#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <utility>

//=============================================================================
//==================== std::pair related stuff ================================
//=============================================================================
//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
const _Ty1& GetFirst( const std::pair<_Ty1, _Ty2>& data )
//-----------------------------------------------------------------------------
{
    return data.first;
}

//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
const _Ty2& GetSecond( const std::pair<_Ty1, _Ty2>& data )
//-----------------------------------------------------------------------------
{
    return data.second;
}

//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
void DeleteFirst( std::pair<_Ty1, _Ty2>& data )
//-----------------------------------------------------------------------------
{
    delete data.first;
    data.first = 0;
}

//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
void DeleteSecond( std::pair<_Ty1, _Ty2>& data )
//-----------------------------------------------------------------------------
{
    delete data.second;
    data.second = 0;
}

//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
class ContainsFirst : public std::unary_function<std::pair<_Ty1, _Ty2>, bool>
//-----------------------------------------------------------------------------
{
    std::map<_Ty1, _Ty2> m_;
public:
    explicit ContainsFirst( const std::map<_Ty1, _Ty2>& m ) : m_( m ) {}
    bool operator()( const std::pair<_Ty1, _Ty2>& x )
    {
        return m_.find( x.first ) != m_.end();
    }
};

//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
class FirstMatches : public std::unary_function<std::pair<_Ty1, _Ty2>, bool>
//-----------------------------------------------------------------------------
{
    std::pair<_Ty1, _Ty2> value_;
    FirstMatches<_Ty1, _Ty2>& operator=( const FirstMatches<_Ty1, _Ty2>& ); // do not allow assignments
public:
    explicit FirstMatches( const std::pair<_Ty1, _Ty2>& val ) : value_( val ) {}
    bool operator()( const std::pair<_Ty1, _Ty2>& x )
    {
        return x.first == value_.first;
    }
};

//-----------------------------------------------------------------------------
template<class _Ty1, class _Ty2>
class SecondMatches : public std::unary_function<std::pair<_Ty1, _Ty2>, bool>
//-----------------------------------------------------------------------------
{
    std::pair<_Ty1, _Ty2> value_;
    SecondMatches<_Ty1, _Ty2>& operator=( const SecondMatches<_Ty1, _Ty2>& );   // do not allow assignments
public:
    explicit SecondMatches( const std::pair<_Ty1, _Ty2>& val ) : value_( val ) {}
    bool operator()( const std::pair<_Ty1, _Ty2>& x )
    {
        return x.second == value_.second;
    }
};

//-----------------------------------------------------------------------------
template<typename _Ty1, typename _Ty2>
bool SecondSmaller( const std::pair<_Ty1, _Ty2>& a, const std::pair<_Ty1, _Ty2>& b )
//-----------------------------------------------------------------------------
{
    if( a.second < b.second )
    {
        return true;
    }

    if( a.second > b.second )
    {
        return false;
    }

    return ( a.first < b.first );
}

//=============================================================================
//==================== various ================================================
//=============================================================================
//-----------------------------------------------------------------------------
template<class _Ty>
void DeleteElement( _Ty& data )
//-----------------------------------------------------------------------------
{
    delete data;
    data = 0;
}

//-----------------------------------------------------------------------------
template<class _Ty>
void DeleteArrayElement( _Ty& data )
//-----------------------------------------------------------------------------
{
    delete [] data;
    data = 0;
}

//-----------------------------------------------------------------------------
/// This could in theory be accomplished with a call to the for_each template:
///
/// for_each( s.begin(), s.end(), ptr_fun(DeleteElement<_Ty*>) );
///
/// But some ports of the STL always return const_iterators for sets (e.g. gcc 4.1.2)
template<class _Ty>
void ClearSetWithHeapAllocatedKeys( std::set<_Ty>& s )
//-----------------------------------------------------------------------------
{
    typename std::set<_Ty>::iterator it = s.begin();
    typename std::set<_Ty>::iterator itEnd = s.end();
    while( it != itEnd )
    {
        delete *it;
        ++it;
    }
    s.clear();
}

//-----------------------------------------------------------------------------
template<class T>
class Sum : public std::unary_function<T, void>
//-----------------------------------------------------------------------------
{
    T result_;
public:
    explicit Sum( T i = 0 ) : result_( i ) {}
    void operator()( T x )
    {
        result_ += x;
    }
    const T& result( void ) const
    {
        return result_;
    }
};

//-----------------------------------------------------------------------------
/// \brief Assigns a new value to a variable when this objects goes out of scope.
///
/// Can be useful if a variable must be set to a defined value at the end of a
/// code block that might rise an exception.
template<class T>
class VarScopeMod
//-----------------------------------------------------------------------------
{
    T& var_;
    T valAtEndOfScope_;
    VarScopeMod( const VarScopeMod& );              // do not allow copy constructor
    VarScopeMod& operator=( const VarScopeMod& );   // do not allow assignments
public:
    explicit VarScopeMod( T& var, T valAtEndOfScope ) : var_( var ), valAtEndOfScope_( valAtEndOfScope ) {}
    explicit VarScopeMod( T& var, T valWithinScope, T valAtEndOfScope ) : var_( var ), valAtEndOfScope_( valAtEndOfScope )
    {
        var = valWithinScope;
    }
    ~VarScopeMod()
    {
        var_ = valAtEndOfScope_;
    }
};

//-----------------------------------------------------------------------------
template<class T>
void removeDuplicates( T& container )
//-----------------------------------------------------------------------------
{
    std::sort( container.begin(), container.end() );
    typename T::iterator it = std::unique( container.begin(), container.end() );
    container.erase( it, container.end() );
}

#endif // STLHelperH
