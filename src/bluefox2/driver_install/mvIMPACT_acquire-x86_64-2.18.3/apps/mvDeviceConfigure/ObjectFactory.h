//-----------------------------------------------------------------------------
#ifndef ObjectFactoryH
#define ObjectFactoryH ObjectFactoryH
//-----------------------------------------------------------------------------
#include <map>
#include <utility>

//-----------------------------------------------------------------------------
template<class _PRODUCT, typename _IDENTIFIER, typename _CREATOR, typename _PARAM>
class ObjectFactory
//-----------------------------------------------------------------------------
{
    typedef std::map<_IDENTIFIER, _CREATOR> CallbackMap;
    CallbackMap callbacks_;
public:
    bool Register( const _IDENTIFIER& id, _CREATOR creator )
    {
        return callbacks_.insert( std::make_pair( id, creator ) ).second;
    }
    bool Unregister( const _IDENTIFIER& id )
    {
        return callbacks_.erase( id ) == 1;
    }
    _PRODUCT* CreateObject( const _IDENTIFIER& id, _PARAM param )
    {
        typename CallbackMap::const_iterator it = callbacks_.find( id );
        if( it == callbacks_.end() )
        {
            return 0;
        }
        return ( it->second )( param );
    }
};

#endif // ObjectFactoryH
