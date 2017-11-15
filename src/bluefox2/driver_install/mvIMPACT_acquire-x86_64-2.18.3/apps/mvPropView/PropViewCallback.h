//-----------------------------------------------------------------------------
#ifndef PropViewCallbackH
#define PropViewCallbackH PropViewCallbackH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

class PropViewFrame;

//-----------------------------------------------------------------------------
class PropViewCallback : public mvIMPACT::acquire::ComponentCallback
//-----------------------------------------------------------------------------
{
    PropViewFrame* pApp_;
public:
    explicit PropViewCallback( void* pUserData = 0 ) : mvIMPACT::acquire::ComponentCallback( pUserData ) {}
    void attachApplication( PropViewFrame* pApp )
    {
        pApp_ = pApp;
    }
    virtual void execute( mvIMPACT::acquire::Component& c, void* pUserData );
};

#endif // PropViewCallbackH
