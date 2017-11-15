//-----------------------------------------------------------------------------
#include "CaptureThread.h"
#include "PropData.h"
#include "PropViewCallback.h"
#include "PropViewFrame.h"
#include <wx/string.h>

using namespace mvIMPACT::acquire;

//=============================================================================
//================= Implementation PropViewCallback ===========================
//=============================================================================
//-----------------------------------------------------------------------------
void PropViewCallback::execute( Component& c, void* /* pUserData */ )
//-----------------------------------------------------------------------------
{
    if( pApp_ )
    {
        wxString info;
        if( pApp_->DetailedInfosOnCallback() )
        {
            info = wxT( " Detailed info: " );
            PropData::AppendComponentInfo( c, info, c.changedCounter(), c.changedCounterAttr() );
            info.Append( wxT( "." ) );
        }
        wxString featureValue;
        if( c.isProp() )
        {
            Property p( c );
            featureValue = wxString::Format( wxT( " Its current value is %s." ), ConvertedString( p.readS() ).c_str() );
        }
        wxCommandEvent* pE = new wxCommandEvent( featureChangedCallbackReceived, pApp_->GetId() );
        const wxString msg( wxString::Format( wxT( "Component %s has changed.%s%s\n" ), ConvertedString( c.name() ).c_str(), featureValue.c_str(), info.c_str() ) );
        pE->SetString( msg.c_str() ); // a deep copy of the string must be created here! See documentation of 'wxQueueEvent' for details.
        QueueEvent( pApp_->GetEventHandler(), pE );
    }
}
