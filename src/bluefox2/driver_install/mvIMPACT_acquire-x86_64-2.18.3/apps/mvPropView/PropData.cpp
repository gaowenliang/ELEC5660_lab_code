//-----------------------------------------------------------------------------
#if defined(linux) || defined(__linux) || defined(__linux__)
#   include <sys/socket.h>
#   include <arpa/inet.h>
#   include <errno.h>
#else
#   include <winsock2.h>
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
#include "DataConversion.h"
#include <limits>
#include "PropTree.h"
#include "PropData.h"
#include "SpinEditorDouble.h"
#include "ValuesFromUserDlg.h"
#include <vector>
#include "wx/propgrid/advprops.h"
#include <wx/msgdlg.h>
#include <wx/settings.h>
#include <wx/stopwatch.h>
#include <wx/tokenzr.h>

#undef min // otherwise we can't work with the 'numeric_limits' template here as Windows defines a macro 'min'
#undef max // otherwise we can't work with the 'numeric_limits' template here as Windows defines a macro 'max'

using namespace std;
using namespace mvIMPACT::acquire;

//=============================================================================
//============== Implementation helper functions ==============================
//=============================================================================
//-----------------------------------------------------------------------------
template<typename _Ty>
wxString GetIPv4AddressAsString( _Ty prop, const int index )
//-----------------------------------------------------------------------------
{
    const int value = static_cast<int>( prop.read( index ) );
    return wxString::Format( wxT( "%d.%d.%d.%d" ), ( value >> 24 ) & 0xFF,
                             ( value >> 16 ) & 0xFF,
                             ( value >> 8 ) & 0xFF,
                             value & 0xFF );
}

//-----------------------------------------------------------------------------
wxString MACAddressToString( int64_type MACAddress )
//-----------------------------------------------------------------------------
{
    return wxString::Format( wxT( "%02x:%02x:%02x:%02x:%02x:%02x" ),
                             static_cast<unsigned char>( ( MACAddress >> 40 ) & 0xFF ),
                             static_cast<unsigned char>( ( MACAddress >> 32 ) & 0xFF ),
                             static_cast<unsigned char>( ( MACAddress >> 24 ) & 0xFF ),
                             static_cast<unsigned char>( ( MACAddress >> 16 ) & 0xFF ),
                             static_cast<unsigned char>( ( MACAddress >> 8  ) & 0xFF ),
                             static_cast<unsigned char>( ( MACAddress       ) & 0xFF ) );
}

//-----------------------------------------------------------------------------
/// \brief This function expects an input in this format: 'AA:BB:CC:DD:EE:FF'
int64_type MACAddressFromString( const wxString& MAC )
//-----------------------------------------------------------------------------
{
    int64_type result = 0;
    wxStringTokenizer tokenizer( MAC, wxString( wxT( ":" ) ), wxTOKEN_STRTOK );
    vector<string> v;
    while( tokenizer.HasMoreTokens() )
    {
        v.push_back( string( tokenizer.GetNextToken().mb_str() ) );
    }

    const vector<string>::size_type cnt = v.size();
    if( cnt != 6 )
    {
        // invalid string format
        return 0;
    }
    for( vector<string>::size_type i = 0; i < cnt; i++ )
    {
        if( v[i].length() != 2 )
        {
            // invalid string format
            return 0;
        }
        if( v[i].find_first_not_of( "0123456789abcdefABCDEF" ) != string::npos )
        {
            // invalid string format
            return 0;
        }
    }
    for( vector<string>::size_type i = 0; i < cnt; i++ )
    {
        unsigned int val;
#if defined(_MSC_VER) && (_MSC_VER >= 1400) // is at least VC 2005 compiler?
        sscanf_s( v[i].c_str(), "%x", &val );
#else
        sscanf( v[i].c_str(), "%x", &val );
#endif // #if defined(_MSC_VER) && (_MSC_VER >= 1400)
        result = result | ( static_cast<int64_type>( val ) << ( 8 * ( cnt - 1 - i ) ) );
    }
    return result;
}

//=============================================================================
//========================= wxBinaryDataProperty ==============================
//=============================================================================
//-----------------------------------------------------------------------------
bool wxBinaryDataProperty::OnButtonClick( wxPropertyGrid* pPropGrid, wxString& value )
//-----------------------------------------------------------------------------
{
    BinaryDataDlg dlg( pPropGrid, GetLabel(), value );
    if( dlg.ShowModal() == wxID_OK )
    {
        value = dlg.GetBinaryData();
        return true;
    }
    return false;
}

#if wxUSE_VALIDATORS
//-----------------------------------------------------------------------------
wxValidator* wxBinaryDataProperty::GetClassValidator( void )
//-----------------------------------------------------------------------------
{
    WX_PG_DOGETVALIDATOR_ENTRY()
    // At least wxPython 2.6.2.1 required that the string argument is given
    static wxString v;
    HEXStringValidator* validator = new HEXStringValidator( &v );
    WX_PG_DOGETVALIDATOR_EXIT( validator )
}

//-----------------------------------------------------------------------------
wxValidator* wxBinaryDataProperty::DoGetValidator( void ) const
//-----------------------------------------------------------------------------
{
    return wxBinaryDataProperty::GetClassValidator();
}
#endif // #if wxUSE_VALIDATORS

//=============================================================================
//========================= PropData ==========================================
//=============================================================================
//-----------------------------------------------------------------------------
PropData::PropData( HOBJ hObj ) : m_GridItemId( wxNullProperty ), m_lastChangedCounter( numeric_limits<unsigned int>::max() ),
    m_lastChangedCounterAttr( numeric_limits<unsigned int>::max() ), m_pParentGridPage( 0 ),
    m_Component( hObj ), m_rootList( ROOT_LIST ) {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void PropData::AppendComponentInfo( mvIMPACT::acquire::Component comp, wxString& info, unsigned int actChangedCount, unsigned int actAttrChangedCount )
//-----------------------------------------------------------------------------
{
    ostringstream oss;
    vector<mvIMPACT::acquire::Component> selectingFeatures;
    comp.selectingFeatures( selectingFeatures );
    vector<mvIMPACT::acquire::Component> selectedFeatures;
    comp.selectedFeatures( selectedFeatures );
    if( !selectingFeatures.empty() || !selectedFeatures.empty() )
    {
        AppendSelectorInfo( oss, selectingFeatures );
        AppendSelectorInfo( oss, selectedFeatures );
    }
    oss << "[" << comp.visibilityAsString()[0] << "]";
    info.Append( ConvertedString( oss.str() ) );

    info.Append( wxString::Format( wxT( ", hObj: 0x%08x, cc(%d/%d), type: %s, flags: %s" ),
                                   comp.hObj(), actChangedCount, actAttrChangedCount,
                                   ConvertedString( comp.typeAsString() ).c_str(),
                                   ConvertedString( comp.flagsAsString() ).c_str() ) );

    if( comp.isProp() )
    {
        const Property prop( comp );
        if( prop.hasMinValue() )
        {
            info.Append( wxString::Format( wxT( ", min: %s" ), ConvertedString( prop.readS( plMinValue ) ).c_str() ) );
        }
        if( prop.hasMaxValue() )
        {
            info.Append( wxString::Format( wxT( ", max: %s" ), ConvertedString( prop.readS( plMaxValue ) ).c_str() ) );
        }
        if( prop.hasStepWidth() )
        {
            info.Append( wxString::Format( wxT( ", inc: %s" ), ConvertedString( prop.readS( plStepWidth ) ).c_str() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropData::AppendComponentInfo( wxString& info, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const
//-----------------------------------------------------------------------------
{
    AppendComponentInfo( m_Component, info, actChangedCount, actAttrChangedCount );
}

//-----------------------------------------------------------------------------
void PropData::AppendPropertyToGrid( wxPGProperty* pProp, wxPGProperty* pParentItem )
//-----------------------------------------------------------------------------
{
    m_GridItemId = pParentItem ? m_pParentGridPage->AppendIn( pParentItem, pProp ) : m_pParentGridPage->Append( pProp );
}

//-----------------------------------------------------------------------------
void PropData::AppendSelectorInfo( ostringstream& oss, const vector<mvIMPACT::acquire::Component>& v )
//-----------------------------------------------------------------------------
{
    const vector<mvIMPACT::acquire::Component>::size_type cnt = v.size();
    oss << "[";
    for( vector<mvIMPACT::acquire::Component>::size_type i = 0; i < cnt; i++ )
    {
        if( i > 0 )
        {
            oss << ", ";
        }
        oss << v[i].name();
    }
    oss << "]";
}

//-----------------------------------------------------------------------------
wxString PropData::BuildFullFeaturePath( void ) const
//-----------------------------------------------------------------------------
{
    wxString fullFeaturePath;
    ComponentIterator cit( m_Component );
    while( cit.isValid() && ( cit.hObj() != m_rootList ) )
    {
        cit = cit.parent();
        if( cit.isValid() )
        {
            fullFeaturePath.Prepend( wxString::Format( wxT( "%s/" ), ConvertedString( cit.name() ).c_str() ) );
        }
    }
    fullFeaturePath.Append( ConvertedString( m_Component.name() ) );
    return fullFeaturePath;
}

//-----------------------------------------------------------------------------
const wxColour& PropData::GetBackgroundColour( void ) const
//-----------------------------------------------------------------------------
{
    if( !m_Component.isVisible() )
    {
        return GlobalDataStorage::Instance()->GetPropGridColour( GlobalDataStorage::pgcInvisibleFeature );
    }

    if( !GlobalDataStorage::Instance()->IsComponentVisibilitySupported() ||
        ( m_Component.visibility() <= GlobalDataStorage::Instance()->GetComponentVisibility() ) )
    {
        return ( m_Component.isList() ? GlobalDataStorage::Instance()->GetPropGridColour( GlobalDataStorage::pgcListBackground ) : *wxWHITE );
    }

    switch( m_Component.visibility() )
    {
    case cvExpert:
        return GlobalDataStorage::Instance()->GetPropGridColour( GlobalDataStorage::pgcInvisibleExpertFeature );
    case cvGuru:
        return GlobalDataStorage::Instance()->GetPropGridColour( GlobalDataStorage::pgcInvisibleGuruFeature );
    case cvInvisible:
    default:
        break;
    }

    return GlobalDataStorage::Instance()->GetPropGridColour( GlobalDataStorage::pgcInvisibleFeature );
}

//-----------------------------------------------------------------------------
wxString PropData::GetDisplayName( EDisplayFlags flags ) const
//-----------------------------------------------------------------------------
{
    if( flags & dfDisplayNames )
    {
        const string displayName( m_Component.displayName() );
        return ConvertedString( displayName.empty() ? m_Component.name() : displayName );
    }
    return ConvertedString( m_Component.name() );
}

//-----------------------------------------------------------------------------
bool PropData::IsVisible( void ) const
//-----------------------------------------------------------------------------
{
    return ( m_Component.isVisible() &&
             ( !GlobalDataStorage::Instance()->IsComponentVisibilitySupported() || ( m_Component.visibility() <= GlobalDataStorage::Instance()->GetComponentVisibility() ) ) );
}

//-----------------------------------------------------------------------------
void PropData::UpdateGridItem( const PropTree* pPropTree, EDisplayFlags flags, bool* pboModified )
//-----------------------------------------------------------------------------
{
    const unsigned int actChangedCount = m_Component.changedCounter();
    const unsigned int actAttrChangedCount = m_Component.changedCounterAttr();

    const bool boModified = ( m_lastChangedCounter != actChangedCount ) || ( m_lastChangedCounterAttr != actAttrChangedCount );
    if( pboModified )
    {
        *pboModified = boModified;
    }

    bool boUpdateNow = m_GridItemId != m_pParentGridPage->GetGrid()->GetSelectedProperty();
    if( ( m_GridItemId == m_pParentGridPage->GetGrid()->GetSelectedProperty() ) &&
        ( IsWriteable() != m_pParentGridPage->IsPropertyEnabled( m_GridItemId ) ) )
    {
        // if the feature currently selected became read-only we need to kill the editor
        boUpdateNow = true;
    }
    if( !boUpdateNow && ( m_Component.type() & ctProp ) )
    {
        Property p( m_Component.hObj() );
        boUpdateNow = m_GridItemId->GetValueAsString() != ConvertedString( p.readS() );
    }
    // do not update the property currently selected and do also NOT update the
    // last changed counter in order to update the feature the next time the
    // update timer fires when this feature is no longer selected.
    if( boUpdateNow )
    {
        if( boModified )
        {
            Update( pPropTree, flags, actChangedCount, actAttrChangedCount );
            m_lastChangedCounter = m_Component.changedCounter();
            m_lastChangedCounterAttr = m_Component.changedCounterAttr();
        }

        if( flags & dfDisplayInvisibleComponents )
        {
            if( m_GridItemId->HasFlag( wxPG_PROP_HIDDEN ) )
            {
                m_pParentGridPage->HideProperty( m_GridItemId, false, wxPG_DONT_RECURSE );
            }
            const wxColour& backgroundColour = GetBackgroundColour();
            // Setting the background colour is rather expensive and can also result in focus losses for the current editor
            // so check if it is really necessary by reading the current colour first
            if( backgroundColour != m_pParentGridPage->GetPropertyBackgroundColour( m_GridItemId ) )
            {
                m_pParentGridPage->SetPropertyBackgroundColour( m_GridItemId, backgroundColour, wxPG_DONT_RECURSE );
            }
        }
        else
        {
            if( m_GridItemId->HasFlag( wxPG_PROP_HIDDEN ) && IsVisible() )
            {
                m_pParentGridPage->HideProperty( m_GridItemId, false, wxPG_DONT_RECURSE );
            }
            else if( !m_GridItemId->HasFlag( wxPG_PROP_HIDDEN ) && !IsVisible() )
            {
                m_pParentGridPage->HideProperty( m_GridItemId, true, wxPG_DONT_RECURSE );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropData::UpdateLabelAndHelpString( EDisplayFlags flags, wxString& label ) const
//-----------------------------------------------------------------------------
{
    const string docString( GetComponent().docString() );
    if( ( flags & dfDisplayDebugInfo ) && docString.empty() )
    {
        label.Prepend( wxT( "?" ) );
    }
    m_GridItemId->SetLabel( label );
    m_GridItemId->SetHelpString( ConvertedString( docString.empty() ? GetComponent().typeAsString() : docString ) );
}

//-----------------------------------------------------------------------------
void PropData::UpdateDefaultState( void ) const
//-----------------------------------------------------------------------------
{
    const bool boPropInGridIsDefault = !m_pParentGridPage->IsPropertyModified( m_GridItemId );
    const bool boPropInDriverIsDefault = GetComponent().isDefault();
    if( boPropInDriverIsDefault != boPropInGridIsDefault )
    {
        m_GridItemId->SetModifiedStatus( !boPropInDriverIsDefault );
    }
}

//=============================================================================
//========================= MethodObject ======================================
//=============================================================================
//-----------------------------------------------------------------------------
MethodObject::MethodObject( HOBJ hObj ) : PropData( hObj ), m_Params( wxT( "" ) ),
    m_FriendlyName( BuildFriendlyName( hObj ) )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
wxString MethodObject::BuildFriendlyName( HOBJ hObj )
//-----------------------------------------------------------------------------
{
    wxString friendlyName;
    try
    {
        Method m( hObj );
        string::size_type end = 0;
        string name( m.name() );
        if( ( end = name.find_first_of( "@" ) ) != string::npos )
        {
            name = name.substr( 0, end );
        }
        const string para_type_str = m.paramList();
        string::const_iterator it = para_type_str.begin();
        friendlyName = ConvertedString( charToType( *it++ ) );
        friendlyName += wxT( " " );
        friendlyName += ConvertedString( name );
        const string::const_iterator itEND = para_type_str.end();
        if( it == itEND )
        {
            friendlyName += wxT( "( void )" );
        }
        else
        {
            friendlyName += wxT( "( " );
            while( it != itEND )
            {
                friendlyName += ConvertedString( charToType( *it ) );
                ++it;
                if( it != itEND )
                {
                    friendlyName += wxT( ", " );
                }
            }
            friendlyName += wxT( " )" );
        }
    }
    catch( const ImpactAcquireException& ) {}
    return friendlyName;
}

//-----------------------------------------------------------------------------
wxString MethodObject::Call( int& callResult ) const
//-----------------------------------------------------------------------------
{
    if( m_GridItemId && GetComponent().isMeth() )
    {
        const wxString params = m_GridItemId->GetValueAsString();
        const Method meth( GetComponent() );
        bool boErrorHandled = false;
        long executionTime_ms = 0;
        const string paramsANSI( params.mb_str() );
        wxStopWatch stopWatch;
        try
        {
            wxBusyCursor busyCursorScope;
            callResult = meth.call( paramsANSI, " " );
            executionTime_ms = stopWatch.Time();
        }
        catch( const ImpactAcquireException& e )
        {
            executionTime_ms = stopWatch.Time();
            callResult = e.getErrorCode();
            wxString errorString( wxString::Format( wxT( "An error occurred while executing function '%s'(actual driver feature name: '%s'). %s(numerical error representation: %d (%s))." ), m_FriendlyName.c_str(), ConvertedString( GetComponent().name() ).c_str(), ConvertedString( e.getErrorString() ).c_str(), e.getErrorCode(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            if( e.getErrorCode() == PROPHANDLING_WRONG_PARAM_COUNT )
            {
                errorString.Append( wxString::Format( wxT( "\n\nWhen executing methods please make sure you have specified the correct amount of parameters.\nThe following parameter types are available:\n  'void': This function either doesn't expect parameters or does not return a value\n  'void*': An arbitrary pointer\n  'int': A 32-bit integer value\n  'int64': A 64-bit integer value\n  'float': A double precision floating type value\n  'char*': A C-type string\nTherefore a function 'int foobar(float, char*) will expect one floating point value and one C-type string(in this order) and will return an integer value. Before executing a method the desired parameters must be confirmed by pressing [ENTER] in the edit box.\n\n" ) ) );
            }
            errorString.append( wxString::Format( wxT( "Error origin: %s" ), ConvertedString( e.getErrorOrigin() ).c_str() ) );
            wxMessageDialog errorDlg( NULL, errorString, wxT( "Method Execution Failed" ), wxOK | wxICON_INFORMATION );
            errorDlg.ShowModal();
            boErrorHandled = true;
        }

        wxString resultMsg( wxString::Format( wxT( "Last call info: %s%s%s%s" ),
                                              m_FriendlyName.c_str(),
                                              params.IsEmpty() ? wxEmptyString : wxT( "( " ),
                                              params.c_str(),
                                              params.IsEmpty() ? wxEmptyString : wxT( " )" ) ) );
        const char retType = meth.paramList()[0];
        if( ( retType == 'v' ) || boErrorHandled )
        {
            callResult = DMR_NO_ERROR;
        }
        if( retType != 'v' )
        {
            resultMsg.append( wxString::Format( wxT( " returned %s(%d)" ), ConvertedString( ImpactAcquireException::getErrorCodeAsString( callResult ) ).c_str(), callResult ) );
        }
        resultMsg.append( wxString::Format( wxT( ", execution time: %ld ms" ), executionTime_ms ) );
        return resultMsg;
    }
    return wxString( wxT( "Invalid method object" ) );
}

//-----------------------------------------------------------------------------
wxString MethodObject::GetNameToUse( EDisplayFlags flags ) const
//-----------------------------------------------------------------------------
{
    return ( flags & dfDontUseFriendlyNamesForMethods ) ? ConvertedString( GetComponent().name() ) : m_FriendlyName;
}

//-----------------------------------------------------------------------------
void MethodObject::UpdatePropData( void )
//-----------------------------------------------------------------------------
{
    if( m_GridItemId )
    {
        m_Params = m_GridItemId->GetValueAsString();
    }
}

//-----------------------------------------------------------------------------
void MethodObject::Update( const PropTree* /*pPropTree*/, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const
//-----------------------------------------------------------------------------
{
    wxString label( GetNameToUse( flags ) );
    if( flags & dfDisplayDebugInfo )
    {
        AppendComponentInfo( label, actChangedCount, actAttrChangedCount );
    }
    if( m_GridItemId )
    {
        m_GridItemId->SetValueFromString( m_Params );
        UpdateLabelAndHelpString( flags, label );
    }
}

//-----------------------------------------------------------------------------
void MethodObject::EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified /* = 0 */ )
//-----------------------------------------------------------------------------
{
    if( !m_GridItemId )
    {
        m_pParentGridPage = pPropTree->GetPropGridPage();
        m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxStringProperty( GetNameToUse( flags ), BuildFullFeaturePath(), wxT( "" ) ) );
        m_pParentGridPage->SetPropertyEditor( m_GridItemId, wxPG_EDITOR( TextCtrlAndButton ) );
        m_pParentGridPage->SetPropertyClientData( m_GridItemId, this );
        m_GridItemId->SetExpanded( false );
        m_Type = _ctrlEdit;
        if( pboModified )
        {
            *pboModified = true;
        }
    }
}

//=============================================================================
//========================= ListObject ========================================
//=============================================================================
//-----------------------------------------------------------------------------
ListObject::ListObject( HOBJ hObj, const char* pTitle /* = 0 */ )
    : PropData( hObj ), m_boExpanded( FALSE ), m_Title( ConvertedString( pTitle ? pTitle : string() ) )
//-----------------------------------------------------------------------------
{
}

//-----------------------------------------------------------------------------
void ListObject::OnExpand( void )
//-----------------------------------------------------------------------------
{
    if( m_GridItemId && m_pParentGridPage )
    {
        m_boExpanded = m_pParentGridPage->IsPropertyExpanded( m_GridItemId );
    }
}

//-----------------------------------------------------------------------------
void ListObject::EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified/* = 0 */ )
//-----------------------------------------------------------------------------
{
    if( !m_GridItemId )
    {
        m_pParentGridPage = pPropTree->GetPropGridPage();
        wxPropertyCategory* const prop = new wxPropertyCategory( GetDisplayName( flags ), BuildFullFeaturePath() );
        AppendPropertyToGrid( prop, pParentGridComponent );
        if( m_GridItemId != prop )
        {
            wxASSERT( !"Invalid parenting" ); // no need to delete prop, wxPropertyGridState::PrepareToAddItem already did that
        }
        m_Type = _ctrlStatic;
        m_pParentGridPage->SetPropertyClientData( m_GridItemId, this );
        m_GridItemId->SetExpanded( false );
        ProcessContentDescriptor();
        if( pboModified )
        {
            *pboModified = true;
        }
    }
}

//-----------------------------------------------------------------------------
void ListObject::ProcessContentDescriptor( void ) const
//-----------------------------------------------------------------------------
{
    const ComponentList list( GetComponent() );
    ConvertedString contDesc( list.contentDescriptor() );
    if( contDesc.empty() )
    {
        if( m_pParentGridPage->GetPropertyTextColour( m_GridItemId ) != *wxBLACK )
        {
            m_GridItemId->SetTextColour( *wxBLACK, wxPG_DONT_RECURSE );
        }
    }
    else
    {
        m_GridItemId->SetValue( contDesc );
        if( m_pParentGridPage->GetPropertyTextColour( m_GridItemId ) != *wxBLUE )
        {
            m_GridItemId->SetTextColour( *wxBLUE, wxPG_DONT_RECURSE );
        }
    }
}

//-----------------------------------------------------------------------------
void ListObject::Update( const PropTree* /*pPropTree*/, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const
//-----------------------------------------------------------------------------
{
    if( ( ( flags & dfDisplayDebugInfo ) && ( actChangedCount != m_lastChangedCounter ) ) ||
        ( actAttrChangedCount != m_lastChangedCounterAttr ) )
    {
        wxString label;
        if( m_Title.empty() )
        {
            label = GetDisplayName( flags );
        }
        else
        {
            label = m_Title;
        }
        if( flags & dfDisplayDebugInfo )
        {
            label.Append( wxString::Format( wxT( "[%d]" ), ComponentList( GetComponent() ).size() ) );
            AppendComponentInfo( label, actChangedCount, actAttrChangedCount );
        }
        UpdateLabelAndHelpString( flags, label );
        ProcessContentDescriptor();
    }
}

//=============================================================================
//========================= PropertyObject ====================================
//=============================================================================
//-----------------------------------------------------------------------------
PropertyObject::PropertyObject( HOBJ hObj, int index /* = 0 */ )
    : PropData( hObj ), m_Index( index < 0 ? 0 : index ), m_boVectorAsList( index >= 0 ), m_emptyString()
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
wxString PropertyObject::BuildFullFeaturePath( void ) const
//-----------------------------------------------------------------------------
{
    wxString fullFeaturePath( PropData::BuildFullFeaturePath() );
    if( ( GetIndex() >= 0 ) && m_boVectorAsList )
    {
        fullFeaturePath.Append( wxString::Format( wxT( "[%d]" ), GetIndex() ) );
    }
    return fullFeaturePath;
}

//-----------------------------------------------------------------------------
wxString PropertyObject::GetCurrentValueAsString( void ) const
//-----------------------------------------------------------------------------
{
    const TComponentType type = GetComponent().type();
    const TComponentRepresentation representation = GetComponent().representation();
    if( ( representation == crBoolean ) && ( type == ctPropInt ) )
    {
        return PropertyI( GetComponent() ).read( m_Index ) == 0 ? wxT( "False" ) : wxT( "True" );
    }
    else if( ( representation == crIPv4Address ) && ( type == ctPropInt ) )
    {
        return GetIPv4AddressAsString( PropertyI( GetComponent() ), m_Index );
    }
    else if( ( representation == crIPv4Address ) && ( type == ctPropInt64 ) )
    {
        return GetIPv4AddressAsString( PropertyI64( GetComponent() ), m_Index );
    }
    else if( ( representation == crMACAddress ) && ( type == ctPropInt64 ) )
    {
        return MACAddressToString( PropertyI64( GetComponent() ).read( m_Index ) );
    }
    else if( ( type == ctPropString ) && ( GetComponent().flags() & cfContainsBinaryData ) )
    {
        // The 'type' check is only needed because some drivers with versions < 1.12.33
        // did incorrectly specify the 'cfContainsBinaryData' flag even though the data type was not 'ctPropString'...
        return ConvertedString( BinaryDataToString( PropertyS( GetComponent() ).readBinary( m_Index ) ) );
    }
    else
    {
        return ConvertedString( Property( GetComponent() ).readS( m_Index, string( ( m_Type == _ctrlMultiChoiceSelector ) ? "\"%s\" " : "" ) ) );
    }
}

//-----------------------------------------------------------------------------
bool PropertyObject::IsWriteable( void ) const
//-----------------------------------------------------------------------------
{
    bool boForceReadOnly = false;

    switch( GetType() )
    {
    case _ctrlMultiChoiceSelector:
        m_pParentGridPage->SetPropertyValueString( m_GridItemId, GetCurrentValueAsString() );
        if( GetComponent().isProp() )
        {
            Property prop( GetComponent() );
            if( prop.dictSize() <= 1 )
            {
                boForceReadOnly = true;
            }
        }
        break;
    default:
        break;
    }

    return GetComponent().isWriteable() && !boForceReadOnly;
}

//-----------------------------------------------------------------------------
void PropertyObject::WritePropVal( const string& value ) const
//-----------------------------------------------------------------------------
{
    if( !IsWriteable() )
    {
        wxASSERT( !"Trying to write to a non-writeable property" );
    }

    if( m_boVectorAsList )
    {
        // only write one value to the property!
        const string::size_type start = value.find_first_not_of( " " );
        if( start == string::npos )
        {
            wxMessageDialog errorDlg( NULL, wxT( "Couldn't set value(Empty string?)" ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
            errorDlg.ShowModal();
        }
        else
        {
            const string::size_type end = value.find_first_of( " ", start );
            WritePropVal( ( end != string::npos ) ? value.substr( start, end ) : value, m_Index );
        }
    }
    else
    {
        WritePropVal( value, 0 );
    }
}

//-----------------------------------------------------------------------------
void PropertyObject::WritePropVal( const string& value, const int index ) const
//-----------------------------------------------------------------------------
{
    const TComponentType type( GetComponent().type() );
    const TComponentRepresentation representation( GetComponent().representation() );
    const TComponentFlag flags( GetComponent().flags() );
    if( ( representation == crBoolean ) && ( type == ctPropInt ) )
    {
        PropertyI prop( GetComponent() );
        prop.write( value == "True", index );
    }
    else if( ( representation == crIPv4Address ) && ( type == ctPropInt ) )
    {
        PropertyI( GetComponent() ).write( ntohl( inet_addr( value.c_str() ) ), index );
    }
    else if( ( representation == crIPv4Address ) && ( type == ctPropInt64 ) )
    {
        PropertyI64( GetComponent() ).write( static_cast<int64_type>( ntohl( inet_addr( value.c_str() ) ) ), index );
    }
    else if( ( representation == crMACAddress ) && ( type == ctPropInt64 ) )
    {
        PropertyI64( GetComponent() ).write( MACAddressFromString( ConvertedString( value ) ), index );
    }
    else if( ( type == ctPropString ) && ( flags & cfContainsBinaryData ) )
    {
        // The 'type' check is only needed because some drivers with versions < 1.12.33
        // did incorrectly specify the 'cfContainsBinaryData' flag even though the data type was not 'ctPropString'...
        PropertyS( GetComponent() ).writeBinary( BinaryDataFromString( value ), index );
    }
    else
    {
        Property prop( GetComponent() );
        prop.writeS( ( ( flags & cfAllowValueCombinations ) && value.empty() ) ? m_emptyString : value, index );
    }
}

//-----------------------------------------------------------------------------
void PropertyObject::SetToLimit( const mvIMPACT::acquire::TPropertyLimits limit ) const
//-----------------------------------------------------------------------------
{
    if( !IsWriteable() )
    {
        wxASSERT( !"Trying to write to a non-writeable property" );
    }

    switch( GetComponent().type() )
    {
    case ctPropInt:
        {
            PropertyI prop( GetComponent() );
            prop.write( prop.read( limit ) );
        }
        break;
    case ctPropInt64:
        {
            PropertyI64 prop( GetComponent() );
            prop.write( prop.read( limit ) );
        }
        break;
    case ctPropFloat:
        {
            PropertyF prop( GetComponent() );
            prop.write( prop.read( limit ) );
        }
        break;
    default:
        {
            Property prop( GetComponent() );
            prop.writeS( prop.readS( limit ) );
        }
        break;
    }
}

//-----------------------------------------------------------------------------
void PropertyObject::UpdatePropData( void )
//-----------------------------------------------------------------------------
{
    if( m_GridItemId )
    {
        wxString errorString;
        const Property prop( GetComponent() );
        try
        {
            string valueANSI( m_GridItemId->GetValueAsString().mb_str() );
            WritePropVal( valueANSI );
        }
        catch( const EValTooSmall& )
        {
            errorString = wxT( "Value too small! Clipping to minimum!" );
            try
            {
                SetToLimit( plMinValue );
            }
            catch( const ImpactAcquireException& e )
            {
                errorString.Append( wxString::Format( wxT( " Can't set value( Error: %s(%s))!" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            }
        }
        catch( const EValTooLarge& )
        {
            errorString = wxT( "Value too large! Clipping to maximum!" );
            try
            {
                SetToLimit( plMaxValue );
            }
            catch( const ImpactAcquireException& e )
            {
                errorString.Append( wxString::Format( wxT( " Can't set value( Error: %s(%s))!" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
            }
        }
        catch( const ENoModifySizeRights& )
        {
            errorString = wxString::Format( wxT( "This property can store %d value(s) only!" ), prop.valCount() );
        }
        catch( const EValidationFailed& )
        {
            errorString = wxT( "Value didn't pass validation check. The device log-file will contain additional information!" );
        }
        catch( const ImpactAcquireException& e )
        {
            errorString = wxString::Format( wxT( "Can't set value( Error: %s(%s))!" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() );
        }

        m_GridItemId->SetValueFromString( GetCurrentValueAsString() );
        if( !errorString.IsEmpty() )
        {
            wxMessageDialog errorDlg( NULL, errorString, wxT( "Error" ), wxOK | wxICON_INFORMATION );
            errorDlg.ShowModal();
        }
    }
}

//-----------------------------------------------------------------------------
wxString PropertyObject::GetTransformedDict( mvIMPACT::acquire::Component comp, wxPGChoices& soc )
//-----------------------------------------------------------------------------
{
    wxString emptyString;
    const TComponentType type = comp.type();
    if( type == ctPropInt )
    {
        const TComponentFlag flags = comp.flags();
        vector<pair<string, int> > dict;
        PropertyI( comp ).getTranslationDict( dict );
        vector<pair<string, int> >::size_type vSize = dict.size();
        for( vector<pair<string, int> >::size_type i = 0; i < vSize; i++ )
        {
            if( ( flags & cfAllowValueCombinations ) && ( dict[i].second == 0 ) )
            {
                emptyString = dict[i].first;
            }
            else
            {
                soc.Add( ConvertedString( dict[i].first ), dict[i].second );
            }
        }
    }
    else if( type == ctPropInt64 )
    {
        // this just works as long as we always use the string values for setting properties as there are no controls
        // for 64 bit integer values so far
        const TComponentFlag flags = comp.flags();
        vector<pair<string, int64_type> > dict;
        PropertyI64( comp ).getTranslationDict( dict );
        vector<pair<string, int64_type> >::size_type vSize = dict.size();
        for( vector<pair<string, int64_type> >::size_type i = 0; i < vSize; i++ )
        {
            if( ( flags & cfAllowValueCombinations ) && ( dict[i].second == 0 ) )
            {
                emptyString = dict[i].first;
            }
            else
            {
                soc.Add( ConvertedString( dict[i].first ), dict[i].second );
            }
        }
    }
    else if( type == ctPropFloat )
    {
        // this just works as long as we always use the string values for setting properties as there are no controls
        // for double values so far
        vector<pair<string, double> > dict;
        PropertyF( comp ).getTranslationDict( dict );
        vector<pair<string, double> >::size_type vSize = dict.size();
        for( vector<pair<string, double> >::size_type i = 0; i < vSize; i++ )
        {
            soc.Add( ConvertedString( dict[i].first ), static_cast<int>( i ) );
        }
    }
    else
    {
        wxASSERT( !"Invalid component type for a combo box control" );
    }
    return emptyString;
}

//-----------------------------------------------------------------------------
void PropertyObject::EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified /* = 0 */ )
//-----------------------------------------------------------------------------
{
    m_pParentGridPage = pPropTree->GetPropGridPage();
    if( !m_GridItemId )
    {
        const wxString elementName( GetDisplayName( flags ) );
        m_Type = GetEditorType( GetComponent(), elementName );
        const bool boIsSelector = GetComponent().selectedFeatureCount() > 0;
        const wxString fullFeaturePath( BuildFullFeaturePath() );
        if( m_pParentGridPage->GetProperty( fullFeaturePath ) )
        {
            // we sometimes might get into situation in which a feature is still present in the tree, but actually
            // should have been deleted and re-created at an earlier moment in time. This is the case when we reach this
            // code as we are about to insert a feature in the tree, which under the very same name is already present.
            pPropTree->CleanupTree();
        }
        const TComponentType type = GetComponent().type();
        switch( m_Type )
        {
        case _ctrlBoolean:
            m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxBoolProperty( elementName, fullFeaturePath ) );
            m_pParentGridPage->SetPropertyAttribute( m_GridItemId, wxPG_BOOL_USE_CHECKBOX, wxVariant( 1 ) );
            break;
        case _ctrlSpinner:
            if( ( type == ctPropInt ) || ( type == ctPropInt64 ) || ( type == ctPropFloat ) )
            {
                m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxStringProperty( elementName, fullFeaturePath ) );
            }
            else
            {
                wxASSERT( !"invalid component type for spinner control" );
            }
            m_GridItemId->SetEditor( wxPGCustomSpinCtrlEditor_PropertyObject::Instance()->GetEditor() );
            break;
        case _ctrlEdit:
            m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxLongStringProperty( elementName, fullFeaturePath ) );
            break;
        case _ctrlCombo:
            {
                wxPGChoices soc;
                GetTransformedDict( soc );
                m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxEnumProperty( elementName, fullFeaturePath, soc ) );
            }
            break;
        case _ctrlMultiChoiceSelector:
            {
                wxPGChoices soc;
                GetTransformedDict( soc );
                wxPGProperty* p = new wxMultiChoiceProperty( elementName, fullFeaturePath, soc );
                m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, p );
            }
            break;
        case _ctrlFileSelector:
            m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxFileProperty( elementName, fullFeaturePath ) );
            break;
        case _ctrlDirSelector:
            m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxDirProperty( elementName, fullFeaturePath, ::wxGetUserHome() ) );
            break;
        case _ctrlBinaryDataEditor:
            m_GridItemId = m_pParentGridPage->AppendIn( pParentGridComponent, new wxBinaryDataProperty( elementName, fullFeaturePath ) );
            m_pParentGridPage->SetPropertyValidator( m_GridItemId, *wxBinaryDataProperty::GetClassValidator() );
            break;
        default:
            break;
        }

        m_pParentGridPage->SetPropertyClientData( m_GridItemId, this );
        m_GridItemId->SetExpanded( false );
        if( pboModified )
        {
            *pboModified = true;
        }

        if( ( flags & dfSelectorGrouping ) && boIsSelector )
        {
            vector<mvIMPACT::acquire::Component> selectedFeatures;
            const vector<mvIMPACT::acquire::Component>::size_type cnt = GetComponent().selectedFeatures( selectedFeatures );
            for( vector<mvIMPACT::acquire::Component>::size_type i = 0; i < cnt; i++ )
            {
                pPropTree->CreateGridProperty( selectedFeatures[i], m_GridItemId );
            }
        }
    }
}

//-----------------------------------------------------------------------------
PropData::ECtrl PropertyObject::GetEditorType( mvIMPACT::acquire::Component comp, const wxString& elementName )
//-----------------------------------------------------------------------------
{
    const TComponentType type = comp.type();
    const TComponentRepresentation representation = comp.representation();
    ECtrl editorType = _ctrlStatic;
    switch( type )
    {
    case ctPropInt:
    case ctPropInt64:
    case ctPropFloat:
        {
            const Property prop( comp );
            const TComponentFlag componentFlags( prop.flags() );
            if( ( componentFlags & cfShouldBeDisplayedAsEnumeration ) || prop.hasDict() )
            {
                if( componentFlags & cfAllowValueCombinations )
                {
                    editorType = _ctrlMultiChoiceSelector;
                }
                else
                {
                    editorType = ( representation == crBoolean ) ? _ctrlBoolean : _ctrlCombo;
                }
            }
            else
            {
                switch( representation )
                {
                case crBoolean:
                    editorType = _ctrlBoolean;
                    break;
                case crIPv4Address:
                case crMACAddress:
                    editorType = _ctrlEdit;
                    break;
                default:
                    editorType = _ctrlSpinner;
                    break;
                }
            }
        }
        break;
    case ctPropString:
        {
            const wxString lowerCaseName( elementName.Lower() );
            if( ( representation == crDirectoryName ) || lowerCaseName.Contains( wxString( wxT( "directory" ) ) ) )
            {
                editorType = _ctrlDirSelector;
            }
            else if( ( representation == crFileName ) || lowerCaseName.Contains( wxString( wxT( "filename" ) ) ) )
            {
                editorType = _ctrlFileSelector;
            }
            else if( comp.flags() & cfContainsBinaryData )
            {
                editorType = _ctrlBinaryDataEditor;
            }
            else
            {
                editorType = _ctrlEdit;
            }
        }
        break;
    case ctPropPtr:
        editorType = _ctrlEdit;
        break;
    default:
        break;
    }
    return editorType;
}

//-----------------------------------------------------------------------------
void PropertyObject::UpdateLabel( EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const
//-----------------------------------------------------------------------------
{
    ConvertedString label( GetDisplayName( flags ) );
    if( m_boVectorAsList )
    {
        label.Append( wxString::Format( ( ( flags & dfHexIndices ) ? wxT( "[0x%x]" ) : wxT( "[%d]" ) ), m_Index ) );
    }
    else
    {
        const int valCount = Property( GetComponent() ).valCount();
        if( valCount > 1 )
        {
            label.Append( wxString::Format( wxT( "[%d]" ), valCount ) );
        }
    }
    if( flags & dfDisplayDebugInfo )
    {
        AppendComponentInfo( label, actChangedCount, actAttrChangedCount );
    }
    UpdateLabelAndHelpString( flags, label );
    // Setting the text colour is rather expensive and can also result in focus losses for the current editor
    // so check if it is really necessary by reading the current colour first
    const wxColour& textColour( IsWriteable() ? GlobalDataStorage::Instance()->GetPropGridColour( GlobalDataStorage::pgcPropertyText ) : wxSystemSettings::GetColour( wxSYS_COLOUR_GRAYTEXT ) );
    if( textColour != m_pParentGridPage->GetPropertyTextColour( m_GridItemId ) )
    {
        m_pParentGridPage->SetPropertyTextColour( m_GridItemId, textColour, wxPG_DONT_RECURSE );
    }
    UpdateDefaultState();
}

//-----------------------------------------------------------------------------
void PropertyObject::Update( const PropTree* /*pPropTree*/, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const
//-----------------------------------------------------------------------------
{
    const Property prop( GetComponent() );
    switch( GetType() )
    {
    case _ctrlEdit:
        {
            const unsigned int valCnt = prop.valCount();
            if( ( valCnt > 1 ) && !m_boVectorAsList )
            {
                string str;
                for( unsigned int i = 0; i < valCnt; i++ )
                {
                    str.append( prop.readS( i ) );
                    str.append( " " );
                }
                m_pParentGridPage->SetPropertyValueString( m_GridItemId, ConvertedString( str ) );
            }
            else
            {
                m_pParentGridPage->SetPropertyValueString( m_GridItemId, GetCurrentValueAsString() );
            }
        }
        break;
    case _ctrlBoolean:
    case _ctrlSpinner:
    case _ctrlDirSelector:
    case _ctrlFileSelector:
    case _ctrlBinaryDataEditor:
    case _ctrlMultiChoiceSelector:
        m_pParentGridPage->SetPropertyValueString( m_GridItemId, GetCurrentValueAsString() );
        break;
    case _ctrlCombo:
        if( actAttrChangedCount != m_lastChangedCounterAttr )
        {
            wxPGChoices soc;
            GetTransformedDict( soc );
            m_GridItemId->SetChoices( soc );
        }
#if wxCHECK_VERSION(3, 1, 0)
        m_pParentGridPage->SetPropertyValueString( m_GridItemId, GetCurrentValueAsString() );
#else
        // This switch-case is a workaround for a problem in wxPropertyGrid in version 3.0.0 - 3.0.2:
        // When assigning a new list of choices setting the value as a string has no effect afterwards
        switch( GetComponent().type() )
        {
        case ctPropInt:
            m_pParentGridPage->SetPropertyValue( m_GridItemId, wxVariant( PropertyI( GetComponent() ).read() ) );
            break;
        case ctPropInt64:
            m_pParentGridPage->SetPropertyValue( m_GridItemId, wxVariant( static_cast<long>( PropertyI64( GetComponent() ).read() ) ) );
            break;
        default:
            m_pParentGridPage->SetPropertyValueString( m_GridItemId, GetCurrentValueAsString() );
            break;
        }
#endif // #if wxCHECK_VERSION(3, 1, 0)
        break;
    default:
        break;
    }

    // read-only / read-write state
    if( !IsWriteable() )
    {
        if( m_pParentGridPage->IsPropertyEnabled( m_GridItemId ) )
        {
            m_pParentGridPage->DisableProperty( m_GridItemId );
        }
    }
    else if( !m_pParentGridPage->IsPropertyEnabled( m_GridItemId ) )
    {
        m_pParentGridPage->EnableProperty( m_GridItemId );
    }
    UpdateLabel( flags, actChangedCount, actAttrChangedCount );
}

//=============================================================================
//========================= VectorPropertyObject ==============================
//=============================================================================
//-----------------------------------------------------------------------------
VectorPropertyObject::VectorPropertyObject( HOBJ hObj )
    : PropData( hObj ), m_boExpanded( false ) {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
VectorPropertyObject::~VectorPropertyObject()
//-----------------------------------------------------------------------------
{
    size_t propCnt = static_cast<size_t>( m_VectorItems.size() );
    m_pParentGridPage->GetGrid()->Freeze();
    for( size_t i = 0; i < propCnt; i++ )
    {
        DeleteGridProperty( i );
    }
    m_pParentGridPage->GetGrid()->Thaw();
}

//-----------------------------------------------------------------------------
void VectorPropertyObject::DeleteGridProperty( size_t index )
//-----------------------------------------------------------------------------
{
    if( m_VectorItems.at( index ) )
    {
        wxPGProperty* item = m_VectorItems[index]->GetGridItem();
        if( item )
        {
            m_pParentGridPage->DeleteProperty( item );
        }
        delete m_VectorItems[index];
        m_VectorItems[index] = 0;
    }
}

//-----------------------------------------------------------------------------
void VectorPropertyObject::OnExpand()
//-----------------------------------------------------------------------------
{
    if( m_GridItemId && m_pParentGridPage )
    {
        m_boExpanded = m_pParentGridPage->IsPropertyExpanded( m_GridItemId );
    }
}

//-----------------------------------------------------------------------------
PropertyObject* VectorPropertyObject::GetVectorItem( int index )
//-----------------------------------------------------------------------------
{
    const int vSize = static_cast<int>( m_VectorItems.size() );
    for( int i = vSize; i <= index; i++ )
    {
        m_VectorItems.push_back( new PropertyObject( GetComponent(), i ) );
    }
    return m_VectorItems.at( index );
}

//-----------------------------------------------------------------------------
void VectorPropertyObject::EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified /* = 0 */ )
//-----------------------------------------------------------------------------
{
    if( !m_GridItemId )
    {
        m_pParentGridPage = pPropTree->GetPropGridPage();
        wxPropertyCategory* const prop = new wxPropertyCategory( GetDisplayName( flags ), BuildFullFeaturePath() );
        AppendPropertyToGrid( prop, pParentGridComponent );
        if( m_GridItemId != prop )
        {
            wxASSERT( !"Invalid parenting" ); // no need to delete prop, wxPropertyGridState::PrepareToAddItem already did that
        }
        m_Type = _ctrlStatic;
        m_pParentGridPage->SetPropertyClientData( m_GridItemId, this );
        m_GridItemId->SetExpanded( false );
        m_GridItemId->SetTextColour( *wxBLACK );
        m_pParentGridPage->DisableProperty( m_GridItemId );
        if( pboModified )
        {
            *pboModified = true;
        }
    }

    const unsigned int actChangedCount = GetComponent().changedCounter();
    const unsigned int actAttrChangedCount = GetComponent().changedCounterAttr();
    if( ( m_lastChangedCounter != actChangedCount ) || ( m_lastChangedCounterAttr != actAttrChangedCount ) )
    {
        const Property prop( GetComponent() );
        const unsigned int valCount = prop.valCount();
        const unsigned int vSize = static_cast<unsigned int>( m_VectorItems.size() );
        if( valCount != vSize )
        {
            m_pParentGridPage->GetGrid()->Freeze();
        }
        for( unsigned int i = 0; i < prop.valCount(); i++ )
        {
            pPropTree->CreateGridProperty( GetComponent(), m_GridItemId, i );
        }
        if( valCount != vSize )
        {
            m_pParentGridPage->GetGrid()->Thaw();
        }
    }
}

//-----------------------------------------------------------------------------
void VectorPropertyObject::RemoveValue( unsigned int index )
//-----------------------------------------------------------------------------
{
    const Property prop( GetComponent() );
    const unsigned int vSize = static_cast<unsigned int>( m_VectorItems.size() );
    if( ( prop.valCount() > index ) && ( vSize > index ) && ( vSize > 1 ) )
    {
        prop.removeValue( index );
        DeleteGridProperty( index );
        m_VectorItems.erase( m_VectorItems.begin() + index );
    }
}

//-----------------------------------------------------------------------------
void VectorPropertyObject::Resize( void )
//-----------------------------------------------------------------------------
{
    const Property prop( GetComponent() );
    const unsigned int valCount = prop.valCount();
    const unsigned int vSize = static_cast<unsigned int>( m_VectorItems.size() );
    if( valCount < vSize )
    {
        m_pParentGridPage->GetGrid()->Freeze();
        for( unsigned int i = valCount; i < vSize; i++ )
        {
            wxPGProperty* item = m_VectorItems[i]->GetGridItem();
            if( item )
            {
                m_pParentGridPage->DeleteProperty( item );
            }
            delete m_VectorItems[i];
        }
        m_VectorItems.resize( valCount );
        m_pParentGridPage->GetGrid()->Thaw();
    }
    else if( valCount > vSize )
    {
        m_pParentGridPage->GetGrid()->Freeze();
        for( unsigned int i = vSize; i < valCount; i++ )
        {
            m_VectorItems.push_back( new PropertyObject( GetComponent(), static_cast<int>( i ) ) );
        }
        m_pParentGridPage->GetGrid()->Thaw();
    }
}

//-----------------------------------------------------------------------------
void VectorPropertyObject::Update( const PropTree* pPropTree, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const
//-----------------------------------------------------------------------------
{
    const Property prop( GetComponent() );
    const unsigned int valCnt = prop.valCount();
    unsigned int vSize = static_cast<unsigned int>( m_VectorItems.size() );
    ConvertedString label( GetDisplayName( flags ) );
    label.Append( wxString::Format( wxT( "[%d]" ), valCnt ) );
    if( flags & dfDisplayDebugInfo )
    {
        AppendComponentInfo( label, actChangedCount, actAttrChangedCount );
    }
    m_pParentGridPage->SetPropertyLabel( m_GridItemId, label );
    wxString val( wxT( "[" ) );
    val.Append( ConvertedString( prop.readSArray( "", ", " ) ) );
    val.Append( wxT( "]" ) );
    m_GridItemId->SetValue( val );
    while( valCnt > vSize )
    {
        pPropTree->CreateGridProperty( GetComponent(), m_GridItemId, vSize++ );
    }
    UpdateDefaultState();
}
