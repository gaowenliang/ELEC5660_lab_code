//-----------------------------------------------------------------------------
#include "DataConversion.h"
#include "PropData.h"
#include "PropTree.h"
//-----------------------------------------------------------------------------

using namespace std;
using namespace mvIMPACT::acquire;

//-----------------------------------------------------------------------------
PropTree::PropTree( HOBJ hObj, const char* pTitle, wxPropertyGridPage* pPropGridPage, EDisplayFlags flags ) :
    m_rootList( hObj ), m_pPropGridPage( pPropGridPage ), m_pTopLevelProp( wxNullProperty ),
    m_Title( pTitle ? pTitle : string() ), m_flags( flags )
//-----------------------------------------------------------------------------
{
}

//-----------------------------------------------------------------------------
PropTree::~PropTree( void )
//-----------------------------------------------------------------------------
{
    PropGridFrozenScope propGridFrozenScope( m_pPropGridPage->GetGrid() );
    CompToDataMap::iterator it = m_CompToDataMap.begin();
    CompToDataMap::iterator itEnd = m_CompToDataMap.end();
    while( it != itEnd )
    {
        delete it->second;
        ++it;
    }
    Delete();
}

//-----------------------------------------------------------------------------
wxPGProperty* PropTree::CreateGridProperty( HOBJ hObj, wxPGProperty* pParentProp, int index /* = -1 */, bool* pboModified /* = 0 */, const char* pTitle /* = 0 */ ) const
//-----------------------------------------------------------------------------
{
    if( pboModified )
    {
        *pboModified = false;
    }

    // check if item is already present
    PropData* pPropData = GetPropData( hObj );
    if( !pPropData )
    {
        const Component comp( hObj );
        bool boCompToDataMap = true;
        switch( comp.type() )
        {
        case ctList:
            pPropData = new ListObject( hObj, pTitle );
            break;
        case ctMeth:
            pPropData = new MethodObject( hObj );
            break;
        case ctPropInt:
        case ctPropInt64:
        case ctPropFloat:
        case ctPropString:
        case ctPropPtr:
            {
                boCompToDataMap = index == -1;
                const bool boPropVector = ( index == -1 ) && ( comp.flags() & cfShouldBeDisplayedAsList );
                if( boPropVector )
                {
                    pPropData = new VectorPropertyObject( hObj );
                }
                else
                {
                    pPropData = new PropertyObject( hObj, index );
                }
            }
            break;
        default:
            wxASSERT( !"unrecognized component type" );
            break;
        }
        pPropData->SetRootList( m_rootList );
        if( boCompToDataMap )
        {
            m_CompToDataMap.insert( make_pair( hObj, pPropData ) );
            m_DataToCompMap.insert( make_pair( pPropData, hObj ) );
        }
    }
    else if( index != -1 )
    {
        VectorPropertyObject* const pPropVec = dynamic_cast<VectorPropertyObject*>( pPropData );
        wxASSERT( pPropVec != 0 );
        pPropVec->Resize();
        pPropData = pPropVec->GetVectorItem( index );
    }
    wxASSERT( pPropData != 0 );
    pPropData->EnsureValidGridItem( this, pParentProp, m_flags, pboModified );
    pPropData->UpdateGridItem( this, m_flags, pboModified );
    return pPropData->GetGridItem();
}

//-----------------------------------------------------------------------------
void PropTree::Delete( void )
//-----------------------------------------------------------------------------
{
    if( m_pTopLevelProp )
    {
        m_pPropGridPage->DeleteProperty( m_pTopLevelProp );
        m_pTopLevelProp = wxNullProperty;
    }
}

//-----------------------------------------------------------------------------
void PropTree::CleanupTree( void ) const
//-----------------------------------------------------------------------------
{
    if( m_pTopLevelProp )
    {
        UpdateGridPropsRecursively2( m_pPropGridPage->GetFirstChild( m_pTopLevelProp ), false, true );
        PropData* const pPropData = reinterpret_cast<PropData*>( m_pPropGridPage->GetPropertyClientData( m_pTopLevelProp ) );
        if( pPropData && pPropData->GetComponent().isValid() )
        {
            ComponentIterator it( pPropData->GetComponent().firstChild() );
            UpdateGridPropsRecursively( it, m_pTopLevelProp, true );
        }
    }
}

//-----------------------------------------------------------------------------
unsigned int PropTree::Draw( bool boForceRedraw )
//-----------------------------------------------------------------------------
{
    if( m_rootList.isValid() )
    {
        bool boModified = false;
        m_pTopLevelProp = CreateGridProperty( m_rootList, 0, -1, &boModified, m_Title.empty() ? 0 : m_Title.c_str() );
        if( m_pTopLevelProp )
        {
            // do NOT change the order of the next two calls as then deleted entries won't be
            // removed from the property grid correctly...
            // As an alternative the '&& pPropData->HasChanged()' could be removed from 'UpdateGridPropsRecursively2'
            UpdateGridPropsRecursively2( m_pPropGridPage->GetFirstChild( m_pTopLevelProp ) );
            UpdateGridPropsRecursively( ComponentIterator( m_rootList.firstChild() ), m_pTopLevelProp, boForceRedraw );
        }
        else
        {
            wxASSERT( !"Invalid mid level wxPGProperty*" );
        }
    }
    else
    {
        wxASSERT( !"Invalid ComponentIterator" );
    }

    return GetChangedCounter();
}

//-----------------------------------------------------------------------------
PropData* PropTree::GetPropData( HOBJ hObj ) const
//-----------------------------------------------------------------------------
{
    CompToDataMap::const_iterator it = m_CompToDataMap.find( hObj );
    return ( it != m_CompToDataMap.end() ) ? it->second : 0;
}

//-----------------------------------------------------------------------------
/// \brief Updates and/or creates grid properties
void PropTree::UpdateGridPropsRecursively( ComponentIterator iter, wxPGProperty* pListRoot, bool boForceRedraw ) const
//-----------------------------------------------------------------------------
{
    while( iter.isValid() )
    {
        bool boModified = false;
        wxPGProperty* pGridProp = CreateGridProperty( iter, pListRoot, -1, &boModified );
        if( iter.isList() && pGridProp && ( boModified || boForceRedraw ) )
        {
            PropData* const pPropData = reinterpret_cast<PropData*>( m_pPropGridPage->GetPropertyClientData( pGridProp ) );
            if( pPropData )
            {
                // this sublist needs to be updated as there have been changes in it...
                UpdateGridPropsRecursively( iter.firstChild(), pGridProp, boForceRedraw );
            }
        }
        ++iter;
    }
}

//-----------------------------------------------------------------------------
/// \brief Removes references and grid items that became invalid.
void PropTree::UpdateGridPropsRecursively2( wxPGProperty* pListRoot, bool boForceDelete /* = false */, bool boForceFullIteration /* = false */ ) const
//-----------------------------------------------------------------------------
{
    while( pListRoot )
    {
        wxPGProperty* toDelete = wxNullProperty;
        PropData* const pPropData = reinterpret_cast<PropData*>( m_pPropGridPage->GetPropertyClientData( pListRoot ) );
        wxASSERT( pPropData != 0 );
        if( pPropData )
        {
            const wxString propertyName( pListRoot->GetName() );
            if( pPropData->GetComponent().isValid() )
            {
                const wxString fullFeaturePath( pPropData->BuildFullFeaturePath() );
                if( propertyName != fullFeaturePath )
                {
                    // the handle might be valid (again!) but has been assigned to a different feature
                    // this might happen when one or more feature lists inside the driver are deleted and
                    // then recreated with new handles
                    if( pPropData->GetComponent().selectingFeatureCount() > 0 )
                    {
                        // The wxPropertyGrid will return the name of children under a 'normal' property as 'parentName.propertyName'!
                        // See 'Tree-like Property Structure' section in the wxPropertyGrid documentation
                        if( propertyName.Find( fullFeaturePath ) == wxNOT_FOUND )
                        {
                            boForceDelete = true;
                        }
                    }
                    else
                    {
                        boForceDelete = true;
                    }
                }
            }
            if( boForceDelete || !pPropData->GetComponent().isValid() )
            {
                if( ( pListRoot->GetChildCount() > 0 ) && !dynamic_cast<VectorPropertyObject*>( pPropData ) )
                {
                    // if we have to delete an entry we also have to delete all children of this entry
                    UpdateGridPropsRecursively2( m_pPropGridPage->GetFirstChild( pListRoot ), true, boForceFullIteration );
                }
                DataToCompMap::iterator it = m_DataToCompMap.find( pPropData );
                if( it != m_DataToCompMap.end() )
                {
                    CompToDataMap::iterator itCompToData = m_CompToDataMap.find( pPropData->GetComponent().hObj() );
                    wxASSERT( ( itCompToData != m_CompToDataMap.end() ) && "Inconsistent internal maps" );
                    if( itCompToData != m_CompToDataMap.end() )
                    {
                        m_CompToDataMap.erase( itCompToData );
                    }
                    m_DataToCompMap.erase( it );
                }
                delete pPropData;
                toDelete = pListRoot;
            }
            else if( ( pPropData->GetComponent().isList() && ( pPropData->HasChanged() || boForceFullIteration ) ) ||
                     ( ( m_flags & dfSelectorGrouping ) && ( pPropData->GetComponent().selectedFeatureCount() > 0 ) && ( pListRoot->GetChildCount() > 0 ) ) )
            {
                UpdateGridPropsRecursively2( m_pPropGridPage->GetFirstChild( pListRoot ), false, boForceFullIteration );
            }
        }

        const wxPGProperty* const parent = pListRoot->GetParent();
        const size_t next_ind = pListRoot->GetIndexInParent() + 1;
        pListRoot = next_ind < parent->GetChildCount() ? parent->Item( next_ind ) : wxNullProperty;
        if( toDelete )
        {
            m_pPropGridPage->DeleteProperty( toDelete );
        }
    }
}
