//-----------------------------------------------------------------------------
#ifndef PropTreeH
#define PropTreeH PropTreeH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <map>
#include <string>
#include <wx/wx.h>
#if wxMAJOR_VERSION < 3
#   error "You need at least Version 3.0.0 of wxWidgets to compile this application"
#endif // #if wxMAJOR_VERSION < 3
#include <wx/propgrid/manager.h>

class PropData;
#ifdef __WXPYTHON__
class MethodObject;
class ListObject;
class PropertyObject;
#endif
class VectorPropertyObject;

typedef std::map<mvIMPACT::acquire::HOBJ, PropData*> CompToDataMap;
typedef std::map<PropData*, mvIMPACT::acquire::HOBJ> DataToCompMap;

//-----------------------------------------------------------------------------
class PropGridFrozenScope
//-----------------------------------------------------------------------------
{
    wxPropertyGrid* pPG_;
public:
    explicit PropGridFrozenScope( wxPropertyGrid* pPG ) : pPG_( pPG )
    {
        pPG_->Freeze();
    }
    ~PropGridFrozenScope()
    {
        pPG_->Thaw();
    }
};

//-----------------------------------------------------------------------------
enum EDisplayFlags
//-----------------------------------------------------------------------------
{
    dfNone = 0x0,
    dfDisplayDebugInfo = 0x1,
    dfHexIndices = 0x2,
    dfDisplayInvisibleComponents = 0x4,
    dfDisplayNames = 0x8,
    dfSelectorGrouping = 0x10,
    dfDontUseFriendlyNamesForMethods = 0x20
};

//-----------------------------------------------------------------------------
class PropTree
//-----------------------------------------------------------------------------
{
    friend class VectorPropertyObject;
    friend class PropertyObject;
    mvIMPACT::acquire::ComponentIterator    m_rootList;
    wxPropertyGridPage* const               m_pPropGridPage;
    wxPGProperty*                           m_pTopLevelProp;
    const std::string                       m_Title;
    mutable CompToDataMap                   m_CompToDataMap;
    mutable DataToCompMap                   m_DataToCompMap;
    const EDisplayFlags                     m_flags;

    wxPGProperty*                           CreateGridProperty( mvIMPACT::acquire::HOBJ hObj, wxPGProperty* pParentProp, int index = -1, bool* pboModified = 0, const char* pTitle = 0 ) const;
    void                                    Delete( void );
    void                                    UpdateGridPropsRecursively( mvIMPACT::acquire::ComponentIterator iter, wxPGProperty* pListRoot, bool boForceRedraw ) const;
    void                                    UpdateGridPropsRecursively2( wxPGProperty* pListRoot, bool boForceDelete = false, bool boForceFullIteration = false ) const;
public:
    explicit                                PropTree( mvIMPACT::acquire::HOBJ hObj, const char* pTitle, wxPropertyGridPage* pPropGridPage, EDisplayFlags flags );
    virtual                                ~PropTree( void );
    void                                    CleanupTree( void ) const;
    unsigned int                            Draw( bool boForceRedraw );
#ifndef SWIG
    PropData*                               GetPropData( mvIMPACT::acquire::HOBJ hObj ) const;
#endif
#ifdef __WXPYTHON__
    MethodObject*                           GetMethodObject( mvIMPACT::acquire::HOBJ hObj ) const
    {
        return reinterpret_cast<MethodObject*>( GetPropData( hObj ) );
    }
    ListObject*                             GetListObject( mvIMPACT::acquire::HOBJ hObj ) const
    {
        return reinterpret_cast<ListObject*>( GetPropData( hObj ) );
    }
    PropertyObject*                         GetPropertyObject( mvIMPACT::acquire::HOBJ hObj ) const
    {
        return reinterpret_cast<PropertyObject*>( GetPropData( hObj ) );
    }
    VectorPropertyObject*                   GetVectorPropertyObject( mvIMPACT::acquire::HOBJ hObj ) const
    {
        return reinterpret_cast<VectorPropertyObject*>( GetPropData( hObj ) );
    }
#endif
    unsigned int                            GetChangedCounter( void ) const
    {
        return m_rootList.isValid() ? m_rootList.changedCounter() : 0;
    };
    const char*                             GetTitle( void ) const
    {
        return m_Title.empty() ? "" : m_Title.c_str();
    }
    wxPropertyGridPage*                     GetPropGridPage( void ) const
    {
        return m_pPropGridPage;
    }
};

#endif // PropTreeH
