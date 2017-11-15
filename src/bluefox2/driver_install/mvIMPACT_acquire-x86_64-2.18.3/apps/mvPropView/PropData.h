//-----------------------------------------------------------------------------
#ifndef PropDataH
#define PropDataH PropDataH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "PropTree.h"
#ifndef SWIG
#   include <string>
#endif

#ifndef SWIG
#   include <apps/Common/wxAbstraction.h>
#endif

//-----------------------------------------------------------------------------
class wxBinaryDataProperty : public wxLongStringProperty
//-----------------------------------------------------------------------------
{
public:
    explicit wxBinaryDataProperty( const wxString& label, const wxString& name, const wxString& value = wxEmptyString ) : wxLongStringProperty( label, name, value ) {}
    virtual ~wxBinaryDataProperty() {}
#if wxUSE_VALIDATORS
    static wxValidator* GetClassValidator( void );
    virtual wxValidator* DoGetValidator( void ) const;
#endif // #if wxUSE_VALIDATORS
    virtual bool OnButtonClick( wxPropertyGrid* pPropGrid, wxString& value );
};

//-----------------------------------------------------------------------------
class PropData
//-----------------------------------------------------------------------------
{
public:
    enum ECtrl
    {
        _ctrlStatic,
        _ctrlSpinner,
        _ctrlEdit,
        _ctrlCombo,
        _ctrlMultiChoiceSelector,
        _ctrlFileSelector,
        _ctrlDirSelector,
        _ctrlBinaryDataEditor,
        _ctrlBoolean
    };
    explicit                            PropData( mvIMPACT::acquire::HOBJ hObj );
    virtual                            ~PropData() {}
    static void                         AppendComponentInfo( const mvIMPACT::acquire::Component comp, wxString& info, unsigned int actChangedCount, unsigned int actAttrChangedCount );
    void                                AppendComponentInfo( wxString& info, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const;
    static void                         AppendSelectorInfo( std::ostringstream& oss, const std::vector<mvIMPACT::acquire::Component>& v );
    virtual wxString                    BuildFullFeaturePath( void ) const;
    virtual void                        EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified = 0  ) = 0;
    mvIMPACT::acquire::Component        GetComponent( void ) const
    {
        return m_Component;
    }
    virtual wxString                    GetDisplayName( EDisplayFlags flags ) const;
    wxPGProperty*                       GetGridItem( void ) const
    {
        return m_GridItemId;
    }
    wxPropertyGridPage*                 GetParentGridPage( void ) const
    {
        return m_pParentGridPage;
    }
    ECtrl                               GetType( void ) const
    {
        return m_Type;
    }
    void                                InvalidatePropGridItem( void )
    {
        m_GridItemId = wxNullProperty;
    }
    bool                                HasChanged( void ) const
    {
        return m_Component.changedCounter() != m_lastChangedCounter;
    }
    virtual void                        OnExpand( void ) {}
    void                                SetRootList( HOBJ hRootList )
    {
        m_rootList = hRootList;
    }
    virtual void                        Update( const PropTree* pPropTree, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const = 0;
    void                                UpdateGridItem( const PropTree* pPropTree, EDisplayFlags flags, bool* pboModified );
    virtual void                        UpdatePropData( void ) {}
protected:
    void                                AppendPropertyToGrid( wxPGProperty* pProp, wxPGProperty* pParentItem );
    void                                UpdateLabelAndHelpString( EDisplayFlags flags, wxString& label ) const;
    void                                UpdateDefaultState( void ) const;

    wxPGProperty*                       m_GridItemId;
    unsigned int                        m_lastChangedCounter;
    unsigned int                        m_lastChangedCounterAttr;
    wxPropertyGridPage*                 m_pParentGridPage;
    ECtrl                               m_Type;
private:
    const wxColour&                     GetBackgroundColour( void ) const;
    bool                                IsVisible( void ) const;
    virtual bool                        IsWriteable( void ) const
    {
        return GetComponent().isWriteable();
    }

    const mvIMPACT::acquire::Component  m_Component;
    HOBJ                                m_rootList;
};

//-----------------------------------------------------------------------------
class MethodObject : public PropData
//-----------------------------------------------------------------------------
{
public:
    explicit                            MethodObject( mvIMPACT::acquire::HOBJ hObj );
    static wxString                     BuildFriendlyName( mvIMPACT::acquire::HOBJ hObj );
    wxString                            Call( int& callResult ) const;
    void                                EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified = 0  );
    const wxString&                     FriendlyName( void ) const
    {
        return m_FriendlyName;
    }
    const wxString&                     Params( void ) const
    {
        return m_Params;
    }
    void                                UpdatePropData( void );
    void                                Update( const PropTree* pPropTree, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const;
private:
    wxString                            GetNameToUse( EDisplayFlags flags ) const;

    wxString                            m_Params;
    wxString                            m_FriendlyName;
};

//-----------------------------------------------------------------------------
class ListObject : public PropData
//-----------------------------------------------------------------------------
{
public:
    explicit                            ListObject( mvIMPACT::acquire::HOBJ hObj, const char* pTitle = 0 );
    bool                                IsExpanded( void ) const
    {
        return m_boExpanded;
    }
    void                                OnExpand( void );
    void                                EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified = 0  );
    void                                Update( const PropTree* pPropTree, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const;
private:
    void                                ProcessContentDescriptor( void ) const;

    bool                                m_boExpanded;
    const wxString                      m_Title;
};

//-----------------------------------------------------------------------------
class PropertyObject : public PropData
//-----------------------------------------------------------------------------
{
public:
    explicit                            PropertyObject( mvIMPACT::acquire::HOBJ hObj, int index = 0 );
    virtual wxString                    BuildFullFeaturePath( void ) const;
    void                                EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified = 0 );
    static ECtrl                        GetEditorType( mvIMPACT::acquire::Component comp, const wxString& elementName );
    int                                 GetIndex( void ) const
    {
        return m_Index;
    }
    static wxString                     GetTransformedDict( mvIMPACT::acquire::Component comp, wxPGChoices& soc );
    void                                Update( const PropTree* pPropTree, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const;
    void                                UpdatePropData( void );
private:
    wxString                            GetCurrentValueAsString( void ) const;
    void                                GetTransformedDict( wxPGChoices& soc ) const
    {
        m_emptyString = GetTransformedDict( GetComponent(), soc );
    }
    virtual bool                        IsWriteable( void ) const;
    void                                SetToLimit( const mvIMPACT::acquire::TPropertyLimits limit ) const;
    void                                UpdateLabel( EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const;
    void                                WritePropVal( const std::string& value ) const;
    void                                WritePropVal( const std::string& value, const int index ) const;

    const int                           m_Index;
    const bool                          m_boVectorAsList;
    mutable std::string                 m_emptyString; // The string that is used for multi-choice editors when non of the choices has been selected
};

//-----------------------------------------------------------------------------
class VectorPropertyObject : public PropData
//-----------------------------------------------------------------------------
{
public:
    explicit                            VectorPropertyObject( mvIMPACT::acquire::HOBJ hObj );
    ~VectorPropertyObject();
    bool                                IsExpanded( void ) const
    {
        return m_boExpanded;
    }
    void                                OnExpand( void );
    PropertyObject*                     GetVectorItem( int index );
    void                                EnsureValidGridItem( const PropTree* pPropTree, wxPGProperty* pParentGridComponent, EDisplayFlags flags, bool* pboModified = 0 );
    void                                RemoveValue( unsigned int index );
    void                                Resize( void );
    void                                Update( const PropTree* pPropTree, EDisplayFlags flags, unsigned int actChangedCount, unsigned int actAttrChangedCount ) const;
private:
    void                                DeleteGridProperty( size_t index );

    bool                                m_boExpanded;
    std::vector<PropertyObject*>        m_VectorItems;
};

#endif // PropDataH
