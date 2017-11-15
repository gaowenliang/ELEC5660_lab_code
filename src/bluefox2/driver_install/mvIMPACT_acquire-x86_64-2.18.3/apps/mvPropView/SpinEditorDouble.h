//-----------------------------------------------------------------------------
#ifndef SpinEditorDoubleH
#define SpinEditorDoubleH SpinEditorDoubleH
//-----------------------------------------------------------------------------
#include <wx/wx.h>
#if wxMAJOR_VERSION < 3
#   error "You need at least Version 3.0.0 of wxWidgets to compile this application"
#endif // #if wxMAJOR_VERSION < 3
#include <wx/propgrid/propgrid.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

//-----------------------------------------------------------------------------
class wxPGCustomSpinCtrlEditor : public wxPGTextCtrlEditor
//-----------------------------------------------------------------------------
{
    bool m_boCreateSlider;


    bool                                    CopyValueFromControl( wxPGProperty* pProperty, wxWindow* pWnd ) const;
    virtual mvIMPACT::acquire::Component    GetComponentFromClientData( wxPGProperty* pProperty ) const = 0;
    wxString                                GetValueFromControlAsString( wxWindow* pWnd ) const;
protected:
    wxPGEditor* m_pEditor;
    explicit                                wxPGCustomSpinCtrlEditor() : wxPGTextCtrlEditor(), m_boCreateSlider( true ) {}
public:
    virtual wxPGWindowList                  CreateControls( wxPropertyGrid* pPropGrid, wxPGProperty* pProperty, const wxPoint& pos, const wxSize& sz ) const;
    void                                    ConfigureControlsCreation( bool boCreateSlider )
    {
        m_boCreateSlider = boCreateSlider;
    }
    wxPGEditor*                             GetEditor( void ) const
    {
        return m_pEditor;
    }
    virtual bool                            GetValueFromControl( wxVariant& variant, wxPGProperty* pProperty, wxWindow* pWnd ) const;
    virtual bool                            OnEvent( wxPropertyGrid* pPropGrid, wxPGProperty* pProperty, wxWindow* pWnd, wxEvent& e ) const;
    virtual void                            OnFocus( wxPGProperty* pProperty, wxWindow* pWnd ) const;
    virtual void                            SetValueToUnspecified( wxPGProperty* pProperty, wxWindow* pWnd ) const;
    virtual void                            SetControlStringValue( wxPGProperty* pProperty, wxWindow* pWnd, const wxString& txt ) const;
    virtual void                            UpdateControl( wxPGProperty* pProperty, wxWindow* pWnd ) const;
};

//-----------------------------------------------------------------------------
class wxPGCustomSpinCtrlEditor_PropertyObject : public wxPGCustomSpinCtrlEditor
//-----------------------------------------------------------------------------
{
    static wxPGCustomSpinCtrlEditor_PropertyObject* m_pInstance;

    virtual mvIMPACT::acquire::Component            GetComponentFromClientData( wxPGProperty* pProperty ) const;
    explicit                                        wxPGCustomSpinCtrlEditor_PropertyObject() : wxPGCustomSpinCtrlEditor()
    {
        m_pEditor = wxPropertyGrid::DoRegisterEditorClass( this, GetName() );
    }
public:
    virtual wxString                                GetName( void ) const
    {
        return wxT( "CustomSpinCtrl_PropertyObject" );
    }
#ifndef SWIG
    static wxPGCustomSpinCtrlEditor_PropertyObject* Instance( void );
#endif
};

//-----------------------------------------------------------------------------
class wxPGCustomSpinCtrlEditor_HOBJ : public wxPGCustomSpinCtrlEditor
//-----------------------------------------------------------------------------
{
    static wxPGCustomSpinCtrlEditor_HOBJ*   m_pInstance;

    virtual mvIMPACT::acquire::Component    GetComponentFromClientData( wxPGProperty* pProperty ) const;
    explicit                                wxPGCustomSpinCtrlEditor_HOBJ() : wxPGCustomSpinCtrlEditor()
    {
        m_pEditor = wxPropertyGrid::DoRegisterEditorClass( this, GetName() );
    }
public:
    virtual wxString                        GetName( void ) const
    {
        return wxT( "CustomSpinCtrl_HOBJ" );
    }
#ifndef SWIG
    static wxPGCustomSpinCtrlEditor_HOBJ*   Instance( void );
#endif
};

#endif // SpinEditorDoubleH
