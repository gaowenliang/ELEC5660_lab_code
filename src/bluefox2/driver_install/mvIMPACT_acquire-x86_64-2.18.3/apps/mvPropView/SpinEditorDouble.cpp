//-----------------------------------------------------------------------------
#include "PropData.h"
#include "SpinEditorDouble.h"
#include "spinctld.h"
#include <limits>
//-----------------------------------------------------------------------------

using namespace std;
using namespace mvIMPACT::acquire;

//=============================================================================
//================= Implementation wxPGCustomSpinCtrlEditor ===================
//=============================================================================
//-----------------------------------------------------------------------------
/// Create controls and initialize event handling.
wxPGWindowList wxPGCustomSpinCtrlEditor::CreateControls( wxPropertyGrid* pPropGrid, wxPGProperty* pProperty, const wxPoint& pos, const wxSize& sz ) const
//-----------------------------------------------------------------------------
{
    // Get initial value (may be none if value is 'unspecified')
    const wxString text = pProperty->IsValueUnspecified() ? wxGetEmptyString() : pProperty->GetValueAsString( 0 );

    // Determine minimum and maximum
    double min = -1. * numeric_limits<double>::max(); // DBL_MIN is NOT what you'd expect it to be!
    double max = numeric_limits<double>::max();
    double step = 1.0;
    wxString format = wxEmptyString;
    TMode mode = mDouble;
    const Component comp( GetComponentFromClientData( pProperty ) );
    if( comp.isValid() )
    {
        switch( comp.type() )
        {
        case ctPropInt:
            {
                PropertyI prop( comp );
                format = ConvertedString( prop.stringFormatString() );
                min = prop.hasMinValue() ? prop.getMinValue() : numeric_limits<int>::min();
                max = prop.hasMaxValue() ? prop.getMaxValue() : numeric_limits<int>::max();
                step = prop.hasStepWidth() ? prop.getStepWidth() : 1.0;
                mode = mInt;
            }
            break;
        case ctPropInt64:
            {
                PropertyI64 prop( comp );
                format = ConvertedString( prop.stringFormatString() );
                min = prop.hasMinValue() ? prop.getMinValue() : numeric_limits<int64_type>::min();
                max = prop.hasMaxValue() ? prop.getMaxValue() : numeric_limits<int64_type>::max();
                step = prop.hasStepWidth() ? prop.getStepWidth() : 1.0;
                mode = mInt64;
            }
            break;
        case ctPropFloat:
            {
                PropertyF prop( comp );
                format = ConvertedString( prop.stringFormatString() );
                min = prop.hasMinValue() ? prop.getMinValue() : -1. * numeric_limits<double>::max(); // DBL_MIN is NOT what you'd expect it to be!
                max = prop.hasMaxValue() ? prop.getMaxValue() : numeric_limits<double>::max();
                if( prop.hasStepWidth() )
                {
                    step = prop.getStepWidth();
                }
                else
                {
                    const double range = max - min;
                    if( range <= 100. )
                    {
                        step = range / 100.;
                    }
                    else if( range <= 100000. )
                    {
                        step = range / 1000.;
                    }
                    else
                    {
                        step = 1.;
                    }
                }
            }
            break;
        default:
            {
                wxASSERT( !"unrecognized type for this editor class" );
                exit( 42 );
            }
        }
    }

    wxString valStr = pPropGrid->GetPropertyValueAsString( pProperty );
    double value = ToDouble( mode, valStr, format );
    const bool boLogarithmicBehaviour = comp.isValid() ? comp.representation() == crLogarithmic : false;

    // Use two stage creation to allow cleaner display on wxMSW
    wxSpinCtrlDbl* pEditor = new wxSpinCtrlDbl();
    pEditor->SetMode( mode );
#ifdef __WXMSW__
    pEditor->Hide();
#endif

    wxSize size( sz );
    size.SetHeight( sz.GetHeight() );
    if( !pEditor->Create( pPropGrid, wxID_ANY, text, pos, size, wxSP_ARROW_KEYS, min, max, value, step, 0xff, format.IsEmpty() ? wxString() : format, wxT( "wxSpinCtrlDbl" ), m_boCreateSlider, boLogarithmicBehaviour ) )
    {
        wxASSERT( !"Failed wxControl::Create" );
    }

#ifdef __WXMSW__
    pEditor->Show();
#endif

    return pEditor;
}

//-----------------------------------------------------------------------------
/// Copies value from property to control
void wxPGCustomSpinCtrlEditor::UpdateControl( wxPGProperty* pProperty, wxWindow* pWnd ) const
//-----------------------------------------------------------------------------
{
    wxSpinCtrlDbl* pEditor = ( wxSpinCtrlDbl* )pWnd;
    wxASSERT( pEditor && pEditor->IsKindOf( CLASSINFO( wxSpinCtrlDbl ) ) );

    wxString valS( pProperty->GetValueAsString() );
    double value = ToDouble( pEditor->GetMode(), valS, pEditor->GetFormat() );
    pEditor->SetValue( value );
}

//-----------------------------------------------------------------------------
/// Control's events are redirected here
bool wxPGCustomSpinCtrlEditor::OnEvent( wxPropertyGrid* pPropGrid, wxPGProperty* pProperty, wxWindow* pWnd, wxEvent& e ) const
//-----------------------------------------------------------------------------
{
    if( e.GetEventType() == wxEVT_COMMAND_SPINCTRL_UPDATED )
    {
        if( CopyValueFromControl( pProperty, pWnd ) )
        {
            return true;
        }
        pPropGrid->EditorsValueWasNotModified();
    }
    else
    {
        return wxPGTextCtrlEditor::OnEvent( pPropGrid, pProperty, pWnd, e );
    }
    return false;
}

//-----------------------------------------------------------------------------
/// Must be overloaded! If not the base class function will raise an exception in
/// debug mode as it will check the RTTI to make sure the correct 'OnFocus' function
/// has been called.
void wxPGCustomSpinCtrlEditor::OnFocus( wxPGProperty* /*pProperty*/, wxWindow* /*pWnd*/ ) const
//-----------------------------------------------------------------------------
{
}

//-----------------------------------------------------------------------------
bool wxPGCustomSpinCtrlEditor::CopyValueFromControl( wxPGProperty* pProperty, wxWindow* pWnd ) const
//-----------------------------------------------------------------------------
{
    return pProperty->SetValueFromString( GetValueFromControlAsString( pWnd ), wxPG_FULL_VALUE );
}

//-----------------------------------------------------------------------------
wxString wxPGCustomSpinCtrlEditor::GetValueFromControlAsString( wxWindow* pWnd ) const
//-----------------------------------------------------------------------------
{
    wxSpinCtrlDbl* pEditor = ( wxSpinCtrlDbl* )pWnd;
    wxASSERT( pEditor && pEditor->IsKindOf( CLASSINFO( wxSpinCtrlDbl ) ) );
    double value = pEditor->GetValue();
    TMode mode = pEditor->GetMode();
    switch( mode )
    {
    case mInt:
        return wxString::Format( pEditor->GetFormat(), static_cast<int>( value ) );
    case mInt64:
        return wxString::Format( pEditor->GetFormat(), static_cast<wxLongLong_t>( value ) );
    case mDouble:
        return wxString::Format( pEditor->GetFormat(), value );
    }
    wxASSERT( !"Unsupported value type detected!" );
    return wxEmptyString;
}

//-----------------------------------------------------------------------------
bool wxPGCustomSpinCtrlEditor::GetValueFromControl( wxVariant& variant, wxPGProperty* pProperty, wxWindow* pWnd ) const
//-----------------------------------------------------------------------------
{
    return pProperty->StringToValue( variant, GetValueFromControlAsString( pWnd ), wxPG_FULL_VALUE );
}

//-----------------------------------------------------------------------------
/// Makes control look like it has unspecified value
void wxPGCustomSpinCtrlEditor::SetValueToUnspecified( wxPGProperty* /*pProperty*/, wxWindow* pWnd ) const
//-----------------------------------------------------------------------------
{
    wxSpinCtrlDbl* pEditor = ( wxSpinCtrlDbl* )pWnd;
    wxASSERT( pEditor && pEditor->IsKindOf( CLASSINFO( wxSpinCtrlDbl ) ) );
    pEditor->SetValue( wxEmptyString, false );
}

//-----------------------------------------------------------------------------
/// Used when control's value is wanted to set from string source
/// (obviously, not all controls can implement this properly,
///  but wxSpinCtrl can)
void wxPGCustomSpinCtrlEditor::SetControlStringValue( wxPGProperty* /*pProperty*/, wxWindow* pWnd, const wxString& txt ) const
//-----------------------------------------------------------------------------
{
    wxSpinCtrlDbl* pEditor = ( wxSpinCtrlDbl* )pWnd;
    wxASSERT( pEditor && pEditor->IsKindOf( CLASSINFO( wxSpinCtrlDbl ) ) );
    pEditor->SetValue( txt, false );
}

//=============================================================================
//================= Implementation wxPGCustomSpinCtrlEditor_PropertyObject ====
//=============================================================================
wxPGCustomSpinCtrlEditor_PropertyObject* wxPGCustomSpinCtrlEditor_PropertyObject::m_pInstance = 0;

//-----------------------------------------------------------------------------
wxPGCustomSpinCtrlEditor_PropertyObject* wxPGCustomSpinCtrlEditor_PropertyObject::Instance( void )
//-----------------------------------------------------------------------------
{
    if( m_pInstance == 0 )
    {
        m_pInstance = new wxPGCustomSpinCtrlEditor_PropertyObject();
    }
    return m_pInstance;
}

//-----------------------------------------------------------------------------
Component wxPGCustomSpinCtrlEditor_PropertyObject::GetComponentFromClientData( wxPGProperty* pProperty ) const
//-----------------------------------------------------------------------------
{
    PropertyObject* const pPropData = reinterpret_cast<PropertyObject*>( pProperty->GetClientData() );
    return pPropData ? pPropData->GetComponent() : Component();
}

//=============================================================================
//================= Implementation wxPGCustomSpinCtrlEditor_HOBJ ==============
//=============================================================================
wxPGCustomSpinCtrlEditor_HOBJ* wxPGCustomSpinCtrlEditor_HOBJ::m_pInstance = 0;

//-----------------------------------------------------------------------------
wxPGCustomSpinCtrlEditor_HOBJ* wxPGCustomSpinCtrlEditor_HOBJ::Instance( void )
//-----------------------------------------------------------------------------
{
    if( m_pInstance == 0 )
    {
        m_pInstance = new wxPGCustomSpinCtrlEditor_HOBJ();
    }
    return m_pInstance;
}
//-----------------------------------------------------------------------------
Component wxPGCustomSpinCtrlEditor_HOBJ::GetComponentFromClientData( wxPGProperty* pProperty ) const
//-----------------------------------------------------------------------------
{
    const HOBJ hObj = static_cast<HOBJ>( reinterpret_cast<intptr_t>( pProperty->GetClientData() ) );
    try
    {
        return Component( hObj );
    }
    catch( const ImpactAcquireException& )
    {
    }
    // HOBJ doesn't reference a valid component
    return Component();
}

