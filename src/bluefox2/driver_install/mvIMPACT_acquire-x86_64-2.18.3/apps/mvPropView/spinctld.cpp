/////////////////////////////////////////////////////////////////////////////
// Name:        spinctld.h
// Author:      John Labenski
// Created:     11/05/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

#if defined(__GNUG__) && !defined(NO_GCC_PRAGMA)
#    pragma implementation "spinctld.h"
#endif

// For compilers that support pre-compilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
#    pragma hdrstop
#endif

#ifndef WX_PRECOMP
#    include "wx/valtext.h"     // for wxTextValidator
#    include "wx/textctrl.h"
#endif // WX_PRECOMP

#include "wx/wxcrtvararg.h"
#include "wx/tooltip.h"
#include "spinctld.h"
#include <string>
#include <math.h>
#include <limits>

#if wxMINOR_VERSION > 4
#    include "wx/math.h"
#else
#if defined(__VISUALC__) || defined(__BORLANDC__) || defined(__WATCOMC__)
#    include <float.h>
#    define wxFinite(x) _finite(x)
#elif defined(__GNUG__)||defined(__GNUWIN32__)||defined(__DJGPP__)|| \
    defined(__SGI_CC__)||defined(__SUNCC__)||defined(__XLC__)|| \
    defined(__HPUX__)||defined(__MWERKS__)
#    define wxFinite(x) finite(x)
#else
#    define wxFinite(x) ((x) == (x))
#endif
#endif

//-----------------------------------------------------------------------------
/// \brief converts a string into an unsigned integer value.
///
/// hex values MUST start with '0x' or '0X' in order to be converted correctly when
/// \a boForceHex is not set. If \a boForceHex is set, the string doesn't need to start
/// with '0x', but should be hex data in order to be converted correctly.
///
/// octal values are not supported
///
/// \return
/// - 0 conversion successful
/// - -1 overflow ( currently only detected for hex values)
template<typename _Ty>
inline int toInteger( const wxString& str, _Ty& result, bool boForceHex = false )
//-----------------------------------------------------------------------------
{
    result = 0;
    if( str.empty() )
    {
        return 0;
    }

    bool isHex = boForceHex;
    bool isNegative = false;
    wxString::size_type offset = 0;
    if( str.length() >= 2 )
    {
        if( ( str.substr( 0, 2 ) == wxT( "0x" ) ) || ( str.substr( 0, 2 ) == wxT( "0X" ) ) )
        {
            isHex = true;
            offset = 2;
        }
        else if( str[0] == '-' )
        {
            offset = 1;
            isNegative = true;
        }
    }

    // only convert until the first invalid character is encountered
    wxString absolutValue = str.substr( offset );
    wxString::size_type end = absolutValue.find_first_not_of( wxT( "0123456789abcdefABCDEF" ) );
    if( end != wxString::npos )
    {
        absolutValue = absolutValue.substr( 0, end );
    }

    const _Ty BASE = ( ( isHex ) ? 16 : 10 );
    _Ty multiplier = 1;
    int error = 0;

    if( isHex )
    {
        if( absolutValue.length() > sizeof( _Ty ) * 2 )
        {
            error = -1; // overflow
        }
    }

    // do conversion
    for( wxString::size_type i = absolutValue.length(); i > 0; i-- )
    {
        if( isdigit( absolutValue[i - 1] ) )
        {
            result += ( absolutValue[i - 1] - wxT( '0' ) ) * multiplier;
        }
        else
        {
            result += static_cast<_Ty>( ( 10 + tolower( absolutValue[i - 1] ) - wxT( 'a' ) ) ) * multiplier;
        }
        multiplier = multiplier * BASE;
    }

    result = ( ( isNegative ) ? -result : result );
    return error;
}

//-----------------------------------------------------------------------------
double ToDouble( TMode mode, const wxString& valStr, const wxString& format )
//-----------------------------------------------------------------------------
{
    double value;
    switch( mode )
    {
    case mInt:
        {
            long tmpVal = 0;
            valStr.ToLong( &tmpVal, ( format.find( wxT( "x" ) ) != wxString::npos ) ? 16 : 10 );
            value = static_cast<double>( tmpVal );
        }
        break;
    case mInt64:
        {
            wxLongLong_t tmpVal = 0;
            toInteger<wxLongLong_t>( valStr, tmpVal, format.find( wxT( "x" ) ) != wxString::npos );
            value = static_cast<double>( tmpVal );
        }
        break;
    default:
        {
            wxString tmpValString( valStr );
            tmpValString.Replace( wxT( "," ), wxT( "." ) );
            tmpValString.ToDouble( &value );
        }
        break;
    }
    return value;
}

//-----------------------------------------------------------------------------
wxString ToString( TMode mode, double value, const wxString& format )
//-----------------------------------------------------------------------------
{
    wxString result;
    switch( mode )
    {
    case mInt:
        return wxString::Format( format.c_str(), static_cast<int>( value ) );
    case mInt64:
        return wxString::Format( format.c_str(), static_cast<wxLongLong_t>( value ) );
    default:
        break;
    }
    return wxString::Format( format.c_str(), value );
}

// NOTES : if the textctrl is focused and the program is ending, a killfocus
//         event is sent in MSW, this is why m_textCtrl is set to NULL in its
//         destructor and there's so many checks for it not being NULL

//----------------------------------------------------------------------------
// wxSpinCtrlDbl
//----------------------------------------------------------------------------

// the textctrl used for the wxSpinCtrlDbl, needed for keypresses
class wxSpinCtrlDblTextCtrl : public wxTextCtrl
{
public:
    wxSpinCtrlDblTextCtrl( wxWindow* parent, wxWindowID id,
                           const wxString& value = wxEmptyString,
                           const wxPoint& pos = wxDefaultPosition,
                           const wxSize& size = wxDefaultSize,
                           long style = 0,
                           const wxValidator& validator = wxDefaultValidator,
                           const wxString& name = wxTextCtrlNameStr );

    // MSW sends extra kill focus event
    virtual ~wxSpinCtrlDblTextCtrl()
    {
        if( m_parent )
        {
            m_parent->m_textCtrl = NULL;
        }
        m_parent = NULL;
    }

    wxSpinCtrlDbl* m_parent;

    void OnChar( wxKeyEvent& e );        // pass chars to wxSpinCtrlDbl
    void OnKillFocus( wxFocusEvent& e ); // sync the spin to textctrl

private:
    DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE( wxSpinCtrlDblTextCtrl, wxTextCtrl )
    EVT_CHAR( wxSpinCtrlDblTextCtrl::OnChar )
    EVT_KILL_FOCUS( wxSpinCtrlDblTextCtrl::OnKillFocus )
END_EVENT_TABLE()

wxSpinCtrlDblTextCtrl::wxSpinCtrlDblTextCtrl( wxWindow* parent, wxWindowID id,
        const wxString& value,
        const wxPoint& pos, const wxSize& size,
        long style,
        const wxValidator& validator,
        const wxString& name )
    : wxTextCtrl( parent, id, value, pos, size, style, validator, name )
{
    m_parent = ( wxSpinCtrlDbl* )parent;
}

void wxSpinCtrlDblTextCtrl::OnChar( wxKeyEvent& e )
{
    if( m_parent )
    {
        m_parent->OnChar( e );
    }
}

void wxSpinCtrlDblTextCtrl::OnKillFocus( wxFocusEvent& e )
{
    if( m_parent )
    {
        m_parent->SyncSpinToText( true );
    }
    e.Skip();
}

//----------------------------------------------------------------------------
// wxSpinCtrlDbl
//----------------------------------------------------------------------------

IMPLEMENT_DYNAMIC_CLASS( wxSpinCtrlDbl, wxControl )

BEGIN_EVENT_TABLE( wxSpinCtrlDbl, wxControl )
    EVT_COMMAND_SCROLL( widSlider, wxSpinCtrlDbl::OnScrollThumbtrack )
    EVT_SPIN_UP       ( wxID_ANY, wxSpinCtrlDbl::OnSpinUp )
    EVT_SPIN_DOWN     ( wxID_ANY, wxSpinCtrlDbl::OnSpinDown )
    EVT_TEXT_ENTER    ( wxID_ANY, wxSpinCtrlDbl::OnTextEnter )
    EVT_SET_FOCUS     ( wxSpinCtrlDbl::OnFocus )
    EVT_KILL_FOCUS    ( wxSpinCtrlDbl::OnKillFocus )
END_EVENT_TABLE()

void wxSpinCtrlDbl::Init()
{
    m_min = 0;
    m_max = 100;
    m_value = 0;
    m_default_value = 0;
    m_sliderCorrectionFactor = 1.;
    m_increment = 1;
    m_digits = wxSPINCTRLDBL_AUTODIGITS;
    m_snap_ticks = false;
    m_spinButton = NULL;
    m_slider = NULL;
    m_boSliderWithLogarithmicBehaviour = false;
    m_logScaleFactor = 1.;
    m_textCtrl = NULL;
    m_nRepeatCount = 0;
    m_LastChange = 0;
}

bool wxSpinCtrlDbl::Create( wxWindow* parent, wxWindowID id,
                            const wxString& value,
                            const wxPoint& pos, const wxSize& size,
                            long style,
                            double min, double max,
                            double initial,
                            double increment, int digits,
                            const wxString& format,
                            const wxString& name,
                            bool boWithSlider /* = false */,
                            bool boSliderWithLogarithmicBehaviour /* = false */ )
{
    if( !wxControl::Create( parent, id, pos, size, style | wxNO_BORDER, wxDefaultValidator, name ) )
    {
        return false;
    }

    wxControl::SetLabel( name );
    wxControl::SetBackgroundColour( parent->GetBackgroundColour() );
    wxControl::SetForegroundColour( parent->GetForegroundColour() );

    int width = size.GetWidth(), height = size.GetHeight();

    wxSize best_size( DoGetBestSize() );
    if( width  == -1 )
    {
        width  = best_size.GetWidth();
    }
    if( height == -1 )
    {
        height = best_size.GetHeight();
    }

    // Create a validator for numbers, +-, and eE for exponential
    wxTextValidator validator( wxFILTER_INCLUDE_CHAR_LIST );

#if wxCHECK_VERSION(2, 5, 4)
    wxArrayString list;

    wxString valid_chars( wxT( " 0123456789+-.,abcdefABCDEF" ) );
    size_t len = valid_chars.Length();
    for ( size_t i = 0; i < len; i++ )
    {
        list.Add( wxString( valid_chars.GetChar( i ) ) );
    }

    validator.SetIncludes( list );
#else
    wxStringList list;

    wxString valid_chars( wxT( " 0123456789+-.,abcdefABCDEF\"" ) );
    size_t len = valid_chars.Length();
    for ( size_t i = 0; i < len; i++ )
    {
        list.Add( wxString( valid_chars.GetChar( i ) ) );
    }

    validator.SetIncludeList( list );
#endif // wxCHECK_VERSION(2, 5, 4)

    m_spinButton = new wxSpinButton( this, id, wxPoint( 0, 0 ), wxSize( -1, height ),
                                     wxSP_ARROW_KEYS | wxSP_VERTICAL | wxSP_WRAP );
    int controlSize = ( width - m_spinButton->GetSize().GetWidth() );
    if( boWithSlider )
    {
        controlSize /= 2;
        const double discreteValues = fabs( ( max - min ) / increment );
        while( ( discreteValues / m_sliderCorrectionFactor ) > std::numeric_limits<int>::max() )
        {
            m_sliderCorrectionFactor *= 10.;
        }
        const int sliderValue = static_cast<int>( initial / ( increment * m_sliderCorrectionFactor ) );
        const int sliderMin = static_cast<int>( min / ( increment * m_sliderCorrectionFactor ) );
        const int sliderMax = static_cast<int>( max / ( increment * m_sliderCorrectionFactor ) );
        if( sliderMin < sliderMax ) // at least since wxWidgets 3.x min == max when creating a slider would cause an assertion and a slider would help much anyway if there is nothing 'to slide to'
        {
            m_slider = new wxSlider( this, widSlider, sliderValue, sliderMin, sliderMax, wxPoint( controlSize, 0 ), wxSize( -1, height ) );
            m_boSliderWithLogarithmicBehaviour = boSliderWithLogarithmicBehaviour && ( sliderMin >= 1 );
            if( m_boSliderWithLogarithmicBehaviour )
            {
                m_logScaleFactor = ( log10( max ) - log10( min ) ) / static_cast<double>( sliderMax - sliderMin );
            }
        }
        else
        {
            m_sliderCorrectionFactor = 1.;
        }
    }
    m_textCtrl = new wxSpinCtrlDblTextCtrl( this, id, value,
                                            wxPoint( 0, 0 ),
                                            wxSize( controlSize, height ),
                                            wxTE_NOHIDESEL | wxTE_PROCESS_ENTER, validator );

    DoSetSize( pos.x, pos.y, width, height );
#if wxCHECK_VERSION(2, 8, 0)
    SetInitialSize( wxSize( width, height ) );
#else
    SetBestSize( wxSize( width, height ) );
#endif

    m_min = min;
    m_max = max;
    m_value = initial;
    m_default_value = initial;
    m_increment = increment;
    if( format.empty() )
    {
        SetDigits( digits );
    }
    else
    {
        m_textFormat = format;
        SetDigits( digits, custom_fmt );
    }

    // set the value here without generating an event
    if( !value.IsEmpty() )
    {
        m_textCtrl->SetValue( value );
    }
    else
    {
        m_textCtrl->SetValue( ToString( m_mode, initial, m_textFormat ) );
    }

    return true;
}

wxSpinCtrlDbl::~wxSpinCtrlDbl()
{
    if( m_textCtrl ) // null this since MSW sends KILL_FOCUS on deletion
    {
        m_textCtrl->m_parent = NULL;

        wxSpinCtrlDblTextCtrl* text = m_textCtrl;
        m_textCtrl = NULL;
        delete text;
    }

    delete m_spinButton;
    m_spinButton = NULL;
    delete m_slider;
    m_slider = NULL;
}

#define wxSPINCTRLDBL_SPIN_WIDTH  15
#define wxSPINCTRLDBL_SPIN_HEIGHT 22

void wxSpinCtrlDbl::DoSetSize( int x, int y, int width, int height, int sizeFlags )
{
    //wxPrintf(wxT("DoSetSize %d, %d %d %d %d %d\n"), GetId(), x, y, width, height, sizeFlags);

    wxSize bestSize( DoGetBestSize() );
    if( width < 0 )
    {
        width  = bestSize.GetWidth();
    }
    if( height < 0 )
    {
        height = bestSize.GetHeight();
    }

    wxWindow::DoSetSize( x, y, width, height, sizeFlags );

    int spinwidth  = wxSPINCTRLDBL_SPIN_WIDTH;
    int spinheight = wxSPINCTRLDBL_SPIN_HEIGHT;
    if( m_spinButton )
    {
        m_spinButton->GetSize( &spinwidth, &spinheight );
    }

    int controlWidth = ( width - spinwidth );
    if( m_slider )
    {
        controlWidth /= 2;
    }
    if( m_textCtrl )
    {
        m_textCtrl->SetSize( 0, 0, controlWidth, height );
    }
    if( m_slider )
    {
        m_slider->SetSize( controlWidth, 0, controlWidth, height );
    }
#ifdef __WIN95__   // humm... these used to be different
    if( m_spinButton )
    {
        m_spinButton->SetSize( width - spinwidth - 2, 0, -1, height );
    }
    //m_textCtrl->SetSize( -3, -3, width - spinwidth, height );   // old wxWin < 2.3.2
    //m_spinButton->SetSize( width-spinwidth-4, -3, -1, height-1 );
#else
    if( m_spinButton )
    {
        m_spinButton->SetSize( width - spinwidth, 0, -1, height );
    }
#endif
}

static wxSize s_spinctrl_bestSize( -999, -999 );

wxSize wxSpinCtrlDbl::DoGetBestSize() const
{
    //wxPrintf(wxT("GetBestSize %d\n"), GetId());
    if( s_spinctrl_bestSize.x == -999 )
    {
        wxSpinCtrl spin( ( wxWindow* )this, wxID_ANY );
        s_spinctrl_bestSize = spin.GetBestSize();
        // oops something went wrong, set to reasonable value
        if( s_spinctrl_bestSize.GetWidth()  < 20 )
        {
            s_spinctrl_bestSize.SetWidth( 95 );
        }
        if( s_spinctrl_bestSize.GetHeight() < 10 )
        {
            s_spinctrl_bestSize.SetHeight( wxSPINCTRLDBL_SPIN_HEIGHT );
        }
    }

    return s_spinctrl_bestSize;
}

void wxSpinCtrlDbl::DoSetToolTip( wxToolTip* tip )
{
    // forward tip to textctrl only since having the tip pop up on the buttons
    // is distracting.
    if( tip && m_textCtrl )
    {
        wxPrintf( wxT( "TIP %s\n" ), tip->GetTip().c_str() );
        m_textCtrl->SetToolTip( tip->GetTip() );
    }

    wxControl::DoSetToolTip( tip );
}

void wxSpinCtrlDbl::DoSendEvent()
{
    wxCommandEvent e( wxEVT_COMMAND_SPINCTRL_UPDATED, GetId() );
    e.SetEventObject( this );
    e.SetInt( ( int )( m_value + 0.5 ) );
    if( m_textCtrl )
    {
        e.SetString( m_textCtrl->GetValue() );
    }
    GetEventHandler()->ProcessEvent( e );
}

double wxSpinCtrlDbl::GetStep( void )
{
    double value = m_increment;
    if( clock() - m_LastChange > CLOCKS_PER_SEC / 4 )
    {
        m_nRepeatCount = 0;
    }
    else
    {
        double factor = pow( ( double )10, ( double )( ( m_nRepeatCount * 2 ) / ( 10 * log( ( double )( ( m_nRepeatCount * 2 ) + 10 ) ) ) ) );
        value = ( int )( ( factor * m_increment ) / m_increment ) * m_increment;
        ++m_nRepeatCount;
    }
    m_LastChange = clock();
    return value;
}

void wxSpinCtrlDbl::OnScrollThumbtrack( wxScrollEvent& e )
{
    if( m_textCtrl && m_textCtrl->IsModified() )
    {
        SyncSpinToText( false );
    }

    if( m_boSliderWithLogarithmicBehaviour )
    {
        m_value = pow( 10, log10( m_min ) + m_logScaleFactor * static_cast<double>( e.GetPosition() - m_slider->GetMin() ) );
    }
    else
    {
        m_value = static_cast<double>( e.GetPosition() ) * m_increment * m_sliderCorrectionFactor;
    }
    SetValue( m_value );
    SyncSpinToText( false );
    DoSendEvent();
}

void wxSpinCtrlDbl::OnSpinUp( wxSpinEvent& WXUNUSED( e ) )
{
    if( m_textCtrl && m_textCtrl->IsModified() )
    {
        SyncSpinToText( false );
    }

    double step = GetStep();
    if( InRange( m_value + step ) )
    {
        m_value += step;
        SetValue( m_value );
        DoSendEvent();
    }
    else
    {
        SetValue( m_max );
        DoSendEvent();
    }
}

void wxSpinCtrlDbl::OnSpinDown( wxSpinEvent& WXUNUSED( e ) )
{
    if( m_textCtrl && m_textCtrl->IsModified() )
    {
        SyncSpinToText( false );
    }

    double step = GetStep();
    if( InRange( m_value - step ) )
    {
        m_value -= step;
        SetValue( m_value );
        DoSendEvent();
    }
    else
    {
        SetValue( m_min );
        DoSendEvent();
    }
}

void wxSpinCtrlDbl::OnTextEnter( wxCommandEvent& e )
{
    SyncSpinToText( true );
    e.Skip();
}

void wxSpinCtrlDbl::OnText( wxCommandEvent& e )
{
    //wxPrintf(wxT("Text '%s'\n"), e.GetString()); fflush(stdout);
    e.Skip();
}

void wxSpinCtrlDbl::OnChar( wxKeyEvent& e )
{
    double modifier = 1.0;
    if( e.m_shiftDown )
    {
        modifier  = 2.0;
    }
    if( e.m_controlDown )
    {
        modifier *= 10.0;
    }
    if( e.m_altDown )
    {
        modifier *= 100.0;
    }

    switch( e.GetKeyCode() )
    {
    case WXK_UP:
        SyncSpinToTextFromKeyboardInput( m_increment * modifier );
        break;
    case WXK_DOWN:
        SyncSpinToTextFromKeyboardInput( -m_increment * modifier );
        break;
    case WXK_PAGEUP:
        SyncSpinToTextFromKeyboardInput( m_increment * 10.0 * modifier );
        break;
    case WXK_PAGEDOWN:
        SyncSpinToTextFromKeyboardInput( -m_increment * 10.0 * modifier );
        break;
    case WXK_SPACE:
        SetValue( m_value );
        e.Skip( false );
        break;
    case WXK_RETURN:
        SyncSpinToTextFromKeyboardInput();
        break;
    case WXK_ESCAPE:
        SetDefaultValue();
        DoSendEvent();
        break;
    case WXK_TAB:
        {
            SyncSpinToTextFromKeyboardInput();

            wxNavigationKeyEvent new_event;
            new_event.SetEventObject( GetParent() );
            new_event.SetDirection( !e.ShiftDown() );
            // CTRL-TAB changes the (parent) window, i.e. switch notebook page
            new_event.SetWindowChange( e.ControlDown() );
            new_event.SetCurrentFocus( this );
            GetParent()->GetEventHandler()->ProcessEvent( new_event );
            break;
        }
    default:
        e.Skip();
        break;
    }
}

void wxSpinCtrlDbl::SetValue( double value )
{
    if( !m_textCtrl || !InRange( value ) )
    {
        return;
    }

    if( m_snap_ticks && ( m_increment != 0 ) )
    {
        double snap_value = ( value - m_default_value ) / m_increment;

        if( wxFinite( snap_value ) ) // FIXME what to do about a failure?
        {
            if( snap_value - floor( snap_value ) < ceil( snap_value ) - snap_value )
            {
                value = m_default_value + floor( snap_value ) * m_increment;
            }
            else
            {
                value = m_default_value + ceil( snap_value ) * m_increment;
            }
        }
    }


    wxString str( ToString( m_mode, value, m_textFormat.c_str() ) );

    if( ( value != m_value ) || ( str != m_textCtrl->GetValue() ) )
    {
        m_textCtrl->SetValue( str );
        m_textCtrl->DiscardEdits();
        m_value = ToDouble( m_mode, str, m_textFormat );
    }

    if( m_slider )
    {
        int sliderValue = 0;
        if( m_boSliderWithLogarithmicBehaviour )
        {
            sliderValue = ( log10( m_value ) - log10( m_min ) ) / m_logScaleFactor + static_cast<double>( m_slider->GetMin() );
        }
        else
        {
            sliderValue = static_cast<int>( m_value / ( m_increment * m_sliderCorrectionFactor ) );
        }
        if( sliderValue != m_slider->GetValue() )
        {
            m_slider->SetValue( sliderValue );
        }
    }
}

void wxSpinCtrlDbl::SetValue( const wxString& text, bool /*force*/ )
{
    if( !m_textCtrl )
    {
        return;
    }

    SetValue( ToDouble( m_mode, text, m_textFormat ) );
}

void wxSpinCtrlDbl::SetRange( double min_val, double max_val )
{
    //wxCHECK_RET(max_val > min_val, wxT("invalid spinctrl range"));
    m_min = min_val;
    m_max = max_val;

    if( HasRange() )
    {
        if( m_value > m_max )
        {
            SetValue( m_max );
        }
        else if( m_value < m_min )
        {
            SetValue( m_min );
        }
    }
}

void wxSpinCtrlDbl::SetIncrement( double increment )
{
    m_increment = increment;
    SetValue( m_value );
}

void wxSpinCtrlDbl::SetDigits( int digits, formatType fmt )
{
    wxCHECK_RET( digits >= -1, wxT( "invalid spinctrl format" ) );

    if( ( digits == wxSPINCTRLDBL_AUTODIGITS ) && ( fmt != lg_fmt ) )
    {
        wxString wxstr;
        int lastplace = -1, extra_digits = 0;
        if( fmt == le_fmt )
        {
            wxstr.Printf( wxT( "%ge" ), m_increment );
            wxstr.LowerCase();
            lastplace = wxstr.Find( wxT( 'e' ) ) - 2;
            long places;
            if( wxstr.AfterFirst( wxT( 'e' ) ).ToLong( &places ) )
            {
                extra_digits = int( labs( places ) );
            }
        }
        else if( fmt == lf_fmt )
        {
            wxstr.Printf( wxT( "%gf" ), m_increment );
            lastplace = static_cast<int>( wxstr.Len() - 1 );
        }

        int decimalplace = wxstr.Find( wxT( '.' ) );
        for ( int i = lastplace; i > decimalplace; i-- )
        {
            if( wxstr.GetChar( i ) != wxT( '0' ) )
            {
                m_digits = extra_digits + i - decimalplace;
                switch ( fmt )
                {
                case le_fmt :
                    m_textFormat.Printf( wxT( "%%.%dle" ), m_digits );
                    break;
                case lf_fmt :
                default     :
                    m_textFormat.Printf( wxT( "%%.%dlg" ), m_digits );
                    break;
                }
                SetValue( m_value );
                return;
            }
        }
        m_digits = 0; // no digits, I guess
    }
    else
    {
        m_digits = digits;
    }
    switch ( fmt )
    {
    case le_fmt :
        m_textFormat.Printf( wxT( "%%.%dle" ), m_digits );
        break;
    case lg_fmt :
        {
            if( m_digits == -1 )
            {
                m_textFormat.Printf( wxT( "%%lg" ) );
            }
            else
            {
                m_textFormat.Printf( wxT( "%%.%dlg" ), m_digits );
            }
            break;
        }
    case custom_fmt:
        break;
    case lf_fmt :
    default     :
        m_textFormat.Printf( wxT( "%%.%dlf" ), m_digits );
        break;
    }

    SetValue( m_value );
}

void wxSpinCtrlDbl::SetFormat( const wxString& format )
{
    wxString wxstr;
    if( wxstr.Printf( format.c_str(), 123456.123456 ) > 0 )
    {
        m_textFormat = format;
    }

    SetValue( m_value );
}

void wxSpinCtrlDbl::SetDefaultValue( double default_value )
{
    if( InRange( default_value ) )
    {
        m_default_value = default_value;
        SetDefaultValue();
    }
}

void wxSpinCtrlDbl::SetSnapToTicks( bool forceTicks )
{
    if( m_snap_ticks != forceTicks )
    {
        m_snap_ticks = forceTicks;
        SetValue( m_value );
    }
}

void wxSpinCtrlDbl::OnFocus( wxFocusEvent& e )
{
    if( m_textCtrl )
    {
        m_default_value = m_value;
        m_textCtrl->SetFocus(); // this is to pass TAB navigation
    }
    e.Skip();
}

void wxSpinCtrlDbl::OnKillFocus( wxFocusEvent& e )
{
    SyncSpinToText( true );
    e.Skip();
}

void wxSpinCtrlDbl::SyncSpinToText( bool send_event, bool force_valid )
{
    if( !m_textCtrl )
    {
        return;
    }

    double txt_value = ToDouble( m_mode, m_textCtrl->GetValue(), m_textFormat );
    if( force_valid || !HasRange() || InRange( txt_value ) )
    {
        if( force_valid && HasRange() )
        {
            if( txt_value > GetMax() )
            {
                txt_value = GetMax();
            }
            else if( txt_value < GetMin() )
            {
                txt_value = GetMin();
            }
        }

        if( m_value != txt_value )
        {
            SetValue( txt_value );
            if( send_event )
            {
                DoSendEvent();
            }
        }
    }
    else if( force_valid )
    {
        // textctrl is out of sync, discard and reset
        SetValue( GetValue() );
    }
}

void wxSpinCtrlDbl::SyncSpinToTextFromKeyboardInput( const double inc /* = 0. */ )
{
    if( m_textCtrl && m_textCtrl->IsModified() )
    {
        SyncSpinToText( false );
    }
    SetValue( m_value + inc );
    DoSendEvent();
}

bool wxSpinCtrlDbl::SetFont( const wxFont& font )
{
    if( !m_textCtrl )
    {
        return false;
    }
    return m_textCtrl->SetFont( font );
}

wxFont wxSpinCtrlDbl::GetFont() const
{
    if( !m_textCtrl )
    {
        return GetFont();
    }
    return m_textCtrl->GetFont();
}

bool wxSpinCtrlDbl::SetBackgroundColour( const wxColour& colour )
{
    if( !m_textCtrl )
    {
        return wxControl::SetBackgroundColour( colour );
    }
    bool ret = false;
    ret = m_textCtrl->SetBackgroundColour( colour );
    m_textCtrl->Refresh(); // FIXME is this necessary in GTK/OSX
    return ret;
}

wxColour wxSpinCtrlDbl::GetBackgroundColour() const
{
    if( !m_textCtrl )
    {
        return wxControl::GetBackgroundColour();
    }
    return m_textCtrl->GetBackgroundColour();
}

bool wxSpinCtrlDbl::SetForegroundColour( const wxColour& colour )
{
    if( !m_textCtrl )
    {
        return wxControl::SetForegroundColour( colour );
    }
    bool ret = false;
    ret = m_textCtrl->SetForegroundColour( colour );
    m_textCtrl->Refresh();
    return ret;
}

wxColour wxSpinCtrlDbl::GetForegroundColour() const
{
    if( !m_textCtrl )
    {
        return wxControl::GetForegroundColour();
    }
    return m_textCtrl->GetForegroundColour();
}
