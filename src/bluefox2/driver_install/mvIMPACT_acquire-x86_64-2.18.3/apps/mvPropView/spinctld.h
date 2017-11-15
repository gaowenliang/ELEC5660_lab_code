/////////////////////////////////////////////////////////////////////////////
// Name:        spinctld.h
// Purpose:     A double valued spin control, compatible with wxSpinCtrl
// Author:      John Labenski
// Created:     11/05/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

/*
wxSpinCtrlDbl is a double valued wxSpinCtrl using non native (wxWidgets) widgets

It consists of a wxSpinButton and a wxTextCtrl on a wxControl and can be used
as a drop in replacement for the wxSpinCtrl. It's been tested in GTK and MSW
and should work with MAC, but you may need to fix the sizing perhaps. It
will not work in Motif as there is no wxSpinButton in Motif.

Differences to wxSpinCtrl:

    It remembers the initial value as a default value, call SetDefaultValue,
        or press <ESC> to return to it

    Shift + Arrow = *2 increment value
    Ctrl  + Arrow = *10 increment value
    Alt   + Arrow = *100 increment value
    combinations of Shift, Ctrl, Alt increment by the product of the factors

    PgUp & PgDn = *10 increment * the product of the Shift, Ctrl, Alt factors

    <SPACE> sets the control's value to the its last valid state

    SetDigits controls the format of the text, # decimal places
        exponential uses the %.Xle format otherwise %.Xlf, where places = X
        for arbitrary formats subclass control and override SyncSpinToText()
        for proper behavior when a user types in a value
*/

#ifndef __wxSPINCTRLDBL_H__
#define __wxSPINCTRLDBL_H__

#if defined(__GNUG__) && !defined(NO_GCC_PRAGMA)
#pragma interface "spinctld.h"
#endif

#include "wx/slider.h"
#include "wx/spinbutt.h"
#include "wx/spinctrl.h" // for EVT_SPINCTRL
#include "thingdef.h"

enum TMode
{
    mDouble,
    mInt,
    mInt64
};

WXDLLIMPEXP_THINGS double ToDouble( TMode mode, const wxString& valStr, const wxString& format );
wxString ToString( TMode mode, double value, const wxString& format );

#if wxCHECK_VERSION(2, 9, 0)
class WXDLLIMPEXP_FWD_CORE wxTextCtrl; // see the comment in dlimpexp.h
#else
class WXDLLEXPORT wxTextCtrl;
#endif
class WXDLLIMPEXP_THINGS wxSpinCtrlDblTextCtrl;

enum
{
    wxSPINCTRLDBL_AUTODIGITS = -1  // try to autocalc the # of digits
};

class WXDLLIMPEXP_THINGS wxSpinCtrlDbl: public wxControl
{
    //-----------------------------------------------------------------------------
    enum TCustomWindowIDs
    //-----------------------------------------------------------------------------
    {
        widSlider = 1
    };
public:
    wxSpinCtrlDbl() : wxControl()
    {
        Init();
    }

    // Native constructor - note &parent, this is to avoid ambiguity
    wxSpinCtrlDbl( wxWindow& parent, wxWindowID id,
                   const wxString& value = wxEmptyString,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxSize( 95, -1 ),
                   long style = 0,
                   double min = 0.0, double max = 100.0,
                   double initial = 0.0,
                   double increment = 1.0, int digits = wxSPINCTRLDBL_AUTODIGITS,
                   const wxString& name = wxT( "wxSpinCtrlDbl" ),
                   bool boWithSlider = false )
    {
        Init();
        Create( &parent, id, value, pos, size, style,
                min, max, initial, increment, digits, wxT( "" ), name, boWithSlider );
    }

    // wxSpinCtrl compatibility, call SetIncrement(increment,digits) after
    wxSpinCtrlDbl( wxWindow* parent, wxWindowID id = wxID_ANY,
                   const wxString& value = wxEmptyString,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxSize( 95, -1 ),
                   long style = 0,
                   int min = 0, int max = 100,
                   int initial = 0,
                   const wxString& name = wxT( "wxSpinCtrlDbl" ),
                   bool boWithSlider = false )
    {
        Init();
        Create( parent, id, value, pos, size, style,
                ( double )min, ( double )max, ( double )initial, 1.0, wxSPINCTRLDBL_AUTODIGITS, wxT( "" ), name, boWithSlider );
    }

    bool Create( wxWindow* parent,
                 wxWindowID id = wxID_ANY,
                 const wxString& value = wxEmptyString,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxSize( 100, -1 ),
                 long style = 0,
                 double min = 0.0, double max = 100.0,
                 double initial = 0.0,
                 double increment = 1.0, int digits = wxSPINCTRLDBL_AUTODIGITS,
                 const wxString& format = wxT( "" ),
                 const wxString& name = wxT( "wxSpinCtrlDbl" ),
                 bool boWithSlider = false,
                 bool boSliderWithLogarithmicBehaviour = false );

    virtual ~wxSpinCtrlDbl();

    // -----------------------------------------------------------------------
    // Public (normal usage) functions
    enum formatType
    {
        lf_fmt, // %lf
        le_fmt, // %le
        lg_fmt, // %lg
        custom_fmt
    };

    virtual void SetValue( double value );
    void SetValue( double value, double min, double max, double increment,
                   int digits = wxSPINCTRLDBL_AUTODIGITS, formatType fmt = lg_fmt )
    {
        SetRange( min, max );
        SetIncrement( increment );
        SetDigits( digits, fmt );
        SetValue( value );
    }
    // Set the value as text, if force then set text as is
    virtual void SetValue( const wxString& text, bool force );
    // Set the allowed range, if max_val < min_val then no range and all vals allowed.
    void SetRange( double min_val, double max_val );
    // Set the increment to use when the spin button or arrow keys pressed.
    void SetIncrement( double increment );
    void SetIncrement( double increment, int digits, formatType fmt = lg_fmt )
    {
        SetIncrement( increment );
        SetDigits( digits, fmt );
    }
    // Set the number of digits to show, use wxSPINCTRLDBL_AUTODIGITS
    //  or specify exact number to show i.e. %.[digits]lf
    //  The format type is used to create an appropriate format string.
    void SetDigits( int digits = wxSPINCTRLDBL_AUTODIGITS, formatType fmt = lg_fmt );
    // Set the format string to use, ie. format="%.2lf" for .01
    void SetFormat( const wxString& format );
    // Set the control the default value.
    virtual void SetDefaultValue()
    {
        SetValue( m_default_value );
    }
    // Set the value of the default value, default is the initial value.
    void SetDefaultValue( double default_value );
    // Force the value to always be divisible by the increment, initially off.
    //   This uses the default_value as the basis, you'll get strange results
    //   for very large differences between the current value and default value
    //   when the increment is very small.
    void SetSnapToTicks( bool forceTicks );

    double   GetValue( void ) const
    {
        return m_value;
    }
    double   GetMin( void ) const
    {
        return m_min;
    }
    double   GetMax( void ) const
    {
        return m_max;
    }
    virtual bool HasRange( void ) const
    {
        return m_max >= m_min;
    }
    virtual bool InRange( double value ) const
    {
        return !HasRange() || ( ( value >= m_min ) && ( value <= m_max ) );
    }
    double   GetIncrement( void ) const
    {
        return m_increment;
    }
    int      GetDigits( void ) const
    {
        return m_digits;
    }
    wxString GetFormat( void ) const
    {
        return m_textFormat;
    }
    double   GetDefaultValue( void ) const
    {
        return m_default_value;
    }
    bool     GetSnapToTicks( void ) const
    {
        return m_snap_ticks;
    }

    bool IsDefaultValue( void ) const
    {
        return ( m_value == m_default_value );
    }

    bool   SetFont( const wxFont& font );
    wxFont GetFont( void ) const;

    virtual bool SetBackgroundColour( const wxColour& colour );
    wxColour GetBackgroundColour( void ) const;

    virtual bool SetForegroundColour( const wxColour& colour );
    wxColour GetForegroundColour( void ) const;

    // for setting... stuff
    wxTextCtrl* GetTextCtrl( void )
    {
        return ( wxTextCtrl* )m_textCtrl;
    }

    TMode GetMode( void ) const
    {
        return m_mode;
    }
    void SetMode( TMode mode )
    {
        m_mode = mode;
    }
protected:
    void OnScrollThumbtrack( wxScrollEvent& e );
    void OnSpinUp( wxSpinEvent& e );
    void OnSpinDown( wxSpinEvent& e );
    void OnTextEnter( wxCommandEvent& e );
    void OnText( wxCommandEvent& e );
    // the textctrl is subclassed to get at pgup/dn and then sent here
    void OnChar( wxKeyEvent& e );

    virtual void SyncSpinToText( bool send_event = true, bool force_valid = true );
    virtual void SyncSpinToTextFromKeyboardInput( const double inc = 0. );

    void DoSendEvent( void );

    virtual void DoSetSize( int x, int y, int width, int height, int sizeFlags = wxSIZE_AUTO );

    virtual wxSize DoGetBestSize( void ) const;
    virtual void DoSetToolTip( wxToolTip* tip );

    void OnFocus( wxFocusEvent& e ); // pass focus to textctrl, for wxTAB_TRAVERSAL
    void OnKillFocus( wxFocusEvent& e );

    wxSpinButton*          m_spinButton;
    wxSlider*              m_slider;
    bool                   m_boSliderWithLogarithmicBehaviour;
    double                 m_logScaleFactor;
    wxSpinCtrlDblTextCtrl* m_textCtrl;

    double   m_min;           // min allowed value
    double   m_max;           // max allowed value
    double   m_value;         // current value
    double   m_default_value; // initial value, or SetDefaultValue(value)
    double   m_sliderCorrectionFactor; // a wxSlider can only cope with a value range that fits into 31(!) bit of an int
    double   m_increment;     // how much to to add per spin
    int      m_digits;        // number of digits displayed after decimal point
    bool     m_snap_ticks;    // value is divisible by increment
    wxString m_textFormat;    // used as wxString.Printf(m_textFormat.c_str(), m_value);
    int      m_nRepeatCount;
    clock_t  m_LastChange;
    TMode    m_mode;
private:
    friend class wxSpinCtrlDblTextCtrl;

    double GetStep( void );
    void Init( void );
    DECLARE_DYNAMIC_CLASS( wxSpinCtrlDbl )
    DECLARE_EVENT_TABLE()
};

#endif  // __wxSPINCTRLDBL_H__

