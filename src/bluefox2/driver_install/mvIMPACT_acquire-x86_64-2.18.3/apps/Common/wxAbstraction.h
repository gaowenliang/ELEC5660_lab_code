//-----------------------------------------------------------------------------
#ifndef wxAbstractionH
#define wxAbstractionH wxAbstractionH
//-----------------------------------------------------------------------------
#include <apps/Common/Info.h>
#include <apps/Common/mvIcon.xpm>
#include <string>
#include <wx/combobox.h>
#include <wx/config.h>
#include <wx/dynlib.h>
#include <wx/gdicmn.h>
#include <wx/hyperlink.h>
#include <wx/log.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/splash.h>
#include <wx/stattext.h>
#include <wx/stopwatch.h>
#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/window.h>

// helper class
struct ConvertedString : wxString
{
#if wxUSE_UNICODE
    ConvertedString( const char* s ) :
        wxString( s, wxConvUTF8 ) {};
    ConvertedString( const std::string& s ) :
        wxString( s.c_str(), wxConvUTF8 ) {};
    ConvertedString( const wxString& s ) :
# if wxCHECK_VERSION(2,9,0)
        wxString( s.mb_str(), wxConvUTF8 ) {};
# else
        wxString( s.c_str(), wxConvUTF8 ) {};
# endif
#else
    ConvertedString( const char* s ) :
        wxString( s ) {};
    ConvertedString( const std::string& s ) :
        wxString( s.c_str() ) {};
    ConvertedString( const wxString& s ) :
# if wxCHECK_VERSION(2,9,0)
        wxString( s.mb_str() ) {};
# else
        wxString( s.c_str() ) {};
# endif
#endif
};

#if ( wxMINOR_VERSION > 6 ) && ( wxMAJOR_VERSION < 3 ) && !WXWIN_COMPATIBILITY_2_6
#include <wx/filedlg.h>
enum
{
    wxOPEN              = wxFD_OPEN,
    wxSAVE              = wxFD_SAVE,
    wxOVERWRITE_PROMPT  = wxFD_OVERWRITE_PROMPT,
    wxFILE_MUST_EXIST   = wxFD_FILE_MUST_EXIST,
    wxMULTIPLE          = wxFD_MULTIPLE,
    wxCHANGE_DIR        = wxFD_CHANGE_DIR
};
#endif

//=============================================================================
//================= Implementation FramePositionStorage =======================
//=============================================================================
//-----------------------------------------------------------------------------
class FramePositionStorage
//-----------------------------------------------------------------------------
{
    wxWindow* m_pWin;
public:
    FramePositionStorage( wxWindow* pWin ) : m_pWin( pWin ) {}
    void Save( void ) const
    {
        Save( m_pWin );
    }
    static void Save( wxWindow* pWin, const wxString& windowName = wxT( "MainFrame" ) )
    {
        wxConfigBase* pConfig( wxConfigBase::Get() );
        int Height, Width, XPos, YPos;
        pWin->GetSize( &Width, &Height );
        pWin->GetPosition( &XPos, &YPos );
        // when we e.g. try to write config stuff on a read-only file system the result can
        // be an annoying message box. Therefore we switch off logging during the storage operation.
        wxLogNull logSuspendScope;
        pConfig->Write( wxString::Format( wxT( "/%s/h" ), windowName.c_str() ), Height );
        pConfig->Write( wxString::Format( wxT( "/%s/w" ), windowName.c_str() ), Width );
        pConfig->Write( wxString::Format( wxT( "/%s/x" ), windowName.c_str() ), XPos );
        pConfig->Write( wxString::Format( wxT( "/%s/y" ), windowName.c_str() ), YPos );
        if( dynamic_cast<wxTopLevelWindow*>( pWin ) )
        {
            pConfig->Write( wxString::Format( wxT( "/%s/maximized" ), windowName.c_str() ), dynamic_cast<wxTopLevelWindow*>( pWin )->IsMaximized() );
        }
        pConfig->Flush();
    }
    static wxRect Load( const wxRect& defaultDimensions, bool& boMaximized, const wxString& windowName = wxT( "MainFrame" ) )
    {
        wxConfigBase* pConfig( wxConfigBase::Get() );
        wxRect rect;
        rect.height = pConfig->Read( wxString::Format( wxT( "/%s/h" ), windowName.c_str() ), defaultDimensions.height );
        rect.width = pConfig->Read( wxString::Format( wxT( "/%s/w" ), windowName.c_str() ), defaultDimensions.width );
        rect.x = pConfig->Read( wxString::Format( wxT( "/%s/x" ), windowName.c_str() ), defaultDimensions.x );
        rect.y = pConfig->Read( wxString::Format( wxT( "/%s/y" ), windowName.c_str() ), defaultDimensions.y );
        boMaximized = pConfig->Read( wxString::Format( wxT( "/%s/maximized" ), windowName.c_str() ), 1l ) != 0;
        int displayWidth = 0;
        int displayHeight = 0;
        wxDisplaySize( &displayWidth, &displayHeight );
        if( ( rect.x >= displayWidth ) || ( ( rect.x + rect.width ) < 0 ) )
        {
            rect.x = 0;
        }
        if( ( rect.y >= displayHeight ) || ( ( rect.y + rect.height ) < 0 ) )
        {
            rect.y = 0;
        }
        return rect;
    }
};

//=============================================================================
//================= Implementation SplashScreenScope ==========================
//=============================================================================
//-----------------------------------------------------------------------------
class SplashScreenScope
//-----------------------------------------------------------------------------
{
    wxSplashScreen* pSplash_;
    wxStopWatch stopWatch_;
public:
    explicit SplashScreenScope( const wxBitmap& bmp ) : pSplash_( 0 ), stopWatch_()
    {
        pSplash_ = new wxSplashScreen( ( wxSystemSettings::GetScreenType() <= wxSYS_SCREEN_SMALL ) ? mvIcon_xpm : bmp, wxSPLASH_CENTRE_ON_SCREEN | wxSPLASH_NO_TIMEOUT, 0, NULL, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );
        pSplash_->Update();
    }
    virtual ~SplashScreenScope()
    {
        const unsigned long startupTime_ms = static_cast<unsigned long>( stopWatch_.Time() );
        const unsigned long minSplashDisplayTime_ms = 1000;
        if( startupTime_ms < minSplashDisplayTime_ms )
        {
            wxMilliSleep( minSplashDisplayTime_ms - startupTime_ms );
        }
        delete pSplash_;
    }
};

//-----------------------------------------------------------------------------
inline wxString LoadGenTLProducer( wxDynamicLibrary& lib )
//-----------------------------------------------------------------------------
{
#ifdef _WIN32
    const wxString libName( wxT( "mvGenTLProducer.cti" ) );
#else
    const wxString libName( wxT( "libmvGenTLProducer.so" ) );
#endif

    wxString message;
    // when we e.g. trying to load a shared library that cannot be found the result can
    // be an annoying message box. Therefore we switch off logging during the load attempts.
    wxLogNull logSuspendScope;
    lib.Load( libName, wxDL_VERBATIM );
    if( !lib.IsLoaded() )
    {
        message = wxString::Format( wxT( "Could not connect to '%s'. Check your installation.\n\n" ), libName.c_str() );
    }
    return message;
}

//-----------------------------------------------------------------------------
inline void AddSourceInfo( wxWindow* pParent, wxSizer* pParentSizer )
//-----------------------------------------------------------------------------
{
    wxBoxSizer* pSizer = new wxBoxSizer( wxHORIZONTAL );
    pSizer->Add( new wxStaticText( pParent, wxID_ANY, wxT( "The complete source of this application can be obtained by contacting " ) ) );
    pSizer->Add( new wxHyperlinkCtrl( pParent, wxID_ANY, COMPANY_NAME, COMPANY_WEBSITE ) );
    pParentSizer->Add( pSizer, 0, wxALL | wxALIGN_CENTER, 5 );
}

//-----------------------------------------------------------------------------
inline void AddSupportInfo( wxWindow* pParent, wxSizer* pParentSizer )
//-----------------------------------------------------------------------------
{
    wxBoxSizer* pSizer = new wxBoxSizer( wxHORIZONTAL );
    pSizer->Add( new wxStaticText( pParent, wxID_ANY, wxT( "Support contact: " ) ) );
    pSizer->Add( new wxHyperlinkCtrl( pParent, wxID_ANY, COMPANY_SUPPORT_MAIL, COMPANY_SUPPORT_MAIL ) );
    pParentSizer->Add( pSizer, 0, wxALL | wxALIGN_CENTER, 5 );
}

//-----------------------------------------------------------------------------
inline void AddwxWidgetsInfo( wxWindow* pParent, wxSizer* pParentSizer )
//-----------------------------------------------------------------------------
{
    wxBoxSizer* pSizer = new wxBoxSizer( wxHORIZONTAL );
    pSizer->Add( new wxStaticText( pParent, wxID_ANY, wxT( "This tool has been written using " ) ) );
    pSizer->Add( new wxHyperlinkCtrl( pParent, wxID_ANY, wxT( "wxWidgets" ), wxT( "http://www.wxwidgets.org" ) ) );
    pSizer->Add( new wxStaticText( pParent, wxID_ANY, wxString::Format( wxT( " %d.%d.%d." ), wxMAJOR_VERSION, wxMINOR_VERSION, wxRELEASE_NUMBER ) ) );
    pParentSizer->Add( pSizer, 0, wxALL | wxALIGN_CENTER, 5 );
}

//-----------------------------------------------------------------------------
inline void AddIconInfo( wxWindow* pParent, wxSizer* pParentSizer )
//-----------------------------------------------------------------------------
{
    wxBoxSizer* pSizer = new wxBoxSizer( wxHORIZONTAL );
    pSizer->Add( new wxStaticText( pParent, wxID_ANY, wxT( "This tool uses modified icons downloaded from here " ) ) );
    pSizer->Add( new wxHyperlinkCtrl( pParent, wxID_ANY, wxT( "icons8" ), wxT( "https://icons8.com/" ) ) );
    pSizer->Add( new wxStaticText( pParent, wxID_ANY, wxT( " and here " ) ) );
    pSizer->Add( new wxHyperlinkCtrl( pParent, wxID_ANY, wxT( "Freepik" ), wxT( "http://www.freepik.com" ) ) );
    pParentSizer->Add( pSizer, 0, wxALL | wxALIGN_CENTER, 5 );
}

//-----------------------------------------------------------------------------
inline void AppendPathSeparatorIfNeeded( wxString& path )
//-----------------------------------------------------------------------------
{
    if( !path.EndsWith( wxT( "/" ) ) && !path.EndsWith( wxT( "\\" ) ) )
    {
        path.append( wxT( "/" ) );
    }
}

//-----------------------------------------------------------------------------
inline wxTextAttr GetBoldStyle( wxTextCtrl*
#if wxCHECK_VERSION(2, 9, 5)
#else
                                pParent
#endif // #if wxCHECK_VERSION(2, 9, 5)
                              )
//-----------------------------------------------------------------------------
{
    wxTextAttr boldStyle;
#if wxCHECK_VERSION(2, 9, 5)
    wxFont boldFont( wxFontInfo( 10 ).Bold().Underlined() );
#else
    pParent->GetStyle( pParent->GetLastPosition(), boldStyle );
    wxFont boldFont( boldStyle.GetFont() );
    boldFont.SetWeight( wxFONTWEIGHT_BOLD );
    boldFont.SetPointSize( 10 );
    boldFont.SetUnderlined( true );
#endif // #if wxCHECK_VERSION(2, 9, 5)
    boldStyle.SetFont( boldFont );
    return boldStyle;
}

//-----------------------------------------------------------------------------
inline bool IsListOfChoicesEmpty( wxComboBox* pCB )
//-----------------------------------------------------------------------------
{
#if wxCHECK_VERSION(2, 9, 3)
    return pCB->IsListEmpty();
#else
    return pCB->IsEmpty();
#endif // #if wxCHECK_VERSION(2, 9, 3)
}

#endif // wxAbstractionH
