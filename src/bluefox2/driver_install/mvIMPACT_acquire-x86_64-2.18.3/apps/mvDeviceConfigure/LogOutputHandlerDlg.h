//-----------------------------------------------------------------------------
#ifndef LogOutputHandlerH
#define LogOutputHandlerH LogOutputHandlerH
//-----------------------------------------------------------------------------
#include <set>
#include <string>
#include "DebugFileParser.h"
#include "DeviceListCtrl.h"
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "wx/wx.h"

//-----------------------------------------------------------------------------
class LogOutputHandlerDlg : public wxDialog
//-----------------------------------------------------------------------------
{
    typedef std::set<std::string> StringSet;

    wxButton*                           m_pBtnOpen;
    wxButton*                           m_pBtnSave;
    LogOutputListCtrl*                  m_pLogListCtrl;
    wxTextCtrl*                         m_pLogWindow;
    mvIMPACT::acquire::DeviceManager&   m_devMgr;
    unsigned int                        m_devMgrLastChangedCounter;
    LogConfigurationVector&             m_debugData;
    wxTimer                             m_logConfigTimer;
    StringSet                           m_missingConfigs;
    //-----------------------------------------------------------------------------
    enum TWidgetIDs
    //-----------------------------------------------------------------------------
    {
        widBtnLoad = 1,
        widBtnSave
    };
    //-----------------------------------------------------------------------------
    enum TTimerEvent
    //-----------------------------------------------------------------------------
    {
        teUpdate
    };
    void        BuildList( void );
    std::string GetAssociatedDevicesString( const std::string& sectionName );
    void        OnBtnLoad( wxCommandEvent& );
    void        OnBtnSave( wxCommandEvent& );
    void        OnTimer( wxTimerEvent& e );
    void        SetupColumn( long index );
    void        UpdateMissingConfigsList( bool boForceRebuild = false );
    void        WriteErrorMessage( const wxString& msg );
    void        WriteLogMessage( const wxString& msg, const wxTextAttr& style = wxTextAttr( wxColour( 0, 0, 0 ) ) );
public:
    LogOutputHandlerDlg( wxWindow* pParent, mvIMPACT::acquire::DeviceManager& devMgr, LogConfigurationVector& debugData );
    void ConfigureLogger( int index );
    void DeleteItem( int index );
    void DeleteList( void );
    void InsertItem( int index );
    // any class wishing to process wxWidgets events must use this macro
    DECLARE_EVENT_TABLE()
};

#endif // LogOutputHandlerH
