//-----------------------------------------------------------------------------
#include <apps/Common/wxAbstraction.h>
#include <common/expat/expatHelper.h>
#include "LogOutputHandlerDlg.h"
#include "LogOutputConfigurationDlg.h"
#include <wx/ffile.h>
#include <wx/splitter.h>

using namespace std;
using namespace mvIMPACT::acquire;

//=============================================================================
//================== Implementation LogOutputHandlerDlg =======================
//=============================================================================

BEGIN_EVENT_TABLE( LogOutputHandlerDlg, wxDialog )
    EVT_BUTTON( widBtnLoad, LogOutputHandlerDlg::OnBtnLoad )
    EVT_BUTTON( widBtnSave, LogOutputHandlerDlg::OnBtnSave )
    EVT_TIMER( wxID_ANY, LogOutputHandlerDlg::OnTimer )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
LogOutputHandlerDlg::LogOutputHandlerDlg( wxWindow* pParent, DeviceManager& devMgr, LogConfigurationVector& debugData )
    : wxDialog( pParent, wxID_ANY, wxString( wxT( "Log Output Configuration" ) ), wxDefaultPosition,
                wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER | wxMAXIMIZE_BOX | wxMINIMIZE_BOX ),
    m_devMgr( devMgr ), m_devMgrLastChangedCounter( 0 ), m_debugData( debugData )
//-----------------------------------------------------------------------------
{
    wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    wxPanel* pPanel = new wxPanel( this );

    // splitter for list control and output console
    wxSplitterWindow* pHorizontalSplitter = new wxSplitterWindow( pPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSIMPLE_BORDER );
    pTopDownSizer->Add( pHorizontalSplitter, wxSizerFlags( 5 ).Expand() );

    // list control
    m_pLogListCtrl = new LogOutputListCtrl( pHorizontalSplitter, LIST_CTRL, wxDefaultPosition, wxDefaultSize, wxLC_REPORT | wxLC_SINGLE_SEL | wxBORDER_NONE, this );
    m_pLogListCtrl->InsertColumn( llcSectionName, wxT( "Section Name" ) );
    m_pLogListCtrl->InsertColumn( llcDevicesUsingThisSection, wxT( "Devices Using This Section" ) );
    m_pLogListCtrl->InsertColumn( llcFlags, wxT( "Flags" ) );
    m_pLogListCtrl->InsertColumn( llcOutputMask, wxT( "Output Destinations" ) );
    m_pLogListCtrl->InsertColumn( llcOutputFile, wxT( "Output File Name" ) );
    m_pLogListCtrl->InsertColumn( llcClearFile, wxT( "Always clear File" ) );
    m_pLogListCtrl->InsertColumn( llcFileFormat, wxT( "Output Format" ) );
    m_pLogListCtrl->InsertColumn( llcStylesheet, wxT( "Stylesheet Used" ) );
    BuildList();

    // text control for information output
    m_pLogWindow = new wxTextCtrl( pHorizontalSplitter, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxBORDER_NONE | wxTE_RICH | wxTE_READONLY );

    // splitter...
    pHorizontalSplitter->SetSashGravity( 0.8 );
    pHorizontalSplitter->SetMinimumPaneSize( 50 );

    pTopDownSizer->AddSpacer( 10 );
    // line of buttons and controls
    wxBoxSizer* pButtonSizer = new wxBoxSizer( wxHORIZONTAL );
    m_pBtnOpen = new wxButton( pPanel, widBtnLoad, wxT( "&Load..." ) );
    pButtonSizer->Add( m_pBtnOpen, wxSizerFlags().Border( wxALL, 7 ) );
    m_pBtnSave = new wxButton( pPanel, widBtnSave, wxT( "&Save..." ) );
    pButtonSizer->Add( m_pBtnSave, wxSizerFlags().Border( wxALL, 7 ) );
    pTopDownSizer->Add( pButtonSizer );

    pPanel->SetSizer( pTopDownSizer );
    pTopDownSizer->SetSizeHints( this );
    SetSizeHints( 750, 500 );
    SetSize( 750, 500 );

    pHorizontalSplitter->SplitHorizontally( m_pLogListCtrl, m_pLogWindow );

    const wxTextAttr boldStyle( GetBoldStyle( m_pLogWindow ) );
    WriteLogMessage( wxT( "Use the 'Load' button to load an existing configuration.\n" ) );
    WriteLogMessage( wxT( "Use the 'Save' button to store the current configuration to disc.\n" ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "Windows:\n" ), boldStyle );
    WriteLogMessage( wxT( "A log configuration MUST be stored under %ALL_USERS%/Documents/MATRIX VISION/mvIMPACT Acquire/Logs in order to be evaluated at runtime.\n" ) );
    WriteLogMessage( wxT( "From this location the current configuration should be opened for modifications.\n" ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "Linux:\n" ), boldStyle );
    WriteLogMessage( wxT( "A log configuration MUST be stored in the applications directory in order to be evaluated at runtime.\n" ) );
    WriteLogMessage( wxT( "The name of the file for runtime evaluation in any case MUST be 'mvDebugFlags.mvd'!\n" ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteLogMessage( wxT( "See the corresponding device manual (chapter 'logging') for additional information!\n" ) );
    WriteLogMessage( wxT( "\n" ) );
    WriteErrorMessage( wxT( "WARNING: Modifications applied in this dialog will be lost when they haven't been saved before the application is terminated!\n" ) );
    m_logConfigTimer.SetOwner( this, teUpdate );
    m_logConfigTimer.Start( 500 );
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::BuildList( void )
//-----------------------------------------------------------------------------
{
    m_pLogListCtrl->DeleteAllItems();
    LogConfigurationVector::size_type vSize = m_debugData.size();
    for( LogConfigurationVector::size_type i = 0; i < vSize; i++ )
    {
        long index = m_pLogListCtrl->InsertItem( static_cast<long>( i ), ConvertedString( m_debugData[i].sectionName ) );
        m_pLogListCtrl->SetItemData( index, static_cast<long>( i ) );
        SetupColumn( index );
    }

    for( unsigned int i = 0; i < llcLAST_COLUMN; i++ )
    {
        m_pLogListCtrl->SetColumnWidth( i, wxLIST_AUTOSIZE_USEHEADER );
    }
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::ConfigureLogger( int index )
//-----------------------------------------------------------------------------
{
    if( ( index < 0 ) || m_debugData.empty() ||
        ( static_cast<LogConfigurationVector::size_type>( index ) > m_debugData.size() - 1 ) )
    {
        return;
    }

    LogOutputConfiguration res = m_debugData[index];
    LogWriterConfigurationDlg dlg( this, res );
    if( dlg.ShowModal() == wxID_OK )
    {
        m_debugData[index] = res;
        SetupColumn( index );
    }
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::DeleteItem( int index )
//-----------------------------------------------------------------------------
{
    if( ( index < 0 ) || m_debugData.empty() ||
        ( static_cast<LogConfigurationVector::size_type>( index ) > m_debugData.size() - 1 ) )
    {
        return;
    }

    m_debugData.erase( m_debugData.begin() + index );
    BuildList();
    UpdateMissingConfigsList( true );
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::DeleteList( void )
//-----------------------------------------------------------------------------
{
    m_debugData.clear();
    UpdateMissingConfigsList( true );
}

//-----------------------------------------------------------------------------
string LogOutputHandlerDlg::GetAssociatedDevicesString( const string& sectionName )
//-----------------------------------------------------------------------------
{
    if( ( sectionName == "mvPropHandling" ) ||
        ( sectionName == "mvDeviceManager" ) )
    {
        return "( will be used by every device )";
    }

    unsigned int devCnt = m_devMgr.deviceCount();
    string associatedDevices;
    for( unsigned int i = 0; i < devCnt; i++ )
    {
        Device* pDev = m_devMgr[i];
        if( pDev )
        {
            string family = pDev->family.read();
            ostringstream oss;
            oss << family << "-";
            oss.fill( '0' );
            oss.width( 3 );
            oss << pDev->deviceID.read();
            if( ( family == sectionName ) ||
                ( oss.str() == sectionName ) )
            {
                if( associatedDevices.length() > 0 )
                {
                    associatedDevices.append( ", " );
                }
                else
                {
                    associatedDevices.append( "( will be used by " );
                }
                associatedDevices.append( pDev->serial.read().c_str() );
            }
        }
    }
    if( associatedDevices.length() > 0 )
    {
        associatedDevices.append( " )" );
    }
    return associatedDevices;
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::InsertItem( int index )
//-----------------------------------------------------------------------------
{
    if( index < 0 )
    {
        index = 0;
    }
    LogOutputConfiguration res;
    res.outputDestinationMask = TDebugOutputDestination( doSystem | doFile );
    res.outputFlagMask = "1111100";
    res.boClearFile = true;
    res.outputFileName = "STDLOGDIR/";
    res.stylesheetName = "mvIMPACT_acquireLogFile.xsl";
    bool boAddItem = false;
    size_t elementCnt = m_missingConfigs.size() + 1;
    wxString* pChoices = new wxString[elementCnt];
    pChoices[0] = wxT( "User defined..." );
    int insertPos = 1;
    StringSet::iterator it = m_missingConfigs.begin();
    while( it != m_missingConfigs.end() )
    {
        pChoices[insertPos] = ConvertedString( ( *it ) );
        ++insertPos;
        ++it;
    }

    {
        wxSingleChoiceDialog dialog( this,
                                     wxT( "Please select the configuration for an unconfigured\ndetected log writer or selected 'User defined...'\nto create a new configuration for an undetected log writer:" ),
                                     wxT( "Configuration selection" ),
                                     static_cast<int>( elementCnt ), pChoices );
        dialog.SetSelection( 0 );

        if( dialog.ShowModal() == wxID_OK )
        {
            string fileExtension( ".xml" );
            if( dialog.GetSelection() == 0 )
            {
                wxTextEntryDialog textEntryDlg( this,
                                                wxT( "Please enter the name of the new log section(without whitespaces" ),
                                                wxT( "Log name selection" ),
                                                wxT( "NewEntry" ),
                                                wxOK | wxCANCEL );

                if( textEntryDlg.ShowModal() == wxID_OK )
                {
                    res.sectionName = textEntryDlg.GetValue().mb_str();
                    res.outputFileName.append( textEntryDlg.GetValue().mb_str() );
                    string::size_type extensionPos = res.outputFileName.find_last_of( fileExtension );
                    // make sure this is an XML file.
                    if( extensionPos != string::npos )
                    {
                        if( ( res.outputFileName.length() - fileExtension.length() ) != extensionPos )
                        {
                            res.outputFileName.append( fileExtension );
                        }
                    }
                    else
                    {
                        res.outputFileName.append( fileExtension );
                    }
                    boAddItem = true;
                }
            }
            else
            {
                string tmp = string( dialog.GetStringSelection().mb_str() );
                string::size_type end = tmp.find_first_of( "(" );
                tmp = tmp.substr( 0, end );
                res.sectionName = tmp;
                res.outputFileName.append( tmp );
                res.outputFileName.append( fileExtension );
                boAddItem = true;
            }
        }
    }

    if( boAddItem )
    {
        m_debugData.insert( m_debugData.begin() + index, res );
        BuildList();
        UpdateMissingConfigsList( true );
    }
    delete [] pChoices;
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::OnBtnLoad( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxFileDialog fileDlg( this, wxT( "Load an existing log configuration" ), wxT( "" ), wxT( "" ), wxT( "log-config Files (*.mvd)|*.mvd" ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        wxString pathName = fileDlg.GetPath();
        wxFFile file( pathName );
        if( !file.IsOpened() )
        {
            WriteErrorMessage( wxT( "ERROR!!! Could not open file.\n" ) );
            return;
        }

        LogConfigurationFileParser parser;
        parser.Create();
        ParseXMLFromFile( parser, file.fp() );
        if( parser.GetErrorCode() != XML_ERROR_NONE )
        {
            WriteErrorMessage( wxString::Format( wxT( "LogOutputHandlerDlg::OnBtnLoad: ERROR!!! XML error: %d(%s).\n" ), parser.GetErrorCode(), parser.GetErrorString( parser.GetErrorCode() ) ) );
            WriteErrorMessage( wxT( "LogOutputHandlerDlg::OnBtnLoad: Stopping file processing.\n" ) );
            return;
        }
        m_debugData = parser.GetResults();
        BuildList();
        UpdateMissingConfigsList( true );
    }
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::OnBtnSave( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_debugData.empty() )
    {
        wxMessageDialog msgDlg( this, wxT( "There is no configuration to store currently. Press 'yes'\nto coninue anyway and store and empty configuration file." ), wxT( "No configuration to store" ), wxYES_NO );
        if( msgDlg.ShowModal() != wxID_YES )
        {
            return;
        }
    }
    wxFileDialog fileDlg( this, wxT( "Save the current log configuration" ), wxT( "" ), wxT( "" ), wxT( "log-config Files (*.mvd)|*.mvd" ), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        wxString pathName = fileDlg.GetPath();
        wxFFile file( pathName, wxT( "w" ) );
        if( !file.IsOpened() )
        {
            WriteErrorMessage( wxT( "ERROR!!! Could not save file.\n" ) );
            return;
        }
        string buf;
        buf.append( "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"yes\" ?>\n" );
        buf.append( "<!DOCTYPE DebugWriterList [\n" );
        buf.append( "  <!ELEMENT DebugWriterList (DebugWriter*)>\n" );
        buf.append( "  <!ELEMENT DebugWriter (#PCDATA)>\n" );
        buf.append( "  <!ATTLIST DebugWriter\n" );
        buf.append( "    name CDATA #REQUIRED\n" );
        buf.append( "    flags CDATA #REQUIRED\n" );
        buf.append( "    outputmask CDATA \"010\"\n" );
        buf.append( "    outputfile CDATA #IMPLIED\n" );
        buf.append( "    clearFile CDATA \"1\"\n" );
        buf.append( "    fileFormat (xml | text | mvlog) 'xml'\n" );
        buf.append( "    stylesheet CDATA \"mvIMPACT_acquireLogFile.xsl\"\n" );
        buf.append( "  >\n" );
        buf.append( "]>\n" );
        buf.append( "<DebugWriterList>\n" );
        buf.append( "  <!--\n" );
        buf.append( "  name: The name of the module OR the device (family name followed by the device ID) this section applies to.\n" );
        buf.append( "  flags: Defines what kind of messages shall be sent to the output. From right to left these messages will be more important, a 1 enables them.\n" );
        buf.append( "  outputmask: From right to left for a 1 the messages will be sent to the systems standard output, a file or the systems debug output.\n" );
        buf.append( "  outputfile: the name of the output file for log messages. STDLOGDIR is predefined.\n" );
        buf.append( "  clearFile: If set to 1 a new file will be created when a log session starts. 0 will append to an existing log-file.\n" );
        buf.append( "  fileFormat: Defines the output format for the log file output. It can be 'text' for *.txt style output, 'xml' for XML file output with a user definable stylesheet or 'mvlog' to create log files that can be displayed using mvLogFileViewer.html. If this parameter is not present XML file output will be used.\n" );
        buf.append( "  stylesheet: Specifies the stylesheet used to transform this log-file(set to 'none' for NOT using a stylesheet)(this parameter will only be used if 'fileFormat' is set to 'xml'.\n" );
        buf.append( "  More information can be found in the manual in the chapter about logging.\n" );
        buf.append( "  -->\n" );
        file.Write( buf.c_str(), buf.length() );
        LogConfigurationVector::size_type vSize = m_debugData.size();
        for( LogConfigurationVector::size_type i = 0; i < vSize; i++ )
        {
            string section( "  <DebugWriter name=\"" );
            section.append( m_debugData[i].sectionName );
            section.append( "\" flags=\"" );
            section.append( m_debugData[i].outputFlagMask );
            section.append( "\" outputmask=\"" );
            section.append( BitmaskToString( m_debugData[i].outputDestinationMask ) );
            section.append( "\" " );
            if( m_debugData[i].outputFileName != "" )
            {
                section.append( "outputfile=\"" );
                section.append( m_debugData[i].outputFileName );
                section.append( "\" " );
            }
            section.append( "clearFile=\"" );
            section.append( m_debugData[i].boClearFile ? "1" : "0" );
            switch( m_debugData[i].outputFileFormat )
            {
            case lffXML:
                break;
            case lffText:
                section.append( "\" fileFormat=\"text" );
                break;
            case lffMVLOG:
                section.append( "\" fileFormat=\"mvlog" );
                break;
            }
            if( ( m_debugData[i].stylesheetName != "" ) && ( m_debugData[i].stylesheetName != "mvIMPACT_acquireLogFile.xsl" ) )
            {
                section.append( "\" stylesheet=\"" );
                section.append( m_debugData[i].stylesheetName );
            }
            section.append( "\"></DebugWriter>\n" );
            file.Write( section.c_str(), section.length() );
        }
        file.Write( wxString( wxT( "</DebugWriterList>" ) ) );
    }
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::OnTimer( wxTimerEvent& e )
//-----------------------------------------------------------------------------
{
    switch( e.GetId() )
    {
    case teUpdate:
        UpdateMissingConfigsList();
        break;
    }
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::SetupColumn( long index )
//-----------------------------------------------------------------------------
{
    m_pLogListCtrl->SetItem( index, llcClearFile, wxString( m_debugData[index].boClearFile ? wxT( "yes" ) : wxT( "no" ) ) );
    m_pLogListCtrl->SetItem( index, llcDevicesUsingThisSection, ConvertedString( GetAssociatedDevicesString( m_debugData[index].sectionName ) ) );
    m_pLogListCtrl->SetItem( index, llcFlags, ConvertedString( m_debugData[index].outputFlagMask ) );
    m_pLogListCtrl->SetItem( index, llcOutputFile, ConvertedString( m_debugData[index].outputFileName ) );
    wxString mask;
    if( m_debugData[index].outputDestinationMask & doFile )
    {
        mask.append( wxT( "file" ) );
    }
    if( m_debugData[index].outputDestinationMask & doSystem )
    {
        if( mask.length() > 0 )
        {
            mask.append( wxT( ", " ) );
        }
        mask.append( wxT( "system debug output" ) );
    }
    if( m_debugData[index].outputDestinationMask & doStdOut )
    {
        if( mask.length() > 0 )
        {
            mask.append( wxT( ", " ) );
        }
        mask.append( wxT( "stdout" ) );
    }
    m_pLogListCtrl->SetItem( index, llcOutputMask, mask );
    m_pLogListCtrl->SetItem( index, llcFileFormat, FileFormatToString( m_debugData[index].outputFileFormat ) );
    m_pLogListCtrl->SetItem( index, llcStylesheet, ConvertedString( m_debugData[index].stylesheetName ) );
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::UpdateMissingConfigsList( bool boForceRebuild /* = false */ )
//-----------------------------------------------------------------------------
{
    if( ( m_devMgr.changedCount() != m_devMgrLastChangedCounter ) || boForceRebuild )
    {
        m_missingConfigs.insert( "mvPropHandling" + GetAssociatedDevicesString( "mvPropHandling" ) );
        m_missingConfigs.insert( "mvDeviceManager" + GetAssociatedDevicesString( "mvDeviceManager" ) );
        m_devMgrLastChangedCounter = m_devMgr.changedCount();
        unsigned int devCnt = m_devMgr.deviceCount();
        Device* pDev = 0;
        for( unsigned int i = 0; i < devCnt; i++ )
        {
            pDev = m_devMgr[i];
            // this might happen more often then needed, but this doesn't matter as there can
            // be only one entry in this set and we don't need to hurry here...
            m_missingConfigs.insert( pDev->family.read() + GetAssociatedDevicesString( pDev->family.read() ) );
            ostringstream oss;
            oss << pDev->family.read() << pDev->deviceID.read();
            m_missingConfigs.insert( oss.str() + GetAssociatedDevicesString( oss.str() ) );
        }

        // now remove the entries already exising again...
        LogConfigurationVector::size_type vSize = m_debugData.size();
        for( LogConfigurationVector::size_type i = 0; i < vSize; i++ )
        {
            StringSet::iterator it = m_missingConfigs.find( m_debugData[i].sectionName + GetAssociatedDevicesString( m_debugData[i].sectionName ) );
            if( it != m_missingConfigs.end() )
            {
                m_missingConfigs.erase( it );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::WriteErrorMessage( const wxString& msg )
//-----------------------------------------------------------------------------
{
    wxTextAttr errorStyle( wxColour( 255, 0, 0 ) );
    WriteLogMessage( msg, errorStyle );
}

//-----------------------------------------------------------------------------
void LogOutputHandlerDlg::WriteLogMessage( const wxString& msg, const wxTextAttr& style /* = wxTextAttr(wxColour(0, 0, 0)) */ )
//-----------------------------------------------------------------------------
{
    if( m_pLogWindow )
    {
        long posBefore = m_pLogWindow->GetLastPosition();
        m_pLogWindow->WriteText( msg );
        long posAfter = m_pLogWindow->GetLastPosition();
        m_pLogWindow->SetStyle( posBefore, posAfter, style );
    }
}
