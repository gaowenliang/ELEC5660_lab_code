//-----------------------------------------------------------------------------
#include "LogOutputConfigurationDlg.h"
#include <apps/Common/wxAbstraction.h>

using namespace std;

//=============================================================================
//============== Implementation LogWriterConfigurationDlg =====================
//=============================================================================

BEGIN_EVENT_TABLE( LogWriterConfigurationDlg, wxDialog )
    EVT_BUTTON( widBtnChooseDir, LogWriterConfigurationDlg::OnBtnChooseDir )
    EVT_BUTTON( widBtnDefault, LogWriterConfigurationDlg::OnBtnDefault )
    EVT_BUTTON( widBtnOk, LogWriterConfigurationDlg::OnBtnOk )
    EVT_BUTTON( widBtnCancel, LogWriterConfigurationDlg::OnBtnCancel )
    EVT_CHECKBOX( widCBFileOutput, LogWriterConfigurationDlg::OnCBFileOutput )
    EVT_TEXT( widCOMFileFormat, LogWriterConfigurationDlg::OnCOMFileFormatTextChanged )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
LogWriterConfigurationDlg::LogWriterConfigurationDlg( wxWindow* pParent, LogOutputConfiguration& data )
    : wxDialog( pParent, wxID_ANY, wxString::Format( wxT( "Log Output Configuration For %s" ), ConvertedString( data.sectionName ).c_str() ) ),
      m_data( data )
//-----------------------------------------------------------------------------
{
    /*
        |-------------------------------------|
        | pTopDownSizer                       |
        | |---------------------------------| |
        | | pOutputFlagsSizer               | |
        | |---------------------------------| |
        |                spacer               |
        | |---------------------------------| |
        | | pOutputDestinationsSizer        | |
        | |---------------------------------| |
        |                spacer               |
        | |---------------------------------| |
        | | pOutputFileSettingsSizer        | |
        | | |-----------------------------| | |
        | | | pLeftRightFileSettingsSizer | | |
        | | |-----------------------------| | |
        | | |-----------------------------| | |
        | | | pLeftRightStylesheetSizer   | | |
        | | |-----------------------------| | |
        | |              spacer             | |
        | |           clear file CB         | |
        | |              spacer             | |
        | |---------------------------------| |
        |                spacer               |
        | |---------------------------------| |
        | | pButtonSizer                    | |
        | |---------------------------------| |
        |-------------------------------------|
    */

    wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    pTopDownSizer->AddSpacer( 10 );

    wxPanel* pPanel = new wxPanel( this );

    // output flags
    wxStaticBoxSizer* pOutputFlagsSizer = new wxStaticBoxSizer( wxVERTICAL, pPanel, wxT( "Type of messages to send to the log writer:" ) );
    m_ppCBOutputFlags[0] = new wxCheckBox( pOutputFlagsSizer->GetStaticBox(), wxID_ANY, wxT( "information (these type of messages will only be available in debug builds)" ) );
    m_ppCBOutputFlags[1] = new wxCheckBox( pOutputFlagsSizer->GetStaticBox(), wxID_ANY, wxT( "important information (these type of messages will only be available in debug builds)" ) );
    m_ppCBOutputFlags[2] = new wxCheckBox( pOutputFlagsSizer->GetStaticBox(), wxID_ANY, wxT( "warnings" ) );
    m_ppCBOutputFlags[3] = new wxCheckBox( pOutputFlagsSizer->GetStaticBox(), wxID_ANY, wxT( "errors" ) );
    m_ppCBOutputFlags[4] = new wxCheckBox( pOutputFlagsSizer->GetStaticBox(), wxID_ANY, wxT( "fatal errors" ) );
    m_ppCBOutputFlags[5] = new wxCheckBox( pOutputFlagsSizer->GetStaticBox(), wxID_ANY, wxT( "standard log messages" ) );
    m_ppCBOutputFlags[6] = new wxCheckBox( pOutputFlagsSizer->GetStaticBox(), wxID_ANY, wxT( "general purpose messages" ) );
    for( unsigned int i = 0; i < MAX_LOG_LEVEL; i++ )
    {
        pOutputFlagsSizer->Add( m_ppCBOutputFlags[i], wxSizerFlags().Left() );
    }
    pOutputFlagsSizer->AddSpacer( 10 ); // see trac.wxwidgets.org/ticket/17239 before changing this
    pTopDownSizer->Add( pOutputFlagsSizer, wxSizerFlags().Expand() );
    pTopDownSizer->AddSpacer( 10 );
    // output destinations
    wxStaticBoxSizer* pOutputDestinationsSizer = new wxStaticBoxSizer( wxVERTICAL, pPanel, wxT( "Send log output to:" ) );
    m_ppCBOutputDestinations[0] = new wxCheckBox( pOutputDestinationsSizer->GetStaticBox(), wxID_ANY, wxT( "the systems standard output" ) );
    m_ppCBOutputDestinations[1] = new wxCheckBox( pOutputDestinationsSizer->GetStaticBox(), wxID_ANY, wxT( "the systems debug output" ) );
    m_ppCBOutputDestinations[2] = new wxCheckBox( pOutputDestinationsSizer->GetStaticBox(), widCBFileOutput, wxT( "a file (specified by the output file name)" ) );
    for( unsigned int i = 0; i < MAX_OUTPUT_MASK_BIT; i++ )
    {
        pOutputDestinationsSizer->Add( m_ppCBOutputDestinations[i], wxSizerFlags().Left() );
    }
    pOutputDestinationsSizer->AddSpacer( 10 ); // see trac.wxwidgets.org/ticket/17239 before changing this
    pTopDownSizer->Add( pOutputDestinationsSizer, wxSizerFlags().Expand() );
    pTopDownSizer->AddSpacer( 10 );

    // output file
    wxStaticBoxSizer* pOutputFileSettingsSizer = new wxStaticBoxSizer( wxVERTICAL, pPanel, wxT( "File output settings:" ) );
    wxBoxSizer* pLeftRightFileSettingsSizer = new wxBoxSizer( wxHORIZONTAL );
    m_pOutputFileLabel = new wxStaticText( pOutputFileSettingsSizer->GetStaticBox(), wxID_ANY, wxT( "output file name: " ) );
    pLeftRightFileSettingsSizer->Add( m_pOutputFileLabel, wxSizerFlags().Left().Center() );
    m_pOutputFileName = new wxTextCtrl( pOutputFileSettingsSizer->GetStaticBox(), wxID_ANY, ConvertedString( m_data.outputFileName ) );
    pLeftRightFileSettingsSizer->Add( m_pOutputFileName, wxSizerFlags( 2 ).Align( wxGROW ) );
    m_pBtnChooseDir = new wxButton( pOutputFileSettingsSizer->GetStaticBox(), widBtnChooseDir, wxT( "Browse..." ) );
    m_pBtnChooseDir->SetToolTip( wxT( "Select a directory for the file associated with this log output" ) );
    pLeftRightFileSettingsSizer->Add( m_pBtnChooseDir, wxSizerFlags().Right().Center() );

    wxBoxSizer* pLeftRightFileFormatSizer = new wxBoxSizer( wxHORIZONTAL );
    pLeftRightFileFormatSizer->Add( new wxStaticText( pOutputFileSettingsSizer->GetStaticBox(), wxID_ANY, wxT( "File Format: " ) ), wxSizerFlags().Left().Center() );
    m_pCOMFileFormat = new wxComboBox( pOutputFileSettingsSizer->GetStaticBox(), widCOMFileFormat, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_DROPDOWN | wxCB_READONLY );
    m_pCOMFileFormat->Append( FileFormatToString( lffText ) );
    m_pCOMFileFormat->Append( FileFormatToString( lffXML ) );
    m_pCOMFileFormat->Append( FileFormatToString( lffMVLOG ) );
    m_pCOMFileFormat->Select( 0 );
    m_pCOMFileFormat->SetToolTip( wxT( "The format used for the log messages written into the file" ) );
    pLeftRightFileFormatSizer->Add( m_pCOMFileFormat, wxSizerFlags( 2 ).Align( wxGROW ) );

    wxBoxSizer* pLeftRightStylesheetSettingsSizer = new wxBoxSizer( wxHORIZONTAL );
    m_pStylesheetLabel = new wxStaticText( pOutputFileSettingsSizer->GetStaticBox(), wxID_ANY, wxT( "stylesheet name: " ) );
    m_pStylesheetLabel->SetToolTip( wxT( "The XML-stylesheet used to display the *.log-files. Set to 'none' if you want no stylesheet to be embedded in the log-file" ) );
    pLeftRightStylesheetSettingsSizer->Add( m_pStylesheetLabel, wxSizerFlags().Left().Center() );
    m_pStylesheetName = new wxTextCtrl( pOutputFileSettingsSizer->GetStaticBox(), wxID_ANY, ConvertedString( m_data.stylesheetName ) );
    m_pStylesheetName->SetToolTip( wxT( "The XML-stylesheet used to display the *.log-files. Set to 'none' if you want no stylesheet to be embedded in the log-file" ) );
    pLeftRightStylesheetSettingsSizer->Add( m_pStylesheetName, wxSizerFlags( 2 ).Align( wxGROW ) );

    m_pCBClearFile = new wxCheckBox( pOutputFileSettingsSizer->GetStaticBox(), wxID_ANY, wxT( "always overwrite existing files" ) );
    pOutputFileSettingsSizer->Add( pLeftRightFileSettingsSizer, wxSizerFlags().Expand() );
    pOutputFileSettingsSizer->AddSpacer( 10 );
    pOutputFileSettingsSizer->Add( pLeftRightFileFormatSizer, wxSizerFlags().Expand() );
    pOutputFileSettingsSizer->AddSpacer( 10 );
    pOutputFileSettingsSizer->Add( pLeftRightStylesheetSettingsSizer, wxSizerFlags().Expand() );
    pOutputFileSettingsSizer->AddSpacer( 10 );
    pOutputFileSettingsSizer->Add( m_pCBClearFile, wxSizerFlags().Left().DoubleBorder( wxBOTTOM ) );
    pTopDownSizer->Add( pOutputFileSettingsSizer, wxSizerFlags().Expand() );
    pTopDownSizer->AddSpacer( 10 );
    // lower line of buttons
    wxBoxSizer* pButtonSizer = new wxBoxSizer( wxHORIZONTAL );
    m_pBtnDefault = new wxButton( pPanel, widBtnDefault, wxT( "&Default" ) );
    m_pBtnDefault->SetToolTip( wxT( "Restore the default settings for a log output" ) );
    pButtonSizer->Add( m_pBtnDefault, wxSizerFlags().Left().Border( wxALL, 7 ) );
    pButtonSizer->AddStretchSpacer( 100 );
    m_pBtnOk = new wxButton( pPanel, widBtnOk, wxT( "&Ok" ) );
    pButtonSizer->Add( m_pBtnOk, wxSizerFlags().Border( wxALL, 7 ) );
    m_pBtnCancel = new wxButton( pPanel, widBtnCancel, wxT( "&Cancel" ) );
    pButtonSizer->Add( m_pBtnCancel, wxSizerFlags().Border( wxALL, 7 ) );
    pTopDownSizer->Add( pButtonSizer, wxSizerFlags().Expand().DoubleBorder() ); // see trac.wxwidgets.org/ticket/17239 before changing this
    pPanel->SetSizer( pTopDownSizer );
    pTopDownSizer->SetSizeHints( this );
    SetupDlgControls();
    SetClientSize( pTopDownSizer->GetMinSize() );
    SetSizeHints( GetSize() );
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::OnBtnChooseDir( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxDirDialog dlg( this );
    if( dlg.ShowModal() == wxID_OK )
    {
        string path( dlg.GetPath().mb_str() );
        path.append( "/" );
        path.append( m_data.outputFileName.substr( m_data.outputFileName.find_last_of( "/\\" ) + 1 ) );
        m_data.outputFileName = path;
        m_pOutputFileName->SetValue( ConvertedString( m_data.outputFileName ) );
    }
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::OnBtnDefault( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    restoreDefaults( m_data );
    SetupDlgControls();
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::OnBtnOk( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    // get modified data
    m_data.boClearFile = m_pCBClearFile->IsChecked();
    int outputDestinationMask = 0;
    for( unsigned int i = 0; i < MAX_OUTPUT_MASK_BIT; i++ )
    {
        outputDestinationMask |= m_ppCBOutputDestinations[i]->IsChecked() << i;
    }
    m_data.outputDestinationMask = TDebugOutputDestination( outputDestinationMask );
    m_data.outputFileName = m_pOutputFileName->GetValue().mb_str();
    int outputFlagMask = 0;
    for( unsigned int i = 0; i < MAX_LOG_LEVEL; i++ )
    {
        outputFlagMask |= m_ppCBOutputFlags[i]->IsChecked() << i;
    }
    m_data.outputFlagMask = BitmaskToString( outputFlagMask );
    m_data.outputFileFormat = FileFormatFromString( m_pCOMFileFormat->GetValue() );
    EndModal( wxID_OK );
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::OnBtnCancel( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    EndModal( wxID_CANCEL );
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::OnCBFileOutput( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_data.outputDestinationMask = TDebugOutputDestination( ~doFile & m_data.outputDestinationMask );
    if( m_ppCBOutputDestinations[2]->GetValue() )
    {
        m_data.outputDestinationMask = TDebugOutputDestination( m_data.outputDestinationMask | doFile );
    }
    SetupFileControls();
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::OnCOMFileFormatTextChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_data.outputFileFormat = FileFormatFromString( m_pCOMFileFormat->GetValue() );
    SetupFileControls();
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::SetupDlgControls( void )
//-----------------------------------------------------------------------------
{
    m_pCBClearFile->SetValue( m_data.boClearFile );
    for( unsigned int i = 0; i < MAX_OUTPUT_MASK_BIT; i++ )
    {
        m_ppCBOutputDestinations[i]->SetValue( ( m_data.outputDestinationMask & ( 1 << i ) ) != 0 );
    }
    unsigned int flagCnt = ( static_cast<unsigned int>( m_data.outputFlagMask.length() ) > static_cast<unsigned int>( MAX_LOG_LEVEL ) ) ? ( unsigned int )MAX_LOG_LEVEL : static_cast<unsigned int>( m_data.outputFlagMask.length() );
    for( int i = flagCnt - 1; i >= 0; i-- )
    {
        m_ppCBOutputFlags[i]->SetValue( *( m_data.outputFlagMask.end() - ( i + 1 ) ) == '1' );
    }
    m_pOutputFileName->SetValue( ConvertedString( m_data.outputFileName ) );
    m_pCOMFileFormat->SetValue( FileFormatToString( m_data.outputFileFormat ) );
    SetupFileControls();
}

//-----------------------------------------------------------------------------
void LogWriterConfigurationDlg::SetupFileControls( void )
//-----------------------------------------------------------------------------
{
    bool boEnableFileOutputRelatedControls = m_ppCBOutputDestinations[CB_FILE_OUTPUT]->IsChecked() == 1;
    bool boFileModeXML = FileFormatFromString( m_pCOMFileFormat->GetValue() ) == lffXML;
    m_pCBClearFile->Enable( boEnableFileOutputRelatedControls );
    m_pOutputFileName->Enable( boEnableFileOutputRelatedControls );
    m_pOutputFileLabel->Enable( boEnableFileOutputRelatedControls );
    m_pBtnChooseDir->Enable( boEnableFileOutputRelatedControls );
    m_pStylesheetLabel->Enable( boEnableFileOutputRelatedControls && boFileModeXML );
    m_pStylesheetName->Enable( boEnableFileOutputRelatedControls && boFileModeXML );
    m_pCOMFileFormat->Enable( boEnableFileOutputRelatedControls );
}

//-----------------------------------------------------------------------------
void restoreDefaults( LogOutputConfiguration& data )
//-----------------------------------------------------------------------------
{
    data.boClearFile = true;
    data.outputDestinationMask = TDebugOutputDestination( doSystem | doFile );
    data.outputFlagMask = "1111100";
    data.outputFileName = "STDLOGDIR/";
    data.outputFileName.append( data.sectionName );
    data.outputFileName.append( ".mvlog" );
    data.outputFileFormat = lffMVLOG;
}
