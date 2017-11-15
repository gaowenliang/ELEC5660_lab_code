#include "DataGrid.h"
#include <apps/Common/wxAbstraction.h>
#include "PropGridFrameBase.h"
#include <sstream>
#include "WizardLUTControl.h"
#include <wx/textfile.h>
#include <wx/progdlg.h>

using namespace std;

wxDEFINE_EVENT( GammaParameterChangeEvent, wxCommandEvent );

//=============================================================================
//============== Implementation WizardLUTCanvas ===============================
//=============================================================================
//-----------------------------------------------------------------------------
WizardLUTCanvas::WizardLUTCanvas( wxWindow* parent, wxWindowID id /* = -1 */, const wxPoint& pos /* = wxDefaultPosition */,
                                  const wxSize& size /* = wxDefaultSize */, long style /* = wxBORDER_NONE */, const wxString& name /* = "LUT" */, bool boActive /* = false */ )
    : PlotCanvas( parent, id, pos, size, style, name, boActive )
//-----------------------------------------------------------------------------
{
    SetActive( true );
}

//-----------------------------------------------------------------------------
WizardLUTCanvas::~WizardLUTCanvas()
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
double WizardLUTCanvas::GetScaleX( wxCoord w ) const
//-----------------------------------------------------------------------------
{
    return static_cast<double>( w - 2 * GetBorderWidth() ) / ( static_cast<double>( lut_.size() ) * ( static_cast<double>( GetPercentageToDraw() ) / 100. ) );
}

//-----------------------------------------------------------------------------
double WizardLUTCanvas::GetScaleY( wxCoord h ) const
//-----------------------------------------------------------------------------
{
    return static_cast<double>( h - 2 * GetBorderWidth() ) / maxOutputValue_;
}

//-----------------------------------------------------------------------------
unsigned int WizardLUTCanvas::GetXMarkerParameters( unsigned int& from, unsigned int& to ) const
//-----------------------------------------------------------------------------
{
    from = 0;
    to = static_cast<unsigned int>( lut_.size() );
    return GetXMarkerStepWidth( from, to );
}

//-----------------------------------------------------------------------------
void WizardLUTCanvas::OnPaintCustom( wxPaintDC& dc )
//-----------------------------------------------------------------------------
{
    wxCoord w( 0 ), h( 0 );
    dc.GetSize( &w, &h );
    const double scaleX = GetScaleX( w );
    DrawMarkerLines( dc, w, h, scaleX );
    dc.DrawText( wxString::Format( wxT( "%d" ), maxOutputValue_ ), GetBorderWidth() / 2, 1 );
    DrawProfileLine( dc, h, GetBorderWidth() + 1, scaleX, GetScaleY( h ), 0, maxOutputValue_, &( *lut_.begin() ), static_cast<int>( lut_.size() ), *wxBLACK );
}

//-----------------------------------------------------------------------------
void WizardLUTCanvas::RefreshData( const vector<int>& lut, const int maxOutputValue )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    lut_ = lut;
    maxOutputValue_ = maxOutputValue;
}

//=============================================================================
//============== Implementation WizardLUTControl ==============================
//=============================================================================
BEGIN_EVENT_TABLE( WizardLUTControl, OkAndCancelDlg )
    EVT_CHECKBOX( widCBEnable, WizardLUTControl::OnCBEnable )
    EVT_TEXT( widCBLUTSelection, WizardLUTControl::OnCBLUTSelectionChanged )
    EVT_BUTTON( widBtnEnableAll, WizardLUTControl::OnBtnEnableAll )
    EVT_BUTTON( widBtnDisableAll, WizardLUTControl::OnBtnDisableAll )
    EVT_BUTTON( widBtnCopyTo, WizardLUTControl::OnBtnCopyTo )
    EVT_BUTTON( widBtnExport, WizardLUTControl::OnBtnExport )
    EVT_BUTTON( widBtnGamma, WizardLUTControl::OnBtnGamma )
    EVT_BUTTON( widBtnImport, WizardLUTControl::OnBtnImport )
    EVT_BUTTON( widBtnInterpolate, WizardLUTControl::OnBtnInterpolate )
    EVT_BUTTON( widBtnInvert, WizardLUTControl::OnBtnInvert )
    EVT_BUTTON( widBtnSynchronize, WizardLUTControl::OnBtnSynchronize )
    EVT_CLOSE( WizardLUTControl::OnClose )
    EVT_COMMAND( widMainFrame, GammaParameterChangeEvent, WizardLUTControl::OnGammaParameterChanged )
    EVT_NOTEBOOK_PAGE_CHANGED( widNBDisplayMethod, WizardLUTControl::OnNBDisplayMethodPageChanged )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
WizardLUTControl::WizardLUTControl( wxWindow* pParent, const wxString& title, mvIMPACT::acquire::GenICam::LUTControl& lc, const wxArrayString& lutSelectorEntries )
    : OkAndCancelDlg( pParent, widMainFrame, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER | wxMAXIMIZE_BOX | wxMINIMIZE_BOX ),
      errorStyle_( *wxRED ), pCBEnable_( 0 ), pCBLUTSelection_( 0 ), pNBDisplayMethod_( 0 ), pNumericalDisplay_( 0 ),
      pBtnEnableAll_( 0 ), pBtnDisableAll_( 0 ),
      pBtnSynchronize_( 0 ), pBtnInvert_( 0 ), pBtnGamma_( 0 ), pBtnInterpolate_( 0 ), pBtnCopyTo_( 0 ), pBtnImport_( 0 ), pBtnExport_( 0 ), pLogWindow_( 0 ),
      displayMethod_( dmNumerical ), lc_( lc ), lutSelectorEntries_( lutSelectorEntries ), lutCnt_( lutSelectorEntries.Count() ), activeLUT_()
//-----------------------------------------------------------------------------
{
    /*
        |-------------------------------------|
        | pTopDownSizer                       |
        |                spacer               |
        | |---------------------------------| |
        | | pUpperControlsSizer             | |
        | |---------------------------------| |
        |                spacer               |
        | |---------------------------------| |
        | | pCentralControlsSizer           | |
        | | |-----------------------------| | |
        | | | Buttons        | Grid       | | |
        | | |-----------------------------| | |
        | |---------------------------------| |
        |                spacer               |
        | |---------------------------------| |
        | | pButtonSizer                    | |
        | |---------------------------------| |
        |-------------------------------------|
    */

    wxPanel* pPanel = new wxPanel( this );

    // upper line of controls
    pCBLUTSelection_ = new wxComboBox( pPanel, widCBLUTSelection, wxT( " LUT " ), wxDefaultPosition, wxSize( 100, wxDefaultCoord ), 0, NULL, wxCB_DROPDOWN | wxCB_READONLY );
    for( size_t i = 0; i < lutCnt_; i++ )
    {
        pCBLUTSelection_->Insert( lutSelectorEntries_[i], 0 );
    }
    pCBLUTSelection_->Enable( lc_.LUTSelector.isValid() && ( lutCnt_ > 1 ) );
    const bool boLUTSelectorIsWriteable = lc_.LUTSelector.isValid() && lc_.LUTSelector.isWriteable();
    pCBLUTSelection_->Select( 0 );
    pCBLUTSelection_->SetToolTip( wxT( "Contains a list of all LUTs currently supported by the device. If this list is read-only, the device does support a single LUT only" ) );
    pBtnSynchronize_ = new wxButton( pPanel, widBtnSynchronize, wxT( "Synchronize" ) );
    pBtnSynchronize_->SetToolTip( wxT( "Force the device and the internal cache to re-synchronize" ) );
    pBtnEnableAll_ = new wxButton( pPanel, widBtnEnableAll, wxT( "Enable All" ) );
    pBtnEnableAll_->SetToolTip( wxT( "Enables all LUTs on the device" ) );
    pBtnEnableAll_->Enable( boLUTSelectorIsWriteable );
    pBtnDisableAll_ = new wxButton( pPanel, widBtnDisableAll, wxT( "Disable All" ) );
    pBtnDisableAll_->SetToolTip( wxT( "Disables all LUTs on the device" ) );
    pBtnDisableAll_->Enable( boLUTSelectorIsWriteable );

    // left middle line of controls
    pCBEnable_ = new wxCheckBox( pPanel, widCBEnable, wxT( "Enable" ) );
    pCBEnable_->SetToolTip( wxT( "Enable/Disable the selected LUT" ) );
    pBtnInvert_ = new wxButton( pPanel, widBtnInvert, wxT( "Invert" ) );
    pBtnInvert_->SetToolTip( wxT( "Invert the data of the selected LUT" ) );
    pBtnGamma_ = new wxButton( pPanel, widBtnGamma, wxT( "Gamma..." ) );
    pBtnGamma_->SetToolTip( wxT( "Create data for the selected LUT from a gamma value. The following formula is used for the gamma LUT calculation: (1 + GammaAlpha)*inputValue^(1/Gamma) - GammaAlpha" ) );
    pBtnInterpolate_ = new wxButton( pPanel, widBtnInterpolate, wxT( "Interpolate..." ) );
    pBtnInterpolate_->SetToolTip( wxT( "Create data from the selected LUT from a selectable number of values using a selectable interpolation method" ) );
    pBtnCopyTo_ = new wxButton( pPanel, widBtnCopyTo, wxT( "Copy To..." ) );
    pBtnCopyTo_->SetToolTip( wxT( "Copy data from the selected LUT into one or many others" ) );
    pBtnCopyTo_->Enable( boLUTSelectorIsWriteable );
    pBtnImport_ = new wxButton( pPanel, widBtnImport, wxT( "Import..." ) );
    pBtnImport_->SetToolTip( wxT( "Import data into the selected LUT from a *.csv file" ) );
    pBtnExport_ = new wxButton( pPanel, widBtnExport, wxT( "Export..." ) );
    pBtnExport_->SetToolTip( wxT( "Export data from the selected LUT into a *.csv file" ) );

    // right middle line of controls
    pNBDisplayMethod_ = new wxNotebook( pPanel, widNBDisplayMethod );
    pGraphicalDisplay_ = new WizardLUTCanvas( pNBDisplayMethod_, widGraphicalDisplay, wxDefaultPosition, wxDefaultSize );
    pNBDisplayMethod_->InsertPage( 0, pGraphicalDisplay_, wxT( "Graphical" ), true );
    pNumericalDisplay_ = new DataGridWithClipboardFeature( pNBDisplayMethod_, widNumericalDisplay );
    pNumericalDisplay_->SetDataProvider( this );
    pNumericalDisplay_->SetTable( new DataGridTable( this, static_cast<long>( activeLUT_.size() ), 1 ), true );
    pNBDisplayMethod_->InsertPage( 1, pNumericalDisplay_, wxT( "Numerical" ) );

    // lower log window
    pLogWindow_ = new wxTextCtrl( pPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxBORDER_NONE | wxTE_RICH | wxTE_READONLY );

    wxBoxSizer* pUpperControlsSizer = new wxBoxSizer( wxHORIZONTAL );
    pUpperControlsSizer->AddSpacer( 5 );
    wxStaticText* pSTLUTSelector( new wxStaticText( pPanel, wxID_ANY, wxT( "LUT Selector: " ) ) );
    pSTLUTSelector->SetToolTip( wxT( "If the LUT selection list is read-only, the device does support a single LUT only" ) );
    pUpperControlsSizer->Add( pSTLUTSelector, wxSizerFlags().Left().Center() );
    pUpperControlsSizer->Add( pCBLUTSelection_ );
    pUpperControlsSizer->AddSpacer( 20 );
    pUpperControlsSizer->Add( pBtnSynchronize_ );
    pUpperControlsSizer->AddSpacer( 4 );
    pUpperControlsSizer->Add( pBtnEnableAll_ );
    pUpperControlsSizer->AddSpacer( 4 );
    pUpperControlsSizer->Add( pBtnDisableAll_ );

    wxBoxSizer* pTopDownSizerCenter = new wxBoxSizer( wxVERTICAL );
    pTopDownSizerCenter->Add( pCBEnable_ );
    pTopDownSizerCenter->AddSpacer( 20 );
    pTopDownSizerCenter->Add( pBtnInvert_, wxSizerFlags().Expand() );
    pTopDownSizerCenter->AddSpacer( 4 );
    pTopDownSizerCenter->Add( pBtnGamma_, wxSizerFlags().Expand() );
    pTopDownSizerCenter->AddSpacer( 4 );
    pTopDownSizerCenter->Add( pBtnInterpolate_, wxSizerFlags().Expand() );
    pTopDownSizerCenter->AddSpacer( 4 );
    pTopDownSizerCenter->Add( pBtnCopyTo_, wxSizerFlags().Expand() );
    pTopDownSizerCenter->AddSpacer( 20 );
    pTopDownSizerCenter->Add( pBtnImport_, wxSizerFlags().Expand() );
    pTopDownSizerCenter->AddSpacer( 4 );
    pTopDownSizerCenter->Add( pBtnExport_, wxSizerFlags().Expand() );

    wxBoxSizer* pCentralControlsSizer = new wxBoxSizer( wxHORIZONTAL );
    pCentralControlsSizer->AddSpacer( 5 );
    pCentralControlsSizer->Add( pTopDownSizerCenter );
    pCentralControlsSizer->AddSpacer( 10 );
    pCentralControlsSizer->Add( pNBDisplayMethod_, wxSizerFlags( 1 ).Expand() );

    wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    pTopDownSizer->AddSpacer( 10 );
    pTopDownSizer->Add( pUpperControlsSizer );
    pTopDownSizer->AddSpacer( 10 );
    pTopDownSizer->Add( pCentralControlsSizer, wxSizerFlags( 7 ).Expand() );
    pTopDownSizer->AddSpacer( 10 );
    pTopDownSizer->Add( pLogWindow_, wxSizerFlags( 1 ).Expand() );
    pTopDownSizer->AddSpacer( 10 );

    AddButtons( pPanel, pTopDownSizer, true );
    FinalizeDlgCreation( pPanel, pTopDownSizer );
    SetSize( 500, 700 );

    pBtnApply_->SetToolTip( wxT( "Write all changes back to the device" ) );
    SetGridValueFormatString( wxT( "%d" ) );
    SetupLUTEnable();
    for( size_t i = 0; i < lutCnt_; i++ )
    {
        ReadLUTFromDevice( lutSelectorEntries_[i], activeLUT_ );
    }
    LUTMap::iterator it = lutMap_.find( pCBLUTSelection_->GetValue() );
    if( it != lutMap_.end() )
    {
        activeLUT_ = it->second.lut_;
    }
    else
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: LUT '%s' not found in cache.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, pCBLUTSelection_->GetValue().c_str() ) );
    }
    UpdateDisplay();
}

//-----------------------------------------------------------------------------
void WizardLUTControl::AddLUTToCache( const wxString& lutIdentifier, vector<int>& lut, bool boDirty )
//-----------------------------------------------------------------------------
{
    LUTMap::iterator it = lutMap_.find( lutIdentifier );
    if( it == lutMap_.end() )
    {
        lutMap_.insert( make_pair( lutIdentifier, LUTEntry( lut, boDirty ) ) );
    }
    else
    {
        it->second.lut_ = lut;
        it->second.boDirty_ = boDirty;
    }

    if( !boDirty )
    {
        it = lutMap_.begin();
        const LUTMap::const_iterator itEND = lutMap_.end();
        while( it != itEND )
        {
            if( it->second.boDirty_ )
            {
                boDirty = true;
                break;
            }
            ++it;
        }
    }

    pBtnApply_->Enable( boDirty );
}

//-----------------------------------------------------------------------------
void WizardLUTControl::CalculateGammaLUT( vector<int>& lut, const GammaData& data )
//-----------------------------------------------------------------------------
{
    try
    {
        double inputMaxVal = static_cast<double>( lc_.LUTIndex.getMaxValue() );
        double outputMaxVal = static_cast<double>( lc_.LUTValue.getMaxValue() );
        lut.resize( lc_.LUTIndex.getMaxValue() + 1 );
        const vector<int>::size_type vSize = lut.size();
        int transformedValue = 0;

        double gamma = 1. / data.gamma_;
        switch( data.gammaMode_ )
        {
        case LUTgmLinearStart:
            {
                double gammaAtThreshold = static_cast<int>( ( ( 1 + data.gammaAlpha_ ) * pow( static_cast<double>( data.gammaStartThreshold_ ) / inputMaxVal, gamma ) - data.gammaAlpha_ ) * outputMaxVal );
                double delta = gammaAtThreshold / static_cast<double>( data.gammaStartThreshold_ );
                for( vector<int>::size_type i = 0; i < static_cast<vector<int>::size_type>( data.gammaStartThreshold_ ); i++ )
                {
                    transformedValue = static_cast<int>( static_cast<double>( i ) * delta );
                    lut[i] = saveAssign( transformedValue, 0, static_cast<int>( outputMaxVal ) );
                }
                for( vector<int>::size_type i = data.gammaStartThreshold_; i < vSize; i++ )
                {
                    transformedValue = static_cast<int>( ( ( 1 + data.gammaAlpha_ ) * pow( static_cast<double>( i ) / inputMaxVal, gamma ) - data.gammaAlpha_ ) * outputMaxVal );
                    lut[i] = saveAssign( transformedValue, 0, static_cast<int>( outputMaxVal ) );
                }
            }
            break;
        case LUTgmStandard:
            for( vector<int>::size_type i = 0; i < vSize; i++ )
            {
                transformedValue = static_cast<int>( ( ( 1 + data.gammaAlpha_ ) * pow( static_cast<double>( i ) / inputMaxVal, gamma ) - data.gammaAlpha_ ) * outputMaxVal );
                lut[i] = saveAssign( transformedValue, 0, static_cast<int>( outputMaxVal ) );
            }
            break;
        }
    }
    catch( const ImpactAcquireException& e )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to calculate gamma LUT.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::ConfigureAllLUTs( bool boEnable )
//-----------------------------------------------------------------------------
{
    try
    {
        const int64_type selectedLUT( lc_.LUTSelector.read() );
        vector<pair<string, int64_type> > v;
        lc_.LUTSelector.getTranslationDict( v );
        const vector<pair<string, int64_type> >::size_type vCnt = v.size();
        for( vector<pair<string, int64_type> >::size_type i = 0; i < vCnt; i++ )
        {
            lc_.LUTSelector.write( v[i].second );
            lc_.LUTEnable.write( boEnable ? bTrue : bFalse );
        }
        lc_.LUTSelector.write( selectedLUT );
    }
    catch( const ImpactAcquireException& ex )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to copy LUT data.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( ex.getErrorString() ).c_str(), ConvertedString( ex.getErrorCodeAsString() ).c_str() ) );
    }
    SetupLUTEnable();
}

//-----------------------------------------------------------------------------
wxString WizardLUTControl::GetGridValue( int row, int col ) const
//-----------------------------------------------------------------------------
{
    if( ( col >= 0 ) && ( row >= 0 ) && ( col < 1 ) && ( static_cast<vector<int>::size_type>( row ) <= activeLUT_.size() ) )
    {
        return wxString::Format( GetGridValueFormatString().c_str(), activeLUT_[row] );
    }
    return wxEmptyString;
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnApply( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    LUTMap::iterator it = lutMap_.begin();
    const LUTMap::iterator itEND = lutMap_.end();
    while( it != itEND )
    {
        if( it->second.boDirty_ )
        {
            WriteLUTToDevice( it->first, it->second.lut_ );
        }
        ++it;
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnCopyTo( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxArrayString choices;
    if( lutCnt_ > 2 )
    {
        choices.Add( wxT( "All" ) );
    }
    for( size_t i = 0; i < lutCnt_; i++ )
    {
        if( lutSelectorEntries_[i] != pCBLUTSelection_->GetValue() )
        {
            choices.Add( lutSelectorEntries_[i] );
        }
    }
    const wxString destination = ::wxGetSingleChoice( wxT( "Please select to which LUT(s) you want to copy the current one to" ), wxT( "LUT Control: Copy" ), choices, this );
    try
    {
        const int64_type selectedLUT( lc_.LUTSelector.read() );
        if( destination == wxT( "All" ) )
        {
            for( size_t i = 0; i < lutCnt_; i++ )
            {
                if( lutSelectorEntries_[i] != pCBLUTSelection_->GetValue() )
                {
                    AddLUTToCache( lutSelectorEntries_[i], activeLUT_, true );
                }
            }
        }
        else
        {
            AddLUTToCache( destination, activeLUT_, true );
        }
        lc_.LUTSelector.write( selectedLUT );
    }
    catch( const ImpactAcquireException& ex )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to copy LUT data.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( ex.getErrorString() ).c_str(), ConvertedString( ex.getErrorCodeAsString() ).c_str() ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnExport( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxFileDialog fileDlg( this, wxT( "Select a filename to store the current LUT to" ), wxT( "" ), wxT( "" ), wxT( "CSV Files (*.csv)|*.csv" ), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        ostringstream oss;
        const vector<int>::size_type vSize = activeLUT_.size();
        for( vector<int>::size_type i = 0; i < vSize; i++ )
        {
            oss << activeLUT_[i] << endl;
        }
        WriteFile( oss.str().c_str(), oss.str().length(), fileDlg.GetPath(), pLogWindow_ );
    }
    else
    {
        WriteLogMessage( wxT( "LUT Export operation canceled by user.\n" ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnGamma( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    vector<int> oldLUT( activeLUT_ );
    GammaData oldGammaData( gammaData_ );
    WizardLUTGammaDlg dlg( this, wxT( "LUT Control: Gamma Parameter Setup" ), static_cast<int>( activeLUT_.size() ) - 1, gammaData_ );
    if( dlg.ShowModal() == wxID_OK )
    {
        CalculateGammaLUT( activeLUT_, gammaData_ );
    }
    else
    {
        WriteLogMessage( wxT( "LUT gamma operation canceled by user.\n" ) );
        gammaData_ = oldGammaData;
        activeLUT_ = oldLUT;
    }
    AddLUTToCache( pCBLUTSelection_->GetValue(), activeLUT_, true );
    UpdateDisplay();
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnImport( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxFileDialog fileDlg( this, wxT( "Select a file to load the current LUT from" ), wxT( "" ), wxT( "" ), wxT( "CSV Files (*.csv)|*.csv" ), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if( fileDlg.ShowModal() == wxID_OK )
    {
        wxTextFile file;
        file.Open( fileDlg.GetPath() );
        if( !file.IsOpened() )
        {
            WriteErrorMessage( wxString::Format( wxT( "Failed to open file %s.\n" ), fileDlg.GetPath().c_str() ) );
            return;
        }

        const size_t elementsToProcess = activeLUT_.size();
        vector<wxString> lines;
        lines.push_back( file.GetFirstLine() );
        size_t i = 1;
        for( ; i < elementsToProcess; i++ )
        {
            if( file.Eof() )
            {
                wxString msg;
                msg << wxT( "Error detected during LUT import: lines read: " ) << i << wxT( ", expected at least: " ) << elementsToProcess << wxT( ".\n" );
                WriteErrorMessage( msg );
                return;
            }
            lines.push_back( file.GetNextLine() );
        }

        for( size_t i = 0; i < elementsToProcess; i++ )
        {
            vector<wxString> lineTokens;
            Split( lines[i], wxT( ";" ), lineTokens );
            activeLUT_[i] = atoi( lineTokens[0].mb_str() );
        }
        AddLUTToCache( pCBLUTSelection_->GetValue(), activeLUT_, true );
        UpdateDisplay();
    }
    else
    {
        WriteLogMessage( wxT( "LUT Import operation canceled by user.\n" ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnInterpolate( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    WriteErrorMessage( wxString::Format( wxT( "%s(%d): Implementation still missing.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__ ) );
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnInvert( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    try
    {
        const vector<int>::size_type lutSize = activeLUT_.size();
        const int maxLUTValue = static_cast<int>( lc_.LUTValue.getMaxValue() );
        for( vector<int>::size_type i = 0; i < lutSize; i++ )
        {
            activeLUT_[i] = maxLUTValue - activeLUT_[i];
        }
        AddLUTToCache( pCBLUTSelection_->GetValue(), activeLUT_, true );
    }
    catch( const ImpactAcquireException& ex )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to invert LUT.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( ex.getErrorString() ).c_str(), ConvertedString( ex.getErrorCodeAsString() ).c_str() ) );
    }
    UpdateDisplay();
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnBtnSynchronize( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxArrayString syncChoices;
    syncChoices.Add( wxT( "Cache -> Device (uploads LUT data to the device)" ) );
    syncChoices.Add( wxT( "Device -> Cache (downloads LUT data from the device)" ) );
    const wxString syncSelection = ::wxGetSingleChoice( wxT( "Please select the direction of the synchronization.\n\n" ) \
                                   wxT( "As reading a LUT from a device as well as writing a LUT to a device\n" ) \
                                   wxT( "takes some time this is not always done automatically but must be explicitly\n" ) \
                                   wxT( "triggered from this dialog. When a LUT has been configured using this wizard\n" ) \
                                   wxT( "the 'Cache -> Device' option must be selected for transferring the new LUT\n" ) \
                                   wxT( "data to the device. To read back the current LUT settings that are effective\n" ) \
                                   wxT( "on the device the 'Device -> Cache' option must be selected." ),
                                   wxT( "LUT Control: Synchronize" ), syncChoices, this );
    if( syncSelection == wxEmptyString )
    {
        WriteLogMessage( wxT( "LUT synchronization canceled by user.\n" ) );
        return;
    }

    wxString LUTSelection = lutSelectorEntries_[0];
    if( lutCnt_ > 1 )
    {
        wxArrayString LUTChoices;
        if( lutCnt_ > 2 )
        {
            LUTChoices.Add( wxT( "All" ) );
        }
        for( size_t i = 0; i < lutCnt_; i++ )
        {
            LUTChoices.Add( lutSelectorEntries_[i] );
        }
        LUTSelection = ::wxGetSingleChoice( wxT( "Please select which LUT(s) you want to synchronize" ), wxT( "LUT Control: Synchronize" ), LUTChoices, this );
        if( LUTSelection == wxEmptyString )
        {
            WriteLogMessage( wxT( "LUT synchronization canceled by user.\n" ) );
            return;
        }
    }

    if( LUTSelection == wxT( "All" ) )
    {
        LUTMap::iterator it = lutMap_.begin();
        const LUTMap::iterator itEND = lutMap_.end();
        while( it != itEND )
        {
            SynchronizeLUT( syncSelection == syncChoices[0], it->first, it->second.lut_ );
            ++it;
        }
    }
    else
    {
        LUTMap::iterator it = lutMap_.find( LUTSelection );
        if( it == lutMap_.end() )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: LUT '%s' not found in cache.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, LUTSelection.c_str() ) );
        }
        else
        {
            SynchronizeLUT( syncSelection == syncChoices[0], it->first, it->second.lut_ );
        }
    }
    UpdateDisplay();
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnCBEnable( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    try
    {
        lc_.LUTEnable.write( e.IsChecked() ? bTrue : bFalse );
        WriteLogMessage( wxString::Format( wxT( "'%s' LUT %s.\n" ), pCBLUTSelection_->GetValue().c_str(), e.IsChecked() ? wxT( "enabled" ) : wxT( "disabled" ) ) );
    }
    catch( const ImpactAcquireException& ex )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to %s LUT Control.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( ex.getErrorString() ).c_str(), ConvertedString( ex.getErrorCodeAsString() ).c_str(), e.IsChecked() ? wxT( "enable" ) : wxT( "disable" ) ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnCBLUTSelectionChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    SetupLUTEnable();
    LUTMap::iterator it = lutMap_.find( pCBLUTSelection_->GetValue() );
    if( it == lutMap_.end() )
    {
        ReadLUTFromDevice( pCBLUTSelection_->GetValue(), activeLUT_ );
    }
    else
    {
        activeLUT_ = it->second.lut_;
    }
    UpdateDisplay( true );
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnClose( wxCloseEvent& e )
//-----------------------------------------------------------------------------
{
    Hide();
    if( e.CanVeto() )
    {
        e.Veto();
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnGammaParameterChanged( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    CalculateGammaLUT( activeLUT_, gammaData_ );
    UpdateDisplay();
}

//-----------------------------------------------------------------------------
void WizardLUTControl::OnNBDisplayMethodPageChanged( wxNotebookEvent& )
//-----------------------------------------------------------------------------
{
    displayMethod_ = static_cast<TDisplayMethod>( pNBDisplayMethod_->GetSelection() );
    UpdateDisplay();
}

//-----------------------------------------------------------------------------
void WizardLUTControl::ReadLUTFromDevice( const wxString& LUTSelectorValue, vector<int>& lut )
//-----------------------------------------------------------------------------
{
    PropGridUpdateTimerSuspendScope timerSuspendScope( dynamic_cast<PropGridFrameBase*>( GetParent() ) );
    try
    {
        if( lc_.LUTSelector.isValid() && lc_.LUTSelector.isWriteable() )
        {
            lc_.LUTSelector.writeS( string( LUTSelectorValue.mb_str() ) );
        }
        const int lutSize = lc_.LUTIndex.getMaxValue() + 1;
        lut.resize( static_cast<vector<int>::size_type>( lutSize ) );
        wxProgressDialog dlg( wxString::Format( wxT( "Reading LUT '%s' from device" ), LUTSelectorValue.c_str() ),
                              wxString::Format( wxT( "Reading LUT '%s' from device(%05d/%05d elements read)" ), LUTSelectorValue.c_str(), 0, lutSize ),
                              lutSize,  // range
                              this,     // parent
                              wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME );
        for( int i = 0; i < lutSize; i++ )
        {
            lc_.LUTIndex.write( i );
            lut[i] = static_cast<int>( lc_.LUTValue.read() );
            if( i % 50 == 0 )
            {
                dlg.Update( i, wxString::Format( wxT( "Reading LUT '%s' from device(%05d/%05d elements read)" ), LUTSelectorValue.c_str(), i, lutSize ) );
            }
        }

        AddLUTToCache( LUTSelectorValue, lut, false );
        WriteLogMessage( wxString::Format( wxT( "LUT '%s' successfully read from device.\n" ), LUTSelectorValue.c_str() ) );
    }
    catch( const ImpactAcquireException& e )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with LUT data.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::SetGridValue( int row, int /*col*/, const wxString& value )
//-----------------------------------------------------------------------------
{
    sscanf( value.mb_str(), GetGridValueFormatString().mb_str(), &activeLUT_[row] );
    LUTMap::iterator it = lutMap_.find( pCBLUTSelection_->GetValue() );
    if( it == lutMap_.end() )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: LUT '%s' not found in cache.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, pCBLUTSelection_->GetValue().c_str() ) );
    }
    else
    {
        if( it->second.lut_[row] != activeLUT_[row] )
        {
            it->second.lut_[row] = activeLUT_[row];
            it->second.boDirty_ = true;
            pBtnApply_->Enable( true );
        }
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::SetGridValueFormatString( const wxString& gridValueFormatString )
//-----------------------------------------------------------------------------
{
    m_GridValueFormatString = gridValueFormatString;
    WriteLogMessage( wxString::Format( wxT( "Grid value format string set to '%s'.\n" ), gridValueFormatString.c_str() ) );
    UpdateDisplay();
}

//-----------------------------------------------------------------------------
void WizardLUTControl::SetupLUTEnable( void )
//-----------------------------------------------------------------------------
{
    try
    {
        if( lc_.LUTSelector.isValid() && lc_.LUTSelector.isWriteable() )
        {
            lc_.LUTSelector.writeS( string( pCBLUTSelection_->GetValue().mb_str() ) );
        }
        pCBEnable_->SetValue( lc_.LUTEnable.read() == bTrue );
    }
    catch( const ImpactAcquireException& e )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with LUT data.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::SynchronizeLUT( const bool boCacheToDevice, const wxString& LUTSelectorValue, vector<int>& lut )
//-----------------------------------------------------------------------------
{
    if( boCacheToDevice == true )
    {
        WriteLUTToDevice( LUTSelectorValue, lut );
    }
    else
    {
        ReadLUTFromDevice( LUTSelectorValue, lut );
        const LUTMap::const_iterator it = lutMap_.find( pCBLUTSelection_->GetValue() );
        if( it != lutMap_.end() )
        {
            activeLUT_ = it->second.lut_;
        }
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::UpdateDialog( void )
//-----------------------------------------------------------------------------
{
    try
    {
        ReadLUTFromDevice( pCBLUTSelection_->GetValue(), activeLUT_ );
        pCBEnable_->SetValue( lc_.LUTEnable.read() == bTrue );
        UpdateDisplay();
    }
    catch( const ImpactAcquireException& e )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with LUT data.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::UpdateDisplay( bool boEraseBackground /* = true */ )
//-----------------------------------------------------------------------------
{
    switch( displayMethod_ )
    {
    case dmGraphical:
        try
        {
            pGraphicalDisplay_->RefreshData( activeLUT_, static_cast<int>( lc_.LUTValue.getMaxValue() ) );
            pGraphicalDisplay_->Refresh( boEraseBackground );
        }
        catch( const ImpactAcquireException& ex )
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to update LUT display.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( ex.getErrorString() ).c_str(), ConvertedString( ex.getErrorCodeAsString() ).c_str() ) );
        }
        break;
    case dmNumerical:
        {
            const int rowCount = pNumericalDisplay_->GetTable()->GetNumberRows();
            const int rowsNeeded = static_cast<int>( activeLUT_.size() );
            if( rowCount < rowsNeeded )
            {
                pNumericalDisplay_->AppendRows( rowsNeeded - rowCount );
            }
            else if( rowCount > rowsNeeded )
            {
                pNumericalDisplay_->DeleteRows( rowsNeeded - 1, rowCount - rowsNeeded );
            }
            for( int i = 0; i < rowsNeeded; i++ )
            {
                pNumericalDisplay_->SetCellValue( i, 0, wxString::Format( GetGridValueFormatString().c_str(), activeLUT_[i] ) );
            }
        }
        pNumericalDisplay_->Refresh( boEraseBackground );
        break;
    }
}

//-----------------------------------------------------------------------------
void WizardLUTControl::WriteLUTToDevice( const wxString& LUTSelectorValue, vector<int>& lut )
//-----------------------------------------------------------------------------
{
    PropGridUpdateTimerSuspendScope timerSuspendScope( dynamic_cast<PropGridFrameBase*>( GetParent() ) );
    try
    {
        if( lc_.LUTSelector.isValid() && lc_.LUTSelector.isWriteable() )
        {
            lc_.LUTSelector.writeS( string( LUTSelectorValue.mb_str() ) );
        }
        const int lutSize = static_cast<int>( lc_.LUTIndex.getMaxValue() + 1 );
        if( static_cast<int>( lut.size() ) != lutSize )
        {
            WriteLogMessage( wxString::Format( wxT( "LUT '%s' supports %d values, while the internal data structure contains %d entries. Internal data structure will be resized.\n" ), LUTSelectorValue.c_str(), lutSize, static_cast<int>( lut.size() ) ) );
            lut.resize( static_cast<vector<int>::size_type>( lutSize ) );
        }
        const int maxLUTValue = static_cast<int>( lc_.LUTValue.getMaxValue() );
        for( int i = 0; i < lutSize; i++ )
        {
            if( lut[i] > maxLUTValue )
            {
                WriteLogMessage( wxString::Format( wxT( "Value %d of LUT '%s' is too large(%d). Clipping to maximum(%d).\n" ), i, LUTSelectorValue.c_str(), lut[i], maxLUTValue ) );
                lut[i] = maxLUTValue;
            }
        }
        wxProgressDialog dlg( wxString::Format( wxT( "Writing data to LUT '%s' of device" ), LUTSelectorValue.c_str() ),
                              wxString::Format( wxT( "Writing data to LUT '%s' of device(%05d/%05d elements written)" ), LUTSelectorValue.c_str(), 0, lutSize ),
                              lutSize,  // range
                              this,     // parent
                              wxPD_AUTO_HIDE | wxPD_APP_MODAL | wxPD_ELAPSED_TIME | wxPD_ESTIMATED_TIME | wxPD_REMAINING_TIME );
        for( int i = 0; i < lutSize; i++ )
        {
            lc_.LUTIndex.write( i );
            lc_.LUTValue.write( static_cast<int>( lut[i] ) );
            if( i % 50 == 0 )
            {
                dlg.Update( i, wxString::Format( wxT( "Writing data to LUT '%s' of device(%05d/%05d elements written)" ), LUTSelectorValue.c_str(), i, lutSize ) );
            }
        }
        AddLUTToCache( LUTSelectorValue, lut, false );
        WriteLogMessage( wxString::Format( wxT( "LUT '%s' successfully written to device.\n" ), LUTSelectorValue.c_str() ) );
    }
    catch( const ImpactAcquireException& e )
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): Internal error: %s(%s) while trying to deal with LUT data.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ) );
    }
    UpdateDisplay();
}

//=============================================================================
//============== Implementation WizardLUTGammaDlg =============================
//=============================================================================
BEGIN_EVENT_TABLE( WizardLUTGammaDlg, OkAndCancelDlg )
    EVT_SPINCTRL( widSCGamma, WizardLUTGammaDlg::OnGammaChanged )
    EVT_SPINCTRL( widSCGammaAlpha, WizardLUTGammaDlg::OnGammaAlphaChanged )
    EVT_TEXT( widCBGammaMode, WizardLUTGammaDlg::OnCBGammaModeChanged )
    EVT_SPINCTRL( widSCGammaStartThreshold, WizardLUTGammaDlg::OnGammaStartThresholdChanged )

#ifdef BUILD_WITH_TEXT_EVENTS_FOR_SPINCTRL // Unfortunately on Linux wxWidgets 2.6.x - ??? handling these messages will cause problems, while on Windows not doing so will not always update the GUI as desired :-(
    EVT_TEXT( widSCGamma, WizardLUTGammaDlg::OnGammaTextChanged )
    EVT_TEXT( widSCGammaAlpha, WizardLUTGammaDlg::OnGammaAlphaTextChanged )
    EVT_TEXT( widSCGammaStartThreshold, WizardLUTGammaDlg::OnGammaStartThresholdTextChanged )
#endif // #ifdef BUILD_WITH_TEXT_EVENTS_FOR_SPINCTRL
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
WizardLUTGammaDlg::WizardLUTGammaDlg( wxWindow* pParent, const wxString& title, const int maxLUTIndex, GammaData& data )
    : OkAndCancelDlg( pParent, wxID_ANY, title ), data_( data )
//-----------------------------------------------------------------------------
{
    /*
        |-------------------------------------|
        | pTopDownSizer                       |
        |                spacer               |
        | |---------------------------------| |
        | | pGridSizer                      | |
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
    wxFlexGridSizer* pGridSizer = new wxFlexGridSizer( 2 );
    pGridSizer->AddGrowableCol( 1, 2 );

    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( " Gamma:" ) ) );

    pSCGamma_ = new wxSpinCtrlDbl();
    pSCGamma_->Create( pPanel, widSCGamma, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0.01, 10., data.gamma_, 0.01, wxSPINCTRLDBL_AUTODIGITS, wxT( "%.2f" ) );
    pSCGamma_->SetMode( mDouble );
    pGridSizer->Add( pSCGamma_, wxSizerFlags().Expand() );

    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( " Gamma Alpha:" ) ) );
    pSCGammaAlpha_ = new wxSpinCtrlDbl();
    pSCGammaAlpha_->Create( pPanel, widSCGammaAlpha, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0.0, 1., data.gammaAlpha_, 0.001, wxSPINCTRLDBL_AUTODIGITS, wxT( "%.3f" ) );
    pSCGammaAlpha_->SetMode( mDouble );
    pGridSizer->Add( pSCGammaAlpha_, wxSizerFlags().Expand() );

    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( " Gamma Mode:" ) ) );
    wxArrayString gammaModeChoices;
    gammaModeChoices.Add( wxT( "Standard" ) );
    gammaModeChoices.Add( wxT( "Linear Start" ) );
    pCBGammaMode_ = new wxComboBox( pPanel, widCBGammaMode, ( data.gammaMode_ == LUTgmLinearStart ) ? wxT( "Linear Start" ) : wxT( "Standard" ), wxDefaultPosition, wxDefaultSize, gammaModeChoices, wxCB_DROPDOWN | wxCB_READONLY );
    pGridSizer->Add( pCBGammaMode_, wxSizerFlags().Expand() );

    pSTGammaStartThreshold_ = new wxStaticText( pPanel, wxID_ANY, wxT( " Gamma Start Threshold:" ) );
    pGridSizer->Add( pSTGammaStartThreshold_ );
    pSCGammaStartThreshold_ = new wxSpinCtrl( pPanel, widSCGammaStartThreshold, wxString::Format( wxT( "%d" ), data.gammaStartThreshold_ ), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, maxLUTIndex, data.gammaStartThreshold_ );
    pGridSizer->Add( pSCGammaStartThreshold_, wxSizerFlags().Expand() );

    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( " " ) ) );
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( " " ) ) );
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( " Gamma Formula:" ) ) );
    pGridSizer->Add( new wxStaticText( pPanel, wxID_ANY, wxT( " (1 + GammaAlpha)*inputValue^(1/Gamma) - GammaAlpha " ) ), wxSizerFlags().Expand() );

    pTopDownSizer->Add( pGridSizer );
    AddButtons( pPanel, pTopDownSizer );
    FinalizeDlgCreation( pPanel, pTopDownSizer );
    SetupControls();
}

//-----------------------------------------------------------------------------
void WizardLUTGammaDlg::SetupControls( void )
//-----------------------------------------------------------------------------
{
    pSTGammaStartThreshold_->Enable( pCBGammaMode_->GetValue() == wxT( "Linear Start" ) );
    pSCGammaStartThreshold_->Enable( pCBGammaMode_->GetValue() == wxT( "Linear Start" ) );
}
