#include <algorithm>
#include <apps/Common/wxAbstraction.h>
#include <common/STLHelper.h>
#include "PropData.h"
#include "SpinEditorDouble.h"
#include "WizardSequencerControl.h"
#include "WizardIcons.h"
#include <wx/combobox.h>
#include <wx/platinfo.h>
#include <wx/propgrid/advprops.h>
#include <wx/spinctrl.h>

using namespace std;

//=============================================================================
//============== Implementation WizardSequencerControl ========================
//=============================================================================
BEGIN_EVENT_TABLE( WizardSequencerControl, OkAndCancelDlg )
    EVT_BUTTON( widAddAnotherSetButton, WizardSequencerControl::OnBtnAddAnotherSet )
    EVT_BUTTON( widRemoveSelectedSetButton, WizardSequencerControl::OnBtnRemoveSelectedSet )
    EVT_BUTTON( widAutoAssignSetsToDisplays, WizardSequencerControl::OnBtnAutoAssignSetsToDisplays )
    EVT_CHECKBOX( widSettingsCheckBox, WizardSequencerControl::OnSettingsCheckBoxChecked )
    EVT_NOTEBOOK_PAGE_CHANGED( widSequencerSetsNotebook, WizardSequencerControl::OnNBSequencerSetsPageChanged )
    EVT_SPINCTRL( widNextSetSpinControl, WizardSequencerControl::OnNextSetSpinControlChanged )
END_EVENT_TABLE()

const wxString WizardSequencerControl::s_displayStringPrefix_( wxT( "Display " ) );
const wxString WizardSequencerControl::s_sequencerSetStringPrefix_( wxT( "Set " ) );

//-----------------------------------------------------------------------------
WizardSequencerControl::WizardSequencerControl( PropViewFrame* pParent, const wxString& title, mvIMPACT::acquire::Device* pDev, size_t displayCount, SequencerSetToDisplayMap& setToDisplayTable )
    : OkAndCancelDlg( pParent, widMainFrame, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxMINIMIZE_BOX ),
      pParent_( pParent ), pDev_( pDev ), sc_( pDev ), setToDisplayTable_( setToDisplayTable ), currentErrors_(), sequencerFeatures_(), sequencerNextSetMap_(), sequencerSetControlsMap_(), startingSequencerSet_( 0 ),
      maxSequencerSetNumber_( 0 ), displayCount_( displayCount ), boCounterDurationCapability_( false ), pSetSettingsNotebook_( 0 ), pStaticBitmapWarning_( 0 ), pInfoText_( 0 )
//-----------------------------------------------------------------------------
{
    /*
        |-------------------------------------|
        | pTopDownSizer                       |
        |                spacer               |
        | |---------------------------------| |
        | | pSettingsSizer                  | |
        | | |--------| |------------------| | |
        | | | Global | |   Set-Specific   | | |
        | | |        | |                  | | |
        | | |Settings| |     Settings     | | |
        | | |--------| |------------------| | |
        | |---------------------------------| |
        |                spacer               |
        | |---------------------------------| |
        | | pButtonSizer                    | |
        | |---------------------------------| |
        |-------------------------------------|
    */

    wxPanel* pMainPanel = new wxPanel( this );
    const size_t MAX_SEQUENCER_FEATURES = 64;
    wxCheckBox* pSettingsCheckBoxes[MAX_SEQUENCER_FEATURES];
    const wxBitmap warningBitmap( wizard_empty_xpm );

    AnalyzeSequencerSetFeatures();
    InquireCurrentlyActiveSequencerSets();

    // add a check box for each "sequencable" property
    int featureNumber = 0;
    const SequencerFeatureContainer::const_iterator itEND = sequencerFeatures_.end();
    for( SequencerFeatureContainer::iterator it = sequencerFeatures_.begin(); it != itEND; it++ )
    {
        try
        {
            Component c( it->first );
            if( sc_.sequencerFeatureSelector.isValid() )
            {
                sc_.sequencerFeatureSelector.writeS( c.name() );
            }
            pSettingsCheckBoxes[featureNumber] = new wxCheckBox( pMainPanel, widSettingsCheckBox, ConvertedString( c.name() ).c_str() );
            pSettingsCheckBoxes[featureNumber]->SetValue( it->second );
            pSettingsCheckBoxes[featureNumber]->Enable( sc_.sequencerFeatureEnable.isWriteable() );
            // special handling of the CounterDuration Feature, extra Information by adding a tool tip
            if( ConvertedString( c.name() ).IsSameAs( "CounterDuration" ) )
            {
                pSettingsCheckBoxes[featureNumber]->SetToolTip( wxT( "The Number Of Images To Be\nTaken With Each Sequencer Set." ) );
            }
        }
        catch( const ImpactAcquireException& e )
        {
            wxMessageBox( wxString::Format( wxT( "An exception was raised: %s(%s)" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Internal Problem" ), wxICON_EXCLAMATION );
        }
        ++featureNumber;
    }

    pSetSettingsNotebook_ = new wxNotebook( pMainPanel, widSequencerSetsNotebook, wxDefaultPosition, wxDefaultSize, wxNB_TOP );
    int64_type setCount = 0;
    for( map<int64_type, int64_type>::iterator it = sequencerNextSetMap_.begin(); it != sequencerNextSetMap_.end(); it++ )
    {
        ComposeSetPanel( displayCount, setCount, it->first, false );
        sc_.sequencerSetSelector.write( sc_.sequencerSetNext.read() );
        ++setCount;
    }

    pStaticBitmapWarning_ = new wxStaticBitmap( pMainPanel, widStaticBitmapWarning, warningBitmap, wxDefaultPosition, wxDefaultSize, 0, wxT( "" ) );
    pInfoText_ = new wxStaticText( pMainPanel, wxID_ANY, wxT( "" ), wxDefaultPosition, wxSize( 360, WXGRID_DEFAULT_ROW_HEIGHT ), wxST_ELLIPSIZE_MIDDLE );
    pBtnOk_ = new wxButton( pMainPanel, widBtnOk, wxT( "&Quit And Run" ) );
    pBtnOk_->SetToolTip( wxT( "Closes this dialog, starts the sequencer and the acquisition" ) );
    pBtnCancel_ = new wxButton( pMainPanel, widBtnCancel, wxT( "&Quit" ) );
    pBtnCancel_->SetToolTip( wxT( "Just closes this dialog but does NOT start the sequencer or the acquisition. Changes applied to the sequencer will continue to prevail" ) );

    // putting it all together
    const int minSpace = ( ( wxPlatformInfo().GetOperatingSystemId() & wxOS_WINDOWS ) != 0 ) ? 2 : 1;

    wxStaticBoxSizer* pFeaturesSizer = new wxStaticBoxSizer( wxVERTICAL, pMainPanel, wxT( "Sequencer Properties" ) );
    pFeaturesSizer->AddSpacer( 4 * minSpace );
    featureNumber = 0;
    for( SequencerFeatureContainer::iterator it = sequencerFeatures_.begin(); it != itEND; it++ )
    {
        pFeaturesSizer->AddSpacer( 2 * minSpace );
        pFeaturesSizer->Add( pSettingsCheckBoxes[featureNumber] );
        featureNumber++;
    }

    wxStaticBoxSizer* pSetManagementSizer = new wxStaticBoxSizer( wxVERTICAL, pMainPanel, wxT( "Set Management" ) );
    pSetManagementSizer->Add( new wxButton( pMainPanel, widAddAnotherSetButton, wxT( "Add Another Set" ) ), wxSizerFlags().Expand().Border( wxALL, 2 ) );
    pSetManagementSizer->Add( new wxButton( pMainPanel, widRemoveSelectedSetButton, wxT( "Remove Selected Set" ) ), wxSizerFlags().Expand().Border( wxALL, 2 ) );

    wxStaticBoxSizer* pDisplayManagementSizer = new wxStaticBoxSizer( wxVERTICAL, pMainPanel, wxT( "Display Management" ) );
    wxButton* pAutoAssignDisplaysToSetsButton = new wxButton( pMainPanel, widAutoAssignSetsToDisplays, wxT( "Auto-Assign Displays To Sets" ) );
    pAutoAssignDisplaysToSetsButton->SetToolTip( wxT( "Automatically Creates A Display Matrix\nBig Enough To Accommodate All Sets\nAnd Assigns Each Set To A Display" ) );
    pDisplayManagementSizer->Add( pAutoAssignDisplaysToSetsButton, wxSizerFlags().Expand().Border( wxALL, 2 ) );

    wxBoxSizer* pGlobalSettingsSizer = new wxBoxSizer( wxVERTICAL );
    pGlobalSettingsSizer->Add( pFeaturesSizer, wxSizerFlags( 6 ).Expand() );
    pGlobalSettingsSizer->Add( pSetManagementSizer, wxSizerFlags( 3 ).Expand() );
    pGlobalSettingsSizer->Add( pDisplayManagementSizer, wxSizerFlags( 2 ).Expand() );

    wxBoxSizer* pSettingsSizer = new wxBoxSizer( wxHORIZONTAL );
    pSettingsSizer->AddSpacer( 5 );
    pSettingsSizer->Add( pGlobalSettingsSizer, wxSizerFlags( 2 ).Expand() );
    pSettingsSizer->AddSpacer( 2 * minSpace );
    pSettingsSizer->Add( pSetSettingsNotebook_, wxSizerFlags( 5 ).Expand() );
    pSettingsSizer->AddSpacer( 2 * minSpace );

    // customizing the last line of buttons
    wxBoxSizer* pButtonSizer = new wxBoxSizer( wxHORIZONTAL );
    pButtonSizer->Add( pStaticBitmapWarning_, wxSizerFlags().Border( wxALL, 7 ) );
    wxBoxSizer* pSmallInfoTextAlignmentSizer = new wxBoxSizer( wxVERTICAL );
    pSmallInfoTextAlignmentSizer->AddSpacer( 15 );
    wxFont font = pInfoText_->GetFont();
    font.SetWeight( wxFONTWEIGHT_BOLD );
    pInfoText_->SetFont( font );
    pInfoText_->SetForegroundColour( wxColour( 255, 0, 0 ) );
    pSmallInfoTextAlignmentSizer->Add( pInfoText_ );
    pButtonSizer->Add( pSmallInfoTextAlignmentSizer );
    pButtonSizer->AddStretchSpacer( 10 );
    pButtonSizer->Add( pBtnOk_, wxSizerFlags().Border( wxALL, 7 ) );
    pButtonSizer->Add( pBtnCancel_, wxSizerFlags().Border( wxALL, 7 ) );

    wxBoxSizer* pTopDownSizer = new wxBoxSizer( wxVERTICAL );
    pTopDownSizer->AddSpacer( 10 );
    pTopDownSizer->Add( pSettingsSizer, wxSizerFlags( 10 ).Expand() );
    pTopDownSizer->AddSpacer( 10 );
    pTopDownSizer->Add( pButtonSizer, wxSizerFlags().Expand() );

    pMainPanel->SetSizer( pTopDownSizer );
    pTopDownSizer->SetSizeHints( this );
    SetClientSize( pTopDownSizer->GetMinSize() );
    SetSizeHints( GetSize() );
    Center();

    LoadSequencerSets();
    SelectSequencerSetAndCallMethod( GetSelectedSequencerSetNr(), sc_.sequencerSetLoad );
    RefreshCurrentPropertyGrid();
}

//-----------------------------------------------------------------------------
WizardSequencerControl::~WizardSequencerControl()
//-----------------------------------------------------------------------------
{
    for_each( sequencerSetControlsMap_.begin(), sequencerSetControlsMap_.end(), ptr_fun( DeleteSecond<const int64_type, SequencerSetControls*> ) );
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::AddError( SequencerErrorInfo& errorInfo )
//-----------------------------------------------------------------------------
{
    RemoveError( errorInfo );
    currentErrors_.push_back( errorInfo );
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::AnalyzeSequencerSetFeatures( void )
//-----------------------------------------------------------------------------
{
    // check for "sequencable" properties supported by the device
    vector<string> selectableFeaturesForSequencer;
    if( sc_.sequencerFeatureSelector.isValid() && sc_.sequencerSetLoad.isValid() )
    {
        sc_.sequencerFeatureSelector.getTranslationDictStrings( selectableFeaturesForSequencer );
        const vector<string>::size_type selectableFeaturesForSequencerCount = selectableFeaturesForSequencer.size();
        ComponentLocator locator( sc_.sequencerSetSelector.parent().parent() );
        int64_type originalSelectorPosition = sc_.sequencerFeatureSelector.read();
        for( vector<string>::size_type i = 0; i < selectableFeaturesForSequencerCount; i++ )
        {
            sc_.sequencerFeatureSelector.writeS( selectableFeaturesForSequencer[i] );
            const bool boEnabled = static_cast< TBoolean >( sc_.sequencerFeatureEnable.read() ) == bTrue;
            const HOBJ hObj = locator.findComponent( selectableFeaturesForSequencer[i] );
            if( hObj != INVALID_ID )
            {
                sequencerFeatures_.push_back( make_pair( hObj, boEnabled ) );
            }
        }
        sc_.sequencerFeatureSelector.write( originalSelectorPosition );
    }

    // Check for the CounterDuration feature ( relevant for the implementation for mv cameras )
    if( sc_.sequencerTriggerSource.isValid() && sc_.sequencerTriggerSource.isWriteable() )
    {
        vector<string> sequencerTriggerSourceValidValues;
        sc_.sequencerTriggerSource.getTranslationDictStrings( sequencerTriggerSourceValidValues );
        if( find( sequencerTriggerSourceValidValues.begin(), sequencerTriggerSourceValidValues.end(), "Counter1End" ) != sequencerTriggerSourceValidValues.end() &&
            find( sequencerTriggerSourceValidValues.begin(), sequencerTriggerSourceValidValues.end(), "ExposureEnd" ) != sequencerTriggerSourceValidValues.end() )
        {
            boCounterDurationCapability_ = true;
        }
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::CheckIfActiveSetsAreReferencedAtLeastOnce( void )
//-----------------------------------------------------------------------------
{
    SequencerErrorInfo errorInfo;
    errorInfo.errorType = swetSetsNotBeingReferenced;
    for( SetNrToControlsMap::iterator activeSetIterator = sequencerSetControlsMap_.begin(); activeSetIterator != sequencerSetControlsMap_.end(); activeSetIterator++ )
    {
        const int64_type currentSet = activeSetIterator->first;
        bool boSetIsBeingReferenced = false;
        for( SetNrToControlsMap::iterator referencingSetIterator = sequencerSetControlsMap_.begin(); referencingSetIterator != sequencerSetControlsMap_.end(); referencingSetIterator++ )
        {
            int64_type referencingSetNextSetSetting = referencingSetIterator->second->pSequencerSetNextSC->GetValue();
            if( currentSet == referencingSetNextSetSetting )
            {
                boSetIsBeingReferenced = true;
                break;
            }
        }
        errorInfo.setNumber = currentSet;
        errorInfo.nextSet = -1;
        if( boSetIsBeingReferenced )
        {
            activeSetIterator->second->pSetPanel->SetBackgroundColour( *wxWHITE );
            activeSetIterator->second->pSetPanel->SetToolTip( 0 );
            RemoveError( errorInfo );
        }
        else
        {
            activeSetIterator->second->pSetPanel->SetBackgroundColour( wxColour( 255, 200, 200 ) );
            activeSetIterator->second->pSetPanel->SetToolTip( wxT( "This set is not set as 'NextSet' by any other set!\nPlease review your sequencer sets configuration!" ) );
            AddError( errorInfo );
        }
    }
    UpdateErrors();
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::CheckIfReferencedSetsAreActive( void )
//-----------------------------------------------------------------------------
{
    SequencerErrorInfo errorInfo;
    errorInfo.errorType = swetReferenceToInactiveSet;
    for( SetNrToControlsMap::iterator referencedSetIterator = sequencerSetControlsMap_.begin(); referencedSetIterator != sequencerSetControlsMap_.end(); referencedSetIterator++ )
    {
        const int64_type referencedSet = referencedSetIterator->second->pSequencerSetNextSC->GetValue();
        bool boReferencedSetIsActive = false;
        for( SetNrToControlsMap::iterator activeSetIterator = sequencerSetControlsMap_.begin(); activeSetIterator != sequencerSetControlsMap_.end(); activeSetIterator++ )
        {
            int64_type currentSet = activeSetIterator->first;
            if( referencedSet == currentSet )
            {
                boReferencedSetIsActive = true;
                break;
            }
        }
        errorInfo.setNumber = referencedSetIterator->first;
        errorInfo.nextSet = referencedSet;
        if( boReferencedSetIsActive )
        {
            referencedSetIterator->second->pSequencerSetNextSC->SetBackgroundColour( *wxWHITE );
            referencedSetIterator->second->pSequencerSetNextSC->SetToolTip( 0 );
            RemoveError( errorInfo );
        }
        else
        {
            referencedSetIterator->second->pSequencerSetNextSC->SetBackgroundColour( wxColour( 255, 200, 200 ) );
            referencedSetIterator->second->pSequencerSetNextSC->SetToolTip( wxT( "This set not active!\nPlease correct this value so\nthat it points to an active set!" ) );
            AddError( errorInfo );
        }
    }
    UpdateErrors();
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::ComposeSetPanel( size_t displayCount, int64_type& setCount, int64_type setNumber, bool boInsertOnFirstFreePosition )
//-----------------------------------------------------------------------------
{
    sc_.sequencerSetSelector.write( ( setCount == 0 ) ? sc_.sequencerSetStart.read() : setNumber );

    SequencerSetControls* pSequencerSet = new SequencerSetControls();
    pSequencerSet->pSetPanel = new wxPanel( pSetSettingsNotebook_, widStaticBitmapWarning + setNumber + 1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, wxString::Format( wxT( "%s%d" ), s_sequencerSetStringPrefix_.c_str(), static_cast< int >( setNumber ) ) );

    // workaround for correct notebook background colors on all platforms
    const wxColour defaultCol = pSetSettingsNotebook_->GetThemeBackgroundColour();
    if( defaultCol.IsOk() )
    {
        pSequencerSet->pSetPanel->SetBackgroundColour( defaultCol );
    }

    pSequencerSet->pSetPropertyGrid = new wxPropertyGrid( pSequencerSet->pSetPanel, pSequencerSet->pSetPanel->GetId() + maxSequencerSetNumber_, wxDefaultPosition, wxDefaultSize, wxPG_BOLD_MODIFIED | wxPG_STATIC_SPLITTER | wxPG_SPLITTER_AUTO_CENTER, pSequencerSet->pSetPanel->GetName() );
    Bind( wxEVT_PG_CHANGED, &WizardSequencerControl::OnPropertyChanged, this, pSequencerSet->pSetPanel->GetId() + maxSequencerSetNumber_ );
    UpdatePropertyGrid( pSequencerSet );
    pSequencerSet->pSetVerticalSizer = new wxBoxSizer( wxVERTICAL );
    pSequencerSet->pSetHorizontalSizer = new wxBoxSizer( wxHORIZONTAL );
    pSequencerSet->pOptionsSizer = new wxBoxSizer( wxHORIZONTAL );
    pSequencerSet->pDisplayToUseCB = new wxComboBox( pSequencerSet->pSetPanel, wxID_ANY );
    for( size_t i = 0; i < displayCount; i++ )
    {
        pSequencerSet->pDisplayToUseCB->Append( wxString::Format( wxT( "%s%d" ), s_displayStringPrefix_.c_str(), static_cast<int>( i ) ) );
    }
    if( setToDisplayTable_.empty() == true ||
        setToDisplayTable_.find( setNumber ) == setToDisplayTable_.end() )
    {
        pSequencerSet->pDisplayToUseCB->Select( static_cast<size_t>( setCount % displayCount ) );
    }
    else
    {
        SequencerSetToDisplayMap::const_iterator displayIt = setToDisplayTable_.find( setNumber );
        pSequencerSet->pDisplayToUseCB->Select( displayIt->second );
    }

    pSequencerSet->pSequencerSetNextSC = new wxSpinCtrl( pSequencerSet->pSetPanel, widNextSetSpinControl );
    pSequencerSet->pSequencerSetNextSC->SetRange( 0, sc_.sequencerSetSelector.getMaxValue() );
    pSequencerSet->pSequencerSetNextSC->SetValue( sc_.sequencerSetNext.read() );

    pSequencerSet->pOptionsSizer->AddSpacer( 5 );
    pSequencerSet->pOptionsSizer->Add( new wxStaticText( pSequencerSet->pSetPanel, wxID_ANY, wxT( "Show On Display:" ) ) );
    pSequencerSet->pOptionsSizer->AddSpacer( 2 );
    pSequencerSet->pOptionsSizer->Add( pSequencerSet->pDisplayToUseCB );
    pSequencerSet->pOptionsSizer->AddSpacer( 10 );
    pSequencerSet->pOptionsSizer->Add( new wxStaticText( pSequencerSet->pSetPanel, wxID_ANY, wxT( "Next Set:" ) ) );
    pSequencerSet->pOptionsSizer->AddSpacer( 2 );
    pSequencerSet->pOptionsSizer->Add( pSequencerSet->pSequencerSetNextSC );
    pSequencerSet->pOptionsSizer->AddSpacer( 2 );

    pSequencerSet->pSetVerticalSizer->AddSpacer( 2 );
    pSequencerSet->pSetVerticalSizer->Add( pSequencerSet->pSetPropertyGrid, wxSizerFlags( 9 ).Expand() );
    pSequencerSet->pSetVerticalSizer->AddSpacer( 2 );
    pSequencerSet->pSetVerticalSizer->Add( pSequencerSet->pOptionsSizer, wxSizerFlags( 1 ).Align( wxALIGN_RIGHT ) );
    pSequencerSet->pSetVerticalSizer->AddSpacer( 2 );

    pSequencerSet->pSetHorizontalSizer->AddSpacer( 2 );
    pSequencerSet->pSetHorizontalSizer->Add( pSequencerSet->pSetVerticalSizer, wxSizerFlags( 10 ).Expand() );
    pSequencerSet->pSetHorizontalSizer->AddSpacer( 2 );

    pSequencerSet->pSetPanel->SetSizer( pSequencerSet->pSetHorizontalSizer );

    sequencerSetControlsMap_.insert( make_pair( setNumber, pSequencerSet ) );

    if( boInsertOnFirstFreePosition )
    {
        // determine the position in which the new Tab will be inserted.
        int64_type position = -1;
        for( SetNrToControlsMap::iterator it = sequencerSetControlsMap_.begin(); it != sequencerSetControlsMap_.end(); it++ )
        {
            position++;
            if( it->first == setNumber )
            {
                break;
            }
        }
        LoadSequencerSet( setNumber );
        SelectSequencerSetAndCallMethod( GetSelectedSequencerSetNr(), sc_.sequencerSetLoad );
        pSetSettingsNotebook_->InsertPage( position, pSequencerSet->pSetPanel, pSequencerSet->pSetPanel->GetName(), true );
        HandleAutomaticNextSetSettingsOnInsert( position, setNumber );
    }
    else
    {
        pSetSettingsNotebook_->AddPage( pSequencerSet->pSetPanel, pSequencerSet->pSetPanel->GetName(), false );
    }
}

//-----------------------------------------------------------------------------
WizardSequencerControl::SetNrToControlsMap::const_iterator WizardSequencerControl::GetSelectedSequencerSetControls( void ) const
//-----------------------------------------------------------------------------
{
    return GetSequencerSetControlsFromPageIndex( pSetSettingsNotebook_->GetSelection() );
}

//-----------------------------------------------------------------------------
int64_type WizardSequencerControl::GetSelectedSequencerSetNr( void ) const
//-----------------------------------------------------------------------------
{
    SetNrToControlsMap::const_iterator it = GetSelectedSequencerSetControls();
    return ( it == sequencerSetControlsMap_.end() ) ? -1LL : it->first;
}

//-----------------------------------------------------------------------------
WizardSequencerControl::SetNrToControlsMap::const_iterator WizardSequencerControl::GetSequencerSetControlsFromPageIndex( size_t pageNr ) const
//-----------------------------------------------------------------------------
{
    wxWindow* pPage = pSetSettingsNotebook_->GetPage( pageNr );
    SetNrToControlsMap::const_iterator itEND = sequencerSetControlsMap_.end();
    for( SetNrToControlsMap::const_iterator it = sequencerSetControlsMap_.begin(); it != itEND; it++ )
    {
        if( it->second->pSetPanel == pPage )
        {
            return it;
        }
    }
    return itEND;
}

//-----------------------------------------------------------------------------
int64_type WizardSequencerControl::GetSequencerSetNrFromPageIndex( size_t pageNr ) const
//-----------------------------------------------------------------------------
{
    SetNrToControlsMap::const_iterator it = GetSequencerSetControlsFromPageIndex( pageNr );
    return ( it == sequencerSetControlsMap_.end() ) ? -1LL : it->first;
}

//-----------------------------------------------------------------------------
const SequencerSetToDisplayMap& WizardSequencerControl::GetSetToDisplayTable( void )
//-----------------------------------------------------------------------------
{
    setToDisplayTable_.clear();
    for( SetNrToControlsMap::iterator it = sequencerSetControlsMap_.begin(); it != sequencerSetControlsMap_.end(); it++  )
    {
        setToDisplayTable_.insert( make_pair( static_cast<long>( it->first ), wxAtoi( it->second->pDisplayToUseCB->GetValue().substr( s_displayStringPrefix_.length() ) ) ) );
    }
    return setToDisplayTable_;
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::HandleAutomaticNextSetSettingsOnInsert( unsigned int tabPageNumber, unsigned int setNumber )
//-----------------------------------------------------------------------------
{
    // if another page exists after the inserted one, overwrite the nextSet setting with the number of the set of the next page.
    // if not then overwrite  the nextSet setting with the number of the set of the first page.
    const int nextTab = wxAtoi( pSetSettingsNotebook_->GetPage( ( tabPageNumber < pSetSettingsNotebook_->GetPageCount() - 1 ) ? tabPageNumber + 1 : 0 )->GetName().substr( s_sequencerSetStringPrefix_.length() ) );
    sequencerSetControlsMap_.find( setNumber )->second->pSequencerSetNextSC->SetValue( nextTab );

    // if another page exists before the inserted one, overwrite the nextSet setting of that page with the currently inserted one.
    if( tabPageNumber > 0 )
    {
        const int previousTab = wxAtoi( pSetSettingsNotebook_->GetPage( tabPageNumber - 1 )->GetName().substr( s_sequencerSetStringPrefix_.length() ) );
        SetNrToControlsMap::const_iterator it = sequencerSetControlsMap_.find( previousTab );
        it->second->pSequencerSetNextSC->SetValue( setNumber );
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::HandleAutomaticNextSetSettingsOnRemove( unsigned int tabPageNumber, unsigned int setNumber )
//-----------------------------------------------------------------------------
{
    // if another page exists before the removed one, overwrite the nextSet setting of that page with the one which replaced the removed.
    if( tabPageNumber > 0 )
    {
        const int previousTab = wxAtoi( pSetSettingsNotebook_->GetPage( tabPageNumber - 1 )->GetName().substr( s_sequencerSetStringPrefix_.length() ) );
        SetNrToControlsMap::const_iterator it = sequencerSetControlsMap_.find( previousTab );
        it->second->pSequencerSetNextSC->SetValue( setNumber );
    }

    // if another page exists after the removed one, overwrite the nextSet setting with the number of the set of the next page.
    // if not then overwrite  the nextSet setting with the number of the set of the first page.
    const int nextTab = wxAtoi( pSetSettingsNotebook_->GetPage( ( tabPageNumber < pSetSettingsNotebook_->GetPageCount() - 1 ) ? tabPageNumber + 1 : 0 )->GetName().substr( s_sequencerSetStringPrefix_.length() ) );
    sequencerSetControlsMap_.find( setNumber )->second->pSequencerSetNextSC->SetValue( nextTab );
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::InquireCurrentlyActiveSequencerSets( void )
//-----------------------------------------------------------------------------
{
    if( sc_.sequencerSetStart.isValid() && sc_.sequencerSetSelector.isValid() )
    {
        startingSequencerSet_ = sc_.sequencerSetStart.read();
        sc_.sequencerSetSelector.write( startingSequencerSet_ );
        maxSequencerSetNumber_ = sc_.sequencerSetSelector.getMaxValue();
        for( int64_type i = 0; i <= maxSequencerSetNumber_; i++ )
        {
            const int64_type nextSet = sc_.sequencerSetNext.read();
            sequencerNextSetMap_.insert( make_pair( sc_.sequencerSetSelector.read(), nextSet ) );
            if( sequencerNextSetMap_.find( nextSet ) != sequencerNextSetMap_.end() )
            {
                break;
            }
            sc_.sequencerSetSelector.write( nextSet );
        }
        sc_.sequencerSetSelector.write( startingSequencerSet_ );
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::LoadSequencerSet( int64_type setNumber )
//-----------------------------------------------------------------------------
{
    if( sc_.sequencerSetSelector.isValid() && sc_.sequencerSetLoad.isValid() )
    {
        int64_type originalSetting = sc_.sequencerSetSelector.read();
        SetNrToControlsMap::iterator it = sequencerSetControlsMap_.find( setNumber );
        ReadPropertiesOfSequencerSet( it );
        sc_.sequencerSetSelector.write( originalSetting );
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::LoadSequencerSets( void )
//-----------------------------------------------------------------------------
{
    if( sc_.sequencerSetSelector.isValid() && sc_.sequencerSetLoad.isValid() )
    {
        int64_type originalSetting = sc_.sequencerSetSelector.read();
        for( SetNrToControlsMap::iterator it = sequencerSetControlsMap_.begin(); it != sequencerSetControlsMap_.end(); it++ )
        {
            ReadPropertiesOfSequencerSet( it );
        }
        sc_.sequencerSetSelector.write( originalSetting );
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::OnBtnAddAnotherSet( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    int64_type setCount = sequencerSetControlsMap_.size();
    if( setCount > maxSequencerSetNumber_ )
    {
        wxMessageBox( wxT( "Maximum Number Of Supported Sequencer Sets Reached!" ), wxT( "Cannot Add Another Set!" ), wxICON_EXCLAMATION );
        return;
    }
    for( int64_type newSetCandidate = 0; newSetCandidate <= maxSequencerSetNumber_; newSetCandidate++ )
    {
        SetNrToControlsMap::iterator it = sequencerSetControlsMap_.find( newSetCandidate );
        if( it == sequencerSetControlsMap_.end() )
        {
            SelectSequencerSetAndCallMethod( GetSelectedSequencerSetNr(), sc_.sequencerSetSave );
            ComposeSetPanel( displayCount_, setCount, newSetCandidate, true );
            break;
        }
    }
    currentErrors_.clear();
    CheckIfActiveSetsAreReferencedAtLeastOnce();
    CheckIfReferencedSetsAreActive();
    Refresh();
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::OnBtnAutoAssignSetsToDisplays( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    unsigned int xCount = 1;
    unsigned int yCount = 1;
    const unsigned int setCount = static_cast<unsigned int>( sequencerSetControlsMap_.size() );
    while( ( xCount * yCount ) < setCount )
    {
        if( xCount <= yCount )
        {
            ++xCount;
        }
        else
        {
            ++yCount;
        }
    }
    pParent_->SetDisplayWindowCount( xCount , yCount );
    const DisplayWindowContainer::size_type displayCount = pParent_->GetDisplayCount();
    wxArrayString displayStrings;
    for( DisplayWindowContainer::size_type i = 0; i < displayCount; i++ )
    {
        displayStrings.Add( wxString::Format( wxT( "%s%d" ), s_displayStringPrefix_.c_str(), static_cast<int>( i ) ) );
    }

    SetNrToControlsMap::const_iterator it = sequencerSetControlsMap_.begin();
    const SetNrToControlsMap::const_iterator itEND = sequencerSetControlsMap_.end();
    while( it != itEND )
    {
        it->second->pDisplayToUseCB->Set( displayStrings );
        it->second->pDisplayToUseCB->Select( static_cast<size_t>( it->first % displayCount ) );
        ++it;
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::OnBtnRemoveSelectedSet( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    const int64_type setCount = sequencerSetControlsMap_.size();
    if( setCount == 0 )
    {
        wxMessageBox( wxT( "No Sets to Remove!" ), wxT( "Cannot Remove Set!" ), wxICON_EXCLAMATION );
        return;
    }
    RemoveSetPanel();
    currentErrors_.clear();
    CheckIfActiveSetsAreReferencedAtLeastOnce();
    CheckIfReferencedSetsAreActive();
    Refresh();
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::OnNBSequencerSetsPageChanged( wxNotebookEvent& e )
//-----------------------------------------------------------------------------
{
    try
    {
        if( e.GetOldSelection() >= 0 )
        {
            SelectSequencerSetAndCallMethod( GetSequencerSetNrFromPageIndex( static_cast<size_t>( e.GetOldSelection() ) ), sc_.sequencerSetSave );
        }
        SelectSequencerSetAndCallMethod( GetSequencerSetNrFromPageIndex( static_cast<size_t>( e.GetSelection() ) ), sc_.sequencerSetLoad );
        RefreshCurrentPropertyGrid();
    }
    catch( const ImpactAcquireException& e )
    {
        wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Internal error while switching sequencer sets: %s(%s))!" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        errorDlg.ShowModal();
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::OnNextSetSpinControlChanged( wxSpinEvent& )
//-----------------------------------------------------------------------------
{
    CheckIfActiveSetsAreReferencedAtLeastOnce();
    CheckIfReferencedSetsAreActive();
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::OnPropertyChanged( wxPropertyGridEvent& e )
//-----------------------------------------------------------------------------
{
    wxString errorString;
    Property prop( static_cast<HOBJ>( reinterpret_cast<intptr_t>( e.GetProperty()->GetClientData() ) ) );
    try
    {
        const string valueANSI( e.GetProperty()->GetValueAsString().mb_str() );
        prop.writeS( valueANSI );
    }
    catch( const ImpactAcquireException& e )
    {
        errorString = wxString::Format( wxT( "Can't set value( Error: %s(%s))!" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() );
    }

    if( !errorString.IsEmpty() )
    {
        wxMessageDialog errorDlg( NULL, errorString, wxT( "Error" ), wxOK | wxICON_INFORMATION );
        errorDlg.ShowModal();
    }
    RefreshCurrentPropertyGrid();
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::OnSettingsCheckBoxChecked( wxCommandEvent& e )
//-----------------------------------------------------------------------------
{
    // update the SequencerFeatureMap with the enabling/disabling of a feature
    ComponentLocator locator( sc_.sequencerSetSelector.parent().parent() );
    const wxString prop = static_cast< wxCheckBox* >( e.GetEventObject() )->GetLabel();
    {
        const HOBJ hObj = locator.findComponent( string( prop.mb_str() ) );
        const SequencerFeatureContainer::const_iterator itEND = sequencerFeatures_.end();
        for( SequencerFeatureContainer::iterator it = sequencerFeatures_.begin(); it != itEND; it++ )
        {
            if( it->first == hObj )
            {
                it->second = e.IsChecked();
                break;
            }
        }
    }

    // update the GUI (property grid)
    for( SetNrToControlsMap::iterator it = sequencerSetControlsMap_.begin() ; it != sequencerSetControlsMap_.end(); it++ )
    {
        UpdatePropertyGrid( it->second );
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::ReadPropertiesOfSequencerSet( SetNrToControlsMap::iterator& it )
//-----------------------------------------------------------------------------
{
    SelectSequencerSetAndCallMethod( it->first, sc_.sequencerSetLoad );
    const SequencerFeatureContainer::const_iterator compItEND = sequencerFeatures_.end();
    for( SequencerFeatureContainer::iterator compIt = sequencerFeatures_.begin(); compIt != compItEND; compIt++ )
    {
        const Component comp( compIt->first );
        wxPGProperty* pGridProperty = 0;
        if( ( compIt->second == true ) &&
            ( comp.type() & ctProp ) )
        {
            const Property prop( compIt->first );
            pGridProperty = it->second->pSetPropertyGrid->GetProperty( prop.name() );
            pGridProperty->SetValue( prop.readS() );
        }
    }
    it->second->pSequencerSetNextSC->SetValue( sc_.sequencerSetNext.read() );
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::RefreshCurrentPropertyGrid( void )
//-----------------------------------------------------------------------------
{
    SetNrToControlsMap::const_iterator itControls = GetSelectedSequencerSetControls();
    if( ( itControls == sequencerSetControlsMap_.end() ) ||
        ( itControls->second == 0 ) )
    {
        return;
    }

    wxPropertyGridIterator it = itControls->second->pSetPropertyGrid->GetIterator();
    while( !it.AtEnd() )
    {
        Property prop( static_cast<HOBJ>( reinterpret_cast<intptr_t>( ( *it )->GetClientData() ) ) );
        ( *it )->SetValueFromString( ConvertedString( prop.readS() ) );
        ++it;
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::RemoveError( SequencerErrorInfo& errorInfo )
//-----------------------------------------------------------------------------
{
    for( vector<SequencerErrorInfo>::iterator it = currentErrors_.begin(); it != currentErrors_.end(); it++ )
    {
        if( it->setNumber == errorInfo.setNumber &&
            it->errorType == errorInfo.errorType )
        {
            currentErrors_.erase( it );
            break;
        }
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::RemoveSetPanel( void )
//-----------------------------------------------------------------------------
{
    const int64_type currentPosition = pSetSettingsNotebook_->FindPage( pSetSettingsNotebook_->GetCurrentPage() );
    const int64_type setNumber = wxAtoi( pSetSettingsNotebook_->GetCurrentPage()->GetName().substr( s_sequencerSetStringPrefix_.length() ) );
    const SetNrToControlsMap::iterator it = sequencerSetControlsMap_.find( setNumber );
    Unbind( wxEVT_PG_CHANGED, &WizardSequencerControl::OnPropertyChanged, this, it->second->pSetPanel->GetId() + maxSequencerSetNumber_ );
    pSetSettingsNotebook_->DeletePage( currentPosition );
    sequencerSetControlsMap_.erase( it );
    if( pSetSettingsNotebook_->GetCurrentPage() != NULL )
    {
        const int64_type newSetNumber = wxAtoi( pSetSettingsNotebook_->GetCurrentPage()->GetName().substr( s_sequencerSetStringPrefix_.length() ) );
        HandleAutomaticNextSetSettingsOnRemove( currentPosition, newSetNumber );
    }
}

//-----------------------------------------------------------------------------
bool WizardSequencerControl::SaveSequencerSets( void )
//-----------------------------------------------------------------------------
{
    if( sequencerSetControlsMap_.empty() )
    {
        wxMessageBox( wxT( "At Least 1 Sequencer Set Has To Be Configured!" ), wxT( "Nothing To Save!" ), wxICON_EXCLAMATION );
        return false;
    }

    if( sc_.sequencerFeatureSelector.isValid() && sc_.sequencerFeatureEnable.isValid() && sc_.sequencerSetSave.isValid() )
    {
        const int64_type originalSetting = sc_.sequencerSetSelector.read();
        // first the enabled/disabled feature sets have to be updated according to the users choices (GUI global check-boxes)
        {
            vector<string> selectableFeaturesForSequencer;
            sc_.sequencerFeatureSelector.getTranslationDictStrings( selectableFeaturesForSequencer );
            const vector<string>::size_type selectableFeaturesForSequencerCount = selectableFeaturesForSequencer.size();
            ComponentLocator locator( sc_.sequencerSetSelector.parent().parent() );
            for( vector<string>::size_type i = 0; i < selectableFeaturesForSequencerCount; i++ )
            {
                sc_.sequencerFeatureSelector.writeS( selectableFeaturesForSequencer[i] );
                HOBJ hObj = locator.findComponent( selectableFeaturesForSequencer[i] );
                if( hObj == INVALID_ID )
                {
                    continue;
                }
                const SequencerFeatureContainer::const_iterator itEND = sequencerFeatures_.end();
                for( SequencerFeatureContainer::iterator it = sequencerFeatures_.begin(); it != itEND; it++ )
                {
                    if( it->first == hObj )
                    {
                        if( sc_.sequencerFeatureEnable.isWriteable() )
                        {
                            sc_.sequencerFeatureEnable.write( TBoolean( it->second ) );
                        }
                        break;
                    }
                }
            }
        }

        // all sets have to be saved
        {
            for( SetNrToControlsMap::iterator it = sequencerSetControlsMap_.begin(); it != sequencerSetControlsMap_.end(); it++ )
            {
                bool boUseCounter1EndRatherThanExposureEnd = false;
                SelectSequencerSetAndCallMethod( it->first, sc_.sequencerSetLoad );
                SequencerFeatureContainer::iterator compItEND = sequencerFeatures_.end();
                for( SequencerFeatureContainer::iterator compIt = sequencerFeatures_.begin(); compIt != compItEND; compIt++ )
                {
                    const Component comp( compIt->first );
                    if( ( compIt->second == true ) &&
                        ( comp.type() & ctProp ) &&
                        ( wxString( comp.name() ).IsSameAs( wxString( wxT( "CounterDuration" ) ) ) ) )
                    {
                        boUseCounter1EndRatherThanExposureEnd = true;
                        break;
                    }
                }
                // implicitly change Sequencer Trigger Source for all sets from ExposureEnd to Counter1End
                // when the CounterDuration Feature Check-box is enabled. Otherwise the Property has no meaning.
                // The reverse must, of course, be done when the CounterDuration Feature Check-box is unchecked.
                if( ( boCounterDurationCapability_ == true ) &&
                    ( sc_.sequencerTriggerSource.isWriteable() ) )
                {
                    sc_.sequencerTriggerSource.writeS( ( boUseCounter1EndRatherThanExposureEnd == true ? "Counter1End" : "ExposureEnd" ) );
                }
                sc_.sequencerSetNext.write( it->second->pSequencerSetNextSC->GetValue() );
                SelectSequencerSetAndCallMethod( it->first, sc_.sequencerSetSave );
            }
        }
        sc_.sequencerSetStart.write( sequencerSetControlsMap_.begin()->first );
        sc_.sequencerSetSelector.write( originalSetting );
    }
    return true;
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::SaveSequencerSetsAndEnd( int returnCode )
//-----------------------------------------------------------------------------
{
    SelectSequencerSetAndCallMethod( GetSelectedSequencerSetNr(), sc_.sequencerSetSave );
    if( SaveSequencerSets() )
    {
        EndModal( returnCode );
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::SelectSequencerSetAndCallMethod( int64_type setNr, Method& meth )
//-----------------------------------------------------------------------------
{
    try
    {
        sc_.sequencerSetSelector.write( setNr );
        meth.call();
    }
    catch( const ImpactAcquireException& e )
    {
        wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Internal error while switching sequencer sets: %s(%s))!" ), ConvertedString( e.getErrorString() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
        errorDlg.ShowModal();
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::UpdateError( const wxString& msg, const wxBitmap& icon ) const
//-----------------------------------------------------------------------------
{
    pStaticBitmapWarning_->SetBitmap( icon );
    pInfoText_->SetLabel( msg );
    pInfoText_->SetToolTip( msg );
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::UpdateErrors( void ) const
//-----------------------------------------------------------------------------
{
    if( currentErrors_.size() > 0 )
    {
        const SequencerErrorInfo lastErrorInfo = currentErrors_.back();
        const TSequencerWizardErrorTypes lastErrorType = lastErrorInfo.errorType;
        const int setNumber = static_cast<int>( lastErrorInfo.setNumber );
        const int setNext = static_cast<int>( lastErrorInfo.nextSet );
        if( lastErrorType == swetReferenceToInactiveSet )
        {
            UpdateError( wxString::Format( wxT( "Set %d: 'Next Set' Points To An Invalid Set (Set %d)!" ), setNumber, setNext ), wxBitmap( wizard_warning_xpm ) );
        }
        else if( lastErrorType == swetSetsNotBeingReferenced )
        {
            UpdateError( wxString::Format( wxT( "Set %d: No Other Set Has Set %d As 'Next Set'!" ), setNumber, setNumber ), wxBitmap( wizard_warning_xpm ) );
        }
    }
    else
    {
        UpdateError( wxString::Format( wxT( "" ) ), wxBitmap( wizard_empty_xpm ) );
    }
}

//-----------------------------------------------------------------------------
void WizardSequencerControl::UpdatePropertyGrid( SequencerSetControls* pSelectedSequencerSet )
//-----------------------------------------------------------------------------
{
    SequencerFeatureContainer::iterator itEND = sequencerFeatures_.end();
    for( SequencerFeatureContainer::iterator it = sequencerFeatures_.begin(); it != itEND; it++ )
    {
        const Property prop( it->first );
        wxPGProperty* pGridProperty = pSelectedSequencerSet->pSetPropertyGrid->GetProperty( prop.name() );
        if( pGridProperty == 0 )
        {
            Component comp( it->first );
            const TComponentType type = comp.type();
            wxPropertyGrid* pGrid = pSelectedSequencerSet->pSetPropertyGrid;
            const wxString elementName( comp.name() );
            switch( PropertyObject::GetEditorType( comp, elementName ) )
            {
            case PropData::_ctrlBoolean:
                pGridProperty = pGrid->Append( new wxBoolProperty( elementName, wxPG_LABEL ) );
                pGrid->SetPropertyAttribute( pGridProperty, wxPG_BOOL_USE_CHECKBOX, wxVariant( 1 ) );
                break;
            case PropData::_ctrlSpinner:
                if( ( type == ctPropInt ) || ( type == ctPropInt64 ) || ( type == ctPropFloat ) )
                {
                    pGridProperty = pGrid->Append( new wxStringProperty( elementName, wxPG_LABEL ) );
                }
                else
                {
                    wxASSERT( !"invalid component type for spinner control" );
                }
                pGridProperty->SetEditor( wxPGCustomSpinCtrlEditor_HOBJ::Instance()->GetEditor() );
                break;
            case PropData::_ctrlEdit:
                pGridProperty = pGrid->Append( new wxLongStringProperty( elementName, wxPG_LABEL ) );
                break;
            case PropData::_ctrlCombo:
                {
                    wxPGChoices soc;
                    PropertyObject::GetTransformedDict( comp, soc );
                    pGridProperty = pGrid->Append( new wxEnumProperty( elementName, wxPG_LABEL, soc ) );
                }
                break;
            case PropData::_ctrlMultiChoiceSelector:
                {
                    wxPGChoices soc;
                    PropertyObject::GetTransformedDict( comp, soc );
                    wxPGProperty* p = new wxMultiChoiceProperty( elementName, wxPG_LABEL, soc );
                    pGridProperty = pGrid->Append( p );
                }
                break;
            case PropData::_ctrlFileSelector:
                pGridProperty = pGrid->Append( new wxFileProperty( elementName, wxPG_LABEL ) );
                break;
            case PropData::_ctrlDirSelector:
                pGridProperty = pGrid->Append( new wxDirProperty( elementName, wxPG_LABEL, ::wxGetUserHome() ) );
                break;
            case PropData::_ctrlBinaryDataEditor:
                pGridProperty = pGrid->Append( new wxBinaryDataProperty( elementName, wxPG_LABEL ) );
                pGrid->SetPropertyValidator( pGridProperty, *wxBinaryDataProperty::GetClassValidator() );
                break;
            default:
                break;
            }
            pGridProperty->SetClientData( reinterpret_cast<void*>( prop.hObj() ) );
            pGridProperty->SetValue( prop.readS() );
        }
        pGridProperty->Hide( !it->second );
    }
}
