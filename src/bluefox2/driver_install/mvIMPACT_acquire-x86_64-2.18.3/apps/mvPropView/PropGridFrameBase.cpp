#include <algorithm>
#include <common/STLHelper.h>
#include <functional>
#include <limits>
#include "PropData.h"
#include "PropGridFrameBase.h"
#include "ValuesFromUserDlg.h"
#include <wx/spinctrl.h>
#include <wx/clipbrd.h>

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
/// \brief Used for internal refresh calls...
template<typename _Ty>
void DummyRead( Component comp )
//-----------------------------------------------------------------------------
{
    _Ty prop( comp );
    vector<typename _Ty::value_type> v;
    prop.read( v, 0 );
}

//=============================================================================
//================= Implementation PropGridFrameBase ==========================
//=============================================================================
BEGIN_EVENT_TABLE( PropGridFrameBase, wxFrame )
    EVT_TIMER( wxID_ANY, PropGridFrameBase::OnTimer )
    EVT_MENU( miPopUpPropForceRefresh, PropGridFrameBase::OnPopUpPropForceRefresh )
    EVT_MENU( miPopUpPropCopyToClipboard, PropGridFrameBase::OnPopUpPropCopyToClipboard )
    EVT_MENU( miPopUpPropSetFromClipboard, PropGridFrameBase::OnPopUpPropSetFromClipboard )
    EVT_MENU( miPopUpPropRestoreDefault, PropGridFrameBase::OnPopUpPropRestoreDefault )
    EVT_MENU( miPopUpPropReadFromFile, PropGridFrameBase::OnPopUpPropReadFromFile )
    EVT_MENU( miPopUpPropWriteToFile, PropGridFrameBase::OnPopUpPropWriteToFile )
    EVT_MENU( miPopUpPropAttachCallback, PropGridFrameBase::OnPopUpPropAttachCallback )
    EVT_MENU( miPopUpPropDetachCallback, PropGridFrameBase::OnPopUpPropDetachCallback )
    EVT_MENU( miPopUpPropPlotInFeatureValueVsTimePlot, PropGridFrameBase::OnPopUpPropPlotInFeatureValueVsTimePlot )
    EVT_MENU( miPopUpPropAppendValue, PropGridFrameBase::OnPopUpPropAppendValue )
    EVT_MENU( miPopUpPropDeleteValue, PropGridFrameBase::OnPopUpPropDeleteValue )
    EVT_MENU( miPopUpPropSetMultiple_FixedValue, PropGridFrameBase::OnPopUpPropSetMultiple_FixedValue )
    EVT_MENU( miPopUpPropSetMultiple_FromToRange, PropGridFrameBase::OnPopUpPropSetMultiple_FromToRange )
    EVT_MENU( miPopUpMethExec, PropGridFrameBase::OnExecutePropGridMethod )
    EVT_MENU( miPopUpDetailedFeatureInfo, PropGridFrameBase::OnPopUpDetailedFeatureInfo )
    EVT_PG_CHANGED( widPropertyGridManager, PropGridFrameBase::OnPropertyChanged )
    EVT_PG_RIGHT_CLICK( widPropertyGridManager, PropGridFrameBase::OnPropertyRightClicked )
    EVT_PG_SELECTED( widPropertyGridManager, PropGridFrameBase::OnPropertySelected )
    EVT_PG_ITEM_COLLAPSED( widPropertyGridManager, PropGridFrameBase::OnPropertyGridItemExpandedOrCollapsed )
    EVT_PG_ITEM_EXPANDED( widPropertyGridManager, PropGridFrameBase::OnPropertyGridItemExpandedOrCollapsed )
    EVT_BUTTON( widPropertyGridManager, PropGridFrameBase::OnExecutePropGridMethod )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
PropGridFrameBase::PropGridFrameBase( wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size )
    : wxFrame( 0, id, title, pos, size ), m_PropertyGridUpdateTimer( this, teListUpdate ),
      m_pPropGridManager( 0 ), m_stopWatch()
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
PropGridFrameBase::~PropGridFrameBase()
//-----------------------------------------------------------------------------
{
    StopPropertyGridUpdateTimer();
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::ConfigureToolTipsForPropertyGrids( const bool boEnable )
//-----------------------------------------------------------------------------
{
    if( m_pPropGridManager != 0 )
    {
        const long style = m_pPropGridManager->GetWindowStyle();
        const long extraStyle = m_pPropGridManager->GetExtraStyle();
        if( boEnable )
        {
            m_pPropGridManager->SetWindowStyle( style | wxPG_TOOLTIPS );
            m_pPropGridManager->SetExtraStyle( extraStyle | wxPG_EX_HELP_AS_TOOLTIPS );
        }
        else
        {
            m_pPropGridManager->SetWindowStyle( style & ~wxPG_TOOLTIPS );
            m_pPropGridManager->SetExtraStyle( extraStyle & ~wxPG_EX_HELP_AS_TOOLTIPS );
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::ExpandPropertyRecursively( wxPGProperty* id )
//-----------------------------------------------------------------------------
{
    if( id && GetPropertyGrid() )
    {
        if( id->GetParent() )
        {
            ExpandPropertyRecursively( id->GetParent() );
        }
        GetPropertyGrid()->Expand( id );
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::ExpandPropertyTreeToDeviceSettings( void )
//-----------------------------------------------------------------------------
{
    wxPropertyGrid* pPropGrid( GetPropertyGrid() );
    if( pPropGrid )
    {
        wxPGProperty* pProp = pPropGrid->GetProperty( wxT( "Setting.Setting/Base.Setting/Base/Camera.Setting/Base/Camera/GenICam" ) );
        if( !pProp )
        {
            pProp = pPropGrid->GetProperty( wxT( "Setting.Setting/Base.Setting/Base/Camera" ) );
        }
        if( pProp )
        {
            SelectPropertyInPropertyGrid( static_cast<PropData*>( pProp->GetClientData() ) );
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnExecutePropGridMethod( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        // make sure we are actually dealing with a method object
        MethodObject* pMethod = dynamic_cast<MethodObject*>( pData );
        if( pMethod )
        {
            int callResult = DMR_NO_ERROR;
            WriteLogMessage( pMethod->Call( callResult ) );
            WriteLogMessage( wxT( '\n' ) );
            if( ( callResult != DMR_NO_ERROR ) && ShowPropGridMethodExecutionErrors() )
            {
                wxString errorString( wxString::Format( wxT( "An error occurred while executing function %s. Error code: %d (%s))." ), pMethod->FriendlyName().c_str(), callResult, ImpactAcquireException::getErrorCodeAsString( callResult ).c_str() ) );
                AppendCustomPropGridExecutionErrorMessage( errorString );
                wxMessageDialog errorDlg( NULL, errorString, wxT( "Method Execution Failed" ), wxOK | wxICON_INFORMATION );
                errorDlg.ShowModal();
            }
        }
        // this branch is also reached by custom editors, that are not methods(e.g. dirSelector Dialog, thus in order to avoid confusion
        //
        //else
        //{
        //  WriteErrorMessage( wxString::Format( wxT("There was an 'execute' message for an object(%s) that is not a method.\n"), pProp->GetLabel().c_str() ) );
        //}
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpDetailedFeatureInfo( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            DetailedFeatureInfoDlg dlg( this, pData->GetComponent() );
            dlg.ShowModal();
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropAppendValue( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            VectorPropertyObject* pProp = dynamic_cast<VectorPropertyObject*>( pData );
            if( pProp )
            {
                Property prop( pProp->GetComponent().hObj() );
                prop.resizeValArray( prop.valCount() + 1 );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropDeleteValue( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            VectorPropertyObject* pProp = dynamic_cast<VectorPropertyObject*>( pData );
            if( pProp )
            {
                Property prop( pProp->GetComponent().hObj() );
                pProp->RemoveValue( prop.valCount() - 1 );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropForceRefresh( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            Component comp( pData->GetComponent() );
            switch( comp.type() )
            {
            case ctPropFloat:
                DummyRead<PropertyF>( comp );
                break;
            case ctPropInt:
                DummyRead<PropertyI>( comp );
                break;
            case ctPropInt64:
                DummyRead<PropertyI64>( comp );
                break;
            case ctPropPtr:
                DummyRead<PropertyPtr>( comp );
                break;
            case ctPropString:
                DummyRead<PropertyS>( comp );
                break;
            default:
                WriteErrorMessage( wxString::Format( wxT( "Unhandled data type in function %s detected. Component %s is of type %s.\n" ),
                                                     ConvertedString( __FUNCTION__ ).c_str(),
                                                     pData->GetDisplayName( GetDisplayFlags() ).c_str(),
                                                     ConvertedString( comp.typeAsString() ).c_str() ) );
                break;
            }
            if( comp.isDefault() )
            {
                pProp->SetModifiedStatus( false );
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropCopyToClipboard( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            Property prop( pData->GetComponent() );
            try
            {
                if( wxTheClipboard->Open() )
                {
                    wxTheClipboard->SetData( new wxTextDataObject( ConvertedString( prop.readSArray( "", "\n" ) ) ) );
                    wxTheClipboard->Flush();
                    wxTheClipboard->Close();
                }
            }
            catch( const ImpactAcquireException& e )
            {
                wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Failed to copy data from feature '%s' into the clipboard(Error: %s)" ), ConvertedString( prop.name() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                errorDlg.ShowModal();
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropSetFromClipboard( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            Property prop( pData->GetComponent() );
            if( wxTheClipboard->Open() )
            {
                wxTextDataObject text;
                if( wxTheClipboard->GetData( text ) )
                {
                    try
                    {
                        Property prop( pData->GetComponent() );
                        wxString theText( text.GetText() );
                        vector<wxString> vwx;
                        vector<string> v;
                        vector<wxString>::size_type tokenCount = Split( text.GetText(), wxT( " " ), vwx );
                        if( vwx.empty() )
                        {
                            tokenCount = Split( text.GetText(), wxT( "\n" ), vwx );
                        }
                        for( vector<wxString>::size_type i = 0; i < tokenCount; i++ )
                        {
                            v.push_back( string( vwx[i].mb_str() ) );
                        }
                        PropertyObject* pPropertyObject = dynamic_cast<PropertyObject*>( pData );
                        if( pPropertyObject )
                        {
                            prop.writeS( v, ( pPropertyObject->GetIndex() >= 0 ) ? pPropertyObject->GetIndex() : 0 );
                        }
                        else
                        {
                            prop.writeS( v, 0 );
                        }
                    }
                    catch( const ImpactAcquireException& e )
                    {
                        wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Failed to copy data from the clipboard to feature '%s'(Error: %s)" ), ConvertedString( prop.name() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                        errorDlg.ShowModal();
                    }
                }
                else
                {
                    wxMessageDialog errorDlg( NULL, wxT( "Failed to get text data from clipboard" ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                    errorDlg.ShowModal();
                }
                wxTheClipboard->Close();
            }
        }
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropRestoreDefault( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            Component comp( pData->GetComponent() );
            if( comp.type() == ctList )
            {
                wxMessageDialog AreYouSureDlg( NULL,
                                               wxString::Format( wxT( "All properties in the list '%s' will be set to default.\n" ),
                                                       pData->GetDisplayName( GetDisplayFlags() ).c_str() ),
                                               wxT( "Warning" ),
                                               wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION );

                if( AreYouSureDlg.ShowModal() != wxID_YES )
                {
                    return;
                }
            }
            if( comp.isDefault() )
            {
                WriteLogMessage( wxString::Format( wxT( "Element '%s' is already set to the default value.\n" ),
                                                   pData->GetDisplayName( GetDisplayFlags() ).c_str() ) );
            }
            else
            {
                try
                {
                    comp.restoreDefault();
                }
                catch( const ImpactAcquireException& e )
                {
                    wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Failed to restore the default for feature '%s'(Error: %s)" ), ConvertedString( comp.name() ).c_str(), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                    errorDlg.ShowModal();
                }
            }
        }
        pProp->SetModifiedStatus( false );
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropSetMultiple_FixedValue( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            VectorPropertyObject* pProp = dynamic_cast<VectorPropertyObject*>( pData );
            if( pProp && ( pProp->GetComponent().type() == ctPropInt ) )
            {
                vector<ValueData*> v;
                try
                {
                    PropertyI prop( pProp->GetComponent() );
                    const unsigned int valCnt = prop.valCount();
                    v.push_back( new ValueRangeData( wxString( wxT( "First value to set" ) ), 0, valCnt - 1, 1, 0 ) );
                    v.push_back( new ValueRangeData( wxString( wxT( "Last value to set" ) ), 0, valCnt - 1, 1, valCnt - 1 ) );
                    if( prop.hasDict() )
                    {
                        wxArrayString stringArray;
                        vector<pair<string, int> > translationDict;
                        prop.getTranslationDict( translationDict );
                        vector<pair<string, int> >::size_type vSize = translationDict.size();
                        for( vector<pair<string, int> >::size_type i = 0; i < vSize; i++ )
                        {
                            stringArray.push_back( ConvertedString( translationDict[i].first ) );
                        }
                        v.push_back( new ValueChoiceData( wxString( wxT( "Value" ) ), stringArray ) );
                    }
                    else
                    {
                        v.push_back( new ValueRangeData( wxString( wxT( "Value" ) ),
                                                         prop.hasMinValue() ? prop.getMinValue() : numeric_limits<int>::min(),
                                                         prop.hasMaxValue() ? prop.getMaxValue() : numeric_limits<int>::max(),
                                                         prop.hasStepWidth() ? prop.getStepWidth() : 1,
                                                         prop.hasMinValue() ? prop.getMinValue() : numeric_limits<int>::min() ) );
                    }
                    ValuesFromUserDlg dlg( this, wxString( wxT( "Select the value and range to apply" ) ), v );
                    if( dlg.ShowModal() == wxID_OK )
                    {
                        const vector<wxControl*>& resultData = dlg.GetUserInputControls();
                        wxASSERT( dynamic_cast<wxSpinCtrl*>( resultData[0] ) && dynamic_cast<wxSpinCtrl*>( resultData[1] ) );
                        int first = dynamic_cast<wxSpinCtrl*>( resultData[0] )->GetValue();
                        int last = dynamic_cast<wxSpinCtrl*>( resultData[1] )->GetValue();
                        if( first <= last )
                        {
                            try
                            {
                                if( prop.hasDict() )
                                {
                                    vector<string> sequence( last - first + 1, string( dynamic_cast<wxComboBox*>( resultData[2] )->GetValue().mb_str() ) );
                                    prop.writeS( sequence, first );
                                }
                                else
                                {
                                    vector<int> sequence( last - first + 1, dynamic_cast<wxSpinCtrl*>( resultData[2] )->GetValue() );
                                    prop.write( sequence, true, first );
                                }
                            }
                            catch( const ImpactAcquireException& e )
                            {
                                wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Couldn't set value range(Error: %s)" ), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                                errorDlg.ShowModal();
                            }
                        }
                        else
                        {
                            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Invalid input data!\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__ ) );
                        }
                    }
                }
                catch( const ImpactAcquireException& e )
                {
                    wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Internal problem(Error: %s)" ), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                    errorDlg.ShowModal();
                }
                for_each( v.begin(), v.end(), ptr_fun( DeleteElement<ValueData*> ) );
            }
            else
            {
                WriteErrorMessage( wxString::Format( wxT( "%s(%d): Element '%s' doesn't seem to be a vector property!\n" ),
                                                     ConvertedString( __FUNCTION__ ).c_str(), __LINE__,
                                                     pData->GetDisplayName( GetDisplayFlags() ).c_str() ) );
            }
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Invalid client data detected!\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__ ) );
        }
    }
    else
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): No item selected!\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__ ) );
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPopUpPropSetMultiple_FromToRange( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
    if( pProp )
    {
        PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
        if( pData )
        {
            VectorPropertyObject* pProp = dynamic_cast<VectorPropertyObject*>( pData );
            if( pProp && ( pProp->GetComponent().type() == ctPropInt ) )
            {
                vector<ValueData*> v;
                try
                {
                    PropertyI prop( pProp->GetComponent() );
                    const unsigned int valCnt = prop.valCount();
                    v.push_back( new ValueRangeData( wxString( wxT( "First value to set" ) ), 0, valCnt - 1, 1, 0 ) );
                    v.push_back( new ValueRangeData( wxString( wxT( "Last value to set" ) ), 0, valCnt - 1, 1, valCnt - 1 ) );
                    wxASSERT( !prop.hasDict() );
                    v.push_back( new ValueRangeData( wxString( wxT( "Start value" ) ),
                                                     prop.hasMinValue() ? prop.getMinValue() : numeric_limits<int>::min(),
                                                     prop.hasMaxValue() ? prop.getMaxValue() : numeric_limits<int>::max(),
                                                     prop.hasStepWidth() ? prop.getStepWidth() : 1,
                                                     prop.hasMinValue() ? prop.getMinValue() : numeric_limits<int>::min() ) );
                    v.push_back( new ValueRangeData( wxString( wxT( "End value" ) ),
                                                     prop.hasMinValue() ? prop.getMinValue() : numeric_limits<int>::min(),
                                                     prop.hasMaxValue() ? prop.getMaxValue() : numeric_limits<int>::max(),
                                                     prop.hasStepWidth() ? prop.getStepWidth() : 1,
                                                     prop.hasMinValue() ? prop.getMaxValue() : numeric_limits<int>::max() ) );
                    ValuesFromUserDlg dlg( this, wxString( wxT( "Select the value and range to apply" ) ), v );
                    if( dlg.ShowModal() == wxID_OK )
                    {
                        const vector<wxControl*>& resultData = dlg.GetUserInputControls();
                        wxASSERT( dynamic_cast<wxSpinCtrl*>( resultData[0] ) && dynamic_cast<wxSpinCtrl*>( resultData[1] ) &&
                                  dynamic_cast<wxSpinCtrl*>( resultData[2] ) && dynamic_cast<wxSpinCtrl*>( resultData[3] ) );
                        int first = dynamic_cast<wxSpinCtrl*>( resultData[0] )->GetValue();
                        int last = dynamic_cast<wxSpinCtrl*>( resultData[1] )->GetValue();
                        int startVal = dynamic_cast<wxSpinCtrl*>( resultData[2] )->GetValue();
                        int endVal = dynamic_cast<wxSpinCtrl*>( resultData[3] )->GetValue();
                        if( first <= last )
                        {
                            try
                            {
                                vector<int> sequence;
                                int valuesToProcess = last - first + 1;
                                double increment = static_cast<double>( endVal - startVal ) / static_cast<double>( valuesToProcess - 1 );
                                for( int i = 0; i < valuesToProcess; i++ )
                                {
                                    int offset = static_cast<int>( i * increment );
                                    if( i * increment - offset > 0.5 )
                                    {
                                        ++offset;
                                    }
                                    sequence.push_back( startVal + offset );
                                }
                                prop.write( sequence, true, first );
                            }
                            catch( const ImpactAcquireException& e )
                            {
                                wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Couldn't set value range(Error: %s)" ), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                                errorDlg.ShowModal();
                            }
                        }
                        else
                        {
                            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Invalid input data!\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__ ) );
                        }
                    }
                }
                catch( const ImpactAcquireException& e )
                {
                    wxMessageDialog errorDlg( NULL, wxString::Format( wxT( "Internal problem(Error: %s)" ), ConvertedString( e.getErrorCodeAsString() ).c_str() ), wxT( "Error" ), wxOK | wxICON_INFORMATION );
                    errorDlg.ShowModal();
                }
                for_each( v.begin(), v.end(), ptr_fun( DeleteElement<ValueData*> ) );
            }
            else
            {
                WriteErrorMessage( wxString::Format( wxT( "%s(%d): Element '%s' doesn't seem to be a vector property!\n" ),
                                                     ConvertedString( __FUNCTION__ ).c_str(),
                                                     __LINE__,
                                                     pData->GetDisplayName( GetDisplayFlags() ).c_str() ) );
            }
        }
        else
        {
            WriteErrorMessage( wxString::Format( wxT( "%s(%d): Invalid client data detected!\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__ ) );
        }
    }
    else
    {
        WriteErrorMessage( wxString::Format( wxT( "%s(%d): No item selected!\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__ ) );
    }
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPropertyChanged( wxPropertyGridEvent& e )
//-----------------------------------------------------------------------------
{
    PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( e.GetProperty() ) );
    if( pData )
    {
        m_stopWatch.Start();
        pData->UpdatePropData();
        if( ShowFeatureChangeTimeConsumption() )
        {
            WriteLogMessage( wxString::Format( wxT( "Changing feature '%s' took %ld ms.\n" ), pData->GetDisplayName( GetDisplayFlags() ).c_str(), m_stopWatch.Time() ) );
        }
    }
    OnPropertyChangedCustom( e );
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnPropertyRightClicked( wxPropertyGridEvent& e )
//-----------------------------------------------------------------------------
{
    if( !e.GetProperty() )
    {
        return;
    }

    PropData* pPropData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( e.GetProperty() ) );
    if( pPropData )
    {
        bool boIsProp = pPropData->GetComponent().isProp();
        if( boIsProp || pPropData->GetComponent().isList() )
        {
            wxMenu menu( wxT( "" ) );
            menu.Append( miPopUpPropForceRefresh, wxT( "&Force Refresh" ) )->Enable( boIsProp );
            menu.Append( miPopUpPropCopyToClipboard, wxT( "&Copy To Clipboard" ) )->Enable( boIsProp );
            menu.Append( miPopUpPropSetFromClipboard, wxT( "&Set From Clipboard" ) )->Enable( boIsProp );
            const bool boWriteable = pPropData->GetComponent().isWriteable();
            menu.Append( miPopUpPropRestoreDefault, wxT( "&Restore Default" ) )->Enable( boWriteable );
            menu.Append( miPopUpDetailedFeatureInfo, wxT( "Detailed Feature Information" ) );
            menu.AppendSeparator();
            bool boAnotherSeparatorNeeded = false;
            if( FeatureChangedCallbacksSupported() )
            {
                const bool boFeatureHasCallbackRegistered = FeatureHasChangedCallback( pPropData->GetComponent() );
                menu.Append( miPopUpPropAttachCallback, wxT( "Attach Callback" ) )->Enable( !boFeatureHasCallbackRegistered );
                menu.Append( miPopUpPropDetachCallback, wxT( "Detach Callback" ) )->Enable( boFeatureHasCallbackRegistered );
                boAnotherSeparatorNeeded = true;
            }
            const TComponentType type = pPropData->GetComponent().type();
            switch( type )
            {
            case ctPropFloat:
            case ctPropInt:
            case ctPropInt64:
                menu.Append( miPopUpPropPlotInFeatureValueVsTimePlot, wxT( "Plot In Feature vs. Time Plot" ) )->Enable( boIsProp );
                boAnotherSeparatorNeeded = true;
                break;
            default:
                // we don't support other component types for this feature
                break;
            }
            if( boAnotherSeparatorNeeded )
            {
                menu.AppendSeparator();
            }
            // The 'type' check is only needed because some drivers with versions < 1.12.33
            // did incorrectly specify the 'cfContainsBinaryData' flag even though the data type was not 'ctPropString'...
            bool boContainsBinaryData = ( ( type == ctPropString ) && ( pPropData->GetComponent().flags() & cfContainsBinaryData ) ) != 0;
            menu.Append( miPopUpPropReadFromFile, wxT( "Read File Into Property Value" ) )->Enable( boContainsBinaryData );
            menu.Append( miPopUpPropWriteToFile, wxT( "Write Property Value To File" ) )->Enable( boContainsBinaryData );
            bool boAppendValuePossible = false;
            bool boDeleteLastValuePossible = false;
            bool boMultiToConstPossible = false;
            bool boMultiToRangePossible = false;
            wxPGProperty* pProp = GetPropertyGrid()->GetSelectedProperty();
            if( pProp )
            {
                PropData* pData = static_cast<PropData*>( GetPropertyGrid()->GetPropertyClientData( pProp ) );
                if( pData )
                {
                    VectorPropertyObject* pProp = dynamic_cast<VectorPropertyObject*>( pData );
                    if( pProp )
                    {
                        Property driverProp( pProp->GetComponent() );
                        if( !( pPropData->GetComponent().flags() & cfFixedSize ) )
                        {
                            const unsigned int valCnt = driverProp.valCount();
                            const unsigned int maxValCnt = driverProp.maxValCount();
                            boAppendValuePossible = ( ( maxValCnt > valCnt ) && boWriteable ) ? true : false;
                            boDeleteLastValuePossible = ( ( valCnt > 1 ) && boWriteable ) ? true : false;
                        }
                        if( driverProp.type() == ctPropInt )
                        {
                            boMultiToConstPossible = boWriteable;
                            if( !driverProp.hasDict() )
                            {
                                boMultiToRangePossible = boWriteable;
                            }
                        }
                    }
                }
            }
            menu.Append( miPopUpPropAppendValue, wxT( "&Append Value" ) )->Enable( boAppendValuePossible );
            menu.Append( miPopUpPropDeleteValue, wxT( "&Delete Last Value" ) )->Enable( boDeleteLastValuePossible );
            wxMenu* pSubMenu = new wxMenu( wxT( "" ) );
            pSubMenu->Append( miPopUpPropSetMultiple_FixedValue, wxT( "&To A Constant Value" ) )->Enable( boMultiToConstPossible );
            pSubMenu->Append( miPopUpPropSetMultiple_FromToRange, wxT( "Via A User Defined &Value Range" ) )->Enable( boMultiToRangePossible );
            menu.Append( wxID_ANY, wxT( "Set Multiple Elements" ), pSubMenu );
            PopupMenu( &menu );
        }
        /// \todo disable read-only methods later?
        else if( ( pPropData->GetComponent().isMeth() ) /* && pPropData->GetComponent().isWriteable()*/ )
        {
            wxMenu menu( wxT( "" ) );
            menu.Append( miPopUpMethExec, wxT( "&Execute" ) );
            menu.Append( miPopUpDetailedFeatureInfo, wxT( "Detailed Feature Information" ) )->Enable( true );
            PopupMenu( &menu );
        }
    }
    OnPropertyRightClickedCustom( e );
}

//-----------------------------------------------------------------------------
void PropGridFrameBase::OnTimer( wxTimerEvent& e )
//-----------------------------------------------------------------------------
{
    try
    {
        switch( e.GetId() )
        {
        case teListUpdate:
            OnPropertyGridTimer();
            break;
        default:
            OnTimerCustom( e );
            break;
        }
    }
    catch( const ImpactAcquireException& theException )
    {
        WriteLogMessage( wxString::Format( wxT( "%s: An exception was generated while updating the state of the property grid: %s(%s)\n" ), ConvertedString( __FUNCTION__ ).c_str(), ConvertedString( theException.getErrorString() ).c_str(), ConvertedString( theException.getErrorCodeAsString() ).c_str() ) );
    }
}

//-----------------------------------------------------------------------------
bool PropGridFrameBase::SelectPropertyInPropertyGrid( PropData* pPropData )
//-----------------------------------------------------------------------------
{
    if( pPropData && m_pPropGridManager )
    {
        wxPGProperty* pProp = pPropData->GetGridItem();
        if( pProp )
        {
            wxPropertyGridPage* pGridPage = pPropData->GetParentGridPage();
            if( pGridPage && GetSelectedPropertyGridPage() )
            {
                if( pGridPage != GetSelectedPropertyGridPage() )
                {
                    m_pPropGridManager->SelectPage( pGridPage );
                }
                if( pProp->GetParent() )
                {
                    ExpandPropertyRecursively( pProp->GetParent() );
                }
                pGridPage->Expand( pProp );
                return pGridPage->GetGrid()->SelectProperty( pProp, true );
            }
        }
    }
    return false;
}
