//-----------------------------------------------------------------------------
#ifndef ValuesFromUserDlgH
#define ValuesFromUserDlgH ValuesFromUserDlgH
//-----------------------------------------------------------------------------
#include "DataConversion.h"
#include "DevData.h"
#include <map>
#include <vector>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <wx/wx.h>
#include <wx/treectrl.h>
#include <wx/filename.h>
#include <wx/spinctrl.h>

class wxSpinCtrl;
class ImageCanvas;
class PropGridFrameBase;

typedef std::vector<ImageCanvas*> DisplayWindowContainer;
typedef std::map<wxString, wxString> StringToStringMap;

std::string BinaryDataFromString( const std::string& value );
std::string BinaryDataToString( const std::string& value );

//-----------------------------------------------------------------------------
template<class _Tx, typename _Ty>
wxString ReadStringDict( _Tx prop, wxArrayString& choices )
//-----------------------------------------------------------------------------
{
    wxString currentValue;
    if( prop.isValid() )
    {
        currentValue = ConvertedString( prop.readS() );
        std::vector<std::pair<std::string, _Ty> > dict;
        prop.getTranslationDict( dict );
        const typename std::vector<std::pair<std::string, _Ty> >::size_type cnt = dict.size();
        for( typename std::vector<std::pair<std::string, _Ty> >::size_type i = 0; i < cnt; i++ )
        {
            choices.Add( ConvertedString( dict[i].first ) );
        }
    }
    return currentValue;
}

std::vector<wxString>::size_type Split( const wxString& str, const wxString& separator, std::vector<wxString>& v );
void WriteFile( const void* pBuf, size_t bufSize, const wxString& pathName, wxTextCtrl* pTextCtrl );
void WriteToTextCtrl( wxTextCtrl* pTextCtrl, const wxString& msg, const wxTextAttr& style = wxTextAttr( *wxBLACK ) );

//-----------------------------------------------------------------------------
class HEXStringValidator : public wxTextValidator
//-----------------------------------------------------------------------------
{
public:
    HEXStringValidator( wxString* valPtr = NULL );
};

//-----------------------------------------------------------------------------
class OkAndCancelDlg : public wxDialog
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
protected:
    wxButton*                   pBtnApply_;
    wxButton*                   pBtnCancel_;
    wxButton*                   pBtnOk_;

    virtual void OnBtnApply( wxCommandEvent& ) {}
    virtual void OnBtnCancel( wxCommandEvent& )
    {
        EndModal( wxID_CANCEL );
    }
    virtual void OnBtnOk( wxCommandEvent& )
    {
        EndModal( wxID_OK );
    }
    void AddButtons( wxWindow* pWindow, wxSizer* pSizer, bool boCreateApplyButton = false );
    void FinalizeDlgCreation( wxWindow* pWindow, wxSizer* pSizer );
    //-----------------------------------------------------------------------------
    enum TWidgetIDs
    //-----------------------------------------------------------------------------
    {
        widBtnOk = 1,
        widBtnCancel = 2,
        widBtnApply = 3,
        widFirst = 100
    };
public:
    explicit OkAndCancelDlg( wxWindow* pParent, wxWindowID id, const wxString& title, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, long style = wxDEFAULT_DIALOG_STYLE, const wxString& name = wxT( "OkAndCancelDlg" ) );
};

//-----------------------------------------------------------------------------
struct ValueData
//-----------------------------------------------------------------------------
{
    wxString caption_;
    explicit ValueData( const wxString& caption ) : caption_( caption ) {}
    virtual ~ValueData() {}
};

//-----------------------------------------------------------------------------
struct ValueRangeData : public ValueData
//-----------------------------------------------------------------------------
{
    int min_;
    int max_;
    int inc_;
    int def_;
    explicit ValueRangeData( const wxString& caption, int min, int max, int inc, int def ) : ValueData( caption ), min_( min ), max_( max ), inc_( inc ), def_( def ) {}
};

//-----------------------------------------------------------------------------
struct ValueChoiceData : public ValueData
//-----------------------------------------------------------------------------
{
    wxArrayString choices_;
    explicit ValueChoiceData( const wxString& caption, const wxArrayString& choices ) : ValueData( caption ), choices_( choices ) {}
};

//-----------------------------------------------------------------------------
class ValuesFromUserDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    std::vector<wxControl*>     ctrls_;
    std::vector<int>            userInputData_;
public:
    explicit ValuesFromUserDlg( wxWindow* pParent, const wxString& title, const std::vector<ValueData*>& inputData );
    const std::vector<wxControl*>& GetUserInputControls( void ) const
    {
        return ctrls_;
    }
};

//-----------------------------------------------------------------------------
class SettingHierarchyDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    void ExpandAll( wxTreeCtrl* pTreeCtrl );
    void ExpandAllChildren( wxTreeCtrl* pTreeCtrl, const wxTreeItemId& item );
    void PopulateTreeCtrl( wxTreeCtrl* pTreeCtrl, wxTreeItemId currentItem, const wxString& currentItemName, const StringToStringMap& settingRelationships );
public:
    explicit SettingHierarchyDlg( wxWindow* pParent, const wxString& title, const StringToStringMap& settingRelationships );
};

//-----------------------------------------------------------------------------
class DetailedRequestInformationDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()

    //-----------------------------------------------------------------------------
    enum TWidgetIDs_DetailedRequestInformation
    //-----------------------------------------------------------------------------
    {
        widSCRequestSelector = widFirst
    };

    mvIMPACT::acquire::FunctionInterface* pFI_;
    wxTreeCtrl* pTreeCtrl_;
    wxSpinCtrl* pSCRequestSelector_;

    void ExpandAll( wxTreeCtrl* pTreeCtrl );
    void ExpandAllChildren( wxTreeCtrl* pTreeCtrl, const wxTreeItemId& item );
    void OnSCRequestSelectorChanged( wxSpinEvent& )
    {
        SelectRequest( pSCRequestSelector_->GetValue() );
    }
    void OnSCRequestSelectorTextChanged( wxCommandEvent& )
    {
        SelectRequest( pSCRequestSelector_->GetValue() );
    }
    void PopulateTreeCtrl( wxTreeCtrl* pTreeCtrl, wxTreeItemId parent, Component itComponent );
    void PopulateTreeCtrl( wxTreeCtrl* pTreeCtrl, const int requestNr );
    void SelectRequest( const int requestNr );
public:
    explicit DetailedRequestInformationDlg( wxWindow* pParent, const wxString& title, mvIMPACT::acquire::FunctionInterface* pFI );
};

//-----------------------------------------------------------------------------
class DriverInformationDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    wxTreeItemId AddComponentListToList( wxTreeCtrl* pTreeCtrl, wxTreeItemId parent, mvIMPACT::acquire::ComponentLocator locator, const char* pName );
    void AddStringPropToList( wxTreeCtrl* pTreeCtrl, wxTreeItemId parent, mvIMPACT::acquire::ComponentLocator locator, const char* pName );
    void ExpandAll( wxTreeCtrl* pTreeCtrl );
    void ExpandAllChildren( wxTreeCtrl* pTreeCtrl, const wxTreeItemId& item );
    void PopulateTreeCtrl( wxTreeCtrl* pTreeCtrl, mvIMPACT::acquire::ComponentIterator itDrivers, const mvIMPACT::acquire::DeviceManager& devMgr );
public:
    explicit DriverInformationDlg( wxWindow* pParent, const wxString& title, mvIMPACT::acquire::ComponentIterator itDrivers, const mvIMPACT::acquire::DeviceManager& devMgr );
};

typedef std::map<wxString, PropData*> NameToFeatureMap;

//-----------------------------------------------------------------------------
class FindFeatureDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
    PropGridFrameBase* pParent_;
    const NameToFeatureMap& nameToFeatureMap_;
    wxTextCtrl* pTCFeatureName_;
    wxCheckBox* pCBMatchCase_;
    wxListBox* pLBFeatureList_;
    //-----------------------------------------------------------------------------
    enum TWidgetIDs_FindFeatures
    //-----------------------------------------------------------------------------
    {
        widTCFeatureName = widFirst,
        widLBFeatureList,
        widCBMatchCase
    };
    void BuildFeatureList( wxArrayString& features, const bool boMatchCase = false, const wxString& pattern = wxEmptyString ) const;
    void OnFeatureListDblClick( wxCommandEvent& e );
    void OnFeatureListSelect( wxCommandEvent& e )
    {
        SelectFeatureInPropertyGrid( e.GetSelection() );
    }
    void OnFeatureNameTextChanged( wxCommandEvent& )
    {
        UpdateFeatureList();
    }
    void OnMatchCaseChanged( wxCommandEvent& )
    {
        UpdateFeatureList();
    }
    void SelectFeatureInPropertyGrid( const wxString& selection );
    void SelectFeatureInPropertyGrid( const int selection );
    void UpdateFeatureList( void );
public:
    explicit FindFeatureDlg( PropGridFrameBase* pParent, const NameToFeatureMap& nameToFeatureMap, const bool boMatchCaseActive );
    bool GetMatchCase( void ) const
    {
        return pCBMatchCase_->GetValue();
    }
};

//-----------------------------------------------------------------------------
class DetailedFeatureInfoDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    wxTextCtrl* pLogWindow_;
    wxTextAttr fixedPitchStyle_;
    wxTextAttr fixedPitchStyleBold_;
    void AddFeatureInfo( const wxString& infoName, const wxString& info );
public:
    explicit DetailedFeatureInfoDlg( wxWindow* pParent, mvIMPACT::acquire::Component comp );
};

//-----------------------------------------------------------------------------
class BinaryDataDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
    HEXStringValidator HEXStringValidator_;
    wxTextCtrl* pTCBinaryData_;
    wxTextCtrl* pTCAsciiData_;
    size_t lastDataLength_;
    wxTextAttr fixedPitchStyle_;
    //-----------------------------------------------------------------------------
    enum TWidgetIDs_FindFeatures
    //-----------------------------------------------------------------------------
    {
        widTCBinaryData = widFirst,
        widTCAsciiData
    };
    void OnBinaryDataTextChanged( wxCommandEvent& );
    void ReformatBinaryData( void );
    static void RemoveSeparatorChars( wxString& data );
    void UpdateAsciiData( void );
public:
    explicit BinaryDataDlg( wxWindow* pParent, const wxString& featureName, const wxString& value );
    static size_t FormatData( const wxString& data, wxString& formattedData, const int lineLength, const int fieldLength );
    wxString GetBinaryData( void ) const;
};

//-----------------------------------------------------------------------------
class AssignSettingsToDisplaysDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    std::vector<wxControl*> ctrls_;
public:
    explicit AssignSettingsToDisplaysDlg( wxWindow* pParent, const wxString& title, const std::vector<std::pair<std::string, int> >& settings, const SettingToDisplayDict& settingToDisplayDict, size_t displayCount );
    const std::vector<wxControl*>& GetUserInputControls( void ) const
    {
        return ctrls_;
    }
};

//-----------------------------------------------------------------------------
class RawImageImportDlg : public OkAndCancelDlg
//-----------------------------------------------------------------------------
{
    PropGridFrameBase* pParent_;
    wxComboBox* pCBPixelFormat_;
    wxComboBox* pCBBayerParity_;
    wxSpinCtrl* pSCWidth_;
    wxSpinCtrl* pSCHeight_;
public:
    explicit RawImageImportDlg( PropGridFrameBase* pParent, const wxString& title, const wxFileName& fileName );
    wxString GetFormat( void ) const;
    wxString GetPixelFormat( void ) const
    {
        return pCBPixelFormat_->GetValue();
    }
    wxString GetBayerParity( void ) const
    {
        return pCBBayerParity_->GetValue();
    }
    long GetWidth( void ) const;
    long GetHeight( void ) const;
};

#endif // ValuesFromUserDlgH
