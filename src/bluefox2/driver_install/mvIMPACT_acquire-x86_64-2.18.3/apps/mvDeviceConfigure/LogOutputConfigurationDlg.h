//-----------------------------------------------------------------------------
#ifndef LogOutputConfigurationDlgH
#define LogOutputConfigurationDlgH LogOutputConfigurationDlgH
//-----------------------------------------------------------------------------
#include "DebugFileParser.h"
#include "wx/wx.h"

//-----------------------------------------------------------------------------
class LogWriterConfigurationDlg : public wxDialog
//-----------------------------------------------------------------------------
{
    LogOutputConfiguration& m_data;
    //-----------------------------------------------------------------------------
    enum
    //-----------------------------------------------------------------------------
    {
        CB_FILE_OUTPUT = 2,
        MAX_OUTPUT_MASK_BIT = 3,
        MAX_LOG_LEVEL = 7
    };
    wxButton*           m_pBtnChooseDir;
    wxButton*           m_pBtnDefault;
    wxButton*           m_pBtnOk;
    wxButton*           m_pBtnCancel;
    wxCheckBox*         m_ppCBOutputDestinations[MAX_OUTPUT_MASK_BIT];
    wxCheckBox*         m_ppCBOutputFlags[MAX_LOG_LEVEL];
    wxCheckBox*         m_pCBClearFile;
    wxStaticText*       m_pOutputFileLabel;
    wxTextCtrl*         m_pOutputFileName;
    wxStaticText*       m_pStylesheetLabel;
    wxTextCtrl*         m_pStylesheetName;
    wxComboBox*         m_pCOMFileFormat;
    //-----------------------------------------------------------------------------
    enum TWidgetIDs
    //-----------------------------------------------------------------------------
    {
        widBtnChooseDir = 1,
        widBtnDefault,
        widBtnOk,
        widBtnCancel,
        widCBFileOutput,
        widCOMFileFormat
    };
    void OnBtnChooseDir( wxCommandEvent& );
    void OnBtnDefault( wxCommandEvent& );
    void OnBtnOk( wxCommandEvent& );
    void OnBtnCancel( wxCommandEvent& );
    void OnCBFileOutput( wxCommandEvent& );
    void OnCOMFileFormatTextChanged( wxCommandEvent& e );
    void SetupDlgControls( void );
    void SetupFileControls( void );
public:
    LogWriterConfigurationDlg( wxWindow* pParent, LogOutputConfiguration& data );
    // any class wishing to process wxWidgets events must use this macro
    DECLARE_EVENT_TABLE()
};

void restoreDefaults( LogOutputConfiguration& data );

#endif // LogOutputConfigurationDlgH
