//-----------------------------------------------------------------------------
#ifndef DeviceListCtrlH
#define DeviceListCtrlH DeviceListCtrlH
//-----------------------------------------------------------------------------
#include "wx/listctrl.h"

class DeviceConfigureFrame;
class LogOutputHandlerDlg;

//-----------------------------------------------------------------------------
/// \brief IDs for the menu commands
enum
//-----------------------------------------------------------------------------
{
    LIST_ABOUT = wxID_ABOUT,
    LIST_QUIT = wxID_EXIT,
    LIST_LIST_VIEW = wxID_HIGHEST,
    LIST_ICON_VIEW,
    LIST_ICON_TEXT_VIEW,
    LIST_SMALL_ICON_VIEW,
    LIST_SMALL_ICON_TEXT_VIEW,
    LIST_REPORT_VIEW,
    LIST_VIRTUAL_VIEW,
    LIST_SMALL_VIRTUAL_VIEW,
    LIST_DESELECT_ALL,
    LIST_SELECT_ALL,
    LIST_DELETE_ALL,
    LIST_DELETE,
    LIST_ADD,
    LIST_EDIT,
    LIST_SORT,
    LIST_SET_FG_COL,
    LIST_SET_BG_COL,
    LIST_TOGGLE_MULTI_SEL,
    LIST_TOGGLE_FIRST,
    LIST_SHOW_COL_INFO,
    LIST_SHOW_SEL_INFO,
    LIST_FOCUS_LAST,
    LIST_FREEZE,
    LIST_THAW,
    LIST_TOGGLE_LINES,
    LIST_CTRL = 1000
};

//-----------------------------------------------------------------------------
enum
//-----------------------------------------------------------------------------
{
    LIST_SET_ID = LIST_CTRL + 1,
    LIST_UPDATE_DESCRIPTION_FILE,
    LIST_UPDATE_FW,
    LIST_UPDATE_KERNEL_DRIVER,
    LIST_UPDATE_DMA_BUFFER_SIZE,
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    LIST_SET_DS_FRIENDLY_NAME,
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    LIST_MV_LAST
};

//-----------------------------------------------------------------------------
enum TListColumn
//-----------------------------------------------------------------------------
{
    lcFamily,
    lcProduct,
    lcSerial,
    lcState,
    lcFWVersion,
    lcKernelDriver,
    lcDeviceID,
    lcAllocatedDMABuffer,
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    lcDSRegistered,
    lcDSFriendlyName,
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    lcLAST_COLUMN
};

//-----------------------------------------------------------------------------
class DeviceListCtrl: public wxListCtrl
//-----------------------------------------------------------------------------
{
public:
    DeviceListCtrl( wxWindow* parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style, DeviceConfigureFrame* pParentFrame );
    void OnActivated( wxListEvent& e );
    void OnColClick( wxListEvent& e );
    void OnSelected( wxListEvent& e );
    void OnDeselected( wxListEvent& e );
    void OnItemRightClick( wxListEvent& e );
    void OnSetID( wxCommandEvent& e );
    void OnUpdateFirmware( wxCommandEvent& e );
    void OnUpdateKernelDriver( wxCommandEvent& e );
    void OnUpdateDMABufferSize( wxCommandEvent& e );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    void OnSetDSFriendlyName( wxCommandEvent& e );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    int  GetCurrentItemIndex( void ) const
    {
        return m_selectedItemID;
    }
private:
    DeviceConfigureFrame*   m_pParentFrame;
    int                     m_selectedItemID;

    DECLARE_NO_COPY_CLASS( DeviceListCtrl )
    DECLARE_EVENT_TABLE()
};

//-----------------------------------------------------------------------------
enum
//-----------------------------------------------------------------------------
{
    LIST_CONFIGURE = LIST_CTRL + 1,
    LIST_ADD_ROW
};

//-----------------------------------------------------------------------------
enum TLogListColumn
//-----------------------------------------------------------------------------
{
    llcSectionName,
    llcDevicesUsingThisSection,
    llcFlags,
    llcOutputMask,
    llcOutputFile,
    llcClearFile,
    llcFileFormat,
    llcStylesheet,
    llcLAST_COLUMN
};

//-----------------------------------------------------------------------------
class LogOutputListCtrl: public wxListCtrl
//-----------------------------------------------------------------------------
{
public:
    LogOutputListCtrl( wxWindow* parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style, LogOutputHandlerDlg* pParent );
    void OnColClick( wxListEvent& e );
    void OnSelected( wxListEvent& e );
    void OnDeselected( wxListEvent& e );
    void OnItemRightClick( wxListEvent& e );
    void OnColumnRightClick( wxListEvent& e );
    void OnModifyItem( wxCommandEvent& e );
    void OnAddItem( wxCommandEvent& e );
    void OnDeleteItem( wxCommandEvent& e );
    void OnDeleteAll( wxCommandEvent& e );
    void ShowPopup( wxListEvent& e );
    int  GetCurrentItemIndex( void ) const
    {
        return m_selectedItemID;
    }
private:
    LogOutputHandlerDlg*    m_pParent;
    int                     m_selectedItemID;

    DECLARE_NO_COPY_CLASS( LogOutputListCtrl )
    DECLARE_EVENT_TABLE()
};

#endif // DeviceListCtrlH

