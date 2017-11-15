//-----------------------------------------------------------------------------
#include "DeviceListCtrl.h"
#include "DeviceConfigureFrame.h"
#include "LogOutputHandlerDlg.h"
#include <string>
#include <memory>
#include <apps/Common/wxAbstraction.h>

BEGIN_EVENT_TABLE( DeviceListCtrl, wxListCtrl )
    EVT_LIST_COL_CLICK( LIST_CTRL, DeviceListCtrl::OnColClick )
    EVT_LIST_ITEM_ACTIVATED( LIST_CTRL, DeviceListCtrl::OnActivated )
    EVT_LIST_ITEM_SELECTED( LIST_CTRL, DeviceListCtrl::OnSelected )
    EVT_LIST_ITEM_RIGHT_CLICK( LIST_CTRL, DeviceListCtrl::OnItemRightClick )
    EVT_LIST_ITEM_DESELECTED( LIST_CTRL, DeviceListCtrl::OnDeselected )
    EVT_MENU( LIST_SET_ID, DeviceListCtrl::OnSetID )
    EVT_MENU( LIST_UPDATE_FW, DeviceListCtrl::OnUpdateFirmware )
    EVT_MENU( LIST_UPDATE_DMA_BUFFER_SIZE, DeviceListCtrl::OnUpdateDMABufferSize )
    EVT_MENU( LIST_UPDATE_KERNEL_DRIVER, DeviceListCtrl::OnUpdateKernelDriver )
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    EVT_MENU( LIST_SET_DS_FRIENDLY_NAME, DeviceListCtrl::OnSetDSFriendlyName )
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
END_EVENT_TABLE()

using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
#if wxCHECK_VERSION(2, 9, 1)
int wxCALLBACK ListCompareFunction( wxIntPtr item1, wxIntPtr item2, wxIntPtr column )
#else
int wxCALLBACK ListCompareFunction( long item1, long item2, long column )
#endif // #if wxCHECK_VERSION(2, 9, 0)
//-----------------------------------------------------------------------------
{
    DeviceManager devMgr;
    switch( column )
    {
    case lcFamily:
        return devMgr[item1]->family.read().compare( devMgr[item2]->family.read() );
    case lcProduct:
        return devMgr[item1]->product.read().compare( devMgr[item2]->product.read() );
    case lcSerial:
        return devMgr[item1]->serial.read().compare( devMgr[item2]->serial.read() );
    case lcState:
        return devMgr[item1]->state.readS().compare( devMgr[item2]->state.readS() );
    case lcFWVersion:
        return devMgr[item1]->firmwareVersion.read() - devMgr[item2]->firmwareVersion.read();
    case lcDeviceID:
        return devMgr[item1]->deviceID.read() - devMgr[item2]->deviceID.read();
    default:
        return 0;
    }
}

//-----------------------------------------------------------------------------
DeviceListCtrl::DeviceListCtrl( wxWindow* parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style, DeviceConfigureFrame* pParentFrame )
    : wxListCtrl( parent, id, pos, size, style ), m_pParentFrame( pParentFrame ), m_selectedItemID( -1 )  {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnActivated( wxListEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pParentFrame )
    {
        m_pParentFrame->ActivateDeviceIn_wxPropView( m_selectedItemID );
    }
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnColClick( wxListEvent& e )
//-----------------------------------------------------------------------------
{
    SortItems( ListCompareFunction, e.GetColumn() );
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnSelected( wxListEvent& e )
//-----------------------------------------------------------------------------
{
    m_selectedItemID = static_cast<int>( GetItemData( e.m_itemIndex ) );
    if( m_pParentFrame )
    {
        m_pParentFrame->UpdateMenu( m_selectedItemID );
    }
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnDeselected( wxListEvent& )
//-----------------------------------------------------------------------------
{
    m_selectedItemID = -1;
    if( m_pParentFrame )
    {
        m_pParentFrame->UpdateMenu( m_selectedItemID );
    }
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnItemRightClick( wxListEvent& e )
//-----------------------------------------------------------------------------
{
    wxMenu menu( wxT( "" ) );
    wxMenuItem* pMISetID = menu.Append( LIST_SET_ID, wxT( "&Set ID" ) );
    wxMenuItem* pMIFWUpdate = menu.Append( LIST_UPDATE_FW, wxT( "Update &Firmware" ) );
    wxMenuItem* pMIKernelDriverUpdate = menu.Append( LIST_UPDATE_KERNEL_DRIVER, wxT( "Update &Kernel Driver" ) );
    wxMenuItem* pMIDMABufferSizeUpdate = menu.Append( LIST_UPDATE_DMA_BUFFER_SIZE, wxT( "Update &Permanent DMA Buffer Size" ) );
#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    menu.Append( LIST_SET_DS_FRIENDLY_NAME, wxT( "Set DirectShow Friendly Name" ) );
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
    bool boSetIDSupported = false;
    bool boFWUPdateSupport = false;
    bool boKernelDriverUpdateSupported = false;
    bool boNewerDriverAvailable = false;
    bool boUpdateDMABufferSize = false;
    if( m_pParentFrame &&
        ( m_selectedItemID >= 0 ) &&
        ( static_cast<int>( m_pParentFrame->GetDeviceManager().deviceCount() ) > m_selectedItemID ) )
    {
        Device* pDev = m_pParentFrame->GetDeviceManager().getDevice( static_cast<unsigned int>( GetItemData( e.m_itemIndex ) ) );
        auto_ptr<DeviceHandler> pHandler( m_pParentFrame->GetHandlerFactory().CreateObject( ConvertedString( pDev->family.read() ), pDev ) );
        if( pHandler.get() )
        {
            boSetIDSupported = pHandler->SupportsSetID();
            boFWUPdateSupport = pHandler->SupportsFirmwareUpdate();
            string kernelDriverName;
            boKernelDriverUpdateSupported = pHandler->SupportsKernelDriverUpdate( boNewerDriverAvailable, kernelDriverName );
            boUpdateDMABufferSize = pHandler->SupportsDMABufferSizeUpdate();
        }
    }
    pMISetID->Enable( boSetIDSupported );
    pMIFWUpdate->Enable( boFWUPdateSupport );
    pMIKernelDriverUpdate->Enable( boKernelDriverUpdateSupported && boNewerDriverAvailable );
    pMIDMABufferSizeUpdate->Enable( boUpdateDMABufferSize );
    PopupMenu( &menu, e.GetPoint() );
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnSetID( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pParentFrame )
    {
        m_pParentFrame->SetID( m_selectedItemID );
    }
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnUpdateFirmware( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pParentFrame )
    {
        m_pParentFrame->UpdateFirmware( m_selectedItemID );
    }
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnUpdateKernelDriver( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pParentFrame )
    {
        m_pParentFrame->UpdateKernelDriver( m_selectedItemID );
    }
}

//-----------------------------------------------------------------------------
void DeviceListCtrl::OnUpdateDMABufferSize( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pParentFrame )
    {
        m_pParentFrame->UpdateDMABufferSize( m_selectedItemID );
    }
}

#ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT
//-----------------------------------------------------------------------------
void DeviceListCtrl::OnSetDSFriendlyName( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    if( m_pParentFrame )
    {
        m_pParentFrame->SetDSFriendlyName( m_selectedItemID );
    }
}
#endif // #ifdef BUILD_WITH_DIRECT_SHOW_SUPPORT

BEGIN_EVENT_TABLE( LogOutputListCtrl, wxListCtrl )
    EVT_LIST_COL_CLICK( LIST_CTRL, LogOutputListCtrl::OnColClick )
    EVT_LIST_COL_RIGHT_CLICK( LIST_CTRL, LogOutputListCtrl::OnColumnRightClick )
    EVT_LIST_ITEM_SELECTED( LIST_CTRL, LogOutputListCtrl::OnSelected )
    EVT_LIST_ITEM_RIGHT_CLICK( LIST_CTRL, LogOutputListCtrl::OnItemRightClick )
    EVT_LIST_ITEM_DESELECTED( LIST_CTRL, LogOutputListCtrl::OnDeselected )
    EVT_MENU( LIST_CONFIGURE, LogOutputListCtrl::OnModifyItem )
    EVT_MENU( LIST_ADD, LogOutputListCtrl::OnAddItem )
    EVT_MENU( LIST_DELETE, LogOutputListCtrl::OnDeleteItem )
    EVT_MENU( LIST_DELETE_ALL, LogOutputListCtrl::OnDeleteAll )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
LogOutputListCtrl::LogOutputListCtrl( wxWindow* parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style, LogOutputHandlerDlg* pParent )
    : wxListCtrl( parent, id, pos, size, style ), m_pParent( pParent ), m_selectedItemID( -1 )  {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnColClick( wxListEvent& )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnColumnRightClick( wxListEvent& e )
//-----------------------------------------------------------------------------
{
    ShowPopup( e );
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnSelected( wxListEvent& e )
//-----------------------------------------------------------------------------
{
    m_selectedItemID = e.m_itemIndex;
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnDeselected( wxListEvent& )
//-----------------------------------------------------------------------------
{
    m_selectedItemID = -1;
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnItemRightClick( wxListEvent& e )
//-----------------------------------------------------------------------------
{
    ShowPopup( e );
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnModifyItem( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pParent->ConfigureLogger( m_selectedItemID );
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnAddItem( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pParent->InsertItem( m_selectedItemID );
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnDeleteItem( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_pParent->DeleteItem( m_selectedItemID );
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::OnDeleteAll( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    m_selectedItemID = -1;
    m_pParent->DeleteList();
    DeleteAllItems();
}

//-----------------------------------------------------------------------------
void LogOutputListCtrl::ShowPopup( wxListEvent& e )
//-----------------------------------------------------------------------------
{
    wxMenu menu( wxT( "" ) );
    if( m_selectedItemID != -1 )
    {
        menu.Append( LIST_CONFIGURE, wxT( "&Configure Item" ) );
    }
    menu.Append( LIST_ADD, wxT( "&Add Item" ) );
    if( m_selectedItemID != -1 )
    {
        menu.Append( LIST_DELETE, wxT( "&Delete Item" ) );
    }
    menu.Append( LIST_DELETE_ALL, wxT( "C&lear List" ) );
    PopupMenu( &menu, e.GetPoint() );
}
