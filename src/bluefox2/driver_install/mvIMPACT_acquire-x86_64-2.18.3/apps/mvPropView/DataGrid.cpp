#include "DataGrid.h"
#include <wx/clipbrd.h>
#include <wx/dataobj.h>
#include <wx/menu.h>
#include <wx/textdlg.h>

//=============================================================================
//================= Implementation DataGridTable ==============================
//=============================================================================
//-----------------------------------------------------------------------------
DataGridTable::DataGridTable( DataGridDataProvider* pDataProvider, long rows, long columns ) : wxGridTableBase(),
    m_pDataProvider( pDataProvider ), m_Rows( rows ), m_Columns( columns )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
wxString DataGridTable::GetValue( int row, int col )
//-----------------------------------------------------------------------------
{
    return m_pDataProvider->GetGridValue( row, col );
}

//-----------------------------------------------------------------------------
void DataGridTable::SetValue( int row, int col, const wxString& val )
//-----------------------------------------------------------------------------
{
    GetDataProvider()->SetGridValue( row, col, val );
}

//-----------------------------------------------------------------------------
bool DataGridTable::UpdateRowsCols( long rows, long cols )
//-----------------------------------------------------------------------------
{
    bool updated = false;
    if( GetView() )
    {
        if( rows > m_Rows )
        {
            wxGridTableMessage msg( this, wxGRIDTABLE_NOTIFY_ROWS_INSERTED, m_Rows - 1, rows - m_Rows );
            GetView()->ProcessTableMessage( msg );
            updated = true;
        }
        else if( rows < m_Rows )
        {
            wxGridTableMessage msg( this, wxGRIDTABLE_NOTIFY_ROWS_DELETED, m_Rows - 1, m_Rows - rows );
            GetView()->ProcessTableMessage( msg );
            updated = true;
        }

        if( cols > m_Columns )
        {
            wxGridTableMessage msg( this, wxGRIDTABLE_NOTIFY_COLS_INSERTED, m_Columns - 1, cols - m_Columns );
            GetView()->ProcessTableMessage( msg );
            updated = true;
        }
        else if( cols < m_Columns )
        {
            wxGridTableMessage msg( this, wxGRIDTABLE_NOTIFY_COLS_DELETED,  m_Columns - 1, m_Columns - cols );
            GetView()->ProcessTableMessage( msg );
            updated = true;
        }
    }
    m_Rows = rows;
    m_Columns = cols;
    return updated;
}

//=============================================================================
//================= Implementation DataGridWithClipboardFeature ===============
//=============================================================================
BEGIN_EVENT_TABLE( DataGridWithClipboardFeature, wxGrid )
    EVT_GRID_CELL_RIGHT_CLICK( DataGridWithClipboardFeature::OnCellRightClick )
    EVT_MENU( GRID_TO_CLIPBOARD, DataGridWithClipboardFeature::OnGridToClipboard )
    EVT_MENU( GRID_SELECTION_TO_CLIPBOARD, DataGridWithClipboardFeature::OnGridSelectionToClipboard )
    EVT_MENU( SET_GRID_VALUE_FORMAT_STRING, DataGridWithClipboardFeature::OnSetGridValueFormatString )
END_EVENT_TABLE()

//-----------------------------------------------------------------------------
DataGridWithClipboardFeature::DataGridWithClipboardFeature( wxWindow* parent, wxWindowID id, const wxPoint& pos /* = wxDefaultPosition*/,
        const wxSize& size /* = wxDefaultSize */, long style /* = wxWANTS_CHARS */,
        const wxString& name /* = wxPanelNameStr */ ) : wxGrid( parent, id, pos, size, style, name ), pDataProvider_( 0 )
//-----------------------------------------------------------------------------
{
    SetRowLabelSize( 0 );
}

//-----------------------------------------------------------------------------
void DataGridWithClipboardFeature::OnCellRightClick( wxGridEvent& e )
//-----------------------------------------------------------------------------
{
    wxMenu menu( wxT( "" ) );
    menu.Append( GRID_TO_CLIPBOARD, wxT( "&Copy Grid To Clipboard" ) );
    //menu.Append( GRID_SELECTION_TO_CLIPBOARD, wxT("Copy Grid &Selection To Clipboard") );
    menu.Append( SET_GRID_VALUE_FORMAT_STRING, wxT( "&Set Grid Value Format String" ) );
    PopupMenu( &menu, e.GetPosition() );
}

//-----------------------------------------------------------------------------
void DataGridWithClipboardFeature::OnGridToClipboard( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    SetClipboardData( 0, GetNumberRows(), 0, GetNumberCols() );
}

//-----------------------------------------------------------------------------
void DataGridWithClipboardFeature::OnGridSelectionToClipboard( wxCommandEvent& )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
void DataGridWithClipboardFeature::OnSetGridValueFormatString( wxCommandEvent& )
//-----------------------------------------------------------------------------
{
    wxString newFormatString = ::wxGetTextFromUser( wxT( "Enter the new format string for this grid(printf style. For the sake of flexibility everything\nis allowed here, but weird/invalid(e.g. %.2y) inputs might cause crashes)." ),
                               wxT( "New format string" ),
                               pDataProvider_->GetGridValueFormatString(),
                               this );
    pDataProvider_->SetGridValueFormatString( newFormatString );
}

//-----------------------------------------------------------------------------
void DataGridWithClipboardFeature::SetClipboardData( int startRow, int rows, int startCol, int cols )
//-----------------------------------------------------------------------------
{
    if( ( rows <= 0 ) || ( cols <= 0 ) )
    {
        return;
    }

    wxString data;
    for( int row = startRow; row < rows; row++ )
    {
        for( int col = startCol; col < cols; col++ )
        {
            data.Append( pDataProvider_->GetGridValue( row, col ) );
            if( col < cols - 1 )
            {
                data.Append( wxT( ", " ) );
            }
        }
        if( row < rows - 1 )
        {
            data.Append( wxT( "\n" ) );
        }
    }

    if( wxTheClipboard->Open() )
    {
        wxTheClipboard->SetData( new wxTextDataObject( data ) );
        wxTheClipboard->Flush();
        wxTheClipboard->Close();
    }
}

//-----------------------------------------------------------------------------
void DataGridWithClipboardFeature::SetDataProvider( DataGridDataProvider* p )
//-----------------------------------------------------------------------------
{
    pDataProvider_ = p;
}
