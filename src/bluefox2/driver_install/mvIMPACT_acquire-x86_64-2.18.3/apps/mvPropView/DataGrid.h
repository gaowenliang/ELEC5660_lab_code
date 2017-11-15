//-----------------------------------------------------------------------------
#ifndef DataGridH
#define DataGridH DataGridH
//-----------------------------------------------------------------------------
#include <wx/grid.h>

//-----------------------------------------------------------------------------
class DataGridDataProvider
//-----------------------------------------------------------------------------
{
protected:
    wxString            m_GridValueFormatString;
public:
    virtual wxString    GetGridValue( int row, int col ) const = 0;
    const wxString&     GetGridValueFormatString( void ) const
    {
        return m_GridValueFormatString;
    }
    virtual void        SetGridValue( int /*row*/, int /*col*/, const wxString& /*value*/ ) {}
    virtual void        SetGridValueFormatString( const wxString& gridValueFormatString ) = 0;
};

//-----------------------------------------------------------------------------
class DataGridTable : public wxGridTableBase
//-----------------------------------------------------------------------------
{
    DataGridDataProvider* m_pDataProvider;
    long m_Rows;
    long m_Columns;
    bool UpdateRowsCols( long rows, long cols );
protected:
    DataGridDataProvider* GetDataProvider( void ) const
    {
        return m_pDataProvider;
    }
public:
    explicit         DataGridTable( DataGridDataProvider* pDataProvider, long rows, long columns );
    virtual int      GetNumberRows( void )
    {
        return m_Rows;
    }
    virtual int      GetNumberCols( void )
    {
        return m_Columns;
    }
    virtual bool     IsEmptyCell( int /*row*/, int /*col*/ )
    {
        return false;
    }
    virtual bool     AppendRows( size_t numRows = 1 )
    {
        return UpdateRowsCols( m_Rows + static_cast<long>( numRows ), m_Columns );
    }
    virtual bool     DeleteRows( size_t /*pos = 0*/, size_t numRows = 1 )
    {
        return UpdateRowsCols( m_Rows - static_cast<long>( numRows ), m_Columns );
    }
    virtual bool     AppendCols( size_t numCols = 1 )
    {
        return UpdateRowsCols( m_Rows, m_Columns + static_cast<long>( numCols ) );
    }
    virtual bool     DeleteCols( size_t /*pos = 0*/, size_t numCols = 1 )
    {
        return UpdateRowsCols( m_Rows, m_Columns - static_cast<long>( numCols ) );
    }
    virtual wxString GetValue( int row, int col );
    virtual void     SetValue( int row, int col, const wxString& val );
};

//-----------------------------------------------------------------------------
class DataGridWithClipboardFeature : public wxGrid
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
    enum
    {
        GRID_TO_CLIPBOARD,
        GRID_SELECTION_TO_CLIPBOARD,
        SET_GRID_VALUE_FORMAT_STRING
    };
    void SetClipboardData( int startRow, int rows, int startCol, int cols );
    DataGridDataProvider* pDataProvider_;
public:
    explicit DataGridWithClipboardFeature() : wxGrid(), pDataProvider_( 0 ) {}
    explicit DataGridWithClipboardFeature( wxWindow* parent, wxWindowID id, const wxPoint& pos = wxDefaultPosition,
                                           const wxSize& size = wxDefaultSize, long style = wxWANTS_CHARS, const wxString& name = wxPanelNameStr );
    void OnCellRightClick( wxGridEvent& e );
    void OnGridToClipboard( wxCommandEvent& e );
    void OnGridSelectionToClipboard( wxCommandEvent& e );
    void SetDataProvider( DataGridDataProvider* p );
    void OnSetGridValueFormatString( wxCommandEvent& e );
};

#endif // DataGridH
