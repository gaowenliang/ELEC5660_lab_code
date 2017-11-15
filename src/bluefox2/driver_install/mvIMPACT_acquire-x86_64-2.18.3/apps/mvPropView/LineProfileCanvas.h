//-----------------------------------------------------------------------------
#ifndef LineProfileCanvasH
#define LineProfileCanvasH LineProfileCanvasH
//-----------------------------------------------------------------------------
#include "PlotCanvasImageAnalysis.h"

//-----------------------------------------------------------------------------
class LineProfileCanvas : public PlotCanvasImageAnalysis
//-----------------------------------------------------------------------------
{
protected:
    int**               m_ppData;
    int                 m_horDataCount;
    int                 m_verDataCount;
    virtual void        CalculateData( const ImageBuffer* pIB, const TBayerMosaicParity bayerParity ) = 0;
    void                DeallocateProfileBuffer( void );
    virtual int         GetDataCount( void ) const = 0;
    virtual double      GetScaleY( wxCoord h ) const;
    virtual void        OnPaintCustom( wxPaintDC& dc );
public:
    explicit            LineProfileCanvas() {}
    explicit            LineProfileCanvas( wxWindow* parent, const wxString& configName, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
                                           const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
                                           const wxString& name = wxT( "Line Profile" ), bool boActive = false );
    ~LineProfileCanvas();
    virtual wxString    GetGridValue( int row, int col ) const;
    virtual void        RefreshData( const RequestData& data, int x = -1, int y = -1, int w = -1, int h = -1, bool boForceRefresh = false );
};

#endif // LineProfileCanvasH
