//-----------------------------------------------------------------------------
#ifndef PlotCanvasH
#define PlotCanvasH PlotCanvasH
//-----------------------------------------------------------------------------
#include "DrawingCanvas.h"
#include <wx/dcclient.h>

//-----------------------------------------------------------------------------
class PlotCanvas : public DrawingCanvas
//-----------------------------------------------------------------------------
{
    DECLARE_EVENT_TABLE()
    int                     m_BorderWidth;

    static double CalculateIntersection( const int x1, const int y1, const int x2, const int y2, const int yIntersection )
    {
        // y = m*x + n
        const double m = static_cast<double>( y2 - y1 ) / static_cast<double>( x2 - x1 );
        const double n = static_cast<double>( y1 ) - m * static_cast<double>( x1 );
        // x = (y - n)/m
        return ( static_cast<double>( yIntersection ) - n ) / m;
    }
    static double CalculateXStart( unsigned int from, unsigned int to, const int x1, const int y1, const int x2, const int y2, const int xCheck, const int yCheck )
    {
        if( static_cast<unsigned int>( yCheck ) < from )
        {
            return CalculateIntersection( x1, y1, x2, y2, from );
        }
        else if( static_cast<unsigned int>( yCheck ) > to )
        {
            return CalculateIntersection( x1, y1, x2, y2, to );
        }
        return static_cast<double>( xCheck );
    }
protected:
    int                     m_ImageCount;
    int                     m_UpdateFrequency;
    int                     m_UpdateFrequencyMin;
    int                     m_UpdateFrequencyMax;

    virtual void            OnPaintCustom( wxPaintDC& dc ) = 0;
    void                    DrawInfoString( wxPaintDC& dc, const wxString& info, wxCoord& xOffset, wxCoord yOffset, const wxColour& colour ) const;
    void                    DrawMarkerLines( wxPaintDC& dc, const wxCoord w, const wxCoord h, const double scaleX ) const;
    void                    DrawProfileLine( wxPaintDC& dc, int h, int startOffset, double scaleX, double scaleY, unsigned int from, unsigned int to, int* pData, int elementCount, const wxColour& colour ) const;
    virtual double          GetScaleX( wxCoord /*w*/ ) const
    {
        return 1.0;
    }
    virtual double          GetScaleY( wxCoord /*h*/ ) const
    {
        return 1.0;
    }
    virtual unsigned int    GetXMarkerParameters( unsigned int& from, unsigned int& to ) const = 0;
    static unsigned int     GetXMarkerStepWidth( const unsigned int from, const unsigned int to );
    virtual void            OnPaint( wxPaintEvent& );
    virtual void            OnUpdateFrequencyChanged( void ) {}
public:
    explicit                PlotCanvas() {}
    explicit                PlotCanvas( wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
                                        const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
                                        const wxString& name = wxT( "PlotCanvas" ), bool boActive = false );
    bool                    SetUpdateFrequency( int frequency );
    int                     GetUpdateFrequency( void ) const
    {
        return m_UpdateFrequency;
    }
    int                     GetUpdateFrequencyMin( void ) const
    {
        return m_UpdateFrequencyMin;
    }
    int                     GetUpdateFrequencyMax( void ) const
    {
        return m_UpdateFrequencyMax;
    }
    int                     GetBorderWidth( void ) const
    {
        return m_BorderWidth;
    }
    void                    SetBorderWidth( int borderWidth );
};

#endif // PlotCanvasH
