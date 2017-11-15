//-----------------------------------------------------------------------------
#ifndef PlotCanvasImageAnalysisH
#define PlotCanvasImageAnalysisH PlotCanvasImageAnalysisH
//-----------------------------------------------------------------------------
#include "DataGrid.h"
#include "DevData.h"
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "PlotCanvas.h"

class ImageCanvas;

//-----------------------------------------------------------------------------
class PlotCanvasImageAnalysis : public PlotCanvas, public DataGridDataProvider
//-----------------------------------------------------------------------------
{
public:
    //-----------------------------------------------------------------------------
    enum TDisplayMethod
    //-----------------------------------------------------------------------------
    {
        dmGraphical,
        dmNumerical
    };
    //-----------------------------------------------------------------------------
    enum TPlotFeature
    //-----------------------------------------------------------------------------
    {
        pfPercentageWindow,
        pfHistoryDepth,
        pfStepWidth,
        pfGridSteps,
        pfProcessBayerParity
    };
private:
    wxColour            m_AOIColour;
    ImageCanvas*        m_pImageCanvas;
    wxString            m_ConfigName;
    bool                m_boAOIFullMode;
    bool                m_boProcessBayerParity;
    TDisplayMethod      m_DisplayMethod;
    bool                SetAOI( const mvIMPACT::acquire::ImageBuffer* pIB, int x, int y, int w, int h );
protected:
    //-----------------------------------------------------------------------------
    struct NamedColour
            //-----------------------------------------------------------------------------
    {
        const wxColour* pColour_;
        wxString description_;
        NamedColour() : pColour_( 0 ), description_( wxT( "" ) ) {}
        NamedColour( const wxColour* p, const wxString& d ) : pColour_( p ), description_( d ) {}
    };
    //-----------------------------------------------------------------------------
    template<typename _Ty>
    bool AssignDisplayParameter( _Ty& parameter, const _Ty& newValue )
    //-----------------------------------------------------------------------------
    {
        if( parameter == newValue )
        {
            return false;
        }

        parameter = newValue;
        if( m_DisplayMethod == dmGraphical )
        {
            UpdateAnalysisOutput( true );
        }
        return true;
    }
    //-----------------------------------------------------------------------------
    enum
    //-----------------------------------------------------------------------------
    {
        VAL_COUNT_8_BIT = 256,
        VAL_COUNT_10_BIT = VAL_COUNT_8_BIT << 2,
        VAL_COUNT_12_BIT = VAL_COUNT_8_BIT << 4,
        VAL_COUNT_14_BIT = VAL_COUNT_8_BIT << 6,
        VAL_COUNT_16_BIT = VAL_COUNT_8_BIT << 8
    };
    //-----------------------------------------------------------------------------
    enum TPlane
    //-----------------------------------------------------------------------------
    {
        pRed = 0,
        pY = 0,
        pEvenEven = 0,
        pChannel0Hor = 0,
        pChannel0 = 0,
        pGreen = 1,
        pU = 1,
        pCb = 1,
        pEvenOdd = 1,
        pChannel0Ver = 1,
        pChannel1 = 1,
        pBlue = 2,
        pCr = 2,
        pV = 2,
        pOddEven = 2,
        pChannel1Hor = 2,
        pChannel2 = 2,
        pOddOdd = 3,
        pChannel1Ver = 3,
        pChannel2Hor = 4,
        pChannel2Ver = 5,
        pChannel3Hor = 6,
        pChannel3Ver = 7,
        PLANE_CNT
    };

    unsigned int                m_valCount;
    int                         m_AOIx;
    int                         m_AOIy;
    int                         m_AOIw;
    int                         m_AOIh;
    AOI*                        m_pAOI;
    std::set<TPlotFeature>      m_plotFeatures;
    unsigned int                m_HistoryDepth;
    int                         m_GridStepsX;
    int                         m_GridStepsY;
    int                         m_ChannelCount;
    std::vector<wxString>       m_PlotNames;
    mvIMPACT::acquire::TImageBufferPixelFormat m_pixelFormat;
    mvIMPACT::acquire::TBayerMosaicParity m_bayerParity;
    bool                        m_boUnsupportedPixelFormat;
    static const wxColour       m_ADarkerKindOfGreen;
    static const wxColour       m_CyanLight;
    static const wxColour       m_Magenta;
    static const wxColour       m_MagentaLight;
    NamedColour                 m_Pens[PLANE_CNT];
    int                         m_DrawStart_percent;
    int                         m_DrawWindow_percent;
    DataGridWithClipboardFeature*   m_pNumericalDisplay;

    virtual void                CreateGridCustom( void );
    int                         GetPercentageToDraw( void ) const
    {
        return ( ( 100 - m_DrawStart_percent ) < m_DrawWindow_percent ) ? ( 100 - m_DrawStart_percent ) : m_DrawWindow_percent;
    }
    void                        GetDrawRange( unsigned int* pFrom, unsigned int* pTo ) const;
    virtual int                 GetRowCountNeededForNumericalDisplay( void ) const;
    virtual void                HandleSizeEvent( wxSizeEvent& e );
    bool                        MustUpdate( const mvIMPACT::acquire::ImageBuffer* pIB, int plotChannelCount, int x, int y, int w, int h, bool boForceRefresh, bool* pboAOIChanged = 0, bool* pboImageFormatChanged = 0 );
    virtual void                RefreshAnalysisData( void ) {}
    void                        SetDefaultPens( mvIMPACT::acquire::TImageBufferPixelFormat pixelFormat );
    void                        SetBayerDescriptions( mvIMPACT::acquire::TBayerMosaicParity bayerParity );
    void                        SetBayerPens( mvIMPACT::acquire::TImageBufferPixelFormat pixelFormat, mvIMPACT::acquire::TBayerMosaicParity bayerParity );
    void                        SetupGDIData( const mvIMPACT::acquire::ImageBuffer* pIB, int channelCount, mvIMPACT::acquire::TBayerMosaicParity bayerParity );
    void                        UpdateAnalysisOutput( bool boEraseBackground );
    virtual void                UpdateGrid( void );
    virtual void                UpdateInternalData( void ) {}
public:
    explicit                    PlotCanvasImageAnalysis() {}
    explicit                    PlotCanvasImageAnalysis( wxWindow* parent, const wxString& configName, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize, long style = wxBORDER_NONE,
            const wxString& name = wxT( "PlotCanvasImageAnalysis" ), bool boActive = false, int channelCount = 0 );
    void                        CreateGrid( wxWindow* pParent, wxWindowID gridID );
    wxGrid*                     GetGrid( void ) const;
    virtual void                SetGridValueFormatString( const wxString& gridValueFormatString );
    void                        SetDisplayMethod( TDisplayMethod displayMethod );
    TDisplayMethod              GetDisplayMethod( void ) const
    {
        return m_DisplayMethod;
    }
    const wxString&             GetConfigName( void ) const
    {
        return m_ConfigName;
    }
    virtual void                RefreshData( const RequestData& data, int x = -1, int y = -1, int w = -1, int h = -1, bool boForceRefresh = false ) = 0;
    virtual void                SetActive( bool boActive );
    bool                        IsAOIFullModeActive( void ) const
    {
        return m_boAOIFullMode;
    }
    void                        SetAOIFullMode( bool boActive );
    void                        SetAOIColour( const wxColour& );
    const AOI*                  GetAOI( void ) const
    {
        return m_pAOI;
    }
    wxRect                      GetAOIRect( void ) const
    {
        return wxRect( m_AOIx, m_AOIy, m_AOIw, m_AOIh );
    }
    void                        SetProcessBayerParity( bool boActive );
    bool                        GetProcessBayerParity( void ) const
    {
        return m_boProcessBayerParity;
    }
    bool                        HasPlotSelection( void ) const
    {
        return !m_PlotNames.empty();
    }
    const std::vector<wxString>& GetAvailablePlotSelections( void ) const
    {
        return m_PlotNames;
    }
    virtual wxString            GetPlotSelection( void ) const
    {
        return wxString( wxT( "" ) );
    }
    virtual void                SetPlotSelection( const wxString& /*plotName*/ ) {}
    bool                        HasFeature( TPlotFeature feature ) const
    {
        return m_plotFeatures.find( feature ) != m_plotFeatures.end();
    }
    void                        SetHistoryDepth( unsigned int historyDepth );
    unsigned int                GetHistoryDepth( void ) const
    {
        return m_HistoryDepth;
    }
    void                        SetDrawStartPercent( int value_percent );
    void                        SetDrawWindowWidthPercent( int value_percent );
    void                        SetGridSteps( int XSteps, int YSteps );
    void                        RegisterImageCanvas( ImageCanvas* p );
    ImageCanvas*                GetImageCanvas( void ) const
    {
        return m_pImageCanvas;
    }
};

//-----------------------------------------------------------------------------
inline void GetYUV411OffsetsForUAndV( const int pixel, int& offsetU, int& offsetV )
//-----------------------------------------------------------------------------
{
    switch( pixel % 4 )
    {
    case 3:
        offsetU = -5;
        offsetV = -2;
        break;
    case 2:
        offsetU = -4;
        offsetV = -1;
        break;
    case 1:
        offsetU = -2;
        offsetV = 1;
        break;
    case 0:
        offsetU = -1;
        offsetV = 2;
        break;
    }
}

//-----------------------------------------------------------------------------
class ProcessingHelperYUV411_UYYVYY
//-----------------------------------------------------------------------------
{
    int offsetToFirstYInEachLine_;
    int offsetU_;
    int offsetV_;
public:
    explicit ProcessingHelperYUV411_UYYVYY( const int AOIx ) : offsetToFirstYInEachLine_( 0 ), offsetU_( 0 ), offsetV_( 0 )
    {
        int YOffsetInGangOfFour = 1 + ( AOIx % 4 );
        // valid value are 1, 2, 4, 5. '3' contains the 'V' component
        if( YOffsetInGangOfFour > 2 )
        {
            ++YOffsetInGangOfFour; // jump over the 'V'
        }
        offsetToFirstYInEachLine_ = YOffsetInGangOfFour + ( ( AOIx / 4 ) * 6 );
    }
    int GetOffsetToFirstY( void ) const
    {
        return offsetToFirstYInEachLine_;
    }
    int GetUOffset( void ) const
    {
        return offsetU_;
    }
    int GetVOffset( void ) const
    {
        return offsetV_;
    }
    void RefreshUAndVOffsets( int pixel )
    {
        GetYUV411OffsetsForUAndV( pixel, offsetU_, offsetV_ );
    }
};

#endif // PlotCanvasImageAnalysisH
