//-----------------------------------------------------------------------------
#ifndef ImageCanvasH
#define ImageCanvasH ImageCanvasH
//-----------------------------------------------------------------------------
#include "DrawingCanvas.h"
#include <map>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <vector>
#include <wx/colordlg.h>
#include <wx/dcclient.h>

typedef std::map<const DrawingCanvas*, AOI*> AOIContainer;

struct ImageCanvasImpl;
class PlotCanvasImageAnalysis;

//-----------------------------------------------------------------------------
class ImageCanvas : public DrawingCanvas
//-----------------------------------------------------------------------------
{
public:
    explicit ImageCanvas() {}
    explicit ImageCanvas( wxWindow* pApp, wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize,
                          long style = wxBORDER_NONE, const wxString& name = wxT( "ImageCanvas" ), bool boActive = true );
    virtual ~ImageCanvas();
    wxString                                GetCurrentPixelDataAsString( void ) const;
    wxRect                                  GetVisiblePartOfImage( void );
    bool                                    IsScaled( void ) const
    {
        return m_boScaleToClientSize;
    }
    bool                                    IsFullScreen( void ) const;
    void                                    SetFullScreenMode( bool boActive );
    bool                                    InfoOverlayActive( void ) const
    {
        return m_boShowInfoOverlay;
    }
    void                                    RefreshScrollbars( bool boMoveToMousePos = false );
    AOI*                                    RegisterAOI( const wxRect&, const wxColour&, const DrawingCanvas* const pOwner );
    bool                                    RegisterMonitorDisplay( ImageCanvas* pMonitorDisplay );
    ImageCanvas*                            GetMonitorDisplay( void ) const
    {
        return m_pMonitorDisplay;
    }
    bool                                    RemoveAOI( const DrawingCanvas* const pOwner );
    //-----------------------------------------------------------------------------
    enum TSaveResult
    //-----------------------------------------------------------------------------
    {
        srOK,
        srNoImage,
        srFailedToSave
    };
    //-----------------------------------------------------------------------------
    enum TScalingMode
    //-----------------------------------------------------------------------------
    {
        smNearestNeighbour,
        smLinear,
        smCubic
    };
    void                                    IncreaseSkippedCounter( size_t count );
    void                                    HandleMouseAndKeyboardEvents( bool boHandle );
    void                                    DisableDoubleClickAndPrunePopupMenu( bool boDisable );
    TSaveResult                             SaveCurrentImage( const wxString& filenameAndPath, const wxString& extension ) const;
    void                                    SetActiveAnalysisPlot( const PlotCanvasImageAnalysis* pPlot );
    const PlotCanvasImageAnalysis*          GetActiveAnalysisPlot( void ) const
    {
        return m_pActiveAnalysisPlot;
    }
    bool                                    SetAOI( const PlotCanvasImageAnalysis* pPlot, int x, int y, int w, int h );
    bool                                    SetImage( const mvIMPACT::acquire::ImageBuffer* pIB, bool boMustRefresh = true );
    const mvIMPACT::acquire::ImageBuffer*   GetImage( void ) const
    {
        return m_pIB;
    }
    void                                    SetScaling( bool boOn );
    void                                    SetInfoOverlay( const std::vector<wxString>& infoStrings );
    void                                    SetInfoOverlayMode( bool boOn );
    void                                    ResetRequestInProgressFlag( void );
    void                                    ResetSkippedImagesCounter( void );
    void                                    SetImageModificationWarningOutput( bool boOn );
    bool                                    GetImageModificationWarningOutput( void ) const
    {
        return m_boShowImageModificationWarning;
    }
    void                                    SetPerformanceWarningOutput( bool boOn );
    bool                                    GetPerformanceWarningOutput( void ) const
    {
        return m_boShowPerformanceWarnings;
    }
    void                                    SetUserData( int userData )
    {
        m_userData = userData;
    }
    int                                     GetUserData( void ) const
    {
        return m_userData;
    }
    void                                    SetScalingMode( TScalingMode mode );
    TScalingMode                            GetScalingMode( void ) const
    {
        return m_scalingMode;
    }
    bool                                    SupportsDifferentScalingModes( void ) const
    {
        return m_boSupportsDifferentScalingModes;
    }
private:
    //-----------------------------------------------------------------------------
    enum
    //-----------------------------------------------------------------------------
    {
        IMAGE_MODIFICATIONS_Y_OFFSET = 20,
        PERFORMANCE_WARNINGS_Y_OFFSET = 40,
        SKIPPED_IMAGE_MESSAGE_Y_OFFSET = 60,
        INFO_Y_OFFSET = 80
    };
    //-----------------------------------------------------------------------------
    enum TZoomIncrementMode
    //-----------------------------------------------------------------------------
    {
        zimMultiply,
        zimDivide,
        zimFixedValue
    };
    //-----------------------------------------------------------------------------
    // IDs for the controls and the menu commands
    enum TMenuItem
    //-----------------------------------------------------------------------------
    {
        miPopUpFitToScreen = 1,
        miPopUpOneToOneDisplay,
        miPopUpFullScreen,
        miPopUpScalerMode_NearestNeighbour,
        miPopUpScalerMode_Linear,
        miPopUpScalerMode_Cubic,
        miPopUpSetShiftValue,
        miPopUpShowRequestInfoOverlay,
        miPopUpSelectRequestInfoOverlayColor,
        miPopUpShowPerformanceWarnings,
        miPopUpShowImageModificationsWarning
    };
    ImageCanvasImpl*                        m_pImpl;
    AOIContainer                            m_aoiContainer;
    bool                                    m_boDoubleClickDisabledAndPopupMenuPruned;
    bool                                    m_boHandleMouseAndKeyboardEvents;
    bool                                    m_boRefreshInProgress;
    bool                                    m_boSupportsFullScreenMode;
    bool                                    m_boSupportsDifferentScalingModes;
    const PlotCanvasImageAnalysis*          m_pActiveAnalysisPlot;
    bool                                    m_boScaleToClientSize;
    bool                                    m_boShowInfoOverlay;
    wxColour                                m_InfoOverlayColor;
    std::vector<wxString>                   m_infoStringsOverlay;
    bool                                    m_boShowImageModificationWarning;
    bool                                    m_boShowPerformanceWarnings;
    wxWindow*                               m_pApp;
    TScalingMode                            m_scalingMode;
    size_t                                  m_skippedImages;
    size_t                                  m_skippedPaintEvents;
    const mvIMPACT::acquire::ImageBuffer*   m_pIB;
    double                                  m_currentZoomFactor;
    double                                  m_zoomFactor_Max;
    static const double                     s_zoomFactor_Min;
    wxPoint                                 m_lastLeftMouseDownPos;
    wxPoint                                 m_lastViewStart;
    wxPoint                                 m_lastRightMouseDownPos;
    wxPoint                                 m_lastRightMouseDownPosRaw;
    AOI                                     m_AOIAtLeftMouseDown;
    AOI                                     m_AOIAtRightMouseDown;
    bool                                    m_boAOIDragInProgress;
    wxPoint                                 m_lastMousePos;
    wxPoint                                 m_lastStartPoint;
    double                                  m_lastScaleFactor;
    ImageCanvas*                            m_pMonitorDisplay;
    AOI*                                    m_pVisiblePartOfImage;
    int                                     m_userData;

    template<typename _Ty> void             AppendYUV411_UYYVYYDataPixelInfo( wxPoint pixel, wxString& pixelInfo, const ImageBuffer* pIB ) const;
    template<typename _Ty> void             AppendYUV422DataPixelInfo( wxPoint pixel, wxString& pixelInfo, const ImageBuffer* pIB ) const;
    template<typename _Ty> void             AppendYUV444DataPixelInfo( wxPoint pixel, wxString& pixelInfo, const ImageBuffer* pIB, const int order[3] ) const;
    template<typename _Ty> void             AppendUYVDataPixelInfo( wxPoint pixel, wxString& pixelInfo, const ImageBuffer* pIB ) const;
    void                                    BlitAOIs( wxPaintDC& dc, double scaleFactor, int bmpXOff, int bmpYOff, int bmpW, int bmpH );
    void                                    BlitInfoStrings( wxPaintDC& dc, double scaleFactor, int bmpScaledViewXOff, int bmpScaledViewVOff, int bmpXOff, int bmpYOff, int bmpW, int bmpH );
    void                                    BlitPerformanceMessages( wxPaintDC& dc, int bmpXOff, int bmpYOff, TImageBufferPixelFormat pixelFormat );
    void                                    ClipAOI( wxRect& rect, bool boForDragging ) const;
    void                                    DeleteAOIs( void );
    void                                    DragImageDisplay( void );
    static int                              GetChannelBitDepth( TImageBufferPixelFormat format );
    wxPoint                                 GetScaledMousePos( int mouseXPos, int mouseYPos ) const;
    void                                    Init( const wxWindow* const pApp );
    void                                    SetMaxZoomFactor( const mvIMPACT::acquire::ImageBuffer* pIB, int oldMaxDim );
    TSaveResult                             StoreImage( const wxImage& img, const wxString& filenameAndPath, const wxString& extension ) const;
    bool                                    SupportsFullScreenMode( void ) const
    {
        return m_boSupportsFullScreenMode;
    }
    void                                    UpdateZoomFactor( TZoomIncrementMode zim, double value );
    void                                    IncreaseShiftValue( void );
    void                                    DecreaseShiftValue( void );
    void                                    SetShiftValue( int value );
    int                                     GetShiftValue( void ) const;
    int                                     GetAppliedShiftValue( void ) const;
    void                                    OnKeyDown( wxKeyEvent& e );
    void                                    OnLeftDblClick( wxMouseEvent& );
    void                                    OnLeftDown( wxMouseEvent& );
    void                                    OnMotion( wxMouseEvent& );
    void                                    OnMouseWheel( wxMouseEvent& );
    void                                    OnPaint( wxPaintEvent& e );
    void                                    OnPopUp_ScalingMode_Changed( wxCommandEvent& e );
    void                                    OnPopUpFitToScreen( wxCommandEvent& e );
    void                                    OnPopUpFullScreen( wxCommandEvent& e );
    void                                    OnPopUpOneToOneDisplay( wxCommandEvent& e );
    void                                    OnPopUpSetShiftValue( wxCommandEvent& e );
    void                                    OnPopUpShowImageModificationsWarning( wxCommandEvent& e );
    void                                    OnPopUpShowPerformanceWarnings( wxCommandEvent& e );
    void                                    OnPopUpShowRequestInfoOverlay( wxCommandEvent& e );
    void                                    OnPopUpSelectRequestInfoOverlayColor( wxCommandEvent& )
    {
        m_InfoOverlayColor = wxGetColourFromUser( this );
    }
    void                                    OnRightDown( wxMouseEvent& );
    void                                    OnRightUp( wxMouseEvent& );
    DECLARE_EVENT_TABLE()
};

#endif // ImageCanvasH
