//-----------------------------------------------------------------------------
#ifndef ImageAnalysisPlotControlsH
#define ImageAnalysisPlotControlsH ImageAnalysisPlotControlsH
//-----------------------------------------------------------------------------

class PlotCanvasImageAnalysis;
class wxCheckBox;
class wxConfigBase;
class wxComboBox;
class wxNotebook;
class wxSpinCtrl;

//-----------------------------------------------------------------------------
struct ImageAnalysisPlotControls
//-----------------------------------------------------------------------------
{
    wxCheckBox*                 m_pCBAOIFullMode;
    wxCheckBox*                 m_pCBProcessBayerParity;
    wxSpinCtrl*                 m_pSCAOIx;
    wxSpinCtrl*                 m_pSCAOIy;
    wxSpinCtrl*                 m_pSCAOIw;
    wxSpinCtrl*                 m_pSCAOIh;
    wxComboBox*                 m_pCoBPlotSelection;
    wxSpinCtrl*                 m_pSCHistoryDepth;
    wxSpinCtrl*                 m_pSCDrawStart_percent;
    wxSpinCtrl*                 m_pSCDrawWindow_percent;
    wxSpinCtrl*                 m_pSCDrawStepWidth;
    wxSpinCtrl*                 m_pSCUpdateSpeed;
    wxSpinCtrl*                 m_pSCGridStepsX;
    wxSpinCtrl*                 m_pSCGridStepsY;
    wxNotebook*                 m_pNBDisplayMethod;
    PlotCanvasImageAnalysis*    m_pPlotCanvas;
    void                        Load( wxConfigBase* pConfig );
    void                        Save( wxConfigBase* pConfig );
    void                        UpdateControls( void );
    explicit ImageAnalysisPlotControls() : m_pCBAOIFullMode( 0 ), m_pCBProcessBayerParity( 0 ),
        m_pSCAOIx( 0 ), m_pSCAOIy( 0 ), m_pSCAOIw( 0 ), m_pSCAOIh( 0 ), m_pCoBPlotSelection( 0 ), m_pSCHistoryDepth( 0 ),
        m_pSCDrawStart_percent( 0 ), m_pSCDrawWindow_percent( 0 ), m_pSCDrawStepWidth( 0 ),
        m_pSCUpdateSpeed( 0 ), m_pSCGridStepsX( 0 ), m_pSCGridStepsY( 0 ), m_pNBDisplayMethod( 0 ), m_pPlotCanvas( 0 ) {}
};

#endif // ImageAnalysisPlotControlsH
