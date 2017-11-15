//-----------------------------------------------------------------------------
#ifndef DevDataH
#define DevDataH DevDataH
//-----------------------------------------------------------------------------
#include <map>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <vector>
#include <wx/wx.h>

class CaptureThread;
class EventThread;
class ImageCanvas;
class PropTree;

//-----------------------------------------------------------------------------
enum TCaptureSettingUsageMode
//-----------------------------------------------------------------------------
{
    csumManual,
    csumAutomatic
};

//-----------------------------------------------------------------------------
enum TWizardIDs
//-----------------------------------------------------------------------------
{
    wColorCorrection,
    wFileAccessControl,
    wLensControl,
    wLUTControl,
    wNone,
    wQuickSetup,
    wSequencerControl
};

typedef std::map<TWizardIDs, std::set<HOBJ> > WizardFeatureMap;
typedef std::vector<PropTree*> PropTreeVector;
typedef std::map<long, long> SequencerSetToDisplayMap;
typedef std::map<HOBJ, std::vector<ImageCanvas*>::size_type> SettingToDisplayDict;

//-----------------------------------------------------------------------------
struct DeviceData
//-----------------------------------------------------------------------------
{
    DeviceData( PropTree* pDrv, PropTree* pDev ) : pDriverTree( pDrv ), pDeviceTree( pDev ), pFuncInterface( 0 ),
        pStatistics( 0 ), pInfo( 0 ), acquisitionMode(), acquisitionFrameCount(), mvAcquisitionMemoryFrameCount(),
        pCaptureThread( 0 ), boWasLive( false ), lockedRequest( mvIMPACT::acquire::INVALID_ID ),
        featureVsTimePlotPath(), featureVsTimeFeature() {}
    PropTree*                               pDriverTree;
    PropTree*                               pDeviceTree;
    PropTreeVector                          vDriverTrees;
    mvIMPACT::acquire::FunctionInterface*   pFuncInterface;
    mvIMPACT::acquire::Statistics*          pStatistics;
    mvIMPACT::acquire::Info*                pInfo;
    mvIMPACT::acquire::Property             acquisitionMode;
    mvIMPACT::acquire::PropertyI64          acquisitionFrameCount;
    mvIMPACT::acquire::PropertyI64          mvAcquisitionMemoryFrameCount;
    CaptureThread*                          pCaptureThread;
    bool                                    boWasLive;
    int                                     lockedRequest;
    WizardFeatureMap                        supportedWizards;
    SequencerSetToDisplayMap                sequencerSetToDisplayMap;
    SettingToDisplayDict                    settingToDisplayDict;
    wxString                                featureVsTimePlotPath;
    mvIMPACT::acquire::Component            featureVsTimeFeature;
};

//-----------------------------------------------------------------------------
struct VariableValue
//-----------------------------------------------------------------------------
{
    TComponentType type;
    union
    {
        int intRep;
        int64_type int64Rep;
        double doubleRep;
    } value;
    explicit VariableValue() : type( ctPropInt )
    {
        value.intRep = 0;
    }
};

//-----------------------------------------------------------------------------
struct RequestInfoData
//-----------------------------------------------------------------------------
{
    VariableValue plotValue_;
    int64_type frameNr_;
    int exposeTime_us_;
    double gain_dB_;
    int64_type timeStamp_us_;
    HOBJ settingUsed_;
    TRequestResult requestResult_;
    int64_type chunkSequencerSetActive_;
    explicit RequestInfoData() : plotValue_(), frameNr_( 0 ), exposeTime_us_( 0 ), gain_dB_( 0.0 ), timeStamp_us_( 0LL ),
        settingUsed_( INVALID_ID ), requestResult_( rrOK ), chunkSequencerSetActive_( 0LL ) {}
};

//-----------------------------------------------------------------------------
struct RequestData
//-----------------------------------------------------------------------------
{
    mvIMPACT::acquire::ImageBufferDesc      image_;
    mvIMPACT::acquire::TBayerMosaicParity   bayerParity_;
    wxString                                pixelFormat_;
    RequestInfoData                         requestInfo_;
    std::vector<wxString>                   requestInfoStrings_;
    int                                     requestNr_;
    static const wxString                   UNKNOWN_PIXEL_FORMAT_STRING_;
    explicit RequestData();
};

#endif // DevDataH
