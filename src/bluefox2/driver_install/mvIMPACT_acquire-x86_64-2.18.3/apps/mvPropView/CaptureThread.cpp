//-----------------------------------------------------------------------------
#include <algorithm>
#include <apps/Common/wxAbstraction.h>
#include "CaptureThread.h"
#include <common/STLHelper.h>
#include <limits>
#include "PropViewFrame.h"
#include <wx/event.h>
#include <wx/utils.h>

using namespace std;
using namespace mvIMPACT::acquire;

//-----------------------------------------------------------------------------
/// This is a temporary fix. It should be removed as soon as possible.
/// See http://trac.wxwidgets.org/ticket/17579 for details.
/// The code has been taken from the wxWidgets 3.0.2 sources.
static void WakeUpIdle( void )
//-----------------------------------------------------------------------------
{
#ifdef _WIN32
#   if wxCHECK_VERSION(3, 1, 1)
#       error "The bug mentioned above should be fixed by now! If so: Clean up the code! If not: Adjust version check here!"
#   elif wxCHECK_VERSION(3, 1, 0) // This is a bug in this particular version
    wxWindow* const topWindow = wxTheApp->GetTopWindow();
    if( topWindow )
    {
        HWND hwndTop = ( HWND )( topWindow->GetHWND() );
        MSG msg;
        if ( !::PeekMessage( &msg, hwndTop, 0, 1, PM_NOREMOVE ) ||
             ::PeekMessage( &msg, hwndTop, 1, 1, PM_NOREMOVE ) )
        {
            ::PostMessage( hwndTop, WM_NULL, 0, 0 );
        }
    }
#       if wxUSE_THREADS
    else
    {
        wxWakeUpMainThread();
    }
#       endif // wxUSE_THREADS
#   endif // #if wxCHECK_VERSION(3, 1, 1)
#endif // #ifdef _WIN32
}

//-----------------------------------------------------------------------------
void QueueEvent( wxEvtHandler* pEventHandler, wxEvent* pEvent )
//-----------------------------------------------------------------------------
{
    ::wxQueueEvent( pEventHandler, pEvent );
    WakeUpIdle();
}

//=============================================================================
//================= Implementation TimerLite ==================================
//=============================================================================
//-----------------------------------------------------------------------------
class TimerLite
//-----------------------------------------------------------------------------
{
    clock_t m_endClocks;
    clock_t Delay2Clocks( unsigned long nextDelay_ms ) const
    {
        return ( nextDelay_ms * CLOCKS_PER_SEC ) / 1000;
    }
public:
    explicit TimerLite() : m_endClocks( 0 ) {}
    explicit TimerLite( unsigned long nextDelay_ms )
    {
        Restart( nextDelay_ms );
    }
    void Restart( unsigned long nextDelay_ms )
    {
        m_endClocks = clock() + Delay2Clocks( nextDelay_ms );
    }
    bool IsExpired( unsigned long nextDelay_ms )
    {
        clock_t curClocks = clock();
        if( curClocks > m_endClocks )
        {
            m_endClocks = curClocks + Delay2Clocks( nextDelay_ms );
            return true;
        }
        return false;
    }
};

//=============================================================================
//================= Implementation CaptureThread ==============================
//=============================================================================

unsigned int CaptureThread::m_activeInstance = 0;
unsigned int CaptureThread::m_instanceCnt = 0;

//-----------------------------------------------------------------------------
CaptureThread::CaptureThread( FunctionInterface* pFuncInterface, Device* pDev, wxWindow* pParentWindow, unsigned int pendingImageQueueDepth ) :
    wxThread( wxTHREAD_JOINABLE ), m_boLive( false ), m_boRecord( false ), m_boContinuousRecording( false ), m_boForwardIncompleteFrames( false ),
    m_currentRequest( INVALID_ID ), m_instanceNr( m_instanceCnt++ ),
    m_lastErrorMessage(), m_currentPlotInfoPath(), m_nextPendingRequest( INVALID_ID ),
    m_pendingRequestsForDisplayMax( static_cast<list<int>::size_type>( ( pendingImageQueueDepth < 1 ) ? 1 : pendingImageQueueDepth ) ), m_pendingRequestsForDisplay(),
    m_parentWindowID( pParentWindow->GetId() ), m_pDev( pDev ), m_pFuncInterface( pFuncInterface ),
    m_ImageRequestControl( pDev ), m_SystemSettings( pDev ), m_ImageProcessingMode( ipmDefault ), m_SettingIndex( 0 ), m_pParentWindowEventHandler( pParentWindow->GetEventHandler() ),
    m_recordedSequence(), m_recordedSequenceMemoryConsumption( 0 ), m_currentSequenceIndexToDisplay( 0 ), m_currentCountToRecord( 0 ), m_totalCountToRecord( 0 ),
    m_maxNumberToRequest( 0 ), m_numberRequested( 0 ), m_numberReturned( 0 ),
    m_skippedImageCount( 0 ), m_capturedImagesCount( 0 ), m_capturedImagesSentToDisplayCount( 0 ),
    m_percentageOfImagesSentToDisplay( 100. ), m_captureQueueDepth( 0 ), m_currentlyActiveRequests( 0 )
//-----------------------------------------------------------------------------
{
#if MULTI_SETTING_HACK
    m_CurrentSetting = 0;
    m_CaptureSettingUsageMode = csumManual;
#endif // #if MULTI_SETTING_HACK
}

//-----------------------------------------------------------------------------
CaptureThread::~CaptureThread()
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    UnlockPendingRequests();
    FreeSequence();
}

//-----------------------------------------------------------------------------
void CaptureThread::CheckedImageRequestUnlock( const int requestNr ) const
//-----------------------------------------------------------------------------
{
    if( m_pFuncInterface->isRequestNrValid( requestNr ) && ( m_pFuncInterface->getRequest( requestNr )->requestState.read() == rsReady ) )
    {
        m_pFuncInterface->imageRequestUnlock( requestNr );
    }
}

//-----------------------------------------------------------------------------
bool CaptureThread::CheckSequenceRestart( bool boInformUserOnRecordStart ) const
//-----------------------------------------------------------------------------
{
    if( !boInformUserOnRecordStart )
    {
        return true;
    }

    bool result = false;
    wxMessageDialog dlg( NULL,
                         wxString::Format( wxT( "Restarts the image recording. All current recorded images (%lu) will be lost. Check 'Settings -> Silent recording' to get rid of this and other questions." ), static_cast<unsigned long int>( GetRecordedSequenceSize() ) ),
                         wxT( "Warning" ),
                         wxNO_DEFAULT | wxYES_NO | wxICON_INFORMATION );

    if( dlg.ShowModal() == wxID_YES )
    {
        result = true;
    }

    return result;
}

//-----------------------------------------------------------------------------
void CaptureThread::CollectBufferInfo( ComponentIterator it, ComponentIterator itImageProcessingResults, vector<wxString>& infoStrings, const wxString& path /* = wxT("") */ ) const
//-----------------------------------------------------------------------------
{
    while( it.isValid() )
    {
        if( it.isVisible() )
        {
            if( it.isProp() )
            {
                Property prop( it );
                const unsigned int valCount = prop.valCount();
                if( valCount > 1 )
                {
                    for( int i = 0; i < static_cast<int>( valCount ); i++ )
                    {
                        infoStrings.push_back( wxString::Format( wxT( "%s%s[%d]: %s" ), path.c_str(), ConvertedString( it.name() ).c_str(), i, ConvertedString( prop.readS( i ) ).c_str() ) );
                    }
                }
                else
                {
                    infoStrings.push_back( wxString::Format( wxT( "%s%s: %s" ), path.c_str(), ConvertedString( it.name() ).c_str(), ConvertedString( prop.readS() ).c_str() ) );
                }
            }
            else if( it.isList() )
            {
                if( it.firstChild().hObj() == itImageProcessingResults.hObj() )
                {
                    CollectBufferInfo_ImageProcessingResults( it.firstChild(), infoStrings );
                }
                else
                {
                    CollectBufferInfo( it.firstChild(), itImageProcessingResults, infoStrings, path + ConvertedString( it.name() ) + wxT( "/" ) );
                }
            }
        }
        ++it;
    }
}

//-----------------------------------------------------------------------------
void CaptureThread::CollectBufferInfo_ImageProcessingResults( ComponentIterator it, vector<wxString>& infoStrings ) const
//-----------------------------------------------------------------------------
{
    wxString data;
    while( it.isValid() )
    {
        if( it.isVisible() && it.isProp() )
        {
            PropertyIImageProcessingResult prop( it );
            const TImageProcessingResult ipr = prop.read();
            switch( ipr )
            {
            case iprApplied:
            case iprFailure:
                if( !data.IsEmpty() )
                {
                    data.Append( wxT( " -> " ) );
                }
                data.Append( ConvertedString( prop.name() ) );
                if( ipr == iprFailure )
                {
                    data.Append( wxT( "(FAILED)" ) );
                }
                break;
            default:
                break;
            }
        }
        ++it;
    }
    if( !data.IsEmpty() )
    {
        infoStrings.push_back( wxString::Format( wxT( "Image Processing Applied: %s" ), data.c_str() ) );
    }
}

//-----------------------------------------------------------------------------
size_t CaptureThread::DisplaySequenceRequest( size_t nr )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    const RequestContainer::size_type sequenceSize = GetRecordedSequenceSize();
    m_currentSequenceIndexToDisplay = ( nr < sequenceSize ) ? nr : ( ( sequenceSize > 0 ) ? sequenceSize - 1 : 0 );
    if( sequenceSize > 0 )
    {
        int req = m_recordedSequence[m_currentSequenceIndexToDisplay].first;
        if( req != m_currentRequest )
        {
            if( m_pendingRequestsForDisplayMax == 1 )
            {
                if( m_nextPendingRequest != INVALID_ID )
                {
                    UnlockRequest( m_nextPendingRequest );
                }
                m_nextPendingRequest = req;
            }
            else
            {
                while( m_pendingRequestsForDisplay.size() >= m_pendingRequestsForDisplayMax )
                {
                    UnlockRequest( m_pendingRequestsForDisplay.front() );
                    m_pendingRequestsForDisplay.pop_front();
                }
                m_pendingRequestsForDisplay.push_back( req );
            }
        }
    }

    return m_currentSequenceIndexToDisplay;
}

//-----------------------------------------------------------------------------
void* CaptureThread::Entry( void )
//-----------------------------------------------------------------------------
{
    const unsigned long waitBeforeBreak = 60;
    TimerLite forceBreakTimer( waitBeforeBreak );
    while( !TestDestroy() )
    {
        int requestNr;
        {
            wxCriticalSectionLocker locker( m_critSect );
            if( m_boLive )
            {
                RequestImages();
            }
        }
        if( ( requestNr = m_pFuncInterface->imageRequestWaitFor( 0, 0 ) ) < 0 )
        {
            if( ( requestNr = m_pFuncInterface->imageRequestWaitFor( 200, 0 ) ) >= 0 )
            {
                forceBreakTimer.Restart( waitBeforeBreak );
            }
        }
        else if( forceBreakTimer.IsExpired( waitBeforeBreak ) )
        {
            Sleep( 10 );
        }
        if( requestNr >= 0 )
        {
            wxCriticalSectionLocker locker( m_critSect );
            if( m_pFuncInterface->isRequestNrValid( requestNr ) )
            {
                const Request* pReq = m_pFuncInterface->getRequest( requestNr );
                if( pReq )
                {
                    if( m_activeInstance == m_instanceNr )
                    {
                        {
                            wxCriticalSectionLocker locker_config( m_critSectConfig );
                            --m_currentlyActiveRequests;
                            ++m_numberReturned;
                        }
                        TRequestResult rr = pReq->requestResult.read();
                        HOBJ hSettingUsed = ( pReq->infoSettingUsed.isValid() ) ? static_cast<HOBJ>( pReq->infoSettingUsed.read() ) : static_cast<HOBJ>( INVALID_ID );
#if MULTI_SETTING_HACK
                        if( hSettingUsed == INVALID_ID )
                        {
                            // deal with drivers that do not support the 'SettingUsed' property
                            if( m_PendingRequests[0] > 0 )
                            {
                                m_PendingRequests[0] = m_PendingRequests[0] - 1;
                            }
                        }
                        else
                        {
                            for( unsigned int i = 0; i < m_SettingCount; i++ )
                            {
                                if( ( m_Settings[i].second == hSettingUsed ) && ( m_PendingRequests[i] > 0 ) )
                                {
                                    if( rr == rrOK )
                                    {
                                        // Support for SVSEM 'TransferFrameType: Increasing':
                                        // one request will be returned multiple times with an increasing count of valid lines (LineCounter),
                                        // but with the same 'FameNr'. So only one request per 'FrameNr' is necessary.
                                        if( pReq->infoFrameNr.read() != m_LastPendingRequestFrameNr[i] )
                                        {
                                            m_PendingRequests[i] = m_PendingRequests[i] - 1;
                                            m_LastPendingRequestFrameNr[i] = pReq->infoFrameNr.read();
                                        }
                                        // Support for SVSEM 'TransferFrameType: Block':
                                        else if( pReq->imageOffsetY.read() != 0 )
                                        {
                                            m_PendingRequests[i] = m_PendingRequests[i] - 1;
                                        }
                                    }
                                    else
                                    {
                                        m_PendingRequests[i] = m_PendingRequests[i] - 1;
                                    }
                                    break;
                                }
                            }
                        }
#endif // #if MULTI_SETTING_HACK
                        wxCommandEvent* pInfoEvent = new wxCommandEvent( imageInfoEvent, m_parentWindowID );
                        RequestInfoData* pReqInfo = new RequestInfoData();
                        GetRequestInfoData( pReqInfo, pReq, hSettingUsed );
                        pInfoEvent->SetClientData( pReqInfo );
                        QueueEvent( pInfoEvent );
                        const bool boRequestProcessingSkipped = pReq->hasProcessingBeenSkipped();
                        if( ( rr != rrRequestAborted ) && !( rr & rrUnprocessibleRequest ) )
                        {
                            ++m_capturedImagesCount;
                        }
                        const unsigned int requestCount = m_pFuncInterface->requestCount();
                        if( ( ( rr == rrOK ) || ( ( rr == rrFrameIncomplete ) && m_boForwardIncompleteFrames ) ) &&
                            !boRequestProcessingSkipped )
                        {
                            if( m_boRecord )
                            {
                                RequestDataEntry data;
                                data.first = requestNr;
                                if( m_totalCountToRecord > requestCount )
                                {
                                    data.second = new RequestData();
                                    GetRequestData( *data.second, requestNr, true );
                                    InternalGetImageInfo( data.second->requestInfoStrings_, pReq );
                                    m_recordedSequenceMemoryConsumption += static_cast<size_t>( data.second->image_.getBuffer()->iSize );
                                }
                                m_recordedSequence.push_back( data );
                            }
                            if( m_pendingRequestsForDisplayMax == 1 )
                            {
                                if( m_nextPendingRequest != INVALID_ID )
                                {
                                    const int settingUsed = GetSettingUsedForRequest( m_pFuncInterface->getRequest( m_nextPendingRequest ) );
                                    UnlockRequest( m_nextPendingRequest );
                                    SendImageSkippedEvent( settingUsed );
                                }
                                else
                                {
                                    SendImageReadyEvent();
                                }
                                m_nextPendingRequest = requestNr;
                            }
                            else
                            {
                                if( m_pendingRequestsForDisplay.size() >= m_pendingRequestsForDisplayMax )
                                {
                                    while( m_pendingRequestsForDisplay.size() >= m_pendingRequestsForDisplayMax )
                                    {
                                        const int settingUsed = GetSettingUsedForRequest( m_pFuncInterface->getRequest( m_pendingRequestsForDisplay.front() ) );
                                        UnlockRequest( m_pendingRequestsForDisplay.front() );
                                        m_pendingRequestsForDisplay.pop_front();
                                        SendImageSkippedEvent( settingUsed );
                                    }
                                }
                                else
                                {
                                    SendImageReadyEvent();
                                }
                                m_pendingRequestsForDisplay.push_back( requestNr );
                            }
                        }
                        else
                        {
                            if( rr == rrTimeout )
                            {
                                QueueEvent( new wxCommandEvent( imageTimeoutEvent, m_parentWindowID ) );
                            }
                            if( rr & rrUnprocessibleRequest )
                            {
                                m_lastErrorMessage = ConvertedString( pReq->requestResult.readS() );
                            }
                            const int settingUsed = boRequestProcessingSkipped ? GetSettingUsedForRequest( m_pFuncInterface->getRequest( requestNr ) ) : INVALID_ID;
                            m_pFuncInterface->imageRequestUnlock( requestNr );
                            if( boRequestProcessingSkipped )
                            {
                                SendImageSkippedEvent( settingUsed );
                            }

                            if( GetLiveMode() )
                            {
                                // Live mode is stopped depending on request result
                                if( rr & rrUnprocessibleRequest )
                                {
                                    InternalSetLiveMode( false );
                                    wxCommandEvent* pE = new wxCommandEvent( liveModeAborted, m_parentWindowID );
                                    pE->SetString( m_lastErrorMessage.c_str() ); // a deep copy of the string must be created here! See documentation of 'wxQueueEvent' for details.
                                    QueueEvent( pE );
                                    if( m_currentCountToRecord )
                                    {
                                        m_currentCountToRecord = 1;
                                    }
                                }
                            }
                        }

                        bool boMultiFrameDone = false;
                        {
                            wxCriticalSectionLocker locker_config( m_critSectConfig );
                            if( m_boLive && ( m_maxNumberToRequest > 0 ) && ( m_maxNumberToRequest == m_numberReturned ) )
                            {
                                boMultiFrameDone = true;
                            }
                        }
                        if( boMultiFrameDone )
                        {
                            InternalSetLiveMode( false );
                            wxCommandEvent* pE = new wxCommandEvent( sequenceReadyEvent, m_parentWindowID );
                            pE->SetInt( eosrMultiFrameDone );
                            QueueEvent( pE );
                        }

                        if( m_boContinuousRecording )
                        {
                            wxCriticalSectionLocker locker_config( m_critSectConfig );
                            if( m_totalCountToRecord > requestCount )
                            {
                                if( ( GetRecordedSequenceSize() > static_cast<RequestContainer::size_type>( m_totalCountToRecord ) ) || MustFreeMemory() )
                                {
                                    FreeLastRequestFromSequence();
                                }
                            }
                            else
                            {
                                RequestContainer::size_type maxSequenceSize = static_cast<RequestContainer::size_type>( ( m_captureQueueDepth > 0 ) ? requestCount - m_captureQueueDepth : requestCount / 2 );
                                if( GetRecordedSequenceSize() > maxSequenceSize )
                                {
                                    if( m_pendingRequestsForDisplayMax == 1 )
                                    {
                                        FreeAndUnlockLastRequestFromSequence( m_currentRequest );
                                    }
                                    else
                                    {
                                        int reqNrFreed = FreeAndUnlockLastRequestFromSequence( m_pendingRequestsForDisplay.empty() ? INVALID_ID : m_pendingRequestsForDisplay.back() );
                                        if( reqNrFreed != INVALID_ID )
                                        {
                                            list<int>::iterator it;
                                            while( ( it = find( m_pendingRequestsForDisplay.begin(), m_pendingRequestsForDisplay.end(), reqNrFreed ) ) != m_pendingRequestsForDisplay.end() )
                                            {
                                                m_pendingRequestsForDisplay.erase( it );
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else if( m_currentCountToRecord && m_boRecord )
                        {
                            --m_currentCountToRecord;
                            // only stop early when requests are copied! In default mode assume there is enough memory for all requests objects!
                            const bool boMustFreeMemory = ( m_totalCountToRecord > requestCount ) && MustFreeMemory();
                            if( ( m_currentCountToRecord == 0 ) || boMustFreeMemory )
                            {
                                InternalSetLiveMode( false );
                                wxCommandEvent* pE = new wxCommandEvent( sequenceReadyEvent, m_parentWindowID );
                                pE->SetInt( boMustFreeMemory ? eosrRecordingDone_OutOfRAM : eosrRecordingDone );
                                QueueEvent( pE );
                            }
                        }
                    }
                    else
                    {
                        m_pFuncInterface->imageRequestUnlock( requestNr );
                    }
                }
            }
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------
int CaptureThread::FreeAndUnlockLastRequestFromSequence( int requestNotToUnlock )
//-----------------------------------------------------------------------------
{
    RequestDataEntry entry = m_recordedSequence.front();
    bool boMustUnlock = entry.first != requestNotToUnlock;
    if( boMustUnlock )
    {
        CheckedImageRequestUnlock( entry.first );
    }
    FreeLastRequestFromSequence( entry );
    return boMustUnlock ? entry.first : INVALID_ID;
}

//-----------------------------------------------------------------------------
void CaptureThread::FreeLastRequestFromSequence( void )
//-----------------------------------------------------------------------------
{
    RequestDataEntry entry = m_recordedSequence.front();
    FreeLastRequestFromSequence( entry );
}

//-----------------------------------------------------------------------------
void CaptureThread::FreeLastRequestFromSequence( RequestDataEntry& entry )
//-----------------------------------------------------------------------------
{
    m_recordedSequence.pop_front();
    if( entry.second )
    {
        DecreaseRecordedSequenceMemoryConsumption( static_cast<size_t>( entry.second->image_.getBuffer()->iSize ) );
    }
    DeleteSecond( entry );
}

//-----------------------------------------------------------------------------
void CaptureThread::FreeSequence( int requestNotToUnlock /* = INVALID_ID */ )
//-----------------------------------------------------------------------------
{
    while( !m_recordedSequence.empty() )
    {
        FreeAndUnlockLastRequestFromSequence( requestNotToUnlock );
    }
    m_currentSequenceIndexToDisplay = 0;
}

//-----------------------------------------------------------------------------
void CaptureThread::GetImageData( RequestData& requestData, const bool boFromRecordedStream ) const
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    if( ( m_pendingRequestsForDisplayMax == 1 ) && ( m_nextPendingRequest != INVALID_ID ) )
    {
        m_currentRequest = m_nextPendingRequest;
        m_nextPendingRequest = INVALID_ID;
    }
    else if( ( m_pendingRequestsForDisplayMax > 1 ) && !m_pendingRequestsForDisplay.empty() )
    {
        m_currentRequest = m_pendingRequestsForDisplay.front();
        m_pendingRequestsForDisplay.pop_front();
    }

    if( boFromRecordedStream && ( m_currentSequenceIndexToDisplay < m_recordedSequence.size() ) && m_recordedSequence[m_currentSequenceIndexToDisplay].second )
    {
        requestData = *m_recordedSequence[m_currentSequenceIndexToDisplay].second;
    }
    else
    {
        GetRequestData( requestData, m_currentRequest );
    }
}

//-----------------------------------------------------------------------------
void CaptureThread::InternalGetImageInfo( vector<wxString>& infoStrings, const Request* pRequest ) const
//-----------------------------------------------------------------------------
{
    if( pRequest )
    {
        ComponentIterator it( pRequest->getInfoIterator() );
        CollectBufferInfo( it, pRequest->getImageProcessingResultsIterator(), infoStrings );
    }
}

//-----------------------------------------------------------------------------
void CaptureThread::GetImageInfo( vector<wxString>& infoStrings ) const
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    if( m_currentRequest != INVALID_ID )
    {
        InternalGetImageInfo( infoStrings, m_pFuncInterface->getRequest( m_currentRequest ) );
    }
}

//-----------------------------------------------------------------------------
double CaptureThread::GetPercentageOfImagesSentToDisplay( void ) const
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    const double result = ( m_capturedImagesCount == 0 ) ? 100. : 100. * ( static_cast<double>( m_capturedImagesSentToDisplayCount ) / static_cast<double>( m_capturedImagesCount ) );
    m_capturedImagesSentToDisplayCount = 0;
    m_capturedImagesCount = 0;
    m_percentageOfImagesSentToDisplay = 0.9 * m_percentageOfImagesSentToDisplay + 0.1 * result;
    return m_percentageOfImagesSentToDisplay;
}

//-----------------------------------------------------------------------------
void CaptureThread::GetRequestData( RequestData& requestData, const int requestNr, const bool boDeepCopy /* = false */ ) const
//-----------------------------------------------------------------------------
{
    requestData.requestNr_ = requestNr;
    if( requestNr != INVALID_ID )
    {
        const Request* pReq = m_pFuncInterface->getRequest( requestNr );
        if( pReq )
        {
            ImageBufferDesc nbd( boDeepCopy ? pReq->getImageBufferDesc().clone() : pReq->getImageBufferDesc() );
            requestData.image_ = nbd;
            //requestData.image_ = ImageBufferDesc(pReq->getImageBufferDesc()); // doesn't work with GCC...
            GetRequestInfoData( &requestData.requestInfo_, pReq, ( pReq->infoSettingUsed.isValid() ) ? static_cast<HOBJ>( pReq->infoSettingUsed.read() ) : static_cast<HOBJ>( INVALID_ID ) );
            requestData.bayerParity_ = pReq->imageBayerMosaicParity.read();
            if( requestData.bayerParity_ != bmpUndefined )
            {
                requestData.pixelFormat_ = wxString::Format( wxT( "%s(BayerPattern=%s)" ), ConvertedString( pReq->imagePixelFormat.readS() ).c_str(), ConvertedString( pReq->imageBayerMosaicParity.readS() ).c_str() );
            }
            else
            {
                requestData.pixelFormat_ = ConvertedString( pReq->imagePixelFormat.readS() );
            }
        }
        else
        {
            requestData.requestInfo_.requestResult_ = rrError;
        }
    }
    else
    {
        requestData = RequestData();
    }
}

//-----------------------------------------------------------------------------
void CaptureThread::GetRequestInfoData( RequestInfoData* pReqInfo, const Request* pReq, HOBJ settingUsed ) const
//-----------------------------------------------------------------------------
{
    if( !m_currentPlotInfoPath.empty() )
    {
        Property p( pReq->getComponentLocator().findComponent( m_currentPlotInfoPath ) );
        pReqInfo->plotValue_.type = p.type();
        switch( p.type() )
        {
        case ctPropInt:
            pReqInfo->plotValue_.value.intRep = PropertyI( p ).read();
            break;
        case ctPropInt64:
            pReqInfo->plotValue_.value.int64Rep = PropertyI64( p ).read();
            break;
        case ctPropFloat:
            pReqInfo->plotValue_.value.doubleRep = PropertyF( p ).read();
            break;
        default:
            break;
        }
    }
    pReqInfo->exposeTime_us_ = pReq->infoExposeTime_us.read();
    pReqInfo->frameNr_ = pReq->infoFrameNr.read();
    pReqInfo->gain_dB_ = pReq->infoGain_dB.read();
    pReqInfo->settingUsed_ = settingUsed;
    pReqInfo->timeStamp_us_ = pReq->infoTimeStamp_us.read();
    pReqInfo->requestResult_ = pReq->requestResult.read();
    pReqInfo->chunkSequencerSetActive_ = ( pReq->chunkSequencerSetActive.isValid() && pReq->chunkSequencerSetActive.isVisible() ) ? pReq->chunkSequencerSetActive.read() : 0LL;
}

//-----------------------------------------------------------------------------
size_t CaptureThread::GetSkippedImageCount( void )
//-----------------------------------------------------------------------------
{
    size_t res = m_skippedImageCount;
    m_skippedImageCount = 0;
    return res;
}

//-----------------------------------------------------------------------------
bool CaptureThread::InternalSetLiveMode( bool boOn, bool boInformUserOnRecordStart /* = true */ )
//-----------------------------------------------------------------------------
{
    m_capturedImagesCount = 0;
    m_capturedImagesSentToDisplayCount = 0;
    m_percentageOfImagesSentToDisplay = 100.;
    if( boOn )
    {
        UnlockPendingRequests();
        if( m_boRecord )
        {
            if( m_recordedSequence.empty() || CheckSequenceRestart( boInformUserOnRecordStart ) )
            {
                InternalSetRecordMode( true, true );
            }
        }
        else
        {
            FreeSequence();
        }
        if( m_SystemSettings.imageProcessingMode.isValid() )
        {
            m_SystemSettings.imageProcessingMode.write( m_boRecord ? ipmDefault : m_ImageProcessingMode );
        }
        m_skippedImageCount = 0;
        int framesRequested = 0;
        int result = RequestImages( &framesRequested );
        if( ( result == DMR_NO_ERROR ) ||
            ( ( result == DEV_NO_FREE_REQUEST_AVAILABLE ) && ( framesRequested > 0 ) ) )
        {
            m_boLive = boOn;
            result = m_pFuncInterface->acquisitionStart();
            switch( result )
            {
            case DMR_NO_ERROR:
            case DMR_FEATURE_NOT_AVAILABLE:
                break;
            case DMR_ACQUISITION_ENGINE_BUSY:
            default:
                wxMessageDialog dlg( NULL, wxString::Format( wxT( "%s(%d): Live not started as calling 'AcquisitionStart' did fail(Reason: %s)! More information can be found in the *.log-file or the debug output.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), wxT( "Error" ), wxOK | wxICON_WARNING );
                dlg.ShowModal();
                break;
            }
        }
        else if( result != DEV_NO_FREE_REQUEST_AVAILABLE )
        {
            wxMessageDialog dlg( NULL, wxString::Format( wxT( "%s(%d): Live not started as queuing requests did fail(Reason: %s)! More information can be found in the *.log-file or the debug output.\n" ), ConvertedString( __FUNCTION__ ).c_str(), __LINE__, ConvertedString( ImpactAcquireException::getErrorCodeAsString( result ) ).c_str() ), wxT( "Error" ), wxOK | wxICON_WARNING );
            dlg.ShowModal();
        }
    }
    else
    {
        m_boLive = false;
        m_pFuncInterface->acquisitionStop();
        RequestReset();
    }

    return m_boLive;
}

//-----------------------------------------------------------------------------
bool CaptureThread::InternalSetRecordMode( bool boOn, bool boUnlockCurrentRequest )
//-----------------------------------------------------------------------------
{
    if( boOn )
    {
        m_currentSequenceIndexToDisplay = 0;
        RequestReset();
        UnlockPendingRequests();
        FreeSequence( boUnlockCurrentRequest ? INVALID_ID : m_currentRequest );
        m_currentCountToRecord = ( m_totalCountToRecord == 0 ) ? m_pFuncInterface->requestCount() : m_totalCountToRecord;
    }
    m_boRecord = boOn;
    return m_boRecord;
}

//-----------------------------------------------------------------------------
int CaptureThread::RequestSingle( bool boInformUserOnRecordStart )
//-----------------------------------------------------------------------------
{
    if( !m_boRecord )
    {
        FreeSequence( m_currentRequest );
    }

#if MULTI_SETTING_HACK
    if( m_CaptureSettingUsageMode == csumAutomatic )
    {
        m_ImageRequestControl.setting.write( m_Settings[m_CurrentSetting].second );
        m_CurrentSetting = static_cast<unsigned int>( ( m_CurrentSetting + 1 ) % m_SettingCount );
    }
#endif // #if MULTI_SETTING_HACK

    int result = m_pFuncInterface->imageRequestSingle( &m_ImageRequestControl );
    if( m_boRecord && ( result < 0 ) && CheckSequenceRestart( boInformUserOnRecordStart ) )
    {
        SetRecordMode( true );
        result = RequestSingle( boInformUserOnRecordStart );
    }
    else if( result == DMR_NO_ERROR )
    {
        if( ( result = m_pFuncInterface->acquisitionStart() ) == DMR_FEATURE_NOT_AVAILABLE )
        {
            // do not report this as an error. Not every device/driver combination will support this
            result = DMR_NO_ERROR;
        }
    }
    return result;
}

//-----------------------------------------------------------------------------
int CaptureThread::RequestReset( void )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker_config( m_critSectConfig );
    int result = m_pFuncInterface->imageRequestReset( 0, 0 );
    m_currentlyActiveRequests = 0;
    for( unsigned int i = 0; i < m_SettingCount; i++ )
    {
#if MULTI_SETTING_HACK
        m_LastPendingRequestFrameNr[i] = numeric_limits<int64_type>::max();
        m_PendingRequests[i] = 0;
#endif // #if MULTI_SETTING_HACK
    }
    return result;
}

//-----------------------------------------------------------------------------
bool CaptureThread::MustFreeMemory( void ) const
//-----------------------------------------------------------------------------
{
    if( sizeof( void* ) == 4 )
    {
        return m_recordedSequenceMemoryConsumption > static_cast<size_t>( MAX_MEMORY_FOR_RECORDING_ON_32_BIT );
    }

    const wxMemorySize freeMemory = ::wxGetFreeMemory();
    static const wxMemorySize s_512MB = 512 * 1024 * 1024;
    return ( ( freeMemory >= 0 ) && ( freeMemory < s_512MB ) );
}

//-----------------------------------------------------------------------------
int CaptureThread::RequestImages( int* pFramesRequested /* = 0 */ )
//-----------------------------------------------------------------------------
{
    int requestCnt = 0;
    int requestResult = DMR_NO_ERROR;
    do
    {
#if MULTI_SETTING_HACK
        unsigned int index = 0;
        unsigned int lastSettingIndex = 0;
#endif // #if MULTI_SETTING_HACK
        {
            wxCriticalSectionLocker locker_config( m_critSectConfig );
            if( m_boRecord && ( m_totalCountToRecord > 0 ) && ( m_currentlyActiveRequests - m_currentCountToRecord <= 0 ) )
            {
                break;
            }

            if( ( m_maxNumberToRequest > 0 ) && ( m_maxNumberToRequest == m_numberRequested ) )
            {
                break;
            }
#if MULTI_SETTING_HACK
            if( m_CaptureSettingUsageMode == csumAutomatic )
            {
                lastSettingIndex = m_SettingIndex;
                unsigned int minVal = numeric_limits<unsigned int>::max();
                for( unsigned int i = 0; i < m_SettingCount; i++ )
                {
                    if( ( m_PendingRequests[m_SettingIndex] < m_pFuncInterface->requestCount() / m_SettingCount ) && ( m_PendingRequests[m_SettingIndex] < minVal ) )
                    {
                        index = m_SettingIndex;
                        minVal = m_PendingRequests[m_SettingIndex];
                    }
                    m_SettingIndex = ( m_SettingIndex + 1 ) % static_cast<int>( m_SettingCount );
                }
                if( minVal == numeric_limits<unsigned int>::max() )
                {
                    m_SettingIndex = lastSettingIndex;
                    m_RequestFailed[m_SettingIndex]++;
                    break;
                }
                m_ImageRequestControl.setting.write( m_Settings[index].second );
                m_SettingIndex = ( index + 1 ) % static_cast<int>( m_SettingCount );
            }
            else
            {
                int currentSetting = m_ImageRequestControl.setting.read();
                for( unsigned int i = 0; i < m_SettingCount; i++ )
                {
                    if( m_Settings[i].second == currentSetting )
                    {
                        index = i;
                        break;
                    }
                }
            }
#endif // #if MULTI_SETTING_HACK
        }
        if( ( requestResult = m_pFuncInterface->imageRequestSingle( &m_ImageRequestControl ) ) != DMR_NO_ERROR )
        {
#if MULTI_SETTING_HACK
            m_SettingIndex = lastSettingIndex;
            m_RequestFailed[m_SettingIndex]++;
#endif // #if MULTI_SETTING_HACK
            break;
        }
#if MULTI_SETTING_HACK
        {
            wxCriticalSectionLocker locker_config( m_critSectConfig );
            if( m_CaptureSettingUsageMode == csumAutomatic )
            {
                m_PendingRequests[index] = m_PendingRequests[index] + 1;
                m_RequestFailed[index] = 0;
            }
        }
#endif // #if MULTI_SETTING_HACK
        ++requestCnt;
        {
            wxCriticalSectionLocker locker_config( m_critSectConfig );
            ++m_currentlyActiveRequests;
            ++m_numberRequested;
            if( ( m_captureQueueDepth > 0 ) && ( m_currentlyActiveRequests >= m_captureQueueDepth ) )
            {
                break;
            }
        }
    }
    while( true );
    if( pFramesRequested )
    {
        *pFramesRequested = requestCnt;
    }
    return requestResult;
}

//-----------------------------------------------------------------------------
void CaptureThread::SendImageSkippedEvent( const int settingUsed )
//-----------------------------------------------------------------------------
{
    if( m_skippedImageCount++ == 0 )
    {
        wxCommandEvent* pEvent = new wxCommandEvent( imageSkippedEvent, m_parentWindowID );
        pEvent->SetInt( settingUsed );
        QueueEvent( pEvent );
    }
}

//-----------------------------------------------------------------------------
void CaptureThread::SendImageReadyEvent( void )
//-----------------------------------------------------------------------------
{
    ++m_capturedImagesSentToDisplayCount;
    wxCommandEvent* pImageReadyEvent = new wxCommandEvent( imageReadyEvent, m_parentWindowID );
    pImageReadyEvent->SetClientData( m_pDev );
    QueueEvent( pImageReadyEvent );
}

//-----------------------------------------------------------------------------
void CaptureThread::SetActive( void )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_activeInstance = m_instanceNr;
}

//-----------------------------------------------------------------------------
void CaptureThread::SetCaptureMode( bool boForwardIncompleteFrames )
//-----------------------------------------------------------------------------
{
    m_boForwardIncompleteFrames = boForwardIncompleteFrames;
}

//-----------------------------------------------------------------------------
void CaptureThread::SetCaptureQueueDepth( int queueDepth )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker_config( m_critSectConfig );
    m_captureQueueDepth = queueDepth;
}

#if MULTI_SETTING_HACK
//-----------------------------------------------------------------------------
void CaptureThread::SetCaptureSettingUsageMode( TCaptureSettingUsageMode mode )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_CaptureSettingUsageMode = mode;
}
#endif // #if MULTI_SETTING_HACK

//-----------------------------------------------------------------------------
void CaptureThread::SetContinuousRecording( bool boContinuous )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_boContinuousRecording = boContinuous;
}

//-----------------------------------------------------------------------------
void CaptureThread::SetCurrentPlotInfoPath( const string& currentPlotInfoPath )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_currentPlotInfoPath = currentPlotInfoPath;
}

//-----------------------------------------------------------------------------
void CaptureThread::SetImageProcessingMode( TImageProcessingMode mode )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_ImageProcessingMode = mode;
}

//-----------------------------------------------------------------------------
bool CaptureThread::SetLiveMode( bool boOn, bool boInformUserOnRecordStart /* = true */ )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    return InternalSetLiveMode( boOn, boInformUserOnRecordStart );
}

//-----------------------------------------------------------------------------
void CaptureThread::SetMultiFrameSequenceSize( size_t sequenceSize )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_maxNumberToRequest = sequenceSize;
    m_numberRequested = 0;
    m_numberReturned = 0;
}

//-----------------------------------------------------------------------------
void CaptureThread::SetPendingImageQueueDepth( unsigned int value )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_pendingRequestsForDisplayMax = value;
}

//-----------------------------------------------------------------------------
bool CaptureThread::SetRecordMode( bool boOn )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    return InternalSetRecordMode( boOn, false );
}

//-----------------------------------------------------------------------------
void CaptureThread::SetRecordSequenceSize( size_t sequenceSize )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_totalCountToRecord = sequenceSize;
}

//-----------------------------------------------------------------------------
void CaptureThread::UnlockPendingRequests( void )
//-----------------------------------------------------------------------------
{
    if( m_pendingRequestsForDisplayMax == 1 )
    {
        if( m_nextPendingRequest != INVALID_ID )
        {
            m_pFuncInterface->imageRequestUnlock( m_nextPendingRequest );
            m_nextPendingRequest = INVALID_ID;
        }
    }
    else
    {
        while( !m_pendingRequestsForDisplay.empty() )
        {
            m_pFuncInterface->imageRequestUnlock( m_pendingRequestsForDisplay.front() );
            m_pendingRequestsForDisplay.pop_front();
        }
    }
}

//-----------------------------------------------------------------------------
void CaptureThread::UnlockRequest( int nr, bool boForceUnlock /* = false */ )
//-----------------------------------------------------------------------------
{
    if( m_totalCountToRecord <= m_pFuncInterface->requestCount() )
    {
        // Unlock of request only if it's not part of the recorded sequence.
        // If the force flag is specified make sure the request will be removed
        // from the recorded sequence (a request usage counter would be better, but...)
        RequestContainer::iterator it = m_recordedSequence.begin();
        RequestContainer::const_iterator itEND = m_recordedSequence.end();
        while( it != itEND )
        {
            if( it->first == nr )
            {
                break;
            }
            ++it;
        }
        if( it == itEND )
        {
            CheckedImageRequestUnlock( nr );
        }
        else if( boForceUnlock )
        {
            CheckedImageRequestUnlock( nr );
            if( it->second )
            {
                DecreaseRecordedSequenceMemoryConsumption( static_cast<size_t>( it->second->image_.getBuffer()->iSize ) );
            }
            DeleteElement( it->second );
            m_recordedSequence.erase( it );
        }
    }
    else
    {
        CheckedImageRequestUnlock( nr );
    }

    if( m_currentRequest == nr )
    {
        m_currentRequest = INVALID_ID;
    }
}

#if MULTI_SETTING_HACK
//-----------------------------------------------------------------------------
void CaptureThread::UpdateSettingTable( void )
//-----------------------------------------------------------------------------
{
    wxCriticalSectionLocker locker( m_critSect );
    m_ImageRequestControl.setting.getTranslationDict( m_Settings );
    m_SettingCount = m_Settings.size();
    m_PendingRequests.resize( m_SettingCount );
    m_RequestFailed.resize( m_SettingCount );
    m_LastPendingRequestFrameNr.resize( m_SettingCount );
    for( unsigned int i = 0; i < m_SettingCount; i++ )
    {
        m_LastPendingRequestFrameNr[i] = numeric_limits<int64_type>::max();
    }
}
#endif // #if MULTI_SETTING_HACK
