//-----------------------------------------------------------------------------
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "exampleHelper_C.h"

#define BUF_SIZE (32)
#define BUF_SIZE_LARGE (256)

//-----------------------------------------------------------------------------
int getIntValFromSTDIn( void )
//-----------------------------------------------------------------------------
{
    int value;
    int conversionResult = 0;

#if defined(_MSC_VER) && (_MSC_VER >= 1400) // is at least VC 2005 compiler?
    conversionResult = scanf_s( "%d", &value );
#else
    conversionResult = scanf( "%d", &value );
#endif // #if defined(_MSC_VER) && (_MSC_VER >= 1400)
    if( conversionResult != 1 )
    {
        printf( "Conversion error: Expected: 1, conversion result: %d.\n", conversionResult );
    }
    return value;
}

//-----------------------------------------------------------------------------
int getPropI( HOBJ hProp, int index )
//-----------------------------------------------------------------------------
{
    int value = 0;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    if( ( result = OBJ_GetI( hProp, &value, index ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "getPropI: Failed to read property value(%s).\n", DMR_ErrorCodeToString( result ) );
        exit( 42 );
    }
    return value;
}

//-----------------------------------------------------------------------------
void setPropI( HOBJ hProp, int value, int index )
//-----------------------------------------------------------------------------
{
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    if( ( result = OBJ_SetI( hProp, value, index ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "setPropI: Failed to write property value(%s).\n", DMR_ErrorCodeToString( result ) );
        exit( 42 );
    }
}

//-----------------------------------------------------------------------------
int64_type getPropI64( HOBJ hProp, int index )
//-----------------------------------------------------------------------------
{
    int64_type value = 0;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    if( ( result = OBJ_GetI64( hProp, &value, index ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "getPropI: Failed to read property value(%s).\n", DMR_ErrorCodeToString( result ) );
        exit( 42 );
    }
    return value;
}

//-----------------------------------------------------------------------------
void setPropI64( HOBJ hProp, int64_type value, int index )
//-----------------------------------------------------------------------------
{
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    if( ( result = OBJ_SetI64( hProp, value, index ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "setPropI: Failed to write property value(%s).\n", DMR_ErrorCodeToString( result ) );
        exit( 42 );
    }
}

//-----------------------------------------------------------------------------
void* getPropP( HOBJ hProp, int index )
//-----------------------------------------------------------------------------
{
    void* value = 0;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    if( ( result = OBJ_GetP( hProp, &value, index ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "getPropP: Failed to read property value(%s).\n", DMR_ErrorCodeToString( result ) );
        exit( 42 );
    }
    return value;
}

//-----------------------------------------------------------------------------
void setPropP( HOBJ hProp, void* value, int index )
//-----------------------------------------------------------------------------
{
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    if( ( result = OBJ_SetP( hProp, value, index ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "setPropP: Failed to write property value(%s).\n", DMR_ErrorCodeToString( result ) );
        exit( 42 );
    }
}

//-----------------------------------------------------------------------------
void setPropS( HOBJ hProp, const char* pVal, int index )
//-----------------------------------------------------------------------------
{
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    if( ( result = OBJ_SetS( hProp, pVal, index ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "setPropS: Failed to write property value %s :%s.\n", pVal, DMR_ErrorCodeToString( result ) );
        exit( 42 );
    }
}

//-----------------------------------------------------------------------------
// This function will try to obtain the handle to a certain driver feature
HOBJ getDriverFeature( HDRV hDrv, const char* pFeatureName, const char* pFeatureType, const char* pAddListName, TDMR_ListType type, unsigned int searchMode )
//-----------------------------------------------------------------------------
{
    TDMR_ERROR dmrResult = DMR_NO_ERROR;
    HOBJ hObj = INVALID_ID;
    HLIST baseList = INVALID_ID;

    // try to locate the base list for these property
    if( ( dmrResult = DMR_FindList( hDrv, pAddListName, type, 0, &baseList ) ) == DMR_NO_ERROR )
    {
        // try to locate the property
        TPROPHANDLING_ERROR objResult;
        if( ( objResult = OBJ_GetHandleEx( baseList, pFeatureName, &hObj, searchMode, INT_MAX ) ) != PROPHANDLING_NO_ERROR )
        {
            printf( "OBJ_GetHandleEx for '%s' failed: %d Handle: %d. This %s might not be supported by this device\n", pFeatureName, objResult, hObj, pFeatureType );
        }
    }
    else
    {
        printf( "DMR_FindList failed: %d. Lists of type %d are not available for this device\n", dmrResult, type );
    }
    return hObj;
}

//-----------------------------------------------------------------------------
// This function will try to obtain the handle to a certain driver feature list
HOBJ getDriverList( HDRV hDrv, const char* pListName, const char* pAddListName, TDMR_ListType type )
//-----------------------------------------------------------------------------
{
    return getDriverFeature( hDrv, pListName, "list", pAddListName, type, smIgnoreProperties | smIgnoreMethods );
}

//-----------------------------------------------------------------------------
// This function will try to obtain the handle to a certain driver property
HOBJ getDriverProperty( HDRV hDrv, const char* pPropName, const char* pAddListName, TDMR_ListType type )
//-----------------------------------------------------------------------------
{
    return getDriverFeature( hDrv, pPropName, "property", pAddListName, type, smIgnoreLists | smIgnoreMethods );
}

//-----------------------------------------------------------------------------
// This function will try to obtain the handle to a certain driver property
HOBJ getDriverMethod( HDRV hDrv, const char* pPropName, const char* pAddListName, TDMR_ListType type )
//-----------------------------------------------------------------------------
{
    return getDriverFeature( hDrv, pPropName, "method", pAddListName, type, smIgnoreProperties | smIgnoreLists );
}

//-----------------------------------------------------------------------------
HOBJ getDeviceProp( HDEV hDev, const char* pPropName )
//-----------------------------------------------------------------------------
{
    TPROPHANDLING_ERROR objResult;
    HOBJ hProp = INVALID_ID;

    // try to locate the property
    if( ( objResult = OBJ_GetHandleEx( hDev, pPropName, &hProp, 0, -1 ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "OBJ_GetHandleEx failed for property '%s': %d Handle: %d\n", pPropName, objResult, hProp );
    }
    return hProp;
}

//-----------------------------------------------------------------------------
HOBJ getInfoProp( HDRV hDrv, const char* pPropName )
//-----------------------------------------------------------------------------
{
    return getDriverProperty( hDrv, pPropName, 0, dmltInfo );
}

//-----------------------------------------------------------------------------
HOBJ getIOSubSystemProp( HDRV hDrv, const char* pPropName )
//-----------------------------------------------------------------------------
{
    return getDriverProperty( hDrv, pPropName, 0, dmltIOSubSystem );
}

//-----------------------------------------------------------------------------
HOBJ getRequestCtrlProp( HDRV hDrv, const char* pRequestCtrlName, const char* pPropName )
//-----------------------------------------------------------------------------
{
    return getDriverProperty( hDrv, pPropName, pRequestCtrlName, dmltRequestCtrl );
}

//-----------------------------------------------------------------------------
HOBJ getRequestProp( HDRV hDrv, int requestNr, const char* pPropName )
//-----------------------------------------------------------------------------
{
    char buf[BUF_SIZE];

#if defined(_MSC_VER) && (_MSC_VER >= 1400) // is at least VC 2005 compiler?
    sprintf_s( buf, BUF_SIZE, "Entry %d", requestNr );
#else
    sprintf( buf, "Entry %d", requestNr );
#endif // #if defined(_MSC_VER) && (_MSC_VER >= 1400) // is at least VC 2005 compiler?
    return getDriverProperty( hDrv, pPropName, buf, dmltRequest );
}

//-----------------------------------------------------------------------------
HOBJ getSettingProp( HDRV hDrv, const char* pSettingName, const char* pPropName )
//-----------------------------------------------------------------------------
{
    return getDriverProperty( hDrv, pPropName, pSettingName, dmltSetting );
}

//-----------------------------------------------------------------------------
HOBJ getSettingMethod( HDRV hDrv, const char* pSettingName, const char* pMethodName )
//-----------------------------------------------------------------------------
{
    return getDriverMethod( hDrv, pMethodName, pSettingName, dmltSetting );
}

//-----------------------------------------------------------------------------
HOBJ getStatisticProp( HDRV hDrv, const char* pPropName )
//-----------------------------------------------------------------------------
{
    return getDriverProperty( hDrv, pPropName, 0, dmltStatistics );
}

//-----------------------------------------------------------------------------
HOBJ getSystemSettingProp( HDRV hDrv, const char* pPropName )
//-----------------------------------------------------------------------------
{
    return getDriverProperty( hDrv, pPropName, 0, dmltSystemSettings );
}

//-----------------------------------------------------------------------------
TPROPHANDLING_ERROR getStringValue( HOBJ hObj, char** pBuf, int index )
//-----------------------------------------------------------------------------
{
    size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
    static const int BUFFER_INCREMENT_FACTOR = 2;

    *pBuf = ( char* )calloc( 1, bufSize );
    while( ( result = OBJ_GetS( hObj, *pBuf, bufSize, index ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
    {
        bufSize *= BUFFER_INCREMENT_FACTOR;
        *pBuf = ( char* )realloc( *pBuf, bufSize );
    }
    if( result != PROPHANDLING_NO_ERROR )
    {
        printf( "Error while reading string property value: Error code: %d(%s).\n", result, DMR_ErrorCodeToString( result ) );
    }
    return result;
}

//-----------------------------------------------------------------------------
TPROPHANDLING_ERROR getValueAsString( HOBJ hObj, const char* pFormat, char** pBuf, int index )
//-----------------------------------------------------------------------------
{
    size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    *pBuf = ( char* )calloc( 1, bufSize );
    while( ( result = OBJ_GetSFormattedEx( hObj, *pBuf, &bufSize, pFormat, index ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
    {
        *pBuf = ( char* )realloc( *pBuf, bufSize );
    }

    if( result != PROPHANDLING_NO_ERROR )
    {
        printf( "Error while reading string property value: Error code: %d(%s).\n", result, DMR_ErrorCodeToString( result ) );
    }
    return result;
}

//-----------------------------------------------------------------------------
// Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
// Whether this is needed or not depends on the property 'AcquisitionStartStopBehaviour' in the list referenced by 'HDEV'.
void manuallyStartAcquisitionIfNeeded( HDRV hDrv )
//-----------------------------------------------------------------------------
{
    const TDMR_ERROR result = DMR_AcquisitionStart( hDrv );
    if( ( result != DMR_NO_ERROR ) &&
        ( result != DMR_FEATURE_NOT_AVAILABLE ) )
    {
        printf( "DMR_AcquisitionStart: Unexpected error(code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
    }
}

//-----------------------------------------------------------------------------
// Stop the acquisition manually if this was requested.
// Whether this is needed or not depends on the property 'AcquisitionStartStopBehaviour' in the list referenced by 'HDEV'.
void manuallyStopAcquisitionIfNeeded( HDRV hDrv )
//-----------------------------------------------------------------------------
{
    const TDMR_ERROR result = DMR_AcquisitionStop( hDrv );
    if( ( result != DMR_NO_ERROR ) &&
        ( result != DMR_FEATURE_NOT_AVAILABLE ) )
    {
        printf( "DMR_AcquisitionStop: Unexpected error(code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
    }
}

//-----------------------------------------------------------------------------
void modifyEnumPropertyI( HDRV hDrv, const char* pSettingName, const char* pPropName )
//-----------------------------------------------------------------------------
{
    HOBJ hProp = INVALID_ID;
    unsigned int dictValCount = 0;
    int* dictVals = NULL;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;

    printf( "Trying to modify property %s:\n", pPropName );
    if( ( hProp = getSettingProp( hDrv, pSettingName, pPropName ) ) != INVALID_ID )
    {
        if( ( result = readTranslationDictValuesI( hProp, &dictVals, &dictValCount, 0 ) ) == PROPHANDLING_NO_ERROR )
        {
            int value = 0;
            printf( "Please select one of the values listed above: " );
            value = getIntValFromSTDIn();
            free( dictVals );
            // set the new trigger mode
            if( ( result = OBJ_SetI( hProp, value, 0 ) ) != PROPHANDLING_NO_ERROR )
            {
                printf( "Failed to set new value for %s. Error code: %d(%s).\n", pPropName, result, DMR_ErrorCodeToString( result ) );
            }
        }
    }
}

//-----------------------------------------------------------------------------
TPROPHANDLING_ERROR readTranslationDictValuesI( HOBJ hObj, int** pDictValues, unsigned int* pDictValCnt, unsigned int silent )
//-----------------------------------------------------------------------------
{
    TPROPHANDLING_ERROR funcResult = PROPHANDLING_NO_ERROR;
    char** ppBuf = 0;
    unsigned int i = 0;
    size_t bufSize = 0;
    const size_t BUFFER_INCREMENT_FACTOR = 6;

    if( ( funcResult = OBJ_GetDictSize( hObj, pDictValCnt ) ) != PROPHANDLING_NO_ERROR )
    {
        return funcResult;
    }

    *pDictValues = ( int* )calloc( *pDictValCnt, sizeof( int ) );
    ppBuf = ( char** )calloc( *pDictValCnt, sizeof( char* ) );
    bufSize = DEFAULT_STRING_SIZE_LIMIT;
    for( i = 0; i < *pDictValCnt; i++ )
    {
        ppBuf[i] = ( char* )calloc( 1, bufSize );
    }

    while( ( funcResult = OBJ_GetIDictEntries( hObj, ppBuf, bufSize, *pDictValues, ( size_t ) * pDictValCnt ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
    {
        bufSize *= BUFFER_INCREMENT_FACTOR;
        for( i = 0; i < *pDictValCnt; i++ )
        {
            ppBuf[i] = ( char* )realloc( ppBuf[i], bufSize );
        }
    }

    if( ( funcResult == PROPHANDLING_NO_ERROR ) &&
        ( silent == 0 ) )
    {
        printf( "Got the following dictionary:\n" );
        for( i = 0; i < *pDictValCnt; i++ )
        {
            printf( "[%d]: %s(numerical rep: %d)\n", i, ppBuf[i], ( *pDictValues )[i] );
        }
    }

    // free memory again
    for( i = 0; i < *pDictValCnt; i++ )
    {
        free( ppBuf[i] );
    }
    free( ppBuf );
    return funcResult;
}

//-----------------------------------------------------------------------------
void conditionalSetPropI( HOBJ hProp, int value, unsigned int silent )
//-----------------------------------------------------------------------------
{
    unsigned int dictValCount = 0;
    size_t i = 0;
    int* dictVals = NULL;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
    char bufName[BUF_SIZE_LARGE];
    char* pBufValue = 0;

    if( ( result = readTranslationDictValuesI( hProp, &dictVals, &dictValCount, silent ) ) == PROPHANDLING_NO_ERROR )
    {
        for( i = 0; i < dictValCount; i++ )
        {
            if( dictVals[i] == value )
            {
                setPropI( hProp, value, 0 );
                memset( bufName, '\0', BUF_SIZE_LARGE );
                OBJ_GetName( hProp, bufName, BUF_SIZE_LARGE );
                getValueAsString( hProp, 0, &pBufValue, 0 );
                if( silent == 0 )
                {
                    printf( "Property '%s' set to '%s'.\n", bufName, pBufValue );
                }
                free( pBufValue );
                break;
            }
        }
        free( dictVals );
    }
    else
    {
        printf( "Failed to read translation dictionary from property. Error code: %d(%s).\n", result, DMR_ErrorCodeToString( result ) );
    }
}

//-----------------------------------------------------------------------------
int getDeviceFromUserInput( HDEV* phDevice, SUPPORTED_DEVICE_CHECK pSupportedDeviceCheckFn, unsigned int automaticallyUseGenICamInterface )
//-----------------------------------------------------------------------------
{
    TDMR_ERROR       result = DMR_NO_ERROR;
    unsigned int     i = 0;
    unsigned int     deviceCount = 0;
    unsigned int     deviceNumber = 0;
    HOBJ             hPropSerial = INVALID_ID;
    HOBJ             hPropProduct = INVALID_ID;
    HOBJ             hPropInterfaceLayout = INVALID_ID;
    HOBJ             hPropAcquisitionStartStopBehaviour = INVALID_ID;
    char*            pSerialStringBuffer = NULL;
    char*            pProductStringBuffer = NULL;

    if( ( result = DMR_GetDeviceCount( &deviceCount ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_GetDeviceCount failed (code: %d(%s))\n", result, DMR_ErrorCodeToString( result ) );
        END_APPLICATION;
    }

    if( deviceCount == 0 )
    {
        printf( "No MATRIX VISION mvIMPACT Acquire compliant device detected.\n" );
        END_APPLICATION;
    }

    printf( "%d MATRIX VISION mvIMPACT Acquire compliant devices detected.\n", deviceCount );
    for( i = 0; i < deviceCount; i++ )
    {
        // try to get access to the device
        if( ( result = DMR_GetDevice( phDevice, dmdsmSerial, "*", i, '*' ) ) != DMR_NO_ERROR )
        {
            printf( "DMR_GetDevice(%d) failed (code: %d(%s))\n", i, result, DMR_ErrorCodeToString( result ) );
            END_APPLICATION;
        }

        if( ( hPropSerial = getDeviceProp( *phDevice, "Serial" ) ) == INVALID_ID )
        {
            continue;
        }
        getStringValue( hPropSerial, &pSerialStringBuffer, 0 );
        if( !pSupportedDeviceCheckFn || pSupportedDeviceCheckFn( *phDevice ) )
        {
            if( ( hPropProduct = getDeviceProp( *phDevice, "Product" ) ) == INVALID_ID )
            {
                continue;
            }
            getStringValue( hPropProduct, &pProductStringBuffer, 0 );
            printf( "[%d] %s (%s", i, pSerialStringBuffer, pProductStringBuffer );
            if( ( hPropInterfaceLayout = getDeviceProp( *phDevice, "InterfaceLayout" ) ) != INVALID_ID )
            {
                char* pStringBuffer = NULL;
                getValueAsString( hPropInterfaceLayout, NULL, &pStringBuffer, 0 );
                if( automaticallyUseGenICamInterface != 0 )
                {
                    conditionalSetPropI( hPropInterfaceLayout, dilGenICam, 1 );
                }
                printf( ", interface layout: %s", pStringBuffer );
                free( pStringBuffer );
            }

            if( ( hPropAcquisitionStartStopBehaviour = getDeviceProp( *phDevice, "AcquisitionStartStopBehaviour" ) ) != INVALID_ID )
            {
                char* pStringBuffer = NULL;
                conditionalSetPropI( hPropAcquisitionStartStopBehaviour, assbUser, 1 );
                getValueAsString( hPropAcquisitionStartStopBehaviour, NULL, &pStringBuffer, 0 );
                printf( ", acquisition start/stop behaviour: %s", pStringBuffer );
                free( pStringBuffer );
            }
            printf( ")\n" );
            free( pProductStringBuffer );
        }
        else
        {
            printf( "%s is not supported by this application.\n", pSerialStringBuffer );
        }
        free( pSerialStringBuffer );
    }

    printf( "Please enter the number in brackets followed by [ENTER] to open it: " );
    deviceNumber = getIntValFromSTDIn();
    // remove the '\n' from the stream
    fgetc( stdin );

    // try to get access to the selected device
    if( ( result = DMR_GetDevice( phDevice, dmdsmSerial, "*", deviceNumber, '*' ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_GetDevice(%d) failed (code: %d(%s))\n", deviceNumber, result, DMR_ErrorCodeToString( result ) );
        printf( "DMR_Close: %d\n", DMR_Close() );
        END_APPLICATION;
    }

    return 0;
}

//-----------------------------------------------------------------------------
static unsigned int isFeatureFlagSet( HOBJ hObj, TComponentFlag flag )
//-----------------------------------------------------------------------------
{
    TComponentFlag flags;

    if( ( hObj == INVALID_ID ) ||
        ( OBJ_GetFlags( hObj, &flags ) != PROPHANDLING_NO_ERROR ) )
    {
        return 0;
    }

    return ( ( flags & flag ) != 0 );
}

//-----------------------------------------------------------------------------
unsigned int isFeatureReadable( HOBJ hObj )
//-----------------------------------------------------------------------------
{
    return isFeatureFlagSet( hObj, cfReadAccess );
}

//-----------------------------------------------------------------------------
unsigned int isFeatureWriteable( HOBJ hObj )
//-----------------------------------------------------------------------------
{
    return isFeatureFlagSet( hObj, cfWriteAccess );
}

#if defined(linux) || defined(__linux) || defined(__linux__)
#   include <sys/types.h>
#   include <unistd.h>
//-----------------------------------------------------------------------------
// returns 0 if timeout, else 1
unsigned waitForInput( int maxWait_sec, int fd )
//-----------------------------------------------------------------------------
{
    fd_set rfds;
    struct timeval tv;

    FD_ZERO( &rfds );
    FD_SET( fd, &rfds );

    tv.tv_sec = maxWait_sec ;
    tv.tv_usec = 0;

    return select( fd + 1, &rfds, NULL, NULL, &tv );
}
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
