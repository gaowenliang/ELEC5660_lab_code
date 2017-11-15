#include <apps/Common/exampleHelper_C.h>
#include <mvDeviceManager/Include/mvDeviceManager.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#   include <conio.h> // _getch under Windows
#endif // #ifdef _WIN32

//-----------------------------------------------------------------------------
typedef struct TMatchbox
//-----------------------------------------------------------------------------
{
    int matches_;
} Matchbox;

//-----------------------------------------------------------------------------
typedef struct TWallet
//-----------------------------------------------------------------------------
{
    double euros_;
} Wallet;

//-----------------------------------------------------------------------------
typedef struct THandbag
//-----------------------------------------------------------------------------
{
    Matchbox matchbox_;
    Wallet wallet_;
} Handbag;

//-----------------------------------------------------------------------------
// Please note that this function may have more hit counts then the provoking loop
// has iterations. This is because the callback gets executed when a monitored
// feature will change in any way thus e.g. also if its visibility or max value
// changes.
//
// Please note that this function might be executed from ANY thread context, which is
// most likely not the same as used by the application thus appropriate mechanisms to ensure
// correct execution must be implemented by an application(e.g. GUI applications might send an
// event to the main thread instead of directly accessing GUI elements).
void DMR_CALL callbackFunction( HOBJ hObj, void* pUserData )
//-----------------------------------------------------------------------------
{
    static unsigned int hitCount = 0;
    Wallet* pWallet = 0;
    Matchbox* pMatchbox = 0;

    printf( "Hit-count: %d, hObj causing the callback: 0x%08x\n", ++hitCount, hObj );

    pWallet = &( ( Handbag* )pUserData )->wallet_;
    pMatchbox = &( ( Handbag* )pUserData )->matchbox_;

    printf( "There are %d matches in my box and %.2f Euros in my wallet.\n", pMatchbox->matches_, pWallet->euros_ );
    if( pMatchbox->matches_ > 0 )
    {
        --pMatchbox->matches_;
        printf( "I lit a match.\n" );
    }
    else if( pWallet->euros_ > 0.5 )
    {
        pWallet->euros_ -= 0.5;
        printf( "I bought a new box of matches.\n" );
        pMatchbox->matches_ = 5;
    }
    else
    {
        printf( "I got no more matches and don't have enough money to buy new ones...\n" );
    }
}

//-----------------------------------------------------------------------------
void checkCallbackCreation( HDRV hDrv )
//-----------------------------------------------------------------------------
{
    Handbag handbag;
    HOBJ hPropImageRequestTimeout_ms = getSettingProp( hDrv, "Base", "ImageRequestTimeout_ms" );
    CallbackHandle hCallback = 0;
    TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
    int i = 0;

    if( hPropImageRequestTimeout_ms == INVALID_ID )
    {
        printf( "Failed to locate 'ImageRequestTimeout_ms' property.\n" );
        return;
    }


    handbag.matchbox_.matches_ = 5;
    handbag.wallet_.euros_ = 2.14;

    if( ( result = OBJ_CreateCallback( ctOnChanged, callbackFunction, &handbag, &hCallback ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "Failed to create callback object. Result: %s\n", DMR_ErrorCodeToString( result ) );
        return;
    }

    if( ( result = OBJ_AttachCallback( hPropImageRequestTimeout_ms, hCallback ) ) == PROPHANDLING_NO_ERROR )
    {
        // provoke some callbacks by modifying the properties we just registered callbacks for.
        // Whenever one of this features is changed in any way the previously attached callback
        // handler will be called.
        for( i = 0; i < 30; i++ )
        {
            printf( "Provoking callback %d.\n", i + 1 );
            setPropI( hPropImageRequestTimeout_ms, i, 0 );
        }

        if( ( result = OBJ_DetachCallback( hPropImageRequestTimeout_ms, hCallback ) ) != PROPHANDLING_NO_ERROR )
        {
            printf( "Failed to detach callback to property. Result: %s\n", DMR_ErrorCodeToString( result ) );
        }
    }
    else
    {
        printf( "Failed to attach callback to property. Result: %s\n", DMR_ErrorCodeToString( result ) );
    }

    if( ( result = OBJ_DeleteCallback( hCallback ) ) != PROPHANDLING_NO_ERROR )
    {
        printf( "Failed to delete callback object. Result: %s\n", DMR_ErrorCodeToString( result ) );
    }
    hCallback = 0;

    // now verify that the callbacks have actually been executed by displaying the amount of money left in the wallet
    if( handbag.wallet_.euros_ < 2.14 )
    {
        printf( "The callback spent some of my money. I got %.2f left.\n", handbag.wallet_.euros_ );
    }
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    HDMR             hDMR = INVALID_ID;
    HDRV             hDrv = INVALID_ID;
    HDEV             hDev = INVALID_ID;
    TDMR_ERROR       result = DMR_NO_ERROR;
    unsigned int     deviceCount = 0;
    unsigned int     i = 0;
    HOBJ             hPropSerial = INVALID_ID;
    char*            pStringBuffer = NULL;
    unsigned int     deviceNumber = 0;

    // get rid of warnings
    argc = argc;
    argv = argv;

    // try to initialise the library.
    if( ( result = DMR_Init( &hDMR ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_Init failed (code: %d)\n", result );
        END_APPLICATION;
    }

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
        if( ( result = DMR_GetDevice( &hDev, dmdsmSerial, "*", i, '*' ) ) != DMR_NO_ERROR )
        {
            printf( "DMR_GetDevice(%d) failed (code: %d(%s))\n", i, result, DMR_ErrorCodeToString( result ) );
            END_APPLICATION;
        }
        if( ( hPropSerial = getDeviceProp( hDev, "Serial" ) ) == INVALID_ID )
        {
            printf( "Failed to obtain device serial property for device %d.\n", i );
            continue;
        }
        getStringValue( hPropSerial, &pStringBuffer, 0 );
        printf( "[%d]: %s.\n", i, pStringBuffer );
        free( pStringBuffer );
    }

    printf( "Please enter the number in front of the listed device followed by [ENTER] to open it: " );
    deviceNumber = getIntValFromSTDIn();

    // try to get access to the selected device
    if( ( result = DMR_GetDevice( &hDev, dmdsmSerial, "*", deviceNumber, '*' ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_GetDevice(%d) failed (code: %d(%s))\n", deviceNumber, result, DMR_ErrorCodeToString( result ) );
        printf( "DMR_Close: %d\n", DMR_Close() );
        END_APPLICATION;
    }

    // try to initialise this device
    if( ( result = DMR_OpenDevice( hDev, &hDrv ) ) != DMR_NO_ERROR )
    {
        printf( "DMR_OpenDevice failed (code: %d)\n", result );
        printf( "DMR_Close: %d\n", DMR_Close() );
        END_APPLICATION;
    }

    checkCallbackCreation( hDrv );

    printf( "DMR_CloseDevice: %d\n", DMR_CloseDevice( hDrv, hDev ) );
    printf( "DMR_Close: %d\n", DMR_Close() );
    END_APPLICATION;
}
