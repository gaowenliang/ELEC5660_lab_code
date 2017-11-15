//--------------------------------------------------------------------------
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "PGMFileFunctions.h"

using namespace std;

//--------------------------------------------------------------------------
static void mkHeader( char* pBuf, const char* pMagicNb, const long lWidth, const long lHeight )
//--------------------------------------------------------------------------
{
    sprintf( pBuf, "%s\n# Created by MATRIX VISION GmbH\n%ld\n%ld\n255\n", pMagicNb, lWidth, lHeight );
}

//--------------------------------------------------------------------------
int writePGMFile( const unsigned char* pData, const unsigned long ulWidth, const unsigned long ulHeight,
                  const unsigned long ulPitch, unsigned int uiBypp, const char* pFileName )
//--------------------------------------------------------------------------
{
    // printf( "%s(%s: %d): pData: %p ulWidth: %ld ulHeight: %ld ulPitch: %ld uiBypp: %d pFileName: %s\n",
    //          __FUNCTION__, __FILE__, __LINE__, pData, ulWidth, ulHeight, ulPitch, uiBypp, pFileName );

    if( uiBypp > 1 )
    {
        cerr << "*** " << __FUNCTION__ << " - only 8-bit per pixel images supported! This image has " << uiBypp * 8 << "-bit per pixel!" << endl;
        return -1;
    }

    FILE* pF = NULL;
    size_t retCount;
    int ret = -1, err = 0;
    char imgHeader[512];
    unsigned int iImageFileSize = ulPitch * ulHeight;
    unsigned int iLineBytes = ulWidth * uiBypp;

    if( ( pF = fopen( pFileName, "wb" ) ) != NULL )
    {
        memset( imgHeader, 0, 512 );
        mkHeader( imgHeader, "P5", ulWidth, ulHeight );

        fwrite( imgHeader, 1, strlen( imgHeader ), pF );

        if( ulPitch == iLineBytes )
        {
            if( ( retCount = fwrite( pData, 1, iImageFileSize, pF ) ) != iImageFileSize )
            {
                cerr << "*** " << __FUNCTION__ << "- only " << retCount << " bytes written to file " << pFileName << endl;
            }
            else
            {
                ret = 0;
            }
        }
        else
        {
            for( unsigned long y = 0; y < ulHeight; y++ )
            {
                if( ( retCount = fwrite( pData + y * ulPitch, 1, iLineBytes, pF ) ) != iLineBytes )
                {
                    cerr << "*** " << __FUNCTION__ << " - only " << retCount << " from " << iLineBytes << " bytes written to file " << pFileName << endl;
                    err = 1;
                    break;
                }
            }

            if( !err )
            {
                ret = 0;
            }
        }
    }
    else
    {
        cerr << "*** " << __FUNCTION__ << " - can not open file " << pFileName << endl;
    }

    if( pF )
    {
        fclose( pF );
    }

    return ret;
}
