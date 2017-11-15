//--------------------------------------------------------------------------
#ifndef PGM_FILE_FUNCTIONS_H
#define PGM_FILE_FUNCTIONS_H PGM_FILE_FUNCTIONS_H
//--------------------------------------------------------------------------

int writePGMFile( const unsigned char* pData, const unsigned long ulWidth, const unsigned long ulHeight,
                  const unsigned long ulPitch, unsigned int uiBypp, const char* pFileName );

#endif // PGM_FILE_FUNCTIONS_H

