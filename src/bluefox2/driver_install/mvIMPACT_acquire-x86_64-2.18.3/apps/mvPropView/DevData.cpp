#include "DevData.h"

//=============================================================================
//================= Implementation RequestData ================================
//=============================================================================

const wxString RequestData::UNKNOWN_PIXEL_FORMAT_STRING_( wxT( "unknown" ) );

//-----------------------------------------------------------------------------
RequestData::RequestData() : image_( 1 ), pixelFormat_( UNKNOWN_PIXEL_FORMAT_STRING_ ), requestInfo_(), requestInfoStrings_(), requestNr_( INVALID_ID )
//-----------------------------------------------------------------------------
{
    image_.getBuffer()->pixelFormat = ibpfMono8;
}
