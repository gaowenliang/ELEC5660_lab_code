//-----------------------------------------------------------------------------
#include "DataConversion.h"

using namespace std;

//-----------------------------------------------------------------------------
string charToFormat( int c )
//-----------------------------------------------------------------------------
{
    switch( c )
    {
    case 'i':
        return string( "%d" );
    case 'I':
        return string( MY_FMT_I64 );
    case 'f':
        return string( "%f" );
    case 's':
        return string( "%s" );
    case 'p':
        return string( "%p" );
    }
    return string( "" );
}

//-----------------------------------------------------------------------------
string charToType( int c )
//-----------------------------------------------------------------------------
{
    switch( c )
    {
    case 'v':
        return string( "void" );
    case 'p':
        return string( "void*" );
    case 'i':
        return string( "int" );
    case 'I':
        return string( "int64" );
    case 'f':
        return string( "float" );
    case 's':
        return string( "char*" );
    }
    return string( "" );
}

GlobalDataStorage* GlobalDataStorage::pInstance_ = 0;

//-----------------------------------------------------------------------------
GlobalDataStorage::GlobalDataStorage() : componentVisibility_( cvBeginner ), boComponentVisibilitySupported_( false ),
    LIST_BACKGROUND_COLOUR_( 230, 230, 230 ), PROPERTY_TEXT_COLOUR_( 5, 165, 5 ), INVISIBLE_GURU_FEATURE_COLOUR_( 255, 192, 64 )
//-----------------------------------------------------------------------------
{

}

//-----------------------------------------------------------------------------
GlobalDataStorage::~GlobalDataStorage()
//-----------------------------------------------------------------------------
{
    pInstance_ = 0;
}

//-----------------------------------------------------------------------------
const wxColour& GlobalDataStorage::GetPropGridColour( TPropGridColour colour ) const
//-----------------------------------------------------------------------------
{
    switch( colour )
    {
    case pgcInvisibleExpertFeature:
        return *wxGREEN;
    case pgcInvisibleFeature:
        return *wxRED;
    case pgcInvisibleGuruFeature:
        return INVISIBLE_GURU_FEATURE_COLOUR_;
    case pgcListBackground:
        return LIST_BACKGROUND_COLOUR_;
    case pgcPropertyText:
        break;
    }
    return PROPERTY_TEXT_COLOUR_;
}

//-----------------------------------------------------------------------------
GlobalDataStorage* GlobalDataStorage::Instance( void )
//-----------------------------------------------------------------------------
{
    if( !pInstance_ )
    {
        pInstance_ = new GlobalDataStorage();
    }
    return pInstance_;
}
