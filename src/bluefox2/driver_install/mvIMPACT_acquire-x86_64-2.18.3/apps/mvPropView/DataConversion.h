//-----------------------------------------------------------------------------
#ifndef DataConversionH
#define DataConversionH DataConversionH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <string>
#include <wx/wx.h>
#include <wx/colour.h>

class PropData;

#if ( defined(linux) || defined(__linux) || defined(__linux__) ) && ( defined(__x86_64__) || defined(__powerpc64__) ) // -m64 makes GCC define __powerpc64__
#   define MY_FMT_I64 "%ld"
#   define MY_FMT_I64_0_PADDED "%020ld"
#else
#   define MY_FMT_I64 "%lld"
#   define MY_FMT_I64_0_PADDED "%020lld"
#endif // #if ( defined(linux) || defined(__linux) || defined(__linux__) ) && ( defined(__x86_64__) || defined(__powerpc64__) ) // -m64 makes GCC define __powerpc64__

std::string charToFormat( int c );
std::string charToType( int c );

//-----------------------------------------------------------------------------
/// \brief This is a singleton class
class GlobalDataStorage
//-----------------------------------------------------------------------------
{
private:
    mvIMPACT::acquire::TComponentVisibility componentVisibility_;
    bool boComponentVisibilitySupported_;
    wxColour LIST_BACKGROUND_COLOUR_;
    const wxColour PROPERTY_TEXT_COLOUR_;
    const wxColour INVISIBLE_GURU_FEATURE_COLOUR_;
    static GlobalDataStorage* pInstance_;
    explicit GlobalDataStorage();
public:
    ~GlobalDataStorage();
    static GlobalDataStorage* Instance( void );
    mvIMPACT::acquire::TComponentVisibility GetComponentVisibility( void ) const
    {
        return componentVisibility_;
    }
    void SetComponentVisibility( mvIMPACT::acquire::TComponentVisibility componentVisibility )
    {
        componentVisibility_ = componentVisibility;
    }
    void SetListBackgroundColour( const wxColour& colour )
    {
        LIST_BACKGROUND_COLOUR_ = colour;
    }
    void ConfigureComponentVisibilitySupport( bool boIsSupported )
    {
        boComponentVisibilitySupported_ = boIsSupported;
    }
    bool IsComponentVisibilitySupported( void ) const
    {
        return boComponentVisibilitySupported_;
    }

    //-----------------------------------------------------------------------------
    enum TPropGridColour
    //-----------------------------------------------------------------------------
    {
        pgcInvisibleExpertFeature,
        pgcInvisibleFeature,
        pgcInvisibleGuruFeature,
        pgcListBackground,
        pgcPropertyText
    };
    const wxColour& GetPropGridColour( TPropGridColour colour ) const;
};

#endif // DataConversionH
