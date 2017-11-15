//-----------------------------------------------------------------------------
#ifndef DebugFileParserH
#define DebugFileParserH DebugFileParserH
//-----------------------------------------------------------------------------
#include <string.h>
#include <Toolkits/expat/include/ExpatImpl.h>
#include <string>
#include <vector>
#include "wx/wx.h"

//-----------------------------------------------------------------------------
enum TDebugOutputDestination
//-----------------------------------------------------------------------------
{
    doStdOut = 0x1,
    doSystem = 0x2,
    doFile   = 0x4
};

//-----------------------------------------------------------------------------
enum TLogFileFormat
//-----------------------------------------------------------------------------
{
    lffXML,
    lffText,
    lffMVLOG
};

//-----------------------------------------------------------------------------
struct LogOutputConfiguration
//-----------------------------------------------------------------------------
{
    std::string                             sectionName;
    std::string                             outputFileName;
    std::string                             stylesheetName;
    TLogFileFormat                          outputFileFormat;
    std::string                             outputFlagMask;
    TDebugOutputDestination                 outputDestinationMask;
    bool                                    boClearFile;
    LogOutputConfiguration() : sectionName( "" ), outputFileName( "" ), stylesheetName( "" ), outputFileFormat( lffXML ),
        outputFlagMask( "" ), outputDestinationMask( TDebugOutputDestination( 0 ) ), boClearFile( false ) {}
};

typedef std::vector<LogOutputConfiguration> LogConfigurationVector;

//-----------------------------------------------------------------------------
class LogConfigurationFileParser : public CExpatImpl<LogConfigurationFileParser>
//-----------------------------------------------------------------------------
{
    // internal types
    enum TTagType
    {
        ttInvalid,
        ttDBWriterList,
        ttDBWriter
    };
    static const std::string        m_ATTR_NAME;
    static const std::string        m_ATTR_FLAGS;
    static const std::string        m_ATTR_OUTPUT_MASK;
    static const std::string        m_ATTR_OUTPUT_FILE_NAME;
    static const std::string        m_ATTR_OUTPUT_FILE_FORMAT;
    static const std::string        m_ATTR_CLEAR_FILE;
    static const std::string        m_ATTR_STYLESHEET_NAME;
    std::string                     m_sectionName;
    LogConfigurationVector          m_results;

    TTagType                        GetTagType( const char* tagName ) const;
public:
    void                            OnPostCreate    ( void );
    void                            OnStartElement  ( const XML_Char* pszName, const XML_Char** papszAttrs );
    const LogConfigurationVector&   GetResults      ( void ) const
    {
        return m_results;
    }
};

unsigned int StringToBitmask( const std::string& input );
//-----------------------------------------------------------------------------
template<class T>
std::string BitmaskToString( T bitmask, bool boSkipLeadingZeros = true )
//-----------------------------------------------------------------------------
{
    std::string result;
    for( int i = sizeof( T ) * 8 - 1; i >= 0; i-- )
    {
        if( bitmask & ( 1 << i ) )
        {
            result.append( "1" );
            continue;
        }

        else if( boSkipLeadingZeros && ( result.length() == 0 ) )
        {
            continue;
        }
        result.append( "0" );
    }
    return result;
}

wxString FileFormatToString( TLogFileFormat fileFormat );
TLogFileFormat FileFormatFromString( const wxString& fileFormat );

#endif // DebugFileParserH
