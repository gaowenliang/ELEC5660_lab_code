//-----------------------------------------------------------------------------
#include <algorithm>
#include <apps/Common/wxAbstraction.h>
#include "DebugFileParser.h"
#include <functional>
#include <map>

using namespace std;

typedef map<string, string> StringMap;

const string LogConfigurationFileParser::m_ATTR_NAME( "name" );
const string LogConfigurationFileParser::m_ATTR_FLAGS( "flags" );
const string LogConfigurationFileParser::m_ATTR_OUTPUT_MASK( "outputmask" );
const string LogConfigurationFileParser::m_ATTR_OUTPUT_FILE_NAME( "outputfile" );
const string LogConfigurationFileParser::m_ATTR_OUTPUT_FILE_FORMAT( "fileFormat" );
const string LogConfigurationFileParser::m_ATTR_CLEAR_FILE( "clearFile" );
const string LogConfigurationFileParser::m_ATTR_STYLESHEET_NAME( "stylesheet" );

//-----------------------------------------------------------------------------
LogConfigurationFileParser::TTagType LogConfigurationFileParser::GetTagType( const char* tagName ) const
//-----------------------------------------------------------------------------
{
    if( strcmp( tagName, "DebugWriter" ) == 0 )
    {
        return ttDBWriter;
    }
    else if( strcmp( tagName, "DebugWriterList" ) == 0 )
    {
        return ttDBWriterList;
    }
    return ttInvalid;
}

//-----------------------------------------------------------------------------
void LogConfigurationFileParser::OnPostCreate( void )
//-----------------------------------------------------------------------------
{
    EnableStartElementHandler();
}

//-----------------------------------------------------------------------------
unsigned int StringToBitmask( const string& input )
//-----------------------------------------------------------------------------
{
    unsigned int result = 0;
    size_t length = input.length();
    for( size_t j = 0; j < length; j++ )
    {
        if( input[length - 1 - j] == '1' )
        {
            result |= 1 << j;
        }
    }
    return result;
}

//-----------------------------------------------------------------------------
void LogConfigurationFileParser::OnStartElement( const XML_Char* pszName, const XML_Char** papszAttrs )
//-----------------------------------------------------------------------------
{
    switch( GetTagType( pszName ) )
    {
    case ttDBWriter:
        {
            // build attribute map
            StringMap attrMap;
            int i = 0;
            while( papszAttrs[i] )
            {
                attrMap.insert( pair<string, string>( papszAttrs[i], papszAttrs[i + 1] ) );
                i += 2;
            }

            LogOutputConfiguration data;
            StringMap::iterator it = attrMap.find( m_ATTR_NAME );
            if( it != attrMap.end() )
            {
                data.sectionName = it->second;
            }

            it = attrMap.find( m_ATTR_FLAGS );
            if( it != attrMap.end() )
            {
                string::size_type firstOne = it->second.find_first_of( "1" );
                // remove leading zeros...
                if( firstOne != string::npos )
                {
                    data.outputFlagMask = it->second.substr( firstOne );
                }
                else
                {
                    data.outputFlagMask = it->second;
                }
            }

            it = attrMap.find( m_ATTR_OUTPUT_MASK );
            if( it != attrMap.end() )
            {
                data.outputDestinationMask = TDebugOutputDestination( StringToBitmask( it->second ) );
            }

            it = attrMap.find( m_ATTR_OUTPUT_FILE_NAME );
            if( it != attrMap.end() )
            {
                data.outputFileName = it->second;
            }

            it = attrMap.find( m_ATTR_OUTPUT_FILE_FORMAT );
            if( it != attrMap.end() )
            {
                transform( it->second.begin(), it->second.end(), it->second.begin(), ptr_fun<int, int>( tolower ) );
                data.outputFileFormat = FileFormatFromString( ConvertedString( it->second ) );
            }

            it = attrMap.find( m_ATTR_CLEAR_FILE );
            if( it != attrMap.end() )
            {
                data.boClearFile = ( atoi( it->second.c_str() ) != 0 );
            }

            it = attrMap.find( m_ATTR_STYLESHEET_NAME );
            if( it != attrMap.end() )
            {
                data.stylesheetName = it->second;
            }
            m_results.push_back( data );
        }
        break;
    case ttDBWriterList:
        // nothing to be done here...
        break;
    default:
        // invalid tag...
        break;
    }
}

//-----------------------------------------------------------------------------
wxString FileFormatToString( TLogFileFormat fileFormat )
//-----------------------------------------------------------------------------
{
    switch( fileFormat )
    {
    case lffXML:
        break;
    case lffText:
        return wxString( wxT( "text" ) );
    case lffMVLOG:
        return wxString( wxT( "mvlog" ) );
    }
    return wxString( wxT( "xml" ) );
}

//-----------------------------------------------------------------------------
TLogFileFormat FileFormatFromString( const wxString& fileFormat )
//-----------------------------------------------------------------------------
{
    if( fileFormat == wxString( wxT( "text" ) ) )
    {
        return lffText;
    }
    else if( fileFormat == wxString( wxT( "mvlog" ) ) )
    {
        return lffMVLOG;
    }
    return lffXML;
}

