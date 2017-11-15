//-----------------------------------------------------------------------------
#include <algorithm>
#include <apps/Common/wxAbstraction.h>
#include "PackageDescriptionParser.h"
#include <map>

typedef std::map<std::string, std::string> StringMap;

using namespace std;

//-----------------------------------------------------------------------------
bool operator<( const Version& a, const Version& b )
//-----------------------------------------------------------------------------
{
    if( a.major_ < b.major_ )
    {
        return true;
    }
    else if( a.major_ == b.major_ )
    {
        if( a.minor_ < b.minor_ )
        {
            return true;
        }
        else if( a.minor_ == b.minor_ )
        {
            if( a.subMinor_ < b.subMinor_ )
            {
                return true;
            }
            else if( a.subMinor_ == b.subMinor_ )
            {
                if( a.release_ < b.release_ )
                {
                    return true;
                }
            }
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
bool operator>( const Version& a, const Version& b )
//-----------------------------------------------------------------------------
{
    return ( !( a < b ) && !( a == b ) );
}

//-----------------------------------------------------------------------------
bool operator>=( const Version& a, const Version& b )
//-----------------------------------------------------------------------------
{
    return ( a > b ) || ( a == b );
}

//-----------------------------------------------------------------------------
bool operator==( const Version& a, const Version& b )
//-----------------------------------------------------------------------------
{
    return ( ( a.major_ == b.major_ ) && ( a.minor_ == b.minor_ ) && ( a.subMinor_ == b.subMinor_ ) && ( a.release_ == b.release_ ) );
}

//-----------------------------------------------------------------------------
bool GetNextVersionNumber( wxString& str, long& number )
//-----------------------------------------------------------------------------
{
    wxString numberString = str.BeforeFirst( wxT( '.' ) );
    str = str.AfterFirst( wxT( '.' ) );
    return numberString.ToLong( &number );
}

//-----------------------------------------------------------------------------
bool ExtractStringAttribute( const StringMap& attrMap, const std::string& attrName, wxString& destination )
//-----------------------------------------------------------------------------
{
    StringMap::const_iterator it = attrMap.find( attrName );
    if( it != attrMap.end() )
    {
        destination = ConvertedString( it->second );
    }
    return ( ( it == attrMap.end() ) ? false : true );
}

//-----------------------------------------------------------------------------
Version VersionFromString( const wxString& versionAsString )
//-----------------------------------------------------------------------------
{
    Version version;
    wxString tmp( versionAsString );
    GetNextVersionNumber( tmp, version.major_ );
    if( !tmp.empty() )
    {
        GetNextVersionNumber( tmp, version.minor_ );
        if( !tmp.empty() )
        {
            GetNextVersionNumber( tmp, version.subMinor_ );
            if( !tmp.empty() )
            {
                GetNextVersionNumber( tmp, version.release_ );
            }
        }
    }
    return version;
}

//-----------------------------------------------------------------------------
bool IsVersionWithinRange( const Version& version, const Version& minVersion, const Version& maxVersion )
//-----------------------------------------------------------------------------
{
    if( version < minVersion )
    {
        return false;
    }

    if( version > maxVersion )
    {
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
bool ExtractVersionInformation( const StringMap& attrMap, const std::string& tagName, Version& versionEntry )
//-----------------------------------------------------------------------------
{
    StringMap::const_iterator it = attrMap.find( tagName );
    if( it != attrMap.end() )
    {
        versionEntry = VersionFromString( ConvertedString( it->second ) );
    }
    return ( ( it == attrMap.end() ) ? false : true );
}

const string PackageDescriptionFileParser::m_ATTR_NAME( "Name" );
const string PackageDescriptionFileParser::m_ATTR_TYPE( "Type" );
const string PackageDescriptionFileParser::m_ATTR_DESCRIPTION( "Description" );
const string PackageDescriptionFileParser::m_ATTR_VERSION( "Version" );
const string PackageDescriptionFileParser::m_ATTR_REVISION_MIN( "RevisionMin" );
const string PackageDescriptionFileParser::m_ATTR_REVISION_MAX( "RevisionMax" );

//-----------------------------------------------------------------------------
PackageDescriptionFileParser::TTagType PackageDescriptionFileParser::GetTagType( const char* tagName ) const
//-----------------------------------------------------------------------------
{
    if( strcmp( tagName, "PackageDescription" ) == 0 )
    {
        return ttPackageDescription;
    }
    else if( strcmp( tagName, "File" ) == 0 )
    {
        return ttFile;
    }
    else if( strcmp( tagName, "SuitableProductKey" ) == 0 )
    {
        return ttSuitableProductKey;
    }
    return ttInvalid;
}

//-----------------------------------------------------------------------------
void PackageDescriptionFileParser::OnPostCreate( void )
//-----------------------------------------------------------------------------
{
    m_packageDescriptionVersion.major_ = 1;
    m_packageDescriptionVersion.minor_ = 0;
    EnableStartElementHandler();
}

//-----------------------------------------------------------------------------
void PackageDescriptionFileParser::OnStartElement( const XML_Char* pszName, const XML_Char** papszAttrs )
//-----------------------------------------------------------------------------
{
    if( ( m_packageDescriptionVersion.major_ != 1 ) ||
        ( m_packageDescriptionVersion.minor_ != 0 ) )
    {
        m_lastError = wxString::Format( wxT( "Unsupported package description version: %ld.%ld.\n" ), m_packageDescriptionVersion.major_, m_packageDescriptionVersion.minor_ );
        return;
    }

    // build attribute map
    StringMap attrMap;
    int i = 0;
    while( papszAttrs[i] )
    {
        attrMap.insert( pair<string, string>( papszAttrs[i], papszAttrs[i + 1] ) );
        i += 2;
    }

    switch( GetTagType( pszName ) )
    {
    case ttPackageDescription:
        {
            if( !ExtractVersionInformation( attrMap, m_ATTR_VERSION, m_packageDescriptionVersion ) )
            {
                m_lastError = wxString( wxT( "Version information tag is missing in package format description" ) );
                m_packageDescriptionVersion = Version();
                break;
            }
        }
        break;
    case ttFile:
        {
            FileEntry entry;
            if( !ExtractStringAttribute( attrMap, m_ATTR_DESCRIPTION, entry.description_ ) ||
                !ExtractStringAttribute( attrMap, m_ATTR_NAME, entry.name_ ) ||
                !ExtractStringAttribute( attrMap, m_ATTR_TYPE, entry.type_ ) ||
                !ExtractVersionInformation( attrMap, m_ATTR_VERSION, entry.version_ ) )
            {
                m_lastError = wxString( wxT( "At least one required attribute for the file tag is missing" ) );
                break;
            }
            m_results.push_back( entry );
        }
        break;
    case ttSuitableProductKey:
        {
            if( m_results.empty() )
            {
                m_lastError = wxString( wxT( "Orphaned 'SuitableProductKey' tag detected" ) );
                break;
            }

            SuitableProductKey data;
            if( !ExtractStringAttribute( attrMap, m_ATTR_NAME, data.name_ ) ||
                !ExtractVersionInformation( attrMap, m_ATTR_REVISION_MAX, data.revisionMax_ ) ||
                !ExtractVersionInformation( attrMap, m_ATTR_REVISION_MIN, data.revisionMin_ ) )
            {
                // at least one required attribute is missing
                break;
            }

            m_results[m_results.size() - 1].suitableProductKeys_.insert( make_pair( data.name_, data ) );
        }
        break;
    default:
        // invalid tag...
        break;
    }
}
