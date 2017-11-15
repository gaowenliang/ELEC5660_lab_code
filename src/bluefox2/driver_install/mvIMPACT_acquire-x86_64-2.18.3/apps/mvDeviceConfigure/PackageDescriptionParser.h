//-----------------------------------------------------------------------------
#ifndef PackageDescriptionParserH
#define PackageDescriptionParserH PackageDescriptionParserH
//-----------------------------------------------------------------------------
#include <string.h>
#include <Toolkits/expat/include/ExpatImpl.h>
#include <map>
#include <vector>
#include "wx/wx.h"

//-----------------------------------------------------------------------------
struct Version
//-----------------------------------------------------------------------------
{
    long major_;
    long minor_;
    long subMinor_;
    long release_;
    explicit Version() : major_( -1 ), minor_( -1 ), subMinor_( -1 ), release_( 0 ) {}
    explicit Version( long ma, long mi, long smi, long re ) : major_( ma ), minor_( mi ), subMinor_( smi ), release_( re ) {}
};

bool operator<( const Version& a, const Version& b );
bool operator>( const Version& a, const Version& b );
bool operator>=( const Version& a, const Version& b );
bool operator==( const Version& a, const Version& b );
Version VersionFromString( const wxString& versionAsString );
bool IsVersionWithinRange( const Version& version, const Version& minVersion, const Version& maxVersion );

//-----------------------------------------------------------------------------
struct SuitableProductKey
//-----------------------------------------------------------------------------
{
    wxString name_;
    Version revisionMin_;
    Version revisionMax_;
    explicit SuitableProductKey() : name_() {}
};

//-----------------------------------------------------------------------------
struct FileEntry
//-----------------------------------------------------------------------------
{
    wxString name_;
    wxString type_;
    wxString description_;
    Version version_;
    std::map<wxString, SuitableProductKey> suitableProductKeys_;
    explicit FileEntry() : name_(), type_(), description_() {}
};

typedef std::vector<FileEntry> FileEntryContainer;

//-----------------------------------------------------------------------------
class PackageDescriptionFileParser : public CExpatImpl<PackageDescriptionFileParser>
//-----------------------------------------------------------------------------
{
    // internal types
    enum TTagType
    {
        ttInvalid,
        ttPackageDescription,
        ttFile,
        ttSuitableProductKey
    };
    static const std::string    m_ATTR_NAME;
    static const std::string    m_ATTR_TYPE;
    static const std::string    m_ATTR_DESCRIPTION;
    static const std::string    m_ATTR_VERSION;
    static const std::string    m_ATTR_REVISION_MIN;
    static const std::string    m_ATTR_REVISION_MAX;

    wxString                    m_lastError;
    FileEntryContainer          m_results;
    Version                     m_packageDescriptionVersion;

    TTagType                    GetTagType      ( const char* tagName ) const;
public:
    void                        OnPostCreate    ( void );
    void                        OnStartElement  ( const XML_Char* pszName, const XML_Char** papszAttrs );
    const wxString&             GetLastError    ( void ) const
    {
        return m_lastError;
    }
    const FileEntryContainer&   GetResults      ( void ) const
    {
        return m_results;
    }
};

#endif // PackageDescriptionParserH
