#ifdef _MSC_VER // is Microsoft compiler?
#   if _MSC_VER < 1300  // is 'old' VC 6 compiler?
#       pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#   endif // #if _MSC_VER < 1300
#endif // #ifdef _MSC_VER
#include <algorithm>
#include <apps/Common/exampleHelper.h>
#include <functional>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

using namespace std;
using namespace mvIMPACT::acquire;

//-----------------------------------------------------------------------------
class VinylRecord
//-----------------------------------------------------------------------------
{
private:
    std::string album_;
    unsigned int timesPlayed_;
public:
    explicit VinylRecord( const std::string& album ) : album_( album ), timesPlayed_( 0 ) {}
    bool display( void )
    {
        cout << "This record is called '" << getAlbum() << "' and since creation it has been played " << getTimesPlayed() << " times." << endl;    // if this function is declared 'const' or does not return a value the 'for_each' calls does not compile with VS 6...
        return true;
    }
    const std::string& getAlbum( void ) const
    {
        return album_;
    }
    unsigned int getTimesPlayed( void ) const
    {
        return timesPlayed_;
    }
    void play( void )
    {
        ++timesPlayed_;
    }
};

typedef vector<VinylRecord*> RecordStack;

//-----------------------------------------------------------------------------
class MyCallback : public ComponentCallback
//-----------------------------------------------------------------------------
{
    unsigned int hitCount_;
public:
    explicit MyCallback( void* pUserData = 0 ) : ComponentCallback( pUserData ),
        hitCount_( 0 ) {}
    // Please note that this function might be executed from ANY thread context, which is
    // most likely not the same as used by the application thus appropriate mechanisms to ensure
    // correct execution must be implemented by an application(e.g. GUI applications might send an
    // event to the main thread instead of directly accessing GUI elements).
    virtual void execute( Component& c, void* pUserData )
    {
        // get access to the user data previously associated with the component that just executes the callback
        RecordStack* pRecordStack = reinterpret_cast<RecordStack*>( pUserData );
        // do some work with the user data object
        const RecordStack::size_type recordCount = pRecordStack->size();
        cout << "Component '" << c.name() << "' did just cause a callback. It's the " << ++hitCount_ << " time this function got called." << endl
             << "This callback carries " << recordCount << " good old vinyl records with it." << endl;
        RecordStack::const_iterator it = pRecordStack->begin();
        const RecordStack::const_iterator itEND = pRecordStack->end();
        unsigned int i = 0;
        while( it != itEND )
        {
            cout << "  There is a record called '" << ( *it )->getAlbum() << "'";
            if( hitCount_ % recordCount == i )
            {
                cout << ", which I will play now";
                ( *it )->play();
            }
            cout << endl;
            ++it;
            ++i;
        }
    }
};

//-----------------------------------------------------------------------------
int main( int /*argc*/, char* /*argv*/[] )
//-----------------------------------------------------------------------------
{
    DeviceManager devMgr;
    Device* pDev = getDeviceFromUserInput( devMgr );
    if( !pDev )
    {
        cout << "Unable to continue!";
        cout << "Press [ENTER] to end the application" << endl;
        cin.get();
        return 0;
    }

    cout << "Initialising the device. This might take some time..." << endl;
    try
    {
        pDev->open();
    }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while opening the device(error code: " << e.getErrorCode() << ")." << endl
             << "Press [ENTER] to end the application" << endl;
        cin.get();
        return 0;
    }

    CameraSettingsBase cs( pDev );
    SystemSettings ss( pDev );

    // create some user data to attach to the callback
    vector<VinylRecord*> vinylRecords;
    vinylRecords.push_back( new VinylRecord( "The number of the Beast" ) );
    vinylRecords.push_back( new VinylRecord( "Holy Diver" ) );
    vinylRecords.push_back( new VinylRecord( "Master of Reality" ) );

    // create the callback with user data
    MyCallback myCallback( &vinylRecords );
    // attach the callback to some features
    myCallback.registerComponent( cs.imageRequestTimeout_ms );
    myCallback.registerComponent( ss.requestCount );

    const vector<VinylRecord*>::size_type vinylRecordCnt = vinylRecords.size();
    cout << "=== List of records BEFORE callbacks ===" << endl;
    for_each( vinylRecords.begin(), vinylRecords.end(), mem_fun( &VinylRecord::display ) );
    // provoke some callbacks by modifying the properties we just registered callbacks for.
    // Whenever one of this features is changed in any way the previously attached callback
    // handler will be called.
    for( int i = 1; i < 30; i++ )
    {
        cs.imageRequestTimeout_ms.write( i );
        ss.requestCount.write( i );
    }

    // now verify that the callbacks have actually been executed by displaying the times all records have
    // been played within the callback handlers
    cout << "=== List of records AFTER callbacks ===" << endl;
    for_each( vinylRecords.begin(), vinylRecords.end(), mem_fun( &VinylRecord::display ) );
    // clean up
    myCallback.unregisterComponent( ss.requestCount );
    myCallback.unregisterComponent( cs.imageRequestTimeout_ms );
    for( vector<VinylRecord*>::size_type j = 0; j < vinylRecordCnt; j++ )
    {
        delete vinylRecords[j];
    }
    vinylRecords.clear();

    cout << "Press [ENTER] to end the application" << endl;
    cin.get();
    return 0;
}
