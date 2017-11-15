//-----------------------------------------------------------------------------
#ifndef EpcsH
#define EpcsH EpcsH
//-----------------------------------------------------------------------------
//  Enhanced Programming Configuration Serial

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

//-----------------------------------------------------------------------------
//Operation codes for serial configuration devices
//-----------------------------------------------------------------------------
#define EpcsWrite_ENA       0x06    /* Write enable */
#define EpcsWrite_DIS       0x04    /* Write disable */
#define EpcsRead_STAT       0x05    /* Read status */
#define EpcsRead_BYTES      0x03    /* Read bytes */
#define EpcsFastRead_BYTES  0x0b    /* Fast Read bytes */
#define EpcsRead_ID         0xab    /* Read silicon id */
#define EpcsWrite_STAT      0x01    /* Write status */
#define EpcsWrite_BYTES     0x02    /* Write bytes */
#define EpcsErase_BULK      0xc7    /* Erase entire device */
#define EpcsErase_SECT      0xd8    /* Erase sector */

//-----------------------------------------------------------------------------
//Device status register bits
//-----------------------------------------------------------------------------
#define EPCS_STATUS_WIP     (1<<0)  /* Write in progress */
#define EPCS_STATUS_WEL     (1<<1)  /* Write enable latch */

#define EPCS_TIMEOUT        100 /* 100 msec timeout */

typedef struct SEpcsDevInfo
{
    const char*  name;      /* Device name */
    unsigned char   id;     /* Device silicon id */
    unsigned char   size;       /* Total size log2(bytes)*/
    unsigned char   num_sects;  /* Number of sectors */
    unsigned char   sz_sect;    /* Sector size log2(bytes) */
    unsigned char   sz_page;    /* Page size log2(bytes) */
    unsigned char   prot_mask;  /* Protection mask */
} TEpcsDevInfo;

typedef volatile struct SNiosSpi
{
    unsigned    rxdata;     /* Rx data reg */
    unsigned    txdata;     /* Tx data reg */
    unsigned    status;     /* Status reg */
    unsigned    control;    /* Control reg */
    unsigned    reserved;   /* (master only) */
    unsigned    slaveselect;    /* SPI slave select mask (master only) */
} TNiosSpi;

#define REG_OFF_SPI_RXDATA  0
#define REG_OFF_SPI_TXDATA  0x4
#define REG_OFF_SPI_STATUS  0x8
#define REG_OFF_SPI_CONTROL 0xC

/* status register */
#define NIOS_SPI_ROE        (1 << 3)    /* rx overrun */
#define NIOS_SPI_TOE        (1 << 4)    /* tx overrun */
#define NIOS_SPI_TMT        (1 << 5)    /* tx empty */
#define NIOS_SPI_TRDY       (1 << 6)    /* tx ready */
#define NIOS_SPI_RRDY       (1 << 7)    /* rx ready */
#define NIOS_SPI_E          (1 << 8)    /* exception */

/* control register */
#define NIOS_SPI_IROE       (1 << 3)    /* rx overrun int ena */
#define NIOS_SPI_ITOE       (1 << 4)    /* tx overrun int ena */
#define NIOS_SPI_ITRDY      (1 << 6)    /* tx ready int ena */
#define NIOS_SPI_IRRDY      (1 << 7)    /* rx ready int ena */
#define NIOS_SPI_IE         (1 << 8)    /* exception int ena */
#define NIOS_SPI_SSO        (1 << 10)   /* override SS_n output */

//-----------------------------------------------------------------------------
typedef struct _RegisterAccess
//-----------------------------------------------------------------------------
{
    unsigned long Offset;
    unsigned long Data;
} TRegisterAccess;

//-----------------------------------------------------------------------------
class CEpcs
//-----------------------------------------------------------------------------
{
public:
    explicit    CEpcs( Device* pDev );
    ~CEpcs();
    int WriteDataToFlash( unsigned char* pFlashData, int bytes );
    void DeviceConfigure( void );
    void Info( void );
    void Erase( unsigned long start, unsigned long end );
    void BulkErase( void )
    {
        EpcsBulkErase();
    }
    void Protect( bool boProtect );
    void Read( unsigned char* pFlashData, unsigned long off, unsigned long cnt );
    int Write( unsigned char* pFlashData, unsigned long off, unsigned long cnt, bool verify );
    int Verify( unsigned char* pFlashData, unsigned long off, unsigned long cnt );
    bool GetSPIDeviceInfo( TEpcsDevInfo& deviceInfo );
    bool IsProtected( void )
    {
        return boProtected_;
    }
    void SectorInfo( void );
    void DbOutput( const char* format, ... );

private:
    int EpcsCS( int assert );
    int EpcsTX( unsigned char c );
    int EpcsRX( void );
    unsigned char EpcsBitRev( unsigned char c );
    void EpcsRcv( unsigned char* pDst, int len );
    void EpcsRRcv( unsigned char* pDst, int len );
    void EpcsSnd( unsigned char* pSrc, int len );
    void EpcsRSnd( unsigned char* pSrc, int len );
    void EpcsRSndEx( unsigned char* pSrc, int len );
    void EpcsWrEnable( void );
    unsigned char EpcsStatusRD( void );
    void EpcsStatusWR( unsigned char status );
    const TEpcsDevInfo* EpcsDevFind( void );
    int EpcsCfgsz( void );
    int EpcsErase( unsigned start, unsigned end );
    int EpcsRead( unsigned char* pFlashData, unsigned long off, unsigned long cnt );
    int EpcsWrite( unsigned char* pFlashData, unsigned long off, unsigned long cnt );
    int EpcsBulkErase( void );
    int EpcsVerify( unsigned char* pFlashData, unsigned long off, unsigned long cnt, unsigned long* pErr );
    int EpcsSectErased( int sect, unsigned* pOffset );
    int EpcsReset( void );
    unsigned long ReadSPI( unsigned long off );
    void WriteSPI( unsigned long off, unsigned long data );

private:
    TNiosSpi* pEpcs_;
    const TEpcsDevInfo* pDeviceInfo_;
    bool boProtected_;
    Device* pDev_;
    Method SPIConfig_;
    Method SPIRead_;
    Method SPIWrite_;
    PropertyI SPIOffset_;
    PropertyI SPIData_;
    int TXError_;
    int RXError_;
    int CSError_;
};

//-----------------------------------------------------------------------------
enum eFlashSelect
//-----------------------------------------------------------------------------
{
    eFSSetI2CSwitchToDefaultFlash = 0,
    eFSSetI2CSwitchToUserFlash,
    eFSSetI2CSwitchOn,
    eFSSetI2CSwitchOffAndForceUserFlashSelect
};

void SelectFlashAccess_mvHYPERION( Device* pDev, eFlashSelect access );
bool FlashUpdate_mvHYPERION( Device* pDev, unsigned char* pFlashData, unsigned long flashDataSize, std::ostream& logger );

#endif //EpcsH
