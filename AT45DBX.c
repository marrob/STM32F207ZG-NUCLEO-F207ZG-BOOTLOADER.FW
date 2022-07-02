/******************** (C) COPYRIGHT 2016 marrob Design *************************
* File Name          : LiveLED.c
* Author             : Margit Róbert
* Date First Issued  : 2016.02.14
* Description        : AT45DB161E
********************************************************************************/
/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "StringPlus.h"
#include "Tick.h"
#include "AT45DBX.h"

/* Private define ------------------------------------------------------------*/
#define AT45DBX_CMDA_RD_PAGE              0xD2        //!< Main Memory Page Read (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_ARRAY_LEG         0xE8        //!< Continuous Array Read, Legacy Command (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_ARRAY_LF_SM       0x03        //!< Continuous Array Read, Low-Frequency Mode (Serial Mode).
#define AT45DBX_CMDA_RD_ARRAY_AF_SM       0x0B        //!< Continuous Array Read, Any-Frequency Mode (Serial Mode).
#define AT45DBX_CMDA_RD_SECTOR_PROT_REG   0x32        //!< Read Sector Protection Register (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_SECTOR_LKDN_REG   0x35        //!< Read Sector Lockdown Register (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_SECURITY_REG      0x77        //!< Read Security Register (Serial/8-bit Mode).

//AT45DBX Group B Commands
#define AT45DBX_CMDB_ER_PAGE              0x81        //!< Page Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_ER_BLOCK             0x50        //!< Block Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_ER_SECTOR            0x7C        //!< Sector Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_ER_CHIP              0xC794809A  //!< Chip Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_XFR_PAGE_TO_BUF1     0x53        //!< Main Memory Page to Buffer 1 Transfer (Serial/8-bit Mode).
#define AT45DBX_CMDB_XFR_PAGE_TO_BUF2     0x55        //!< Main Memory Page to Buffer 2 Transfer (Serial/8-bit Mode).
#define AT45DBX_CMDB_CMP_PAGE_TO_BUF1     0x60        //!< Main Memory Page to Buffer 1 Compare (Serial/8-bit Mode).
#define AT45DBX_CMDB_CMP_PAGE_TO_BUF2     0x61        //!< Main Memory Page to Buffer 2 Compare (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF1_TO_PAGE_ER   0x83        //!< Buffer 1 to Main Memory Page Program with Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF2_TO_PAGE_ER   0x86        //!< Buffer 2 to Main Memory Page Program with Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF1_TO_PAGE      0x88        //!< Buffer 1 to Main Memory Page Program without Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF2_TO_PAGE      0x89        //!< Buffer 2 to Main Memory Page Program without Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_PAGE_TH_BUF1      0x82        //!< Main Memory Page Program through Buffer 1 (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_PAGE_TH_BUF2      0x85        //!< Main Memory Page Program through Buffer 2 (Serial/8-bit Mode).
#define AT45DBX_CMDB_RWR_PAGE_TH_BUF1     0x58        //!< Auto Page Rewrite through Buffer 1 (Serial/8-bit Mode).
#define AT45DBX_CMDB_RWR_PAGE_TH_BUF2     0x59        //!< Auto Page Rewrite through Buffer 2 (Serial/8-bit Mode).

//AT45DBX Group C Commands
#define AT45DBX_CMDC_RD_BUF1_LF_SM        0xD1        //!< Buffer 1 Read, Low-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF2_LF_SM        0xD3        //!< Buffer 2 Read, Low-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF1_AF_SM        0xD4        //!< Buffer 1 Read, Any-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF2_AF_SM        0xD6        //!< Buffer 2 Read, Any-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF1_AF_8M        0x54        //!< Buffer 1 Read, Any-Frequency Mode (8-bit Mode).
#define AT45DBX_CMDC_RD_BUF2_AF_8M        0x56        //!< Buffer 2 Read, Any-Frequency Mode (8-bit Mode).
#define AT45DBX_CMDC_WR_BUF1              0x84        //!< Buffer 1 Write (Serial/8-bit Mode).
#define AT45DBX_CMDC_WR_BUF2              0x87        //!< Buffer 2 Write (Serial/8-bit Mode).
#define AT45DBX_CMDC_RD_STATUS_REG        0xD7        //!< Status Register Read (Serial/8-bit Mode).        AT45DB161D!
#define AT45DBX_CMDC_RD_MNFCT_DEV_ID_SM   0x9F        //!< Manufacturer and Device ID Read (Serial Mode). 

//AT45DBX Group D Commands
#define AT45DBX_CMDD_EN_SECTOR_PROT       0x3D2A7FA9  //!< Enable Sector Protection (Serial/8-bit Mode).
#define AT45DBX_CMDD_DIS_SECTOR_PROT      0x3D2A7F9A  //!< Disable Sector Protection (Serial/8-bit Mode).
#define AT45DBX_CMDD_ER_SECTOR_PROT_REG   0x3D2A7FCF  //!< Erase Sector Protection Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_PR_SECTOR_PROT_REG   0x3D2A7FFC  //!< Program Sector Protection Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_LKDN_SECTOR          0x3D2A7F30  //!< Sector Lockdown (Serial/8-bit Mode).
#define AT45DBX_CMDD_PR_SECURITY_REG      0x9B000000  //!< Program Security Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_PR_CONF_REG          0x3D2A80A6  //!< Program Configuration Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_DEEP_PWR_DN          0xB9        //!< Deep Power-down (Serial/8-bit Mode).
#define AT45DBX_CMDD_RSM_DEEP_PWR_DN      0xAB        //!< Resume from Deep Power-down (Serial/8-bit Mode).

//Bit-Masks and Values for the Status Register
#define AT45DBX_MSK_BUSY                  0x80        //!< Busy status bit-mask.
#define AT45DBX_BUSY                      0x00        //!< Busy status value (0x00 when busy, 0x80 when ready).
#define AT45DBX_MSK_DENSITY               0x3C        //!< Device density bit-mask.
#define AT45DBX_MSK_PAGE_SIZE             0x01        // 0-> 528 byte, 1-> 512 byte
#define AT45DBX_PAGE_512_BYTE             0x01        //                 



#if AT45DBX_MEM_SIZE == AT45DBX_2MB
#define AT45DBX_DENSITY                   0x2C        //!< Device density value.
#define AT45DBX_BYTE_ADDR_BITS            10          //!< Address bits for byte position within buffer.
#endif

/* Private function prototypes -----------------------------------------------*/
static uint8_t ReadMnfId(At45dbxHandle_Type *hnd, uint8_t *id, size_t size, uint32_t timeout);
static uint8_t ReadSecurityRegister(At45dbxHandle_Type *hnd, uint8_t *reg, size_t size, uint32_t timeout);
static uint8_t PageSizeConfiguration(At45dbxHandle_Type *hnd, uint8_t setPage512bytes, uint32_t timeout);
static uint8_t WaitForReady(At45dbxHandle_Type *hnd, uint32_t timeout);
/* Includes ------------------------------------------------------------------*/
#if AT45DBX_DEBUG_LEVEL > 0
static char StringBuffer[1024];
#endif

/*****************************************************************
512 page mode:

The first 12 bits (A20 - A9) of the 21-bit address sequence specify
which page of the main memory
array to read, and the last nine bits (A8 - A0) of the 21-bit
address sequence specify the starting byte address within that page.

|<    Byte1        ->|<-     Byte2         ->|<-          Byte 3   ->|<-      Byte 4       ->|
                           |<----Main Memory Array A21-A10------->|<--In Page Addr B9-B0---->| = 9 + 11 = 21
|31|30|29|28|27|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10|09|08|07|06|05|04|03|02|01|00|
*****************************************************************/

/*****************************************************************
Functional Name: At45dbxInit
return: AT45DBX_OK
*****************************************************************/
uint8_t At45dbxInit(At45dbxHandle_Type *hnd, At45dbxInterface_Type *itf)
{
    hnd->ChipSelect = itf->ChipSelect;
    hnd->Reset = itf->Reset;
    hnd->IsConfigured = 0;

    hnd->Reset(AT45DBX_PIN_LOW);
    DelayUs(10);
    hnd->Reset(AT45DBX_PIN_HIGH);
    
    if(ReadMnfId(hnd, hnd->MnfId, AT45DBX_MNF_ID_SIZE, AT45DBX_TIMEOUT_MS) != AT45DBX_OK)
        return AT45DBX_FAIL;
    if(ReadSecurityRegister(hnd, (uint8_t*)&hnd->SecurtyRegister, sizeof(hnd->SecurtyRegister),AT45DBX_TIMEOUT_MS) != AT45DBX_OK)
        return AT45DBX_FAIL;
    
    uint8_t status = 0;
    if(ReadStatus(hnd, &status, AT45DBX_TIMEOUT_MS)!= AT45DBX_OK)
        return AT45DBX_FAIL;
   
    if((status & AT45DBX_MSK_PAGE_SIZE)!= AT45DBX_PAGE_512_BYTE)
    {
       if(PageSizeConfiguration(hnd,AT45DBX_PAGE_512_BYTE,AT45DBX_TIMEOUT_MS)!= AT45DBX_OK)
           return AT45DBX_FAIL;
    }
    hnd->IsConfigured = 1;
    return AT45DBX_OK;
}

/*****************************************************************
Functional Name: At45dbxPageTransferModifyWrite
return: AT45DBX_OK
*****************************************************************/
uint8_t At45dbxPageTransferModifyWrite(At45dbxHandle_Type *hnd, uint32_t address, const void *data, size_t size, uint32_t timeout)
{
    uint8_t status = 0;
    At45dbxDbgLog("Read To Buffer, Modify Buffer, Write:");
    At45dbxDbgLog("Address:0x%08X.",address);
    At45dbxDbgLog("Size:%dbyte(s).",size);
    
    if(size == 0)
        return AT45DBX_OK;

    //---------------------------------------------------------------
    hnd->ChipSelect(AT45DBX_PIN_LOW);
    //                      CMD 0x53             |      X,X,X A20-A16   |       A15-A8   |    A7-A0 
    uint8_t cmd[] = {AT45DBX_CMDB_XFR_PAGE_TO_BUF1, (address >> 16)& 0x1F, address >> 0x08 , address};
    At45dbxDbgLog("AT45DBX_CMDB_XFR_PAGE_TO_BUF1:%s.", 
    StringPlusDataToHexaString(cmd, StringBuffer, sizeof(cmd)));
    
    if(At45dbxSpiWriteCallback(hnd, cmd, sizeof(cmd), timeout)!= AT45DBX_OK)
    {
        hnd->ChipSelect(AT45DBX_PIN_HIGH);
        At45dbxDbgLog("Fail."); 
        return AT45DBX_FAIL;
    }
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    //---------------------------------------------------------------
    if((status = WaitForReady(hnd, timeout)) != AT45DBX_OK)
        return status;
    //---------------------------------------------------------------
    //                      CMD  0x82              |      X,X,X A20-A16   |       A15-A8   |    A7-A0 
    uint8_t cmd1[] = {AT45DBX_CMDB_PR_PAGE_TH_BUF1, (address >> 16)& 0x1F, address >> 0x08 , address,};
    At45dbxDbgLog("AT45DBX_CMDB_PR_PAGE_TH_BUF1:%s.", 
    StringPlusDataToHexaString(cmd1, StringBuffer, sizeof(cmd1)));
    hnd->ChipSelect(AT45DBX_PIN_LOW);

    if(At45dbxSpiWriteCallback(hnd, cmd1, sizeof(cmd1), timeout)!= AT45DBX_OK)
    {
        hnd->ChipSelect(AT45DBX_PIN_HIGH);
        At45dbxDbgLog("Fail."); 
        return AT45DBX_FAIL;
    }
    //-----------------------
    if(At45dbxSpiWriteCallback(hnd, data, size, timeout)!= AT45DBX_OK)
    {
        hnd->ChipSelect(AT45DBX_PIN_HIGH);
        At45dbxDbgLog("Fail."); 
        return AT45DBX_FAIL;
    }
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    At45dbxDbgLog("Data:%s.", 
    StringPlusDataToLimitedHexaString((uint8_t*)data, 16, StringBuffer, size));
    //---------------------------------------------------------------
    if((status = WaitForReady(hnd, timeout)) != AT45DBX_OK)
        return status;
    
    return AT45DBX_OK;
}

/*****************************************************************
Functional Name: At45dbxWrite
return: AT45DBX_OK
*****************************************************************/
uint8_t At45dbxWrite(At45dbxHandle_Type *hnd, uint32_t writeAddr, const void *pData, size_t size, uint32_t timeout)
{
uint32_t addr = 0, count = 0, temp = 0 ,numOfPage = 0, numOfSingle = 0;

    uint8_t *ptr = (uint8_t*)pData;
    addr = writeAddr % AT45DBX_PAGE_SIZE;   //(ha ez nulla akkor page-re van igazítva)
    count = AT45DBX_PAGE_SIZE - addr;       //Adott page-en ennyi bájtot írhat még (hogy ne lépjen át a következöre)
    numOfPage =  size / AT45DBX_PAGE_SIZE;  //Összesen ennyi page-et ír.
    numOfSingle = size % AT45DBX_PAGE_SIZE; //Page-en kívül ennyi bájtot ír.
  
    if(addr == 0)
    {
        //Address is Page aligned.      
        if(numOfPage == 0)//size < AT45DBX_PAGE_SIZE"
        {
            At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, size, timeout);
        }
        else
        {
            while(numOfPage--) //size > AT45DBX_PAGE_SIZE
            {
                At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, AT45DBX_PAGE_SIZE, timeout);
                writeAddr += AT45DBX_PAGE_SIZE;
                ptr += AT45DBX_PAGE_SIZE;
            }
            At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, numOfSingle, timeout);
        }
    }
    else 
    { //Address is NOT Page aligned.
        if(numOfPage == 0)
        {//size < AT45DBX_PAGE_SIZE.
             
            if(numOfSingle > count)
            {//Data in 2 Page... size + Address > AT45DBX_PAGE_SIZE
                temp = numOfSingle - count;
                At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, count, timeout);
                writeAddr += count;
                ptr += count;
                At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, temp, timeout);
            }
            else
            { //size + Address < AT45DBX_PAGE_SIZE
                At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, size, timeout);
            }
        }
        else
        {
            //size > AT45DBX_PAGE_SIZE
            size -= count;
            numOfPage =  size / AT45DBX_PAGE_SIZE;
            numOfSingle = size % AT45DBX_PAGE_SIZE;
          
            At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, count, timeout);
            writeAddr += count;
            ptr += count;
         
            while(numOfPage--)
            {
                At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, AT45DBX_PAGE_SIZE, timeout);
                writeAddr +=  AT45DBX_PAGE_SIZE;
                ptr += AT45DBX_PAGE_SIZE;  
            }
            if(numOfSingle != 0)
            {
                At45dbxPageTransferModifyWrite(hnd, writeAddr, ptr, numOfSingle, timeout);
            }
        }
    }

return AT45DBX_OK;
}

/*****************************************************************
Functional Name: At45dbxRead
return: AT45DBX_OK
*****************************************************************/
uint8_t At45dbxRead(At45dbxHandle_Type *hnd, uint32_t readAddress, void *pData, size_t size, uint32_t timeout)
{
uint32_t addr = 0, count = 0, temp = 0 ,numOfPage = 0, numOfSingle = 0;
    
    uint8_t *ptr = (uint8_t*)pData;
    addr = readAddress % AT45DBX_PAGE_SIZE;   //(ha ez nulla akkor page-re van igazítva)
    count = AT45DBX_PAGE_SIZE - addr;         //Adott page-en ennyi bájtot dolgozhat (hogy ne lépjen át a következöre)
    numOfPage =  size / AT45DBX_PAGE_SIZE;    //Összesen ennyi page-et ír.
    numOfSingle = size % AT45DBX_PAGE_SIZE;   //Page-en kívül ennyi bájtot ír.
  
    if(addr == 0)
    {
        //Address is Page aligned.
        if(numOfPage == 0)//size < AT45DBX_PAGE_SIZE"
        {
            At45dbxPageRead(hnd, readAddress, ptr, size, timeout);
        }
        else
        {
            while(numOfPage--) //size > AT45DBX_PAGE_SIZE
            {
                At45dbxPageRead(hnd, readAddress, ptr, AT45DBX_PAGE_SIZE, timeout);
                readAddress += AT45DBX_PAGE_SIZE;
                ptr += AT45DBX_PAGE_SIZE;
            }
            if(numOfSingle!= 0)
                At45dbxPageRead(hnd, readAddress, ptr, numOfSingle, timeout);
        }
    }
    else 
    { //Address is NOT Page aligned.
        if(numOfPage == 0)
        {//size < AT45DBX_PAGE_SIZE.
             
            if(numOfSingle > count)
            {//Data in 2 Page... size + Address > AT45DBX_PAGE_SIZE
                temp = numOfSingle - count;
                At45dbxPageRead(hnd, readAddress, ptr, count, timeout);
                readAddress += count;
                ptr += count; 
                At45dbxPageRead(hnd, readAddress, ptr, temp, timeout);
            }
            else
            { //size + Address < AT45DBX_PAGE_SIZE
                At45dbxPageRead(hnd, readAddress, ptr, size, timeout);
            }
        }
        else
        {
            //size > AT45DBX_PAGE_SIZE
            size -= count;
            numOfPage =  size / AT45DBX_PAGE_SIZE;
            numOfSingle = size % AT45DBX_PAGE_SIZE;
          
            At45dbxPageRead(hnd, readAddress, ptr, count, timeout);
            readAddress += count;
            ptr += count;
         
            while(numOfPage--)
            {
                At45dbxPageRead(hnd, readAddress, ptr, AT45DBX_PAGE_SIZE, timeout);
                readAddress +=  AT45DBX_PAGE_SIZE;
                ptr += AT45DBX_PAGE_SIZE;  
            }
            if(numOfSingle != 0)
            {
                At45dbxPageRead(hnd, readAddress, ptr, numOfSingle, timeout);
            }
        }
    }
return AT45DBX_OK;
}


/*****************************************************************
Functional Name: At45dbxPageRead
return: AT45DBX_OK
*****************************************************************/
uint8_t At45dbxPageRead(At45dbxHandle_Type *hnd, uint32_t address, void *data, size_t size, uint32_t timeout)
{
    uint16_t status = 0;
    At45dbxDbgLog("At45dbxRead:");
    At45dbxDbgLog("Address:0x%08X.",address);
    //---------------------------------------------------------------
    
    //                      CMD          |      X,X,X A20-A16   |       A15-A8   |    A7-A0 | <-  Dummy  ->
    uint8_t cmd[] = {AT45DBX_CMDA_RD_PAGE, (address >> 16)& 0x1F, address >> 0x08 , address, 0x00,0x00,0x00,0x00};
    At45dbxDbgLog("AT45DBX_CMDA_RD_PAGE:%s.", 
    StringPlusDataToHexaString(cmd, StringBuffer, sizeof(cmd)));

    hnd->ChipSelect(AT45DBX_PIN_LOW);
    if(At45dbxSpiWriteCallback(hnd, cmd, sizeof(cmd), timeout)!= AT45DBX_OK)
    {
        hnd->ChipSelect(AT45DBX_PIN_HIGH);
        At45dbxDbgLog("Fail."); 
        return AT45DBX_FAIL;
    }
    //---------------------------
    if(At45dbxSpiReadCallback(hnd, (uint8_t*)data, size, timeout)!= AT45DBX_OK)
    {
        hnd->ChipSelect(AT45DBX_PIN_HIGH);
        At45dbxDbgLog("Fail."); 
        return AT45DBX_FAIL;
    }
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    At45dbxDbgLog("Readed Data:%s.", StringPlusDataToLimitedHexaString((uint8_t*)data, 16, StringBuffer, size));
    //---------------------------------------------------------------
    if((status = WaitForReady(hnd, timeout)) != AT45DBX_OK)
        return status;  
    return AT45DBX_OK;
}
/*****************************************************************
Functional Name: ReadMnfId
return: AT45DBX_OK
*****************************************************************/
static uint8_t ReadMnfId(At45dbxHandle_Type *hnd, uint8_t *id, size_t size, uint32_t timeout)
{
    uint8_t cmd[] = {AT45DBX_CMDC_RD_MNFCT_DEV_ID_SM };
    hnd->ChipSelect(AT45DBX_PIN_LOW);
    if( At45dbxSpiWriteCallback(hnd, cmd, sizeof(cmd), timeout)!= AT45DBX_OK)
        return AT45DBX_FAIL;
    else
        if(At45dbxSpiReadCallback(hnd, id, size, timeout)!= AT45DBX_OK)
            return AT45DBX_FAIL;
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    return AT45DBX_OK;
}
/*****************************************************************
Functional Name: ReadMnfId
return: AT45DBX_OK
*****************************************************************/
uint8_t ReadStatus(At45dbxHandle_Type *hnd, uint8_t *dfstatus, uint32_t timeout)
{
    uint8_t status = AT45DBX_OK;
    uint8_t cmd[] = {AT45DBX_CMDC_RD_STATUS_REG };
    
    hnd->ChipSelect(AT45DBX_PIN_LOW);
    if( At45dbxSpiWriteCallback(hnd, cmd, sizeof(cmd), timeout)!= AT45DBX_OK)
        status = AT45DBX_FAIL;
    else
        if(At45dbxSpiReadCallback(hnd, dfstatus, sizeof(uint8_t), timeout)!= AT45DBX_OK)
            status = AT45DBX_FAIL;
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    return status;
}
/*****************************************************************
Functional Name: WaitForReady
return: AT45DBX_OK
*****************************************************************/
static uint8_t WaitForReady(At45dbxHandle_Type *hnd, uint32_t timeout)
{
    int32_t timestamp;
    uint8_t status  = 0;
    //---------------------------------------------------------------
    TickSetTimestampMs(timestamp);
    do
    {
        if(TickTimeIsElapsedMs(timeout, timestamp))
        {
            At45dbxErrLog("Timeout."); 
            return AT45DBX_TIMEOUT;
        }
        if(ReadStatus(hnd, (uint8_t*)&status, AT45DBX_TIMEOUT_MS)!= AT45DBX_OK)
            return AT45DBX_FAIL;
    }while((status & AT45DBX_MSK_BUSY) == AT45DBX_BUSY);

return AT45DBX_OK;
}
/*****************************************************************
Functional Name: ReadSecurityRegister
return: AT45DBX_OK
*****************************************************************/
static uint8_t ReadSecurityRegister(At45dbxHandle_Type *hnd, uint8_t *reg, size_t size, uint32_t timeout)
{
    uint8_t status = AT45DBX_OK;
    hnd->ChipSelect(AT45DBX_PIN_LOW);
    uint8_t cmd[] = {AT45DBX_CMDA_RD_SECURITY_REG, 0xFF, 0xFF, 0xFF };
    if( At45dbxSpiWriteCallback(hnd, cmd,sizeof(cmd), timeout)!= AT45DBX_OK)
        status = AT45DBX_FAIL;
    else
        if(At45dbxSpiReadCallback(hnd, reg, size, timeout)!= AT45DBX_OK)
            status = AT45DBX_FAIL;
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    return status;
}
/*****************************************************************
Functional Name: PageSizeConfiguration
return: AT45DBX_OK
*****************************************************************/
static uint8_t PageSizeConfiguration(At45dbxHandle_Type *hnd, uint8_t setPage512bytes, uint32_t timeout)
{
    uint8_t status = AT45DBX_OK;
    hnd->ChipSelect(AT45DBX_PIN_LOW);
    uint8_t cmd[] = {0x3D, 0x2A, 0x80, 0x00 };
    if(setPage512bytes)
        cmd[3] = 0xA6;
    else
        cmd[3] = 0xA7;
    if(At45dbxSpiWriteCallback(hnd, cmd, sizeof(cmd), timeout)!= AT45DBX_OK)
        status = AT45DBX_FAIL;
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    return status;
}
/*****************************************************************
Functional Name: At45dbxPageErase
return: AT45DBX_OK
*****************************************************************/
uint8_t At45dbxPageErase(At45dbxHandle_Type *hnd, uint32_t page, uint32_t timeout)
{
    At45dbxDbgLog("Page Erase:");
    At45dbxDbgLog("Page Address:0x%08X.",page);
    uint8_t status = AT45DBX_OK;
    int32_t timestamp;
    //---------------------------------------------------------------
    TickSetTimestampMs(timestamp);
    do
    {
        if(TickTimeIsElapsedMs(timeout, timestamp))
            return AT45DBX_FAIL;
        if(ReadStatus(hnd, (uint8_t*)&status, AT45DBX_TIMEOUT_MS)!= AT45DBX_OK)
            return AT45DBX_FAIL;
    }while((status & AT45DBX_MSK_BUSY) == AT45DBX_BUSY);
    //---------------------------------------------------------------
    uint8_t cmd[] = {AT45DBX_CMDB_ER_PAGE, 0x00,(page >> 8) & 0x01, page };
    hnd->ChipSelect(AT45DBX_PIN_LOW);
    if(At45dbxSpiWriteCallback(hnd, cmd, sizeof(cmd),timeout)!= AT45DBX_OK)
    {
        hnd->ChipSelect(AT45DBX_PIN_HIGH);
        At45dbxDbgLog("Fail."); 
        return AT45DBX_FAIL;
    }
    hnd->ChipSelect(AT45DBX_PIN_HIGH);
    //---------------------------------------------------------------
    TickSetTimestampMs(timestamp);
    do
    {
        if(TickTimeIsElapsedMs(timeout, timestamp))
            return AT45DBX_FAIL;
        if(ReadStatus(hnd, (uint8_t*)&status, AT45DBX_TIMEOUT_MS)!= AT45DBX_OK)
            return AT45DBX_FAIL;
    }while((status & AT45DBX_MSK_BUSY) == AT45DBX_BUSY);
    //---------------------------------------------------------------
    At45dbxDbgLog("Erase Done.");
    
    return status;
}


/*****************************************************************
Functional Name: At45dbxSpiWriteCallback
return: AT45DBX_OK
*****************************************************************/
__weak uint8_t At45dbxSpiWriteCallback(At45dbxHandle_Type *hnd, const uint8_t *data, size_t size, uint32_t timeout)
{
    return AT45DBX_OK;
}
/*****************************************************************
Functional Name: At45dbxSpiReadCallback
return: AT45DBX_OK
*****************************************************************/
__weak uint8_t At45dbxSpiReadCallback (At45dbxHandle_Type *hnd, uint8_t *data, size_t size, uint32_t timeout)
{
    return AT45DBX_OK;
}




/******************* (C) COPYRIGHT 2016 marrob Design *****END OF FILE******/
