/******************** (C) COPYRIGHT 2016 marrob Design *************************
* File Name          : LiveLED.c
* Author             : Margit Róbert
* Date First Issued  : 2016-02-14
* Description        : AT45DB161E
********************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _AT45DB161__H_
#define _AT45DB161__H_ 1

#define AT45DBX_DEBUG_LEVEL 0
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

/* Exported macro ------------------------------------------------------------*/
#if (AT45DBX_DEBUG_LEVEL > 0)
#define  At45dbxUsrLog(...)     {printf("AT45DBX:");\
                                 printf(__VA_ARGS__);\
                                 printf("\n");}
#else
#define At45dbxUsrLog(...)
#endif

#if (AT45DBX_DEBUG_LEVEL > 1)

#define  At45dbxErrLog(...)     {printf("ERROR.AT45DBX:") ;\
                                 printf(__VA_ARGS__);\
                                 printf("\n");}
#else
#define At45dbxErrLog(...)
#endif

#if (AT45DBX_DEBUG_LEVEL > 2)
#define  At45dbxDbgLog(...)     {printf("DEBUG.AT45DBX:") ;\
                                 printf(__VA_ARGS__);\
                                 printf("\n");}
#else
#define At45dbxDbgLog(...)
#endif

/* Includes ------------------------------------------------------------------*/
#define AT45DBX_OK                  0x00
#define AT45DBX_FAIL                0x01
#define AT45DBX_TIMEOUT             0x02

#define AT45DBX_PIN_HIGH            0x01
#define AT45DBX_PIN_LOW             0x00

#define AT45DBX_MEM_SIZE            AT45DBX_2MB //MegaByte
#define AT45DBX_MNF_ID_SIZE         5

#define AT45DBX_PAGE_BITS           9
#define AT45DBX_PAGE_SIZE          (1 << AT45DBX_PAGE_BITS)
#define AT45DBX_TIMEOUT_MS          800


/* Public typedef ------------------------------------------------------------*/
typedef struct _At45dbxHandle_Type
{
    struct SecurtyRegister
    {
        uint8_t Otp[64];
        uint8_t AdestoId[64];
    }SecurtyRegister;
    uint8_t MnfId[AT45DBX_MNF_ID_SIZE];
    
    uint32_t MemoryAddress;
    
    uint8_t ReadingInProcess;
    uint8_t WriteInProcess;
    uint8_t DensityCode; // -> AT45DB161D = 0x21
    void (*ChipSelect)(uint8_t state);
    void (*Reset)(uint8_t);
    uint8_t IsConfigured;
}At45dbxHandle_Type;

typedef struct _At45dbxInterface_Type
{
    void (*ChipSelect)(uint8_t state);
    void (*Reset)(uint8_t state);

}At45dbxInterface_Type;

/* Exported functions --------------------------------------------------------*/
uint8_t At45dbxInit(At45dbxHandle_Type *hnd, At45dbxInterface_Type *itf);
uint8_t At45dbxPageRead(At45dbxHandle_Type *hnd, uint32_t address, void *data, size_t size, uint32_t timeout);
uint8_t At45dbxPageTransferModifyWrite(At45dbxHandle_Type *hnd, uint32_t address, const void *data, size_t size, uint32_t timeout);

uint8_t ReadStatus(At45dbxHandle_Type *hnd, uint8_t *dfstatus, uint32_t timeout);
uint8_t At45dbxPageErase(At45dbxHandle_Type *hnd, uint32_t address, uint32_t timeout);

uint8_t At45dbxWrite(At45dbxHandle_Type *hnd, uint32_t writeAddr, const void *pData, size_t size, uint32_t timeout);
uint8_t At45dbxRead(At45dbxHandle_Type *hnd, uint32_t readAddress, void *pData, size_t size, uint32_t timeout);

__weak uint8_t At45dbxSpiWriteCallback(At45dbxHandle_Type *hnd, const uint8_t *data, size_t size, uint32_t timeout);
__weak uint8_t At45dbxSpiReadCallback (At45dbxHandle_Type *hnd, uint8_t *data, size_t size, uint32_t timeout);
#endif //_AT45DB161__H_
/******************* (C) COPYRIGHT 2016 marrob Design *****END OF FILE******/
