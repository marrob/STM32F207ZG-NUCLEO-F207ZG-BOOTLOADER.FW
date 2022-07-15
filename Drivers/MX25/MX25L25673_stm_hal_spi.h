/*
 * GuiItf.h
 *
 *  Created on: 2022. máj. 14.
 *      Author: Margit Robert
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MX25_H_
#define _MX25_H_



#define MX25_OK                 0
#define MX25_FAIL               1
#define MX25_NOT_WRITE_ENALBE   3
#define MX25_NOT_ALIGNED        4
#define MX25_ARG_ERR            5
#define MX25_TIMEOUT_ERROR      6



#define MX25_FLASH_SIZE                      0x2000000 /* 256 MBits => 32MBytes */
#define MX25_SUBSECTOR_SIZE                  0x1000    /* 4096 subsectors of 4kBytes */
#define MX25_PAGE_SIZE                       0x100     /* 65536 pages of 256 bytes */

#define MX25_4L_DUMMY_CYCLES_READ            6
#define MX25_BULK_ERASE_MAX_TIME             250000
#define MX25_SUBSECTOR_ERASE_MAX_TIME        800


#define MX25_4L_DUMMY_CYCLES_READ            6
#define MX25_BULK_ERASE_MAX_TIME             250000
#define MX25_ERASE_64K_MAX_TIME              800

/*** Identification Operations ***/
#define MX25_1L_READ_ID_CMD                  0x9F

/*** Reset Operations ***/
#define MX25_1L_RESET_ENABLE_CMD             0x66
#define MX25_1L_RESET_CMD                    0x99

/*** Read Operations ***/
#define MX25_3B_1L_READ                      0x03
#define MX25_4B_1L_READ                      0x13
#define MX25_4B_4L_READ                      0xEC

/*** Write Operations ***/
#define MX25_1L_WRITE_EN_CMD                 0x06

/*** Config ***/
#define MX25_1L_READ_CFG_CMD                 0x15
#define MX25_1L_WRITE_CFG_SR_CMD             0x01
#define MX25_1L_READ_STATUS_CMD              0x05

/*** Program Operations ***/
#define MX25_3B_1L_PP_CMD                    0x02
#define MX25_4B_1L_PP_CMD                    0x12
#define MX25_4B_4L_PP_CMD                    0x3E

/*** Erase Operations ***/
#define MX25_3B_1L_SECTOR_ERASE_CMD          0x20
#define MX25_4B_1L_SECTOR_ERASE_CMD          0xDC
#define MX25_1L_BULK_ERASE_CMD               0xC7

/*** Status Register ***/
#define MX25_SR_WIP                         ((uint8_t)0x01)
#define MX25_SR_WEL                         ((uint8_t)0x02)


/* Exported functions prototypes ---------------------------------------------*/


/* Mx25 ---------------------------------------------------------------------*/

uint8_t Mx25Init(SPI_HandleTypeDef *spi);
uint8_t Mx25ReadId (uint16_t *id);
uint8_t Mx25ReadStatusReg(void);
uint8_t Mx25PageProgram(uint32_t addr, uint8_t *data, uint32_t size);
uint8_t Mx25WriteEnable(void);
uint8_t Mx25WriteDisable(void);
uint8_t Mx25Read(uint32_t addr, uint8_t *data, uint32_t size);
uint8_t Mx25ChipErase(void);
uint8_t Mx25ResetQuadSpi(void);
uint8_t Mx25Erase64kBlock(uint32_t addr);
uint8_t Mx25Reset(void);
uint8_t Mx25ReadSecurityReg(void);

#endif /* _MX25_H_ */
