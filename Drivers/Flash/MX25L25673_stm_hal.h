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
#define MX25_WRITE_IN_PROCESS   2
#define MX25_NOT_WRITE_ENALBE   3
#define MX25_NOT_ALIGNED        4
#define MX25_ARG_ERR            5
#define MX25_TIMEOUT_ERR        6

#define MX25_PAGE_SIZE 256
#define MX25_SIZE             32000000/MX25_PAGE_SIZE


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
uint8_t Mx25WritInProcess();
uint8_t Mx25Erase4kSector(uint32_t addr);
uint8_t Mx25Erase64kBlock(uint32_t addr);
uint8_t Mx25Reset(void);
uint8_t Mx25ReadSecurityReg(void);
uint8_t Mx25Erase32kBlock(uint32_t addr);

#endif /* _MX25_H_ */
