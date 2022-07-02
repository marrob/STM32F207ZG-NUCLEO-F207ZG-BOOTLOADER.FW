/*
 * GuiItf.h
 *
 *  Created on: 2022. máj. 14.
 *      Author: Margit Robert
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MX25_H_
#define _MX25_H_

#define MX25_OK              0
#define MX25_FAIL            1
#define MX25_ARG_ERR         2
#define MX25_TIMEOUT_ERR     3

#define MX25_PAGE_SIZE 256
#define MX25_SIZE             32000000/MX25_PAGE_SIZE


/* Exported functions prototypes ---------------------------------------------*/


/* Mx25 ---------------------------------------------------------------------*/

uint8_t Mx25Init(SPI_HandleTypeDef *spi);
uint8_t Mx25ReadStatusReg(void);
uint8_t Mx25PageProgram(uint32_t addr, uint8_t *data, uint32_t size);
uint8_t Mx25WriteEnable(void);
uint8_t Mx25WriteDisable(void);
uint8_t Mx25Read(uint32_t addr, uint8_t *data, uint32_t size);
uint8_t Mx25ChipErase(void);

#endif /* _MX25_H_ */
