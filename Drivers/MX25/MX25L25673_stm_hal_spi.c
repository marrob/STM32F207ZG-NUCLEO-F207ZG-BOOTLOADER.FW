/*
 * GuiItf.c
 *
 *  Created on: 2022. máj. 14.
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/


#include "main.h"
#include "stdio.h"
#include <string.h>
#include "MX25L25673_stm_hal_spi.h"

static SPI_HandleTypeDef *_spi;
static inline void Mx25ChipEnable(uint8_t state);

#define MX25_CE_LOW   0
#define MX25_CE_HIGH  1

#define MX25_TIMEOUT  100

/* Private user code ---------------------------------------------------------*/
uint8_t Mx25Init(SPI_HandleTypeDef *spi)
{
  _spi = spi;

  Mx25Reset();

  HAL_Delay(10);

  /*** EN4B - Enter 4 Byte mode***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, (uint8_t[]){0xB7}, 1, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  Mx25ResetQuadSpi();

  return MX25_OK;
}

static inline void Mx25ChipEnable(uint8_t state)
{
  if(state)
    HAL_GPIO_WritePin(MX25_CS_GPIO_Port, MX25_CS_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(MX25_CS_GPIO_Port, MX25_CS_Pin, GPIO_PIN_RESET);
}

/*
 * A flash 256Mbit-es, ez 32MByte (32000000Byte)
 * Egy page mérete 256Byte, ez igy 32000000/256 = 125000 ez "125 000 Line"
 * 0x00 00 00 00 -> Page 0 (Line) Start
 * 0x00 00 00 FF -> Page 1 (Line) End
 *
 * 0x00 00 01 00 -> Page 2 Start
 * 0x00 00 01 FF -> Page 2 End
 * ha leveszem a page-en belül címeket, akkor:
 * 0x00 00 00 .. FF FF FF ez 0-tól 16777215-ig terjedo cimtartomány (ehhez kell a 4 bájtos címzés)
 */


/**
 * id:0x20C2
 */
uint8_t Mx25ReadId (uint16_t *id)
{
  /*** RDID - read identification***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, (uint8_t[]){0x9F}, 1, MX25_TIMEOUT);
  HAL_SPI_Receive(_spi, (uint8_t*)id, sizeof(uint16_t), MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);
  return MX25_OK;
}

/*
 *
 * addr: 0x0000 0000 - 0x0200 0000
 * address: aligned only page:
 * example:
 * page 1. address is: 0x0000 (0dec)
 * page 2. address is  0x0100 (256dec)
 * page 3. address is  0x0200
 *
 * size: 0..256
 * The Page Program instruction requires
 * that all the data bytes fall within the same 256-byte page. The low order address byte A[7:0] specifies the starting
 * address within the selected page. Bytes that will cross a page boundary will wrap to the beginning of the selected
 * page. The device can accept (256 minus A[7:0]) data bytes without wrapping. If 256 data bytes are going to be
 * programmed, A[7:0] should be set to 0.
 */
uint8_t Mx25PageProgram(uint32_t addr, uint8_t *data, uint32_t size)
{
  if(Mx25WritInProcess() != MX25_OK )
    return MX25_WRITE_IN_PROCESS;

  if((addr % 256) + size > 256)
    return MX25_NOT_ALIGNED;

  Mx25WriteEnable();

  /*** PP4B - Page Program***/
  uint8_t cmd[5];
  cmd[0] = 0x12;
  cmd[1] = addr >> 24;
  cmd[2] = addr >> 16;
  cmd[3] = addr >> 8;
  cmd[4] = addr;

  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, cmd, sizeof(cmd), MX25_TIMEOUT);
  HAL_SPI_Transmit(_spi, data, size, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  return MX25_OK;
}

uint8_t Mx25ResetQuadSpi(void)
{
  /*** RSTQIO - RSTQIO***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, (uint8_t[]){0xF5}, 1, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);
  return MX25_OK;
}

/*
 * WEL automatically clears to “0” when a program or erase operation completes.
 */
uint8_t Mx25WriteEnable(void)
{
  /*** WREN - Write Enable***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, (uint8_t[]){0x06}, 1, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);
  return MX25_OK;
}

uint8_t Mx25WriteDisable(void)
{
  /*** WRDI - Write Disable ***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, (uint8_t[]){0x04}, 1, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);
  return MX25_OK;
}

uint8_t Mx25Read(uint32_t addr, uint8_t *data, uint32_t size)
{
  /*** READ4B - Read Data Byte by 4 byte address ***/
  uint8_t cmd[5];
  cmd[0] = 0x13;
  cmd[1] = addr >> 24;
  cmd[2] = addr >> 16;
  cmd[3] = addr >> 8;
  cmd[4] = addr;

  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, cmd, sizeof(cmd), MX25_TIMEOUT);
  HAL_SPI_Receive(_spi, data, size, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);
  return MX25_OK;
}
/*
 * we have to wait for a long time ...
 * 110-150sec cycle time
 */
uint8_t Mx25ChipErase(void)
{
  if(Mx25WritInProcess() != MX25_OK )
    return MX25_WRITE_IN_PROCESS;

  Mx25WriteEnable();

  /*** CE - Chip Erase ***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, (uint8_t[]){0x60}, 1, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  return MX25_OK;
}

uint8_t Mx25Erase4kSector(uint32_t addr)
{
  if(Mx25WritInProcess() != MX25_OK )
    return MX25_WRITE_IN_PROCESS;

  Mx25WriteEnable();

  /*** SE Sector Erase ***/
  uint8_t cmd[4];
  cmd[0] = 0x20;
  cmd[1] = addr >> 16;
  cmd[2] = addr >> 8;
  cmd[3] = addr;

  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, cmd, sizeof(cmd), MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);
  return MX25_OK;
}

uint8_t Mx25Erase32kBlock(uint32_t addr)
{
  if(Mx25WritInProcess() != MX25_OK )
    return MX25_WRITE_IN_PROCESS;

  Mx25WriteEnable();

  /*** BE32K4B Block Erase ***/
  uint8_t cmd[5];
  cmd[0] = 0x5C;
  cmd[1] = addr >> 24;
  cmd[2] = addr >> 16;
  cmd[3] = addr >> 8;
  cmd[4] = addr;

  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, cmd, sizeof(cmd), MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  return MX25_OK;
}


uint8_t Mx25Erase64kBlock(uint32_t addr)
{
  if(Mx25WritInProcess() != MX25_OK )
    return MX25_WRITE_IN_PROCESS;

  Mx25WriteEnable();

  /*** BE4B Block Erase ***/
  uint8_t cmd[5];
  cmd[0] = 0xDC;
  cmd[1] = addr >> 24;
  cmd[2] = addr >> 16;
  cmd[3] = addr >> 8;
  cmd[4] = addr;

  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi, cmd, sizeof(cmd), MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  return MX25_OK;
}

uint8_t Mx25WritInProcess()
{
  uint8_t status = Mx25ReadStatusReg();
  /*** WIP bit ***/
  if(status & 0x01)
    return MX25_WRITE_IN_PROCESS;
  else
    return MX25_OK;
}

uint8_t Mx25ReadStatusReg(void)
{
  uint8_t status = 0;
  /*** RDSR - Read Status Register ***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi,(uint8_t[]){0x05}, 1, MX25_TIMEOUT);
  HAL_SPI_Receive(_spi, &status, sizeof(status), MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);
  return status;
}


uint8_t Mx25Reset(void)
{
  /*** RSTEN Reset Enable ***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi,(uint8_t[]){0x66}, 1, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  /*** RSTEN Reset Enable ***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi,(uint8_t[]){0x99}, 1, MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  return MX25_OK;
}

uint8_t Mx25ReadSecurityReg(void)
{
  uint8_t status = 0;
  /*** RDSR - Read Status Register ***/
  Mx25ChipEnable(MX25_CE_LOW);
  HAL_SPI_Transmit(_spi,(uint8_t[]){0x2B}, 1, MX25_TIMEOUT);
  HAL_SPI_Receive(_spi, &status, sizeof(status), MX25_TIMEOUT);
  Mx25ChipEnable(MX25_CE_HIGH);

  return status;
}
