/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/


#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <MX25L25673_stm_hal.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _AppTypeDef
{
  struct _Diag
  {
    uint32_t UsbUartErrorCnt;
    uint32_t UsbUartResponseCnt;
  }Diag;
/*
  char FW[DEVICE_FW_SIZE];
  char UID[DEVICE_UID_SIZE];
  char PCB[DEVICE_PCB_SIZE];
  */
  uint32_t BootUpCnt;
  uint64_t UpTimeSec;
  uint8_t IsFwUpdateMode;

  uint16_t FlashId;
}Device_t;

typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define INT_FLASH_BASE_ADDR       0x08000000 //thumb address...
#define APP_FLASH_SIZE            0x100000 - 0x40000 //-> 768KB
#define BTLDR_SIZE                0x40000
#define BTLDR_FLASH_LAST_SECTOR   5 // 5 is the booloader
#define EXT_FLASH_BASE_ADDR       0x10000000
#define EXT_FLASH_SIZE            0x02000000 //last address is 0x01FFFFFF

#define APP_FLASH_START_ADDR      INT_FLASH_BASE_ADDR + BTLDR_SIZE - 1


#define BTLDR_WAIT_FOR_FW_UPDATE  1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
LiveLED_HnadleTypeDef hLiveLed;
Device_t Device;

static char USB_UART_RxBuffer[USB_BUFFER_SIZE];
static char USB_UART_TxBuffer[USB_BUFFER_SIZE];
#define husb huart3

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/*** Usb ***/
void UsbParser(char *request);
void UsbRxTask(void);
void UsbTxTask(void);

/*** LiveLED ***/
void LiveLedOff(void);
void LiveLedOn(void);

/*** Tools ***/
void UpTimeTask(void);
uint8_t SpaceCount(char *str);
void StringArrayToBytes(char *str, uint8_t *data, uint16_t bsize);
void BytesToHexaString(uint8_t *data, char *dest, uint16_t bsize);
uint16_t CalcCrc16Ansi(uint16_t initValue, const void* address, size_t size);

void BootTask(void);


/*** Flash ***/
uint32_t FlashSectorErase(uint8_t start, uint8_t number);
uint32_t FlashProgram(uint32_t address, uint8_t *data, uint16_t size);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

  /*** Defaults ***/
  Device.IsFwUpdateMode = 0;

  Mx25Init(&hspi1);
  Mx25ReadId(&Device.FlashId);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LiveLedTask(&hLiveLed);
    UpTimeTask();

    UsbRxTask();
    UsbTxTask();

    //BootTask();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_UART_Receive_DMA (&husb, (uint8_t*)USB_UART_RxBuffer, sizeof(USB_UART_RxBuffer));
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MX25_CS_GPIO_Port, MX25_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MX25_CS_Pin */
  GPIO_InitStruct.Pin = MX25_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MX25_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIVE_LED_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIVE_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Usb------------------------------------------------------------------------*/
void UsbRxTask(void)
{
  static uint32_t timestamp;
  static uint8_t startFlag;

  if(strlen(USB_UART_RxBuffer)!=0)
  {
    if(!startFlag)
    {
      timestamp = HAL_GetTick();
      startFlag = 1;
    }
    for(uint16_t i=0; i < sizeof(USB_UART_RxBuffer); i++)
    {
      if(USB_UART_RxBuffer[i]=='\n')
      {
        USB_UART_RxBuffer[i] = 0;
        startFlag = 0;
        HAL_UART_DMAStop(&husb);
        UsbParser(USB_UART_RxBuffer);
        memset(USB_UART_RxBuffer, 0x00, sizeof(USB_UART_RxBuffer));
        HAL_UART_Receive_DMA(&husb, (uint8_t*) USB_UART_RxBuffer, sizeof(USB_UART_RxBuffer));
        break;
      }
    }
    if(startFlag)
    {
      if(HAL_GetTick() - timestamp > 500)
      {
        if(__HAL_UART_GET_FLAG(&husb, UART_FLAG_ORE))
          __HAL_UART_CLEAR_FLAG(&husb,UART_FLAG_ORE);
        if(__HAL_UART_GET_FLAG(&husb, USART_SR_NE))
          __HAL_UART_CLEAR_FLAG(&husb,USART_SR_NE);
        if(__HAL_UART_GET_FLAG(&husb, USART_SR_FE))
          __HAL_UART_CLEAR_FLAG(&husb,USART_SR_FE);
        startFlag = 0;
        HAL_UART_DMAStop(&husb);
        memset(USB_UART_RxBuffer, 0x00, sizeof(USB_UART_RxBuffer));
        HAL_UART_Receive_DMA(&husb, (uint8_t*) USB_UART_RxBuffer, sizeof(USB_UART_RxBuffer));
        Device.Diag.UsbUartErrorCnt ++;
      }
    }
  }
}
void UsbTxTask(void)
{
  uint16_t len = strlen(USB_UART_TxBuffer);
  if(len != 0)
  {
    Device.Diag.UsbUartResponseCnt++;
    HAL_UART_Transmit(&husb, (uint8_t*) USB_UART_TxBuffer, len, 100);
    USB_UART_TxBuffer[0] = 0;
  }
}

void UsbParser(char *request)
{
  uint8_t spaces = 0;
  static char cmd[USB_CMD_LENGTH];
  static char arg1[USB_ARG1_LENGTH];
  static char arg2[USB_ARG2_LENGTH];
  static char arg3[USB_ARG3_LENGTH];
  static char arg4[USB_ARG4_LENGTH];
  static uint8_t data[256];

  memset(cmd,0x00, sizeof(cmd));
  memset(arg1,0x00, sizeof(arg1));
  memset(arg2,0x00, sizeof(arg2));
  memset(arg3,0x00, sizeof(arg3));
  memset(arg4,0x00, sizeof(arg4));
  memset(data, 0, sizeof(data));

  if(strlen(USB_UART_RxBuffer) !=0)
  {
    memset(USB_UART_TxBuffer, 0x00, sizeof(USB_UART_TxBuffer));
    spaces = SpaceCount(USB_UART_RxBuffer);
    if(spaces == 0)
    {
      sscanf(request, "%s", cmd);
      /*** parameterless commands ***/
      if(!strcmp(cmd, "*OPC?"))
      {
        strcpy(USB_UART_TxBuffer, "*OPC");
      }
      else if(!strcmp(cmd, "*RDY?"))
      {
        strcpy(USB_UART_TxBuffer, "*RDY");
      }
      else if(!strcmp(cmd, "*WHOIS?"))
      {
        strcpy(USB_UART_TxBuffer, DEVICE_NAME); //TESTED
      }
      else if(!strcmp(cmd, "*VER?"))
      {
        strcpy(USB_UART_TxBuffer, DEVICE_FW); //TESTED
      }
      else if(!strcmp(cmd, "*UID?"))
      {
        sprintf(USB_UART_TxBuffer, "%4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
      }
      else if(!strcmp(cmd,"UPTIME?"))
      {
        sprintf(USB_UART_TxBuffer, "%lld", Device.UpTimeSec);
      }
      else if(!strcmp(cmd,"FW-UPDATE"))
      {
        Device.IsFwUpdateMode = 1;
        strcpy(USB_UART_TxBuffer, "OK");
      }

      else
      {
        strcpy(USB_UART_TxBuffer, "!UNKNOWN");
      }
    }
    else if(spaces == 1)
    {
      sscanf(request, "%s %s", cmd, arg1);

      if(!strcmp(cmd, "FL"))
      {/*** Flash Lock ***/
        if(arg1[0]=='I')
        {
           if(HAL_FLASH_Lock() != HAL_OK)
             strcpy(USB_UART_TxBuffer, "!INT LOCK ERROR");
           else
             strcpy(USB_UART_TxBuffer, "OK");
        }
        else if(arg1[0]=='E')
        {
          strcpy(USB_UART_TxBuffer, "NOT SUPPORTED");
        }
      }
      else if(!strcmp(cmd, "FU"))
      {/*** Flash Unlock ***/
        if(arg1[0]=='I')
        {
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
          __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
          if(HAL_FLASH_Unlock() != HAL_OK)
            strcpy(USB_UART_TxBuffer, "!INT FLASH UNLOCK ERROR");
          else
            strcpy(USB_UART_TxBuffer, "OK");
        }
        else if(arg1[0]=='E')
        {
          strcpy(USB_UART_TxBuffer, "NOT SUPPORTED");
        }
      }
      else if(!strcmp(cmd,"FB"))
      {/*** Flash Is Busy ***/
        if(arg1[0]=='I')
        {

        }
        else if(arg1[0]=='E')
        {
          if(Mx25WritInProcess()== MX25_OK)
            strcpy(USB_UART_TxBuffer, "FREE"); //TESTED
          else
            strcpy(USB_UART_TxBuffer, "BUSY");
        }
      }
      else
      {
        strcpy(USB_UART_TxBuffer, "!UNKNOWN");
      }
    }
    else if(spaces == 2)
    {
      sscanf(request, "%s %s %s", cmd, arg1, arg2);
      if(!strcmp(cmd, "FE")) //cmd where sector
      {/*** Flash Erase ***/
        if(arg1[0]=='I')
        {
          uint32_t sector = strtol(arg2, NULL, 16);
          if(sector <= BTLDR_FLASH_LAST_SECTOR)
          {
            strcpy(USB_UART_TxBuffer, "ERROR: YOU TRY TO ERASE A BOOTLOADER SECTOR!"); //TESTED
          }
          else
          {
            uint32_t erase_status = FlashSectorErase(sector,1);
            if(erase_status == 0xFFFFFFFF)
              strcpy(USB_UART_TxBuffer,"OK");
            else
              printf(USB_UART_TxBuffer,"!ERASE ERROR: %08lX", erase_status);
          }
        }
        else if(arg1[0]=='E')
        {
          uint32_t address = strtol(arg2, NULL, 16);
          if((address & EXT_FLASH_BASE_ADDR) == EXT_FLASH_BASE_ADDR)
          {
            address &= ~EXT_FLASH_BASE_ADDR;
            if(address < EXT_FLASH_SIZE - 1)
            {
              uint8_t chip_status = Mx25Erase64kBlock(address);
              if(chip_status == MX25_OK)
                strcpy(USB_UART_TxBuffer,"OK");
              else
                sprintf(USB_UART_TxBuffer,"!ERASE ERROR: %hhX", chip_status);
            }
            else
            {
              strcpy(USB_UART_TxBuffer,"ERROR: YOU TRY TO ERASE OUT OF EXT FLASH AREA!");
            }
          }
          else
          {
            strcpy(USB_UART_TxBuffer,"ERROR: TRY TO ERASE NOT EXT FLASH AREA!");
          }
        }
      }
      else if(!strcmp(cmd, "FR")) //cmd addr size
      {/*** Flash Read ***/
        uint32_t address = strtol(arg1, NULL, 16);
        uint16_t bsize = strtol(arg2, NULL, 16);
        if(bsize > 256)
          strcpy(USB_UART_TxBuffer, "!SIZE ERROR");
        if((address & EXT_FLASH_BASE_ADDR) ==  EXT_FLASH_BASE_ADDR)
        {
          address &= ~EXT_FLASH_BASE_ADDR;
          Mx25Read(address, data, bsize);
        }
        else if ((address & INT_FLASH_BASE_ADDR) == INT_FLASH_BASE_ADDR)
        {
          for(uint16_t i = 0; i < bsize; i++)
            data[i] = *(__IO uint8_t*)(address + i);
        }
        else
        {
          strcpy(USB_UART_TxBuffer, "!ADDRESS ERROR");
        }
        uint16_t crc = CalcCrc16Ansi(0, data, bsize);
        memset(arg3, 0, sizeof(arg3));
        BytesToHexaString(data, arg3, bsize);
        sprintf(USB_UART_TxBuffer,"%08lX %02X %s %04X", address, bsize, arg3, crc ); //addr size data crc
      }
      else
      {
        strcpy(USB_UART_TxBuffer, "!UNKNOWN");
      }
    }
    else if(spaces == 4)
    {
      sscanf(request, "%s %s %s %s %s", cmd, arg1, arg2, arg3, arg4); //cmd addr bsize data crc
      if(!strcmp(cmd, "FP"))
      {/*** Flash Program ***/
        uint32_t status = 0;
        uint32_t address = strtol(arg1, NULL, 16);
        uint16_t bsize = strtol(arg2, NULL, 16);
        uint16_t crc = strtol(arg4, NULL, 16);
        strcpy(USB_UART_TxBuffer, "OK");
        if(strlen(arg3) != bsize * 2) //e.g: 010203 -> bsize = 3
          strcpy(USB_UART_TxBuffer, "ERROR: RECEIVED DATA SIZE IS INVALID!"); //TESTED
        else
        {
          strcpy(USB_UART_TxBuffer, "OK");
          StringArrayToBytes(arg3, data, bsize);
          uint16_t clacCrc = CalcCrc16Ansi(0, data, bsize);
          if(crc != clacCrc)
          {
            strcpy(USB_UART_TxBuffer, "ERROR: RECEIVED DATA HAS INVALID CRC!"); //TESTED
          }
          else
          {
            if((address & EXT_FLASH_BASE_ADDR) ==  EXT_FLASH_BASE_ADDR)
            {
              address &= ~EXT_FLASH_BASE_ADDR;
              if(address + bsize > EXT_FLASH_SIZE - 1)
              {
                strcpy(USB_UART_TxBuffer, "ERROR: YOU TRY TO WRITE OUT OF EXT FLASH AREA!"); //TESTED
              }
              else
              {
                status = Mx25PageProgram(address, data, bsize);
                if(status == MX25_WRITE_IN_PROCESS)
                  strcpy(USB_UART_TxBuffer, "ERROR: WRITE IN PROCESS, PLEASE WAIT!");
                else if (status == MX25_NOT_ALIGNED)
                  strcpy(USB_UART_TxBuffer, "ERROR: NOT ALIGNED!"); //TESTED
                else if(status == MX25_NOT_WRITE_ENALBE)
                  strcpy(USB_UART_TxBuffer, "ERROR: NOT WRITE ENABLE!");
                else
                  strcpy(USB_UART_TxBuffer, "OK");
              }
            }
            else if ((address & INT_FLASH_BASE_ADDR) == INT_FLASH_BASE_ADDR)
            {
              if(address < INT_FLASH_BASE_ADDR + BTLDR_SIZE)
              {
                strcpy(USB_UART_TxBuffer, "ERROR: YOU TRY TO WRITE BOOTLOADER AREA!"); //TESTED
              }
              else
              {
                if(address + bsize > INT_FLASH_BASE_ADDR + APP_FLASH_SIZE)
                {
                  strcpy(USB_UART_TxBuffer, "ERROR: YOU TRY TO WRITE OUT OF APP FLASH AREA!"); //TESTED
                }
                else
                {
                  status = FlashProgram(address, data, bsize);
                  if(status != HAL_FLASH_ERROR_NONE)
                    sprintf(USB_UART_TxBuffer, "%s %08lX", "!INT PROG ERROR", status);
                  else
                    strcpy(USB_UART_TxBuffer, "OK");
                }
              }
            }
            else
            {
              strcpy(USB_UART_TxBuffer, "ERROR: YOU TRY TO WRITE INVALID ADDRESS!"); //TESTED
            }
          }
        }
      }
      else
      {
        strcpy(USB_UART_TxBuffer, "!UNKNOWN");
      }
    }
    strcat(USB_UART_TxBuffer,"\n");
  }
}


void BootTask(void)
{
  static uint32_t timestamp = 0;
  static uint16_t counter = BTLDR_WAIT_FOR_FW_UPDATE;
  if( HAL_GetTick() - timestamp > 1000 && !Device.IsFwUpdateMode)
  {
    timestamp = HAL_GetTick();
    counter--;

    //DeviceUsrLog("Wait for a client... %d",  counter);
    if(!counter)
    {
      if (((*(__IO uint32_t*)APP_FLASH_START_ADDR) & 0x2FFE0000 ) == 0x20000000)
      {
        //DeviceDbgLog("I found a valid firmware!");
        //IWatchdogDeInit();
        HAL_DeInit();
        HAL_UART_MspDeInit(&husb);
        HAL_SPI_MspDeInit(&hspi1);

        uint32_t appAddress = *(__IO uint32_t*) (APP_FLASH_START_ADDR + 4);
        /* Jump to user application */
        pFunction pApp = (pFunction) appAddress;
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) APP_FLASH_START_ADDR);

        pApp();
      }
      else
      {
        //DeviceUsrLog("There is nothing");
        counter = BTLDR_WAIT_FOR_FW_UPDATE;
      }
    }
  }
}

/* Tools----------------------------------------------------------------------*/
uint8_t SpaceCount(char *str)
{
  uint8_t space = 0;
  for(uint32_t i = 0; i < strlen(str); i++)
  {
    if(str[i]==' ')
      space++;
  }
  return space;
}

void StringArrayToBytes(char *str, uint8_t *data, uint16_t bsize)
{
  memset(data, 0, bsize);
  uint8_t byteIndex = 0;
  for(uint16_t i = 0; i < bsize*2; i+=2)
  {
    char sb[2] = {0,0};
    sb[0] = str[i];
    sb[1] = str[i+1];
    data[byteIndex++] = strtol(sb, NULL, 16);
  }
}

void BytesToHexaString(uint8_t *data, char *dest, uint16_t bsize)
{
  if(bsize)
    for(uint16_t i = 0; i < bsize; i++)
        sprintf((dest + (i * 2)),"%02X", *(data + i));
  else
    dest[0] = 0;
}

void UpTimeTask(void)
{
  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.UpTimeSec++;
  }
}

uint16_t CalcCrc16Ansi(uint16_t initValue, const void* address, size_t size)
{
  uint16_t remainder = initValue;
  uint16_t polynomial = 0x8005;
  uint8_t *_address = (uint8_t*)address;

  for (size_t i = 0; i < size; ++i)
  {
    remainder ^= (_address[i] << 8);
    for (uint8_t bit = 8; bit > 0; --bit)
    {
      if (remainder & 0x8000)
        remainder = (remainder << 1) ^ polynomial;
      else
        remainder = (remainder << 1);
    }
  }
  return (remainder);
}

/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

/* printf --------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
 // HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, 100);
  return len;
}


/* Flash ---------------------------------------------------------------------*/
uint32_t FlashSectorErase(uint8_t start, uint8_t number)
{
  uint32_t sectorError = 0x0;
  FLASH_EraseInitTypeDef hEraseInit;
  hEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  hEraseInit.Sector = start;
  hEraseInit.NbSectors = number;
  hEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  HAL_FLASHEx_Erase(&hEraseInit, &sectorError);
  return sectorError;
}

uint32_t FlashProgram(uint32_t address, uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status = HAL_OK;
  for(uint16_t i = 0; i< size; i++ )
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data[i]);
    if(status != HAL_OK)
      return HAL_FLASH_GetError();
  }
  return HAL_FLASH_ERROR_NONE;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
