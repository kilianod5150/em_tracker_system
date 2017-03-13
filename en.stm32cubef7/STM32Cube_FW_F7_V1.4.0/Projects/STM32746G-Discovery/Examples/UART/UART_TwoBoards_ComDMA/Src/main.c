
#include "main.h"



#include <stdio.h>
#include <stdlib.h>
#include "stm32f7xx_hal_tim.h"
//
//#include <stm32f7xx_hal_tim.c>
#include "stm32f7xx_hal_tim_ex.h"

//#define TIMx                           TIM3
//#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()


/* Definition for TIMx's NVIC */
//#define TIMx_IRQn                      TIM3_IRQn
//#define TIMx_IRQHandler                TIM3_IRQHandler



/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComDMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TRANSMITTER_BOARD

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */


TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef    ADA_TimHandle; // TIM handle for ADA SPI acquisition
TIM_HandleTypeDef    UART_TimHandle; // TIM handle for UART acquisition

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz\n";
uint8_t SPI_TxBuffer[] = "hello";
uint8_t SPI_RxBuffer[SPI_BUFFERSIZE];
uint8_t SPI_STMF0_FLAG=0; // flag for handling SPI comms to the STM32F0, flag is 0 when free, and 1 when it is active
//uint8_t SPI_RxBuffer[] = "hello";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);

/* Private define ------------------------------------------------------------*/
enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

uint32_t system_time =0;

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

/* Private functions ---------------------------------------------------------*/
//HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the MPU attributes as Write Through */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 216 MHz */
  SystemClock_Config();
  
  /* Configure LED1 */
  BSP_LED_Init(LED1);
	
	  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 230400;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  

	
	  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.Mode = SPI_MODE_MASTER;

  HAL_Delay(5);  
	
	
	    GPIO_InitTypeDef  gpio_init_structure_new; // define an object for the GPIO
   	__HAL_RCC_GPIOI_CLK_ENABLE();  // D7 or PI3 is on port I so enable the GPIO update clock (although it isactually enabled already by the LEDs)

	  gpio_init_structure_new.Pin = GPIO_PIN_0 | GPIO_PIN_3 ;
		//gpio_init_structure.Pin = GPIO_PIN_0 ;
    gpio_init_structure_new.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure_new.Pull = GPIO_NOPULL;
    gpio_init_structure_new.Speed = GPIO_SPEED_HIGH;
  
    HAL_GPIO_Init(GPIOI, &gpio_init_structure_new); // initialise this GPIO
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);  // set low
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);  // set CS pin high
    HAL_Delay(1);  
		
		GPIO_InitTypeDef  gpio_init_structure_b; // define an object for the GPIO
		__HAL_RCC_GPIOB_CLK_ENABLE();  // clock for port B gpios
		gpio_init_structure_b.Pin = GPIO_PIN_8; 
		gpio_init_structure_b.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure_b.Pull = GPIO_NOPULL;
    gpio_init_structure_b.Speed = GPIO_SPEED_HIGH;
		HAL_GPIO_Init(GPIOB, &gpio_init_structure_b); // initialise this GPIO
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // set low

    HAL_Delay(100);


	  /* Compute the prescaler value to have TIMx counter clock equal to 100000 Hz */
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 100000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;
  ADA_TimHandle.Instance = ADA_TIMx; 
	UART_TimHandle.Instance = UART_TIMx; 
  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock / 2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 100000 - 1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
	
	ADA_TimHandle.Init.Period            = 1000000 - 1;
  ADA_TimHandle.Init.Prescaler         = uwPrescalerValue;
  ADA_TimHandle.Init.ClockDivision     = 0;
  ADA_TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  ADA_TimHandle.Init.RepetitionCounter = 0;
	
	UART_TimHandle.Init.Period            = 1000000 - 1;
  UART_TimHandle.Init.Prescaler         = uwPrescalerValue;
  UART_TimHandle.Init.ClockDivision     = 0;
  UART_TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  UART_TimHandle.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	  if (HAL_TIM_Base_Init(&ADA_TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
		if (HAL_TIM_Base_Init(&UART_TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&ADA_TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
	
		  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&UART_TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	


//  /* Infinite loop */
  while (1)
  {

		
		// if the HAL SPI is finished, pull the spi enable pin low, could change this to simply toggle
		if( (HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_READY) && (SPI_STMF0_FLAG==1) )
		{
		//	 HAL_Delay(100);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // set low
       SPI_STMF0_FLAG=0;
		}
		
		
		
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
	
	

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	
	    if (htim->Instance==TIM3) //check if this assosiated with TIM3
      {
			//	BSP_LED_Toggle(LED1); 
			//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer, strlen(aTxBuffer));
				  HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)SPI_RxBuffer, strlen(SPI_RxBuffer)); //send what is recieved from the SPI
			}
			
			if (htim->Instance==TIM4) //check if this assosiated with TIM3
      {
				
				//need to check which timer to use
			//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer, strlen(aTxBuffer));
						if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_READY)
		{
			BSP_LED_Toggle(LED1); 
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);  // set high
		
	  if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)SPI_TxBuffer, (uint8_t *)SPI_RxBuffer, SPI_BUFFERSIZE) != HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }	
	
	
	SPI_STMF0_FLAG=1;
	
//		while (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_READY)
// {
// } 
		
   
//	HAL_Delay(100);
// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // set low
	}
			//	BSP_LED_Toggle(LED1); 
			//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer, strlen(aTxBuffer));
			}
			
			if (htim->Instance==TIM2) //check if this assosiated with TIM3
      {
			//	BSP_LED_Toggle(LED1); 
			//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer, strlen(aTxBuffer));
			}
			
}



void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }  
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

  
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}





/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Configure LED1 which is shared with SPI2_SCK signal */
  //BSP_LED_Init(LED1);
  /* Turn LED1 on: Transfer in transmission/reception process is complete */
  //BSP_LED_On(LED1);
  wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}

static void Error_Handler(void)
{
  /* Turn LED1 on */
  //BSP_LED_On(LED1);
  while(1)
  {
    /* Error if LED1 is slowly blinking (1 sec. period) */
 //   BSP_LED_Toggle(LED1); 
  //  HAL_Delay(1000); 
  }  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


