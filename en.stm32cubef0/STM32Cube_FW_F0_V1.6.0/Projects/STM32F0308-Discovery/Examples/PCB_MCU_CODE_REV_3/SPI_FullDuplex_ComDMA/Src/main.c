/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComDMA/Src/main.c
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    27-May-2016
  * @brief   This sample code shows how to use STM32F0xx SPI HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          DMA transfer.
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define ARM_MATH_CM0
#include "math.h"
#include "core_cm0.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup SPI_FullDuplex_ComDMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define  PERIOD_VALUE       (uint32_t)(666 - 1)  /* Period Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*50/100) /* Capture Compare 4 Value  */

/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle_TS;
TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

/* ADC handle declaration */
ADC_HandleTypeDef             AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef        ADCConfig;

/* Converted value declaration */
uint32_t                      aResultDMA;


/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

 float Freq_Store[8]={17000,18000,19000,20000,21000,22000,23000,24000};
 float F_Sample=50000;
 int32_t trig_scale_factor=4096; // Scale factor to attenuate Sine signal by,
 #define TRIG_SAMPLES 50
 
 q31_t Sine_F1[TRIG_SAMPLES];
 q31_t Sine_F2[TRIG_SAMPLES];
 q31_t Sine_F3[TRIG_SAMPLES];
 q31_t Sine_F4[TRIG_SAMPLES];
 q31_t Sine_F5[TRIG_SAMPLES];
 q31_t Sine_F6[TRIG_SAMPLES];
 q31_t Sine_F7[TRIG_SAMPLES];
 q31_t Sine_F8[TRIG_SAMPLES];
 
 q31_t Cosine_F1[TRIG_SAMPLES];
 q31_t Cosine_F2[TRIG_SAMPLES];
 q31_t Cosine_F3[TRIG_SAMPLES];
 q31_t Cosine_F4[TRIG_SAMPLES];
 q31_t Cosine_F5[TRIG_SAMPLES];
 q31_t Cosine_F6[TRIG_SAMPLES];
 q31_t Cosine_F7[TRIG_SAMPLES];
 q31_t Cosine_F8[TRIG_SAMPLES];

 int32_t sample_counter=0; // variable to count the number of cycles of the ADC sampler circuit
 int32_t sample_counter_period=1000; // sets the number of cycles to run through for each channel of the circuit
 int32_t current_channel=1; // flag to select what channel is currently being demodulated, this can be either channel 1-8, but just 1 for the moment
 q31_t ADC_Value;

/* Buffer used for transmission */
//uint8_t aTxBuffer[] = "hello\n";
uint8_t aTxBuffer[] = {'x','y','z',1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,'\n'}; // define array for transmitting the recorded currents
uint8_t otherbuffer[] = "HELLO \n\n";
uint8_t otherbuffer2[] = "\n";
/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];
	

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void UART_Init(void);
static void SPI_Init(void);
static void ADC_Init(void);
static void TIM_Init(void);
static void TIM_PWM_Init(void);
static void Demodulator_Init(void);
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
static void Timeout_Error_Handler(void);
static void EXTI0_1_IRQHandler_Config(void);
static void EXTI4_15_IRQHandler_Config(void);
static void SPI_Handler_Function(void);
/* Private functions ---------------------------------------------------------*/
uint8_t SPI_Flag=0;
uint8_t SPI_Init_Flag=0; // flag that is set the first time the SPI is activated, the spi handler wont start until this goes high
uint8_t LED_Test_Flag=0; 
uint16_t Demodulated_Current[8]; //define a 16 bit array of all the measured coil currents
uint16_t Test_Current=0; // a variable to incremented to simulated a varying current signal
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  // Configure everything first 
	
	
	
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
	
	  __HAL_RCC_GPIOA_CLK_ENABLE(); 
		GPIO_InitTypeDef  gpio_init_structure_new; // define an object for the GPIO

	  gpio_init_structure_new.Pin = GPIO_PIN_3 ;
		gpio_init_structure_new.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure_new.Pull = GPIO_NOPULL;
    gpio_init_structure_new.Speed = GPIO_SPEED_HIGH;
  
    HAL_GPIO_Init(GPIOA, &gpio_init_structure_new); // initialise this GPIO
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // set high
	  HAL_Delay(1000); // seemingly we need this...?
  //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);  // set low
		HAL_Delay(100);
	
	
	
	
	  /* -2- Configure EXTI_Line0 (connected to PA.00 pin) in interrupt mode */
  //EXTI0_1_IRQHandler_Config();
	  // Start up SPI 
  SPI_Init();
		  /* -2- Configure EXTI_Line0 (connected to PA.04 pin) in interrupt mode */
  EXTI4_15_IRQHandler_Config();

		// Start up UART if required for Debug
 //UART_Init();


	
	// Start up ADC
  //ADC_Init();
	
	//Start Up main sample rate clock
	//TIM_Init();
	
	//Start up PWM output for coil driver
	//TIM_PWM_Init();
	
	//Run any calculations required for the demodulator, generating sine tables for example
	//Demodulator_Init(); //something in here causing issues
	
	
	
	



		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // set high
		HAL_Delay(1000);


	
  /* Infinite loop */  
	// put low priority timing stuff in here
  while (1)
  {
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // set high
	//	HAL_Delay(1000);
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // set high
	//	HAL_Delay(1000);
		SPI_Handler_Function(); // function that handles slave SPI stuff
	}
	
	
	
}




static void SPI_Init(void)
{
	  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	//SpiHandle.Init.NSS               = SPI_NSS_HARD_OUTPUT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  //SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  //SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;


  SpiHandle.Init.Mode = SPI_MODE_SLAVE;


  /* Slave board must wait until Master Board is ready. This to guarantee the 
     correctness of transmitted/received data */
 // HAL_Delay(5);  

  if (HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

	HAL_Delay(5);
	
	  /*##-2- Start the Full Duplex Communication process ########################*/  
  /* While the SPI in TransmitReceive process, user can transmit data through 
     "aTxBuffer" buffer & receive data through "aRxBuffer" */
  //if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
  //{
    /* Transfer error in transmission process */
  //  Error_Handler();
  //}

	
	
}

static void SPI_Handler_Function(void)
{
	
			
		
		//if( (SPI_Flag==1) && (HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_READY) )
	  if( (SPI_Flag==1) && (SPI_Init_Flag==1))
		{
			
			//Test_Current++; // for debug only, increments everytime the spi comms are activated
			// Now add these measurements to the aTxBuffer
			
			
			if( (aRxBuffer[0]=='a') && (aRxBuffer[1]=='b') && (aRxBuffer[2]=='c') )
					{
				for(int i=0; i<8; i++)
					{
					//aTxBuffer[2*i+3]=(uint8_t)((Test_Current&0xFF00)>>8); // shift the high bit down
          //aTxBuffer[2*i+4]=(uint8_t)(Test_Current&0x00FF);
						aTxBuffer[2*i+3]=aRxBuffer[2*i+3]; 
						aTxBuffer[2*i+4]=aRxBuffer[2*i+4]; 
					}	
			
					}
			
			
			  if (HAL_SPI_DeInit(&SpiHandle) != HAL_OK) //disable spi
  {
    /* Initialization Error */
    Error_Handler();
  }
	
			if (HAL_SPI_Init(&SpiHandle) != HAL_OK) //enable spi
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	//transmit and recieve
		  if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }
		
	while (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_READY) // wait until the transfer is finished
  {
		
  } 
  SPI_Flag=0;
	
	//HAL_Delay(1);
  }
		

		
	
}

static void TIM_Init(void)
{
	    /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

  /* Set TIMx instance */
  TimHandle_TS.Instance = TIM_TS;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle_TS.Init.Period            = 10000 - 1;
  TimHandle_TS.Init.Prescaler         = uwPrescalerValue;
  TimHandle_TS.Init.ClockDivision     = 0;
  TimHandle_TS.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_TS.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&TimHandle_TS) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle_TS) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


			
			// ADC grab, must happen at start of loop
				HAL_ADC_Start(&AdcHandle);

	
	   // need to improve this, very dumb
        if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
        {
            ADC_Value = HAL_ADC_GetValue(&AdcHandle);
        }
			
			
    if(sample_counter<sample_counter_period)
		{
			// Sine/Cosine multiplication
			
			
			
			// Push to IIR Filter
			
			
		sample_counter++;
		}
		else
		{
			sample_counter=0;
		}
		
		
	
}


static void TIM_PWM_Init(void)
{
  /* Compute the prescaler value to have TIM3 counter clock equal to 16000000 Hz */
  uhPrescalerValue = (uint32_t)(SystemCoreClock / 16000000) - 1;
	  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
  TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.

    In this example TIM3 input clock (TIM3CLK) is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM3CLK = PCLK1
      PCLK1 = HCLK
      => TIM3CLK = HCLK = SystemCoreClock

    To get TIM3 counter clock at 16 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock) /16 MHz) - 1

    To get TIM3 output clock at 24 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM3 counter clock / TIM3 output clock) - 1
           = 665

    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR + 1)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR + 1)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR + 1)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR + 1)* 100 = 12.5%

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Initialize TIMx peripheral as follows:
       + Prescaler = (SystemCoreClock / 16000000) - 1
       + Period = (666 - 1)
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Instance = TIMx;


  TimHandle.Init.Prescaler         = uhPrescalerValue;
  TimHandle.Init.Period            = PERIOD_VALUE;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;



  /* Set the pulse value for channel 4 */
  sConfig.Pulse = PULSE4_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }


  /* Start channel 4 */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
}

static void ADC_Init(void)
{
	    /* ### - 1 - Initialize ADC peripheral #################################### */
  /*
   *  Instance                  = ADC1.
   *  ClockPrescaler            = PCLK divided by 4.
   *  LowPowerAutoWait          = Disabled
   *  LowPowerAutoPowerOff      = Disabled
   *  Resolution                = 12 bit (increased to 16 bit with oversampler)
   *  ScanConvMode              = ADC_SCAN_ENABLE 
   *  DataAlign                 = Right
   *  ContinuousConvMode        = Enabled
   *  DiscontinuousConvMode     = Enabled
   *  ExternalTrigConv          = ADC_SOFTWARE_START
   *  ExternalTrigConvEdge      = None (Software start)
   *  EOCSelection              = End Of Conversion event
   *  DMAContinuousRequests     = ENABLE
   */
  
  AdcHandle.Instance = ADC1;
  
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ContinuousConvMode    = ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	//  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
 
  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 3 - Channel configuration ######################################## */
  /* Select Channel 0 to be converted */
  ADCConfig.Channel      = ADC_CHANNEL_0;
  ADCConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  ADCConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &ADCConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
	
}

static void Demodulator_Init(void)
{
	
	
		// For maximum accuracy, precalulate these values in MATLAB. Small rounding errors are present here. Of the order of 0.001% error.
	
	for(q31_t i=0; i<TRIG_SAMPLES; i++)
	{
		double fractpart, intpart;
		
		
		q31_t angle1_q31_t, angle2_q31_t, angle3_q31_t, angle4_q31_t, angle5_q31_t, angle6_q31_t, angle7_q31_t, angle8_q31_t;
		float angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8;
		
		angle1=(2*PI*Freq_Store[0]*i/F_Sample);
		if(angle1>(2*PI))
		{
			fractpart = modf(angle1/(2*PI), &intpart);
			angle1=2*PI*(float)fractpart;
		}
		angle1=angle1*(0.99999999999999999999999/(2*PI));
		
		angle2=(2*PI*Freq_Store[1]*i/F_Sample);
		if(angle2>(2*PI))
		{
			fractpart = modf(angle2/(2*PI), &intpart);
			angle2=2*PI*(float)fractpart;
		}
		angle2=angle2*(0.99999999999999999999999/(2*PI));		
		
		angle3=(2*PI*Freq_Store[2]*i/F_Sample);
		if(angle3>(2*PI))
		{
			fractpart = modf(angle3/(2*PI), &intpart);
			angle3=2*PI*(float)fractpart;
		}
		angle3=angle3*(0.99999999999999999999999/(2*PI));	
		
		angle4=(2*PI*Freq_Store[3]*i/F_Sample);
		if(angle4>(2*PI))
		{
			fractpart = modf(angle4/(2*PI), &intpart);
			angle4=2*PI*(float)fractpart;
		}
		angle4=angle4*(0.99999999999999999999999/(2*PI));

		angle5=(2*PI*Freq_Store[4]*i/F_Sample);
		if(angle5>(2*PI))
		{
			fractpart = modf(angle5/(2*PI), &intpart);
			angle5=2*PI*(float)fractpart;
		}
		angle5=angle5*(0.99999999999999999999999/(2*PI));		

		angle6=(2*PI*Freq_Store[5]*i/F_Sample);
		if(angle6>(2*PI))
		{
			fractpart = modf(angle6/(2*PI), &intpart);
			angle6=2*PI*(float)fractpart;
		}
		angle6=angle6*(0.99999999999999999999999/(2*PI));		

		angle7=(2*PI*Freq_Store[6]*i/F_Sample);
		
		if(angle7>(2*PI))
		{
			fractpart = modf(angle7/(2*PI), &intpart);
			angle7=2*PI*(float)fractpart;
		}
		angle7=angle7*(0.99999999999999999999999/(2*PI));			

		angle8=(2*PI*Freq_Store[7]*i/F_Sample);
		if(angle8>(2*PI))
		{
			fractpart = modf(angle8/(2*PI), &intpart);
			angle8=2*PI*(float)fractpart;
		}
		angle8=angle8*(0.99999999999999999999999/(2*PI));		
			
		arm_float_to_q31(&angle1,&angle1_q31_t,1);
		arm_float_to_q31(&angle2,&angle2_q31_t,1);
		arm_float_to_q31(&angle3,&angle3_q31_t,1);
		arm_float_to_q31(&angle4,&angle4_q31_t,1);
		arm_float_to_q31(&angle5,&angle5_q31_t,1);
		arm_float_to_q31(&angle6,&angle6_q31_t,1);
		arm_float_to_q31(&angle7,&angle7_q31_t,1);
		arm_float_to_q31(&angle8,&angle8_q31_t,1);
		
		Cosine_F1[i]=arm_cos_q31(angle1_q31_t)/trig_scale_factor; 
		Cosine_F2[i]=arm_cos_q31(angle2_q31_t)/trig_scale_factor; 
		Cosine_F3[i]=arm_cos_q31(angle3_q31_t)/trig_scale_factor; 
		Cosine_F4[i]=arm_cos_q31(angle4_q31_t)/trig_scale_factor; 
		Cosine_F5[i]=arm_cos_q31(angle5_q31_t)/trig_scale_factor; 
		Cosine_F6[i]=arm_cos_q31(angle6_q31_t)/trig_scale_factor; 
		Cosine_F7[i]=arm_cos_q31(angle7_q31_t)/trig_scale_factor; 
		Cosine_F8[i]=arm_cos_q31(angle8_q31_t)/trig_scale_factor; 
		
		Sine_F1[i]=arm_sin_q31(angle1_q31_t)/trig_scale_factor; 
		Sine_F2[i]=arm_sin_q31(angle2_q31_t)/trig_scale_factor; 
		Sine_F3[i]=arm_sin_q31(angle3_q31_t)/trig_scale_factor; 
		Sine_F4[i]=arm_sin_q31(angle4_q31_t)/trig_scale_factor; 
		Sine_F5[i]=arm_sin_q31(angle5_q31_t)/trig_scale_factor; 
		Sine_F6[i]=arm_sin_q31(angle6_q31_t)/trig_scale_factor; 
		Sine_F7[i]=arm_sin_q31(angle7_q31_t)/trig_scale_factor; 
		Sine_F8[i]=arm_sin_q31(angle8_q31_t)/trig_scale_factor; 
		
		
	
}
}


static void UART_Init(void)
{
		  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
			  /*##-2- Start the transmission process #####################################*/  
  /* While the UART in reception process, user can transmit data through 
     "aTxBuffer" buffer */
  if(HAL_UART_Transmit(&UartHandle, (uint8_t*)otherbuffer, strlen(otherbuffer), 5000)!= HAL_OK)
  {
    Error_Handler();   
  }
	
	
	
	
	
	
	
	
}
/**
  * @brief  Configures EXTI line 0 (connected to PA.00 pin) in interrupt mode
  * @param  None
  * @retval None
  */


static void EXTI0_1_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI line 0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
  * @brief  Configures EXTI line 0 (connected to PA.00 pin) in interrupt mode
  * @param  None
  * @retval None
  */


static void EXTI4_15_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI line 0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	  if (GPIO_Pin == GPIO_PIN_4)
			
		if(SPI_Init_Flag==0)
		{
			SPI_Init_Flag=1; // set the flag high forever
		}
	//if(1)
  {
		if(LED_Test_Flag==0)
		{
			LED_Test_Flag=1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // set high
		}
		else
		{
			LED_Test_Flag=0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // set high
		}
    /* Toggle LED3 */
    //BSP_LED_Toggle(LED3);
		
    SPI_Flag=1; // flag to run the SPI handler function
	
  }
	
	
//	  if (GPIO_Pin == GPIO_PIN_0)
//	//if(1)
//  {
//		if(LED_Test_Flag==0)
//		{
//			LED_Test_Flag=1;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // set high
//		}
//		else
//		{
//			LED_Test_Flag=0;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // set high
//		}
//    /* Toggle LED3 */
//    //BSP_LED_Toggle(LED3);
//		
//    //SPI_Flag=1;
//	
//  }
	
	
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED3 on: Transfer in transmission/reception process is correct */
 // BSP_LED_On(LED3);
	//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // set high
	//      HAL_Delay(100);
	//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // set low
	//      HAL_Delay(100);
	//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // set high
	//      HAL_Delay(100);
	//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // set low
	//      HAL_Delay(100);
	//	  if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aRxBuffer, BUFFERSIZE, 1000)!= HAL_OK)
  //{
  //  Error_Handler();   
  //}
	
	//  if(HAL_UART_Transmit(&UartHandle, (uint8_t*)otherbuffer2, strlen(otherbuffer2), 5000)!= HAL_OK)
  //{
  //  Error_Handler();   
  //}
	

	
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Timeout_Error_Handler(void)
{
  /* Toggle LED4 on */
  while(1)
  {
   // BSP_LED_On(LED4);
   // HAL_Delay(500);
   // BSP_LED_Off(LED4);
   // HAL_Delay(500);
  }
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
  /* Turn LED4 on: Transfer error in reception/transmission process */
 // BSP_LED_On(LED4);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  //BSP_LED_On(LED4);
  while (1)
  {
	//	BSP_LED_Toggle(LED4);
    HAL_Delay(1000);
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
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
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
