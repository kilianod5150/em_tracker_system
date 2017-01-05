/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComDMA/Src/main.c 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    27-May-2016
  * @brief   This sample code shows how to use UART HAL API to transmit
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
#define ARM_MATH_CM0
#include "main.h"
#include "math.h"
#include "core_cm0.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComDMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TRANSMITTER_BOARD
int	len;
uint8_t aTxBuffer2[] = " Hello World \n";
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;




/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/* ADC handle declaration */
ADC_HandleTypeDef             AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef        sConfig;

/* Converted value declaration */
uint32_t                      aResultDMA;

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
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  //HAL_NVIC_SetPriority(SysTick_IRQn,1,0);
	arm_biquad_casd_df1_inst_q31  S;
	arm_biquad_casd_df1_inst_q15  S2;
	uint32_t 	numStages=2;
	q31_t pCoeffs[10]={1,2,3,4,5,6,7,8,9,10};
	q31_t pState[8];
	q15_t pState2[8];
	arm_biquad_cascade_df1_init_q31(&S, numStages,(q31_t *)  &pCoeffs,&pState[0],0);
	arm_biquad_cascade_df1_init_q15(&S2, numStages,(q15_t *)  &pCoeffs,&pState2[0],0);

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
		
		
//		float cos_test=cos(angle1*(2*PI)/0.99999999999999999999999);
//		q31_t cos_test_q;
//		arm_float_to_q31(&cos_test,&Cosine_F1[i],1);
		
	}
	
	//arm_cos_q31
	
	HAL_Init();
  
  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
  

    /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 10000 - 1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }



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
  sConfig.Channel      = ADC_CHANNEL_0;
  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  


	  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  BSP_LED_On(LED3);

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

	
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

	HAL_Delay(2000) ;
	//Print out sine and cos table to check for errors
	   uint8_t aTxBuffer3[512];
	   	sprintf(aTxBuffer3,"Cos(F1) \t Cos(F2) \t Cos(F3) \t Cos(F4) \t Cos(F5) \t Cos(F6) \t Cos(F7) \t Cos(F8) \n \n");
	    HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
	
	HAL_Delay(100) ;
				for(int16_t i=0; i<TRIG_SAMPLES; i++)
		{
			sprintf(aTxBuffer3,"%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t \n",Cosine_F1[i],Cosine_F2[i],Cosine_F3[i],Cosine_F4[i],Cosine_F5[i],Cosine_F6[i],Cosine_F7[i],Cosine_F8[i]);
			HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
			HAL_Delay(250) ;
		}
	
  while (1)
  {
		

		
//	uint8_t aTxBuffer3[64];
//	
//		
//	
//	uint32_t trig_time_1=HAL_GetTick();
//	uint32_t out1;
//		uint32_t out2;
//		q31_t test_array[]={123,123};
//		q15_t test_out;
//			for(int16_t i=0; i<10000; i++)
//		{
//			
//	//out1  = arm_cos_q31	(	i	)	;
//	//		out1=arm_sqrt_q31(i,&out2);
//		//	arm_power_q31(test_array,2,&test_out);
//		//	arm_biquad_cascade_df1_fast_q31(&S,&i,&test_out,1);
//		//	arm_biquad_cascade_df1_fast_q15(&S2,&i,&test_out,1);
//		//	arm_sin_cos_q31(i,&out1,&out2);
//			
////	  if (HAL_ADC_Start_DMA(&AdcHandle, &aResultDMA, 1) != HAL_OK)
////  {
////    Error_Handler();
////  }
////	//uint32_t ADC_State_Check_1 =HAL_ADC_GetState(&AdcHandle);
////	
////	//HAL_ADC_PollForConversion(&AdcHandle, 250);
////	while(HAL_ADC_GetState(&AdcHandle)==256)
////	{
////		
////	}

//			    HAL_ADC_Start(&AdcHandle);

//        if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
//        {
//            out2 = HAL_ADC_GetValue(&AdcHandle);
//        }
//    
//			
//		}
//		
//	uint32_t trig_time_2=HAL_GetTick();
//	
//	sprintf(aTxBuffer3,"Trig Time = %d \n", trig_time_2-trig_time_1 );
//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
//	
//		

  }
	
	
	
	
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_LED_Toggle(LED3);
	
//	/* ### - 4 - Start conversion in DMA mode ################################# */
//  if (HAL_ADC_Start_DMA(&AdcHandle, &aResultDMA, 1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//	uint32_t ADC_State_Check_1 =HAL_ADC_GetState(&AdcHandle);
//	
//	//HAL_ADC_PollForConversion(&AdcHandle, 250);
//	while(HAL_ADC_GetState(&AdcHandle)==256)
//	{
//		
//	}

//	uint32_t ADC_State_Check_2 =HAL_ADC_GetState(&AdcHandle);
//	uint8_t aTxBuffer3[64];
//	sprintf(aTxBuffer3,"ADC = %d , ADC State 1 = %d, ADC State 2 = %d \n", aResultDMA ,ADC_State_Check_1,ADC_State_Check_2);
//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
//		
//	
//	uint32_t trig_time_1=HAL_GetTick();
//	
//			for(int32_t i=0; i<10000; i++)
//		{
//			
//	int32_t  cos_test= arm_cos_q31	(	i	)	;
//		}
//		
//	uint32_t trig_time_2=HAL_GetTick();
//	
//	sprintf(aTxBuffer3,"Trig Time = %d \n", trig_time_2-trig_time_1 );
//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
	
	  if(sample_counter<sample_counter_period)
		{
			
			// ADC grab
						    HAL_ADC_Start(&AdcHandle);

        if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
        {
            ADC_Value = HAL_ADC_GetValue(&AdcHandle);
        }
			
			
			// Sine/Cosine multiplication
			
			
			
			// Push to IIR Filter
			
			
		sample_counter++;
		}
		else
		{
			sample_counter=0;
		}
		
		
	
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
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Select HSE Oscillator as PLL source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
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

  /* Turn LED3 on: Transfer in transmission process is correct */
  BSP_LED_On(LED3); 
  
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

  /* Turn LED3 on: Transfer in reception process is correct */
  BSP_LED_On(LED3);
  
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
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {  
    UserButtonStatus = 1;
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
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
    /* Error if LED3 is slowly blinking (1 sec. period) */
    BSP_LED_Toggle(LED3); 
    HAL_Delay(1000); 
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
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
