/**
  ******************************************************************************
  * @file    USB_Host/DualCore_Standalone/Src/msc_menu.c 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    22-April-2016
  * @brief   Mass Storage Process
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern DEMO_StateMachine demo;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Manages MSC Menu Process.
  * @param  None
  * @retval None
  */
void MSC_MenuProcess(void)
{
  switch(demo.msc_state)
  {  
  case APPLI_MSC_START:
    if(Appli_HS_state == APPLICATION_HS_READY)
    {
      BSP_LCD_ClearStringLine(19);
      BSP_LCD_ClearStringLine(20);
       
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(19, (uint8_t *)"Press User button to start read and write operations");
      
      /* Wait for User Input */
      while((BSP_PB_GetState(BUTTON_TAMPER) != SET) && (Appli_HS_state != APPLICATION_HS_DISCONNECT))
      {
      }
      demo.msc_state = APPLI_MSC_FILE_OPERATIONS;
      
      /* Prevent debounce effect for user key */
      HAL_Delay(400);
      
      BSP_LCD_ClearStringLine(19);
    }
    break;
    
  case APPLI_MSC_FILE_OPERATIONS:  
    /* Read and Write File Here */
    if(Appli_HS_state == APPLICATION_HS_READY)
    {
      MSC_File_Operations();
      
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(19, (uint8_t *)"Press User button to display disk content");
      
      /* Wait for User Input */
      while((BSP_PB_GetState(BUTTON_TAMPER) != SET) && (Appli_HS_state != APPLICATION_HS_DISCONNECT))
      {
      }
      demo.msc_state = APPLI_MSC_EXPLORER;
      
      /* Prevent debounce effect for user key */
      HAL_Delay(400);
      
      BSP_LCD_ClearStringLine(19);
    }
    break; 
    
  case APPLI_MSC_EXPLORER:
    /* Display disk content */
    if(Appli_HS_state == APPLICATION_HS_READY)
    {        
      Explore_Disk("0:/", 1);
      demo.msc_state = APPLI_MSC_START;
      demo.state = DEMO_IDLE;
      
      BSP_LCD_ClearStringLine(19);
      BSP_LCD_ClearStringLine(20);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(20, (uint8_t *)"Press User button to exit");
      
      /* Prevent debounce effect for user key */
      HAL_Delay(400);
      
      /* Wait for User Input */
      while((BSP_PB_GetState(BUTTON_TAMPER) != SET) && (Appli_HS_state != APPLICATION_HS_DISCONNECT))
      {
      }
      
      /* Prevent debounce effect for user key */
      HAL_Delay(400);
    }
    break; 
    
  default:
    break;
  }
} 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
