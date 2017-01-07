/**
  ******************************************************************************
  * @file    USB_Host/DualCore_Standalone/Src/hid_menu.c 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    22-April-2016
  * @brief   This file implements HID Menu Functions
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
extern HID_MOUSE_Info_TypeDef mouse_info;

/* Private function prototypes -----------------------------------------------*/
static void USBH_MouseDemo(USBH_HandleTypeDef *phost);
static void USBH_KeybdDemo(USBH_HandleTypeDef *phost);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Manages HID Menu Process.
  * @param  None
  * @retval None
  */
void HID_MenuProcess(void)
{
  switch(demo.hid_state)
  {
  case APPLI_HID_START:
    if(Appli_FS_state == APPLICATION_FS_READY)
    {
      if(USBH_HID_GetDeviceType(&hUSBHost_FS) == HID_KEYBOARD)
      {
        demo.keyboard_state = HID_KEYBOARD_IDLE; 
        demo.hid_state = APPLI_HID_KEYBOARD;
        
        BSP_LCD_ClearStringLine(19);
        BSP_LCD_ClearStringLine(20);
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_DisplayStringAtLine(20, (uint8_t *)"Press User button to exit");
        HID_KeyboardMenuProcess();
      }
      else if(USBH_HID_GetDeviceType(&hUSBHost_FS) == HID_MOUSE)
      {
        demo.mouse_state = HID_MOUSE_IDLE;  
        demo.hid_state = APPLI_HID_MOUSE;
        
        BSP_LCD_ClearStringLine(19);
        BSP_LCD_ClearStringLine(20);
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_DisplayStringAtLine(20, (uint8_t *)"Press User button to exit");
        
        HID_MouseMenuProcess();        
      }
    }
    else
    {
      LCD_ErrLog("No supported HID device!\n");
      demo.hid_state = APPLI_HID_START;
    }
    break;
    
  case APPLI_HID_MOUSE:
    if(Appli_FS_state == APPLICATION_FS_READY)
    {
      USBH_MouseDemo(&hUSBHost_FS);
      
      if(BSP_PB_GetState(BUTTON_TAMPER) == GPIO_PIN_SET)
      {
        demo.hid_state = APPLI_HID_START;
        demo.state = DEMO_IDLE;
      }
    }
    break; 
    
  case APPLI_HID_KEYBOARD:
    if(Appli_FS_state == APPLICATION_FS_READY)
    {
      USBH_KeybdDemo(&hUSBHost_FS);
      
      if(BSP_PB_GetState(BUTTON_TAMPER) == GPIO_PIN_SET)
      {
        demo.hid_state = APPLI_HID_START;
        demo.state = DEMO_IDLE;
      }
    }
    break;
    
  default:
    break;
  } 
}

/**
  * @brief  Main routine for Mouse application
  * @param  phost: Host handle
  * @retval None
  */
static void USBH_MouseDemo(USBH_HandleTypeDef *phost)
{
  HID_MOUSE_Info_TypeDef *m_pinfo;  
  
  m_pinfo = USBH_HID_GetMouseInfo(phost);
  
  if(m_pinfo != NULL)
  {
    /* Handle Mouse data position */
    USR_MOUSE_ProcessData(&mouse_info);
    
    if(m_pinfo->buttons[0])
    {
      HID_MOUSE_ButtonPressed(0);
    }
    else
    {
      HID_MOUSE_ButtonReleased(0);
    }
    
    if( m_pinfo->buttons[1])
    {
      HID_MOUSE_ButtonPressed(2);
    }
    else
    {
      HID_MOUSE_ButtonReleased(2);
    }
    
    if( m_pinfo->buttons[2])
    {
      HID_MOUSE_ButtonPressed(1);
    }
    else
    {
      HID_MOUSE_ButtonReleased(1);
    }
  }
}

/**
  * @brief  Main routine for Keyboard application
  * @param  phost: Host handle
  * @retval None
  */
static void USBH_KeybdDemo(USBH_HandleTypeDef *phost)
{
  HID_KEYBD_Info_TypeDef *k_pinfo; 
  char c;
  
  k_pinfo = USBH_HID_GetKeybdInfo(phost);
  
  if(k_pinfo != NULL)
  {
    c = USBH_HID_GetASCIICode(k_pinfo);
    if(c != 0)
    {
      USR_KEYBRD_ProcessData(c);
    }
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
