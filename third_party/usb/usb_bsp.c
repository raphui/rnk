/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11-September-2013
  * @brief   This file is responsible to offer board support package and is 
  *          configurable by user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "usb.h"

//void system_init( const char *path ) ;

void OTG_FS_WKUP_IRQHandler(void)
{
  if(UsbDriverData->USB_OTG_dev.cfg.low_power)
  {
	/* Reset SLEEPDEEP and SLEEPONEXIT bits */
	//SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

	/* After wake-up from sleep mode, reconfigure the system clock */
	//system_init( "/ahb1/rcc" ) ;//SystemInit();
    USB_OTG_UngateClock(&UsbDriverData->USB_OTG_dev);
  }
  //EXTI_ClearITPendingBit(EXTI_Line18);
}

/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
#ifdef CONFIG_USB_IF
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

	USBD_OTG_ISR_Handler (&UsbDriverData->USB_OTG_dev);
#endif
}

void _OTG_FS_IRQHandler( device *dev, int a, int b )
{
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
	usb_priv *priv = dev->priv ;
	USBD_OTG_ISR_Handler (&priv->USB_OTG_dev);
}

