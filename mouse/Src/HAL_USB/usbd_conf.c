/**
  ******************************************************************************
  * @file    USB_Device/HID_Standalone/Src/usbd_conf.c
  * @author  MCD Application Team
  * @brief   This file implements the USB Device library callbacks and MSP
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
#include "stm32f7xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

/** @addtogroup STM32F7xx_HAL_Applications
  * @{
  */

/** @addtogroup USB_Device_HID_Standalone
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd; // moved to usb.c
//__IO uint32_t remotewakeupon = 0;
//uint8_t HID_Buffer[4];
//extern USBD_HandleTypeDef USBD_Device; // unused

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
  
/*******************************************************************************
                       PCD BSP Routines
*******************************************************************************/
#if 0
/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
	/* Enable HS PHY Clock */
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_OTGPHYCEN);

	/* Enable USB HS Clocks */
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSEN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSULPIEN);

	/* Set USBHS Interrupt to the lowest priority */
	NVIC_SetPriority(OTG_HS_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));

	/* Enable USBHS Interrupt */
	NVIC_EnableIRQ(OTG_HS_IRQn);
}
#endif
///**
//  * @brief  De-Initializes the PCD MSP.
//  * @param  hpcd: PCD handle
//  * @retval None
//  */
//void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
//{
//  if(hpcd->Instance == USB_OTG_FS)
//  {
//    /* Disable USB FS Clock */
////    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
//	  RCC->AHB2ENR &= ~(RCC_AHB2ENR_OTGFSEN);
////    __HAL_RCC_SYSCFG_CLK_DISABLE();
//		RCC->APB2ENR &= ~(RCC_APB2ENR_SYSCFGEN);
//  }
//  else if(hpcd->Instance == USB_OTG_HS)
//  {
//    /* Disable USB HS Clocks */
////    __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
//	  RCC->AHB1ENR &= ~(RCC_AHB1ENR_OTGHSEN);
////    __HAL_RCC_SYSCFG_CLK_DISABLE();
//		RCC->APB2ENR &= ~(RCC_APB2ENR_SYSCFGEN);
//  }
//}

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

/**
  * @brief  SetupStage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
//  USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
  USBD_HandleTypeDef *pdev = hpcd->pData;
  uint8_t *psetup = (uint8_t *)hpcd->Setup;

  USBD_ParseSetupRequest(&pdev->request, psetup);

  pdev->ep0_state = USBD_EP0_SETUP;

  pdev->ep0_data_len = pdev->request.wLength;

  switch (pdev->request.bmRequest & 0x1FU)
  {
    case USB_REQ_RECIPIENT_DEVICE:
      USBD_StdDevReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_INTERFACE:
      USBD_StdItfReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_ENDPOINT:
      USBD_StdEPReq(pdev, &pdev->request);
      break;

    default:
      USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
      break;
  }
}

/**
  * @brief  DataOut Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) // TODO: this does nothing?
{
//  USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
  USBD_HandleTypeDef *pdev = hpcd->pData;
  uint8_t *pdata = hpcd->OUT_ep[epnum].xfer_buff;
  USBD_EndpointTypeDef *pep;
//  USBD_StatusTypeDef ret;

  if (epnum == 0U)
  {
	pep = &pdev->ep_out[0];

	if (pdev->ep0_state == USBD_EP0_DATA_OUT)
	{
	  if (pep->rem_length > pep->maxpacket)
	  {
		pep->rem_length -= pep->maxpacket;

		(void)USBD_CtlContinueRx(pdev, pdata, MIN(pep->rem_length, pep->maxpacket));
	  }
	  else
	  {
		if ((pdev->pClass->EP0_RxReady != NULL) &&
			(pdev->dev_state == USBD_STATE_CONFIGURED))
		{
		  pdev->pClass->EP0_RxReady(pdev);
		}
		(void)USBD_CtlSendStatus(pdev);
	  }
	}
//	else
//	{
//#if 0
//	  if (pdev->ep0_state == USBD_EP0_STATUS_OUT)
//	  {
//		/*
//		 * STATUS PHASE completed, update ep0_state to idle
//		 */
//		pdev->ep0_state = USBD_EP0_IDLE;
//		(void)USBD_LL_StallEP(pdev, 0U);
//	  }
//#endif
//	}
  }
//  else if ((pdev->pClass->DataOut != NULL) &&
//		   (pdev->dev_state == USBD_STATE_CONFIGURED))
//  {
//	ret = (USBD_StatusTypeDef)pdev->pClass->DataOut(pdev, epnum);
//
//	if (ret != USBD_OK)
//	{
//	  return ret;
//	}
//  }
//  else
//  {
//	/* should never be in this condition */
//	return USBD_FAIL;
//  }

//  return USBD_OK;
}

/**
  * @brief  DataIn Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
//  USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
  USBD_HandleTypeDef *pdev = hpcd->pData;
  uint8_t *pdata = hpcd->IN_ep[epnum].xfer_buff;

  USBD_EndpointTypeDef *pep;
//  USBD_StatusTypeDef ret;
///  extern char abcd[1000];
///  extern int a;

  if (epnum == 0U)
  {
    pep = &pdev->ep_in[0];

    if (pdev->ep0_state == USBD_EP0_DATA_IN)
    {
      if (pep->rem_length > pep->maxpacket)
      {///abcd[a++]='x';
        pep->rem_length -= pep->maxpacket;

        (void)USBD_CtlContinueSendData(pdev, pdata, pep->rem_length);

        /* Prepare endpoint for premature end of transfer */
       (void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
      }
      else
      {
        /* last packet is MPS multiple, so send ZLP packet */
        if ((pep->maxpacket == pep->rem_length) &&
            (pep->total_length >= pep->maxpacket) &&
            (pep->total_length < pdev->ep0_data_len))
        {///abcd[a++]='y';
          (void)USBD_CtlContinueSendData(pdev, NULL, 0U);
          pdev->ep0_data_len = 0U;

          /* Prepare endpoint for premature end of transfer */
          (void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
        }
        else
        {///abcd[a++]='z';
          if ((pdev->pClass->EP0_TxSent != NULL) &&
              (pdev->dev_state == USBD_STATE_CONFIGURED))
          {
            pdev->pClass->EP0_TxSent(pdev);
          }
          (void)USBD_LL_StallEP(pdev, 0x80U); // TODO not necessary?
          (void)USBD_CtlReceiveStatus(pdev);
        }
      }
    }
    else
    {
#if 0
      if ((pdev->ep0_state == USBD_EP0_STATUS_IN) ||
          (pdev->ep0_state == USBD_EP0_IDLE))
      {
        (void)USBD_LL_StallEP(pdev, 0x80U);
      }
#endif
    }

//    if (pdev->dev_test_mode == 1U)
//    {
//      (void)USBD_RunTestMode(pdev);
//      pdev->dev_test_mode = 0U;
//    }
  }
  else if ((pdev->pClass->DataIn != NULL) &&
           (pdev->dev_state == USBD_STATE_CONFIGURED))
  {
//    ret = (USBD_StatusTypeDef)
	USBD_HID_DataIn(pdev, epnum);

//    if (ret != USBD_OK)
//    {
//      return ret;
//    }
  }
//  else
//  {
//    /* should never be in this condition */
//    return USBD_FAIL;
//  }
//
//  return USBD_OK;
}


/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{   
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
  
  /* Set USB Current Speed */
  switch(hpcd->Init.speed)
  {
  case PCD_SPEED_HIGH:
    speed = USBD_SPEED_HIGH;
    break;
    
  case PCD_SPEED_FULL:
    speed = USBD_SPEED_FULL;
    break;   
    
  default:
    speed = USBD_SPEED_FULL;
    break;
  }
  
  /* Reset Device */
//  USBD_LL_Reset(hpcd->pData);
  USBD_HandleTypeDef *pdev = hpcd->pData;
  /* Upon Reset call user call back */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->ep0_state = USBD_EP0_IDLE;
  pdev->dev_config = 0U;
  pdev->dev_remote_wakeup = 0U;

  if (pdev->pClassData != NULL)
  {
    USBD_HID_DeInit(pdev, (uint8_t)pdev->dev_config);
  }

    /* Open EP0 OUT */
  (void)USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_out[0x00U & 0xFU].is_used = 1U;

  pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  (void)USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_in[0x80U & 0xFU].is_used = 1U;

  pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

  
//  USBD_LL_SetSpeed(hpcd->pData, speed);
  ((USBD_HandleTypeDef *)hpcd->pData)->dev_speed = speed;
}

/**
  * @brief  Suspend callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
//    USBD_LL_Suspend(hpcd->pData);
	USBD_HandleTypeDef *pdev = hpcd->pData;
	pdev->dev_old_state = pdev->dev_state;
	pdev->dev_state = USBD_STATE_SUSPENDED;
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
}

/**
  * @brief  Resume callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  __HAL_PCD_UNGATE_PHYCLOCK(hpcd);
//  USBD_LL_Resume(hpcd->pData);
  USBD_HandleTypeDef *pdev = hpcd->pData;
  if (pdev->dev_state == USBD_STATE_SUSPENDED)
  {
    pdev->dev_state = pdev->dev_old_state;
  }
}

#if 0
/**
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected(hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected(hpcd->pData);
}
#endif

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/
#if 0
/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
//#ifdef USE_USB_HS
  /* Set LL Driver parameters */
  hpcd.Instance = USB_OTG_HS; // code assumes HS
  hpcd.Init.dev_endpoints = 9;
  hpcd.Init.use_dedicated_ep1 = 0;
  
  /* Be aware that enabling DMA mode will result in data being sent only by
  multiple of 4 packet sizes. This is due to the fact that USB DMA does
  not allow sending data from non word-aligned addresses.
  For this specific application, it is advised to not enable this option
  unless required. */
  hpcd.Init.dma_enable = 0; // code assumes 0
  hpcd.Init.low_power_enable = 0; // code assumes 0
  hpcd.Init.lpm_enable = 0; // code assumes 0
  hpcd.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY; 
  hpcd.Init.Sof_enable = 0; // code assumes 0
  hpcd.Init.speed = PCD_SPEED_HIGH;
  hpcd.Init.vbus_sensing_enable = 1;
  
  /* Link The driver to the stack */
  hpcd.pData = pdev;
  pdev->pData = &hpcd;
  
  /* Initialize LL Driver */
  HAL_PCD_Init(&hpcd);
  
  HAL_PCDEx_SetRxFiFo(&hpcd, 0x200);
  HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x174);
//#endif
  
  return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
//USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
//{
//  HAL_PCD_DeInit(pdev->pData);
//  return USBD_OK;
//}

/**
  * @brief  Starts the Low Level portion of the Device driver. 
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK;
}
#endif
///**
//  * @brief  Stops the Low Level portion of the Device driver.
//  * @param  pdev: Device handle
//  * @retval USBD Status
//  */
//USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
//{
//  HAL_PCD_Stop(pdev->pData);
//  return USBD_OK;
//}



/**
  * @brief  Set the USB Device address.
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t address)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  __HAL_LOCK(hpcd);
  hpcd->USB_Address = address;
  (void)USB_SetDevAddress(hpcd->Instance, address);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}
/**
  * @brief  Open and configure an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packet size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev,
                                  uint8_t ep_addr,
                                  uint8_t ep_type,
                                  uint16_t ep_mps)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  HAL_StatusTypeDef  ret = HAL_OK;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->num = ep_addr & EP_ADDR_MSK;
  ep->maxpacket = ep_mps;
  ep->type = ep_type;

  if (ep->is_in != 0U)
  {
    /* Assign a Tx FIFO */
    ep->tx_fifo_num = ep->num;
  }
  /* Set initial data PID. */
  if (ep_type == EP_TYPE_BULK)
  {
    ep->data_pid_start = 0U;
  }

  __HAL_LOCK(hpcd);
  (void)USB_ActivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return ret;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }
  ep->num   = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_DeactivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}


/**
  * @brief  Receive an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval HAL status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                                          uint8_t ep_addr,
                                          uint8_t *pBuf,
                                          uint32_t len)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  PCD_EPTypeDef *ep;

  ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(hpcd->Instance, ep);
  }
  else
  {
    (void)USB_EPStartXfer(hpcd->Instance, ep);
  }

  return HAL_OK;
}

/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval HAL status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev,
                                    uint8_t ep_addr,
                                    uint8_t *pBuf,
                                    uint32_t len)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(hpcd->Instance, ep);
  }
  else
  {
    (void)USB_EPStartXfer(hpcd->Instance, ep);
  }

  return HAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
    ep->is_in = 0U;
  }

  ep->is_stall = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);

  (void)USB_EPSetStall(hpcd->Instance, ep);
  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0_OutStart(hpcd->Instance);
  }
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->is_stall = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_EPClearStall(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}


#if 0
/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev,
                                  uint8_t ep_addr,
                                  uint8_t ep_type,
                                  uint16_t ep_mps)
{
  HAL_PCD_EP_Open(pdev->pData,
                  ep_addr,
                  ep_mps,
                  ep_type);
  
  return USBD_OK;
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK; 
}
#endif
/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  
  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}
#if 0
/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK; 
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size    
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, 
                                    uint8_t ep_addr,
                                    uint8_t *pbuf,
                                    uint32_t size)
{
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, 
                                          uint8_t ep_addr,
                                          uint8_t *pbuf,
                                          uint32_t size)
{
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

