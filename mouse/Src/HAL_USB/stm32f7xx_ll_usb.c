/**
  ******************************************************************************
  * @file    stm32f7xx_ll_usb.c
  * @author  MCD Application Team
  * @brief   USB Low Layer HAL module driver.
  *
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization/de-initialization functions
  *           + I/O operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Fill parameters of Init structure in USB_OTG_CfgTypeDef structure.

      (#) Call USB_CoreInit() API to initialize the USB Core peripheral.

      (#) The upper HAL HCD/PCD driver will call the right routines for its internal processes.

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/** @addtogroup STM32F7xx_LL_USB_DRIVER
  * @{
  */

#if defined (HAL_PCD_MODULE_ENABLED) || defined (HAL_HCD_MODULE_ENABLED)
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
#if 0
static HAL_StatusTypeDef USB_CoreReset(USB_OTG_GlobalTypeDef *USBx);

#ifdef USB_HS_PHYC
static HAL_StatusTypeDef USB_HS_PHYCInit(USB_OTG_GlobalTypeDef *USBx);
#endif
#endif
/* Exported functions --------------------------------------------------------*/
/** @defgroup USB_LL_Exported_Functions USB Low Layer Exported Functions
  * @{
  */

/** @defgroup USB_LL_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
                      ##### Initialization/de-initialization functions #####
 ===============================================================================

@endverbatim
  * @{
  */
#if 0
/**
  * @brief  Initializes the USB Core
  * @param  USBx USB Instance
  * @param  cfg pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg)
{
  HAL_StatusTypeDef ret;

  if (cfg.phy_itface == USB_OTG_ULPI_PHY)
  {
    USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);

    /* Init The ULPI Interface */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F730xx) || defined(STM32F732xx) || defined(STM32F733xx)
    /* Select ULPI Interface */
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
#endif /* defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F730xx) || defined(STM32F732xx) || defined(STM32F733xx) */

    /* Select vbus source */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
    if (cfg.use_external_vbus == 1U)
    {
      USBx->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;
    }
    /* Reset after a PHY select  */
    ret = USB_CoreReset(USBx);
  }
#ifdef USB_HS_PHYC
  else if (cfg.phy_itface == USB_OTG_HS_EMBEDDED_PHY)
  {
    USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);

    /* Init The UTMI Interface */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

    /* Select vbus source */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

    /* Select UTMI Interace */
    USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
    USBx->GCCFG |= USB_OTG_GCCFG_PHYHSEN;

    /* Enables control of a High Speed USB PHY */
    if (USB_HS_PHYCInit(USBx) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (cfg.use_external_vbus == 1U)
    {
      USBx->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;
    }
    /* Reset after a PHY select  */
    ret = USB_CoreReset(USBx);
  }
#endif
  else /* FS interface (embedded Phy) */
  {
    /* Select FS Embedded PHY */
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    /* Reset after a PHY select and set Host mode */
    ret = USB_CoreReset(USBx);

    /* Activate the USB Transceiver */
    USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;
  }

  if (cfg.dma_enable == 1U)
  {
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_2;
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
  }

  return ret;
}
#endif

/**
  * @brief  Set the USB turnaround time
  * @param  USBx USB Instance
  * @param  hclk: AHB clock frequency
  * @retval USB turnaround time In PHY Clocks number
  */
HAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef *USBx,
                                        uint32_t hclk, uint8_t speed)
{
  uint32_t UsbTrd;

  /* The USBTRD is configured according to the tables below, depending on AHB frequency
  used by application. In the low AHB frequency range it is used to stretch enough the USB response
  time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access
  latency to the Data FIFO */
  if (speed == USBD_FS_SPEED)
  {
    if ((hclk >= 14200000U) && (hclk < 15000000U))
    {
      /* hclk Clock Range between 14.2-15 MHz */
      UsbTrd = 0xFU;
    }
    else if ((hclk >= 15000000U) && (hclk < 16000000U))
    {
      /* hclk Clock Range between 15-16 MHz */
      UsbTrd = 0xEU;
    }
    else if ((hclk >= 16000000U) && (hclk < 17200000U))
    {
      /* hclk Clock Range between 16-17.2 MHz */
      UsbTrd = 0xDU;
    }
    else if ((hclk >= 17200000U) && (hclk < 18500000U))
    {
      /* hclk Clock Range between 17.2-18.5 MHz */
      UsbTrd = 0xCU;
    }
    else if ((hclk >= 18500000U) && (hclk < 20000000U))
    {
      /* hclk Clock Range between 18.5-20 MHz */
      UsbTrd = 0xBU;
    }
    else if ((hclk >= 20000000U) && (hclk < 21800000U))
    {
      /* hclk Clock Range between 20-21.8 MHz */
      UsbTrd = 0xAU;
    }
    else if ((hclk >= 21800000U) && (hclk < 24000000U))
    {
      /* hclk Clock Range between 21.8-24 MHz */
      UsbTrd = 0x9U;
    }
    else if ((hclk >= 24000000U) && (hclk < 27700000U))
    {
      /* hclk Clock Range between 24-27.7 MHz */
      UsbTrd = 0x8U;
    }
    else if ((hclk >= 27700000U) && (hclk < 32000000U))
    {
      /* hclk Clock Range between 27.7-32 MHz */
      UsbTrd = 0x7U;
    }
    else /* if(hclk >= 32000000) */
    {
      /* hclk Clock Range between 32-200 MHz */
      UsbTrd = 0x6U;
    }
  }
  else if (speed == USBD_HS_SPEED)
  {
    UsbTrd = USBD_HS_TRDT_VALUE;
  }
  else
  {
    UsbTrd = USBD_DEFAULT_TRDT_VALUE;
  }

  USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
  USBx->GUSBCFG |= (uint32_t)((UsbTrd << 10) & USB_OTG_GUSBCFG_TRDT);

  return HAL_OK;
}
#if 0
/**
  * @brief  USB_EnableGlobalInt
  *         Enables the controller's Global Int in the AHB Config reg
  * @param  USBx  Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
  return HAL_OK;
}

/**
  * @brief  USB_DisableGlobalInt
  *         Disable the controller's Global Int in the AHB Config reg
  * @param  USBx  Selected device
  * @retval HAL status
*/
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
  return HAL_OK;
}

/**
  * @brief  USB_SetCurrentMode : Set functional mode
  * @param  USBx  Selected device
  * @param  mode   current core mode
  *          This parameter can be one of these values:
  *            @arg USB_DEVICE_MODE: Peripheral mode
  *            @arg USB_HOST_MODE: Host mode
  *            @arg USB_DRD_MODE: Dual Role Device mode
  * @retval HAL status
  */
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_OTG_ModeTypeDef mode)
{
  USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);

  if (mode == USB_HOST_MODE)
  {
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;
  }
  else if (mode == USB_DEVICE_MODE)
  {
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
  }
  else
  {
    return HAL_ERROR;
  }
  HAL_Delay(50U);

  return HAL_OK;
}

/**
  * @brief  USB_DevInit : Initializes the USB_OTG controller registers
  *         for device mode
  * @param  USBx  Selected device
  * @param  cfg   pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t i;

  for (i = 0U; i < 15U; i++)
  {
    USBx->DIEPTXF[i] = 0U;
  }

  /* VBUS Sensing setup */
  if (cfg.vbus_sensing_enable == 0U)
  {
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;

    /* Deactivate VBUS Sensing B */
    USBx->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

    /* B-peripheral session valid override enable */
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  }
  else
  {
    /* Enable HW VBUS sensing */
    USBx->GCCFG |= USB_OTG_GCCFG_VBDEN;
  }

  /* Restart the Phy Clock */
  USBx_PCGCCTL = 0U;

  /* Device mode configuration */
  USBx_DEVICE->DCFG |= DCFG_FRAME_INTERVAL_80;

  if (cfg.phy_itface == USB_OTG_ULPI_PHY)
  {
    if (cfg.speed == USBD_HS_SPEED)
    {
      /* Set Core speed to High speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH);
    }
    else
    {
      /* Set Core speed to Full speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH_IN_FULL);
    }
  }
  else if (cfg.phy_itface == USB_OTG_HS_EMBEDDED_PHY)
  {
    if (cfg.speed == USBD_HS_SPEED)
    {
      /* Set Core speed to High speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH);
    }
    else
    {
      /* Set Core speed to Full speed mode */
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH_IN_FULL);
    }
  }
  else
  {
    /* Set Core speed to Full speed mode */
    (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_FULL);
  }

  /* Flush the FIFOs */
  if (USB_FlushTxFifo(USBx, 0x10U) != HAL_OK) /* all Tx FIFOs */
  {
    ret = HAL_ERROR;
  }

  if (USB_FlushRxFifo(USBx) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  /* Clear all pending Device Interrupts */
  USBx_DEVICE->DIEPMSK = 0U;
  USBx_DEVICE->DOEPMSK = 0U;
  USBx_DEVICE->DAINTMSK = 0U;

  for (i = 0U; i < cfg.dev_endpoints; i++)
  {
    if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
    {
      if (i == 0U)
      {
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
      }
      else
      {
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
      }
    }
    else
    {
      USBx_INEP(i)->DIEPCTL = 0U;
    }

    USBx_INEP(i)->DIEPTSIZ = 0U;
    USBx_INEP(i)->DIEPINT  = 0xFB7FU;
  }

  for (i = 0U; i < cfg.dev_endpoints; i++)
  {
    if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      if (i == 0U)
      {
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
      }
      else
      {
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
      }
    }
    else
    {
      USBx_OUTEP(i)->DOEPCTL = 0U;
    }

    USBx_OUTEP(i)->DOEPTSIZ = 0U;
    USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
  }

  USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

  /* Disable all interrupts. */
  USBx->GINTMSK = 0U;

  /* Clear any pending interrupts */
  USBx->GINTSTS = 0xBFFFFFFFU;

  /* Enable the common interrupts */
  if (cfg.dma_enable == 0U)
  {
    USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
  }

  /* Enable interrupts matching to the Device mode ONLY */
  USBx->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
                   USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
                   USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_WUIM;

  if (cfg.vbus_sensing_enable == 1U)
  {
    USBx->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
  }

  return ret;
}

/**
  * @brief  USB_OTG_FlushTxFifo : Flush a Tx FIFO
  * @param  USBx  Selected device
  * @param  num  FIFO number
  *         This parameter can be a value from 1 to 15
            15 means Flush all Tx FIFOs
  * @retval HAL status
  */
HAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num)
{
  uint32_t count = 0U;

  USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << 6));

  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

  return HAL_OK;
}

/**
  * @brief  USB_FlushRxFifo : Flush Rx FIFO
  * @param  USBx  Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t count = 0;

  USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

  return HAL_OK;
}

/**
  * @brief  USB_SetDevSpeed  Initializes the DevSpd field of DCFG register
  *         depending the PHY type and the enumeration speed of the device.
  * @param  USBx  Selected device
  * @param  speed  device speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_HIGH_IN_FULL: High speed core in Full Speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  * @retval  Hal status
  */
HAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx, uint8_t speed)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_DEVICE->DCFG |= speed;
  return HAL_OK;
}
#endif
/**
  * @brief  USB_GetDevSpeed  Return the Dev Speed
  * @param  USBx  Selected device
  * @retval speed  device speed
  *          This parameter can be one of these values:
  *            @arg PCD_SPEED_HIGH: High speed mode
  *            @arg PCD_SPEED_FULL: Full speed mode
  */
uint8_t USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint8_t speed;
  uint32_t DevEnumSpeed = USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD;

  if (DevEnumSpeed == DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ)
  {
    speed = USBD_HS_SPEED;
  }
  else if ((DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ) ||
           (DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_48MHZ))
  {
    speed = USBD_FS_SPEED;
  }
  else
  {
    speed = 0xFU;
  }

  return speed;
}

/**
  * @brief  Activate and configure an endpoint
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK));

    if ((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U)
    {
      USBx_INEP(epnum)->DIEPCTL |= (ep->maxpacket & USB_OTG_DIEPCTL_MPSIZ) |
                                   ((uint32_t)ep->type << 18) | (epnum << 22) |
                                   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                   USB_OTG_DIEPCTL_USBAEP;
    }
  }
  else
  {
    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16);

    if (((USBx_OUTEP(epnum)->DOEPCTL) & USB_OTG_DOEPCTL_USBAEP) == 0U)
    {
      USBx_OUTEP(epnum)->DOEPCTL |= (ep->maxpacket & USB_OTG_DOEPCTL_MPSIZ) |
                                    ((uint32_t)ep->type << 18) |
                                    USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                    USB_OTG_DOEPCTL_USBAEP;
    }
  }
  return HAL_OK;
}

/**
  * @brief  De-activate and de-initialize an endpoint
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  /* Read DEPCTLn register */
  if (ep->is_in == 1U)
  {
    if ((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
    {
      USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
      USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
    }

    USBx_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
    USBx_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
    USBx_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_USBAEP |
                                   USB_OTG_DIEPCTL_MPSIZ |
                                   USB_OTG_DIEPCTL_TXFNUM |
                                   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                   USB_OTG_DIEPCTL_EPTYP);
  }
  else
  {
    if ((USBx_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
      USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
    }

    USBx_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
    USBx_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
    USBx_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_USBAEP |
                                    USB_OTG_DOEPCTL_MPSIZ |
                                    USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
                                    USB_OTG_DOEPCTL_EPTYP);
  }

  return HAL_OK;
}

/**
  * @brief  USB_EPStartXfer : setup and starts a transfer over an EP
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;
  uint16_t pktcnt;

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /* Zero Length Packet? */
    if (ep->xfer_len == 0U)
    {
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
    }
    else
    {
      /* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
      */
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket) << 19));
      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);

      if (ep->type == EP_TYPE_ISOC)
      {
        USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
        USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1U << 29));
      }
    }

      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

      if (ep->type != EP_TYPE_ISOC)
      {
        /* Enable the Tx FIFO Empty Interrupt for this EP */
        if (ep->xfer_len > 0U)
        {
          USBx_DEVICE->DIEPEMPMSK |= 1UL << (ep->num & EP_ADDR_MSK);
        }
      }
      else
      {
        if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
        {
          USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
        }
        else
        {
          USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }

        (void)USB_WritePacket(USBx, ep->xfer_buff, ep->num, (uint16_t)ep->xfer_len);
      }
  }
  else /* OUT endpoint */
  {
    /* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
    */
    USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
    USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

    if (ep->xfer_len == 0U)
    {
      USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->maxpacket);
      USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
    }
    else
    {
      pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
      USBx_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_PKTCNT & ((uint32_t)pktcnt << 19);
      USBx_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_XFRSIZ & (ep->maxpacket * pktcnt);
    }

    if (ep->type == EP_TYPE_ISOC)
    {
      if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
      {
        USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
      }
      else
      {
        USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
      }
    }
    /* EP enable */
    USBx_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
  }

  return HAL_OK;
}

/**
  * @brief  USB_EP0StartXfer : setup and starts a transfer over the EP  0
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /* Zero Length Packet? */
    if (ep->xfer_len == 0U)
    {
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
    }
    else
    {
      /* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
      */
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);

      if (ep->xfer_len > ep->maxpacket)
      {
        ep->xfer_len = ep->maxpacket;
      }
      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);
    }

      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

      /* Enable the Tx FIFO Empty Interrupt for this EP */
      if (ep->xfer_len > 0U)
      {
        USBx_DEVICE->DIEPEMPMSK |= 1UL << (ep->num & EP_ADDR_MSK);
      }
  }
  else /* OUT endpoint */
  {
    /* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
    */
    USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
    USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

    if (ep->xfer_len > 0U)
    {
      ep->xfer_len = ep->maxpacket;
    }

    USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
    USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & (ep->maxpacket));

    /* EP enable */
    USBx_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
  }

  return HAL_OK;
}

/**
  * @brief  USB_WritePacket : Writes a packet into the Tx FIFO associated
  *         with the EP/channel
  * @param  USBx  Selected device
  * @param  src   pointer to source buffer
  * @param  ch_ep_num  endpoint or host channel number
  * @param  len  Number of bytes to write
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @retval HAL status
  */
HAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t *pSrc = (uint32_t *)src;
  uint32_t count32b, i;

    count32b = ((uint32_t)len + 3U) / 4U;
    for (i = 0U; i < count32b; i++)
    {
      USBx_DFIFO((uint32_t)ch_ep_num) = __UNALIGNED_UINT32_READ(pSrc);
      pSrc++;
    }

  return HAL_OK;
}

/**
  * @brief  USB_ReadPacket : read a packet from the RX FIFO
  * @param  USBx  Selected device
  * @param  dest  source pointer
  * @param  len  Number of bytes to read
  * @retval pointer to destination buffer
  */
void *USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t *pDest = (uint32_t *)dest;
  uint32_t i;
  uint32_t count32b = ((uint32_t)len + 3U) / 4U;

  for (i = 0U; i < count32b; i++)
  {
    __UNALIGNED_UINT32_WRITE(pDest, USBx_DFIFO(0U));
    pDest++;
  }

  return ((void *)pDest);
}

/**
  * @brief  USB_EPSetStall : set a stall condition over an EP
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    if (((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U) && (epnum != 0U))
    {
      USBx_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);
    }
    USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  }
  else
  {
    if (((USBx_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U) && (epnum != 0U))
    {
      USBx_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);
    }
    USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
  }

  return HAL_OK;
}

/**
  * @brief  USB_EPClearStall : Clear a stall condition over an EP
  * @param  USBx  Selected device
  * @param  ep pointer to endpoint structure
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    USBx_INEP(epnum)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
    {
      USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM; /* DATA0 */
    }
  }
  else
  {
    USBx_OUTEP(epnum)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
    if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
    {
      USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM; /* DATA0 */
    }
  }
  return HAL_OK;
}


/**
  * @brief  USB_SetDevAddress : Stop the usb device mode
  * @param  USBx  Selected device
  * @param  address  new device address to be assigned
  *          This parameter can be a value from 0 to 255
  * @retval HAL status
  */
HAL_StatusTypeDef  USB_SetDevAddress(USB_OTG_GlobalTypeDef *USBx, uint8_t address)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
  USBx_DEVICE->DCFG |= ((uint32_t)address << 4) & USB_OTG_DCFG_DAD;

  return HAL_OK;
}
#if 0
/**
  * @brief  USB_DevConnect : Connect the USB device by enabling the pull-up/pull-down
  * @param  USBx  Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef  USB_DevConnect(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
  HAL_Delay(3U);

  return HAL_OK;
}

/**
  * @brief  USB_DevDisconnect : Disconnect the USB device by disabling the pull-up/pull-down
  * @param  USBx  Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef  USB_DevDisconnect(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
  HAL_Delay(3U);

  return HAL_OK;
}
#endif
/**
  * @brief  USB_ReadInterrupts: return the global USB interrupt status
  * @param  USBx  Selected device
  * @retval HAL status
  */
uint32_t  USB_ReadInterrupts(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t tmpreg;

  tmpreg = USBx->GINTSTS;
  tmpreg &= USBx->GINTMSK;

  return tmpreg;
}

/**
  * @brief  USB_ReadDevAllOutEpInterrupt: return the USB device OUT endpoints interrupt status
  * @param  USBx  Selected device
  * @retval HAL status
  */
uint32_t USB_ReadDevAllOutEpInterrupt(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_DEVICE->DAINT;
  tmpreg &= USBx_DEVICE->DAINTMSK;

  return ((tmpreg & 0xffff0000U) >> 16);
}

/**
  * @brief  USB_ReadDevAllInEpInterrupt: return the USB device IN endpoints interrupt status
  * @param  USBx  Selected device
  * @retval HAL status
  */
uint32_t USB_ReadDevAllInEpInterrupt(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_DEVICE->DAINT;
  tmpreg &= USBx_DEVICE->DAINTMSK;

  return ((tmpreg & 0xFFFFU));
}

/**
  * @brief  Returns Device OUT EP Interrupt register
  * @param  USBx  Selected device
  * @param  epnum  endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device OUT EP Interrupt register
  */
uint32_t USB_ReadDevOutEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_OUTEP((uint32_t)epnum)->DOEPINT;
  tmpreg &= USBx_DEVICE->DOEPMSK;

  return tmpreg;
}

/**
  * @brief  Returns Device IN EP Interrupt register
  * @param  USBx  Selected device
  * @param  epnum  endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device IN EP Interrupt register
  */
uint32_t USB_ReadDevInEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg, msk, emp;

  msk = USBx_DEVICE->DIEPMSK;
  emp = USBx_DEVICE->DIEPEMPMSK;
  msk |= ((emp >> (epnum & EP_ADDR_MSK)) & 0x1U) << 7;
  tmpreg = USBx_INEP((uint32_t)epnum)->DIEPINT & msk;

  return tmpreg;
}
#if 0
/**
  * @brief  Returns USB core mode
  * @param  USBx  Selected device
  * @retval return core mode : Host or Device
  *          This parameter can be one of these values:
  *           0 : Host
  *           1 : Device
  */
uint32_t USB_GetMode(USB_OTG_GlobalTypeDef *USBx)
{
  return ((USBx->GINTSTS) & 0x1U);
}
#endif
/**
  * @brief  Activate EP0 for Setup transactions
  * @param  USBx  Selected device
  * @retval HAL status
  */
HAL_StatusTypeDef  USB_ActivateSetup(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* Set the MPS of the IN EP0 to 64 bytes */
  USBx_INEP(0U)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;

  USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;

  return HAL_OK;
}

/**
  * @brief  Prepare the EP0 to start the first control setup
  * @param  USBx  Selected device
  * @param  dma USB dma enabled or disabled
  *          This parameter can be one of these values:
  *           0 : DMA feature not used
  *           1 : DMA feature used
  * @param  psetup  pointer to setup packet
  * @retval HAL status
  */
HAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USBx->CID + 0x1U);

  if (gSNPSiD > USB_OTG_CORE_ID_300A)
  {
    if ((USBx_OUTEP(0U)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      return HAL_OK;
    }
  }

  USBx_OUTEP(0U)->DOEPTSIZ = 0U;
  USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
  USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
  USBx_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;

  return HAL_OK;
}
#if 0
/**
  * @brief  Reset the USB Core (needed after USB clock settings change)
  * @param  USBx  Selected device
  * @retval HAL status
  */
static HAL_StatusTypeDef USB_CoreReset(USB_OTG_GlobalTypeDef *USBx)
{
  uint32_t count = 0U;

  /* Wait for AHB master IDLE state. */
  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  /* Core Soft Reset */
  count = 0U;
  USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

  return HAL_OK;
}

#ifdef USB_HS_PHYC
/**
  * @brief  Enables control of a High Speed USB PHYs
  *         Init the low level hardware : GPIO, CLOCK, NVIC...
  * @param  USBx  Selected device
  * @retval HAL status
  */
static HAL_StatusTypeDef USB_HS_PHYCInit(USB_OTG_GlobalTypeDef *USBx)
{
  UNUSED(USBx);
  uint32_t count = 0U;

  /* Enable LDO */
  USB_HS_PHYC->USB_HS_PHYC_LDO |= USB_HS_PHYC_LDO_ENABLE;

  /* wait for LDO Ready */
  while ((USB_HS_PHYC->USB_HS_PHYC_LDO & USB_HS_PHYC_LDO_STATUS) == 0U)
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }

  /* Controls PHY frequency operation selection */
  if (HSE_VALUE == 12000000U) /* HSE = 12MHz */
  {
    USB_HS_PHYC->USB_HS_PHYC_PLL = (0x0U << 1);
  }
  else if (HSE_VALUE == 12500000U) /* HSE = 12.5MHz */
  {
    USB_HS_PHYC->USB_HS_PHYC_PLL = (0x2U << 1);
  }
  else if (HSE_VALUE == 16000000U) /* HSE = 16MHz */
  {
    USB_HS_PHYC->USB_HS_PHYC_PLL = (0x3U << 1);
  }
  else if (HSE_VALUE == 24000000U) /* HSE = 24MHz */
  {
    USB_HS_PHYC->USB_HS_PHYC_PLL = (0x4U << 1);
  }
  else if (HSE_VALUE == 25000000U) /* HSE = 25MHz */
  {
    USB_HS_PHYC->USB_HS_PHYC_PLL = (0x5U << 1);
  }
  else if (HSE_VALUE == 32000000U) /* HSE = 32MHz */
  {
    USB_HS_PHYC->USB_HS_PHYC_PLL = (0x7U << 1);
  }
  else
  {
    /* ... */
  }

  /* Control the tuning interface of the High Speed PHY */
  USB_HS_PHYC->USB_HS_PHYC_TUNE |= USB_HS_PHYC_TUNE_VALUE;

  /* Enable PLL internal PHY */
  USB_HS_PHYC->USB_HS_PHYC_PLL |= USB_HS_PHYC_PLL_PLLEN;

  /* 2ms Delay required to get internal phy clock stable */
  HAL_Delay(2U);

  return HAL_OK;
}
#endif //0
#endif /* USB_HS_PHYC */

#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @}
  */

/**
  * @}
  */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
#endif /* defined (HAL_PCD_MODULE_ENABLED) || defined (HAL_HCD_MODULE_ENABLED) */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
