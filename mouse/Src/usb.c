#include "usb.h"
//#include "usbd_def.h"
//#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"
#include "stm32f7xx_hal.h"

PCD_HandleTypeDef hpcd;
USBD_HandleTypeDef USBD_Device;

static void FlushRxFifo(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);
}

static void FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num)
{
  USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << 6));
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);
}

// TODO argument USB_OTG_GlobalTypeDef
static void SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size)
{
  hpcd->Instance->GRXFSIZ = size;
}

static void SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size)
{
	uint32_t Tx_Offset = hpcd->Instance->GRXFSIZ;
	if (fifo == 0U) {
		hpcd->Instance->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;
	} else {
		Tx_Offset += (hpcd->Instance->DIEPTXF0_HNPTXFSIZ) >> 16;
		for (int i = 0; i < (fifo - 1); i++) {
			Tx_Offset += (hpcd->Instance->DIEPTXF[i] >> 16);
		}
		hpcd->Instance->DIEPTXF[fifo - 1] = ((uint32_t)size << 16) | Tx_Offset;
	}
}

void usb_init(int hs_usb)
{
	// USBD_Init(&USBD_Device, &HID_Desc, 0)
	USBD_Device.pClass = NULL;
	USBD_Device.pConfDesc = NULL;
	USBD_Device.pDesc = &HID_Desc;
	USBD_Device.dev_state = USBD_STATE_DEFAULT;
	USBD_Device.id = 0;
	// USBD_LL_Init(pdev)
	hpcd.Instance = USB_OTG_HS;
	hpcd.Init.dev_endpoints = 9;
	hpcd.Init.speed = hs_usb ? PCD_SPEED_HIGH : PCD_SPEED_HIGH_IN_FULL;
	hpcd.Init.vbus_sensing_enable = 0;
	hpcd.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY; // assumes
	hpcd.Init.use_dedicated_ep1 = 0; // code assumes 0
	hpcd.Init.dma_enable = 0; // code assumes 0
	hpcd.Init.low_power_enable = 0; // code assumes 0
	hpcd.Init.lpm_enable = 0; // code assumes 0
	hpcd.Init.Sof_enable = 0; // code assumes 0
	hpcd.pData = &USBD_Device;
	USBD_Device.pData = &hpcd;
	// HAL_PCD_Init(&hpcd)
	hpcd.Lock = HAL_UNLOCKED;
	// HAL_PCD_MspInit(hpcd)
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_OTGPHYCEN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSEN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSULPIEN);
//	NVIC_SetPriority(OTG_HS_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(OTG_HS_IRQn);
	// HAL_PCD_Init
	hpcd.State = HAL_PCD_STATE_BUSY;
	hpcd.Instance->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	// USB_CoreInit(hpcd->Instance, hpcd->Init)
	hpcd.Instance->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
	hpcd.Instance->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);
	hpcd.Instance->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
	hpcd.Instance->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
	hpcd.Instance->GCCFG |= USB_OTG_GCCFG_PHYHSEN;
	// USB_HS_PHYCInit(USBx)
	USB_HS_PHYC->USB_HS_PHYC_LDO |= USB_HS_PHYC_LDO_ENABLE;
	while ((USB_HS_PHYC->USB_HS_PHYC_LDO & USB_HS_PHYC_LDO_STATUS) == 0U);
	if (HSE_VALUE == 24000000U) {
		USB_HS_PHYC->USB_HS_PHYC_PLL = (0x4U << 1);
	} else if (HSE_VALUE == 25000000U) {
		USB_HS_PHYC->USB_HS_PHYC_PLL = (0x5U << 1);
	}
	USB_HS_PHYC->USB_HS_PHYC_TUNE |= USB_HS_PHYC_TUNE_VALUE;
	USB_HS_PHYC->USB_HS_PHYC_PLL |= USB_HS_PHYC_PLL_PLLEN;
	HAL_Delay(2);
	// USB_CoreReset(USBx)
	while ((hpcd.Instance->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);
	hpcd.Instance->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
	while ((hpcd.Instance->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);
	// USB_SetCurrentMode(hpcd->Instance, USB_DEVICE_MODE)
	hpcd.Instance->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
	hpcd.Instance->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
	HAL_Delay(50U);
	// HAL_PCD_Init
	for (int i = 0; i < hpcd.Init.dev_endpoints; i++) {
		hpcd.IN_ep[i].is_in = 1U;
		hpcd.IN_ep[i].num = i;
		hpcd.IN_ep[i].tx_fifo_num = i;
		hpcd.IN_ep[i].type = EP_TYPE_CTRL;
		hpcd.IN_ep[i].maxpacket = 0U;
		hpcd.IN_ep[i].xfer_buff = 0U;
		hpcd.IN_ep[i].xfer_len = 0U;

		hpcd.OUT_ep[i].is_in = 0U;
		hpcd.OUT_ep[i].num = i;
		hpcd.OUT_ep[i].type = EP_TYPE_CTRL;
		hpcd.OUT_ep[i].maxpacket = 0U;
		hpcd.OUT_ep[i].xfer_buff = 0U;
		hpcd.OUT_ep[i].xfer_len = 0U;
	}
	// USB_DevInit(hpcd->Instance, hpcd->Init)
	uint32_t USBx_BASE = (uint32_t)hpcd.Instance;

	for (int i = 0; i < 15; i++)
		hpcd.Instance->DIEPTXF[i] = 0U;

	if (hpcd.Init.vbus_sensing_enable == 0U) {
		USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
		hpcd.Instance->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
		hpcd.Instance->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
		hpcd.Instance->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
	} else {
		hpcd.Instance->GCCFG |= USB_OTG_GCCFG_VBDEN;
	}

	USBx_PCGCCTL = 0U;
	USBx_DEVICE->DCFG |= DCFG_FRAME_INTERVAL_80;
	if (hpcd.Init.speed == USBD_HS_SPEED) {
		USBx_DEVICE->DCFG |= USB_OTG_SPEED_HIGH;
	} else {
		USBx_DEVICE->DCFG |= USB_OTG_SPEED_HIGH_IN_FULL;
	}

	FlushTxFifo(hpcd.Instance, 0x10U);
	FlushRxFifo(hpcd.Instance);

	USBx_DEVICE->DIEPMSK = 0U;
	USBx_DEVICE->DOEPMSK = 0U;
	USBx_DEVICE->DAINTMSK = 0U;

	for (int i = 0; i < hpcd.Init.dev_endpoints; i++) {
		if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
			if (i == 0U) {
				USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
			} else {
				USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
			}
		} else {
			USBx_INEP(i)->DIEPCTL = 0U;
		}
		USBx_INEP(i)->DIEPTSIZ = 0U;
		USBx_INEP(i)->DIEPINT  = 0xFB7FU;
	}
	for (int i = 0; i < hpcd.Init.dev_endpoints; i++) {
		if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			if (i == 0U) {
				USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
			} else {
				USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
			}
		} else {
			USBx_OUTEP(i)->DOEPCTL = 0U;
		}
		USBx_OUTEP(i)->DOEPTSIZ = 0U;
		USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
	}

	USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

	hpcd.Instance->GINTMSK = 0U;
	hpcd.Instance->GINTSTS = 0xBFFFFFFFU;
	hpcd.Instance->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
	hpcd.Instance->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
				   USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
				   USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_WUIM;

	if (hpcd.Init.vbus_sensing_enable == 1U) {
		hpcd.Instance->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
	}
	// HAL_PCD_Init
	hpcd.USB_Address = 0;
	hpcd.State = HAL_PCD_STATE_READY;
	//(void)USB_DevDisconnect(hpcd.Instance); // not necessary
	// USBD_LL_Init
	SetRxFiFo(&hpcd, 0x200);
	// TODO changeback
//	SetTxFiFo(&hpcd, 0, 0x80);
	SetTxFiFo(&hpcd, 0, 0x20);
	SetTxFiFo(&hpcd, 1, 0x174);
	// USBD_RegisterClass(&USBD_Device, USBD_HID_CLASS)
	USBD_Device.pClass = USBD_HID_CLASS;
	USBD_Device.pConfDesc = (void *)USBD_HID_GetHSCfgDesc((uint16_t []){0}); // argument unused
	// USBD_Start(&USBD_Device)
	while (hpcd.Lock == HAL_LOCKED);
	hpcd.Lock = HAL_LOCKED;
	USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
	HAL_Delay(3);
	hpcd.Instance->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
	hpcd.Lock = HAL_UNLOCKED;
}

void usb_wait_configured(void)
{
	volatile uint8_t *state = &USBD_Device.dev_state;
	while (*state != USBD_STATE_CONFIGURED)
		__WFI();
}

///char abcd[1000];
///int a;

/**
  * @brief  Check FIFO for the next packet to be loaded.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USB_OTG_EPTypeDef *ep;
  uint32_t len;
  uint32_t len32b;
  uint32_t fifoemptymsk;

  ep = &hpcd->IN_ep[epnum];

  if (ep->xfer_count > ep->xfer_len)
  {
    return HAL_ERROR;
  }

  len = ep->xfer_len - ep->xfer_count;

  if (len > ep->maxpacket)
  {
    len = ep->maxpacket;
  }

  len32b = (len + 3U) / 4U;

  while (((USBx_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= len32b) &&
         (ep->xfer_count < ep->xfer_len) && (ep->xfer_len != 0U))
  {
    /* Write the FIFO */
    len = ep->xfer_len - ep->xfer_count;

    if (len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }
    len32b = (len + 3U) / 4U;

    (void)USB_WritePacket(USBx, ep->xfer_buff, (uint8_t)epnum, (uint16_t)len);

    ep->xfer_buff  += len;
    ep->xfer_count += len;
  }

  if (ep->xfer_len <= ep->xfer_count)
  {
    fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
    USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
  }

  return HAL_OK;
}


/**
  * @brief  process EP OUT transfer complete interrupt.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_OutXfrComplete_int(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USBx->CID + 0x1U);

    if (gSNPSiD == USB_OTG_CORE_ID_310A)
    {
      /* StupPktRcvd = 1 this is a setup packet */
   	  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;
      if ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
      }
      else
      {
        if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
        {
          CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
        }

        HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
      }
    }
    else
    {
      if ((epnum == 0U) && (hpcd->OUT_ep[epnum].xfer_len == 0U))
      {
        /* this is ZLP, so prepare EP0 for next setup */
        (void)USB_EP0_OutStart(hpcd->Instance);
      }

      HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
    }

  return HAL_OK;
}


/**
  * @brief  process EP OUT setup packet received interrupt.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_OutSetupPacket_int(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USBx->CID + 0x1U);
  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

  if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
      ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
  {
    CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
  }

  /* Inform the upper layer that a setup packet is available */
  HAL_PCD_SetupStageCallback(hpcd);

  return HAL_OK;
}

///USBD_SetupReqTypedef stps[50];
///int istp;

void OTG_HS_IRQHandler(void)
{
	if ((USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_SOF) != 0) {
		USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_SOF;
		return;
	}

  USB_OTG_GlobalTypeDef *USBx = hpcd.Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;

	// this is disabled by main loop
	USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;

  uint32_t i, ep_intr, epint, epnum;
  uint32_t fifoemptymsk, temp;
  USB_OTG_EPTypeDef *ep;
///  abcd[a++]=',';
     /* Handle RxQLevel Interrupt */
    if (__HAL_PCD_GET_FLAG(&hpcd, USB_OTG_GINTSTS_RXFLVL))
    {///abcd[a++]='R';
      USB_MASK_INTERRUPT(hpcd.Instance, USB_OTG_GINTSTS_RXFLVL);
      temp = USBx->GRXSTSP;
      uint32_t pktsts = (temp & USB_OTG_GRXSTSP_PKTSTS) >> 17;
      ep = &hpcd.OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];
      if (pktsts == STS_SETUP_UPDT)
      {///abcd[a++]='0'+pktsts;
        (void)USB_ReadPacket(USBx, (uint8_t *)hpcd.Setup, 8U);
///USBD_ParseSetupRequest(&stps[istp++], (uint8_t *)hpcd.Setup);
        ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
      }
      USB_UNMASK_INTERRUPT(hpcd.Instance, USB_OTG_GINTSTS_RXFLVL);
    }

    if (__HAL_PCD_GET_FLAG(&hpcd, USB_OTG_GINTSTS_OEPINT))
    {///abcd[a++]='O';
      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllOutEpInterrupt(hpcd.Instance);
      for (epnum = 0; epnum < hpcd.Init.dev_endpoints; epnum++)
      {
        if ((ep_intr & (1 << epnum)) != 0)
        {
          epint = USB_ReadDevOutEPInterrupt(hpcd.Instance, (uint8_t)epnum);

          if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC)
          {///abcd[a++]='1';
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);
            (void)PCD_EP_OutXfrComplete_int(&hpcd, epnum);
          }

          if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP)
          {///abcd[a++]='2';
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
            /* Class B setup phase done for previous decoded setup */
            (void)PCD_EP_OutSetupPacket_int(&hpcd, epnum);
          }
        }
      }
    }

    if (__HAL_PCD_GET_FLAG(&hpcd, USB_OTG_GINTSTS_IEPINT))
    {///abcd[a++]='I';
      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllInEpInterrupt(hpcd.Instance);

      for (epnum = 0; epnum < hpcd.Init.dev_endpoints; epnum++)
      {
        if ((ep_intr & (1 << epnum)) != 0)
        {
          epint = USB_ReadDevInEPInterrupt(hpcd.Instance, (uint8_t)epnum);

          if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC)
          {///abcd[a++]='1';
            fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
            USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);

            HAL_PCD_DataInStageCallback(&hpcd, (uint8_t)epnum);
          }

          if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
          {///abcd[a++]='6';
            (void)PCD_WriteEmptyTxFifo(&hpcd, epnum);
          }
        }
      }
    }

    /* Handle Resume Interrupt */
    if (__HAL_PCD_GET_FLAG(&hpcd, USB_OTG_GINTSTS_WKUINT))
    {///abcd[a++]='W';
      /* Clear the Remote Wake-up Signaling */
      USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
      HAL_PCD_ResumeCallback(&hpcd);
      __HAL_PCD_CLEAR_FLAG(&hpcd, USB_OTG_GINTSTS_WKUINT);
    }

    /* Handle Suspend Interrupt */
    if (__HAL_PCD_GET_FLAG(&hpcd, USB_OTG_GINTSTS_USBSUSP))
    {///abcd[a++]='S';
      if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
        HAL_PCD_SuspendCallback(&hpcd);
      }
      __HAL_PCD_CLEAR_FLAG(&hpcd, USB_OTG_GINTSTS_USBSUSP);
    }

    /* Handle Reset Interrupt */
    if (__HAL_PCD_GET_FLAG(&hpcd, USB_OTG_GINTSTS_USBRST)) {///abcd[a++]='T';
      USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
      FlushTxFifo(hpcd.Instance, 0x10);
      for (i = 0U; i < hpcd.Init.dev_endpoints; i++) {
        USBx_INEP(i)->DIEPINT = 0xFB7FU;
        USBx_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        USBx_INEP(i)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
        USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
        USBx_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        USBx_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
      }
      USBx_DEVICE->DAINTMSK |= 0x10001U;
      USBx_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
      USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;
      USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD; /* Set Default Address to 0 */
      (void)USB_EP0_OutStart(hpcd.Instance); /* setup EP0 to receive SETUP packets */
      __HAL_PCD_CLEAR_FLAG(&hpcd, USB_OTG_GINTSTS_USBRST);
    }

    /* Handle Enumeration done Interrupt */
    if (__HAL_PCD_GET_FLAG(&hpcd, USB_OTG_GINTSTS_ENUMDNE))
    {///abcd[a++]='E';
      (void)USB_ActivateSetup(hpcd.Instance);
      hpcd.Init.speed = USB_GetDevSpeed(hpcd.Instance);
      /* Set USB Turnaround time */
      (void)USB_SetTurnaroundTime(hpcd.Instance,
                                  216000000, // HAL_RCC_GetHCLKFreq(), //hard code
                                  (uint8_t)hpcd.Init.speed);
      HAL_PCD_ResetCallback(&hpcd);
      __HAL_PCD_CLEAR_FLAG(&hpcd, USB_OTG_GINTSTS_ENUMDNE);
    }
}

