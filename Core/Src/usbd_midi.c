/**
 ******************************************************************************
 * @file    usbd_audio.c
 * @author  MCD Application Team
 * @brief   This file provides the Audio core functions.
 *
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 * @verbatim
 *
 *          ===================================================================
 *                                AUDIO Class  Description
 *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
 *           Audio Devices V1.0 Mar 18, 98".
 *           This driver implements the following aspects of the specification:
 *             - Device descriptor management
 *             - Configuration descriptor management
 *             - Standard AC Interface Descriptor management
 *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
 *             - 1 Audio Streaming Endpoint
 *             - 1 Audio Terminal Input (1 channel)
 *             - Audio Class-Specific AC Interfaces
 *             - Audio Class-Specific AS Interfaces
 *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
 *             - Audio Feature Unit (limited to Mute control)
 *             - Audio Synchronization type: Asynchronous
 *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
 *          The current audio class version supports the following audio features:
 *             - Pulse Coded Modulation (PCM) format
 *             - sampling rate: 48KHz.
 *             - Bit resolution: 16
 *             - Number of channels: 2
 *             - No volume control
 *             - Mute/Unmute capability
 *             - Asynchronous Endpoints
 *
 * @note     In HS mode and when the DMA is used, all variables and data structures
 *           dealing with the DMA during the transaction process should be 32-bit aligned.
 *
 *
 *  @endverbatim
 ******************************************************************************
 */

/* BSPDependencies
 - "stm32xxxxx_{eval}{discovery}.c"
 - "stm32xxxxx_{eval}{discovery}_io.c"
 - "stm32xxxxx_{eval}{discovery}_audio.c"
 EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi.h"
#include "usbd_ctlreq.h"

#include <math.h>
/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_AUDIO
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Macros
 * @{
 */
#ifndef AUDIO_SAMPLE_FREQ
#define AUDIO_SAMPLE_FREQ(frq) \
  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))
#endif

#ifndef AUDIO_PACKET_SZE
#define AUDIO_PACKET_SZE(frq) \
  (uint8_t)(((frq * 2U * 2U) / 1000U) & 0xFFU), (uint8_t)((((frq * 2U * 2U) / 1000U) >> 8) & 0xFFU)
#endif

#ifdef USE_USBD_COMPOSITE
#define AUDIO_PACKET_SZE_WORD(frq)     (uint32_t)((((frq) * 2U * 2U)/1000U))
#endif /* USE_USBD_COMPOSITE  */
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
 * @{
 */
static uint8_t USBD_MIDI_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_MIDI_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_MIDI_Setup(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req);
#ifndef USE_USBD_COMPOSITE
static uint8_t* USBD_MIDI_GetCfgDesc(uint16_t *length);
static uint8_t* USBD_MIDI_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */
static uint8_t USBD_MIDI_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_MIDI_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_MIDI_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_MIDI_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_MIDI_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_MIDI_IsoINIncomplete(USBD_HandleTypeDef *pdev,
		uint8_t epnum);
static uint8_t USBD_MIDI_IsoOutIncomplete(USBD_HandleTypeDef *pdev,
		uint8_t epnum);
static void MIDI_REQ_GetCurrent(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req);
static void MIDI_REQ_SetCurrent(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req);
static void* USBD_MIDI_GetAudioHeaderDesc(uint8_t *pConfDesc);

/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Variables
 * @{
 */

USBD_ClassTypeDef USBD_MIDI = { USBD_MIDI_Init, USBD_MIDI_DeInit,
		USBD_MIDI_Setup, USBD_MIDI_EP0_TxReady, USBD_MIDI_EP0_RxReady,
		USBD_MIDI_DataIn, USBD_MIDI_DataOut, USBD_MIDI_SOF,
		USBD_MIDI_IsoINIncomplete, USBD_MIDI_IsoOutIncomplete,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
		USBD_MIDI_GetCfgDesc, // USB Highspeed
		USBD_MIDI_GetCfgDesc,  // USB Fullspeed
		USBD_MIDI_GetCfgDesc,  // USB Other speed
		USBD_MIDI_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE  */
		};

#ifndef USE_USBD_COMPOSITE
/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CfgDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END
= {
/* Configuration 1 */
0x09, /* bLength */
USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType */
LOBYTE(USB_MIDI_CONFIG_DESC_SIZ), /* wTotalLength */
HIBYTE(USB_MIDI_CONFIG_DESC_SIZ), 0x02, /* bNumInterfaces */
0x01, /* bConfigurationValue */
0x00, /* iConfiguration */
//#if (USBD_SELF_POWERED == 1U)
//  0xC0,                                 /* bmAttributes: Bus Powered according to user configuration */
//#else
//  0x80,                                 /* bmAttributes: Bus Powered according to user configuration */
//#endif /* USBD_SELF_POWERED */
		0xC0,
		0xFA,//USBD_MAX_POWER, /* MaxPower (mA) */
		/* 09 byte*/

		/* USB Speaker Standard interface descriptor */
		AUDIO_INTERFACE_DESC_SIZE, /* bLength */
		USB_DESC_TYPE_INTERFACE, /* bDescriptorType */
		0x00, /* bInterfaceNumber */
		0x00, /* bAlternateSetting */
		0x00, /* bNumEndpoints */
		USB_DEVICE_CLASS_AUDIO, /* bInterfaceClass */
		AUDIO_SUBCLASS_AUDIOCONTROL, /* bInterfaceSubClass */
		AUDIO_PROTOCOL_UNDEFINED, /* bInterfaceProtocol */
		0x00, /* iInterface */
		/* 09 byte*/

		/* USB Speaker Class-specific AC Interface Descriptor */
		AUDIO_INTERFACE_DESC_SIZE, /* bLength */
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
		AUDIO_CONTROL_HEADER, /* bDescriptorSubtype */
		0x00, /* 1.00 *//* bcdADC */
		0x01, 0x1E, /* wTotalLength */
		0x00, 0x01, /* bInCollection */
		0x01, /* baInterfaceNr */
		/* 09 byte*/

		/* USB Speaker Input Terminal Descriptor */
		AUDIO_INPUT_TERMINAL_DESC_SIZE, /* bLength */
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
		AUDIO_CONTROL_INPUT_TERMINAL, /* bDescriptorSubtype */
		0x01, /* bTerminalID */
		0x01, /* wTerminalType microphone   0x0201 */
		0x02, 0x00, /* bAssocTerminal */
		0x01, /* bNrChannels */
		0x00, /* wChannelConfig 0x0000  Mono */
		0x00, 0x00, /* iChannelNames */
		0x00, /* iTerminal */
		/* 12 byte*/

		/* USB Speaker Output Terminal Descriptor */
		0x09, /* bLength */
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
		AUDIO_CONTROL_OUTPUT_TERMINAL, /* bDescriptorSubtype */
		0x02, /* bTerminalID */
		0x01, /* wTerminalType  0x0101 , this used to be 0x0301 because it was originally
		 prorammed for powering a speaker perhaps.  consult  Universal Serial Bus Device Class Definition for Terminal Types.pdf*/
		0x01, 0x00, /* bAssocTerminal */
		0x01, /* bSourceID */
		0x00, /* iTerminal */
		/* 09 byte */

		/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */
		/* Interface 1, Alternate Setting 0                                              */
		AUDIO_INTERFACE_DESC_SIZE, /* bLength */
		USB_DESC_TYPE_INTERFACE, /* bDescriptorType */
		0x01, /* bInterfaceNumber */
		0x00, /* bAlternateSetting */
		0x00, /* bNumEndpoints */
		USB_DEVICE_CLASS_AUDIO, /* bInterfaceClass */
		AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
		AUDIO_PROTOCOL_UNDEFINED, /* bInterfaceProtocol */
		0x00, /* iInterface */
		/* 09 byte*/

		/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
		/* Interface 1, Alternate Setting 1                                           */
		AUDIO_INTERFACE_DESC_SIZE, /* bLength */
		USB_DESC_TYPE_INTERFACE, /* bDescriptorType */
		0x01, /* bInterfaceNumber */
		0x01, /* bAlternateSetting */
		0x01, /* bNumEndpoints */
		USB_DEVICE_CLASS_AUDIO, /* bInterfaceClass */
		AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
		AUDIO_PROTOCOL_UNDEFINED, /* bInterfaceProtocol */
		0x00, /* iInterface */
		/* 09 byte*/

		/* USB Speaker Audio Streaming Interface Descriptor */
		AUDIO_STREAMING_INTERFACE_DESC_SIZE, /* bLength */
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
		AUDIO_STREAMING_GENERAL, /* bDescriptorSubtype */
		0x02, /* bTerminalLink */
		0x00, /* bDelay */
		0x01, /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */
		0x00,
		/* 07 byte*/

		/* USB Speaker Audio Type III Format Interface Descriptor */
		0x0B, /* bLength */
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
		AUDIO_STREAMING_FORMAT_TYPE, /* bDescriptorSubtype */
		AUDIO_FORMAT_TYPE_I, /* bFormatType */
		0x01, /* bNrChannels */
		0x02, /* bSubFrameSize :  2 Bytes per frame (16bits) */
		16, /* bBitResolution (16-bits per sample) */
		0x01, /* bSamFreqType only one frequency supported */
		AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
		/* 11 byte*/

		/* Endpoint 1 - Standard Descriptor */
		AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
		USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
		AUDIO_IN_EP, /* bEndpointAddress 1 in endpoint */
//				USBD_EP_TYPE_BULK,
//				USBD_EP_TYPE_INTR,
//		0x05,
//		0b00001101,
		0x0D,//USBD_EP_TYPE_ISOC,
		//				0b00001101, /* bmAttributes */
//  AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
//  (uint8_t)(((USBD_AUDIO_FREQ * 2U * 2U) / 1000U) & 0xFFU),
//  (uint8_t)((((USBD_AUDIO_FREQ * 2U * 2U) / 1000U) >> 8) & 0xFFU),
		//0x00, 0x02,
//		0x00,0x02,
		MIC_PACKET_SZE(USBD_AUDIO_FREQ),
		AUDIO_FS_BINTERVAL, /* bInterval */
		0x00, /* bRefresh */
		0x00, /* bSynchAddress */
		/* 09 byte*/

		/* Endpoint - Audio Streaming Descriptor */
		AUDIO_STREAMING_ENDPOINT_DESC_SIZE, /* bLength */
		AUDIO_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType */
		AUDIO_ENDPOINT_GENERAL, /* bDescriptor */
		0x00, /* bmAttributes */
		0x02, /* bLockDelayUnits */
		0x00, /* wLockDelay */
		0x00,
/* 07 byte*/
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END
		= {
		USB_LEN_DEV_QUALIFIER_DESC,
		USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
				0x01, 0x00, };
#endif /* USE_USBD_COMPOSITE  */

volatile uint32_t tx_flag = 1;
volatile uint32_t fnsof = 0;

static uint8_t AUDIOOutEpAdd = AUDIO_OUT_EP;
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Functions
 * @{
 */

/**
 * @brief  USBD_AUDIO_Init
 *         Initialize the AUDIO interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_MIDI_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	UNUSED(cfgidx);
	USBD_MIDI_HandleTypeDef *haudio;

	/* Allocate Audio structure */
	haudio = (USBD_MIDI_HandleTypeDef*) USBD_malloc (
	sizeof(USBD_MIDI_HandleTypeDef));

	haudio->data_in_called = 0;

//  pdev->pClassData = (void *)haudio;
	if (haudio == NULL) {
		pdev->pClassDataCmsit[pdev->classId] = NULL;
		return (uint8_t) USBD_EMEM;
	}

	pdev->pClassDataCmsit[pdev->classId] = (void*) haudio;
	pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];
	/* Open IN endpoint (mic) */
	pdev->ep_in[AUDIO_IN_EP & 0xFU].bInterval = AUDIO_FS_BINTERVAL;

	USBD_LL_OpenEP(pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
	pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

	haudio->data_ready_flag = 0;
	haudio->data_in_called = 0;
	haudio->omega = 0.0f;
	haudio->alt_setting = 0U;
	haudio->offset = MIDI_OFFSET_UNKNOWN;
	haudio->wr_ptr = 0U;
	haudio->rd_ptr = 0U;
	haudio->rd_enable = 0U;

	USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
	USBD_LL_Transmit(pdev, AUDIO_IN_EP,
			(uint8_t*) &haudio->in_buffer[(AUDIO_IN_PACKET/2)
					* !!haudio->in_buffer_half], AUDIO_IN_PACKET);

	return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_Init
 *         DeInitialize the AUDIO layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_MIDI_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	UNUSED(cfgidx);

//#ifdef USE_USBD_COMPOSITE
//  /* Get the Endpoints addresses allocated for this class instance */
//  AUDIOOutEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC);
//#endif /* USE_USBD_COMPOSITE */

	/* Open EP OUT */
	USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
	(void) USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
	pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;
	pdev->ep_in[AUDIO_IN_EP & 0xFU].bInterval = 0U;

	/* Clear feedback transmission flag */
	tx_flag = 0U;

	/* DeInit  physical Interface components */
	if (pdev->pClassDataCmsit[pdev->classId] != NULL) {
		((USBD_MIDI_ItfTypeDef*) pdev->pUserData[pdev->classId])->DeInit(0U);
		(void) USBD_free(pdev->pClassDataCmsit[pdev->classId]);
		pdev->pClassDataCmsit[pdev->classId] = NULL;
		pdev->pClassData = NULL;
	}

	return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_Setup
 *         Handle the AUDIO specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_MIDI_Setup(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req) {
	USBD_MIDI_HandleTypeDef *haudio;
	uint16_t len;
	uint8_t *pbuf;
	uint16_t status_info = 0U;
	USBD_StatusTypeDef ret = USBD_OK;

	haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

	if (haudio == NULL) {
		return (uint8_t) USBD_FAIL;
	}

	switch (req->bmRequest & USB_REQ_TYPE_MASK) {
	case USB_REQ_TYPE_CLASS:
		switch (req->bRequest) {
		case AUDIO_REQ_GET_CUR:
			MIDI_REQ_GetCurrent(pdev, req);
			break;

		case AUDIO_REQ_SET_CUR:
			MIDI_REQ_SetCurrent(pdev, req);
			break;

		default:
			USBD_CtlError(pdev, req);
			ret = USBD_FAIL;
			break;
		}
		break;

	case USB_REQ_TYPE_STANDARD:
		switch (req->bRequest) {
		case USB_REQ_GET_STATUS:
			if (pdev->dev_state == USBD_STATE_CONFIGURED) {
				(void) USBD_CtlSendData(pdev, (uint8_t*) &status_info, 2U);
			} else {
				USBD_CtlError(pdev, req);
				ret = USBD_FAIL;
			}
			break;

		case USB_REQ_GET_DESCRIPTOR:
			if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE) {
				pbuf = (uint8_t*) USBD_MIDI_GetAudioHeaderDesc(pdev->pConfDesc);
				if (pbuf != NULL) {
					len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);
					(void) USBD_CtlSendData(pdev, pbuf, len);
				} else {
					USBD_CtlError(pdev, req);
					ret = USBD_FAIL;
				}
			}
			break;

		case USB_REQ_GET_INTERFACE:
			if (pdev->dev_state == USBD_STATE_CONFIGURED) {
				(void) USBD_CtlSendData(pdev, (uint8_t*) &haudio->alt_setting,
						1U);
			} else {
				USBD_CtlError(pdev, req);
				ret = USBD_FAIL;
			}
			break;

		case USB_REQ_SET_INTERFACE:
			if (pdev->dev_state == USBD_STATE_CONFIGURED) {
				if ((uint8_t) (req->wValue) <= USBD_MAX_NUM_INTERFACES) {
					haudio->alt_setting = (uint8_t) (req->wValue);
					USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
				} else {
					/* Call the error management function (command will be NAKed */
					USBD_CtlError(pdev, req);
					ret = USBD_FAIL;
				}
			} else {
				USBD_CtlError(pdev, req);
				ret = USBD_FAIL;
			}
			break;

		case USB_REQ_CLEAR_FEATURE:
			break;

		default:
			USBD_CtlError(pdev, req);
			ret = USBD_FAIL;
			break;
		}
		break;
	default:
		USBD_CtlError(pdev, req);
		ret = USBD_FAIL;
		break;
	}

	return (uint8_t) ret;
}

#ifndef USE_USBD_COMPOSITE
/**
 * @brief  USBD_AUDIO_GetCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_MIDI_GetCfgDesc(uint16_t *length) {
	*length = (uint16_t) sizeof(USBD_MIDI_CfgDesc);

	return USBD_MIDI_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
 * @brief  USBD_AUDIO_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MIDI_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
//	UNUSED(pdev);
//	UNUSED(epnum);

	if (epnum == (AUDIO_IN_EP & 0x7F)) {
		tx_flag = 0U;
	}
	return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_MIDI_EP0_RxReady(USBD_HandleTypeDef *pdev) {
	USBD_MIDI_HandleTypeDef *haudio;
	haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

	if (haudio == NULL) {
		return (uint8_t) USBD_FAIL;
	}

	if (haudio->control.cmd == AUDIO_REQ_SET_CUR) {
		/* In this driver, to simplify code, only SET_CUR request is managed */

		if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL) {
			((USBD_MIDI_ItfTypeDef*) pdev->pUserData[pdev->classId])->MuteCtl(
					haudio->control.data[0]);
			haudio->control.cmd = 0U;
			haudio->control.len = 0U;
		}
	}

	return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_EP0_TxReady
 *         handle EP0 TRx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_MIDI_EP0_TxReady(USBD_HandleTypeDef *pdev) {
	UNUSED(pdev);

	/* Only OUT control data are processed */
	return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */

static uint8_t USBD_MIDI_SOF(USBD_HandleTypeDef *pdev) {
//	UNUSED(pdev);

	USBD_MIDI_HandleTypeDef *haudio;
	uint8_t retval = USBD_OK;

	haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
	if (haudio != NULL) {
	} else {
		return;
	}

	if (tx_flag == 0U) {
		USB_OTG_GlobalTypeDef *USBx = USB_OTG_FS;
		uint32_t USBx_BASE = (uint32_t) USBx;
		uint32_t volatile fnsof_new = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF)
				>> 8;

		if ((fnsof & 0x1) == (fnsof_new & 0x1)) {
			if (haudio != NULL && haudio->data_ready_flag == 0) {
				haudio->frame_rate++;
				haudio->in_buffer_half = !haudio->in_buffer_half; // also serves as init to 1 or 0
				haudio->data_ready_flag = 1;
				uint16_t prev = (AUDIO_IN_PACKET / 2) * !haudio->in_buffer_half;

				/*
				 * if i choose to bypass throttling logic then i should call this before transmit
				 * USBD_LL_FlushEP  (pdev, AUDIO_IN_EP);
				 */

				USBD_LL_Transmit(pdev, AUDIO_IN_EP,
						(uint8_t*) (haudio->in_buffer + prev), AUDIO_IN_PACKET);
				/* Block transmission until it's finished. */
				tx_flag = 1U;
			}
		}
	}

	return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @param  offset: audio offset
 * @retval status
 */
void USBD_MIDI_Sync(USBD_HandleTypeDef *pdev, MIDI_OffsetTypeDef offset) {
}

/**
 * @brief  USBD_AUDIO_IsoINIncomplete
 *         handle data ISO IN Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MIDI_IsoINIncomplete(USBD_HandleTypeDef *pdev,
		uint8_t epnum) {
//	  UNUSED(pdev);
//	  UNUSED(epnum);

	USB_OTG_GlobalTypeDef *USBx = USB_OTG_FS;
	uint32_t USBx_BASE = (uint32_t) USBx;
	fnsof = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

	USBD_MIDI_HandleTypeDef *haudio;
	haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
	if (haudio != NULL) {
	} else {
		return;
	}

	if (epnum == (AUDIO_IN_EP & 0x7F)) {
		if (tx_flag == 1U) {
			tx_flag = 0U;
			haudio->frame_rate++;
			USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
		}
	}

	return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_IsoOutIncomplete
 *         handle data ISO OUT Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MIDI_IsoOutIncomplete(USBD_HandleTypeDef *pdev,
		uint8_t epnum) {
	UNUSED(pdev);
	UNUSED(epnum);

	return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MIDI_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	uint16_t PacketSize;
	USBD_MIDI_HandleTypeDef *haudio;

#ifdef USE_USBD_COMPOSITE
	  /* Get the Endpoints addresses allocated for this class instance */
	  AUDIOOutEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC);
	#endif /* USE_USBD_COMPOSITE */

	haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

	if (haudio == NULL) {
		return (uint8_t) USBD_FAIL;
	}

	if (epnum == AUDIOOutEpAdd) {
		/* Get received data packet length */
		PacketSize = (uint16_t) USBD_LL_GetRxDataSize(pdev, epnum);

		/* Packet received Callback */
		((USBD_MIDI_ItfTypeDef*) pdev->pUserData[pdev->classId])->PeriodicTC(
				&haudio->buffer[haudio->wr_ptr], PacketSize, AUDIO_OUT_TC);

		/* Increment the Buffer pointer or roll it back when all buffers are full */
		haudio->wr_ptr += PacketSize;

		if (haudio->wr_ptr == AUDIO_TOTAL_BUF_SIZE) {
			/* All buffers are full: roll back */
			haudio->wr_ptr = 0U;

			if (haudio->offset == MIDI_OFFSET_UNKNOWN) {
				((USBD_MIDI_ItfTypeDef*) pdev->pUserData[pdev->classId])->AudioCmd(
						&haudio->buffer[0],
						AUDIO_TOTAL_BUF_SIZE / 2U, MIDI_CMD_START);
				haudio->offset = MIDI_OFFSET_NONE;
			}
		}

		if (haudio->rd_enable == 0U) {
			if (haudio->wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2U)) {
				haudio->rd_enable = 1U;
			}
		}

		/* Prepare Out endpoint to receive next audio packet */
		(void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd,
				&haudio->buffer[haudio->wr_ptr],
				AUDIO_OUT_PACKET);
	}

	return (uint8_t) USBD_OK;
}

/**
 * @brief  AUDIO_Req_GetCurrent
 *         Handles the GET_CUR Audio control request.
 * @param  pdev: device instance
 * @param  req: setup class request
 * @retval status
 */
static void MIDI_REQ_GetCurrent(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req) {
	USBD_MIDI_HandleTypeDef *haudio;
	haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

	if (haudio == NULL) {
		return;
	}

	(void) USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);

	/* Send the current mute state */
	(void) USBD_CtlSendData(pdev, haudio->control.data,
			MIN(req->wLength, USB_MAX_EP0_SIZE));

}

/**
 * @brief  AUDIO_Req_SetCurrent
 *         Handles the SET_CUR Audio control request.
 * @param  pdev: device instance
 * @param  req: setup class request
 * @retval status
 */
static void MIDI_REQ_SetCurrent(USBD_HandleTypeDef *pdev,
		USBD_SetupReqTypedef *req) {
	USBD_MIDI_HandleTypeDef *haudio;
	haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

	if (haudio == NULL) {
		return;
	}

	if (req->wLength != 0U) {
		haudio->control.cmd = AUDIO_REQ_SET_CUR; /* Set the request value */
		haudio->control.len = (uint8_t) MIN(req->wLength, USB_MAX_EP0_SIZE); /* Set the request data length */
		haudio->control.unit = HIBYTE(req->wIndex); /* Set the request target unit */

		/* Prepare the reception of the buffer over EP0 */
		(void) USBD_CtlPrepareRx(pdev, haudio->control.data,
				haudio->control.len);
	}

}

#ifndef USE_USBD_COMPOSITE
/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_MIDI_GetDeviceQualifierDesc(uint16_t *length) {
	*length = (uint16_t) sizeof(USBD_MIDI_DeviceQualifierDesc);

	return USBD_MIDI_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
 * @brief  USBD_AUDIO_RegisterInterface
 * @param  pdev: device instance
 * @param  fops: Audio interface callback
 * @retval status
 */
uint8_t USBD_MIDI_RegisterInterface(USBD_HandleTypeDef *pdev,
		USBD_MIDI_ItfTypeDef *fops) {
	if (fops == NULL) {
		return (uint8_t) USBD_FAIL;
	}

	pdev->pUserData[pdev->classId] = fops;

	return (uint8_t) USBD_OK;
}

#ifdef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetEpPcktSze
  * @param  pdev: device instance (reserved for future use)
  * @param  If: Interface number (reserved for future use)
  * @param  Ep: Endpoint number (reserved for future use)
  * @retval status
  */
uint32_t USBD_MIDI_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep)
{
  uint32_t mps;

  UNUSED(pdev);
  UNUSED(If);
  UNUSED(Ep);

  mps = MIC_PACKET_SZE(USBD_AUDIO_FREQ);

  /* Return the wMaxPacketSize value in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
  return mps;
}
#endif /* USE_USBD_COMPOSITE */

/**
 * @brief  USBD_AUDIO_GetAudioHeaderDesc
 *         This function return the Audio descriptor
 * @param  pdev: device instance
 * @param  pConfDesc:  pointer to Bos descriptor
 * @retval pointer to the Audio AC Header descriptor
 */
static void* USBD_MIDI_GetAudioHeaderDesc(uint8_t *pConfDesc) {
	USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef*) (void*) pConfDesc;
	USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef*) (void*) pConfDesc;
	uint8_t *pAudioDesc = NULL;
	uint16_t ptr;

	if (desc->wTotalLength > desc->bLength) {
		ptr = desc->bLength;

		while (ptr < desc->wTotalLength) {
			pdesc = USBD_GetNextDesc((uint8_t*) pdesc, &ptr);
			if ((pdesc->bDescriptorType == AUDIO_INTERFACE_DESCRIPTOR_TYPE)
					&& (pdesc->bDescriptorSubType == AUDIO_CONTROL_HEADER)) {
				pAudioDesc = (uint8_t*) pdesc;
				break;
			}
		}
	}
	return pAudioDesc;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
