/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_audio.c file.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_MIDI_H
#define __USB_MIDI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */


/** @defgroup USBD_AUDIO_Exported_Defines
  * @{
  */
#ifndef USBD_AUDIO_FREQ
/* AUDIO Class Config */
#define USBD_AUDIO_FREQ                               48000U
#endif /* USBD_AUDIO_FREQ */

#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                       1U
#endif /* USBD_AUDIO_FREQ */

#ifndef AUDIO_HS_BINTERVAL
#define AUDIO_HS_BINTERVAL                            0x01U
#endif /* AUDIO_HS_BINTERVAL */

#ifndef AUDIO_FS_BINTERVAL
#define AUDIO_FS_BINTERVAL                            0x01U
#endif /* AUDIO_FS_BINTERVAL */

#ifndef AUDIO_OUT_EP
#define AUDIO_OUT_EP                                  0x01U
#endif /* AUDIO_OUT_EP */

#ifndef USB_MIDI_CONFIG_DESC_SIZ
#define USB_MIDI_CONFIG_DESC_SIZ                     0x6DU
#endif


#ifndef AUDIO_INTERFACE_DESC_SIZE
#define AUDIO_INTERFACE_DESC_SIZE                     0x09U
#endif

#ifndef USB_AUDIO_DESC_SIZ
#define USB_AUDIO_DESC_SIZ                            0x09U
#endif

#ifndef AUDIO_STANDARD_ENDPOINT_DESC_SIZE
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09U
#endif

#ifndef AUDIO_STREAMING_ENDPOINT_DESC_SIZE
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07U
#endif

#ifndef AUDIO_DESCRIPTOR_TYPE
#define AUDIO_DESCRIPTOR_TYPE                         0x21U
#endif

#ifndef USB_DEVICE_CLASS_AUDIO
#define USB_DEVICE_CLASS_AUDIO                        0x01U
#endif

#ifndef AUDIO_SUBCLASS_AUDIOCONTROL
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01U
#endif

#ifndef AUDIO_SUBCLASS_AUDIOSTREAMING
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02U
#endif

#ifndef AUDIO_PROTOCOL_UNDEFINED
#define AUDIO_PROTOCOL_UNDEFINED                      0x00U
#endif

#ifndef AUDIO_STREAMING_GENERAL
#define AUDIO_STREAMING_GENERAL                       0x01U
#endif

#ifndef AUDIO_STREAMING_FORMAT_TYPE
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02U
#endif

/* Audio Descriptor Types */
#ifndef AUDIO_INTERFACE_DESCRIPTOR_TYPE
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24U
#endif

#ifndef AUDIO_ENDPOINT_DESCRIPTOR_TYPE
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25U
#endif

/* Audio Control Interface Descriptor Subtypes */
#ifndef AUDIO_CONTROL_HEADER
#define AUDIO_CONTROL_HEADER                          0x01U
#endif

#ifndef AUDIO_CONTROL_INPUT_TERMINAL
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02U
#endif

#ifndef AUDIO_CONTROL_OUTPUT_TERMINAL
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03U
#endif

#ifndef AUDIO_CONTROL_FEATURE_UNIT
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06U
#endif

#ifndef AUDIO_INPUT_TERMINAL_DESC_SIZE
#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0CU
#endif

#ifndef AUDIO_OUTPUT_TERMINAL_DESC_SIZE
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09U
#endif

#ifndef AUDIO_STREAMING_INTERFACE_DESC_SIZE
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07U
#endif

#ifndef AUDIO_CONTROL_MUTE
#define AUDIO_CONTROL_MUTE                            0x0001U
#endif

#ifndef AUDIO_FORMAT_TYPE_I
#define AUDIO_FORMAT_TYPE_I                           0x01U
#endif

#ifndef AUDIO_FORMAT_TYPE_III
#define AUDIO_FORMAT_TYPE_III                         0x03U
#endif

#ifndef AUDIO_ENDPOINT_GENERAL
#define AUDIO_ENDPOINT_GENERAL                        0x01U
#endif

#ifndef AUDIO_REQ_GET_CUR
#define AUDIO_REQ_GET_CUR                             0x81U
#endif

#ifndef AUDIO_REQ_SET_CUR
#define AUDIO_REQ_SET_CUR                             0x01U
#endif

#ifndef AUDIO_OUT_STREAMING_CTRL
#define AUDIO_OUT_STREAMING_CTRL                      0x02U
#endif

#ifndef AUDIO_OUT_TC
#define AUDIO_OUT_TC                                  0x01U
#endif

#ifndef AUDIO_IN_TC
#define AUDIO_IN_TC                                   0x02U
#endif


#ifndef AUDIO_OUT_PACKET
#define AUDIO_OUT_PACKET                              (uint16_t)(((USBD_AUDIO_FREQ * 2U * 2U) / 1000U))
#endif

#ifndef AUDIO_DEFAULT_VOLUME
#define AUDIO_DEFAULT_VOLUME                          70U
#endif

/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
  that it is an even number and higher than 3 */
#ifndef AUDIO_OUT_PACKET_NUM
#define AUDIO_OUT_PACKET_NUM                          80U
#endif
/* Total size of the audio transfer buffer */
#ifndef AUDIO_TOTAL_BUF_SIZE
#define AUDIO_TOTAL_BUF_SIZE                          ((uint16_t)(AUDIO_OUT_PACKET * AUDIO_OUT_PACKET_NUM))
#endif

/* Audio Commands enumeration */
typedef enum
{
  MIDI_CMD_START = 1,
  MIDI_CMD_PLAY,
  MIDI_CMD_STOP,
} MIDI_CMD_TypeDef;


typedef enum
{
  MIDI_OFFSET_NONE = 0,
  MIDI_OFFSET_HALF,
  MIDI_OFFSET_FULL,
  MIDI_OFFSET_UNKNOWN,
} MIDI_OffsetTypeDef;
/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct
{
  uint8_t cmd;
  uint8_t data[USB_MAX_EP0_SIZE];
  uint8_t len;
  uint8_t unit;
} USBD_MIDI_ControlTypeDef;


typedef struct
{
  uint32_t alt_setting;
  uint8_t buffer[AUDIO_TOTAL_BUF_SIZE];
  MIDI_OffsetTypeDef offset;
  uint8_t rd_enable;
  uint16_t rd_ptr;
  uint16_t wr_ptr;
  USBD_MIDI_ControlTypeDef control;
} USBD_MIDI_HandleTypeDef;


typedef struct
{
  int8_t (*Init)(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
  int8_t (*DeInit)(uint32_t options);
  int8_t (*AudioCmd)(uint8_t *pbuf, uint32_t size, uint8_t cmd);
  int8_t (*VolumeCtl)(uint8_t vol);
  int8_t (*MuteCtl)(uint8_t cmd);
  int8_t (*PeriodicTC)(uint8_t *pbuf, uint32_t size, uint8_t cmd);
  int8_t (*GetState)(void);
} USBD_MIDI_ItfTypeDef;

/*
 * Audio Class specification release 1.0
 */

/* Table 4-2: Class-Specific AC Interface Header Descriptor */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bDescriptorSubtype;
//  uint16_t          bcdADC;
//  uint16_t          wTotalLength;
//  uint8_t           bInCollection;
//  uint8_t           baInterfaceNr;
//} __PACKED USBD_SpeakerIfDescTypeDef;
//
///* Table 4-3: Input Terminal Descriptor */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bDescriptorSubtype;
//  uint8_t           bTerminalID;
//  uint16_t          wTerminalType;
//  uint8_t           bAssocTerminal;
//  uint8_t           bNrChannels;
//  uint16_t          wChannelConfig;
//  uint8_t           iChannelNames;
//  uint8_t           iTerminal;
//} __PACKED USBD_SpeakerInDescTypeDef;
//
///* USB Speaker Audio Feature Unit Descriptor */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bDescriptorSubtype;
//  uint8_t           bUnitID;
//  uint8_t           bSourceID;
//  uint8_t           bControlSize;
//  uint16_t          bmaControls;
//  uint8_t           iTerminal;
//} __PACKED USBD_SpeakerFeatureDescTypeDef;
//
///* Table 4-4: Output Terminal Descriptor */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bDescriptorSubtype;
//  uint8_t           bTerminalID;
//  uint16_t          wTerminalType;
//  uint8_t           bAssocTerminal;
//  uint8_t           bSourceID;
//  uint8_t           iTerminal;
//} __PACKED USBD_SpeakerOutDescTypeDef;
//
///* Table 4-19: Class-Specific AS Interface Descriptor */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bDescriptorSubtype;
//  uint8_t           bTerminalLink;
//  uint8_t           bDelay;
//  uint16_t          wFormatTag;
//} __PACKED USBD_SpeakerStreamIfDescTypeDef;
//
///* USB Speaker Audio Type III Format Interface Descriptor */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bDescriptorSubtype;
//  uint8_t           bFormatType;
//  uint8_t           bNrChannels;
//  uint8_t           bSubFrameSize;
//  uint8_t           bBitResolution;
//  uint8_t           bSamFreqType;
//  uint8_t           tSamFreq2;
//  uint8_t           tSamFreq1;
//  uint8_t           tSamFreq0;
//} USBD_SpeakerIIIFormatIfDescTypeDef;
//
///* Table 4-17: Standard AC Interrupt Endpoint Descriptor */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bEndpointAddress;
//  uint8_t           bmAttributes;
//  uint16_t          wMaxPacketSize;
//  uint8_t           bInterval;
//  uint8_t           bRefresh;
//  uint8_t           bSynchAddress;
//} __PACKED USBD_SpeakerEndDescTypeDef;
//
///* Table 4-21: Class-Specific AS Isochronous Audio Data Endpoint Descriptor        */
//typedef struct
//{
//  uint8_t           bLength;
//  uint8_t           bDescriptorType;
//  uint8_t           bDescriptor;
//  uint8_t           bmAttributes;
//  uint8_t           bLockDelayUnits;
//  uint16_t          wLockDelay;
//} __PACKED USBD_SpeakerEndStDescTypeDef;

/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_MIDI;
#define USBD_MIDI_CLASS &USBD_MIDI
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_MIDI_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_MIDI_ItfTypeDef *fops);

void USBD_MIDI_Sync(USBD_HandleTypeDef *pdev, MIDI_OffsetTypeDef offset);

#ifdef USE_USBD_COMPOSITE
uint32_t USBD_MIDI_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep);
#endif /* USE_USBD_COMPOSITE */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_MIDI_H */
/**
  * @}
  */

/**
  * @}
  */
