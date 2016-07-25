/**
  ******************************************************************************
  * @file    usbd_audio_out_if.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the Audio Out (play back) interface API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
#include "usbd_audio_core.h"
#include "usbd_audio_out_if.h"

#include <string.h>

uint16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
int inCurIndex = 0;


/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_audio_out_if 
  * @brief usbd out interface module
  * @{
  */ 

/** @defgroup usbd_audio_out_if_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_out_if_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_out_if_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_out_if_Private_FunctionPrototypes
  * @{
  */
static uint8_t  Init         (uint32_t  AudioFreq, uint32_t Volume, uint32_t options);
static uint8_t  DeInit       (uint32_t options);
static uint8_t  AudioCmd     (uint8_t* pbuf, uint32_t size, uint8_t cmd);
static uint8_t  VolumeCtl    (uint8_t vol);
static uint8_t  MuteCtl      (uint8_t cmd);
static uint8_t  PeriodicTC   (uint8_t cmd);
static uint8_t  GetState     (void);

/**
  * @}
  */ 

/** @defgroup usbd_audio_out_if_Private_Variables
  * @{
  */ 
AUDIO_FOPS_TypeDef  AUDIO_OUT_fops = 
{
  Init,
  DeInit,
  AudioCmd,
  VolumeCtl,
  MuteCtl,
  PeriodicTC,
  GetState
};

/*static*/ uint8_t AudioState = AUDIO_STATE_INACTIVE;

/**
  * @}
  */ 

/** @defgroup usbd_audio_out_if_Private_Functions
  * @{
  */ 

/**
  * @brief  Init
  *         Initialize and configures all required resources for audio play function.
  * @param  AudioFreq: Startup audio frequency.
  * @param  Volume: Startup volume to be set.
  * @param  options: specific options passed to low layer function.
  * @retval AUDIO_OK if all operations succeed, AUDIO_FAIL else.
  */
static uint8_t  Init         (uint32_t AudioFreq, 
                              uint32_t Volume, 
                              uint32_t options)
{
  static uint32_t Initialized = 0;
  
  /* Check if the low layer has already been initialized */
  if (Initialized == 0)
  {
	/* Setup DAC here */
	init_dac();

    /* Set the Initialization flag to prevent reinitializing the interface again */
    Initialized = 1;
  }
  
  /* Update the Audio state machine */
  AudioState = AUDIO_STATE_ACTIVE;
    
  return AUDIO_OK;
}

void init_dac(){
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitTypeDef DMA_InitStruct2;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Enable DAC clock */
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	/* Enable DMA1 clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	// Configuration
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T4_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	// IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Set DAC DHR12L register
	DAC_SetChannel1Data(DAC_Align_12b_L, 0);
	DAC_SetChannel2Data(DAC_Align_12b_L, 0);

	/* Enable timer clock */
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	// Clock data 4 times the rate data is coming in
	// 96kHz data over USB, DAC clocked at 384kHz
	uint16_t period = 84e3/(FREQ_KHZ*2);//84e3/(FREQ_KHZ*4)-2;

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
	TIM_TimeBaseStruct.TIM_Period = period;
	TIM_TimeBaseStruct.TIM_Prescaler = 0;
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

	/* Initialize timer */
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

	/* Enable TIM selection */
	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);

//	/* Set DMA options */
//	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&sampleBuffer[0];
//	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStruct.DMA_BufferSize = sizeof(sampleBuffer)/4;
//	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
//	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
//	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//
//	/* Set peripheral location = 12bit left aligned for channel 1 */
//	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&DAC->DHR12L1;
//	/* Disable DMA */
//	DMA_DeInit(DAC_SIGNAL_DMA_DAC1_STREAM);
//	/* Set channel used */
//	DMA_InitStruct.DMA_Channel = DAC_SIGNAL_DMA_DAC1_CHANNEL;
//	/* Initialize DMA */
//	DMA_Init(DAC_SIGNAL_DMA_DAC1_STREAM, &DMA_InitStruct);
//	/* Enable DMA Stream for DAC Channel 1 */
//	DMA_Cmd(DAC_SIGNAL_DMA_DAC1_STREAM, ENABLE);
	/* Enable DAC Channel 1 */
	DAC_Cmd(DAC_Channel_1, ENABLE);
	/* Enable DMA for DAC Channel 1 */
	DAC_DMACmd(DAC_Channel_1, ENABLE);

//	DMA_InitStruct2.DMA_Memory0BaseAddr = (uint32_t)(&sampleBuffer[1]);
//	DMA_InitStruct2.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStruct2.DMA_BufferSize = sizeof(sampleBuffer)/4;
//	DMA_InitStruct2.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStruct2.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStruct2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
//	DMA_InitStruct2.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
//	DMA_InitStruct2.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStruct2.DMA_Priority = DMA_Priority_High;
//	DMA_InitStruct2.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStruct2.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//	DMA_InitStruct2.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStruct2.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//
//	/* Set peripheral location = 12bit left aligned for channel 2 */
//	DMA_InitStruct2.DMA_PeripheralBaseAddr = (uint32_t)&DAC->DHR12L2;
//	/* Disable DMA */
//	DMA_DeInit(DAC_SIGNAL_DMA_DAC2_STREAM);
//	/* Set channel used */
//	DMA_InitStruct2.DMA_Channel = DAC_SIGNAL_DMA_DAC2_CHANNEL;
//	/* Initialize DMA */
//	DMA_Init(DAC_SIGNAL_DMA_DAC2_STREAM, &DMA_InitStruct2);
//	/* Enable DMA Stream for DAC Channel 2 */
//	DMA_Cmd(DAC_SIGNAL_DMA_DAC2_STREAM, ENABLE);
//	/* Enable DAC Channel 2 */
//	DAC_Cmd(DAC_Channel_2, ENABLE);
//	/* Enable DMA for DAC Channel 2 */
//	DAC_DMACmd(DAC_Channel_2, ENABLE);
//
//	TIM4->CR1 |= TIM_CR1_CEN;
//	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}


/**
  * @brief  DeInit
  *         Free all resources used by low layer and stops audio-play function.
  * @param  options: options passed to low layer function.
  * @retval AUDIO_OK if all operations succeed, AUDIO_FAIL else.
  */
static uint8_t  DeInit       (uint32_t options)
{
  /* Update the Audio state machine */
  AudioState = AUDIO_STATE_INACTIVE;
  
  return AUDIO_OK;
}

/**
  * @brief  AudioCmd 
  *         Play, Stop, Pause or Resume current file.
  * @param  pbuf: address from which file should be played.
  * @param  size: size of the current buffer/file.
  * @param  cmd: command to be executed, can be AUDIO_CMD_PLAY , AUDIO_CMD_PAUSE, 
  *              AUDIO_CMD_RESUME or AUDIO_CMD_STOP.
  * @retval AUDIO_OK if all operations succeed, AUDIO_FAIL else.
  */
static uint8_t  AudioCmd(uint8_t* pbuf, 
                         uint32_t size,
                         uint8_t cmd)
{
  static int startPlay = 0;

  /* Check the current state */
  if ((AudioState == AUDIO_STATE_INACTIVE) || (AudioState == AUDIO_STATE_ERROR))
  {
    AudioState = AUDIO_STATE_ERROR;
    return AUDIO_FAIL;
  }
  
  switch (cmd)
  {
    /* Process the PLAY command ----------------------------*/
  case AUDIO_CMD_PLAY:
    /* If current state is Active or Stopped */
    if ((AudioState == AUDIO_STATE_ACTIVE) || \
       (AudioState == AUDIO_STATE_STOPPED) || \
       (AudioState == AUDIO_STATE_PLAYING) || \
       (AudioState == AUDIO_STATE_PAUSED))
    {
      AudioState = AUDIO_STATE_PLAYING;
      /* inCurIndex indexes a 16 bit memory space. Check if still within the space */
      if (inCurIndex < (sizeof(sampleBuffer) / 2)){
    	  /* If within the space, copy data from USB buffer into sampleBuffer */
    	  memcpy(&sampleBuffer[inCurIndex], pbuf, size);
    	  inCurIndex += size / 2;
      }
      else{
    	  memcpy(&sampleBuffer[0], pbuf, size);
    	  inCurIndex = size / 2;
      }

      if (!startPlay){
    	  if (inCurIndex >= (sizeof(sampleBuffer) / 4)){
    			TIM4->CR1 |= TIM_CR1_CEN;
    			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    		  startPlay = 1;
    	  }
      }

      return AUDIO_OK;
    }

    /* If current state is Paused */
    else if (AudioState == AUDIO_STATE_PAUSED)
    {
      {
        AudioState = AUDIO_STATE_PLAYING;
        return AUDIO_OK;
      } 
    } 
    else /* Not allowed command */
    {
      return AUDIO_FAIL;
    }
    
    /* Process the STOP command ----------------------------*/
  case AUDIO_CMD_STOP:
    if (AudioState != AUDIO_STATE_PLAYING)
    {
      /* Unsupported command */
      return AUDIO_FAIL;
    }
    else
    	{
    		if (AudioState == AUDIO_STATE_PLAYING)
    		{
    			TIM4->CR1 &= ~TIM_CR1_CEN;
    			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    			memset(sampleBuffer, 2048, sizeof(sampleBuffer));
    			inCurIndex = 0;
    			startPlay = 0;
    		}
    		AudioState = AUDIO_STATE_STOPPED;
    		return AUDIO_OK;
    	}
  
    /* Process the PAUSE command ---------------------------*/
  case AUDIO_CMD_PAUSE:
    if (AudioState != AUDIO_STATE_PLAYING)
    {
      /* Unsupported command */
      return AUDIO_FAIL;
    }
    else
    	{
    		if (AudioState == AUDIO_STATE_PLAYING)
    		{
    			TIM4->CR1 &= ~TIM_CR1_CEN;
    			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    			memset(sampleBuffer, 2048, sizeof(sampleBuffer));
    			inCurIndex = 0;
    			startPlay = 0;
    		}
    		AudioState = AUDIO_STATE_PAUSED;
    		return AUDIO_OK;
    	}
    
    /* Unsupported command ---------------------------------*/
  default:
    return AUDIO_FAIL;
  }  
}

/**
  * @brief  VolumeCtl
  *         Set the volume level in %
  * @param  vol: volume level to be set in % (from 0% to 100%)
  * @retval AUDIO_OK if all operations succeed, AUDIO_FAIL else.
  */
static uint8_t  VolumeCtl    (uint8_t vol)
{
  
  return AUDIO_OK;
}

/**
  * @brief  MuteCtl
  *         Mute or Unmute the audio current output
  * @param  cmd: can be 0 to unmute, or 1 to mute.
  * @retval AUDIO_OK if all operations succeed, AUDIO_FAIL else.
  */
static uint8_t  MuteCtl      (uint8_t cmd)
{
  
  return AUDIO_OK;
}

/**
  * @brief  
  *         
  * @param  
  * @param  
  * @retval AUDIO_OK if all operations succeed, AUDIO_FAIL else.
  */
static uint8_t  PeriodicTC   (uint8_t cmd)
{
  return AUDIO_OK;
}


/**
  * @brief  GetState
  *         Return the current state of the audio machine
  * @param  None
  * @retval Current State.
  */
static uint8_t  GetState   (void)
{
  return AudioState;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
