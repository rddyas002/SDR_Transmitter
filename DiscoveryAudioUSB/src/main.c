
#include "main.h"

#include "usbd_usr.h"
#include "usbd_desc.h"

#include "stm32f4xx_conf.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
 #if defined   (__CC_ARM) /*!< ARM Compiler */
  __align(4)
 #elif defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
 #elif defined (__GNUC__) /*!< GNU Compiler */
 #pragma pack(4)
 #elif defined  (__TASKING__) /*!< TASKING Compiler */
  __align(4)
 #endif /* __CC_ARM */
#endif

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

#include "usbd_audio_core.h"
#include "usbd_audio_out_if.h"

int main(void){
	RCC_ClocksTypeDef RCC_Clocks;
	/* Initialize LEDS */
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
 
	/* Green Led On: start of application */
	STM_EVAL_LEDOn(LED4);
       
	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  
	RCC_HSEConfig(RCC_HSE_ON);
	while(!RCC_WaitForHSEStartUp());

	USBD_Init(0, &USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc,
				  &AUDIO_cb, &USR_cb);
	while(1);
}
