

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void OTG_HS_IRQHandler(void);
void OTG_HS_EP1_IN_IRQHandler(void);
void OTG_HS_EP1_OUT_IRQHandler(void);
void OTG_FS_IRQHandler(void);

#ifdef __cplusplus
}
#endif
