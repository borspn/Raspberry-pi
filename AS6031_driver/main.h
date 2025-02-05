/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "HAL/raspPi3A_hal_gpio.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define SCK_Pin GPIO_PIN_
#define MISO_Pin GPIO_PIN_21
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define INTN_Pin GPIO_PIN_9
#define INTN_GPIO_Port GPIOA
#define INTN_EXTI_IRQn EXTI9_5_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SSN_Pin GPIO_PIN_6
#define SSN_GPIO_Port GPIOB

#endif /* __MAIN_H */