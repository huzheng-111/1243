#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* LED Definitions */
#define LED1_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1

/* Button Definitions */
#define KEY1_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_0
#define KEY2_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_1
#define KEY3_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_2

/* System State Machine */
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_NORMAL = 1,
} robot_state_e;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
