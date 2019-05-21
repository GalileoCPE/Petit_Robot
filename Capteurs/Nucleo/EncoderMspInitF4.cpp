#include "mbed.h"
/*
 * HAL_TIM_Encoder_MspInit()
 * Overrides the __weak function stub in stm32f4xx_hal_tim.h
 *
 * Edit the below for your preferred pin wiring & pullup/down
 * I have encoder common at 3V3, using GPIO_PULLDOWN on inputs.
 * Encoder A&B outputs connected directly to GPIOs.
 *
 * www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00102166.pdf
 * www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00141306.pdf
 *
 * TIM3_CH1: AF2 @ PA_6, PB_4, PC_6
 * TIM3_CH2: AF2 @ PA_7, PB_5, PC_7
 *
 * TIM4_CH1: AF2 @ PB_6, PD_12
 * TIM4_CH2: AF2 @ PB_7, PD_13
 *
 */

#ifdef TARGET_STM32F4
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // Timer 3:
    if (htim->Instance == TIM3) { // PB4 PB5 = Nucleo D5 D4
        __TIM3_CLK_ENABLE();
        __GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        // Interruption:
        HAL_NVIC_SetPriority(TIM3_IRQn,0,0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
    // Timer 4:
    else if (htim->Instance == TIM4) { // PB6 PB7 = Nucleo D10 MORPHO_PB7
        __TIM4_CLK_ENABLE();
        __GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // Interruption:
        HAL_NVIC_SetPriority(TIM4_IRQn,0,0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}
#endif
