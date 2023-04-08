```
Use FSP v1.2.0 to configure the peripherals and update the generated bsp

https://github.com/renesas/rzt-fsp/releases/tag/v1.2.0

Standalone FSP Smart Configurator

setup_rztfsp_v1_2_0_rzsc_v2023-01.exe

see Config.cmake to suit your own toolchain directory

You make have to manually change the GeneratedCfg.cmake fsp.ld to fsp_xspi0_boot.ld to compile correctly

```

- [x] initial config FSP Smart Configurator
- [ ] import ../MESC_Common
- [ ] find stm code in ../MESC_Common and make MCU agnostic 


```
MESCBLDC.c

    TIM_HandleTypeDef
    TIM_OC_InitTypeDef
    TIMx_CHANNEL_GPIO_PORT
    TIMx_CHANNEL_GPIO_PIN
    TIMx_CHANNEL_AF
    __HAL_RCC_TIMx_CLK_ENABLE()
    HAL_TIM_PWM_Init()
    HAL_TIM_PWM_Start()
    HAL_TIM_PWM_Stop()
    HAL_TIM_PWM_ConfigChannel()

MESC_Comms.c

    UART_HandleTypeDef
    __HAL_RCC_GPIOx_CLK_ENABLE()
    __HAL_RCC_USARTx_CLK_ENABLE()
    HAL_GPIO_Init()
    HAL_UART_Init()
    HAL_UART_Receive_IT()
    HAL_UART_Transmit_IT()
    HAL_UART_IRQHandler()

MESCmotor.c

    GPIO_InitTypeDef
    TIM_HandleTypeDef
    TIM_OC_InitTypeDef
    TIMx_GPIO_PORT
    TIMx_GPIO_PIN
    TIMx_CHANNEL_GPIO_PORT
    TIMx_CHANNEL_GPIO_PIN
    TIMx_CHANNEL_AF
    __HAL_RCC_TIMx_CLK_ENABLE()
    HAL_TIM_PWM_Init()
    HAL_TIM_PWM_Start()
    HAL_TIM_PWM_Stop()
    HAL_TIM_PWM_ConfigChannel()
    HAL_GPIO_Init()

MESCmotor_state.c

    GPIO_InitTypeDef
    __HAL_RCC_GPIOx_CLK_ENABLE()
    HAL_GPIO_WritePin()
    HAL_GPIO_ReadPin()

MESCposition.c

    ADC_HandleTypeDef
    ADCx_CHANNEL_GPIO_PORT
    ADCx_CHANNEL_GPIO_PIN
    ADCx_CHANNEL
    __HAL_RCC_ADCx_CLK_ENABLE()
    HAL_ADC_Init()
    HAL_ADC_Start()
    HAL_ADC_PollForConversion()
    HAL_ADC_Stop()

MESCspeed.c

    GPIO_InitTypeDef
    __HAL_RCC_GPIOx_CLK_ENABLE()
    __HAL_RCC_TIMx_CLK_ENABLE()
    TIM_HandleTypeDef
    HAL_TIM_Base_Init()
    HAL_TIM_Base_Start()
    HAL_TIM_Base_Stop()
    HAL_GPIO_Init()
    HAL_GPIO_WritePin()
    HAL_GPIO_ReadPin()

MESCtemp.c

    ADC_HandleTypeDef
    ADCx_CHANNEL_GPIO_PORT
    ADCx_CHANNEL_GPIO_PIN
    ADCx_CHANNEL
    __HAL_RCC_ADCx_CLK_ENABLE()
    HAL_ADC_Init()
    HAL_ADC_Start()
    HAL_ADC_PollForConversion()
    HAL_ADC_Stop()

MESCuart.c

    UART_HandleTypeDef
    __HAL_RCC_GPIOx_CLK_ENABLE()
    __HAL_RCC_USARTx_CLK_ENABLE()
    HAL_UART_Init()
    HAL_UART_Receive_IT()
    HAL_UART_Transmit_IT()
    HAL_UART_IRQHandler()
```
