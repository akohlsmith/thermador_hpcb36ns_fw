#include "main.h"
#include "stm32g4xx_it.h"

void NMI_Handler(void)
{
	while (1) ;
}

void HardFault_Handler(void)
{
	while (1) ;
}

void MemManage_Handler(void)
{
	while (1) ;
}

void BusFault_Handler(void)
{
	while (1) ;
}

void UsageFault_Handler(void)
{
	while (1) ;
}

void SVC_Handler(void)
{
	while (1) ;
}

void DebugMon_Handler(void)
{
	while (1) ;
}

void PendSV_Handler(void)
{
	while (1) ;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}

void EXTI9_5_IRQHandler(void)
{
	HAL_ResumeTick();
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

void EXTI15_10_IRQHandler(void)
{
	HAL_ResumeTick();
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
}
