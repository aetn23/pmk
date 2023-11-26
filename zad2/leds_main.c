#include "buttons.h"
#include "usart.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <stm32f411xe.h>

void configure_rcc() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  __NOP();
}

void configure_usart() {
  GPIOafConfigure(GPIOA, 2, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_USART2);
  GPIOafConfigure(GPIOA, 3, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_UP,
                  GPIO_AF_USART2);
  USART2->CR1 = USART_CR1_RE | USART_CR1_TE;
  USART2->CR2 = 0;
  USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;
  USART2->CR3 = USART_CR3_DMAT;
}

void configure_dma() {
  DMA1_Stream6->CR =
      4U << 25 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
  DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

  DMA1->HIFCR = DMA_HIFCR_CTCIF6;
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void configure_buttons() {
  GPIOinConfigure(GPIOC, USER_BTN_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                  EXTI_Trigger_Falling);
  GPIOinConfigure(JOYSTICK_GPIO, UP_BTN_PIN, GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                  EXTI_Trigger_Falling);
  GPIOinConfigure(JOYSTICK_GPIO, DOWN_BTN_PIN, GPIO_PuPd_UP,
                  EXTI_Mode_Interrupt, EXTI_Trigger_Falling);
  GPIOinConfigure(JOYSTICK_GPIO, RIGHT_BTN_PIN, GPIO_PuPd_UP,
                  EXTI_Mode_Interrupt, EXTI_Trigger_Falling);
  GPIOinConfigure(JOYSTICK_GPIO, LEFT_BTN_PIN, GPIO_PuPd_UP,
                  EXTI_Mode_Interrupt, EXTI_Trigger_Falling);
  GPIOinConfigure(JOYSTICK_GPIO, ACTION_BTN_PIN, GPIO_PuPd_UP,
                  EXTI_Mode_Interrupt, EXTI_Trigger_Falling);
  GPIOinConfigure(AT_BTN_GPIO, AT_BTN_PIN, GPIO_PuPd_DOWN, EXTI_Mode_Interrupt,
                  EXTI_Trigger_Falling);
}

void enable_falling_raising(uint8_t pin) {
  EXTI->FTSR |= (1 << pin);
  EXTI->RTSR |= (1 << pin);
}

void configure_exti() {
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB;
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
  enable_falling_raising(USER_BTN_PIN);
  enable_falling_raising(UP_BTN_PIN);
  enable_falling_raising(DOWN_BTN_PIN);
  enable_falling_raising(RIGHT_BTN_PIN);
  enable_falling_raising(LEFT_BTN_PIN);
  enable_falling_raising(ACTION_BTN_PIN);
  enable_falling_raising(AT_BTN_PIN);
}

void configure_nvic() {
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_EnableIRQ(EXTI4_IRQn);
}

int main() {
  configure_rcc();
  configure_usart();
  configure_dma();
  configure_buttons();
  configure_exti();
  configure_nvic();
  USART2->CR1 |= USART_CR1_UE;

  for (;;) {
  }
}
