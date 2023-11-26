#include "buttons.h"
#include "usart.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <stm32f411xe.h>

#define OUTPUT_SIZE 15

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define GREEN2_LED_GPIO GPIOA
#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define GREEN2_LED_PIN 5

#define GREEN_POS 0
#define RED_POS 1
#define BLUE_POS 2
#define GREEN2_POS 3

#define RedLEDon() RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
#define RedLEDoff() RED_LED_GPIO->BSRR = 1 << RED_LED_PIN

#define BlueLEDon() BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16)
#define BlueLEDoff() BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN

#define GreenLEDon() GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16)
#define GreenLEDoff() GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN

#define Green2LEDon() GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
#define Green2LEDoff() GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)

void configure_leds() {
  RedLEDoff();
  GreenLEDoff();
  BlueLEDoff();
  Green2LEDoff();

  GPIOoutConfigure(RED_LED_GPIO, RED_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);

  GPIOoutConfigure(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);

  GPIOoutConfigure(BLUE_LED_GPIO, BLUE_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);

  GPIOoutConfigure(GREEN2_LED_GPIO, GREEN2_LED_PIN, GPIO_OType_PP,
                   GPIO_Low_Speed, GPIO_PuPd_NOPULL);
}

void configure_rcc() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  __NOP();
}

void configure_usart() {
  GPIOafConfigure(GPIOA, 2, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_USART2);

  GPIOafConfigure(GPIOA, 3, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_UP,
                  GPIO_AF_USART2);

  USART2->CR1 = USART_Mode_Rx_Tx | USART_WordLength_8b | USART_Parity_No;
  USART2->CR2 = USART_StopBits_1;
  USART2->CR3 = USART_FlowControl_None;
  USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;
  USART2->CR1 |= USART_Enable;
}

char input[3];
int input_c = 0;
char lights_state = 0;

void execute_command() {
  if (input[0] == 'L') {
    if (input[1] == 'R') {
      if (input[2] == '1') {
        lights_state |= (1 << RED_POS);
        RedLEDon();
      } else if (input[2] == '0') {
        lights_state &= ~(1 << RED_POS);
        RedLEDoff();
      } else if (input[2] == 'T') {
        if (lights_state & (1 << RED_POS)) {
          RedLEDoff();
          lights_state &= ~(1 << RED_POS);
        } else {
          RedLEDon();
          lights_state |= (1 << RED_POS);
        }
      }

    } else if (input[1] == 'G') {
      if (input[2] == '1') {
        lights_state |= (1 << GREEN_POS);
        GreenLEDon();
      } else if (input[2] == '0') {
        lights_state &= ~(1 << GREEN_POS);
        GreenLEDoff();
      } else if (input[2] == 'T') {
        if (lights_state & (1 << GREEN_POS)) {
          GreenLEDoff();
          lights_state &= ~(1 << GREEN_POS);
        } else {
          GreenLEDon();
          lights_state |= (1 << GREEN_POS);
        }
      }

    } else if (input[1] == 'B') {
      if (input[2] == '1') {
        lights_state |= (1 << BLUE_POS);
        BlueLEDon();
      } else if (input[2] == '0') {
        lights_state &= ~(1 << BLUE_POS);
        BlueLEDoff();
      } else if (input[2] == 'T') {
        if (lights_state & (1 << BLUE_POS)) {
          BlueLEDoff();
          lights_state &= ~(1 << BLUE_POS);
        } else {
          BlueLEDon();
          lights_state |= (1 << BLUE_POS);
        }
      }

    } else if (input[1] == 'g') {
      if (input[2] == '1') {
        lights_state |= (1 << GREEN2_POS);
        Green2LEDon();
      } else if (input[2] == '0') {
        lights_state &= ~(1 << GREEN2_POS);
        Green2LEDoff();
      } else if (input[2] == 'T') {
        if (lights_state & (1 << GREEN2_POS)) {
          Green2LEDoff();
          lights_state &= ~(1 << GREEN2_POS);
        } else {
          Green2LEDon();
          lights_state |= (1 << GREEN2_POS);
        }
      }
    }
  }
}

int main() {
  configure_rcc();
  configure_leds();
  configure_usart();

  for (;;) {
    handle_buttons();
    if (USART2->SR & USART_SR_RXNE) {
      char c = USART2->DR;
      input[input_c] = c;
      input_c++;
      if (input_c == 3) {
        execute_command();
        input_c = 0;
      }
    }
  }
}
