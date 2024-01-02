#include "buttons.h"
#include "usart.h"
#include <delay.h>
#include <gpio.h>
#include <i2c_configure.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32.h>
#include <stm32f411xe.h>
#include <string.h>

#define LIS35DE_ADDR 0x1C
#define ACCEL_CTRL_REG1 0x20
#define ACCEL_OUT_X 0x29
#define ACCEL_OUT_Y 0x2B
#define ACCEL_OUT_Z 0x2D

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
#define TIMEOUT 100000000

void int8_to_bits(char buffer[8], int8_t val) {
  uint8_t mask = 1 << 7;
  for (uint8_t i = 0; mask != 0; mask >> 1, i++) {
    buffer[i] = val & mask;
  }
}

uint8_t my_abs(int16_t x) { return x = x & 0x80 ? ~((~x + 1) & 0xFF) : x; }

void turn_off_leds() {
  BlueLEDoff();
  RedLEDoff();
  GreenLEDoff();
}

void usart_out(char *str) {
  for (size_t i = 0; str[i] != '\0';) {
    if (USART2->SR & USART_SR_TXE) {
      USART2->DR = (int)str[i];
      i++;
    }
  }
}

void config_i2c() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  GPIOafConfigure(GPIOB, 8, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);
  GPIOafConfigure(GPIOB, 9, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);
  I2C1->CR1 = 0;
#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16
  I2C1->CCR = (PCLK1_MHZ * 1000000) / (I2C_SPEED_HZ << 1);
  I2C1->CR2 = PCLK1_MHZ;
  I2C1->TRISE = PCLK1_MHZ + 1;
  I2C1->CR1 |= I2C_CR1_PE;
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

void write_i2c(char reg, char *val, size_t to_send, char slave_addr) {
  int i = 0;
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB)) {
    i++;
    if (i > TIMEOUT)
      return;
  }
  i = 0;
  I2C1->DR = slave_addr << 1;
  while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
    i++;
    if (i > TIMEOUT)
      return;
  }
  i = 0;
  I2C1->SR2;
  I2C1->DR = reg;
  while (!(I2C1->SR1 & I2C_SR1_TXE)) {
    i++;
    if (i > TIMEOUT)
      return;
  }
  i = 0;
  for (size_t i = 0; i < to_send - 1; i++) {
    I2C1->DR = val[i];
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
      i++;
      if (i > TIMEOUT)
        return;
    }
  }
  i = 0;

  I2C1->DR = val[to_send - 1];
  while (!(I2C1->SR1 & I2C_SR1_BTF)) {
    i++;
    if (i > TIMEOUT)
      return;
  }
  i = 0;

  I2C1->CR1 |= I2C_CR1_STOP;
}

uint32_t read_i2c(char reg, char slave_addr) {
  int i = 0;
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB)) {
    i++;
    if (i > TIMEOUT) {
      usart_out("Im returning 1\n");
      return 0;
    }
  }
  i = 0;
  I2C1->DR = slave_addr << 1;
  while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
    i++;
    if (i > TIMEOUT) {
      usart_out("Im returning 2\n");
      return 0;
    }
  }
  i = 0;
  I2C1->SR2;
  I2C1->DR = reg;
  while (!(I2C1->SR1 & I2C_SR1_BTF)) {
    i++;
    if (i > TIMEOUT) {
      usart_out("Im returning 3\n");
      return 0;
    }
  }
  i = 0;
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB)) {
    i++;
    if (i > TIMEOUT) {
      usart_out("Im returning 4\n");
      return 0;
    }
  }
  i = 0;
  I2C1->DR = slave_addr << 1 | 1;
  I2C1->CR1 &= ~I2C_CR1_ACK;
  while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
    i++;
    if (i > TIMEOUT) {
      usart_out("Im returning 5\n");
      return 0;
    }
  }
  i = 0;
  I2C1->SR2;
  I2C1->CR1 |= I2C_CR1_STOP;
  while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
    i++;
    if (i > TIMEOUT) {
      usart_out("Im returning 10\n");
      return 0;
    }
  }
  return I2C1->DR;
}

int main() {
  config_i2c();
  configure_rcc();
  configure_leds();
  configure_usart();
  char test = 0x47;
  write_i2c(ACCEL_CTRL_REG1, &test, 1, LIS35DE_ADDR);
  char inital_config = read_i2c(ACCEL_CTRL_REG1, LIS35DE_ADDR);
  char output[] = {inital_config, 0};
  usart_out(output);
  char debug_buffer[8];
  for (;;) {
    int8_t x = my_abs(read_i2c(ACCEL_OUT_X, LIS35DE_ADDR));
    int8_t y = my_abs(read_i2c(ACCEL_OUT_Y, LIS35DE_ADDR));
    int8_t z = my_abs(read_i2c(ACCEL_OUT_Z, LIS35DE_ADDR));
    char outout[] = {'x', x, 'y', y, 'z', z, '\n', '\0'};
    Delay(1000000);
    usart_out(outout);
    int8_to_bits(debug_buffer, x);
    usart_out(debug_buffer);
    int8_to_bits(debug_buffer, z);
    usart_out(debug_buffer);
    int8_to_bits(debug_buffer, y);
    usart_out(debug_buffer);
    if (y > x) {
      turn_off_leds();
      GreenLEDon();
    }
    if (z > x) {
      turn_off_leds();
      BlueLEDon();
    } else {
      turn_off_leds();
      RedLEDon();
    }
  }

  return 0;
}
