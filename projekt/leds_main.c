#include "buttons.h"
#include "usart.h"
#include <delay.h>
#include <gpio.h>
#include <i2c_configure.h>
#include <irq.h>
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

#define ITBUFEN (1 << 10)
#define ITEVTEN (1 << 9)

void usart_out(char *str) {
  for (size_t i = 0; str[i] != '\0';) {
    if (USART2->SR & USART_SR_TXE) {
      USART2->DR = (int)str[i];
      i++;
    }
  }
}

void int8_to_bits(char *buffer, int8_t val, char id) {
  uint8_t mask = 1 << 7;
  for (uint8_t i = 0; mask != 0; mask = mask >> 1, i++) {
    buffer[i] = (mask & val) == 0 ? '0' : '1';
  }

  buffer[8] = id;
  buffer[9] = '\n';
  buffer[10] = '\0';
}

typedef struct rec_data {
  char reg;
  char slave_addr;
  char to_send;
} rec_data;

rec_data data;

char state = 0;

void I2C1_EV_IRQHandler() {
  irq_level_t level = IRQprotectAll();
  // //usart_out("Begin inter\n");
  if (state & (1 << 7)) {
    if (!(state & (1 << 0)) && I2C1->SR1 & I2C_SR1_SB) {
      // //usart_out("sb hit\n");
      I2C1->DR = data.slave_addr << 1;
      state |= (1 << 0);
    }
    if (!(state & (1 << 1)) && I2C1->SR1 & I2C_SR1_ADDR) {
      // usart_out("addr hit\n");
      I2C1->SR1;
      I2C1->SR2;
      I2C1->DR = data.reg;
      state |= (1 << 1);
    }
    if (!(state & (1 << 2)) && I2C1->SR1 & I2C_SR1_TXE) {
      I2C1->DR = data.to_send;
      I2C1->CR2 &= ~(ITBUFEN);
      // usart_out("txe hit\n");
      state |= (1 << 2);
    }
    if ((I2C1->SR1 & I2C_SR1_BTF)) {
      I2C1->DR;
      // usart_out("btf hit\n");
      I2C1->CR1 |= I2C_CR1_STOP;
      state = 0;
      state |= (1 << 6);
      data.reg = ACCEL_OUT_X;
    }

  } else if (state & (1 << 6)) {
    // TIM3->CR1 &= ~TIM_CR1_CEN; // turn of timer so it won't intefere
    // TIM3->DIER &= ~TIM_DIER_UIE;
    I2C1->CR2 |= ITBUFEN;
    // TIM3->SR |= (TIM_SR_UIF);
    if (!(state & (1 << 0)) && I2C1->SR1 & I2C_SR1_SB) {
      // usart_out("sb_send hit\n");
      I2C1->DR = data.slave_addr << 1;
      state |= (1 << 0);
    }
    if (!(state & (1 << 1)) && I2C1->SR1 & I2C_SR1_ADDR) {
      // usart_out("addr_send hit\n");
      I2C1->SR1;
      I2C1->SR2;
      I2C1->DR = data.reg;
      state |= (1 << 1);
    }
    if (!(state & (1 << 2)) && (I2C1->SR1 & I2C_SR1_BTF)) {
      I2C1->DR;
      // usart_out("btf_send hit\n");
      I2C1->CR1 |= I2C_CR1_START;
      state |= (1 << 2);
    }

    if (!(state & (1 << 3)) && I2C1->SR1 & I2C_SR1_SB) {
      // usart_out("sb_send 2hit\n");
      I2C1->DR = data.slave_addr << 1 | 1;
      I2C1->CR1 &= ~I2C_CR1_ACK;
      state |= (1 << 3);
    }

    if (!(state & (1 << 4)) && I2C1->SR1 & I2C_SR1_ADDR) {
      // usart_out("addr_send 22hit\n");
      I2C1->SR2;
      I2C1->CR1 |= I2C_CR1_STOP;
      state |= (1 << 4);
    }

    if (I2C1->SR1 & I2C_SR1_RXNE) {
      // usart_out("rxne_send 22hit\n");
      int res = I2C1->DR;
      char reg = 'x';
      if (data.reg == ACCEL_OUT_Y)
        reg = 'y';
      char buf[20];
      int8_to_bits(buf, res, reg);
      usart_out(buf);
      state = 0;
      state |= (1 << 6);
      if (data.reg == ACCEL_OUT_X)
        data.reg = ACCEL_OUT_Y;
      else {
        //        I2C1->CR1 |= I2C_CR1_START;
        data.reg = ACCEL_OUT_X;
      }
      //      TIM3->CR1 |= TIM_CR1_CEN;
      //      TIM3->DIER = TIM_DIER_UIE;
    }
  }

  // usart_out("end inter\n");
  IRQunprotectAll(level);
}

void I2C1_ER_IRQHandler() { usart_out("ER hit\n"); }

uint8_t my_abs(int16_t x) {
  return x;
  return x = x & 0x80 ? ~((~x + 1) & 0xFF) : x;
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
  I2C1->CR2 |= ITBUFEN;
  I2C1->CR2 |= ITEVTEN;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  //  NVIC_EnableIRQ(I2C1_ER_IRQn); <-- error interupt, what for?
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
void configure_timer() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->CR1 = 0; // counting up
  TIM3->ARR = 16000;
  TIM3->PSC = 10; // update 10 times per second
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = ~(TIM_SR_UIF);
  TIM3->DIER = TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void) {
  irq_level_t l = IRQprotectAll();
  //usart_out("t\n");
  uint32_t it_status = TIM3->SR & TIM3->DIER;
  if (it_status & TIM_SR_UIF) {
    TIM3->SR = ~TIM_SR_UIF;
    I2C1->CR1 |= I2C_CR1_START;
  }

 // if (it_status & TIM_SR_CC1IF) {
 //   TIM3->SR = ~TIM_SR_CC1IF;
 // }

  IRQunprotectAll(l);
}

int main() {
  config_i2c();
  configure_rcc();
  configure_usart();
  configure_timer();
  // I2C1->CR2 &= ~ITBUFEN;
  // I2C1->CR2 &= ~ITEVTEN;
  char test = 0x47;
  // write_i2c(ACCEL_CTRL_REG1, &test, 1, LIS35DE_ADDR);
  // int8_t x = my_abs(read_i2c(ACCEL_OUT_X, LIS35DE_ADDR));
  // int8_to_bits(debug_buffer, x, 'x');
  // I2C1->CR2 |= ITBUFEN;
  // I2C1->CR2 |= ITEVTEN;
  state |= (1 << 7);
  data.to_send = test;
  data.slave_addr = LIS35DE_ADDR;
  data.reg = ACCEL_CTRL_REG1;
  I2C1->CR1 |= I2C_CR1_START;
  Delay(1000000);
  TIM3->CR1 |= TIM_CR1_CEN;
  for (;;) {
    // data.sending = 0;
    //    state |= (1 << 6);
    // I2C1->CR1 |= I2C_CR1_START;

    // Delay(1000000);
    //  data.reg = ACCEL_OUT_X;
    //  data.slave_addr = LIS35DE_ADDR;
    //  I2C1->CR1 |= I2C_CR1_START;
    //  Delay(100000000);

    // return 0;
    //  int8_t x = my_abs(read_i2c(ACCEL_OUT_X, LIS35DE_ADDR));

    // int8_t y = my_abs(read_i2c(ACCEL_OUT_Y, LIS35DE_ADDR));
    // int8_t z = my_abs(read_i2c(ACCEL_OUT_Z, LIS35DE_ADDR));
    // char outout[] = {'x', x, 'y', y, 'z', z, '\n', '\0'};
    // char newlin[] = {'\n', 0};
    // Delay(10000000);
    ////    ////usart_out(outout);
    //    int8_to_bits(debug_buffer, x, 'x');
    //    ////usart_out(debug_buffer);
    //// int8_to_bits(debug_buffer, z, 'z');
    //// ////usart_out(debug_buffer);
    //// int8_to_bits(debug_buffer, y, 'y');
    //// ////usart_out(debu
  }
  return 0;
}
