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

#define ITBUFEN (1 << 10)
#define ITEVTEN (1 << 9)

#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16

#define BUF_MAX_SIZE 255 

void usart_out(char *str) {
  for (size_t i = 0; str[i] != '\0';) {
    if (USART2->SR & USART_SR_TXE) {
      USART2->DR = (int)str[i];
      i++;
    }
  }
}
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
  USART2->CR1 |= USART_Enable;
}

void configure_dma() {
  DMA1_Stream6->CR =
      4U << 25 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
  DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

  DMA1->HIFCR = DMA_HIFCR_CTCIF6;
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

static char dma_to_send[BUF_MAX_SIZE];
static char buffer[BUF_MAX_SIZE];
static uint8_t dma_size = 0;
static uint8_t buf_size = 0;
char array_use = 0;

void append_to_comms(const char *str) {
  size_t i = 0;
  if (array_use) {
    while (dma_size < BUF_MAX_SIZE && str[i] != 0) {
      dma_to_send[dma_size++] = str[i++];
      // usart_out(buffer);
    }
    dma_to_send[dma_size++] = 0;

  } else {
    while (buf_size < BUF_MAX_SIZE && str[i] != 0) {
      buffer[buf_size++] = str[i++];
      // usart_out(buffer);
    }
    buffer[buf_size++] = 0;
  }
  // usart_out(buffer);
  //   usart_out(str);
}

void dma_send() {
  irq_level_t level = IRQprotectAll();
  uint8_t size = array_use ? dma_size : buf_size;
  if (size > 0 && (DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
      (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
    // usart_out("send\n");
    //     usart_out(buffer);
    if (array_use) {
      DMA1_Stream6->M0AR = (uint32_t)dma_to_send;
      DMA1_Stream6->NDTR = dma_size;
    } else {
      DMA1_Stream6->M0AR = (uint32_t)buffer;
      DMA1_Stream6->NDTR = buf_size;
    }
    array_use = ~array_use;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    dma_size = 0;
    buf_size = 0;
  }
  IRQunprotectAll(level);
}

void DMA1_Stream6_IRQHandler() {
  //  usart_out("dma inter hit\n");
  irq_level_t level = IRQprotectAll();
  uint32_t isr = DMA1->HISR;
  if (isr & DMA_HISR_TCIF6) {
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    dma_send();
  }
  IRQunprotectAll(level);
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

typedef struct i2c_transer_data {
  char reg;
  char slave_addr;
  char to_send;
} i2c_transfer_data;

i2c_transfer_data data;

uint8_t state = 0;

void I2C1_EV_IRQHandler() {
  //  irq_level_t level = IRQprotectAll();
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
    I2C1->CR2 |= ITBUFEN;
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
      char buf[11];
      int8_to_bits(buf, res, reg);
      append_to_comms(buf);
      dma_send();
      // usart_out(buf);
      state = 0;
      state |= (1 << 6);
      if (data.reg == ACCEL_OUT_X)
        data.reg = ACCEL_OUT_Y;
      else {
        data.reg = ACCEL_OUT_X;
      }
    }
  }

  // usart_out("end inter\n");
  //  IRQunprotectAll(level);
}

void configure_i2c() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  GPIOafConfigure(GPIOB, 8, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);
  GPIOafConfigure(GPIOB, 9, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);
  I2C1->CR1 = 0;
  I2C1->CCR = (PCLK1_MHZ * 1000000) / (I2C_SPEED_HZ << 1);
  I2C1->CR2 = PCLK1_MHZ;
  I2C1->TRISE = PCLK1_MHZ + 1;
  I2C1->CR1 |= I2C_CR1_PE;
  I2C1->CR2 |= ITBUFEN;
  I2C1->CR2 |= ITEVTEN;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
}

void configure_timer() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->CR1 = 0; // counting up
  TIM3->ARR = 16000;
  TIM3->PSC = 10; // update 100 times per second
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = ~(TIM_SR_UIF);
  TIM3->DIER = TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void) {
  irq_level_t l = IRQprotectAll();
  // usart_out("t\n");
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
  configure_i2c();
  configure_rcc();
  configure_usart();
  configure_timer();
  configure_dma();
  char test = 0x47;
  state |= (1 << 7);
  data.to_send = test;
  data.slave_addr = LIS35DE_ADDR;
  data.reg = ACCEL_CTRL_REG1;
  I2C1->CR1 |= I2C_CR1_START;
  Delay(1000000);
  TIM3->CR1 |= TIM_CR1_CEN;
  for (;;) {
  }
  return 0;
}
