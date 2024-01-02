#include "buttons.h"
#include <gpio.h>
#include <irq.h>
#include <stm32f411xe.h>
#include <string.h>

const char user_press_com[] = "USER PRESSED\r\n";
const char user_rel_com[] = "USER REALEASD\r\n";

static const char up_press_com[] = "UP PRESSED\r\n";
static const char up_rel_com[] = "UP RELEASED\r\n";

static const char down_press_com[] = "DOWN PRESSED\r\n";
static const char down_rel_com[] = "DOWN RELEASED\r\n";

static const char left_press_com[] = "LEFT PRESSED\r\n";
static const char left_rel_com[] = "LEFT REL\r\n";

static const char right_press_com[] = "RIGHT PRESSED\r\n";
static const char right_rel_com[] = "RIGHT REL\r\n";

static const char action_press_com[] = "ACTION PRESSED\r\n";
static const char action_rel_com[] = "ACTION REL\r\n";

static const char at_press_com[] = "AT PRESSED\r\n";
static const char at_rel_com[] = "AT REL\r\n";

static char buttons_states = 0 | (1 << UP_BTN_POS) | (1 << DOWN_BTN_POS) |
                             (1 << RIGHT_BTN_POS) | (1 << LEFT_BTN_POS) |
                             (1 << USER_BTN_POS) | (1 << ACTION_BTN_POS);

static char dma_to_send[OUTPUT_BUFFER_SIZE];
static char dma_size = 0;

static char buttons_buffer[OUTPUT_BUFFER_SIZE];
static int button_c = 0;

static char *comms_buf[OUTPUT_BUFFER_SIZE / sizeof(char *)];
static int comms_c = 0;

void append_to_comms(const char *str) {
  if (comms_c == OUTPUT_BUFFER_SIZE) {
    return;
  }

  comms_buf[comms_c++] = str;
}

void cpy_to_dma() {
  for (int i = 0; i < comms_c; i++) {
    for (unsigned int j = 0;
         comms_buf[i][j] != 0 && dma_size < OUTPUT_BUFFER_SIZE; j++)
      dma_to_send[dma_size++] = comms_buf[i][j];
  }
}

void append_to_buttons(const char *to_append, int len) {
  for (int i = 0; i < len; i++, button_c++) {
    if (button_c == OUTPUT_BUFFER_SIZE)
      return;
    buttons_buffer[button_c] = to_append[i];
  }
}

void cpy_to_dma2(char *str, int len) {
  int i = 0;
  for (; i < len; i++) {
    dma_to_send[i] = str[i];
  }
  dma_size = i;
}

void dma_send() {
  irq_level_t level = IRQprotectAll();
  if (comms_c > 0 && (DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
      (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
    cpy_to_dma();
    DMA1_Stream6->M0AR = (uint32_t)dma_to_send;
    DMA1_Stream6->NDTR = dma_size;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    comms_c = 0;
    button_c = 0;
    dma_size = 0;
  }
  IRQunprotectAll(level);
}

void DMA1_Stream6_IRQHandler() {

  irq_level_t level = IRQprotectAll();
  /* Odczytaj zgłoszone przerwania DMA1. */
  uint32_t isr = DMA1->HISR;
  if (isr & DMA_HISR_TCIF6) {
    /* Obsłuż zakończenie transferu
    w strumieniu 6. */
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    dma_send();
  }
  IRQunprotectAll(level);
}

void handle_button(char btn_pos, char btn_pin, const char string[],
                   const char string2[], GPIO_TypeDef *gpio, char on_state) {
  if (!(((buttons_states & (1 << btn_pos)) >> btn_pos) == on_state) &&
      (((gpio->IDR >> btn_pin) & 1) == on_state)) {
    if (on_state)
      buttons_states |= (1 << btn_pos);
    else
      buttons_states &= ~(1 << btn_pos);
    append_to_comms(string);
    dma_send();
  }

  if ((((buttons_states & (1 << btn_pos)) >> btn_pos) == on_state) &&
      !(((gpio->IDR >> btn_pin) & 1) == on_state)) {
    if (on_state)
      buttons_states &= ~(1 << btn_pos);
    else
      buttons_states |= (1 << btn_pos);
    append_to_comms(string2);
    dma_send();
  }
}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & (1 << USER_BTN_PIN)) {
    handle_button(USER_BTN_POS, USER_BTN_PIN, user_press_com, user_rel_com,
                  USER_GPIO, USER_ON_STATE);
    EXTI->PR = EXTI_PR_PR13;
  }
  if (EXTI->PR & (1 << ACTION_BTN_PIN)) {
    handle_button(ACTION_BTN_POS, ACTION_BTN_PIN, action_press_com,
                  action_rel_com, JOYSTICK_GPIO, JOYSTICK_ON_STATE);
    EXTI->PR = EXTI_PR_PR10;
  }
}

void EXTI9_5_IRQHandler(void) {
  if (EXTI->PR & (1 << UP_BTN_PIN)) {
    handle_button(UP_BTN_POS, UP_BTN_PIN, up_press_com, up_rel_com,
                  JOYSTICK_GPIO, JOYSTICK_ON_STATE);
    EXTI->PR = EXTI_PR_PR5;
  }

  if (EXTI->PR & (1 << DOWN_BTN_PIN)) {
    handle_button(DOWN_BTN_POS, DOWN_BTN_PIN, down_press_com, down_rel_com,
                  JOYSTICK_GPIO, JOYSTICK_ON_STATE);
    EXTI->PR = EXTI_PR_PR6;
  }
}

void EXTI0_IRQHandler(void) {
  EXTI->PR = EXTI_PR_PR0;
  handle_button(AT_BTN_POS, AT_BTN_PIN, at_press_com, at_rel_com, AT_BTN_GPIO,
                AT_ON_STATE);
}

void EXTI3_IRQHandler(void) {
  handle_button(LEFT_BTN_POS, LEFT_BTN_PIN, left_press_com, left_rel_com,
                JOYSTICK_GPIO, JOYSTICK_ON_STATE);
  EXTI->PR = EXTI_PR_PR3;
}

void EXTI4_IRQHandler(void) {
  handle_button(RIGHT_BTN_POS, RIGHT_BTN_PIN, right_press_com, right_rel_com,
                JOYSTICK_GPIO, JOYSTICK_ON_STATE);
  EXTI->PR = EXTI_PR_PR4;
}
