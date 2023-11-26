#include "buttons.h"
#include <gpio.h>
#include <stm32f411xe.h>

static char buttons_states = 0 | (1 << UP_BTN_POS) | (1 << DOWN_BTN_POS) |
                             (1 << RIGHT_BTN_POS) | (1 << LEFT_BTN_POS) |
                             (1 << USER_BTN_POS) | (1 << ACTION_BTN_POS);
static char output_buffer[OUTPUT_BUFFER_SIZE];
static int output_begin = 0;
static int output_end = 0;
static int elements_c = 0;

void append_to_output(char *to_append, int len) {
  for (int i = 0; i < len; i++) {
    if (elements_c == OUTPUT_BUFFER_SIZE)
      return;
    output_buffer[output_end] = to_append[i];
    output_end = (output_end + 1) % OUTPUT_BUFFER_SIZE;
    elements_c++;
  }
}

void handle_button(char btn_pos, char btn_pin, char *string, char str_len,
                      char *string2, char str2_len, GPIO_TypeDef *gpio,
                      char on_state) {
  if (!(((buttons_states & (1 << btn_pos)) >> btn_pos) == on_state) &&
      (((gpio->IDR >> btn_pin) & 1) == on_state)) {
    if (on_state)
      buttons_states |= (1 << btn_pos);
    else
      buttons_states &= ~(1 << btn_pos);
    append_to_output(string, str_len);
  }

  if ((((buttons_states & (1 << btn_pos)) >> btn_pos) == on_state) &&
      !(((gpio->IDR >> btn_pin) & 1) == on_state)) {
    append_to_output(string2, str2_len);
    if (on_state)
      buttons_states &= ~(1 << btn_pos);
    else
      buttons_states |= (1 << btn_pos);
  }

  if (USART2->SR & USART_SR_TXE) {
    if (elements_c > 0) {
      USART2->DR = output_buffer[output_begin];
      output_begin = (output_begin + 1) % OUTPUT_BUFFER_SIZE;
      elements_c--;
    }
  }
}

void handle_buttons() {
  handle_button(DOWN_BTN_POS, DOWN_BTN_PIN, "DOWN PRESSED\r\n", 14,
                   "DOWN RELEASED\r\n", 15, JOYSTICK_GPIO, JOYSTICK_ON_STATE);
  handle_button(UP_BTN_POS, UP_BTN_PIN, "UP PRESSED\r\n", 12,
                   "UP RELEASED\r\n", 13, JOYSTICK_GPIO, JOYSTICK_ON_STATE);
  handle_button(RIGHT_BTN_POS, RIGHT_BTN_PIN, "RIGHT PRESSED\r\n", 15,
                   "RIGHT RELEASED\r\n", 16, JOYSTICK_GPIO, JOYSTICK_ON_STATE);
  handle_button(LEFT_BTN_POS, LEFT_BTN_PIN, "LEFT PRESSED\r\n", 14,
                   "LEFT RELEASED\r\n", 15, JOYSTICK_GPIO, JOYSTICK_ON_STATE);
  handle_button(ACTION_BTN_POS, ACTION_BTN_PIN, "ACTION PRESSED\r\n", 16,
                   "ACTION RELEASED\r\n", 17, JOYSTICK_GPIO, JOYSTICK_ON_STATE);
  handle_button(USER_BTN_POS, USER_BTN_PIN, "USER PRESSED\r\n", 14,
                   "USER RELEASED\r\n", 15, USER_GPIO, USER_ON_STATE);
  handle_button(AT_BTN_POS, AT_BTN_PIN, "AT PRESSED\r\n", 12,
                   "AT RELEASED\r\n", 13, AT_BTN_GPIO, AT_ON_STATE);
}
