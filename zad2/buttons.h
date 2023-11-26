#ifndef __BUTTONS__
#define __BUTTONS__

#define BUTTONS_COUNT 7

#define JOYSTICK_GPIO GPIOB
#define JOYSTICK_ON_STATE 0

#define UP_BTN_PIN 5
#define UP_BTN_POS 0

#define DOWN_BTN_PIN 6
#define DOWN_BTN_POS 1

#define LEFT_BTN_PIN 3
#define LEFT_BTN_POS 2

#define RIGHT_BTN_PIN 4
#define RIGHT_BTN_POS 3

#define ACTION_BTN_PIN 10
#define ACTION_BTN_POS 4

#define AT_ON_STATE 1
#define AT_BTN_GPIO GPIOA
#define AT_BTN_PIN 0
#define AT_BTN_POS 5

#define USER_ON_STATE 0
#define USER_GPIO GPIOC
#define USER_BTN_PIN 13
#define USER_BTN_POS 6

#define OUTPUT_BUFFER_SIZE 100
#define DMA_MESS_SIZE 30

void handle_buttons();

#endif
