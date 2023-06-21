#ifndef __PERIPHERY_HPP
#define __PERIPHERY_HPP

#include "at32f403a_407.h"

/*GPIO*/
#define DISC_OUT_PORT  GPIOC
#define DISC_OUT_PORT2 GPIOD

#define DISC_OUT1 GPIO_PINS_8
#define DISC_OUT8 GPIO_PINS_7

/* Клавиатура */

#define KEYBOARD_PORT GPIOD

#define button1 GPIO_PINS_8
#define button2 GPIO_PINS_9
#define button3 GPIO_PINS_10
#define button4 GPIO_PINS_11
#define button5 GPIO_PINS_12
#define button6 GPIO_PINS_13


#endif