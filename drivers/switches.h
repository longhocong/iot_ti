/*
 * switches.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Quoc Bao
 */
#ifndef SWITCHES_H_
#define SWITCHES_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

#define SW_GPIO_PERIPH     SYSCTL_PERIPH_GPIOF
#define SW_GPIO_BASE       GPIO_PORTF_BASE
#define SW1_PIN            GPIO_PIN_4
#define SW2_PIN            GPIO_PIN_0

#define BUTTONS_GPIO_PERIPH     SYSCTL_PERIPH_GPIOF
#define BUTTONS_GPIO_BASE       GPIO_PORTF_BASE

#define NUM_BUTTONS             2
#define LEFT_BUTTON             GPIO_PIN_4
#define RIGHT_BUTTON            GPIO_PIN_0

#define ALL_BUTTONS             (LEFT_BUTTON | RIGHT_BUTTON)
typedef     enum {PRESSED,RELEASED}  sw_t;
void        switchInit(void);

sw_t        switchState(int SWnumber);
void   swIntEnable(void);
#endif /* SWITCHES_H_ */
