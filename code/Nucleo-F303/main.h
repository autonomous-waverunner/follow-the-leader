#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_misc.h"
#include "string.h"
#include "stdio.h"
#include <stdint.h>
#include<stdlib.h> 
#include <stdbool.h>

volatile uint32_t msTicks;
volatile char rcvd_str[80];
volatile int cnt = 0;
volatile bool str_built = false;

	
#endif //PROTOCOL_H