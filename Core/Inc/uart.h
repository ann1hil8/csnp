/*
 * uart.h
 *
 *  Created on: Oct 22, 2024
 *      Author: Ryan
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "main.h"

extern UART_HandleTypeDef huart2;

void myprintf(const char *fmt, ...);

#endif /* INC_UART_H_ */
