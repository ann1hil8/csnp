/*
 * uart.c
 *
 *  Created on: Oct 22, 2024
 *      Author: Ryan
 */

#include "uart.h"

void myprintf(const char *fmt, ...) {
    static char buffer[4095]; // Static buffer to store the formatted string
    // Initialize variable arguments list
    va_list args;
    va_start(args, fmt);
    // Format the string into the buffer
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    // Clean up variable arguments list
    va_end(args);
    // Transmit the formatted string to UART2
    HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY); // Use HAL_MAX_DELAY for blocking transmission
}
