#pragma once

#include <termios.h>

typedef int (*open_uart_func_t)(uint8_t, speed_t);
typedef int (*write_uart_func_t)(int, void*, size_t);
typedef int (*read_uart_func_t)(int,  void*, size_t);

void configure_uart_callbacks(open_uart_func_t, write_uart_func_t, read_uart_func_t);
