/*

         88
  ,d     88
  88     88
MM88MMM  88  ,adPPYba,   ,adPPYba,
  88     88  I8[    ""  a8"     "8a
  88     88   `"Y8ba,   8b       d8
  88,    88  aa    ]8I  "8a,   ,a8"
  "Y888  88  `"YbbdP"'   `"YbbdP"'
         88

Copyright Torso Electronics 2020
All rights reserved.

*-0-*-0-*-0-*-0-*-0-*-0-*-0-*-0-*-0-*

UartIram.h

*/

#pragma once

#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

void uartWriteBytesFromISR(uart_port_t port, void* data, size_t size);

void uartReadBytesFromISR(uart_port_t port, void* data, int size);

int uartGetTxFifoLengthFromISR(uart_port_t port);

int uartGetRxFifoLengthFromISR(uart_port_t port);

#ifdef __cplusplus
}
#endif
