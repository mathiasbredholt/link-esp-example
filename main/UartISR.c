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

UartISR.c

*/

#include "UartISR.h"

#include "hal/uart_hal.h"

IRAM_ATTR void uartWriteBytesFromISR(uart_port_t port, void* data,
                                     size_t size) {
  uart_ll_write_txfifo(UART_LL_GET_HW(port), data, size);
}

IRAM_ATTR void uartReadBytesFromISR(uart_port_t port, void* data, int size) {
  uart_ll_read_rxfifo(UART_LL_GET_HW(port), data, size);
}

IRAM_ATTR int uartGetTxFifoLengthFromISR(uart_port_t port) {
  return uart_ll_get_txfifo_len(UART_LL_GET_HW(port));
}

IRAM_ATTR int uartGetRxFifoLengthFromISR(uart_port_t port) {
  return uart_ll_get_rxfifo_len(UART_LL_GET_HW(port));
}
