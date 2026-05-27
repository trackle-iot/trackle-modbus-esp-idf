#ifndef MODBUS_HAL_H
#define MODBUS_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

// UART functions for RTU
int hal_hardware_uart_config(int uart_num, int baud_rate, int data_bits, int parity, int stop_bits, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num);
int hal_software_uart_config(const char *uart_port, int baud_rate, int data_bits, int parity, int stop_bits);
int hal_uart_write(const uint8_t *data, size_t length);
int hal_uart_read(uint8_t *buffer, size_t length, int timeout);
int hal_uart_flush();
int hal_uart_flush_input();
int hal_uart_wait_tx_done(int timeout_ms);
void hal_dere_set(bool tx_mode);

// Socket functions for TCP
int hal_socket_connect(int *sock, uint8_t ip[4], int port);
int hal_socket_read(int sock, void *buf, size_t count);
int hal_socket_write(int sock, const void *buf, size_t count);
void hal_socket_close(int *sock);

#endif // HAL_H
