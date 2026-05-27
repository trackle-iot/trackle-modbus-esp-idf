#include "modbus_hal.h"
#include <driver/uart.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "esp_log.h"
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

#define MB_HAL_TAG "mb_hal"
#define ECHO_READ_TOUT (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
uart_port_t modbus_uart_num;

#define MB_PORTNUM UART_NUM_1
#define MB_UART_TXD 17
#define MB_UART_RXD 18
#define MB_DERE 8

// UART functions for RTU
int hal_hardware_uart_config(int uart_num, int baud_rate, int data_bits, int parity, int stop_bits, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = data_bits,
        .parity = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    modbus_uart_num = uart_num;

    int result = 0;
    result += uart_param_config(uart_num, &uart_config);
    result += uart_set_pin(uart_num, tx_io_num, rx_io_num, rts_io_num, cts_io_num);
    result += uart_driver_install(uart_num, 512, 512, 10, NULL, 0);
    result += uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
    result += uart_set_rx_timeout(uart_num, ECHO_READ_TOUT);
    return result;
}

int hal_uart_write(const uint8_t *data, size_t length)
{
    int len = uart_write_bytes(modbus_uart_num, data, length);
    return len;
}

int hal_uart_read(uint8_t *buffer, size_t length, int timeout)
{
    return uart_read_bytes(modbus_uart_num, buffer, length, timeout / portTICK_PERIOD_MS);
}

int hal_uart_flush()
{
    return uart_flush(modbus_uart_num);
}

// Socket functions for TCP

int hal_socket_connect(int *sock, uint8_t ip[4], int port)
{
    ESP_LOGI(MB_HAL_TAG, "hal_socket_connect %d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], port);

    struct sockaddr_in server_addr;
    *sock = socket(AF_INET, SOCK_STREAM, 0);
    if (*sock < 0)
    {
        ESP_LOGI(MB_HAL_TAG, "Unable to create socket: errno %d\n", errno);
        return -1;
    }

    // Impostare il socket in modalità non bloccante (manteniamo O_NONBLOCK)
    int flags = fcntl(*sock, F_GETFL, 0);
    if (flags < 0 || fcntl(*sock, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        ESP_LOGI(MB_HAL_TAG, "fcntl error\n");
        hal_socket_close(sock);
        return -1;
    }

    memcpy(&server_addr.sin_addr.s_addr, ip, 4);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &server_addr.sin_addr, ip_str, sizeof(ip_str));
    ESP_LOGI(MB_HAL_TAG, "Attempting to connect to %s:%d", ip_str, ntohs(server_addr.sin_port));

    int err = connect(*sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0 && errno != EINPROGRESS)
    {
        ESP_LOGI(MB_HAL_TAG, "Socket unable to connect: errno %d\n", errno);
        hal_socket_close(sock);
        return -1;
    }

    // Attendere fino a 2 secondi per la connessione
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(*sock, &write_fds);

    err = select(*sock + 1, NULL, &write_fds, NULL, &tv);
    if (err <= 0)
    {
        ESP_LOGI(MB_HAL_TAG, "Connection timeout or error\n");
        hal_socket_close(sock);
        return -1;
    }

    // Controllare eventuali errori sulla connessione
    int so_error = 0;
    socklen_t len = sizeof(so_error);
    if (getsockopt(*sock, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0 || so_error != 0)
    {
        ESP_LOGI(MB_HAL_TAG, "Socket error: %d\n", so_error);
        hal_socket_close(sock);
        return -1;
    }

    // Mantieni socket NON bloccante (non togliere O_NONBLOCK)
    // rimuovo la riga che prima lo rimetteva bloccante:
    // fcntl(*sock, F_SETFL, flags);

    // Impostare il timeout sul socket per la ricezione (qui opzionale)
    tv.tv_sec = 0;
    tv.tv_usec = 0; // 0 = read ritorna immediatamente se non ci sono dati
    if (setsockopt(*sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        ESP_LOGI(MB_HAL_TAG, "setsockopt error\n");
        hal_socket_close(sock);
        return -1;
    }

    return 0;
}

int hal_socket_read(int fd, void *buf, size_t count)
{
    return read(fd, buf, count);
}

int hal_socket_write(int fd, const void *buf, size_t count)
{
    return send(fd, buf, count, 0);
}

void hal_socket_close(int *fd)
{
    ESP_LOGI(MB_HAL_TAG, "hal_socket_close %d\n", *fd);

    if (*fd >= 0)
    {
        close(*fd);
        *fd = -1;
    }
}