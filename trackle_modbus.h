#include "driver/uart.h"
#include <lightmodbus/lightmodbus.h>

#define TRACKLE_MODBUS_VERSION "0.9.0"

/**
 * @brief Modbus configuration parameters
 */
typedef struct
{
    uart_port_t uart_num;         /*!< UART port number, can be UART_NUM_0 ~ (UART_NUM_MAX -1) */
    int baud_rate;                /*!< UART baud rate */
    uart_word_length_t data_bits; /*!< UART byte size */
    uart_parity_t parity;         /*!< UART parity mode */
    uart_stop_bits_t stop_bits;   /*!< UART stop bits */
    int tx_io_num;                /*!< UART TX pin GPIO number */
    int rx_io_num;                /*!< UART RX pin GPIO number */
    int rts_io_num;               /*!< UART RTS pin GPIO number */
    int cts_io_num;               /*!< UART CTS pin GPIO number */
    uart_mode_t mode;             /*!< UART mode to set */
} modbus_config_t;

/**
 * @brief Modbus function enumeration
 */
typedef enum TracleModbusFunction
{
    T_READ_COIL_STATUS = 1,
    T_READ_INPUT_STATUS = 2,
    T_READ_HOLDING_REGISTERS = 3,
    T_READ_INPUT_REGISTERS = 4,
    T_WRITE_SINGLE_COIL = 5,
    T_WRITE_SINGLE_REGISTER = 6,
    T_WRITE_MULTIPLE_COILS = 15,
    T_WRITE_MULTIPLE_REGISTERS = 16,
} TracleModbusFunction;

/**
 * @brief Initialize Modbus communication
 * @param modbus_config /ref modbus_config_t to configure Modbus intergace
 * @return
 *    - ESP_OK if init operation was successful
 *    - ESP_FAIL if modbusMasterInit was not ok
 *    - other error codes from the uart driver
 */
esp_err_t Trackle_Modbus_init(const modbus_config_t *modbus_config);

/**
 * @brief Execute a Modbus command
 * @param function /ref TracleModbusFunction to specify command type
 * @param address Slave address to read / write
 * @param start First register to read / write
 * @param size Number of registers to read / write
 * @param value For write function is the buffer to write, for read function is buffer to be fulled by read operation
 * @return
 *    - MODBUS_ERROR_OK if operation was successful
 *    - MODBUS_ERROR_LENGTH if read / write operation goes in timeout
 *    - other error codes from the lightmodbus library
 */
ModbusError Trackle_Modbus_execute_command(TracleModbusFunction function, uint16_t address, uint16_t start, uint16_t size, void *value);

/**
 * @brief Set Modbus timeout for any command
 * @param timeout_ms Modbus max wait time for respone in milliseconds
 * @return true if success
 */
bool Trackle_Modbus_set_timeout(uint16_t timeout_ms);

/**
 * @brief Set Modbus pause between any command
 * @param pause_ms Modbus minimum pause in milliseconds
 * @return true if success
 */
bool Trackle_Modbus_set_pause_between_packets(uint16_t pause_ms);