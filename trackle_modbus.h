/**
 ******************************************************************************
  Copyright (c) 2022 IOTREADY S.r.l.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

#include "driver/uart.h"
#include <lightmodbus/lightmodbus.h>

#define TRACKLE_MODBUS_VERSION "1.0.0"

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

#define MAX_SLAVE_ADDRESSES_NUM 32

/**
 * @brief Trackle Modbus data type
 */
typedef enum
{
    T_HOLDING_REGISTER = 1, //!< Holding register
    T_INPUT_REGISTER = 2,   //!< Input register
    T_COIL = 4,             //!< Coil
    T_DISCRETE_INPUT = 8    //!< Discrete input
} TrackleModbusDataType_t;

/**
 * @brief Modbus slave configuration parameters
 */
typedef struct
{
    uart_port_t uart_num;                            /*!< UART port number, can be UART_NUM_0 ~ (UART_NUM_MAX -1) */
    int baud_rate;                                   /*!< UART baud rate */
    uart_word_length_t data_bits;                    /*!< UART byte size */
    uart_parity_t parity;                            /*!< UART parity mode */
    uart_stop_bits_t stop_bits;                      /*!< UART stop bits */
    int tx_io_num;                                   /*!< UART TX pin GPIO number */
    int rx_io_num;                                   /*!< UART RX pin GPIO number */
    int rts_io_num;                                  /*!< UART RTS pin GPIO number */
    int cts_io_num;                                  /*!< UART CTS pin GPIO number */
    uart_mode_t mode;                                /*!< UART mode to set */
    uint8_t slaveAddresses[MAX_SLAVE_ADDRESSES_NUM]; /*!< Slave addresses array */
    uint8_t slaveAddressesCount;                     /*!< Slave addresses array length */
    bool (*slaveRequestCallback)(bool check,
                                 uint8_t destAddr,
                                 uint16_t index,
                                 uint16_t valueToWrite,
                                 uint16_t *readValue,
                                 uint8_t function,
                                 TrackleModbusDataType_t dataType);
} modbus_slave_config_t;

/**
 * @brief Modbus function enumeration
 */
typedef enum TrackleModbusFunction
{
    T_READ_COIL_STATUS = 1,
    T_READ_INPUT_STATUS = 2,
    T_READ_HOLDING_REGISTERS = 3,
    T_READ_INPUT_REGISTERS = 4,
    T_WRITE_SINGLE_COIL = 5,
    T_WRITE_SINGLE_REGISTER = 6,
    T_WRITE_MULTIPLE_COILS = 15,
    T_WRITE_MULTIPLE_REGISTERS = 16,
} TrackleModbusFunction;

/**
 * @brief Initialize Modbus master communication
 * @param modbus_config #modbus_config_t to configure Modbus interface
 * @return
 *    - ESP_OK if init operation was successful
 *    - ESP_FAIL if modbusMasterInit was not ok
 *    - other error codes from the uart driver
 */
esp_err_t Trackle_Modbus_init(const modbus_config_t *modbus_config);

/**
 * @brief Initialize Modbus slave communication
 * @param config configuration of modbus slave interface
 * @return
 *    - ESP_OK if init was successful
 *    - ESP_FAIL if modbusSlaveInit was not ok
 *    - other error codes from the uart driver
 */
esp_err_t Trackle_Modbus_Slave_init(const modbus_slave_config_t *config);

/**
 * @brief Execute a Modbus command
 * @param function #TrackleModbusFunction to specify command type
 * @param address Slave address to read / write
 * @param start First register to read / write
 * @param size Number of registers to read / write
 * @param value For write function is the buffer to write, for read function is buffer to be fulled by read operation
 * @return
 *    - MODBUS_ERROR_OK if operation was successful
 *    - MODBUS_ERROR_LENGTH if read / write operation goes in timeout
 *    - other error codes from the lightmodbus library
 */
ModbusError Trackle_Modbus_execute_command(TrackleModbusFunction function, uint16_t address, uint16_t start, uint16_t size, void *value);

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
