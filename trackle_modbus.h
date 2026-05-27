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

#ifndef TRACKLE_MODBUS_H_
#define TRACKLE_MODBUS_H_

#include "driver/uart.h"
#include <lightmodbus/lightmodbus.h>
#include "modbus_hal.h"

#define TRACKLE_MODBUS_VERSION "3.0.0"

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
} modbus_config_rtu_t;

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

typedef enum
{
    READ = 1,
    WRITE_SINGLE = 2,
    WRITE_MULTIPLE = 3,
} TrackleModbusAction_t;

/**
 * @brief Modbus function enumeration
 */
typedef enum TrackleModbusFunction
{
    T_FUNCTION_ERROR = -1,
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
 * @brief Map a Modbus data type and action to the corresponding Modbus function command
 * @param dataType #TrackleModbusDataType_t specifying the type of Modbus data (Coil, Discrete Input, Holding Register, Input Register)
 * @param action #TrackleModbusAction_t specifying the action (Read, Write Single, Write Multiple)
 * @return
 *    - Corresponding #TrackleModbusFunction if mapping is successful
 *    - -1 if the data type and action combination is invalid
 */
static TrackleModbusFunction Trackle_Modbus_Command_from_DataType(TrackleModbusDataType_t dataType, TrackleModbusAction_t action)
{
    switch (dataType)
    {
    case T_COIL:
        if (action == READ)
        {
            return T_READ_COIL_STATUS;
        }
        else if (action == WRITE_SINGLE)
        {
            return T_WRITE_SINGLE_COIL;
        }
        else if (action == WRITE_MULTIPLE)
        {
            return T_WRITE_MULTIPLE_COILS;
        }
        break;
    case T_DISCRETE_INPUT:
        if (action == READ)
        {
            return T_READ_INPUT_STATUS;
        }
        break;
    case T_HOLDING_REGISTER:
        if (action == READ)
        {
            return T_READ_HOLDING_REGISTERS;
        }
        else if (action == WRITE_SINGLE)
        {
            return T_WRITE_SINGLE_REGISTER;
        }
        else if (action == WRITE_MULTIPLE)
        {
            return T_WRITE_MULTIPLE_REGISTERS;
        }
        break;
    case T_INPUT_REGISTER:
        if (action == READ)
        {
            return T_READ_INPUT_REGISTERS;
        }
        break;
    default:
        break;
    }
    return -1; // or an appropriate error value if you have one defined
}

/**
 * @brief Initialize Modbus master communication
 * @return
 *    - ESP_OK if init operation was successful
 *    - ESP_FAIL if modbusMasterInit was not ok
 *    - other error codes from the uart driver
 */
esp_err_t Trackle_Modbus_init_rtu();
esp_err_t Trackle_Modbus_init_tcp();

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
ModbusError Trackle_Modbus_execute_command_rtu(TrackleModbusFunction function, uint16_t address, uint16_t start, uint16_t size, void *value);
ModbusError Trackle_Modbus_execute_command_tcp(TrackleModbusFunction function, uint16_t address, uint16_t start, uint16_t size, void *value, int *sock, int device_id);

/**
 * @brief Set Modbus timeout for any command
 * @param timeout_ms Modbus max wait time for respone in milliseconds
 * @return true if success
 */
bool Trackle_Modbus_set_timeout_rtu(uint16_t timeout_ms);

/**
 * @brief Set Modbus pause between any command
 * @param pause_ms Modbus minimum pause in milliseconds
 * @return true if success
 */
bool Trackle_Modbus_set_pause_between_packets_rtu(uint16_t pause_ms);

int Trackle_Modbus_read_socket(TrackleModbusFunction function, uint16_t size, void *value, int sock, int device_id);
esp_err_t Trackle_Modbus_create_instance_tcp(int device_id);
esp_err_t Trackle_Modbus_delete_instance_tcp(int device_id);

#endif
