#include "trackle_modbus.h"

#define ESP_RETURN_ON_ERROR(x)           \
    {                                    \
        esp_err_t err_rc_ = (x);         \
        if (unlikely(err_rc_ != ESP_OK)) \
        {                                \
            return err_rc_;              \
        }                                \
    }

#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/uart.h>
#include <inttypes.h>
#include <esp_log.h>

#define TAG_M "modbus"

#define ECHO_READ_TOUT (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define MODBUS_MAX_PACKET_SIZE 256

uart_port_t modbus_uart_num;
uint8_t modbus_uart_read_buffer[MODBUS_MAX_PACKET_SIZE * 2] = {0};
uint16_t modbus_read_buffer[MODBUS_MAX_PACKET_SIZE] = {0};
uint16_t MODBUS_WAIT_BETWEEN_PACKETS = 20 / portTICK_RATE_MS;
uint16_t MODBUS_TIMEOUT = 100 / portTICK_RATE_MS;

TickType_t xLastWriteTime;
ModbusMaster master;
ModbusSlave slave;
ModbusExceptionCode lastError = MODBUS_EXCEP_NONE;

#define MAX_SLAVE_ADDRESSES_NUM 32
static uart_port_t modbus_slave_uart_num;
static uint8_t modbus_slave_uart_read_buffer[MODBUS_MAX_PACKET_SIZE * 2] = {0};
static uint8_t slaveAddresses[MAX_SLAVE_ADDRESSES_NUM] = {0};
static uint8_t slaveAddressesCount = 0;
static bool (*slaveRequestCallback)(bool check,
                                    uint8_t destAddr,
                                    uint16_t index,
                                    uint16_t valueToWrite,
                                    uint16_t *readValue,
                                    uint8_t function,
                                    TrackleModbusDataType_t dataType,
                                    uint16_t regIdxInRequest,
                                    uint16_t regsTotCountInRequest);

/*
return len of message for a given function
*/
static int modbus_response_len(TrackleModbusFunction function, int data_len)
{
    int len = 0;

    switch (function)
    {
    case T_READ_COIL_STATUS:
    case T_READ_INPUT_STATUS:
        len = (5 + ceil((double)data_len / 8));
        break;

    case T_READ_HOLDING_REGISTERS:
    case T_READ_INPUT_REGISTERS:
        len = (5 + data_len * 2);
        break;

    default:
        ESP_LOGD(TAG_M, "Write function is alway size 8");
        len = 8;
        break;
    }

    return len;
}

/*
    Data callback for printing all incoming data
*/
static ModbusError dataCallback(const ModbusMaster *master, const ModbusDataCallbackArgs *args)
{
    uint16_t index = ((uint16_t)modbusMasterGetRequest(master)[2] << 8) | modbusMasterGetRequest(master)[3];
    ESP_LOGD(TAG_M, "index %u", index);
    ESP_LOGD(TAG_M, "args->index %u", args->index);
    ESP_LOGD(TAG_M, "value %u", args->value);

    // shift first register to 0
    modbus_read_buffer[args->index - index] = args->value;

    // Always return MODBUS_OK
    return MODBUS_OK;
}

static TrackleModbusDataType_t dataTypeInternalToExternal(ModbusDataType dataType)
{
    switch (dataType)
    {
    case MODBUS_HOLDING_REGISTER:
        return T_HOLDING_REGISTER;
    case MODBUS_INPUT_REGISTER:
        return T_INPUT_REGISTER;
    case MODBUS_COIL:
        return T_COIL;
    case MODBUS_DISCRETE_INPUT:
        return T_DISCRETE_INPUT;
    }
    ESP_LOGE(TAG_M, "Invalid data type. Don't know how to convert.");
    return -1;
}

/*
    Data callback for requests to slave
*/
static ModbusError slaveDataCallback(const ModbusSlave *slave, const ModbusRegisterCallbackArgs *args, ModbusRegisterCallbackResult *result)
{
    ESP_LOGE("mb", "ENTERED");

    const bool check = args->query == MODBUS_REGQ_R_CHECK || args->query == MODBUS_REGQ_W_CHECK;

    const bool success = slaveRequestCallback(check,
                                              args->rtuSlaveAddr,
                                              args->index,
                                              args->value,
                                              &result->value,
                                              args->function,
                                              dataTypeInternalToExternal(args->type),
                                              args->regIdxInRequest,
                                              args->regsTotCountInRequest);

    result->exceptionCode = success ? MODBUS_EXCEP_NONE : MODBUS_EXCEP_SLAVE_FAILURE;

    ESP_LOGE("mb", "EXITED");

    return MODBUS_OK;
}

/*
    Exception callback for printing out exceptions on master side
*/
ModbusError masterExceptionCallback(const ModbusMaster *master, uint8_t address, uint8_t function, ModbusExceptionCode code)
{
    ESP_LOGE(TAG_M, "Received slave %d exception %d (function %d)\n", address, (code), function);
    lastError = code;

    // Always return MODBUS_OK
    return MODBUS_OK;
}

ModbusError Trackle_Modbus_execute_command(TrackleModbusFunction function, uint16_t address, uint16_t start, uint16_t size, void *value)
{

    ModbusErrorInfo err;
    lastError = MODBUS_EXCEP_NONE;

    // reset read buffer
    memset(modbus_uart_read_buffer, 0, MODBUS_MAX_PACKET_SIZE * 2);
    memset(modbus_read_buffer, 0, sizeof(modbus_read_buffer));

    // fix size > max size
    if (size > MODBUS_MAX_PACKET_SIZE)
    {
        size = MODBUS_MAX_PACKET_SIZE;
    }

    switch (function)
    {
    case T_READ_COIL_STATUS:
        size = 1;
        err = modbusBuildRequest01RTU(&master, address, start, size);
        break;

    case T_READ_INPUT_STATUS:
        size = 1;
        err = modbusBuildRequest02RTU(&master, address, start, size);
        break;

    case T_READ_HOLDING_REGISTERS:
        err = modbusBuildRequest03RTU(&master, address, start, size);
        break;

    case T_READ_INPUT_REGISTERS:
        err = modbusBuildRequest04RTU(&master, address, start, size);
        break;

    case T_WRITE_SINGLE_COIL:
        size = 1;
        err = modbusBuildRequest05RTU(&master, address, start, ((uint8_t *)value)[0]);
        break;

    case T_WRITE_SINGLE_REGISTER:
        size = 1;
        err = modbusBuildRequest06RTU(&master, address, start, ((uint16_t *)value)[0]);
        break;

    case T_WRITE_MULTIPLE_COILS:
        err = modbusBuildRequest15RTU(&master, address, start, size, (uint8_t *)value);
        break;

    case T_WRITE_MULTIPLE_REGISTERS:
        err = modbusBuildRequest16RTU(&master, address, start, size, (uint16_t *)value);
        break;

    default:
        ESP_LOGE(TAG_M, "No valid modbus function");
        err = MODBUS_GENERAL_ERROR(FUNCTION);
        break;
    }

    if (!modbusIsOk(err))
    {
        return modbusGetErrorCode(err);
    }

    // write to serial
    vTaskDelayUntil(&xLastWriteTime, MODBUS_WAIT_BETWEEN_PACKETS);
    uart_flush(modbus_uart_num); // very important!!!!
    int len = uart_write_bytes(modbus_uart_num, modbusMasterGetRequest(&master), modbusMasterGetRequestLength(&master));

    // error writing to serial
    if (len < 0)
    {
        return MODBUS_ERROR_LENGTH;
    }

    len = uart_read_bytes(modbus_uart_num, modbus_uart_read_buffer, modbus_response_len(function, size), MODBUS_TIMEOUT);
    xLastWriteTime = xTaskGetTickCount();

    // parse response
    // ESP_LOG_BUFFER_HEX_LEVEL(TAG_M, modbus_uart_read_buffer, len, ESP_LOG_WARN);
    err = modbusParseResponseRTU(&master, modbusMasterGetRequest(&master),
                                 modbusMasterGetRequestLength(&master), modbus_uart_read_buffer, len);

    // force error if masterExceptionCallback is fired
    if (lastError > 0)
    {
        err.error = MODBUS_ERROR_OTHER;
    }

    if (!modbusIsOk(err))
    {
        if (function <= T_READ_INPUT_REGISTERS)
        {
            ESP_LOGW(TAG_M, "Could not read from device %d (%d): %i %i", address, len, err.source, err.error);
        }
        else
        {
            ESP_LOGW(TAG_M, "Could not write to device %d (%d): %i %i", address, len, err.source, err.error);
        }
        return modbusGetErrorCode(err);
    }

    // if read function, write results to value buffer
    if (function == T_READ_COIL_STATUS || function == T_READ_INPUT_STATUS)
    {
        ESP_LOGI(TAG_M, "Updating result value uint8_t buffer");
        for (int i = 0; i < size; i++)
        {
            ((uint8_t *)value)[i] = (uint8_t)modbus_read_buffer[i];
        }
    }
    else if (function == T_READ_HOLDING_REGISTERS || function == T_READ_INPUT_REGISTERS)
    {
        ESP_LOGD(TAG_M, "Updating result value uint16_t buffer");
        for (int i = 0; i < size; i++)
        {
            ((uint16_t *)value)[i] = modbus_read_buffer[i];
        }
    }
    else
    {
        ESP_LOGD(TAG_M, "Not a read function, not updating");
    }

    // all ok
    return MODBUS_ERROR_OK;
}

// Modbus master initialization
esp_err_t Trackle_Modbus_init(const modbus_config_t *modbus_config)
{

    // Initialize and start Modbus controller
    uart_config_t uart_config = {
        .baud_rate = modbus_config->baud_rate,
        .data_bits = modbus_config->data_bits,
        .parity = modbus_config->parity,
        .stop_bits = modbus_config->stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    modbus_uart_num = modbus_config->uart_num;
    ESP_RETURN_ON_ERROR(uart_param_config(modbus_config->uart_num, &uart_config));
    uart_set_pin(modbus_config->uart_num, modbus_config->tx_io_num, modbus_config->rx_io_num, modbus_config->rts_io_num, modbus_config->cts_io_num);

    ESP_RETURN_ON_ERROR(uart_driver_install(modbus_config->uart_num, 512, 512, 10, NULL, 0));
    ESP_RETURN_ON_ERROR(uart_set_mode(modbus_config->uart_num, modbus_config->mode));
    ESP_RETURN_ON_ERROR(uart_set_rx_timeout(modbus_config->uart_num, ECHO_READ_TOUT));

    ModbusErrorInfo err = modbusMasterInit(&master,
                                           dataCallback,                    // Callback for handling incoming data
                                           masterExceptionCallback,         // Exception callback (optional)
                                           modbusDefaultAllocator,          // Memory allocator used to allocate request
                                           modbusMasterDefaultFunctions,    // Set of supported functions
                                           modbusMasterDefaultFunctionCount // Number of supported functions
    );

    if (!modbusIsOk(err))
    {
        ESP_LOGE(TAG_M, "modbusMasterInit() failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void mbSlaveTaskCode(void *args)
{
    ESP_LOGE("slave", "slave task created and started");

    TickType_t latestWakeTick = xTaskGetTickCount();
    for (;;)
    {
        const int readBytes = uart_read_bytes(modbus_slave_uart_num, modbus_slave_uart_read_buffer, MODBUS_MAX_PACKET_SIZE, 10 / portTICK_PERIOD_MS);
        if (readBytes > 0)
        {
            const ModbusErrorInfo err = modbusParseRequestRTU(&slave, slaveAddresses, slaveAddressesCount, modbus_slave_uart_read_buffer, readBytes);
            if (modbusIsOk(err) && modbusSlaveGetResponseLength(&slave))
                uart_write_bytes(modbus_slave_uart_num, modbusSlaveGetResponse(&slave), modbusSlaveGetResponseLength(&slave));
        }
        vTaskDelayUntil(&latestWakeTick, 10 / portTICK_PERIOD_MS);
    }
}

esp_err_t Trackle_Modbus_Slave_init(const modbus_slave_config_t *modbus_config)
{
    // Initialize and start Modbus controller
    uart_config_t uart_config = {
        .baud_rate = modbus_config->baud_rate,
        .data_bits = modbus_config->data_bits,
        .parity = modbus_config->parity,
        .stop_bits = modbus_config->stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Copy slave addresses we must answer to
    if (modbus_config->slaveAddressesCount <= 0)
        return ESP_FAIL;
    slaveAddressesCount = modbus_config->slaveAddressesCount;
    for (uint8_t i = 0; i < slaveAddressesCount; i++)
        slaveAddresses[i] = modbus_config->slaveAddresses[i];

    // Save internally the slave request callback
    if (modbus_config->slaveRequestCallback == NULL)
        return ESP_FAIL;
    slaveRequestCallback = modbus_config->slaveRequestCallback;

    // Configure UART parameters
    modbus_slave_uart_num = modbus_config->uart_num;
    ESP_RETURN_ON_ERROR(uart_param_config(modbus_config->uart_num, &uart_config));
    uart_set_pin(modbus_config->uart_num, modbus_config->tx_io_num, modbus_config->rx_io_num, modbus_config->rts_io_num, modbus_config->cts_io_num);

    ESP_RETURN_ON_ERROR(uart_driver_install(modbus_config->uart_num, 512, 512, 10, NULL, 0));
    ESP_RETURN_ON_ERROR(uart_set_mode(modbus_config->uart_num, modbus_config->mode));
    ESP_RETURN_ON_ERROR(uart_set_rx_timeout(modbus_config->uart_num, ECHO_READ_TOUT));

    ModbusErrorInfo err = modbusSlaveInit(
        &slave,
        slaveDataCallback,
        NULL,
        modbusDefaultAllocator,
        modbusSlaveDefaultFunctions,
        modbusSlaveDefaultFunctionCount);

    if (!modbusIsOk(err))
    {
        ESP_LOGE(TAG_M, "modbusSlaveInit() failed");
        return ESP_FAIL;
    }

    BaseType_t res = xTaskCreate(mbSlaveTaskCode, "mbslave_task", 8192, NULL, tskIDLE_PRIORITY + 5, NULL);

    if (res != pdTRUE)
    {
        ESP_LOGE("slave", "slave task creation error");
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool Trackle_Modbus_set_timeout(uint16_t timeout_ms)
{
    MODBUS_TIMEOUT = timeout_ms / portTICK_RATE_MS;
    return true;
}

bool Trackle_Modbus_set_pause_between_packets(uint16_t pause_ms)
{
    MODBUS_WAIT_BETWEEN_PACKETS = pause_ms / portTICK_RATE_MS;
    return true;
}
