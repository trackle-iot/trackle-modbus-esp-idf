#include "trackle_modbus.h"
#include <esp_log.h>

void app_main()
{
    ModbusError res;

    // Initialize and start Modbus controller
    modbus_config_t modbus_config = {
        .uart_num = UART_NUM_1,
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .tx_io_num = 17,
        .rx_io_num = 16,
        .rts_io_num = UART_PIN_NO_CHANGE,
        .cts_io_num = UART_PIN_NO_CHANGE,
        .mode = UART_MODE_UART,
    };

    Trackle_Modbus_init(&modbus_config);
    Trackle_Modbus_set_timeout(250);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uint16_t result[10];
    uint16_t write[10];

    while (1)
    {
        ESP_LOGI("main", "------------");
        ESP_LOGI("main", "T_READ_HOLDING_REGISTERS");
        res = Trackle_Modbus_execute_command(T_READ_HOLDING_REGISTERS, 1, 231, 1, &result);
        ESP_LOGI("main", "Function %d, res %d, value %d", T_READ_HOLDING_REGISTERS, res, result[0]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI("main", "");

        write[0] = 200;
        ESP_LOGI("main", "------------");
        ESP_LOGI("main", "T_WRITE_SINGLE_REGISTER");
        res = Trackle_Modbus_execute_command(T_WRITE_SINGLE_REGISTER, 1, 231, 1, &write);
        ESP_LOGI("main", "Function %d, res %d", T_WRITE_SINGLE_REGISTER, res);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI("main", "");

        /*write[0] = 200;
        ESP_LOGI("main", "------------");
        ESP_LOGI("main", "T_WRITE_MULTIPLE_REGISTERS");
        res = modbus_execute_command(T_WRITE_MULTIPLE_REGISTERS, 1, 231, 1, &write);
        ESP_LOGI("main", "Function %d, res %d", T_WRITE_MULTIPLE_REGISTERS, res);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI("main", "");*/
    }
}