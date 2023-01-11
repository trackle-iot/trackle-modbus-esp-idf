
# Trackle Modbus

Trackle Modbus is a library for communicating with Modbus slave devices. 
It provides an easy-to-use interface for executing Modbus commands, such as reading and writing registers, and setting timeouts and pauses between packets.

## Requirements 
* ESP32 microcontroller programmed with esp-idf framework
 
## Installation  

1. Download latest library version from [here](https://github.com/trackle-iot/trackle-modbus-esp-idf/releases/latest).

2. Include the library in your project: #include "trackle_modbus.h"`  
3. Configure the modbus parameters:  

 ```c++  
    modbus_config_t modbus_config = {  
        .uart_num = UART_NUM_0,  
        .baud_rate = 9600,  
        .data_bits = UART_DATA_8_BITS,  
        .parity = UART_PARITY_DISABLE,  
        .stop_bits = UART_STOP_BITS_1,  
        .tx_io_num = 17,  
        .rx_io _num = 16 ,  
        .rts _io _num = -1 ,    // optional if flow control is not used 
        .cts _io _num = -1 ,    // optional if flow control is not used 
        .mode=UARTMODE ,     // optional if default mode is desired 
    };  

    esp _err _t ret=TrackleModbusInit(&modbusConfig);     // initialize modbus interface with given configs      
```     
    
4. Execute a Modbus command
```c++  
res = Trackle_Modbus_execute_command(T_READ_HOLDING_REGISTERS, 1, 231, 1, &result);
```     

## Features
* Supports TX/RX/RTS/CTS pin GPIO numbers 
* Supports UART modes of RX only, TX only or full-duplex 
* Supports set timeouts and pauses between packets 
* Executes Modbus commands such as reading and writing registers
* It uses [liblightmodbus](https://github.com/Jacajack/liblightmodbus), but gives synchronous commands execution.
