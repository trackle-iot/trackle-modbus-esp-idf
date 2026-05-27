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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "uthash.h"

#define TAG_M "modbus"

#define ECHO_READ_TOUT_TCP (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define MODBUS_MAX_PACKET_SIZE_TCP 256

uart_port_t modbus_uart_num_tcp;
bool modbus_global_error = false; // used if error with instance o generic

// Struttura per istanza Modbus per device
typedef struct ModbusTCPInstance
{
    int device_id;                                            // ID del device
    ModbusMaster master;                                      // Master Modbus dedicato
    ModbusExceptionCode lastError;                            // Ultimo errore per questo device
    uint16_t transaction_id;                                  // Transaction ID dedicato
    uint8_t uart_read_buffer[MODBUS_MAX_PACKET_SIZE_TCP * 2]; // BUffer lettura
    uint16_t read_buffer[MODBUS_MAX_PACKET_SIZE_TCP];         // buffer risposta
    UT_hash_handle hh;                                        // Hash table handle
} ModbusTCPInstance;

// Variabili globali statiche
static ModbusTCPInstance *tcp_instances = NULL;  // Hash table delle istanze
static SemaphoreHandle_t instances_mutex = NULL; // Mutex per proteggere hash table

static ModbusTCPInstance *find_instance(int device_id)
{
    ModbusTCPInstance *instance = NULL;

    if (instances_mutex && xSemaphoreTake(instances_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        HASH_FIND_INT(tcp_instances, &device_id, instance);
        xSemaphoreGive(instances_mutex);
    }
    else
    {
        ESP_LOGW(TAG_M, "Timeout acquisizione mutex per ricerca device %d", device_id);
    }

    return instance;
}

/*
    Data callback for printing all incoming data
*/
static ModbusError dataCallback_tcp(const ModbusMaster *master, const ModbusDataCallbackArgs *args)
{
    ModbusTCPInstance *instance = (ModbusTCPInstance *)modbusMasterGetUserPointer(master);
    if (!instance)
    {
        modbus_global_error = true;
        return MODBUS_ERROR_OTHER;
    }

    uint16_t index = ((uint16_t)modbusMasterGetRequest(master)[8] << 8) | modbusMasterGetRequest(master)[9];
    /*ESP_LOGI(TAG_M, "index %u", index);
    ESP_LOGI(TAG_M, "args->index %u", args->index);
    ESP_LOGI(TAG_M, "value %u", args->value);
    ESP_LOGI(TAG_M, "-----------------");*/

    // shift first register to 0
    instance->read_buffer[args->index - index] = args->value;
    // Always return MODBUS_OK
    return MODBUS_OK;
}

/*
    Exception callback for printing out exceptions on master side
*/
ModbusError masterExceptionCallback_tcp(const ModbusMaster *master, uint8_t address, uint8_t function, ModbusExceptionCode code)
{
    ESP_LOGE(TAG_M, "Received slave %d exception %d (function %d)\n", address, (code), function);
    modbus_global_error = true;

    ModbusTCPInstance *instance = (ModbusTCPInstance *)modbusMasterGetUserPointer(master);
    if (instance)
    {
        instance->lastError = code;
        ESP_LOGE(TAG_M, "Device %d received slave %d exception %d (function %d)",
                 instance->device_id, address, (code), function);
    }
    else
    {
        ESP_LOGE(TAG_M, "Received slave %d exception %d (function %d) - instance unknown",
                 address, (code), function);
    }

    // Always return MODBUS_OK
    return MODBUS_OK;
}

ModbusError Trackle_Modbus_execute_command_tcp(TrackleModbusFunction function, uint16_t address, uint16_t start, uint16_t size, void *value, int *sock, int device_id)
{
    // Trova l'istanza per questo device
    ModbusTCPInstance *instance = find_instance(device_id);
    if (!instance)
    {
        ESP_LOGE(TAG_M, "Istanza non trovata per device %d", device_id);
        return MODBUS_ERROR_OTHER;
    }

    // Validazione parametri
    if (!sock)
    {
        ESP_LOGE(TAG_M, "Device %d: parametri non validi", device_id);
        return MODBUS_ERROR_OTHER;
    }

    ModbusErrorInfo err;
    instance->lastError = MODBUS_EXCEP_NONE;

    // fix size > max size
    if (size > MODBUS_MAX_PACKET_SIZE_TCP)
    {
        size = MODBUS_MAX_PACKET_SIZE_TCP;
    }

    // Incrementa transaction_id per questa istanza
    instance->transaction_id++;
    if (instance->transaction_id > (device_id * 1000 + 999))
    {
        instance->transaction_id = device_id * 1000 + 1; // Wrap around
    }

    switch (function)
    {
    case T_READ_COIL_STATUS:
        err = modbusBuildRequest01TCP(&instance->master, instance->transaction_id, address, start, size);
        break;

    case T_READ_INPUT_STATUS:
        err = modbusBuildRequest02TCP(&instance->master, instance->transaction_id, address, start, size);
        break;

    case T_READ_HOLDING_REGISTERS:
        err = modbusBuildRequest03TCP(&instance->master, instance->transaction_id, address, start, size);
        break;

    case T_READ_INPUT_REGISTERS:
        err = modbusBuildRequest04TCP(&instance->master, instance->transaction_id, address, start, size);
        break;

    case T_WRITE_SINGLE_COIL:
        size = 1;
        err = modbusBuildRequest05TCP(&instance->master, instance->transaction_id, address, start, ((uint8_t *)value)[0]);
        break;

    case T_WRITE_SINGLE_REGISTER:
        size = 1;
        err = modbusBuildRequest06TCP(&instance->master, instance->transaction_id, address, start, ((uint16_t *)value)[0]);
        break;

    case T_WRITE_MULTIPLE_COILS:
        err = modbusBuildRequest15TCP(&instance->master, instance->transaction_id, address, start, size, (uint8_t *)value);
        break;

    case T_WRITE_MULTIPLE_REGISTERS:
        err = modbusBuildRequest16TCP(&instance->master, instance->transaction_id, address, start, size, (uint16_t *)value);
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

    ESP_LOG_BUFFER_HEX_LEVEL(TAG_M, modbusMasterGetRequest(&instance->master), modbusMasterGetRequestLength(&instance->master), ESP_LOG_DEBUG);
    int len = hal_socket_write(*sock, modbusMasterGetRequest(&instance->master), modbusMasterGetRequestLength(&instance->master));

    // error writing to serial
    if (len < 0)
    {
        hal_socket_close(sock);
        return MODBUS_ERROR_LENGTH;
    }
    // all ok
    return MODBUS_ERROR_OK;
}

int Trackle_Modbus_read_socket(TrackleModbusFunction function, uint16_t size, void *value, int sock, int device_id)
{

    ModbusTCPInstance *instance = find_instance(device_id);
    if (!instance)
    {
        ESP_LOGE(TAG_M, "Istanza non trovata per device %d", device_id);
        return -1;
    }

    if (!value || sock <= 0)
    {
        ESP_LOGE(TAG_M, "Device %d: parametri non validi", device_id);
        return -1;
    }

    // reset read buffer
    memset(instance->uart_read_buffer, 0, sizeof(instance->uart_read_buffer));
    memset(instance->read_buffer, 0, sizeof(instance->read_buffer));

    int len = recv(sock, instance->uart_read_buffer, sizeof(instance->uart_read_buffer), 0);

    if (len > 0)
    {
        ESP_LOGD(TAG_M, "Ricevuti %d byte da socket %d", len, sock);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG_M, instance->uart_read_buffer, len, ESP_LOG_DEBUG);

        // Reset error prima del parsing
        instance->lastError = MODBUS_EXCEP_NONE;
        modbus_global_error = false;

        ModbusErrorInfo err = modbusParseResponseTCP(&instance->master,
                                                     modbusMasterGetRequest(&instance->master),
                                                     modbusMasterGetRequestLength(&instance->master),
                                                     instance->uart_read_buffer, len);

        // Controlla prima se c'è stata un'eccezione dal callback
        if (modbus_global_error)
        {
            ESP_LOGE(TAG_M, "Device %d: global_error %d", device_id, modbus_global_error);
            return -10;
        }
        else if (instance->lastError > 0)
        {
            ESP_LOGE(TAG_M, "Device %d: Modbus exception %d", device_id, instance->lastError);
            return -instance->lastError;
        }

        if (modbusIsOk(err))
        {
            // Aggiorna i valori solo per funzioni di lettura
            if (function == T_READ_COIL_STATUS || function == T_READ_INPUT_STATUS)
            {
                ESP_LOGD(TAG_M, "Updating coil/input values");
                for (int i = 0; i < size; i++)
                {
                    int byte_index = i / 8;
                    int bit_index = i % 8;
                    if (instance->read_buffer[i])
                    {
                        ((uint8_t *)value)[byte_index] |= (1 << bit_index);
                    }
                }
            }
            else if (function == T_READ_HOLDING_REGISTERS || function == T_READ_INPUT_REGISTERS)
            {
                ESP_LOGD(TAG_M, "Updating register values");
                for (int i = 0; i < size; i++)
                {
                    ((uint16_t *)value)[i] = instance->read_buffer[i];
                    ESP_LOGD(TAG_M, "Device %d Register[%d] = %d", device_id, i, instance->read_buffer[i]);
                }
            }

            return 1; // Successo
        }
        else
        {
            ESP_LOGW(TAG_M, "Device %d: Parse error - source:%d error:%d",
                     device_id, err.source, err.error);

            // Errore di parsing, magari letto transaction errato
            // e il dato corretto arriva sucessivamente
            return 0;
        }
    }
    else if (len == 0)
    {
        ESP_LOGE(TAG_M, "Device %d: Peer closed socket %d", device_id, sock);
        return -3; // Connessione chiusa
    }
    else // len < 0
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0; // Nessun dato disponibile
        }
        else
        {
            ESP_LOGE(TAG_M, "Device %d: Socket error %d", device_id, errno);
            return -4; // Errore socket
        }
    }
}

// Modbus master initialization
esp_err_t Trackle_Modbus_create_instance_tcp(int device_id)
{
    // Inizializza mutex se non esiste
    if (instances_mutex == NULL)
    {
        instances_mutex = xSemaphoreCreateMutex();
        if (instances_mutex == NULL)
        {
            ESP_LOGE(TAG_M, "Errore creazione mutex instances");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG_M, "Mutex instances creato");
    }

    // Verifica se l'istanza esiste già
    if (find_instance(device_id) != NULL)
    {
        ESP_LOGW(TAG_M, "Istanza Modbus già esistente per device %d", device_id);
        return ESP_OK;
    }

    // Alloca nuova istanza
    ModbusTCPInstance *instance = malloc(sizeof(ModbusTCPInstance));
    if (!instance)
    {
        ESP_LOGE(TAG_M, "Errore allocazione memoria per device %d", device_id);
        return ESP_FAIL;
    }

    // Inizializza l'istanza
    memset(instance, 0, sizeof(ModbusTCPInstance));
    instance->device_id = device_id;
    instance->transaction_id = device_id * 1000; // Offset per evitare conflitti
    instance->lastError = MODBUS_EXCEP_NONE;

    ESP_LOGI(TAG_M, "Inizializzazione master Modbus per device %d...", device_id);

    // Inizializza il master Modbus
    ModbusErrorInfo err = modbusMasterInit(&instance->master,
                                           dataCallback_tcp,
                                           masterExceptionCallback_tcp,
                                           modbusDefaultAllocator,
                                           modbusMasterDefaultFunctions,
                                           modbusMasterDefaultFunctionCount);

    if (!modbusIsOk(err))
    {
        ESP_LOGE(TAG_M, "modbusMasterInit() failed per device %d: errore %d.%d",
                 device_id, err.source, err.error);
        free(instance);
        return ESP_FAIL;
    }

    modbusMasterSetUserPointer(&instance->master, instance);
    ESP_LOGI(TAG_M, "UserPointer impostato per device %d: %p", device_id, instance);

    // Aggiungi alla hash table
    if (xSemaphoreTake(instances_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        HASH_ADD_INT(tcp_instances, device_id, instance);
        xSemaphoreGive(instances_mutex);
        ESP_LOGI(TAG_M, "Istanza aggiunta alla hash table per device %d", device_id);
    }
    else
    {
        ESP_LOGE(TAG_M, "Errore acquisizione mutex per device %d", device_id);
        free(instance);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_M, "Istanza Modbus TCP inizializzata con successo per device %d", device_id);
    return ESP_OK;
}

esp_err_t Trackle_Modbus_delete_instance_tcp(int device_id)
{
    if (!instances_mutex)
    {
        ESP_LOGW(TAG_M, "Sistema Modbus non inizializzato per cleanup device %d", device_id);
        return ESP_OK; // Niente da fare se non inizializzato
    }

    ModbusTCPInstance *instance = NULL;

    // Rimuovi dalla hash table
    if (xSemaphoreTake(instances_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        HASH_FIND_INT(tcp_instances, &device_id, instance);
        if (instance)
        {
            HASH_DEL(tcp_instances, instance);
            ESP_LOGI(TAG_M, "Istanza rimossa dalla hash table per device %d", device_id);
        }
        xSemaphoreGive(instances_mutex);
    }
    else
    {
        ESP_LOGE(TAG_M, "Errore acquisizione mutex per cleanup device %d", device_id);
        return ESP_FAIL;
    }

    // Libera la memoria
    if (instance)
    {
        ESP_LOGI(TAG_M, "Cleanup istanza Modbus TCP per device %d", device_id);

        // Cleanup del master Modbus (se necessario)
        modbusMasterDestroy(&instance->master); // Se disponibile nella libreria

        // Libera la memoria dell'istanza
        free(instance);

        ESP_LOGI(TAG_M, "Istanza liberata per device %d", device_id);
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG_M, "Istanza non trovata per cleanup device %d", device_id);
        return ESP_ERR_NOT_FOUND;
    }
}
