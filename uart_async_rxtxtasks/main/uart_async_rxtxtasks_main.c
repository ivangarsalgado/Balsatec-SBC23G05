#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "driver/adc.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_timer.h"
#include <time.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_partition.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "ds18b20.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

static const char *TAG = "BALSATEC";
#define EXAMPLE_ESP_WIFI_SSID      "Sorivanyo"
#define EXAMPLE_ESP_WIFI_PASS      "30160127"
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#define TEMP_BUS 17

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_10)
#define RXD_PIN (GPIO_NUM_9)

uint8_t pantalla[] = {0x04,0x00,0x04};

DeviceAddress tempSensors[2];

static EventGroupHandle_t s_wifi_event_group;

#define  PIN_FOTOTRANSISTOR GPIO_NUM_34
#define PIN_SALIDA GPIO_NUM_4

#define ADC_CHANNEL ADC1_CHANNEL_6
#define ADC_RESOLUTION ADC_WIDTH_BIT_12

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define TDS_ANALOG_GPIO     ADC1_CHANNEL_6
#define TDS_STABILISATION_DELAY     10  
#define TDS_NUM_SAMPLES             10  
#define TDS_SAMPLE_PERIOD           20  
#define TDS_TEMPERATURE             18.0  
#define TDS_VREF                    1.18  
#define RES2 820.0
#define ECREF 200.0

float _rawEC = 0;
float _kvalue = 1.0;
float _kvalueHigh = 1.5;
float _kvalueLow = 0.5;
float _ecvalue = 0;

float sampleDelay = (TDS_SAMPLE_PERIOD / TDS_NUM_SAMPLES) * 1000;


static int s_retry_num = 0;

float version = 1.7; //Version del codigo.

uint8_t temp_pantalla[50][6] = {
    {0x01, 0x08, 0x00, 0x00, 0x00, 0x09},  // Conjunto 1
    {0x01, 0x08, 0x00, 0x00, 0x01, 0x08},  // Conjunto 2
    {0x01, 0x08, 0x00, 0x00, 0x02, 0x0B},  // Conjunto 3
    {0x01, 0x08, 0x00, 0x00, 0x03, 0x0A},  // Conjunto 4
    {0x01, 0x08, 0x00, 0x00, 0x04, 0x0D},  // Conjunto 5
    {0x01, 0x08, 0x00, 0x00, 0x05, 0x0C},  // Conjunto 6
    {0x01, 0x08, 0x00, 0x00, 0x06, 0x0F},  // Conjunto 7
    {0x01, 0x08, 0x00, 0x00, 0x07, 0x0E},  // Conjunto 8
    {0x01, 0x08, 0x00, 0x00, 0x08, 0x01},  // Conjunto 9
    {0x01, 0x08, 0x00, 0x00, 0x09, 0x00},  // Conjunto 10
    {0x01, 0x08, 0x00, 0x00, 0x0A, 0x03},  // Conjunto 11
    {0x01, 0x08, 0x00, 0x00, 0x0B, 0x02},  // Conjunto 12
    {0x01, 0x08, 0x00, 0x00, 0x0C, 0x05},  // Conjunto 13
    {0x01, 0x08, 0x00, 0x00, 0x0D, 0x04},  // Conjunto 14
    {0x01, 0x08, 0x00, 0x00, 0x0E, 0x07},  // Conjunto 15
    {0x01, 0x08, 0x00, 0x00, 0x0F, 0x06},  // Conjunto 16
    {0x01, 0x08, 0x00, 0x00, 0x10, 0x19},  // Conjunto 17
    {0x01, 0x08, 0x00, 0x00, 0x11, 0x18},  // Conjunto 18
    {0x01, 0x08, 0x00, 0x00, 0x12, 0x1B},  // Conjunto 19
    {0x01, 0x08, 0x00, 0x00, 0x13, 0x1A},  // Conjunto 20
    {0x01, 0x08, 0x00, 0x00, 0x14, 0x1D},  // Conjunto 21
    {0x01, 0x08, 0x00, 0x00, 0x15, 0x1C},  // Conjunto 22
    {0x01, 0x08, 0x00, 0x00, 0x16, 0x1F},  // Conjunto 23
    {0x01, 0x08, 0x00, 0x00, 0x17, 0x1E},  // Conjunto 24
    {0x01, 0x08, 0x00, 0x00, 0x18, 0x11},  // Conjunto 25
    {0x01, 0x08, 0x00, 0x00, 0x19, 0x10},  // Conjunto 26
    {0x01, 0x08, 0x00, 0x00, 0x1A, 0x13},  // Conjunto 27
    {0x01, 0x08, 0x00, 0x00, 0x1B, 0x12},  // Conjunto 28
    {0x01, 0x08, 0x00, 0x00, 0x1C, 0x15},  // Conjunto 29
    {0x01, 0x08, 0x00, 0x00, 0x1D, 0x14},  // Conjunto 30
    {0x01, 0x08, 0x00, 0x00, 0x1E, 0x17},  // Conjunto 31
    {0x01, 0x08, 0x00, 0x00, 0x1F, 0x16},  // Conjunto 32
    {0x01, 0x08, 0x00, 0x00, 0x20, 0x29},  // Conjunto 33
    {0x01, 0x08, 0x00, 0x00, 0x21, 0x28},  // Conjunto 34
    {0x01, 0x08, 0x00, 0x00, 0x22, 0x2B},  // Conjunto 35
    {0x01, 0x08, 0x00, 0x00, 0x23, 0x2A},  // Conjunto 36
    {0x01, 0x08, 0x00, 0x00, 0x24, 0x2D},  // Conjunto 37
    {0x01, 0x08, 0x00, 0x00, 0x25, 0x2C},  // Conjunto 38
    {0x01, 0x08, 0x00, 0x00, 0x26, 0x2F},  // Conjunto 39
    {0x01, 0x08, 0x00, 0x00, 0x27, 0x2E},  // Conjunto 40
    {0x01, 0x08, 0x00, 0x00, 0x28, 0x21},  // Conjunto 41
    {0x01, 0x08, 0x00, 0x00, 0x29, 0x20},  // Conjunto 42
    {0x01, 0x08, 0x00, 0x00, 0x2A, 0x23},  // Conjunto 43
    {0x01, 0x08, 0x00, 0x00, 0x2B, 0x22},  // Conjunto 44
    {0x01, 0x08, 0x00, 0x00, 0x2C, 0x25},  // Conjunto 45
    {0x01, 0x08, 0x00, 0x00, 0x2D, 0x24},  // Conjunto 46
    {0x01, 0x08, 0x00, 0x00, 0x2E, 0x27},  // Conjunto 47
    {0x01, 0x08, 0x00, 0x00, 0x2F, 0x26},  // Conjunto 48
    {0x01, 0x08, 0x00, 0x00, 0x30, 0x39},  // Conjunto 49
    {0x01, 0x08, 0x00, 0x00, 0x31, 0x38}   // Conjunto 50
};


uint8_t k_pantalla[50][6] = {
    {0x01, 0x08, 0x02, 0x00, 0x00, 0x0B},  // Conjunto 1
    {0x01, 0x08, 0x02, 0x00, 0x01, 0x0A},  // Conjunto 2
    {0x01, 0x08, 0x02, 0x00, 0x02, 0x09},  // Conjunto 3
    {0x01, 0x08, 0x02, 0x00, 0x03, 0x08},  // Conjunto 4
    {0x01, 0x08, 0x02, 0x00, 0x04, 0x0F},  // Conjunto 5
    {0x01, 0x08, 0x02, 0x00, 0x05, 0x0E},  // Conjunto 6
    {0x01, 0x08, 0x02, 0x00, 0x06, 0x0D},  // Conjunto 7
    {0x01, 0x08, 0x02, 0x00, 0x07, 0x0C},  // Conjunto 8
    {0x01, 0x08, 0x02, 0x00, 0x08, 0x03},  // Conjunto 9
    {0x01, 0x08, 0x02, 0x00, 0x09, 0x02},  // Conjunto 10
    {0x01, 0x08, 0x02, 0x00, 0x0A, 0x01},  // Conjunto 11
    {0x01, 0x08, 0x02, 0x00, 0x0B, 0x00},  // Conjunto 12
    {0x01, 0x08, 0x02, 0x00, 0x0C, 0x07},  // Conjunto 13
    {0x01, 0x08, 0x02, 0x00, 0x0D, 0x06},  // Conjunto 14
    {0x01, 0x08, 0x02, 0x00, 0x0E, 0x05},  // Conjunto 15
    {0x01, 0x08, 0x02, 0x00, 0x0F, 0x04},  // Conjunto 16
    {0x01, 0x08, 0x02, 0x00, 0x10, 0x1B},  // Conjunto 17
    {0x01, 0x08, 0x02, 0x00, 0x11, 0x1A},  // Conjunto 18
    {0x01, 0x08, 0x02, 0x00, 0x12, 0x19},  // Conjunto 19
    {0x01, 0x08, 0x02, 0x00, 0x13, 0x18},  // Conjunto 20
    {0x01, 0x08, 0x02, 0x00, 0x14, 0x1F},  // Conjunto 21
    {0x01, 0x08, 0x02, 0x00, 0x15, 0x1E},  // Conjunto 22
    {0x01, 0x08, 0x02, 0x00, 0x16, 0x1D},  // Conjunto 23
    {0x01, 0x08, 0x02, 0x00, 0x17, 0x1C},  // Conjunto 24
    {0x01, 0x08, 0x02, 0x00, 0x18, 0x13},  // Conjunto 25
    {0x01, 0x08, 0x02, 0x00, 0x19, 0x12},  // Conjunto 26
    {0x01, 0x08, 0x02, 0x00, 0x1A, 0x11},  // Conjunto 27
    {0x01, 0x08, 0x02, 0x00, 0x1B, 0x10},  // Conjunto 28
    {0x01, 0x08, 0x02, 0x00, 0x1C, 0x17},  // Conjunto 29
    {0x01, 0x08, 0x02, 0x00, 0x1D, 0x16},  // Conjunto 30
    {0x01, 0x08, 0x02, 0x00, 0x1E, 0x15},  // Conjunto 31
    {0x01, 0x08, 0x02, 0x00, 0x1F, 0x14},  // Conjunto 32
    {0x01, 0x08, 0x02, 0x00, 0x20, 0x2B},  // Conjunto 33
    {0x01, 0x08, 0x02, 0x00, 0x21, 0x2A},  // Conjunto 34
    {0x01, 0x08, 0x02, 0x00, 0x22, 0x29},  // Conjunto 35
    {0x01, 0x08, 0x02, 0x00, 0x23, 0x28},  // Conjunto 36
    {0x01, 0x08, 0x02, 0x00, 0x24, 0x2F},  // Conjunto 37
    {0x01, 0x08, 0x02, 0x00, 0x25, 0x2E},  // Conjunto 38
    {0x01, 0x08, 0x02, 0x00, 0x26, 0x2D},  // Conjunto 39
    {0x01, 0x08, 0x02, 0x00, 0x27, 0x2C},  // Conjunto 40
    {0x01, 0x08, 0x02, 0x00, 0x28, 0x23},  // Conjunto 41
    {0x01, 0x08, 0x02, 0x00, 0x29, 0x22},  // Conjunto 42
    {0x01, 0x08, 0x02, 0x00, 0x2A, 0x21},  // Conjunto 43
    {0x01, 0x08, 0x02, 0x00, 0x2B, 0x20},  // Conjunto 44
    {0x01, 0x08, 0x02, 0x00, 0x2C, 0x27},  // Conjunto 45
    {0x01, 0x08, 0x02, 0x00, 0x2D, 0x26},  // Conjunto 46
    {0x01, 0x08, 0x02, 0x00, 0x2E, 0x25},  // Conjunto 47
    {0x01, 0x08, 0x02, 0x00, 0x2F, 0x24},  // Conjunto 48
    {0x01, 0x08, 0x02, 0x00, 0x30, 0x3B},  // Conjunto 49
    {0x01, 0x08, 0x02, 0x00, 0x31, 0x3A}   // Conjunto 50
};


uint8_t ppm_pantallas[100][6] = {
    {0x01, 0x0B, 0x00, 0x00, 0x00, 0x0A},  // Conjunto 1
    {0x01, 0x0B, 0x00, 0x00, 0x01, 0x0B},  // Conjunto 2
    {0x01, 0x0B, 0x00, 0x00, 0x02, 0x08},  // Conjunto 3
    {0x01, 0x0B, 0x00, 0x00, 0x03, 0x09},  // Conjunto 4
    {0x01, 0x0B, 0x00, 0x00, 0x04, 0x0E},  // Conjunto 5
    {0x01, 0x0B, 0x00, 0x00, 0x05, 0x0F},  // Conjunto 6
    {0x01, 0x0B, 0x00, 0x00, 0x06, 0x0C},  // Conjunto 7
    {0x01, 0x0B, 0x00, 0x00, 0x07, 0x0D},  // Conjunto 8
    {0x01, 0x0B, 0x00, 0x00, 0x08, 0x02},  // Conjunto 9
    {0x01, 0x0B, 0x00, 0x00, 0x09, 0x03},  // Conjunto 10
    {0x01, 0x0B, 0x00, 0x00, 0x0A, 0x00},  // Conjunto 11
    {0x01, 0x0B, 0x00, 0x00, 0x0B, 0x01},  // Conjunto 12
    {0x01, 0x0B, 0x00, 0x00, 0x0C, 0x06},  // Conjunto 13
    {0x01, 0x0B, 0x00, 0x00, 0x0D, 0x07},  // Conjunto 14
    {0x01, 0x0B, 0x00, 0x00, 0x0E, 0x04},  // Conjunto 15
    {0x01, 0x0B, 0x00, 0x00, 0x0F, 0x05},  // Conjunto 16
    {0x01, 0x0B, 0x00, 0x00, 0x10, 0x1A},  // Conjunto 17
    {0x01, 0x0B, 0x00, 0x00, 0x11, 0x1B},  // Conjunto 18
    {0x01, 0x0B, 0x00, 0x00, 0x12, 0x18},  // Conjunto 19
    {0x01, 0x0B, 0x00, 0x00, 0x13, 0x19},  // Conjunto 20
    {0x01, 0x0B, 0x00, 0x00, 0x14, 0x1E},  // Conjunto 21
    {0x01, 0x0B, 0x00, 0x00, 0x15, 0x1F},  // Conjunto 22
    {0x01, 0x0B, 0x00, 0x00, 0x16, 0x1C},  // Conjunto 23
    {0x01, 0x0B, 0x00, 0x00, 0x17, 0x1D},  // Conjunto 24
    {0x01, 0x0B, 0x00, 0x00, 0x18, 0x12},  // Conjunto 25
    {0x01, 0x0B, 0x00, 0x00, 0x19, 0x13},  // Conjunto 26
    {0x01, 0x0B, 0x00, 0x00, 0x1A, 0x10},  // Conjunto 27
    {0x01, 0x0B, 0x00, 0x00, 0x1B, 0x11},  // Conjunto 28
    {0x01, 0x0B, 0x00, 0x00, 0x1C, 0x16},  // Conjunto 29
    {0x01, 0x0B, 0x00, 0x00, 0x1D, 0x17},  // Conjunto 30
    {0x01, 0x0B, 0x00, 0x00, 0x1E, 0x14},  // Conjunto 31
    {0x01, 0x0B, 0x00, 0x00, 0x1F, 0x15},  // Conjunto 32
    {0x01, 0x0B, 0x00, 0x00, 0x20, 0x2A},  // Conjunto 33
    {0x01, 0x0B, 0x00, 0x00, 0x21, 0x2B},  // Conjunto 34
    {0x01, 0x0B, 0x00, 0x00, 0x22, 0x28},  // Conjunto 35
    {0x01, 0x0B, 0x00, 0x00, 0x23, 0x29},  // Conjunto 36
    {0x01, 0x0B, 0x00, 0x00, 0x24, 0x2E},  // Conjunto 37
    {0x01, 0x0B, 0x00, 0x00, 0x25, 0x2F},  // Conjunto 38
    {0x01, 0x0B, 0x00, 0x00, 0x26, 0x2C},  // Conjunto 39
    {0x01, 0x0B, 0x00, 0x00, 0x27, 0x2D},  // Conjunto 40
    {0x01, 0x0B, 0x00, 0x00, 0x28, 0x22},  // Conjunto 41
    {0x01, 0x0B, 0x00, 0x00, 0x29, 0x23},  // Conjunto 42
    {0x01, 0x0B, 0x00, 0x00, 0x2A, 0x20},  // Conjunto 43
    {0x01, 0x0B, 0x00, 0x00, 0x2B, 0x21},  // Conjunto 44
    {0x01, 0x0B, 0x00, 0x00, 0x2C, 0x26},  // Conjunto 45
    {0x01, 0x0B, 0x00, 0x00, 0x2D, 0x27},  // Conjunto 46
    {0x01, 0x0B, 0x00, 0x00, 0x2E, 0x24},  // Conjunto 47
    {0x01, 0x0B, 0x00, 0x00, 0x2F, 0x25},  // Conjunto 48
    {0x01, 0x0B, 0x00, 0x00, 0x30, 0x3A},  // Conjunto 49
    {0x01, 0x0B, 0x00, 0x00, 0x31, 0x3B},  // Conjunto 50
    {0x01, 0x0B, 0x00, 0x00, 0x32, 0x38},  // Conjunto 51
    {0x01, 0x0B, 0x00, 0x00, 0x33, 0x39},  // Conjunto 52
    {0x01, 0x0B, 0x00, 0x00, 0x34, 0x3E},  // Conjunto 53
    {0x01, 0x0B, 0x00, 0x00, 0x35, 0x3F},  // Conjunto 54
    {0x01, 0x0B, 0x00, 0x00, 0x36, 0x3C},  // Conjunto 55
    {0x01, 0x0B, 0x00, 0x00, 0x37, 0x3D},  // Conjunto 56
    {0x01, 0x0B, 0x00, 0x00, 0x38, 0x32},  // Conjunto 57
    {0x01, 0x0B, 0x00, 0x00, 0x39, 0x33},  // Conjunto 58
    {0x01, 0x0B, 0x00, 0x00, 0x3A, 0x30},  // Conjunto 59
    {0x01, 0x0B, 0x00, 0x00, 0x3B, 0x31},  // Conjunto 60
    {0x01, 0x0B, 0x00, 0x00, 0x3C, 0x36},  // Conjunto 61
    {0x01, 0x0B, 0x00, 0x00, 0x3D, 0x37},  // Conjunto 62
    {0x01, 0x0B, 0x00, 0x00, 0x3E, 0x34},  // Conjunto 63
    {0x01, 0x0B, 0x00, 0x00, 0x3F, 0x35},  // Conjunto 64
    {0x01, 0x0B, 0x00, 0x00, 0x40, 0x4A},  // Conjunto 65
    {0x01, 0x0B, 0x00, 0x00, 0x41, 0x4B},  // Conjunto 66
    {0x01, 0x0B, 0x00, 0x00, 0x42, 0x48},  // Conjunto 67
    {0x01, 0x0B, 0x00, 0x00, 0x43, 0x49},  // Conjunto 68
    {0x01, 0x0B, 0x00, 0x00, 0x44, 0x4E},  // Conjunto 69
    {0x01, 0x0B, 0x00, 0x00, 0x45, 0x4F},  // Conjunto 70
    {0x01, 0x0B, 0x00, 0x00, 0x46, 0x4C},  // Conjunto 71
    {0x01, 0x0B, 0x00, 0x00, 0x47, 0x4D},  // Conjunto 72
    {0x01, 0x0B, 0x00, 0x00, 0x48, 0x42},  // Conjunto 73
    {0x01, 0x0B, 0x00, 0x00, 0x49, 0x43},  // Conjunto 74
    {0x01, 0x0B, 0x00, 0x00, 0x4A, 0x40},  // Conjunto 75
    {0x01, 0x0B, 0x00, 0x00, 0x4B, 0x41},  // Conjunto 76
    {0x01, 0x0B, 0x00, 0x00, 0x4C, 0x46},  // Conjunto 77
    {0x01, 0x0B, 0x00, 0x00, 0x4D, 0x47},  // Conjunto 78
    {0x01, 0x0B, 0x00, 0x00, 0x4E, 0x44},  // Conjunto 79
    {0x01, 0x0B, 0x00, 0x00, 0x4F, 0x45},  // Conjunto 80
    {0x01, 0x0B, 0x00, 0x00, 0x50, 0x5A},  // Conjunto 81
    {0x01, 0x0B, 0x00, 0x00, 0x51, 0x5B},  // Conjunto 82
    {0x01, 0x0B, 0x00, 0x00, 0x52, 0x58},  // Conjunto 83
    {0x01, 0x0B, 0x00, 0x00, 0x53, 0x59},  // Conjunto 84
    {0x01, 0x0B, 0x00, 0x00, 0x54, 0x5E},  // Conjunto 85
    {0x01, 0x0B, 0x00, 0x00, 0x55, 0x5F},  // Conjunto 86
    {0x01, 0x0B, 0x00, 0x00, 0x56, 0x5C},  // Conjunto 87
    {0x01, 0x0B, 0x00, 0x00, 0x57, 0x5D},  // Conjunto 88
    {0x01, 0x0B, 0x00, 0x00, 0x58, 0x52},  // Conjunto 89
    {0x01, 0x0B, 0x00, 0x00, 0x59, 0x53},  // Conjunto 90
    {0x01, 0x0B, 0x00, 0x00, 0x5A, 0x50},  // Conjunto 91
    {0x01, 0x0B, 0x00, 0x00, 0x5B, 0x51},  // Conjunto 92
    {0x01, 0x0B, 0x00, 0x00, 0x5C, 0x56},  // Conjunto 93
    {0x01, 0x0B, 0x00, 0x00, 0x5D, 0x57},  // Conjunto 94
    {0x01, 0x0B, 0x00, 0x00, 0x5E, 0x54},  // Conjunto 95
    {0x01, 0x0B, 0x00, 0x00, 0x5F, 0x55},  // Conjunto 96
    {0x01, 0x0B, 0x00, 0x00, 0x60, 0x6A},  // Conjunto 97
    {0x01, 0x0B, 0x00, 0x00, 0x61, 0x6B},  // Conjunto 98
    {0x01, 0x0B, 0x00, 0x00, 0x62, 0x68},  // Conjunto 99
};

uint8_t imprimir_temp[] = {0x01, 0x08, 0x00, 0x00, 0x12, 0x1B};
uint8_t imprimir_k[] = {0x01,0x08,0x02,0x00,0x00,0x0B};
uint8_t imprimir_ppm[] = {0x01,0x0B,0x00,0x00,0x00,0x0A};


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void backtofactory()
{
    esp_partition_iterator_t pi ; // Iterator for find
    const esp_partition_t* factory ; // Factory partition
    esp_err_t err ;
 pi = esp_partition_find ( ESP_PARTITION_TYPE_APP, // Get partition
                            ESP_PARTITION_SUBTYPE_APP_FACTORY, // factory partition
                            "factory" ) ;
    if ( pi == NULL ) // Check result
    {
            ESP_LOGE ( TAG, "Failed to find factory partition" ) ;
    }
 else
 {
    factory = esp_partition_get ( pi ) ; // Get partition struct
    esp_partition_iterator_release ( pi ) ; // Release the iterator
    err = esp_ota_set_boot_partition ( factory ) ; // Set partition for

 if ( err != ESP_OK ) // Check error
    {
    ESP_LOGE ( TAG, "Failed to set boot partition" ) ;
    }
else
{
 esp_restart() ; // Restart ESP
 }
 }
}





void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{

    size_t len = 6;
    const int txBytes = uart_write_bytes(UART_NUM_0, (const char*)imprimir_temp, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_mqtt_client_handle_t mqtt_client_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://demo.thingsboard.io",
        .event_handle = mqtt_event_handler,
        .port = 1883,
        .username = "RyjPuWq83zjR2pSWXBqH", //token
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    return client;
}

void mqtt_send_data(esp_mqtt_client_handle_t client, const char *topic, cJSON *data)
{
    char *post_data = cJSON_PrintUnformatted(data);
    esp_mqtt_client_publish(client, topic, post_data, 0, 1, 0);
    free(post_data);
}


float convert_to_ppm(int analogReading, float temperature){
    ESP_LOGI(TAG, "Converting an analog value to a TDS PPM value.");
    //https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
    float adcCompensation = 1 + (1/3.9); // 1/3.9 (11dB) attenuation.
    float vPerDiv = (TDS_VREF / 4096) * adcCompensation; // Calculate the volts per division using the VREF taking account of the chosen attenuation value.
    float averageVoltage = analogReading * vPerDiv; // Convert the ADC reading into volts
    float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;  //temperature compensation
    float tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

    ESP_LOGI(TAG, "Volts per division = %f", vPerDiv);
    ESP_LOGI(TAG, "Average Voltage = %f", averageVoltage);
    ESP_LOGI(TAG, "Temperature (currently fixed, we should measure this) = %f", temperature);
    ESP_LOGI(TAG, "Compensation Coefficient = %f", compensationCoefficient);
    ESP_LOGI(TAG, "Compensation Voltge = %f", compensationVolatge);
    ESP_LOGI(TAG, "tdsValue = %f ppm", tdsValue);
    return tdsValue;
}


float convert_to_ec(float voltage, float temperature){
    float value = 0, valueTemp = 0;
    value = voltage;                 
    return value;
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SBC",
            .password = "30160127",
	     .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}



void app_main(void)
{

    init_uart();
    // vTaskDelay(500);
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-5, NULL);
    // xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-5, NULL);
    // vTaskDelay(2500);
    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_NUM_26, 0);
    vTaskDelay(100);
    gpio_set_level(GPIO_NUM_26, 1);

    ds18b20_init(TEMP_BUS);
    ds18b20_setResolution(tempSensors,2,10);
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    example_connect();


    esp_mqtt_client_handle_t client = mqtt_client_init();
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6 ,ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7 ,ADC_ATTEN_DB_11);

    int raw_ppm,raw_ec,sum_raw_ppm;
    float ppm, ec, cTemp;


    cJSON *dataversion = cJSON_CreateObject();
    cJSON_AddNumberToObject(dataversion, "version", version);
    mqtt_send_data(client, "v1/devices/me/telemetry", dataversion);

    const int total_execution_time = 1 * 60 * 1000000;  // 5 minutos en microsegundos
    int elapsed_time = 0;
    
    while(1){
        // Apagar el GPIO 26 (establecerlo en 0, bajo)
        
        printf("Leyendo sensores: \n");
        //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);

        cTemp = ds18b20_get_temp();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        sum_raw_ppm = 0;

        for(int i=0; i<10;i++)
        {
            raw_ppm = adc1_get_raw(ADC1_CHANNEL_6);
            //ppm = convert_to_ppm(raw_ppm,cTemp);
            sum_raw_ppm  += raw_ppm;
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        sum_raw_ppm = sum_raw_ppm / 10;
        ppm = (sum_raw_ppm * 100) / 4095;
        vTaskDelay(500 / portTICK_PERIOD_MS);
        raw_ec = adc1_get_raw(ADC1_CHANNEL_7);
        //ec = convert_to_ec(raw_ec,cTemp);
        ec = raw_ec / 100;
        vTaskDelay(500 / portTICK_PERIOD_MS);
        int tempIndex = ((int)cTemp);
        int ppmIndex = 100 - ppm; //de esta forma calcularemos el inverso. Si hay muchas particulas mandaremos un numero alto. Si es claro el agua un numero bajo.
        int ecIndex = ((int)ec) % 50; //la electroconductividad la dividiremos entre 10 y ya.
        printf("Temperatura: %f\n",cTemp);
        printf("PPM  = %f \n", ppm);
        printf("Electroconductividad  = %f \n", ec);
        cJSON *data = cJSON_CreateObject();
        cJSON_AddNumberToObject(data, "temperature", cTemp);
        mqtt_send_data(client, "v1/devices/me/telemetry", data);
        cJSON *data2 = cJSON_CreateObject();
        cJSON_AddNumberToObject(data2, "ppm", ppm);
        mqtt_send_data(client, "v1/devices/me/telemetry", data2);
        cJSON *data3 = cJSON_CreateObject();
        cJSON_AddNumberToObject(data3, "k", ec);
        mqtt_send_data(client, "v1/devices/me/telemetry", data3);
        
        printf("Escribiendo datos por UART: \n");
        size_t len = 6;
        uart_write_bytes(UART_NUM_1, (const char*)temp_pantalla[tempIndex], sizeof(temp_pantalla[tempIndex]));
        vTaskDelay(500 / portTICK_PERIOD_MS);
        uart_write_bytes(UART_NUM_1, (const char*)ppm_pantallas[ppmIndex], sizeof(ppm_pantallas[ppmIndex]));
        vTaskDelay(500 / portTICK_PERIOD_MS);
        uart_write_bytes(UART_NUM_1, (const char*)k_pantalla[ecIndex], sizeof(k_pantalla[ecIndex]));
        vTaskDelay(500 / portTICK_PERIOD_MS);


        struct timeval now;
        gettimeofday(&now, NULL);

        struct tm timeinfo;
        localtime_r(&now.tv_sec, &timeinfo);
        // // Imprimir la estructura struct tm
        printf("Hour: %d\n", timeinfo.tm_hour);
        printf("Minute: %d\n", timeinfo.tm_min);
        printf("Second: %d\n", timeinfo.tm_sec);

        if (timeinfo.tm_min >= 1 && timeinfo.tm_min <= 2) {
            printf("Back to factory. Ota process: \n");
            vTaskDelay(2 * 60 * 1000 / portTICK_PERIOD_MS);
            backtofactory();
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Actualizar el tiempo transcurrido
        // elapsed_time += 15 * 1000000;  // Asumiendo que el bucle principal dura aproximadamente 15 segundos

        // // Verificar si se alcanzó el tiempo total de ejecución
        // if (elapsed_time >= total_execution_time) {
        //     // Reiniciar el tiempo transcurrido
        //     elapsed_time = 0;

        //     // Configurar temporizador para light sleep (5 minutos)
        //     esp_sleep_enable_timer_wakeup(1 * 60 * 1000000);

        //     // Configurar el modo de light sleep para consumir la menor cantidad de energía posible
        //     esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
        //     esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
        //     esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

        //     // Habilitar el modo de light sleep
        //     esp_light_sleep_start();

        //     // El código que se ejecutará después de despertar
        //     ESP_LOGI(TAG, "El dispositivo se ha despertado. Continuando con la ejecución...");
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // }
    }
}
