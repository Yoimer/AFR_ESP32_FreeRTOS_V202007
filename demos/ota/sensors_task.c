#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "aws_application_version.h"
#include "esp_log.h"
#include "iot_ble_config.h"
#include "driver/uart.h"
#include "minmea.h"
#include "sensors.h"

char version[VERSION_STRING_LENGTH];
char string_latitude[LATITUDE_STRING_LENGTH];
char string_longitude[LONGITUDE_STRING_LENGTH];
extern char sensorsPayload[PAYLOAD_STRING_LENGTH];

#define UART_NUM                UART_NUM_2
#define UART_RX_PIN             33
#define UART_RX_BUF_SIZE        (1024)

static char tag[] = "gps";
float latitude = -200.00;
float longitude = -200.00;

static void uart_setup()
{
    uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM,
            // UART_PIN_NO_CHANGE, 21,
            UART_PIN_NO_CHANGE, UART_RX_PIN,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

char *readLine(uart_port_t uart) {
    static char line[256];
    int size;
    char *ptr = line;
    while(1) {
        size = uart_read_bytes(uart, (unsigned char *)ptr, 1, portMAX_DELAY);
        if (size == 1) {
            if (*ptr == '\n') {
                ptr++;
                *ptr = 0;
                return line;
            }
            ptr++;
        } // End of read a character
    } // End of loop
} // End of readLine

void sensors_task(void *pvParameters)
{

    uart_setup();
    while (1)
    {

        sprintf(string_latitude, "%.6f", latitude);
        sprintf(string_longitude, "%.6f", longitude);
        sprintf(sensorsPayload, "{\"lat\":\"%s\",\"lot\":\"%s\"}", string_latitude, string_longitude);

        char * line = readLine(UART_NUM);
        //printf("line: %s\n", line);
        switch(minmea_sentence_id(line, false)) {
        case MINMEA_SENTENCE_RMC:
            printf("Sentence - MINMEA_SENTENCE_RMC: %s\n", tag);
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc( & frame, line)) {
                latitude = minmea_tocoord( & frame.latitude);
                longitude = minmea_tocoord( & frame.longitude);
                printf("$xxRMC floating point degree coordinates: (%f,%f)\n", latitude, longitude);

            } else {
                printf("$xxRMC sentence is not parsed %s\n", tag);
            }
            break;
        case MINMEA_SENTENCE_GGA:
            printf("MINMEA_SENTENCE_GGA: %s\n", tag);
            break;
        case MINMEA_SENTENCE_GSV:
            printf("MINMEA_SENTENCE_GSV: %s\n", tag);
            break;
        default:
            //ESP_LOGD(tag, "Sentence - other");
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}