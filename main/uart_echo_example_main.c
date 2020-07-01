/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"


#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/queue.h"


///command struct
typedef struct
{
	//uint8_t type;
	uint8_t opcode;
	
	uint8_t board_addr;
	uint8_t lock_addr;//-------
	uint8_t gu_ding;//---
	
	uint8_t bcc;
}command1_struct;

/////start process the data in

command1_struct m_data;

uint8_t ComputXor(uint8_t *InData, uint16_t Len)
{
	uint8_t Sum = 0;
	uint16_t i;
	for(i = 0; i < Len; i++)
	{
		Sum ^= InData[i];	
	}
	return Sum;
}

static const char *TAG = "uart_events";
/**
 * This is an example which echos any data it receives on UART1 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD  (GPIO_NUM_4)
#define ECHO_TEST_RXD  (GPIO_NUM_5)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)


#define ECHO_TEST2_TXD  (GPIO_NUM_17)
#define ECHO_TEST2_RXD  (GPIO_NUM_16)
#define ECHO_TEST2_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST2_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)
uint8_t data_rx[BUF_SIZE] = {0};
int len_rx;

static void echo_task()
{
    uint16_t bl_addr=0;//bianliang
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_config_t uart_config2 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };


        // Set UART log level
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start ttl application test and configure UART2.\r\n");

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);


    uart_param_config(UART_NUM_2, &uart_config2);
    uart_set_pin(UART_NUM_2, ECHO_TEST2_TXD, ECHO_TEST2_RXD, ECHO_TEST2_RTS, ECHO_TEST2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    //uint8_t *data_rx = (uint8_t *) malloc(BUF_SIZE);

    //ESP_LOGI(TAG, "UART1 start recieve loop.\r\n");

    while (1) {
        // Read data from the UART
        len_rx = uart_read_bytes(UART_NUM_1, data_rx, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        
        //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);
        //uart_write_bytes(UART_NUM_2, (const char *) data_rx, len_rx);
        
        if (len_rx > 0) {
            ESP_LOGI(TAG, "Received %u bytes:", len_rx);
            for (int i = 0; i < len_rx; i++) {
                printf("0x%.2X ", (uint8_t)data_rx[i]);
            }
            printf("] \n");
            
            if((0x5A == data_rx[0])
                &&(0xA5 == data_rx[1])
                &&((len_rx-3) == data_rx[2]))
                {
                    switch (data_rx[3])
                    {
                    case 0x82:
                        //uart_write_bytes(UART_NUM_2, (const char *) (data_rx+4), len_rx-4);
                        ESP_LOGI(TAG, "----------------0x82---------------.\r\n");
                        break;

                    case 0x83:
                        if( data_rx[6] == (len_rx-7)/2)
                        {
                            //uart_write_bytes(UART_NUM_2, (const char *) (data_rx+4), len_rx-4);
                            ESP_LOGI(TAG, "----------------0x83---------------.\r\n");
                            bl_addr = (data_rx[4]<<8) + data_rx[5];

                            uint8_t tx_Buffer[256]={0};  
                            uint8_t bcc_temp=0;
                            switch (bl_addr)
                            {
                            case 0x1050:
                                ESP_LOGI(TAG, "----------------phone number---------------.\r\n");
                                memcpy(tx_Buffer,"star",4);
                                tx_Buffer[4]= 0x8A;//m_data.opcode;
                                tx_Buffer[5]= 0x01;//m_data.board_addr;
                                tx_Buffer[6]= 0x02;//m_data.lock_addr;
                                tx_Buffer[7]= 0x11;//guding
                                bcc_temp = ComputXor(tx_Buffer+4,4);
                                tx_Buffer[8]= bcc_temp;
                                memcpy(tx_Buffer+9,"endo",4);
                                
                                tx_Buffer[13]='\0';
                                uart_write_bytes(UART_NUM_2, (const char *) tx_Buffer, 13);
                                break;
                            case 0x1060:
                                ESP_LOGI(TAG, "----------------password---------------.\r\n");

                                memcpy(tx_Buffer,"star",4);
                                tx_Buffer[4]= 0x8A;//m_data.opcode;
                                tx_Buffer[5]= 0x01;//m_data.board_addr;
                                tx_Buffer[6]= 0x01;//m_data.lock_addr;
                                tx_Buffer[7]= 0x11;//guding
                                bcc_temp = ComputXor(tx_Buffer+4,4);
                                tx_Buffer[8]= bcc_temp;
                                memcpy(tx_Buffer+9,"endo",4);
                                
                                tx_Buffer[13]='\0';
                                uart_write_bytes(UART_NUM_2, (const char *) tx_Buffer, 13);
                                break;
                            default:
                                break;
                            }

                        }

                        break;
                
                    default:
                        break;
                    }

                }
        }
    }
}

void app_main()
{
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
}
