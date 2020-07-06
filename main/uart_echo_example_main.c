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

#include "AS608.h" 

#define usart2_baund  57600//串口2波特率，根据指纹模块波特率更改

SysPara AS608Para;//指纹模块AS608参数
u16 ValidN;//模块内有效指纹个数
u8** kbd_tbl;
void Add_FR(void);	//录指纹
void Del_FR(u16 num);	//删除指纹
void press_FR(void);//刷指纹
void ShowErrMessage(u8 ensure);//显示确认码错误信息

u8 zhiwen_num_id;


void delay_ms(u16 nms)
{	 	
    vTaskDelay(nms / portTICK_PERIOD_MS);
}



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

//static 
const char *TAG = "uart_events";
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

// #define ECHO_TEST_TXD  (GPIO_NUM_4)
// #define ECHO_TEST_RXD  (GPIO_NUM_5)
// #define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
// #define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)


// #define ECHO_TEST2_TXD  (GPIO_NUM_17)
// #define ECHO_TEST2_RXD  (GPIO_NUM_16)
// #define ECHO_TEST2_RTS  (UART_PIN_NO_CHANGE)
// #define ECHO_TEST2_CTS  (UART_PIN_NO_CHANGE)

// #define ECHO_TEST3_TXD  (GPIO_NUM_19)
// #define ECHO_TEST3_RXD  (GPIO_NUM_18)
// #define ECHO_TEST3_RTS  (UART_PIN_NO_CHANGE)
// #define ECHO_TEST3_CTS  (UART_PIN_NO_CHANGE)

#define ECHO_TEST_TXD  (GPIO_NUM_33)
#define ECHO_TEST_RXD  (GPIO_NUM_32)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)


#define ECHO_TEST2_TXD  (GPIO_NUM_2)
#define ECHO_TEST2_RXD  (GPIO_NUM_34)
#define ECHO_TEST2_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST2_CTS  (UART_PIN_NO_CHANGE)

    #define ECHO_TEST3_TXD  (GPIO_NUM_19)
    #define ECHO_TEST3_RXD  (GPIO_NUM_4)
    #define ECHO_TEST3_RTS  (UART_PIN_NO_CHANGE)
    #define ECHO_TEST3_CTS  (UART_PIN_NO_CHANGE)

    #define ECHO_TEST4_TXD  (GPIO_NUM_21)
    #define ECHO_TEST4_RXD  (GPIO_NUM_36)
    #define ECHO_TEST4_RTS  (UART_PIN_NO_CHANGE)
    #define ECHO_TEST4_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)
uint8_t data_rx[BUF_SIZE] = {0};
int len_rx;

uint8_t data_rx2[BUF_SIZE] = {0};
int len_rx2;
uint8_t data_rx2_m[BUF_SIZE] = {0};
int len_rx2_m;
uint8_t flag_rx2;


#define TX1_LEN 10




static void echo_task2()
{

    
    // while(1)
    // {
    //     vTaskDelay(500 / portTICK_PERIOD_MS);

    // }
    uint16_t bl_addr=0;//bianliang
    while(1)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
                //&&(flag_rx2 ==0)
        // if ((len_rx2 > 0) ) {
        //     flag_rx2 =1;
        //     len_rx2_m = len_rx2;
        //     memcpy(data_rx2_m,data_rx2,len_rx2_m);
        //     ESP_LOGI(TAG, "uart2-Received %u bytes:", len_rx2_m);
        //     for (int i = 0; i < len_rx2_m; i++) {
        //         printf("0x%.2X ", (uint8_t)data_rx2_m[i]);
        //     }
        //     printf("] \n");
        // }

								
		


        if (len_rx > 0) {
            // ESP_LOGI(TAG, "uart1-Received %u bytes:", len_rx);
            // for (int i = 0; i < len_rx; i++) {
            //     printf("0x%.2X ", (uint8_t)data_rx[i]);
            // }
            // printf("] \n");
            
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

                            uint8_t tx_Buffer[50]={0};  
                            uint8_t bcc_temp=0;
                            switch (bl_addr)
                            {

                            case 0x2010://zhiwen  or   mima
                                ESP_LOGI(TAG, "----------------zhiwen or mima---------------.\r\n");   
                                //if -> huise tupian?

                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;
                                tx_Buffer[2] = 0x05;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x10;
                                tx_Buffer[5] = 0x20;//dizhi

                                tx_Buffer[6] = 0x00;
                                tx_Buffer[7] = 0x11;//data shengyu dagezi
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, 8);

                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;
                                tx_Buffer[2] = 0x05;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x10;
                                tx_Buffer[5] = 0x30;//dizhi

                                tx_Buffer[6] = 0x00;
                                tx_Buffer[7] = 0x22;//data shengyu dagezi todo
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, 8);

                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;
                                tx_Buffer[2] = 0x05;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x10;
                                tx_Buffer[5] = 0x40;//dizhi

                                tx_Buffer[6] = 0x00;
                                tx_Buffer[7] = 0x33;//data shengyu dagezi
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, 8);




                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;
                                tx_Buffer[2] = 0x07;
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x00;
                                tx_Buffer[5] = 0x84;//dizhi

                                tx_Buffer[6] = 0x5A;
                                tx_Buffer[7] = 0x01;//data guding

                                tx_Buffer[8] = 0x00;
                                if(01== data_rx[8])//zhiwen cun
                                {
                                    ESP_LOGI(TAG, "----------------zhiwen---------------.\r\n");  
                                    tx_Buffer[9] = 0x04;//baocun
                                }
                                else if(02== data_rx[8])//mi ma
                                {
                                    tx_Buffer[9] = 0x03;
                                    ESP_LOGI(TAG, "----------------mima---------------.\r\n");  

                                }
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);


                                //zhiwen_num_id =0;
                                //zhiwen_num_id = data_rx[7];
                                //Add_FR();		//录指纹	
                                break;
                            case 0x2020://da zhong xiao
                                ESP_LOGI(TAG, "----------------daxiao---------------.\r\n");   
                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;

                                tx_Buffer[2] = 0x07;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x00;
                                tx_Buffer[5] = 0x84;

                                tx_Buffer[6] = 0x5A;
                                tx_Buffer[7] = 0x01;
                                if(1)//2010 密码
                                {
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x06;
                                }
                                else//2010 指纹 ->指纹
                                {
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x04;
                                }
                                

                                //todo
                                if(01== data_rx[8])//da
                                {
                                    ESP_LOGI(TAG, "----------------da---------------.\r\n");  
                                    //cun qi lai   quanju yihuiyong
                                    
                                }
                                else if(02== data_rx[8])//zhong
                                {
                                    ESP_LOGI(TAG, "----------------zhong---------------.\r\n");  


                                }
                                else if(03== data_rx[8])//xiao
                                {
                                    ESP_LOGI(TAG, "----------------xiao---------------.\r\n");  
                                }
                                
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);
                                break;



                            case 0x1050:
                                //5A A5 10 83   10 50   06    31 32 33 34 35 36 37 38 39 30  31 00
                                ESP_LOGI(TAG, "----------------phone number---------------.\r\n");
                                //panduan   -  zan cun quanju
                                //if has, todo

                                //if weishu
                                if(06== data_rx[6])
                                {
                                    //zancun
                                }
                                else
                                {
                                    tx_Buffer[0] = 0x5A;
                                    tx_Buffer[1] = 0xA5;

                                    tx_Buffer[2] = 0x07;//len
                                    tx_Buffer[3] = 0x82;

                                    tx_Buffer[4] = 0x00;
                                    tx_Buffer[5] = 0x84;

                                    tx_Buffer[6] = 0x5A;
                                    tx_Buffer[7] = 0x01;

                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x07;
                                }
                                
                                break;

                            case 0x1060:
                                //5A A5 0A 83   10 60   03   31 32 33 34 35 36 
                                ESP_LOGI(TAG, "----------------password---------------.\r\n");
                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;

                                tx_Buffer[2] = 0x07;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x00;
                                tx_Buffer[5] = 0x84;

                                tx_Buffer[6] = 0x5A;
                                tx_Buffer[7] = 0x01;


                                if(03== data_rx[6])
                                {
                                    //cun
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x08;
                                }
                                else
                                {
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x07;
                                }
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);

                                break;











                            case 0x3050:
                                ESP_LOGI(TAG, "----------------lock1 ts---------------.\r\n");
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
                            case 0x3060:
                                ESP_LOGI(TAG, "----------------lock2 ts---------------.\r\n");

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




                            case 0x2090:
                                ESP_LOGI(TAG, "----------------zhiwen uart2test uart3 todo---------------.\r\n");
                                
                                //zhiwen_num_id =0;
                                //zhiwen_num_id = data_rx[7];
                                //Add_FR();		//录指纹	
                                
                                //Del_FR(0);		//删指纹
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
        // if(PS_Sta)	 //检测PS_Sta状态，如果有手指按下
		// {
		// 	press_FR();//刷指纹			
		// }
    }
    //vTaskDelay(1);
    
    vTaskDelete(NULL);

}





static void echo_task()
{
    
    // while(1)
    // {
    //     vTaskDelay(50 / portTICK_PERIOD_MS);

    // }
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    // uart_config_t uart_config2 = {
    //     .baud_rate = 9600,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity    = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    // };
    uart_config_t uart_config2 = {
        .baud_rate = 57600,
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

    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // ESP_LOGI(TAG, "Start ttl application test and configure UART2.\r\n");

    //1
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    //2
    uart_param_config(UART_NUM_2, &uart_config2);
    uart_set_pin(UART_NUM_2, ECHO_TEST2_TXD, ECHO_TEST2_RXD, ECHO_TEST2_RTS, ECHO_TEST2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);

    // //3 io moni
    // uart_param_config(UART_NUM_3, &uart_config3);
    // uart_set_pin(UART_NUM_3, ECHO_TEST3_TXD, ECHO_TEST3_RXD, ECHO_TEST3_RTS, ECHO_TEST3_CTS);
    // uart_driver_install(UART_NUM_3, BUF_SIZE * 2, 0, 0, NULL, 0);



    // Configure a temporary buffer for the incoming data
    //uint8_t *data_rx = (uint8_t *) malloc(BUF_SIZE);

    //ESP_LOGI(TAG, "UART1 start recieve loop.\r\n");

    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // Read data from the UART
        len_rx = uart_read_bytes(UART_NUM_1, data_rx, BUF_SIZE, 20 / portTICK_RATE_MS);
        len_rx2 = uart_read_bytes(UART_NUM_2, data_rx2, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        
        //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);
        //uart_write_bytes(UART_NUM_2, (const char *) data_rx, len_rx);


        if ((len_rx2 > 0) ) {
            flag_rx2 =1;
            len_rx2_m = len_rx2;
            memcpy(data_rx2_m,data_rx2,len_rx2_m);
            ESP_LOGI(TAG, "uart2-Received %u bytes:", len_rx2_m);
            for (int i = 0; i < len_rx2_m; i++) {
                printf("0x%.2X ", (uint8_t)data_rx2_m[i]);
            }
            printf("] \n");
        }

								
		


        if (len_rx > 0) {
            ESP_LOGI(TAG, "uart1-Received %u bytes:", len_rx);
            for (int i = 0; i < len_rx; i++) {
                printf("0x%.2X ", (uint8_t)data_rx[i]);
            }
            printf("] \n");
        }
	
    }
    //vTaskDelay(1);
    
    vTaskDelete(NULL);
}








//显示确认码错误信息
void ShowErrMessage(u8 ensure)
{
		ESP_LOGI(TAG,"%s\r\n",(u8*)EnsureMessage(ensure));
}
//录指纹
void Add_FR(void)
{
	u8 i=0,ensure ,processnum=0;
	//u16 ID;
	while(1)
	{
        //vTaskDelay(100 / portTICK_PERIOD_MS);
		switch (processnum)
		{
			case 0:
				i++;
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				ESP_LOGI(TAG,"请按指纹\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//BEEP=1;------------------------
					ensure=PS_GenChar(CharBuffer1);//生成特征
					//BEEP=0;
					if(ensure==0x00)
					{
						//LCD_Fill(0,120,lcddev.width,160,WHITE);
						ESP_LOGI(TAG,"指纹正常\r\n");
						i=0;
						processnum=1;//跳到第二步						
					}
                    else 
                    {
                        ESP_LOGI(TAG,"2-ensure=%d\r\n",ensure);
                        ShowErrMessage(ensure);			
                    }
                        	
				}
                else 
                {
                    ESP_LOGI(TAG,"1-ensure=%d\r\n",ensure);
                    ShowErrMessage(ensure);	
                }
                    					
				break;
			
			case 1:
				i++;
				////LCD_Fill(0,100,lcddev.width,160,WHITE);
				ESP_LOGI(TAG,"请按再按一次指纹\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//BEEP=1;
					ensure=PS_GenChar(CharBuffer2);//生成特征
					//BEEP=0;
					if(ensure==0x00)
					{
						////LCD_Fill(0,120,lcddev.width,160,WHITE);
						ESP_LOGI(TAG,"指纹正常\r\n");
						i=0;
						processnum=2;//跳到第三步
					}else ShowErrMessage(ensure);	
				}else ShowErrMessage(ensure);		
				break;

			case 2:
				////LCD_Fill(0,100,lcddev.width,160,WHITE);
				ESP_LOGI(TAG,"对比两次指纹\r\n");
				ensure=PS_Match();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					ESP_LOGI(TAG,"对比成功,两次指纹一样\r\n");
					processnum=3;//跳到第四步
				}
				else 
				{
					//LCD_Fill(0,100,lcddev.width,160,WHITE);
					ESP_LOGI(TAG,"对比失败，请重新录入指纹\r\n");
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//跳回第一步		
				}
				delay_ms(1200);
				break;

			case 3:
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				ESP_LOGI(TAG,"生成指纹模板\r\n");
				ensure=PS_RegModel();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					ESP_LOGI(TAG,"生成指纹模板成功\r\n");
					processnum=4;//跳到第五步
				}else {processnum=0;ShowErrMessage(ensure);}
				delay_ms(1200);
				break;
				
			case 4:	
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				ESP_LOGI(TAG,"请输入储存ID,按Enter保存\r\n");
				ESP_LOGI(TAG,"0=< ID <=299\r\n");// 0 - 100---------------------

                
				// do
				// 	ID=GET_NUM();
				while(!(zhiwen_num_id<AS608Para.PS_max));//输入ID必须小于最大存储数值



				ensure=PS_StoreChar(CharBuffer2,zhiwen_num_id);//储存模板
				if(ensure==0x00) 
				{			
					//LCD_Fill(0,100,lcddev.width,160,WHITE);					
					ESP_LOGI(TAG,"录入指纹成功\r\n");
					PS_ValidTempleteNum(&ValidN);//读库指纹个数
					ESP_LOGI(TAG,"zhiwen number=%d\r\n",AS608Para.PS_max-ValidN);
					delay_ms(1500);
					//LCD_Fill(0,100,240,160,WHITE);
					return ;
				}else {processnum=0;ShowErrMessage(ensure);}					
				break;				
		}
		delay_ms(400);
		if(i==5)//超过5次没有按手指则退出
		{
			//LCD_Fill(0,100,lcddev.width,160,WHITE);
			break;	
		}				
	}
}

//刷指纹
void press_FR(void)
{
	SearchResult seach;
	u8 ensure;
	//char *str;
	ensure=PS_GetImage();
	if(ensure==0x00)//获取图像成功 
	{	
		//BEEP=1;//打开蜂鸣器	---------------
		ensure=PS_GenChar(CharBuffer1);
		if(ensure==0x00) //生成特征成功
		{		
			//BEEP=0;//关闭蜂鸣器	-------------
			ensure=PS_HighSpeedSearch(CharBuffer1,0,AS608Para.PS_max,&seach);
			if(ensure==0x00)//搜索成功
			{				
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				ESP_LOGI(TAG,"刷指纹成功\r\n");				

				ESP_LOGI(TAG,"确有此人,ID:%d  匹配得分:%d\r\n",seach.pageID,seach.mathscore);

				//myfree(SRAMIN,str);
			}
			else 
				ShowErrMessage(ensure);					
	  }
		else
			ShowErrMessage(ensure);
	 //BEEP=0;//关闭蜂鸣器-----------
	 delay_ms(600);
	 //LCD_Fill(0,100,lcddev.width,160,WHITE);
	}
		
}

//删除指纹
void Del_FR(u16 num)
{
	u8  ensure;
	//u16 num;
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	ESP_LOGI(TAG,"删除指纹\r\n");
	ESP_LOGI(TAG,"请输入指纹ID按Enter发送\r\n");
	ESP_LOGI(TAG,"0=< ID <=299\r\n");
	delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_delFR);
	//num=GET_NUM();//获取返回的数值

	if(num==0xFFFF)
        return ;
		//goto MENU ; //返回主页面-------------
	else if(num==0xFF00)
		ensure=PS_Empty();//清空指纹库
	else 
		ensure=PS_DeletChar(num,1);//删除单个指纹
	if(ensure==0)
	{
		//LCD_Fill(0,120,lcddev.width,160,WHITE);
		ESP_LOGI(TAG,"删除指纹成功\r\n");		
	}
  else
		ShowErrMessage(ensure);	
	delay_ms(1200);
	PS_ValidTempleteNum(&ValidN);//读库指纹个数
	//LCD_ShowNum(56,80,AS608Para.PS_max-ValidN,3,16);
    ESP_LOGI(TAG,"zhiwen number =%d\r\n",AS608Para.PS_max-ValidN);
//MENU:	
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_menu);
}







void app_main()
{
    u8 ensure;
    uint8_t tx_Buffer[50]={0};  

    xTaskCreate(echo_task, "uart_echo_task", 4* 1024, NULL, 5, NULL);//1024 10
    //vTaskDelay(70 / portTICK_PERIOD_MS);
    xTaskCreate(echo_task2, "uart_echo_task2",4* 1024, NULL, 5, NULL);
	
    vTaskDelay(1000 / portTICK_PERIOD_MS);





    // ESP_LOGI(TAG, "与AS608模块握手....\r\n");
	// while(PS_HandShake(&AS608Addr))//与AS608模块握手
	// {
	// 	delay_ms(400);
    //     ESP_LOGI(TAG, "未检测到模块!!!\r\n");
    //     delay_ms(800);
    //     ESP_LOGI(TAG, "尝试连接模块...\r\n");	

    //     // u8 data = 0x35;
    //     // uart_write_bytes(UART_NUM_0, (const char *) &data, 1);//------UART_NUM_2------	  
    //     // uart_write_bytes(UART_NUM_1, (const char *) &data, 1);//------UART_NUM_2------	  
	// }

    // ESP_LOGI(TAG,"通讯成功!!!\r\n");
    // ESP_LOGI(TAG,"波特率:%d   地址:%x\r\n",usart2_baund,AS608Addr);


	// ensure=PS_ValidTempleteNum(&ValidN);//读库指纹个数
	// if(ensure!=0x00)
    // {
    //     ESP_LOGI(TAG,"1-ensure = %d\r\n",ensure);
    //     ShowErrMessage(ensure);//显示确认码错误信息	
    // }
		
    // delay_ms(100);

	// ensure=PS_ReadSysPara(&AS608Para);  //读参数 
	// if(ensure==0x00)
	// {
	// 		// mymemset(str,0,50);
	// 		// sprintf(str,"库容量:%d     对比等级: %d",AS608Para.PS_max-ValidN,AS608Para.PS_level);
	// 		// Show_Str(0,80,240,16,(u8*)str,16,0);
    //     ESP_LOGI(TAG,"库容量:%d     对比等级: %d",AS608Para.PS_max-ValidN,AS608Para.PS_level);
	// }
	// else
    // {
    //     ESP_LOGI(TAG,"2-ensure = %d\r\n",ensure);
    //     ShowErrMessage(ensure);	
    // }


    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = 0x07;
    tx_Buffer[3] = 0x82;
    tx_Buffer[4] = 0x00;
    tx_Buffer[5] = 0x84;
    tx_Buffer[6] = 0x5A;
    tx_Buffer[7] = 0x01;
    tx_Buffer[8] = 0x00;
    //todo
    tx_Buffer[9] = 0x00;//kaiji

    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);
    ESP_LOGI(TAG,"切换到开机画面!!!\r\n");




    

}
