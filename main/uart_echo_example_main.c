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
#include <stdint.h>


#include "nvs_flash.h"
#include "nvs.h"

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


#define ECHO_TEST_TXD  GPIO_NUM_32//(GPIO_NUM_33)//GPIO_NUM_4
#define ECHO_TEST_RXD  GPIO_NUM_33//(GPIO_NUM_32)//GPIO_NUM_5
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)


#define ECHO_TEST2_TXD  (GPIO_NUM_2)//2-deng    23
#define ECHO_TEST2_RXD  (GPIO_NUM_34)//34        22
#define ECHO_TEST2_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST2_CTS  (UART_PIN_NO_CHANGE)
#define RE_485_GPIO     (GPIO_NUM_18)

    #define ECHO_TEST3_TXD  (GPIO_NUM_19)
    #define ECHO_TEST3_RXD  (GPIO_NUM_4)
    #define ECHO_TEST3_RTS  (UART_PIN_NO_CHANGE)
    #define ECHO_TEST3_CTS  (UART_PIN_NO_CHANGE)

    #define ECHO_TEST4_TXD  (GPIO_NUM_21)
    #define ECHO_TEST4_RXD  (GPIO_NUM_36)//SVP
    #define ECHO_TEST4_RTS  (UART_PIN_NO_CHANGE)
    #define ECHO_TEST4_CTS  (UART_PIN_NO_CHANGE)






#define BUF_SIZE (1024)
uint8_t data_rx[BUF_SIZE] = {0};
uint16_t len_rx;

uint8_t data_rx2[BUF_SIZE] = {0};
int len_rx2;
uint8_t data_rx2_m[BUF_SIZE] = {0};
int len_rx2_m;
uint8_t flag_rx2;


#define TX1_LEN 12//10 tupian

#define TX1_LEN_BL 10//8 bianliang




#define STORAGE_NAMESPACE "storage"//guizi
#define STORAGE_NAMESPACE_ADM "storage_adm"//guizi admin
#define STORAGE_NAMESPACE_USR "storage_usr"//yonghu

#define ADM_KEY_SHENGYU_D "ad_shengyu_d"
#define ADM_KEY_SHENGYU_Z "ad_shengyu_z"
#define ADM_KEY_SHENGYU_X "ad_shengyu_x"







#define SHENYU_GEZI_MAX 50//all kong




//admin   need save   实时更新
//shuliang
uint16_t shengyu_all=30;//
uint16_t shengyu_da=15;
uint16_t shengyu_zhong=10;
uint16_t shengyu_xiao=5;



//admin 
//and cun de yong
uint16_t shengyu_all_max=30;//shengyu max admin, guding

    uint16_t shengyu_da_max=15;//
    uint16_t shengyu_zhong_max=10;
    uint16_t shengyu_xiao_max=5;



int16_t guimen1_gk_max=16;
int16_t guimen2_gk_max=16;
int16_t guimen3_gk_max=16;//8  guding

int16_t guimen4_gk_max=16;
int16_t guimen5_gk_max=16;
int16_t guimen6_gk_max=16;
int16_t guimen7_gk_max=16;
int16_t guimen8_gk_max=16;
int16_t guimen9_gk_max=16;
int16_t guimen10_gk_max=16;
int16_t guimen11_gk_max=16;
int16_t guimen12_gk_max=16;
int16_t guimen13_gk_max=16;
int16_t guimen14_gk_max=16;
int16_t guimen15_gk_max=16;
int16_t guimen16_gk_max=16;
int16_t guimen17_gk_max=16;
int16_t guimen18_gk_max=16;
int16_t guimen19_gk_max=16;



//1 admin
typedef struct
{
    uint8_t shengyu1;
    uint8_t shengyu2;


}shujuku_struct_admin;

shujuku_struct_admin database_ad=
{

    .shengyu1 = 0,
};




//3
typedef struct
{

    //zhiwen mima
    uint8_t cunwu_mode_gz;

    //daxiao
    uint8_t dzx_mode_gz;//default

    //yonghu xin xi
    uint64_t phone_number_nvs_gz;  //
    uint32_t mima_number_nvs_gz;  //

    uint8_t state_gz; //zaiyong 是否



    // //格口编号
	// uint16_t dIndx_gz;//gaidao shuzuzhong
    
    bool lock;
    bool changqi;
}shujuku_struct_gz;
//={0,0,0,3};//柜子

shujuku_struct_gz database_gz[SHENYU_GEZI_MAX];//i
// shujuku_struct_gz database_gz=
// {
//     .state = 0 ,
// };


// //zhiwen mima
// uint8_t cunwu_mode;
// //leixing
// uint8_t dzx_mode=00;

uint8_t phone_weishu_ok=00;

//lock 格口编号
//weiyi xuhao

//donn't need save todo


//2
///command struct
typedef struct
{
    //zhiwen mima
    uint8_t cunwu_mode;

    //gekou leixing
    uint8_t dzx_mode;

    //yonghu xin xi
    uint64_t phone_number_nvs;  //
    uint32_t mima_number_nvs;  //


    //状态 在用是否
	bool state;





    //格口编号
	uint16_t dIndx;//dangqian shuru

    //取物唯一编号 time
    uint16_t unique_number;
}shujuku_struct_user;//用户

//shujuku_struct_user database_cw;
shujuku_struct_user database_cw=
{
    .dzx_mode = 0 ,
};



// esp_err_t save_i16_value(char* key, int16_t out_value)
// {
//     nvs_handle_t my_handle;
//     esp_err_t err;

//     // Open
//     err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) return err;

//     // Read
//     //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
//     err = nvs_get_i16(my_handle, key, &out_value);
//     if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

//     // Write
//     err = nvs_set_i16(my_handle, key, out_value);
//     if (err != ESP_OK) return err;

//     // Commit written value.
//     // After setting any values, nvs_commit() must be called to ensure changes are written
//     // to flash storage. Implementations may write to storage at other times,
//     // but this is not guaranteed.
//     err = nvs_commit(my_handle);
//     if (err != ESP_OK) return err;

//     // Close
//     nvs_close(my_handle);
//     return ESP_OK;
// }











//8
esp_err_t save_u8_value(const char* name, char* key, uint8_t out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    uint8_t out_value_t = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_u8(my_handle, key, &out_value_t);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    printf("w_r8-%s=%u\r\n",key,(uint8_t)(out_value_t));

    // Write
    err = nvs_set_u8(my_handle, key, out_value);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t read_u8_value(const char* name,char* key, uint8_t* out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS

    err = nvs_get_u8(my_handle, key, out_value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;


    printf("rd8-%s=%u\r\n\r\n",key,(uint8_t)(*out_value));

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}











//16
esp_err_t save_u16_value(const char* name,char* key, uint16_t out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    uint16_t out_value_t = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_u16(my_handle, key, &out_value_t);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    printf("w_r16-%s=%u\r\n",key,(uint16_t)(out_value_t));

    // Write
    err = nvs_set_u16(my_handle, key, out_value);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t read_u16_value(const char* name,char* key, uint16_t* out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS

    err = nvs_get_u16(my_handle, key, out_value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;


    printf("rd16-%s=%u\r\n\r\n",key,(uint16_t)(*out_value));

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}










//32
esp_err_t save_u32_value(const char* name,char* key, uint32_t out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    uint32_t out_value_t = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_u32(my_handle, key, &out_value_t);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    printf("wr_32-%s=%u\r\n",key,(out_value_t));

    // Write
    err = nvs_set_u32(my_handle, key, out_value);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t read_u32_value(const char* name,char* key, uint32_t* out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS

    err = nvs_get_u32(my_handle, key, out_value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;


    printf("r32-%s=%u\r\n\r\n",key,(*out_value));

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}







//64
esp_err_t save_u64_value(const char* name,char* key, uint64_t out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    uint64_t out_value_t = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_u64(my_handle, key, &out_value_t);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    printf("wr_64-%s=%llu\r\n",key,(out_value_t));

    // Write
    err = nvs_set_u64(my_handle, key, out_value);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t read_u64_value(const char* name,char* key, uint64_t* out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS

    err = nvs_get_u64(my_handle, key, out_value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;


    printf("r64-%s=%llu\r\n\r\n",key,(*out_value));

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}


// /* Save the number of module restarts in NVS
//    by first reading and then incrementing
//    the number that has been saved previously.
//    Return an error if anything goes wrong
//    during this process.
//  */
// esp_err_t save_restart_counter(void)
// {
//     nvs_handle_t my_handle;
//     esp_err_t err;

//     // Open
//     err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) return err;

//     // Read
//     int32_t dIndx_t = 0; // value will default to 0, if not set yet in NVS
//     err = nvs_get_u16(my_handle, "dw_dIndx", &dIndx_t);
//     if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

//     // Write
//     //restart_counter++;
//     //database_cw.dIndx ++;
//     err = nvs_set_u16(my_handle, "dw_dIndx", database_cw.dIndx);
//     if (err != ESP_OK) return err;

//     // Commit written value.
//     // After setting any values, nvs_commit() must be called to ensure changes are written
//     // to flash storage. Implementations may write to storage at other times,
//     // but this is not guaranteed.
//     err = nvs_commit(my_handle);
//     if (err != ESP_OK) return err;

//     // Close
//     nvs_close(my_handle);
//     return ESP_OK;
// }



// esp_err_t print_what_saved(void)
// {
//     nvs_handle_t my_handle;
//     esp_err_t err;

//     // Open
//     err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) return err;



//     // // Read restart counter
//     // int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
//     // err = nvs_get_i32(my_handle, "restart_conter", &restart_counter);
//     // if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
//     // printf("Restart counter = %d\n", restart_counter);



//     err = nvs_get_u16(my_handle, "dw_dIndx", &database_cw.dIndx);
//     if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
//     printf("dIndx = %d\n", database_cw.dIndx);



//     // // Read run time blob
//     // size_t required_size = 0;  // value will default to 0, if not set yet in NVS
//     // // obtain required memory space to store blob being read from NVS
//     // err = nvs_get_blob(my_handle, "run_time", NULL, &required_size);
//     // if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
//     // printf("Run time:\n");
//     // if (required_size == 0) {
//     //     printf("Nothing saved yet!\n");
//     // } else {
//     //     uint32_t* run_time = malloc(required_size);
//     //     err = nvs_get_blob(my_handle, "run_time", run_time, &required_size);
//     //     if (err != ESP_OK) {
//     //         free(run_time);
//     //         return err;
//     //     }
//     //     for (int i = 0; i < required_size / sizeof(uint32_t); i++) {
//     //         printf("%d: %d\n", i + 1, run_time[i]);
//     //     }
//     //     free(run_time);
//     // }

//     // Close
//     nvs_close(my_handle);
//     return ESP_OK;
// }


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



static void RS485_delay(u32 nCount)
{
	for(; nCount != 0; nCount--);
} 

/*??????·?????*/
//????????????,±??????????±????485???í?ê????
#define RS485_RX_EN()			RS485_delay(1000); gpio_set_level(RE_485_GPIO, 0);//rx;  RS485_delay(1000);
//????·???????,±??????????±????485???í?ê????
#define RS485_TX_EN()			RS485_delay(1000); gpio_set_level(RE_485_GPIO, 1);//rx;  RS485_delay(1000);



static const uint8_t auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};
 
static const uint8_t auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};
 
uint16_t CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
  uint8_t uchCRCHi = 0xFF; // 高CRC字节初始化
  uint8_t uchCRCLo = 0xFF; // 低CRC 字节初始化
  uint32_t uIndex; // CRC循环中的索引
  while (usDataLen--) // 传输消息缓冲区
  {
    uIndex = uchCRCHi ^ *puchMsg++; // 计算CRC
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];
  }
  return (((uint16_t)uchCRCLo << 8u) | uchCRCHi);
}
//版权声明：本文为CSDN博主「lhsfly」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
//原文链接：https://blog.csdn.net/lhsfly/java/article/details/84072316

void uart0_debug_str(uint8_t* str,uint8_t len)
{
    printf("debug_str:");
    for(uint8_t i=0;i<len;i++)
        printf("%c ",str[i]);
    printf("\r\n");
}
void uart0_debug_data(uint8_t* data,uint8_t len)
{
    printf("debug_data:");
    for(uint8_t i=0;i<len;i++)
        printf("%02x ",data[i]);
    printf("\r\n");
}
void uart0_debug_data_dec(uint16_t* data,uint16_t len)
{
    printf("debug_data:");
    for(uint16_t i=0;i<len;i++)
        printf("%02d ",data[i]);
    printf("\r\n");
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





void kaiji_huamian(void)
{
    uint16_t crc16_temp=0;
    uint8_t tx_Buffer[50]={0};  

    //kaijie huamian
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = 0x09;//len
    tx_Buffer[3] = 0x82;
    tx_Buffer[4] = 0x00;
    tx_Buffer[5] = 0x84;
    tx_Buffer[6] = 0x5A;
    tx_Buffer[7] = 0x01;
    tx_Buffer[8] = 0x00;
    //todo
    tx_Buffer[9] = 0x26;//kaiji
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN-5);
    printf("tx CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[10] = crc16_temp&0xff;
    tx_Buffer[11] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);

    ESP_LOGI(TAG,"切换到开机画面!!!\r\n");



}




void tongbu_gekou_shuliang_all(uint16_t temp)
{
    uint8_t tx_Buffer[50]={0};  
    uint16_t crc16_temp=0;
    //da
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = 0x07;//len
    tx_Buffer[3] = 0x82;

    tx_Buffer[4] = 0x10;
    tx_Buffer[5] = 0x10;//dizhi

    tx_Buffer[6] = temp/256;
    tx_Buffer[7] = temp%256;//data shengyu dagezi

    printf("shengyu_da1:0x%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    printf("tx1 CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[8] = crc16_temp&0xff;
    tx_Buffer[9] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN_BL);

}

void tongbu_gekou_shuliang_d(uint16_t temp)
{
    uint8_t tx_Buffer[50]={0};  
    uint16_t crc16_temp=0;
    //da
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = 0x07;//len
    tx_Buffer[3] = 0x82;

    tx_Buffer[4] = 0x10;
    tx_Buffer[5] = 0x20;//dizhi

    tx_Buffer[6] = temp/256;
    tx_Buffer[7] = temp%256;//data shengyu dagezi

    printf("shengyu_da1:0x%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    printf("tx1 CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[8] = crc16_temp&0xff;
    tx_Buffer[9] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN_BL);

}

void tongbu_gekou_shuliang_z(uint16_t temp )
{
    uint8_t tx_Buffer[50]={0};  
    uint16_t crc16_temp=0;
    //zhong
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = 0x07;//len
    tx_Buffer[3] = 0x82;

    tx_Buffer[4] = 0x10;
    tx_Buffer[5] = 0x30;//dizhi

    tx_Buffer[6] = temp/256;
    tx_Buffer[7] = temp%256;
    printf("shengyu_zhong1:0x%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    printf("tx2 CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[8] = crc16_temp&0xff;
    tx_Buffer[9] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN_BL);



}

void tongbu_gekou_shuliang_x(uint16_t temp)
{
    uint8_t tx_Buffer[50]={0};  
    uint16_t crc16_temp=0;
    //xiao
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = 0x07;//len
    tx_Buffer[3] = 0x82;

    tx_Buffer[4] = 0x10;
    tx_Buffer[5] = 0x40;//dizhi

    tx_Buffer[6] = temp/256;
    tx_Buffer[7] = temp%256;
    printf("shengyu_xiao1:0x%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    printf("tx3 CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[8] = crc16_temp&0xff;
    tx_Buffer[9] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN_BL);

}




void send_cmd_to_lcd_bl(uint16_t opCode, uint16_t temp)//变量
{
    uint8_t tx_Buffer[50]={0};  
    uint16_t crc16_temp=0;
    //xiao
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = 0x07;//len
    tx_Buffer[3] = 0x82;

    tx_Buffer[4] = opCode/256;
    tx_Buffer[5] = opCode%256;//dizhi

    tx_Buffer[6] = temp/256;
    tx_Buffer[7] = temp%256;
    printf("temp-bl:0x%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    printf("tx CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[8] = crc16_temp&0xff;
    tx_Buffer[9] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN_BL);
}

void send_cmd_to_lcd_pic(uint16_t temp)//图片
{
    uint8_t tx_Buffer[50]={0};  
    uint16_t crc16_temp=0;

    ESP_LOGI(TAG, "-----pic-----.\r\n");
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;

    tx_Buffer[2] = 0x09;//len
    tx_Buffer[3] = 0x82;

    tx_Buffer[4] = 0x00;
    tx_Buffer[5] = 0x84;

    tx_Buffer[6] = 0x5A;
    tx_Buffer[7] = 0x01;

        
    tx_Buffer[8] = temp/256;
    tx_Buffer[9] = temp%256;
    printf("temp-pic:0x%04d\r\n",temp);

    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN - 5);
    printf("tx CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[10] = crc16_temp&0xff;
    tx_Buffer[11] = (crc16_temp>>8)&0xff;
    
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);

}

// void lcd_send_cmd_response(uint8_t rsp)
// {

// }


uint8_t phone_number[11]={0};  
uint8_t mima_number[6]={0};  

static void echo_task2()//lcd
{
    uint16_t bl_addr=0;//bianliang lcd
    uint16_t crc16_temp=0;
    
    uint8_t data_rx_t[BUF_SIZE] = {0};
    uint16_t len_rx_t= len_rx;

    memcpy(data_rx_t,data_rx,len_rx_t);
    //while(1)
    {
        //vTaskDelay(40 / portTICK_PERIOD_MS);
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

								
		


        if (len_rx_t > 0) {
            // ESP_LOGI(TAG, "uart1-Received %u bytes:", len_rx_t);
            // for (int i = 0; i < len_rx_t; i++) {
            //     printf("0x%.2X ", (uint8_t)data_rx_t[i]);
            // }
            // printf("] \n");
            crc16_temp = CRC16(data_rx_t+3, data_rx_t[2] -2);
            //printf("rx CRC16 result:0x%04X\r\n",crc16_temp);
            



            // printf("data_rx_t[0]:0x%02X, data_rx_t[1]:0x%02X\r\n",data_rx_t[0], data_rx_t[1]);

            // printf("(len_rx_t-3):0x%02X,data_rx_t[2]:0x%02X\r\n",(len_rx_t-3),data_rx_t[2]);

            // printf("data_rx_t[data_rx_t[2] +2 -1]0x%02X\r\n",data_rx_t[data_rx_t[2] +2 -1]);
            // printf("(crc16_temp & 0xff):0x%02X\r\n",(crc16_temp & 0xff));

            // printf("data_rx_t[data_rx_t[2] +2 -1+1]0x%02X\r\n",data_rx_t[data_rx_t[2] +2 -1+1]);
            // printf("((crc16_temp>>8) & 0xff):0x%02X\r\n",((crc16_temp>>8) & 0xff));

            
            if((0x5A == data_rx_t[0])
                &&(0xA5 == data_rx_t[1])
                &&((len_rx_t-3) == data_rx_t[2])
                &&(data_rx_t[data_rx_t[2]+2-1]==(crc16_temp&0xff))
                &&(data_rx_t[data_rx_t[2]+2-1+1]==((crc16_temp>>8)&0xff)))
                {
                    switch (data_rx_t[3])
                    {
                    case 0x82:
                        //uart_write_bytes(UART_NUM_2, (const char *) (data_rx_t+4), len_rx_t-4);
                        
                        bl_addr = (data_rx_t[4]<<8) + data_rx_t[5];
                        ESP_LOGI(TAG, "--0x82--.bl_addr=%04x\r\n",bl_addr);

                        uint8_t tx_Buffer[50]={0};  
                        uint8_t bcc_temp=0;
                        switch (bl_addr)
                        {

                        case 0x4F4B:
                            ESP_LOGI(TAG, "-----ok-----.\r\n");
                            //todo
                            break;

                        default:
                            ESP_LOGI(TAG, "----------------83 default---------------.\r\n");
                            break;
                        }

                        break;

                    case 0x83:
                        if( data_rx_t[6] == (len_rx_t-7 -2)/2)
                        {
                            //uart_write_bytes(UART_NUM_2, (const char *) (data_rx_t+4), len_rx_t-4);
                            bl_addr = (data_rx_t[4]<<8) + data_rx_t[5];
                            ESP_LOGI(TAG, "--0x83--.bl_addr=%04x\r\n",bl_addr);

                            uint8_t tx_Buffer[50]={0};  
                            uint8_t tx_Buffer2[50]={0};  
                            uint8_t bcc_temp=0;
                            switch (bl_addr)
                            {
                            case 0x2080://
                                ESP_LOGI(TAG, "--cunwu--.\r\n");   
                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;

                                tx_Buffer[2] = 0x09;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x00;
                                tx_Buffer[5] = 0x84;

                                tx_Buffer[6] = 0x5A;
                                tx_Buffer[7] = 0x01;
                                if((0== shengyu_da)
                                    &&(0== shengyu_zhong)
                                    &&(0== shengyu_xiao))
                                {
                                    ESP_LOGI(TAG, "--zanwu kongxiang--.\r\n"); // houbian sheng  
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x01;
                                }
                                else//
                                {
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x02;//ji xu
                                }

                                //crc
                                crc16_temp = CRC16(tx_Buffer+3, TX1_LEN-5);
                                //printf("tx CRC16 result:0x%04X\r\n",crc16_temp);

                                tx_Buffer[10] = crc16_temp&0xff;
                                tx_Buffer[11] = (crc16_temp>>8)&0xff;
                                
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);


                                break;

                            case 0x2010://zhiwen  or   mima
                                ESP_LOGI(TAG, "---zhiwen or mima---.\r\n");   
                                //if -> huise tupian?

                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;
                                tx_Buffer[2] = 0x09;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x00;
                                tx_Buffer[5] = 0x84;//dizhi

                                tx_Buffer[6] = 0x5A;
                                tx_Buffer[7] = 0x01;//data guding


                                if((shengyu_xiao==0) && (shengyu_zhong==0))
                                {
                                    if(01== data_rx_t[8])//zhiwen cun
                                    {
                                        ESP_LOGI(TAG, "--2 zhiwen--.\r\n");  
                                        //baocun 1
                                        tx_Buffer[8] = 0x00;
                                        tx_Buffer[9] = 0x04;//tiao 选择格子类型
                                    }
                                    else if(02== data_rx_t[8])//mi ma
                                    {
                                        ESP_LOGI(TAG, "--2 mima--.\r\n");  
                                        //baocun 2
                                        tx_Buffer[8] = 0x00;
                                        tx_Buffer[9] = 0x06;//tiao 选择格子类型
                                    }
                                    ESP_LOGI(TAG, "---2 da---.\r\n");  
                                    database_cw.dzx_mode = 1;
                                }
                                else if((shengyu_da==0) && (shengyu_xiao==0))
                                {
                                    if(01== data_rx_t[8])//zhiwen cun
                                    {
                                        ESP_LOGI(TAG, "--2 zhiwen--.\r\n");  
                                        //baocun 1
                                        tx_Buffer[8] = 0x00;
                                        tx_Buffer[9] = 0x04;//tiao 选择格子类型
                                    }
                                    else if(02== data_rx_t[8])//mi ma
                                    {
                                        ESP_LOGI(TAG, "--2 mima--.\r\n");  
                                        //baocun 2
                                        tx_Buffer[8] = 0x00;
                                        tx_Buffer[9] = 0x06;//tiao 选择格子类型
                                    }
                                    ESP_LOGI(TAG, "---2 zhong---.\r\n");  
                                    database_cw.dzx_mode = 2;
                                }
                                else if((shengyu_da==0) && (shengyu_zhong==0))
                                {
                                    if(01== data_rx_t[8])//zhiwen cun
                                    {
                                        ESP_LOGI(TAG, "--2zhiwen--.\r\n");  
                                        //baocun 1
                                        tx_Buffer[8] = 0x00;
                                        tx_Buffer[9] = 0x04;//tiao 选择格子类型
                                    }
                                    else if(02== data_rx_t[8])//mi ma
                                    {
                                        ESP_LOGI(TAG, "--2 mima--.\r\n");  
                                        //baocun 2
                                        tx_Buffer[8] = 0x00;
                                        tx_Buffer[9] = 0x06;//tiao 选择格子类型
                                    }
                                    ESP_LOGI(TAG, "---2xiao---.\r\n");  
                                    database_cw.dzx_mode = 3;
                                }
                                else
                                {
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x03;//tiao 选择格子类型
                                    if(01== data_rx_t[8])//zhiwen cun
                                    {
                                        ESP_LOGI(TAG, "--zhiwen--.\r\n");  
                                        //baocun 1
                                        database_cw.cunwu_mode = 1;
                                    }
                                    else if(02== data_rx_t[8])//mi ma
                                    {
                                        ESP_LOGI(TAG, "--mima--.\r\n");  
                                        //baocun 2
                                        database_cw.cunwu_mode = 2;

                                    }
                                }

                                //crc
                                crc16_temp = CRC16(tx_Buffer+3, TX1_LEN-5);
                                printf("tx CRC16 result:0x%04X\r\n",crc16_temp);

                                tx_Buffer[10] = crc16_temp&0xff;
                                tx_Buffer[11] = (crc16_temp>>8)&0xff;

                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);


                                //zhiwen_num_id =0;
                                //zhiwen_num_id = data_rx_t[7];
                                //Add_FR();		//录指纹	
                                break;

                            case 0x2020://da zhong xiao    ke sheng
                                ESP_LOGI(TAG, "----------------daxiao---------------.\r\n");   
                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;

                                tx_Buffer[2] = 0x09;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x00;
                                tx_Buffer[5] = 0x84;

                                tx_Buffer[6] = 0x5A;
                                tx_Buffer[7] = 0x01;
                                if(02 == database_cw.cunwu_mode)//2010 密码
                                {
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x06;
                                }
                                else if(01 == database_cw.cunwu_mode)//2010 指纹 ->指纹判断 0005 todo
                                {
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x04;
                                }

                                //crc
                                crc16_temp = CRC16(tx_Buffer+3, TX1_LEN-5);
                                printf("tx CRC16 result:0x%04X\r\n",crc16_temp);

                                tx_Buffer[10] = crc16_temp&0xff;
                                tx_Buffer[11] = (crc16_temp>>8)&0xff;
                                




                                //todo
                                if((01== data_rx_t[8])&&(0!=shengyu_da))//da
                                {
                                    ESP_LOGI(TAG, "---da---.\r\n");  
                                    //cun qi lai   quanju yihuiyong
                                    database_cw.dzx_mode = 1;
                                }
                                else if((02== data_rx_t[8])&&(0!=shengyu_zhong))//zhong
                                {
                                    ESP_LOGI(TAG, "---zhong---.\r\n");  
                                    database_cw.dzx_mode = 2;
                                }
                                else if((03== data_rx_t[8])&&(0!=shengyu_xiao))//xiao
                                {
                                    ESP_LOGI(TAG, "---xiao---.\r\n");  
                                    database_cw.dzx_mode = 3;
                                }
                                else
                                {
                                    ESP_LOGI(TAG, "---zanwu kongxiang---.\r\n");  
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x01;
                                }
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);
                                

                                break;



                            case 0x1050:
                                //5A A5 10 83   10 50   06    31 32 33 34 35 36 37 38 39 30  31 00
                                ESP_LOGI(TAG, "--phone number--.\r\n");
                                //panduan   -  zan cun quanju
                                //if has, todo  -----------------------------------

                                // //if weishu    -------------------------------------
                                // if(06== data_rx_t[6])//12
                                // {
                                //     //zancun
                                //     phone_weishu_ok =1;
                                //     memcpy( phone_number,data_rx_t+7 ,11);

                                // }

                                //ESP_LOGI(TAG, "data_rx_t[6]=%d---.\r\n",data_rx_t[6]);
                                if(05== data_rx_t[6])//12
                                {
                                    //zancun
                                    phone_weishu_ok =1;
                                    memcpy( phone_number,data_rx_t+7 ,10);
                                    ESP_LOGI(TAG, "---phone_weishu_ok=%d---.\r\n",phone_weishu_ok);

                                }
                                else
                                {
                                    ESP_LOGI(TAG, "-------1 - shoujihao weishu err--------.\r\n");
                                }
                                
                                break;

                            case 0x1060:

                                //5A A5 0A 83   10 60   03   31 32 33 34 35 36 
                                ESP_LOGI(TAG, "--password--.\r\n");
                                //ESP_LOGI(TAG, "---phone_weishu_ok=%d---.\r\n",phone_weishu_ok);
                                tx_Buffer[0] = 0x5A;
                                tx_Buffer[1] = 0xA5;

                                tx_Buffer[2] = 0x09;//len
                                tx_Buffer[3] = 0x82;

                                tx_Buffer[4] = 0x00;
                                tx_Buffer[5] = 0x84;

                                tx_Buffer[6] = 0x5A;
                                tx_Buffer[7] = 0x01;
                                //存物的格口编号（123）、格口类型（1：小，2：中，3：大）
                                //存物手机号（11位）密码（6位）            或者指纹(----)   


                                if((1 == phone_weishu_ok)&&(03== data_rx_t[6]))//6   ok todo shoujihao yiyou
                                {
                                    memcpy( mima_number,data_rx_t+7 ,6);

                                    printf("phone_number=");
                                    uart0_debug_str(phone_number,11);

                                    printf("mima_number=");
                                    uart0_debug_str(mima_number,6);

                                    uint16_t j=0,k=0;
                                    uint16_t database_gz_temp[SHENYU_GEZI_MAX]={0};
                                    uint16_t database_gz_temp_onuse[SHENYU_GEZI_MAX]={0};

            



                                    database_cw.phone_number_nvs = atoi((const char*)phone_number);
                                    database_cw.mima_number_nvs = atoi((const char*)mima_number);
                                    printf("phone?=%11llu,mima?=%6u,", database_cw.phone_number_nvs, database_cw.mima_number_nvs);
                                    

                                    
                                    for (uint16_t i = 1; i <= shengyu_all_max; i++)//todo changqi and suoding
                                    {

                                        if(database_gz[i].phone_number_nvs_gz == database_cw.phone_number_nvs)
                                        {
                                            if(database_gz[i].mima_number_nvs_gz == database_cw.mima_number_nvs) 
                                            {
                                                //chongfu_flag =1;
                                                printf("---phone_number and key_number has in database\r\n");
                                                goto done;
                                            }
                                            else
                                            {
                                            
                                                printf("---only mima_number will xin, phone_number has in database\r\n");
                                            }


                                        }
                                        else//ok
                                        {
                                            //printf("---phone_number and mima will xin\r\n");
                                        }
                                        
                                    }
                                    printf("---phone_number and mima will xin\r\n");



                                    // start
                                    if(1 == database_cw.dzx_mode)
                                    {
                                        
                                        for (uint16_t i = 1; i <= shengyu_all_max; i++)//todo changqi and suoding
                                        {
                                            if((database_gz[i].state_gz ==0) //no use
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==1))//d
                                            {
                                                database_gz_temp[j++] =i;
                                            }
                                            else if((database_gz[i].state_gz ==1) //used
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==1))//d
                                            {
                                                database_gz_temp_onuse[k++] =i;
                                            }
                                            printf("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                                    i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                                        }
                                        printf("shengyu j=%d, onuse k=%d\r\n",j,k);
                                        uart0_debug_data_dec(database_gz_temp,j);
                                        uart0_debug_data_dec(database_gz_temp_onuse,k);

                                        if(j>0)
                                        {
                                            database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

                                            database_cw.state=1;
                                            // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                            printf("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);




                                            uint8_t j=0,k=0;
                                            //j=5; k=0;

                                            // suiji kaimen
                                            // if(shengyu_da >0)
                                            if(((int16_t)database_cw.dIndx-guimen1_gk_max)>0)
                                            {
                                                k++;//board
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;//lock
                                            }

                                            if(((int16_t)database_cw.dIndx-\
                                                guimen2_gk_max-\
                                                guimen1_gk_max)>0)
                                            {
                                                k++;
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;
                                            }

                                            if(((int16_t)database_cw.dIndx-\
                                                guimen3_gk_max-\
                                                guimen2_gk_max-\
                                                guimen1_gk_max)>0)
                                            {
                                                k++;
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;
                                            }
                                            printf("--open- board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);

                            





                                            //old
                                            shengyu_da --;
                                            tongbu_gekou_shuliang_d(shengyu_da);


                                            ESP_LOGI(TAG, "-da-lock:%d ok--.\r\n",j);
                                            memcpy(tx_Buffer2,"star",4);
                                            tx_Buffer2[4]= 0x8A;//m_data.opcode;
                                            tx_Buffer2[5]= (uint8_t)(k+1);//m_data.board_addr;
                                            tx_Buffer2[6]= (uint8_t)j;//m_data.lock_addr;
                                            tx_Buffer2[7]= 0x11;//guding
                                            bcc_temp = ComputXor(tx_Buffer2+4,4);
                                            tx_Buffer2[8]= bcc_temp;
                                            memcpy(tx_Buffer2+9,"endo",4);
                                            
                                            tx_Buffer2[13]='\0';

                                            RS485_TX_EN();

                                            printf("tx_Buffer2=");
                                            uart0_debug_data(tx_Buffer2, 13);
                                            uart_write_bytes(UART_NUM_2, (const char *) tx_Buffer2, 13);
                                            RS485_RX_EN();
                                        }    
                                        else
                                        {
                                            ESP_LOGI(TAG, "-------state d have no-----.\r\n");
                                        }
                                        

                                    }
                                    else if(2 == database_cw.dzx_mode)
                                    {
                                        for (uint16_t i = 1; i <= shengyu_all_max; i++)//todo changqi and suoding
                                        {
                                            if((database_gz[i].state_gz ==0) //no use
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==2))//z
                                            {
                                                database_gz_temp[j++] =i;
                                            }
                                            else if((database_gz[i].state_gz ==1) //used
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==2))//z
                                            {
                                                database_gz_temp_onuse[k++] =i;
                                            }
                                            printf("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                                    i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                                        }
                                        printf("shengyu j=%d, onuse k=%d\r\n",j,k);
                                        uart0_debug_data_dec(database_gz_temp,j);
                                        uart0_debug_data_dec(database_gz_temp_onuse,k);

                                        if(j>0)
                                        {
                                            database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

                                            database_cw.state=1;
                                            // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                            printf("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);




                                            uint8_t j=0,k=0;
                                            //j=5; k=0;

                                            // suiji kaimen
                                            // if(shengyu_da >0)
                                            if(((int16_t)database_cw.dIndx-guimen1_gk_max)>0)
                                            {
                                                k++;//board
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;//lock
                                            }

                                            if(((int16_t)database_cw.dIndx-\
                                                guimen2_gk_max-\
                                                guimen1_gk_max)>0)
                                            {
                                                k++;
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;
                                            }

                                            if(((int16_t)database_cw.dIndx-\
                                                guimen3_gk_max-\
                                                guimen2_gk_max-\
                                                guimen1_gk_max)>0)
                                            {
                                                k++;
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;
                                            }
                                            printf("--open- board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);

                            


                                            //if(shengyu_zhong >0)
                                            {
                                                shengyu_zhong --;
                                                tongbu_gekou_shuliang_z(shengyu_zhong);

                                                //ESP_LOGI(TAG, "--lock2 ok--.\r\n");

                                                ESP_LOGI(TAG, "-zhong-lock:%d ok--.\r\n",j);
                                                memcpy(tx_Buffer2,"star",4);
                                                tx_Buffer2[4]= 0x8A;//m_data.opcode;
                                                tx_Buffer2[5]= (uint8_t)(k+1);//m_data.board_addr;
                                                tx_Buffer2[6]= (uint8_t)j;//m_data.lock_addr;
                                                tx_Buffer2[7]= 0x11;//guding
                                                bcc_temp = ComputXor(tx_Buffer2+4,4);
                                                tx_Buffer2[8]= bcc_temp;
                                                memcpy(tx_Buffer2+9,"endo",4);
                                                
                                                tx_Buffer2[13]='\0';

                                                RS485_TX_EN();

                                                printf("tx_Buffer2=");
                                                uart0_debug_data(tx_Buffer2, 13);
                                                uart_write_bytes(UART_NUM_2, (const char *) tx_Buffer2, 13);
                                                RS485_RX_EN();
                                                
                                            }
                                            // else
                                            // {
                                            //     ESP_LOGI(TAG, "-------lock2  zhong  no-----.\r\n");
                                            // }  

                                        }
                                        else
                                        {
                                            ESP_LOGI(TAG, "-------state z have no-----.\r\n");
                                        }
                                        

                                    }
                                    else if(3 == database_cw.dzx_mode)
                                    {


                                        for (uint16_t i = 1; i <= shengyu_all_max; i++)//todo changqi and suoding
                                        {
                                            if((database_gz[i].state_gz ==0) //no use
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==3))//x
                                            {
                                                database_gz_temp[j++] =i;
                                            }
                                            else if((database_gz[i].state_gz ==1) //used
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==3))//x
                                            {
                                                database_gz_temp_onuse[k++] =i;
                                            }
                                            printf("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                                    i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                                        }
                                        printf("shengyu j=%d, onuse k=%d\r\n",j,k);
                                        uart0_debug_data_dec(database_gz_temp,j);
                                        uart0_debug_data_dec(database_gz_temp_onuse,k);

                                        if(j>0)
                                        {
                                            database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

                                            database_cw.state=1;
                                            // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                            printf("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);




                                            uint8_t j=0,k=0;
                                            //j=5; k=0;

                                            // suiji kaimen
                                            // if(shengyu_da >0)
                                            if(((int16_t)database_cw.dIndx-guimen1_gk_max)>0)
                                            {
                                                k++;//board
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;//lock
                                            }

                                            if(((int16_t)database_cw.dIndx-\
                                                guimen2_gk_max-\
                                                guimen1_gk_max)>0)
                                            {
                                                k++;
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;
                                            }

                                            if(((int16_t)database_cw.dIndx-\
                                                guimen3_gk_max-\
                                                guimen2_gk_max-\
                                                guimen1_gk_max)>0)
                                            {
                                                k++;
                                            }
                                            else
                                            {
                                                j=database_cw.dIndx;
                                            }
                                            printf("--open- board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);

                            





                                            //if(shengyu_xiao >0)
                                            {
                                                shengyu_xiao --;
                                                tongbu_gekou_shuliang_x(shengyu_xiao);
                                                //ESP_LOGI(TAG, "----------------lock3 ts---------------.\r\n");
                                                
                                                ESP_LOGI(TAG, "-xiao-lock:%d ok--.\r\n",j);
                                                memcpy(tx_Buffer2,"star",4);
                                                tx_Buffer2[4]= 0x8A;//m_data.opcode;
                                                tx_Buffer2[5]= (uint8_t)(k+1);//m_data.board_addr;
                                                tx_Buffer2[6]= (uint8_t)j;//m_data.lock_addr;
                                                tx_Buffer2[7]= 0x11;//guding
                                                bcc_temp = ComputXor(tx_Buffer2+4,4);
                                                tx_Buffer2[8]= bcc_temp;
                                                memcpy(tx_Buffer2+9,"endo",4);
                                                
                                                tx_Buffer2[13]='\0';

                                                RS485_TX_EN();

                                                printf("tx_Buffer2=");
                                                uart0_debug_data(tx_Buffer2, 13);
                                                uart_write_bytes(UART_NUM_2, (const char *) tx_Buffer2, 13);
                                                RS485_RX_EN();
                                            
                                            
                                            }
                                            // else
                                            // {
                                            //     ESP_LOGI(TAG, "----------------lock3  xiao  no---------------.\r\n");
                                            // } 
                                        }
                                        else
                                        {
                                            ESP_LOGI(TAG, "-------state x have no-----.\r\n");
                                        }
                                            
                                    }



                                    ESP_LOGI(TAG, "----test--.\r\n");  
                                    

                                    if(0 != database_cw.dzx_mode)
                                    {


                                        shengyu_all -- ;
                                        tongbu_gekou_shuliang_all(shengyu_all);
                                        send_cmd_to_lcd_bl(0x1070,database_cw.dIndx);

                                        char key_name[15];//15
                                        esp_err_t err;

                                        database_gz[database_cw.dIndx].cunwu_mode_gz = database_cw.cunwu_mode;
                                        sprintf(key_name, "%03d_dz_cw_md", database_cw.dIndx);
                                        printf("--key_name=%s--\r\n",key_name);

                                        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].cunwu_mode_gz);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].cunwu_mode_gz));
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));



                                        database_gz[database_cw.dIndx].dzx_mode_gz = database_cw.dzx_mode;
                                        sprintf(key_name, "%03d_dz_dzxmd", database_cw.dIndx);
                                        printf("--key_name=%s--\r\n",key_name);

                                        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].dzx_mode_gz);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].dzx_mode_gz));
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));




                                        //database_cw.phone_number_nvs = atoi((const char*)phone_number);

                                        database_gz[database_cw.dIndx].phone_number_nvs_gz = database_cw.phone_number_nvs;
                                        sprintf(key_name, "%03d_dz_phone", database_cw.dIndx);
                                        printf("--key_name=%s--\r\n",key_name);

                                        err = save_u64_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].phone_number_nvs_gz);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));
                                        
                                        err = read_u64_value(STORAGE_NAMESPACE,key_name,&database_gz[database_cw.dIndx].phone_number_nvs_gz);
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));




                                        //database_cw.mima_number_nvs = atoi((const char*)mima_number);
                                        
                                        database_gz[database_cw.dIndx].mima_number_nvs_gz = database_cw.mima_number_nvs;
                                        sprintf(key_name, "%03d_dz_mima", database_cw.dIndx);
                                        printf("--key_name=%s--\r\n",key_name);

                                        err = save_u32_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].mima_number_nvs_gz);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        err = read_u32_value(STORAGE_NAMESPACE,key_name,&database_gz[database_cw.dIndx].mima_number_nvs_gz);
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                                        


                                        database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                        sprintf(key_name, "%03d_dz_st", database_cw.dIndx);
                                        printf("--key_name=%s--\r\n",key_name);

                                        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].state_gz);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].state_gz));
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));




                                        //admin
                                        //d
                                        //printf("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);

                                        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_D,shengyu_da);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_D, (uint16_t*)(&shengyu_da));
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


                                        //z
                                        //printf("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);

                                        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_Z,shengyu_zhong);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_Z, (uint16_t*)(&shengyu_zhong));
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));



                                        //x
                                        //printf("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);

                                        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_X,shengyu_xiao);
                                        if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_X, (uint16_t*)(&shengyu_xiao));
                                        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));







                                        //custumer   mingming kongjian   todo
                                        // esp_err_t err = save_u16_value(STORAGE_NAMESPACE,"dw_dIndx",database_cw.dIndx);
                                        // if (err != ESP_OK) printf("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
                                        // if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                                        //unique number

                                    }
                                    //cun
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x08;

                                }
                                else
                                {
                                    if(03!= data_rx_t[6])
                                    {
                                      ESP_LOGI(TAG, "----------------2 - mima weishu err---------------.\r\n");  
                                    }
done:
                                    ESP_LOGI(TAG, "----test2-error--.\r\n");  
                                    tx_Buffer[8] = 0x00;
                                    tx_Buffer[9] = 0x07;
                                }



                                ESP_LOGI(TAG, "----test3-done--.\r\n");  
                                //crc
                                crc16_temp = CRC16(tx_Buffer+3, TX1_LEN-5);
                                //printf("tx CRC16 result:0x%04X\r\n",crc16_temp);

                                tx_Buffer[10] = crc16_temp&0xff;
                                tx_Buffer[11] = (crc16_temp>>8)&0xff;
                                uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);



                                
                                database_cw.cunwu_mode =0;
                                database_cw.dzx_mode = 0 ;
                                database_cw.state=0;

                                phone_weishu_ok =0;
                                break;








                            // case 0x3050:
                            //     ESP_LOGI(TAG, "----------------lock1 ts---------------.\r\n");
                            //     memcpy(tx_Buffer2,"star",4);
                            //     tx_Buffer2[4]= 0x8A;//m_data.opcode;
                            //     tx_Buffer2[5]= 0x01;//m_data.board_addr;
                            //     tx_Buffer2[6]= 0x02;//m_data.lock_addr;
                            //     tx_Buffer2[7]= 0x11;//guding
                            //     bcc_temp = ComputXor(tx_Buffer2+4,4);
                            //     tx_Buffer2[8]= bcc_temp;
                            //     memcpy(tx_Buffer2+9,"endo",4);
                                
                            //     tx_Buffer2[13]='\0';
                            //     uart_write_bytes(UART_NUM_2, (const char *) tx_Buffer2, 13);
                            //     break;
                            // case 0x3060:
                            //     ESP_LOGI(TAG, "----------------lock2 ts---------------.\r\n");

                            //     memcpy(tx_Buffer2,"star",4);
                            //     tx_Buffer2[4]= 0x8A;//m_data.opcode;
                            //     tx_Buffer2[5]= 0x01;//m_data.board_addr;
                            //     tx_Buffer2[6]= 0x01;//m_data.lock_addr;
                            //     tx_Buffer2[7]= 0x11;//guding
                            //     bcc_temp = ComputXor(tx_Buffer2+4,4);
                            //     tx_Buffer2[8]= bcc_temp;
                            //     memcpy(tx_Buffer2+9,"endo",4);
                                
                            //     tx_Buffer2[13]='\0';
                            //     uart_write_bytes(UART_NUM_2, (const char *) tx_Buffer2, 13);
                            //     break;



                            //zhiwen todo
                            case 0x2090:
                                ESP_LOGI(TAG, "----------------zhiwen uart2test uart3 todo---------------.\r\n");
                                
                                //zhiwen_num_id =0;
                                //zhiwen_num_id = data_rx_t[7];
                                //Add_FR();		//录指纹	
                                
                                //Del_FR(0);		//删指纹
                                break;







//---------------------------------取物---------------------------------------------------
                            case 0x2030:
                                ESP_LOGI(TAG, "-----qu-----.\r\n");
                                send_cmd_to_lcd_pic(0x000a);
                                break;

                            default:
                                ESP_LOGI(TAG, "----------------83 default---------------.\r\n");
                                break;
                            }

                        }
                        else
                        {
                            ESP_LOGI(TAG, "----------------len2 err---------------.\r\n");
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
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);

}


///command struct
typedef struct
{
	//uint8_t type;
	uint8_t len;
	uint8_t opcode;
	uint16_t sum;
	uint8_t dIndx;
	uint8_t data[256];
	
}command_struct;

command_struct g_data;


#define CMD_SUCCESS 0x01
#define CMD_FAIL	     0x00

/////start process the data in
uint8_t uState = 0;
// static void user_cmd_buffer_clear(void)
// {
// 	uState = 0;
// 	g_data.len = 0;
// 	g_data.type = 0;
// 	g_data.opcode = 0;
// 	g_data.sum = 0;
// 	g_data.dIndx = 0;
// }

///command enum
typedef enum
{
	UART_IDLE =0,
	UART_HEADER,
	UART_HEADER2,
	// UART_TYPE,
	UART_LENGTH,
	UART_OPCODE,
	UART_PAYLOAD,
	UART_CHECKSUM,
	UART_RESERVE,
}UART_STATE_ENUM;

void user_send_cmd_response(uint8_t opCode, uint8_t rsp)
{
	// uint8_t sum = 0;
	// uint8_t responseBuffer[32] = {0};
	// responseBuffer[0] = 0x55;
	// responseBuffer[1] = 0x02;
	// responseBuffer[2] = 4;
	// responseBuffer[3] = opCode;
	// responseBuffer[4] = rsp;

	// sum += responseBuffer[2];
	// sum += responseBuffer[3];
	// sum += responseBuffer[4];

	// responseBuffer[5] = sum;

    //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);
	//spear_uart_send_datas(responseBuffer, 6);
}
bool spear_uart_process_data(uint8_t byt)
{
	bool r = false;
    uint8_t data_t[128];
    // uint8_t data_crc1=0;
    // uint8_t data_crc2=0;
	ESP_LOGI(TAG, " %x: ", byt);
	switch(uState)
	{
		case UART_IDLE:
		case UART_HEADER:
			if(byt == 0x5A)
			{
				uState = UART_HEADER2;
				//user_start_cmd_receive(true);
			}
			//ESP_LOGI(TAG, "header = %02x\r\n", byt);
			break;

		case UART_HEADER2:
			if(byt == 0xA5)
			{
				uState = UART_LENGTH;
				//user_start_cmd_receive(true);
			}
			//ESP_LOGI(TAG, "header = %02x\r\n", byt);
			break;

		case UART_LENGTH:
			g_data.len = byt;//-3   no jiaoyan len
			g_data.dIndx = 0;
			
			uState++;
			//ESP_LOGI(TAG, "length = %02x\r\n", byt);
			break;
		case UART_OPCODE:
			g_data.opcode = byt;
			//g_data.sum += byt;
            //g_data.sum = byt;
			uState++;
			//ESP_LOGI(TAG, "opcode = %02x\r\n", byt);
			break;
		case UART_PAYLOAD:



            if(g_data.dIndx == g_data.len-3 )//crc1   2
            {
                g_data.data[g_data.dIndx] = byt;
                //ESP_LOGI(TAG, "crc data[%d] = %02x\r\n",g_data.dIndx, byt);
                g_data.dIndx++;
            }
			else if(g_data.dIndx < g_data.len-3 )//<   1
			{
				g_data.data[g_data.dIndx] = byt;
				//g_data.sum += byt;
				//ESP_LOGI(TAG, "data[%d] = %02x\r\n",g_data.dIndx, byt);
                g_data.dIndx++;
			}
			else//crc2
			{
                g_data.data[g_data.dIndx] = byt;
                // ESP_LOGI(TAG, "crc data[%d] = %02x\r\n",g_data.dIndx, byt);
                g_data.dIndx++;
				//user_start_cmd_receive(false);
				uState = 0;
                data_t[0] = g_data.opcode;
                memcpy(data_t+1,g_data.data,g_data.len -3);

                g_data.sum = CRC16(data_t, g_data.len -3 +1);
				// ESP_LOGI(TAG, "sum = %04x\r\n",g_data.sum);

                // ESP_LOGI(TAG, "g_data.data[g_data.dIndx -2] = %02x, g_data.data[g_data.dIndx-1] = %02x\r\n",g_data.data[g_data.dIndx -2], g_data.data[g_data.dIndx-1]);
                // ESP_LOGI(TAG, "g_data.sum &0xff = %02x, (g_data.sum>>8)&0xff = %02x\r\n",g_data.sum &0xff, (g_data.sum>>8)&0xff);


				if( ((g_data.sum &0xff) == g_data.data[g_data.dIndx -2])
                    &&(((g_data.sum>>8)&0xff) == g_data.data[g_data.dIndx-1]) )//byt
				{
                    ESP_LOGI(TAG,"处理数据1-pass\r\n");
					r = true;
				}
				else
				{
                    ESP_LOGI(TAG,"处理数据1-fail\r\n");
					user_send_cmd_response(g_data.opcode, CMD_FAIL);
				}
			}
			break;
		case UART_RESERVE:
		default:
			break;
	}

	return r;
}



static void echo_task()
{
    
    // Configure a temporary buffer for the incoming data
    //uint8_t *data_rx = (uint8_t *) malloc(BUF_SIZE);

    //ESP_LOGI(TAG, "UART1 start recieve loop.\r\n");

    while (1) {
        //vTaskDelay(10 / portTICK_PERIOD_MS);
        // Read data from the UART
        len_rx = uart_read_bytes(UART_NUM_1, data_rx, BUF_SIZE, 20 / portTICK_RATE_MS);
        len_rx2 = uart_read_bytes(UART_NUM_2, data_rx2, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        
        //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);//len =0 budayin
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

            //uart_write_bytes(UART_NUM_2, (const char *) data_rx2, len_rx2);
        }

								
		


        if (len_rx > 0) {

            // if(len_rx!= g_data.len +3)
            // {
            //     len_rx = g_data.len +3;
            // }

            // if(len_rx!= data_rx[2] +3)
            // {
            //     len_rx = data_rx[2] +3;
            // }

            ESP_LOGI(TAG, "uart1-Received %u bytes:", len_rx);
            for (int i = 0; i < len_rx; i++) {
                printf("0x%.2X ", (uint8_t)data_rx[i]);
                // if(spear_uart_process_data(data_rx[i]))
                // {
                //     ////send event to 
                //     //spear_sched_set_evt(NULL, 0,user_cmd_process_event_handle);

                //     ESP_LOGI(TAG,"处理数据2-start\r\n");
                //     xTaskCreate(echo_task2, "uart_echo_task2",4* 1024, NULL, 2, NULL);
                // }
            }
            printf("] \n");

            //vTaskDelay(2 / portTICK_PERIOD_MS);
            xTaskCreate(echo_task2, "uart_echo_task2",4* 1024, NULL, 2, NULL);

            //echo_task2();

            //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);
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


void uart_init_all(void)
{

    // while(1)
    // {
    //     vTaskDelay(50 / portTICK_PERIOD_MS);

    // }
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,// lcd
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_config_t uart_config2 = {
        .baud_rate = 9600,// lock
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // uart_config_t uart_config3 = {
    //     .baud_rate = 57600,// zhiwen
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity    = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    // };


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



}


void read_nvs_guizi_all()
{
    char key_num[3];//65536
    //todo
    char key_name[15];
    esp_err_t err;

    for(uint16_t i=1;i<=shengyu_all_max;i++)
    {

        //database_gz[i].cunwu_mode_gz = database_cw.cunwu_mode;
        sprintf(key_name, "%03d", i);
        strcpy(key_name+3,"_dz_cw_md");
        // itoa(i,key_num,10);
        // strcat(key_name,"_dz_cw_md");
        printf("--key_name=%s--\r\n",key_name);

        err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[i].cunwu_mode_gz));
        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));



        //database_gz[i].dzx_mode_gz = database_cw.dzx_mode;
        sprintf(key_name, "%03d", i);
        strcpy(key_name+3,"_dz_dzxmd");
        // itoa(i,key_num,10);
        // strcat(key_name,"_dz_dzxmd");
        printf("--key_name=%s--\r\n",key_name);

        err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[i].dzx_mode_gz));
        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));




        // database_cw.phone_number_nvs = atoi((const char*)phone_number);

        // database_gz[i].phone_number_nvs_gz = database_cw.phone_number_nvs;
        sprintf(key_name, "%03d", i);
        strcpy(key_name+3,"_dz_phone");
        // itoa(i,key_num,10);
        // strcat(key_name,"_dz_phone");
        printf("--key_name=%s--\r\n",key_name);

        err = read_u64_value(STORAGE_NAMESPACE,key_name,&database_gz[i].phone_number_nvs_gz);
        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));




        //database_cw.mima_number_nvs = atoi((const char*)mima_number);
        
        //database_gz[i].mima_number_nvs_gz = database_cw.mima_number_nvs;
        sprintf(key_name, "%03d", i);
        strcpy(key_name+3,"_dz_mima");
        // itoa(i,key_num,10);
        // strcat(key_name,"_dz_mima");
        printf("--key_name=%s--\r\n",key_name);

        err = read_u32_value(STORAGE_NAMESPACE,key_name,&database_gz[i].mima_number_nvs_gz);
        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

        



        sprintf(key_name, "%03d", i);
        strcpy(key_name+3,"_dz_st");
        // itoa(i,key_num,10);
        // strcat(key_name,"_dz_st");
        printf("--key_name=%s--\r\n",key_name);

        err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[i].state_gz));
        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
    
        printf("\r\n");
    }


  //admin
    //d
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_D, (uint16_t*)(&shengyu_da));
    if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


    //z
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_Z, (uint16_t*)(&shengyu_zhong));
    if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


    //x
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_X, (uint16_t*)(&shengyu_xiao));
    if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

    shengyu_all = shengyu_da + shengyu_zhong + shengyu_xiao;
    
    printf("---shengyu_all=%d----\n",shengyu_all);



    for(uint16_t i=1;i<=shengyu_all_max;i++)
    {
        printf("shengyu index =%03d,cunwu_mode_gz =%d,dzx_mode_gz =%d,",\
                i, database_gz[i].cunwu_mode_gz,database_gz[i].dzx_mode_gz);


        printf("phone?=%11llu,mima?=%6u,", database_gz[i].phone_number_nvs_gz, database_gz[i].mima_number_nvs_gz);

        printf("state?=%d,lock?=%d,changqi?=%d\r\n",\
                database_gz[i].state_gz,\
                database_gz[i].lock,\
                database_gz[i].changqi);

    }

}


void app_main()
{
    u8 ensure;


    gpio_pad_select_gpio(RE_485_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RE_485_GPIO, GPIO_MODE_OUTPUT);
    
    RS485_RX_EN();



    uart_init_all();
    //vTaskDelay(500 / portTICK_PERIOD_MS);

    //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
    xTaskCreate(echo_task, "uart_echo_task", 2* 1024, NULL, 1, NULL);//1024 10

	
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







    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // //err = print_what_saved();//dayin
    // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
    // if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


    // err = read_u64_value(STORAGE_NAMESPACE,"dw_phone_number",&database_cw.phone_number_nvs);
    // if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


    // err = read_u32_value(STORAGE_NAMESPACE,"dw_mima_number",&database_cw.mima_number_nvs);
    // if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));





    






    read_nvs_guizi_all();



    
    kaiji_huamian();




    // shengyu_da=shengyu_da_max;//read
    // shengyu_zhong=shengyu_zhong_max;
    // shengyu_xiao=shengyu_xiao_max;


    tongbu_gekou_shuliang_d(shengyu_da);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_z(shengyu_zhong);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_x(shengyu_xiao);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    tongbu_gekou_shuliang_all(shengyu_all);



    //debug
    for (uint16_t i = 1; i <= shengyu_da_max; i++)//15
    {
        database_gz[i].dzx_mode_gz =1;
        //database_gz[i].state_gz =0;
    }
    for (uint16_t i = 16; i <= shengyu_zhong_max+15; i++)//10
    {
        database_gz[i].dzx_mode_gz =2;
        //database_gz[i].state_gz =0;
    }
    for (uint16_t i = 26; i <= shengyu_xiao_max+25; i++)//5
    {
        database_gz[i].dzx_mode_gz =3;
        //database_gz[i].state_gz =0;
    }

    // for (uint16_t i = 11; i < 21; i++)
    // {
    //     database_gz[i].dzx_mode_gz =1;
    // }
    // for (uint16_t i = 21; i < 31; i++)
    // {
    //     database_gz[i].dzx_mode_gz =2;
    // }


    //database_gz[0].state =1;

    // err = save_restart_counter();
    //  if (err != ESP_OK) printf("Error (%s) saving restart counter to NVS!\n", esp_err_to_name(err));


}
