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
//#include "time.h"
#include <sys/time.h>
#include "esp_log.h"


#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/queue.h"

#include "AS608.h" 
#include <stdint.h>


#include "nvs_flash.h"
#include "nvs.h"





#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



#include <stdio.h>

#include "esp_log.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_error.h"
#include "tone_stream.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"

#include "board.h"


#include "audio_tone_uri.h"
// #if __has_include("audio_tone_uri.h")
//     #include "audio_tone_uri.h"
// #else
//     #error "please refer the README, and then make the tone file"
// #endif



#include "esp_timer.h"


#include "esp_flash.h"
#include "driver/spi_common_internal.h"
#include "esp_flash_spi_init.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"



#include "esp_wifi.h"

static void smartconfig_example_task(void * parm);
static void lock_all_open_task();
static void lock_all_clear_task();
void tongbu_da(void);
void tongbu_zh(void);
void tongbu_changqi(void);
void tongbu_locked(void);
void audio_init(void);
void close_mp3(void);

TaskHandle_t taskhandle1= NULL;
TaskHandle_t taskhandle_temp= NULL;
TaskHandle_t taskhandle_mp3 = NULL;
TaskHandle_t taskhandle_uart2 = NULL;
esp_timer_handle_t oneshot_timer;
bool wifi_connected_flag;
u8 wifi_peiwang_over_flag;

u16 wifi_led_duration_time=500;


u8 audio_play_mp3_over;


void audio_play_my_mp3(void);
void audio_play_one_mp3(int num);
void simple_ota_example_task(void *pvParameter);
static void event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data);


/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const int WIFI_FAIL_BIT =    BIT2;


// static const char *TAG = "PLAY_MP3_FLASH";

/*
   To embed it in the app binary, the mp3 file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
// low rate mp3 audio


int test_i;
// audio_pipeline_handle_t pipeline;
audio_element_handle_t tone_stream_reader, i2s_stream_writer, mp3_decoder;
audio_pipeline_handle_t pipeline;
audio_event_iface_handle_t m_audio_evt;

void es7134_pa_power(bool enable);







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
#define UART_NUM_LOCK UART_NUM_0



#define ECHO_TEST_TXD  GPIO_NUM_33//32(GPIO_NUM_33)//GPIO_NUM_4
#define ECHO_TEST_RXD  GPIO_NUM_32//33(GPIO_NUM_32)//GPIO_NUM_5
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)


//xin 1
#define ECHO_TEST2_TXD  (GPIO_NUM_23)//2-deng    23     hard UART2 0
#define ECHO_TEST2_RXD  (GPIO_NUM_34)//34        22
#define ECHO_TEST2_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST2_CTS  (UART_PIN_NO_CHANGE)//zhiwen


    #define ECHO_TEST3_TXD  (GPIO_NUM_19)//lock  uart3
    #define ECHO_TEST3_RXD  (GPIO_NUM_4)
    #define ECHO_TEST3_RTS  (UART_PIN_NO_CHANGE)
    #define ECHO_TEST3_CTS  (UART_PIN_NO_CHANGE)
    #define RE_485_GPIO     (GPIO_NUM_18)

//xin 0
// #define ECHO_TEST2_TXD  (GPIO_NUM_23)//2-deng    23     hard UART2 0
// #define ECHO_TEST2_RXD  (GPIO_NUM_34)//34        22
// #define ECHO_TEST2_RTS  (UART_PIN_NO_CHANGE)
// #define ECHO_TEST2_CTS  (UART_PIN_NO_CHANGE)//lock
// #define RE_485_GPIO     (GPIO_NUM_18)

//     #define ECHO_TEST3_TXD  (GPIO_NUM_19)//zhiwen  uart3
//     #define ECHO_TEST3_RXD  (GPIO_NUM_4)
//     #define ECHO_TEST3_RTS  (UART_PIN_NO_CHANGE)
//     #define ECHO_TEST3_CTS  (UART_PIN_NO_CHANGE)




// //old debug
// #define ECHO_TEST2_TXD  (GPIO_NUM_19)//2-deng    23    moni
// #define ECHO_TEST2_RXD  (GPIO_NUM_4)//34        22
// #define ECHO_TEST2_RTS  (UART_PIN_NO_CHANGE)
// #define ECHO_TEST2_CTS  (UART_PIN_NO_CHANGE)//lock   or uart3 uart0
// #define RE_485_GPIO     (GPIO_NUM_18)

//     #define ECHO_TEST3_TXD  (GPIO_NUM_23)//zhiwen 19    hard UART2
//     #define ECHO_TEST3_RXD  (GPIO_NUM_34)//4
//     #define ECHO_TEST3_RTS  (UART_PIN_NO_CHANGE)
//     #define ECHO_TEST3_CTS  (UART_PIN_NO_CHANGE)



    // #define ECHO_TEST4_TXD  (GPIO_NUM_21)
    // #define ECHO_TEST4_RXD  (GPIO_NUM_36)//SVP
    // #define ECHO_TEST4_RTS  (UART_PIN_NO_CHANGE)
    // #define ECHO_TEST4_CTS  (UART_PIN_NO_CHANGE)

#define LED_BLUE     (GPIO_NUM_2)
#define LED_GRREN     (GPIO_NUM_22)
#define LED_RED         (GPIO_NUM_27)


// #define GPIO_INPUT_IO_ZW_2     (4)

#define GPIO_INPUT_IO_ADMIN     39//4
// #define GPIO_INPUT_IO_ZW_JC     35//5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_ADMIN))
#define ESP_INTR_FLAG_DEFAULT 0
// | (1ULL<<GPIO_INPUT_IO_ZW_JC))


bool HandShakeFlag = 0;
bool io_shouzhi_down_flag = 0;


//1024
#define BUF_SIZE (512)

// uint8_t data_rx0[BUF_SIZE] = {0};
// uint16_t len_rx0;

uint8_t data_rx[BUF_SIZE] = {0};
uint16_t len_rx;

uint8_t data_rx0[BUF_SIZE] = {0};//485 dtu debug        data_rx2
int len_rx0;

uint8_t data_rx2_m[BUF_SIZE] = {0};
int len_rx2_m;
uint8_t flag_rx2;


#define TX1_LEN 12//10 tupian

#define TX1_LEN_BL 10//8 bianliang




#define STORAGE_NAMESPACE "storage"//guizi
#define STORAGE_NAMESPACE_ADM "storage_adm"//guizi admin
#define STORAGE_NAMESPACE_USR "storage_usr"//yonghu


//1
#define ADM_KEY_SHENGYU_ALL "ad_shengyu_a"
#define ADM_KEY_SHENGYU_D "ad_shengyu_d"
#define ADM_KEY_SHENGYU_Z "ad_shengyu_z"
#define ADM_KEY_SHENGYU_X "ad_shengyu_x"


#define ADM_KEY_MIMA      "ad_mima"
#define ADM_KEY_SHENGYU_ALL_MAX "ad_sy_all_m"
#define ADM_KEY_SHENGYU_D_MAX "ad_sy_d_m"
#define ADM_KEY_SHENGYU_Z_MAX "ad_sy_z_m"
#define ADM_KEY_SHENGYU_X_MAX "ad_sy_x_m"
#define ADM_KEY_ZW_PAGEID_F "_ad_zwpid"
#define ADM_KEY_MP3_CTL "_ad_mp3_ctl"


//3
#define DZ_CW_MD "_dz_cw_md"//guizi    _dz_cw_md
#define DZ_DZXMD "_dz_dzxmd"
#define DZ_PHONE "_dz_phone"
#define DZ_MIMA  "_dz_mima"
#define DZ_ST    "_dz_st"
#define DZ_LK    "_dz_lk"
#define DZ_LONG_T    "_dz_lt"
#define DZ_INDEX "_dz_idx"
#define DZ_FENPEI "_dz_fp"//<=12
#define DZ_ZW_PAGEID "_dz_zwpid"



//35//25//all kong   18    12   20
#define BOARD_GK_MAX 8

//288//300//310//all kong   480    432
#define SHENYU_GEZI_MAX 192
//288//50

//300//all kong   //120
#define ZHIWEN_PAGE_ID_MAX 300


int16_t hang_shu_max;

// #define GUIMENX_GK_MAX 24//all kong


// int16_t guimen_x_gk_max[25]={12,10,10,16,16,16,16,16,16,16,
//                            16,16,16,16,16,16,16,16,16};//600

// int16_t guimen_x_gk_max[BOARD_GK_MAX];//600=24*25   need save?




//admin   need save   实时更新
//shuliang
uint16_t shengyu_all=0;//
uint16_t shengyu_da=1;
uint16_t shengyu_zhong=15;
uint16_t shengyu_xiao=100;
u8 audio_play_mp3_stop;

bool audio_play_mp3_task;

//admin 
//and cun de yong
uint16_t shengyu_all_max=0;//shengyu max admin, guding

    uint16_t shengyu_da_max=0;//
    uint16_t shengyu_zhong_max=0;
    uint16_t shengyu_xiao_max=0;



char wifi_ssid[33] = { 0 };     /* 定义一个数组用来存储ssid*/
char wifi_passwd[65] = { 0 };   /* 定义一个数组用来存储passwd */





//1 admin
typedef struct
{
    uint32_t mima_number_adm;  //
    
    bool zhiwen_page_id_adm[ZHIWEN_PAGE_ID_MAX];//flag
    // uint16_t zhiwen_gz_index[300];//gz
    
    // uint8_t shengyu1;
    // uint8_t shengyu2;


}shujuku_struct_admin;

shujuku_struct_admin database_ad=
{
    .mima_number_adm = 666888,
    // .shengyu1 = 0,
};




//3
typedef struct
{
        //格口编号 箱门-nolock
        uint16_t dIndx_gz;//add
        uint8_t state_fenpei_gz; //fenpei 是否
        //daxiao
        uint8_t dzx_mode_gz;//default


        bool lock;

        uint8_t changqi;

        uint8_t state_gz; //zaiyong 是否

    //zhiwen mima
    uint8_t cunwu_mode_gz;

    //yonghu_zhiwen_id
    uint16_t zhiwen_page_id_gz;

    //yonghu xin xi
    uint64_t phone_number_nvs_gz;  //
    uint32_t mima_number_nvs_gz;  //



}shujuku_struct_gz;
//={0,0,0,3};//柜子

shujuku_struct_gz database_gz[SHENYU_GEZI_MAX+1];//i/lock  idx
// shujuku_struct_gz database_gz[SHENYU_GEZI_MAX]=
// {
//     .state_gz = 0 ,
//     .dzx_mode_gz = 3,
// };


// //zhiwen mima
// uint8_t cunwu_mode;
// //leixing
// uint8_t dzx_mode=00;

uint8_t phone_weishu_ok;
uint8_t phone_weishu_ok_a;

uint8_t mima_weishu_ok_a1;

//lock 格口编号
//weiyi xuhao

//donn't need save todo


//2 temp
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
	uint8_t state;

    uint8_t changqi_tmp;


    //lock编号 箱门-nolock
	uint16_t dIndx;//dangqian rcv

    //编号 箱门-nolock
	//uint16_t tmp_dIndx_gz;//dangqian rcv

    //zhiwen index
    uint16_t zhiwen_page_id;//page id temp


    //取物唯一编号 time
    uint16_t unique_number;
}shujuku_struct_user;//用户

//shujuku_struct_user database_cw;
shujuku_struct_user database_cw=
{
    .dzx_mode = 0 ,
};

shujuku_struct_user database_cw_adm=
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




uint16_t find_lock_index(uint16_t guimen_index)
{
    uint16_t lock_index=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        if((database_gz[i].state_fenpei_gz == 1)
            &&(database_gz[i].dIndx_gz == guimen_index))
        {
            lock_index = i;
            DB_PR("--ok--lock_index=%d--.\r\n",lock_index); 
            return lock_index;
        }
        
    }
    DB_PR("--err--lock_index no find--.\r\n"); 
    return 0;
}

uint16_t find_pid_lock_idx(uint16_t temp_index)
{
    uint16_t lock_index=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        if(database_gz[i].zhiwen_page_id_gz == temp_index)
        {
            lock_index = i;
            DB_PR("-zw-ok--lock_index=%d--.\r\n",lock_index); 
            return lock_index;
        }
        
    }
    DB_PR("-zw-err--lock_index no find--.\r\n"); 
    return 0;
}





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

    DB_PR("w_r8-%s=%u\r\n",key,(uint8_t)(out_value_t));

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


    DB_PR("rd8-%s=%u\r\n\r\n",key,(uint8_t)(*out_value));

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

    DB_PR("w_r16-%s=%u\r\n",key,(uint16_t)(out_value_t));

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


    DB_PR("rd16-%s=%u\r\n\r\n",key,(uint16_t)(*out_value));

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

    DB_PR("wr_32-%s=%u\r\n",key,(out_value_t));

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


    DB_PR("r32-%s=%u\r\n\r\n",key,(*out_value));

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

    DB_PR("wr_64-%s=%llu\r\n",key,(out_value_t));

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


    DB_PR("r64-%s=%llu\r\n\r\n",key,(*out_value));

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}



//str todo
esp_err_t save_str_value(const char* name,char* key, char* out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // DB_PR("-----0------%s\r\n",key);
    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // DB_PR("-----1------%s\r\n",key);

    // // Read
    // char* out_value_t=NULL; // value will default to 0, if not set yet in NVS
    // size_t* len=NULL;
    // err = nvs_get_str(my_handle, key, out_value_t,len);
    // if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // DB_PR("1_to_wr_str-%s=%s\r\n",key,(out_value_t));

    DB_PR("2_to_rw_str-%s=%s\r\n",key,out_value);
    // Write
    err = nvs_set_str(my_handle, key, out_value);
    if (err != ESP_OK) return err;


    // DB_PR("-----2------%s\r\n",key);
    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;


    // DB_PR("-----3------%s\r\n",key);
    // Close
    nvs_close(my_handle);
    return ESP_OK;

        // nvs_handle_t wificfg_nvs_handler;
        // ESP_ERROR_CHECK( nvs_open(STORAGE_NAMESPACE_ADM, NVS_READWRITE, &wificfg_nvs_handler) );
        // ESP_ERROR_CHECK( nvs_set_str(wificfg_nvs_handler,"wifi_ssid",ssid) );
        // ESP_ERROR_CHECK( nvs_set_str(wificfg_nvs_handler,"wifi_passwd",password) );
        // ESP_ERROR_CHECK( nvs_commit(wificfg_nvs_handler) ); /* 提交 */
        // nvs_close(wificfg_nvs_handler);                     /* 关闭 */ 
        // DB_PR("smartconfig save wifi_cfg to NVS .\n");
}

//todo--out_value---
esp_err_t read_str_value(const char* name,char* key, char* out_value,  size_t* len)
{
    nvs_handle_t my_handle;

    esp_err_t err;

    // Open
    err = nvs_open(name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    // size_t required_size;
    err =nvs_get_str(my_handle, key, NULL,len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    out_value = malloc(*len);

    err = nvs_get_str(my_handle, key, out_value,len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;


    DB_PR("rdstr-%s=%s\r\n\r\n",key,out_value);

    // Close
    nvs_close(my_handle);
    return ESP_OK;




    // /* 打开一个NVS命名空间 */
    // ESP_ERROR_CHECK( nvs_open(STORAGE_NAMESPACE_ADM, NVS_READWRITE, &wificfg_nvs_handler) );
    // len = sizeof(wifi_ssid);    /* 从NVS中获取ssid */
    // ESP_ERROR_CHECK( nvs_get_str(wificfg_nvs_handler,"wifi_ssid",wifi_ssid,&len) );
    // len = sizeof(wifi_passwd);      /* 从NVS中获取ssid */
    // ESP_ERROR_CHECK( nvs_get_str(wificfg_nvs_handler,"wifi_passwd",wifi_passwd,&len) );
    // ESP_ERROR_CHECK( nvs_commit(wificfg_nvs_handler) ); /* 提交 */
    // nvs_close(wificfg_nvs_handler);                     /* 关闭 */






}





void Add_FR_CQ();
void del_zw_database(u16 num);
#define usart2_baund  57600//串口2波特率，根据指纹模块波特率更改

SysPara AS608Para;//指纹模块AS608参数
u16 ValidN;//模块内有效指纹个数
u8** kbd_tbl;
void Add_FR_First(void);	//录指纹
void Add_FR(void);	//录指纹
void Del_FR(u16 num);	//删除指纹
void press_FR(void);//刷指纹
void ShowErrMessage(u8 ensure);//显示确认码错误信息

// u16 zhiwen_num_id;//page id


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
#define RS485_RX_EN()			delay_ms(70); gpio_set_level(RE_485_GPIO, 0);delay_ms(70);//rx;  RS485_delay(1000);
//????·???????,±??????????±????485???í?ê????
#define RS485_TX_EN()			delay_ms(70); gpio_set_level(RE_485_GPIO, 1);delay_ms(70);//rx;  RS485_delay(1000);



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

void uart0_debug_str(uint8_t* str,uint16_t len)
{
    DB_PR("--1str---debug_str:");
    for(uint8_t i=0;i<len;i++)
        DB_PR("%c ",str[i]);
    DB_PR("\r\n");
}
void uart0_debug_data(uint8_t* data,uint16_t len)
{
    DB_PR("---2----debug_data:");
    for(int i=0;i<len;i++)
        DB_PR("%02x ",data[i]);
    DB_PR("\r\n");
}
void uart0_debug_data_d(uint8_t* data,uint16_t len)
{
    DB_PR("---3---debug_data:");
    for(int i=0;i<len;i++)
        DB_PR("%02d ",data[i]);
    DB_PR("\r\n");
}

//2字节
void uart0_debug_data_dec(uint16_t* data,uint16_t len)//16
{
    DB_PR("----4----debug_data:");
    for(int i=0;i<len;i++)
        DB_PR("%02d ",data[i]);
    DB_PR("\r\n");
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

    DB_PR("shengyu_all_1:%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    DB_PR("tx1 CRC16 result:0x%04X\r\n",crc16_temp);

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

    DB_PR("shengyu_da1:%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    DB_PR("tx1 CRC16 result:0x%04X\r\n",crc16_temp);

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
    DB_PR("shengyu_zhong1:%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    DB_PR("tx2 CRC16 result:0x%04X\r\n",crc16_temp);

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
    DB_PR("shengyu_xiao1:%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    DB_PR("tx3 CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[8] = crc16_temp&0xff;
    tx_Buffer[9] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN_BL);

}


#define  BL_XM_SZ 0x11A0
#define  BL_GK_SZ_D 0x1600
#define  BL_GK_SZ_Z 0x1700
#define  BL_GK_SZ_X 0x1800


#define  BL_GK_BH_D 0x1400//0x1240
#define  BL_GK_BH_Z 0x1300//0x1230
#define  BL_GK_BH_CHANGQI 0x1500//0x1210
#define  BL_GK_BH_LOCKED 0x1900//add

#define  BL_GK_BH_MAX_LEN 0xFC//


#define  BL_GK_BH_D_LEN 0xFC//0xCE//0x23//(0xCD)
//0x132//306

#define  BL_GK_BH_Z_LEN 0xFC//0x23

#define  BL_GK_BH_CHANGQI_LEN 0xFC//(0xCD)

#define  BL_GK_BH_LOCKED_LEN 0xFC//(0xCD)
//0xCD//0xCE

//数组
void send_cmd_to_lcd_bl_len(uint16_t opCode, uint8_t* buff_temp,uint16_t data_len)//变量
{
    uint8_t tx_Buffer[256]={0};  
    uint16_t crc16_temp=0;
    //xiao
    tx_Buffer[0] = 0x5A;
    tx_Buffer[1] = 0xA5;
    tx_Buffer[2] = data_len;//len  
    tx_Buffer[3] = 0x82;

    tx_Buffer[4] = opCode/256;
    tx_Buffer[5] = opCode%256;//dizhi

    DB_PR("-------data_len=%d--------\r\n",data_len);
    for (int i = 0; i < data_len-2 ; i++) {
        tx_Buffer[6+i] = buff_temp[i];
        //DB_PR("0x%.2X ", (uint8_t)buff_temp[i]);
    }
    DB_PR("\r\n");


    //crc
    crc16_temp = CRC16(tx_Buffer+3, data_len-2);
    DB_PR("tx CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[3+ data_len-2 ] = crc16_temp&0xff;
    tx_Buffer[3+ data_len-2 +1] = (crc16_temp>>8)&0xff;
    DB_PR("---------debug1---------\r\n");
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, 3+ data_len);
    DB_PR("---------debug2---------\r\n");
    uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
}

//2个字节
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
    DB_PR("temp-bl:0x%04d\r\n",temp);
    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN_BL -5);
    DB_PR("tx CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[8] = crc16_temp&0xff;
    tx_Buffer[9] = (crc16_temp>>8)&0xff;
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN_BL);

    uart0_debug_data(tx_Buffer, TX1_LEN_BL);
}


#define  MUSIC_OFF_PIC 0x002d//
#define  MUSIC_ON_PIC 0x002e//


#define  FAIL_XMH_PIC 0x0015//

#define  FAIL_CHANGQI_PIC 0x0019//
#define  CHANQI_CW_MODE_PIC 0x001A//
#define  XIN_CHANGQI_OK_PIC 0x001c//ok
#define  UN_CHANGQI_OK_PIC 0x003f//ok


#define  XIN_MIMA_ADMIN_OK_PIC 0x0038//ok
#define  FAIL_MIMA_ADMIN_OK_PIC 0x0037//


#define  DEFAULT_ADMIN_OK_PIC 0x003a//ok

#define  ALLOPEN_OK_PIC 0x003d//ok

#define  CLEAR_ONE_OK_PIC 0x0024//ok
#define  CLEAR_ALL_OK_PIC 0x003e//ok


#define  LK_OK_PIC 0x002A//ok
#define  UNLK_OK_PIC 0x002C//ok

#define  BOOT_PIC 0x0041//qushezhi->guimen
#define  GUIMEN_OK_PIC 0x0031//ok

#define  KAIJI_PIC 0x0026
#define  ADMIN_LOGIN_PIC 0x000f


#define  GEKOU_PIC 0x0032//da
#define  GEKOU_Z_PIC 0x0033//zhong
#define  GEKOU_X_PIC 0x0034//xiao
#define  GEKOU_OK_PIC 0x0035//ok



void send_cmd_to_lcd_pic(uint16_t temp)//图片
{
    uint8_t tx_Buffer[50]={0};  
    uint16_t crc16_temp=0;

    DB_PR("-----pic-----.\r\n");
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
    DB_PR("temp-pic:0x%04x\r\n",temp);

    //crc
    crc16_temp = CRC16(tx_Buffer+3, TX1_LEN - 5);
    DB_PR("tx CRC16 result:0x%04X\r\n",crc16_temp);

    tx_Buffer[10] = crc16_temp&0xff;
    tx_Buffer[11] = (crc16_temp>>8)&0xff;
    
    uart_write_bytes(UART_NUM_1, (const char *) tx_Buffer, TX1_LEN);

    uart0_debug_data(tx_Buffer, TX1_LEN);
}


void send_cmd_to_lock(uint8_t board_addr, uint8_t lock_addr)//变量
{
    uint8_t tx_Buffer2[50]={0};  
    uint8_t bcc_temp=0;
    memcpy(tx_Buffer2,"star",4);
    tx_Buffer2[4]= 0x8A;//m_data.opcode;
    tx_Buffer2[5]= (uint8_t)board_addr;//m_data.board_addr;
    tx_Buffer2[6]= (uint8_t)lock_addr;//m_data.lock_addr;
    tx_Buffer2[7]= 0x11;//guding
    bcc_temp = ComputXor(tx_Buffer2+4,4);
    tx_Buffer2[8]= bcc_temp;
    memcpy(tx_Buffer2+9,"endo",4);

    tx_Buffer2[13]='\0';

    RS485_TX_EN();

    DB_PR("tx_Buffer2=");
    uart0_debug_data(tx_Buffer2, 13);
    uart_write_bytes(UART_NUM_LOCK, (const char *) tx_Buffer2, 13);
    RS485_RX_EN();

}


//2 2.1
void send_cmd_to_lock_all(uint8_t opcode, uint8_t board_addr)//变量
{
    uint8_t tx_Buffer2[50]={0};  
    uint8_t bcc_temp=0;
    memcpy(tx_Buffer2,"star",4);
    tx_Buffer2[4]= opcode;//m_data.opcode;
    tx_Buffer2[5]= (uint8_t)board_addr;//m_data.board_addr;
    bcc_temp = ComputXor(tx_Buffer2+4,2);
    tx_Buffer2[6]= bcc_temp;
    memcpy(tx_Buffer2+7,"endo",4);

    tx_Buffer2[11]='\0';

    RS485_TX_EN();

    DB_PR("tx_Buffer2=");
    uart0_debug_data(tx_Buffer2, 11);
    uart_write_bytes(UART_NUM_LOCK, (const char *) tx_Buffer2, 11);
    RS485_RX_EN();

}


// void lcd_send_cmd_response(uint8_t rsp)
// {

// }





//-------------------guizi--------------------
void nvs_wr_cunwu_mode_gz(uint8_t mode)//1 write; 0 read
{
    char key_name[15];//15
    esp_err_t err;

    //database_gz[database_cw.dIndx].cunwu_mode_gz = database_cw.cunwu_mode;
    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_CW_MD);
    DB_PR("--key_name=%s--\r\n",key_name);

    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].cunwu_mode_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].cunwu_mode_gz));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}


void nvs_wr_dzx_mode_gz(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;
    //database_gz[database_cw.dIndx].dzx_mode_gz = database_cw.dzx_mode;
    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_DZXMD);//dz_dzxmd
    DB_PR("--key_name=%s--\r\n",key_name);
    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].dzx_mode_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].dzx_mode_gz));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


}

void nvs_wr_phone_number_nvs_gz(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;

    //database_cw.phone_number_nvs = atoll((const char*)phone_number);

    //database_gz[database_cw.dIndx].phone_number_nvs_gz = database_cw.phone_number_nvs;
    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_PHONE);
    DB_PR("--key_name=%s--\r\n",key_name);
    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u64_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].phone_number_nvs_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u64_value(STORAGE_NAMESPACE,key_name,&database_gz[database_cw.dIndx].phone_number_nvs_gz);
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


}

void nvs_wr_mima_number_nvs_gz(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;
    //database_cw.mima_number_nvs = atoi((const char*)mima_number);

    //database_gz[database_cw.dIndx].mima_number_nvs_gz = database_cw.mima_number_nvs;
    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_MIMA);
    DB_PR("--key_name=%s--\r\n",key_name);
    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u32_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].mima_number_nvs_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u32_value(STORAGE_NAMESPACE,key_name,&database_gz[database_cw.dIndx].mima_number_nvs_gz);
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


}

void nvs_wr_state_gz(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;

    //database_gz[database_cw.dIndx].state_gz =database_cw.state;
    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_ST);
    DB_PR("--key_name=%s--\r\n",key_name);
    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].state_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].state_gz));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}


void nvs_wr_glock_gz(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;

    //database_gz[database_cw.dIndx].state_gz =database_cw.state;
    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_LK);
    DB_PR("--key_name=%s--\r\n",key_name);
    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].lock);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].lock));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}

void nvs_wr_glongtime_gz(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;

    //database_gz[database_cw.dIndx].state_gz =database_cw.state;
    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_LONG_T);
    DB_PR("--key_name=%s--\r\n",key_name);
    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].changqi);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].changqi));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}







///////////////////////
void nvs_wr_index_gz(uint8_t mode)//1 write; 0 read
{
    char key_name[15];//15
    esp_err_t err;

    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_INDEX);
    DB_PR("--key_name=%s--\r\n",key_name);

    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].dIndx_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u16_value(STORAGE_NAMESPACE,key_name, (uint16_t*)(&database_gz[database_cw.dIndx].dIndx_gz));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}

void nvs_wr_fenpei_gz(uint8_t mode)//1 write; 0 read
{
    char key_name[15];//15
    esp_err_t err;

    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_FENPEI);
    DB_PR("--key_name=%s--\r\n",key_name);

    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].state_fenpei_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE,key_name, (uint8_t*)(&database_gz[database_cw.dIndx].state_fenpei_gz));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}

//zw pageid
void nvs_wr_zw_pageid_gz(uint8_t mode)//1 write; 0 read
{
    char key_name[15];//15
    esp_err_t err;

    sprintf(key_name, "%03d%s", database_cw.dIndx,DZ_ZW_PAGEID);
    DB_PR("--key_name=%s--\r\n",key_name);

    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE,key_name,database_gz[database_cw.dIndx].zhiwen_page_id_gz);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u16_value(STORAGE_NAMESPACE,key_name, (uint16_t*)(&database_gz[database_cw.dIndx].zhiwen_page_id_gz));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}



 //--------------admin----------------

void nvs_wr_shengyu_da(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //d
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_D,shengyu_da);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

    }

    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_D, (uint16_t*)(&shengyu_da));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}

void nvs_wr_shengyu_zhong(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //z
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_Z,shengyu_zhong);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

    }

    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_Z, (uint16_t*)(&shengyu_zhong));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

}


void nvs_wr_shengyu_xiao(uint8_t mode)//->all
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //x
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_X,shengyu_xiao);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_X, (uint16_t*)(&shengyu_xiao));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
}



void nvs_wr_mima_number_adm(uint8_t mode)
{
    char key_name[15];//15
    esp_err_t err;

    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u32_value(STORAGE_NAMESPACE_ADM,ADM_KEY_MIMA,database_ad.mima_number_adm);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u32_value(STORAGE_NAMESPACE_ADM,ADM_KEY_MIMA,&database_ad.mima_number_adm);
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


}


//--------------------------duoyu------------------------------
void nvs_wr_shengyu_all_max(uint8_t mode)//->all
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //x
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_ALL_MAX,shengyu_all_max);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_ALL_MAX, (uint16_t*)(&shengyu_all_max));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
}


void nvs_wr_shengyu_da_max(uint8_t mode)//->all
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //x
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_D_MAX,shengyu_da_max);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_D_MAX, (uint16_t*)(&shengyu_da_max));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
}

void nvs_wr_shengyu_zhong_max(uint8_t mode)//->all
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //x
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_Z_MAX,shengyu_zhong_max);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_Z_MAX, (uint16_t*)(&shengyu_zhong_max));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
}

//-----duoyu---no----
void nvs_wr_shengyu_xiao_max(uint8_t mode)//->all
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //x
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_X_MAX,shengyu_xiao_max);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u16_value(STORAGE_NAMESPACE_ADM,ADM_KEY_SHENGYU_X_MAX, (uint16_t*)(&shengyu_xiao_max));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
}

//page id flag
void nvs_wr_adm_zwpageid_flag(uint8_t mode,uint16_t zhiwen_page_id_temp)//->all
{
    char key_name[15];//15
    esp_err_t err;
    sprintf(key_name, "%03d%s", zhiwen_page_id_temp,ADM_KEY_ZW_PAGEID_F);
    DB_PR("--key_name=%s--\r\n",key_name);

    if(mode == 1)
    {
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE_ADM,key_name,database_ad.zhiwen_page_id_adm[zhiwen_page_id_temp]);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE_ADM,key_name, (uint8_t*)(&database_ad.zhiwen_page_id_adm[zhiwen_page_id_temp]));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
}

//
void nvs_wr_mp3_ctl(uint8_t mode)//->all
{
    char key_name[15];//15
    esp_err_t err;
    if(mode == 1)
    {
        //x
        //DB_PR("--key_name=%s--\r\n",NAMESPACE_ADM_KEY_SHENGYU_D);
        DB_PR("--write--\r\n");
        err = save_u8_value(STORAGE_NAMESPACE_ADM,ADM_KEY_MP3_CTL,audio_play_mp3_stop);
        if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));
    }
    err = read_u8_value(STORAGE_NAMESPACE_ADM,ADM_KEY_MP3_CTL, (uint16_t*)(&audio_play_mp3_stop));
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
}





uint8_t buff_t[100]={0};
void inputBox_clear(void)
{
    send_cmd_to_lcd_bl_len(0x11A0,(uint8_t*)buff_t,30*4+5);//gekou
    send_cmd_to_lcd_bl_len(BL_GK_SZ_D,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
    send_cmd_to_lcd_bl_len(BL_GK_SZ_Z,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
    send_cmd_to_lcd_bl_len(BL_GK_SZ_X,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
    send_cmd_to_lcd_bl_len(0x10c0,(uint8_t*)buff_t,2*2+5);
    send_cmd_to_lcd_bl_len(0x1130,(uint8_t*)buff_t,2*2+5);//
    send_cmd_to_lcd_bl_len(0x1150,(uint8_t*)buff_t,2*2+5);//
    send_cmd_to_lcd_bl_len(0x1180,(uint8_t*)buff_t,2*2+5);//
    send_cmd_to_lcd_bl_len(0x10e0,(uint8_t*)buff_t,2*2+5);//


    send_cmd_to_lcd_bl_len(0x1100,(uint8_t*)buff_t,30+5);//phone
    send_cmd_to_lcd_bl_len(0x1110,(uint8_t*)buff_t,30+5);//key del
    send_cmd_to_lcd_bl_len(0x1120,(uint8_t*)buff_t,2*2+5);//

    send_cmd_to_lcd_bl_len(0x11e0,(uint8_t*)buff_t,30+5);//key
    send_cmd_to_lcd_bl_len(0x11f0,(uint8_t*)buff_t,30+5);//key2

    send_cmd_to_lcd_bl_len(0x1050,(uint8_t*)buff_t,30+5);//phone
    send_cmd_to_lcd_bl_len(0x1060,(uint8_t*)buff_t,30+5);//key del

    send_cmd_to_lcd_bl_len(0x1080,(uint8_t*)buff_t,30+5);//phone
    send_cmd_to_lcd_bl_len(0x1090,(uint8_t*)buff_t,30+5);//key
    send_cmd_to_lcd_bl_len(0x10B0,(uint8_t*)buff_t,30+5);//key
}

void default_factory_set(void)
{
    //wifi mima erase
    bzero(wifi_ssid, sizeof(wifi_ssid));
    bzero(wifi_passwd, sizeof(wifi_passwd));

    DB_PR( "-w-SSID:%s\r\n", wifi_ssid);
    DB_PR( "-w-PASSWORD:%s\r\n", wifi_passwd);
    /* 将得到的WiFi名称和密码存入NVS*/
    esp_err_t err = save_str_value(STORAGE_NAMESPACE,"wifi_ssid",wifi_ssid );
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

    err = save_str_value(STORAGE_NAMESPACE,"wifi_passwd",wifi_passwd );
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));


    u8  ensure;
	ensure=PS_Empty();//清空指纹库
	if(ensure==0)
	{
		//LCD_Fill(0,120,lcddev.width,160,WHITE);
		DB_PR("---del all zhiwen ok -----删除指纹成功 \r\n");		
	}
    else
		ShowErrMessage(ensure);	

        
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        database_cw.dIndx = i;


        database_gz[database_cw.dIndx].dIndx_gz =0;
        // database_gz[database_cw.dIndx].state_fenpei_gz =0;

        nvs_wr_index_gz(1);
        // nvs_wr_fenpei_gz(1);


        database_gz[database_cw.dIndx].changqi = 0;
        database_gz[database_cw.dIndx].lock = 0;
        nvs_wr_glock_gz(1);
        nvs_wr_glongtime_gz(1);


        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
        database_gz[database_cw.dIndx].dzx_mode_gz = 3;
        nvs_wr_cunwu_mode_gz(1);
        nvs_wr_dzx_mode_gz(1);

        database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
        database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
        nvs_wr_phone_number_nvs_gz(1);
        nvs_wr_mima_number_nvs_gz(1);

        database_gz[database_cw.dIndx].state_gz =0;
        nvs_wr_state_gz(1);

        database_gz[database_cw.dIndx].zhiwen_page_id_gz =0;
        nvs_wr_zw_pageid_gz(1);

    }

    for(uint16_t i=0;i<AS608Para.PS_max;i++)
    {
        database_ad.zhiwen_page_id_adm[i] =0;
        nvs_wr_adm_zwpageid_flag(1,i);
    }

    database_ad.mima_number_adm =666888;
    //adm
    nvs_wr_mima_number_adm(1);

    DB_PR("---database_ad.mima_number_adm=%d----\n",database_ad.mima_number_adm);

    shengyu_da =0;
    shengyu_zhong =0;
    shengyu_xiao =0;
    nvs_wr_shengyu_da(1);
    nvs_wr_shengyu_zhong(1);
    nvs_wr_shengyu_xiao(1);


    shengyu_all = shengyu_da + shengyu_zhong + shengyu_xiao;


    tongbu_gekou_shuliang_all(shengyu_all);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_d(shengyu_da);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_z(shengyu_zhong);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_x(shengyu_xiao);
    //vTaskDelay(10 / portTICK_PERIOD_MS);





    // DB_PR("---shengyu_all=%d----\n",shengyu_all);
    // DB_PR("---shengyu_da=%d----\n",shengyu_da);
    // DB_PR("---shengyu_zhong=%d----\n",shengyu_zhong);
    // DB_PR("---shengyu_xiao=%d----\n",shengyu_xiao);

    shengyu_da_max =0;
    shengyu_zhong_max =0;
    shengyu_xiao_max =0;
    shengyu_all_max = 0;


    nvs_wr_shengyu_all_max(1);//duoyu
    nvs_wr_shengyu_da_max(1);
    nvs_wr_shengyu_zhong_max(1);
    nvs_wr_shengyu_xiao_max(1);




    // //shengyu_all_max = shengyu_da_max + shengyu_zhong_max + shengyu_xiao_max;
    DB_PR("-2-shengyu_all_max=%d----\n",shengyu_all_max);
    // DB_PR("---shengyu_da_max=%d----\n",shengyu_da_max);
    // DB_PR("---shengyu_zhong_max=%d----\n",shengyu_zhong_max);
    // DB_PR("---shengyu_xiao_max=%d----\n",shengyu_xiao_max);
    tongbu_da();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_zh();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_changqi();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_locked();   


    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        database_cw.dIndx = i;
        database_gz[database_cw.dIndx].state_fenpei_gz =0;
        nvs_wr_fenpei_gz(1);
    }

    audio_play_mp3_stop=0;
    nvs_wr_mp3_ctl(1);

    inputBox_clear();
}


void default_factory_set_first(void)
{
    //wifi mima erase
    bzero(wifi_ssid, sizeof(wifi_ssid));
    bzero(wifi_passwd, sizeof(wifi_passwd));

    DB_PR( "-w-SSID:%s\r\n", wifi_ssid);
    DB_PR( "-w-PASSWORD:%s\r\n", wifi_passwd);
    /* 将得到的WiFi名称和密码存入NVS*/
    esp_err_t err = save_str_value(STORAGE_NAMESPACE,"wifi_ssid",wifi_ssid );
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

    err = save_str_value(STORAGE_NAMESPACE,"wifi_passwd",wifi_passwd );
    if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));



    u8  ensure;
	ensure=PS_Empty();//清空指纹库
	if(ensure==0)
	{
		//LCD_Fill(0,120,lcddev.width,160,WHITE);
		DB_PR("---del all zhiwen ok -----删除指纹成功 \r\n");		
	}
    else
		ShowErrMessage(ensure);	



    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        database_cw.dIndx = i;


        // database_gz[database_cw.dIndx].dIndx_gz =0;
        database_gz[database_cw.dIndx].state_fenpei_gz =0;

        // nvs_wr_index_gz(1);
        nvs_wr_fenpei_gz(1);


        database_gz[database_cw.dIndx].changqi = 0;
        database_gz[database_cw.dIndx].lock = 0;
        nvs_wr_glock_gz(1);
        nvs_wr_glongtime_gz(1);


        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
        database_gz[database_cw.dIndx].dzx_mode_gz = 3;
        nvs_wr_cunwu_mode_gz(1);
        nvs_wr_dzx_mode_gz(1);

        // database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
        // database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
        // nvs_wr_phone_number_nvs_gz(1);
        // nvs_wr_mima_number_nvs_gz(1);

        // database_gz[database_cw.dIndx].state_gz =0;
        // nvs_wr_state_gz(1);

        database_gz[database_cw.dIndx].zhiwen_page_id_gz =0;
        nvs_wr_zw_pageid_gz(1);

    }

    for(uint16_t i=0;i<AS608Para.PS_max;i++)
    {
        database_ad.zhiwen_page_id_adm[i] =0;
        nvs_wr_adm_zwpageid_flag(1,i);
    }

    database_ad.mima_number_adm =666888;
    //adm
    nvs_wr_mima_number_adm(1);

    DB_PR("---database_ad.mima_number_adm=%d----\n",database_ad.mima_number_adm);

    shengyu_da =0;
    shengyu_zhong =0;
    shengyu_xiao =0;
    nvs_wr_shengyu_da(1);
    nvs_wr_shengyu_zhong(1);
    nvs_wr_shengyu_xiao(1);


    shengyu_all = shengyu_da + shengyu_zhong + shengyu_xiao;


    tongbu_gekou_shuliang_all(shengyu_all);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_d(shengyu_da);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_z(shengyu_zhong);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_x(shengyu_xiao);
    //vTaskDelay(10 / portTICK_PERIOD_MS);





    // DB_PR("---shengyu_all=%d----\n",shengyu_all);
    // DB_PR("---shengyu_da=%d----\n",shengyu_da);
    // DB_PR("---shengyu_zhong=%d----\n",shengyu_zhong);
    // DB_PR("---shengyu_xiao=%d----\n",shengyu_xiao);

    shengyu_da_max =0;
    shengyu_zhong_max =0;
    shengyu_xiao_max =0;
    shengyu_all_max = 0;


    nvs_wr_shengyu_all_max(1);//duoyu
    nvs_wr_shengyu_da_max(1);
    nvs_wr_shengyu_zhong_max(1);
    nvs_wr_shengyu_xiao_max(1);




    // //shengyu_all_max = shengyu_da_max + shengyu_zhong_max + shengyu_xiao_max;
    DB_PR("-2-shengyu_all_max=%d----\n",shengyu_all_max);
    // DB_PR("---shengyu_da_max=%d----\n",shengyu_da_max);
    // DB_PR("---shengyu_zhong_max=%d----\n",shengyu_zhong_max);
    // DB_PR("---shengyu_xiao_max=%d----\n",shengyu_xiao_max);

    audio_play_mp3_stop=0;
    nvs_wr_mp3_ctl(1);

    inputBox_clear();
}







void tongbu_da(void)
{
    u16 buff_temp1[SHENYU_GEZI_MAX]={0};

    u8 buff_temp1_c[SHENYU_GEZI_MAX]={0};//char


    uint16_t k=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        if(1== database_gz[i].state_fenpei_gz)
        {

            if(1== database_gz[i].dzx_mode_gz)
            {
                //char *	_EXFUN(itoa,(int, char *, int));

                buff_temp1[k] = database_gz[i].dIndx_gz;
                DB_PR("b_temp1[k]= %03d ",buff_temp1[k]);//xiangmenhao
                
                itoa(buff_temp1[k],(char*)(buff_temp1_c+4*(k)),10);//+4*(i-1)
                k++;

                DB_PR("\r\n");
            }

            
        }

        // DB_PR("---i=%d\r\n",i);

    }
    //vTaskDelay(10 / portTICK_PERIOD_MS);

    // uart0_debug_data_d(buff_temp1,0x9b);
    // uart0_debug_data_d(buff_temp1,0x9b);


    for(uint16_t i=0;i<SHENYU_GEZI_MAX;i++)
    {
        if(buff_temp1_c[i]==0)
        {
            buff_temp1_c[i]=0x20;
            DB_PR("kongd ");
        }
    }


    // vTaskDelay(5 / portTICK_PERIOD_MS);
    DB_PR("-----gekouleixing-d-----\r\n");
    send_cmd_to_lcd_bl_len(BL_GK_BH_D,buff_temp1_c,BL_GK_BH_D_LEN);//300
    vTaskDelay(1 / portTICK_PERIOD_MS);

}

void tongbu_zh(void)
{
    u16 buff_temp2[SHENYU_GEZI_MAX]={0};
    u8 buff_temp2_c[SHENYU_GEZI_MAX]={0};//150

    uint16_t l=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        if(1== database_gz[i].state_fenpei_gz)
        {

            if(2== database_gz[i].dzx_mode_gz)
            {
                buff_temp2[l] = database_gz[i].dIndx_gz;
                DB_PR("b_temp2[l]= %03d ",buff_temp2[l]);//xiangmenhao
                itoa(buff_temp2[l],(char*)(buff_temp2_c+4*(l)),10);//+4*(i-1)
                l++;
                DB_PR("\r\n");
            }

            
        }

        //DB_PR("---i=%d\r\n",i);

    }
    //vTaskDelay(10 / portTICK_PERIOD_MS);

    // uart0_debug_data_d(buff_temp1,0x9b);
    // uart0_debug_data_d(buff_temp1,0x9b);


    for(uint16_t i=0;i<SHENYU_GEZI_MAX;i++)
    {
        if(buff_temp2_c[i]==0)
        {
            DB_PR("kongz ");
            buff_temp2_c[i]=0x20;
        }
    }


    // vTaskDelay(5 / portTICK_PERIOD_MS);
    DB_PR("-----gekouleixing-z-----\r\n");
    send_cmd_to_lcd_bl_len(BL_GK_BH_Z,buff_temp2_c,BL_GK_BH_Z_LEN);//300 0x23 30
    vTaskDelay(1 / portTICK_PERIOD_MS);


}


void tongbu_changqi(void)
{
    u16 buff_temp2[SHENYU_GEZI_MAX]={0};
    u8 buff_temp2_c[SHENYU_GEZI_MAX]={0};//150

    uint16_t l=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        //vTaskDelay(1);
        if(1== database_gz[i].state_fenpei_gz)
        {

            if((1== database_gz[i].changqi)
                ||(2== database_gz[i].changqi))
            {
                buff_temp2[l] = database_gz[i].dIndx_gz;
                DB_PR("b_temp2[l]= %03d ",buff_temp2[l]);//xiangmenhao
                itoa(buff_temp2[l],(char*)(buff_temp2_c+4*(l)),10);//+4*(i-1)
                l++;
                DB_PR("\r\n");
            }

        }

        //DB_PR("---i=%d\r\n",i);

    }
    //vTaskDelay(10 / portTICK_PERIOD_MS);


    for(uint16_t i=0;i<SHENYU_GEZI_MAX;i++)
    {
        //vTaskDelay(1);
        if(buff_temp2_c[i]==0)
        {
            buff_temp2_c[i]=0x20;
            DB_PR("kong ");
        }
    }


    // vTaskDelay(10 / portTICK_PERIOD_MS);
    DB_PR("-----changqi--sync----\r\n");
    send_cmd_to_lcd_bl_len(BL_GK_BH_CHANGQI,buff_temp2_c,BL_GK_BH_CHANGQI_LEN);//300 0x23 30
    vTaskDelay(1);


}


void tongbu_locked(void)
{
    u16 buff_temp2[SHENYU_GEZI_MAX]={0};
    u8 buff_temp2_c[SHENYU_GEZI_MAX]={0};//150

    uint16_t l=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        //vTaskDelay(1);
        if(1== database_gz[i].state_fenpei_gz)
        {

            if(1== database_gz[i].lock)
            {
                buff_temp2[l] = database_gz[i].dIndx_gz;
                DB_PR("b_temp2[l]= %03d ",buff_temp2[l]);//xiangmenhao
                itoa(buff_temp2[l],(char*)(buff_temp2_c+4*(l)),10);//+4*(i-1)
                l++;
                DB_PR("\r\n");
            }

        }

        //DB_PR("---i=%d\r\n",i);

    }
    //vTaskDelay(10 / portTICK_PERIOD_MS);


    for(uint16_t i=0;i<SHENYU_GEZI_MAX;i++)
    {
        //vTaskDelay(1);
        if(buff_temp2_c[i]==0)
        {
            buff_temp2_c[i]=0x20;
            DB_PR("kong ");
        }
    }


    // vTaskDelay(10 / portTICK_PERIOD_MS);
    DB_PR("-----changqi--sync----\r\n");
    send_cmd_to_lcd_bl_len(BL_GK_BH_LOCKED,buff_temp2_c,BL_GK_BH_LOCKED_LEN);//300 0x23 30
    vTaskDelay(1);


}



uint8_t phone_number[11]={0};  
uint8_t mima_number[6]={0xff, 0xff,0xff,0xff,0xff,0xff};  


uint8_t phone_number_a[11]={0};  
uint8_t mima_number_a[6]={0};  

uint8_t mima_number_a1[6]={0};  
uint8_t mima_number_a2[6]={0};  


uint8_t return_cause;//xiangmen fail

uint8_t return_cause_zanwu_kx;
uint8_t return_cause_has_be_lock;

uint8_t return_cause_zw;
uint8_t return_cause_phone;

uint8_t return_cause_zw_fail;
uint8_t return_cause_zw_handshake_fail;

static void echo_task2()//lcd
{
    uint8_t buff_t[256]={0};
    uint16_t bl_addr=0;//bianliang lcd
    uint16_t crc16_temp=0;
    
    uint8_t data_rx_t[BUF_SIZE] = {0};
    uint16_t len_rx_t= len_rx;
    int32_t guimen_gk_temp =0;


    memcpy(data_rx_t,data_rx,len_rx_t);
    //while(1)
    {
        //vTaskDelay(40 / portTICK_PERIOD_MS);


								
		


        if (len_rx_t > 0) {
            // DB_PR("uart1-Received %u bytes:", len_rx_t);
            // for (int i = 0; i < len_rx_t; i++) {
            //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
            // }
            // DB_PR("] \n");
            crc16_temp = CRC16(data_rx_t+3, data_rx_t[2] -2);
            //DB_PR("rx CRC16 result:0x%04X\r\n",crc16_temp);
            



            // DB_PR("data_rx_t[0]:0x%02X, data_rx_t[1]:0x%02X\r\n",data_rx_t[0], data_rx_t[1]);

            // DB_PR("(len_rx_t-3):0x%02X,data_rx_t[2]:0x%02X\r\n",(len_rx_t-3),data_rx_t[2]);

            // DB_PR("data_rx_t[data_rx_t[2] +2 -1]0x%02X\r\n",data_rx_t[data_rx_t[2] +2 -1]);
            // DB_PR("(crc16_temp & 0xff):0x%02X\r\n",(crc16_temp & 0xff));

            // DB_PR("data_rx_t[data_rx_t[2] +2 -1+1]0x%02X\r\n",data_rx_t[data_rx_t[2] +2 -1+1]);
            // DB_PR("((crc16_temp>>8) & 0xff):0x%02X\r\n",((crc16_temp>>8) & 0xff));

            
            if((0x5A == data_rx_t[0])
                &&(0xA5 == data_rx_t[1])
                &&((len_rx_t-3) == data_rx_t[2])
                &&(data_rx_t[data_rx_t[2]+2-1]==(crc16_temp&0xff))
                &&(data_rx_t[data_rx_t[2]+2-1+1]==((crc16_temp>>8)&0xff)))
                {
                    bl_addr = (data_rx_t[4]<<8) + data_rx_t[5];
                    DB_PR("-----.bl_addr=%04x\r\n",bl_addr);
                    switch (data_rx_t[3])
                    {
                    // case 0x82:
                    //     //uart_write_bytes(UART_NUM_2, (const char *) (data_rx_t+4), len_rx_t-4);
                        
                    //     bl_addr = (data_rx_t[4]<<8) + data_rx_t[5];
                    //     DB_PR("--0x82--.bl_addr=%04x\r\n",bl_addr);

                    //     uint8_t tx_Buffer[50]={0};  
                    //     uint8_t bcc_temp=0;
                    //     switch (bl_addr)
                    //     {

                    //     case 0x4F4B:
                    //         DB_PR("---lcd---ok-----.\r\n");
                    //         //todo
                    //         break;

                    //     default:
                    //         DB_PR("----------------83 default---------------.\r\n");
                    //         break;
                    //     }

                    //     break;

                    case 0x83:
                        if( data_rx_t[6] == (len_rx_t-7 -2)/2)
                        {
                            //uart_write_bytes(UART_NUM_2, (const char *) (data_rx_t+4), len_rx_t-4);
                            bl_addr = (data_rx_t[4]<<8) + data_rx_t[5];
                            DB_PR("--0x83--.bl_addr=%04x\r\n",bl_addr);

                            //uint8_t tx_Buffer[50]={0};  
                            uint8_t tx_Buffer2[200]={0};  
                            uint8_t Buffer2_ok[200]={0};  
                            uint8_t bcc_temp=0;
                            switch (bl_addr)
                            {




//-------------------admin--------------guimenshezhi-----------2次------------------------------------------
                            case 0x11A0://
                                DB_PR("--kaijishezhi--.\r\n");   
                                int j=0;
                                for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                    DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                    tx_Buffer2[j]=data_rx_t[i];
                                    // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                    // if(tx_Buffer2[j] != 0x2D)
                                    j++;
                                }
                                DB_PR("\r\n");
                                
                                send_cmd_to_lcd_bl_len(0x11A0,(uint8_t*)buff_t,30*4+5);//
                                if(data_rx_t[2] == 0x6A)//0x9c
                                {
                                    

                                    // u8 temp_s_flag=0;
                                    // send_cmd_to_lcd_bl(0x11A0,0);
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    int j = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;

                                    while('-' == *p)    
                                    {
                                        *p =0;
                                        p=p+1;
                                        q=p;
                                        //DB_PR("0-a *p=%02x\r\n",*p);//0x2d
                                    }
                                    DB_PR("0-b *p=%02x\r\n\r\n",*p);//0x2d

                                    while( NULL != (p = strchr(p,'-')))    
                                    {
                                        
                                        if(*(p-1) == '-')
                                        {
                                            DB_PR("0-c\r\n\r\n");//0x2d
                                            *(p-1)='\0';
                                            p=p+1;
                                            q=p;
                                            continue;
                                        }
                                        
                                        
                                        // temp_s_flag =1;
                                        strncpy(show[i],q,p-q);
                                        show[i][p-q] = '\0';

                                        DB_PR("1 *p=%02x\r\n",*p);//0x2d
                                        DB_PR("1 i=%02d\r\n",i);//0x2d
                                        
                                        p = p+1;
                                        // j++;
                                        // if( NULL == ((p-1) = strchr(p-1,'-')))
                                        // if()
                                        {
                                            DB_PR("*(p-2)=%02x\r\n",*(p-2));//0x2d
                                            q = p;
                                            i ++;
                                        }  

                                        DB_PR("2 *p=%02x\r\n",*p);//0x2d
                                        DB_PR("2 i=%02d\r\n\r\n",i);//0x2d
                                        // else
                                        // {
                                        //     DB_PR("2p-1=%02x\r\n",*(p-1));//0x2d
                                        //     DB_PR("2p-1=%c\r\n",*(p-1));//0x2d
                                        // }
                                        
                                        

                                    }

                                    if (p == NULL)
                                    {
                                        DB_PR("3-----\r\n\r\n");//0x2d
                                        strncpy(show[i],q,strlen(q));
                                        show[i][strlen(q)] = '\0';
                                    }

                                    hang_shu_max =i;//hang shu

                                    DB_PR("show0=\r\n");
                                    for(j = 0; j <= strlen(show[hang_shu_max]); j++)//最后一行
                                    {
                                        if(0xff == show[hang_shu_max][j])
                                        {
                                            show[hang_shu_max][j] = '\0';
                                        }

                                        DB_PR("%02x-",show[hang_shu_max][j]);
                                    }

                                    DB_PR("\r\nshow1=\r\n");
                                    for(j = 0; j <= hang_shu_max; j++)
                                    {
                                        DB_PR("j=%s\r\n",show[j]);
                                    }
                                    DB_PR("hang_shu_max=%03d\r\n",hang_shu_max);//0

                                    DB_PR("\r\n");




                                    
                                    uint16_t shengyu_all_max_temp=0;//shengyu max admin, guding
                                    int16_t guimen_x_gk_max_temp[BOARD_GK_MAX]={0};

                                    DB_PR("hang_shu_max+1=%03d\r\n",hang_shu_max+1);//0

  



                                    if((hang_shu_max+1)> BOARD_GK_MAX)
                                    {
                                        DB_PR("-----err2 >BOARD_GK_MAX lock------\r\n");
                                        //send_cmd_to_lcd_pic(0x004F);
                                        // break;
                                        goto guimen_set_fail;
                                    }

                                    int gm_one_num=0;
                                    if(hang_shu_max< BOARD_GK_MAX)//300/24 =12.5
                                    {
                                        DB_PR("show2=\r\n");
                                        for(j = 0; j < hang_shu_max+1; j++)//i 个柜子
                                        {
                                            gm_one_num = atoi((const char*)show[j]);
                                            DB_PR("-1-gm_one_num = %05d\r\n\r\n",gm_one_num);
                                            if(gm_one_num<0)
                                            {
                                                DB_PR("-----xmh < 0------\r\n");
                                                guimen_x_gk_max_temp[j] =24;
                                            }
                                            else if(gm_one_num>=24)
                                            {
                                                DB_PR("-----xmh >24------\r\n");
                                                guimen_x_gk_max_temp[j] =24;
                                            }
                                            else if((gm_one_num>0)&&(gm_one_num<24))//==0?
                                            {
                                                DB_PR("-----xmh == 0 -24------\r\n");
                                                guimen_x_gk_max_temp[j] =gm_one_num;//normal
                                            }
                                            else if(gm_one_num ==0)
                                            {
                                                DB_PR("-----xmh == 0------\r\n");
                                                // continue;
                                                guimen_x_gk_max_temp[j] =0;
                                            }
                                            

                                            
                                            DB_PR("-2-guimen_x_gk_max_temp[j] = %05d\r\n\r\n",guimen_x_gk_max_temp[j]);

                                            for(int k=1; k<= guimen_x_gk_max_temp[j]; k++)//列
                                            {
                                                database_gz[j*24 + k].state_fenpei_gz = 1;
                                                DB_PR("-1-lock index=%03d\r\n",j*24 + k);
                                            }
                                            // if(guimen_x_gk_max_temp[12] > 12)
                                            // {
                                            //     guimen_x_gk_max_temp[12] =12;
                                            //     DB_PR("-----err1-1 >12gm 12 lock------\r\n");

                                            // }
                                            if((guimen_x_gk_max_temp[j] >=0)
                                                &&(guimen_x_gk_max_temp[j] <24))
                                            {
                                                for(int k=guimen_x_gk_max_temp[j]+1; k<=24; k++)//列
                                                {
                                                    database_gz[j*24 + k].state_fenpei_gz = 0;
                                                    database_gz[j*24 + k].dIndx_gz =0;
                                                    DB_PR("-2-lock index=%03d\r\n",j*24 + k);
                                                }
                                            }

                                            shengyu_all_max_temp = shengyu_all_max_temp +guimen_x_gk_max_temp[j];
                                        }

                                        for(j = hang_shu_max+1; j < BOARD_GK_MAX; j++)//i 个柜子 =
                                        {
                                            //guimen_x_gk_max_temp[j] =0;
                                            for(int k=1; k<= 24; k++)//列
                                            {
                                                if(database_gz[j*24 + k].state_fenpei_gz == 1)
                                                {
                                                    database_gz[j*24 + k].state_fenpei_gz = 0;
                                                    database_gz[j*24 + k].dIndx_gz =0;
                                                    DB_PR("-3-lock index=%03d\r\n",j*24 + k);
                                                }

                                            }

                                        }



                                        // if((hang_shu_max+1) == 13)
                                        // {
                                        //     if(guimen_x_gk_max_temp[12] > 12)
                                        //     {
                                        //         DB_PR("-----err1-2 >12gm 12 lock------\r\n");
                                        //         //send_cmd_to_lcd_pic(0x004F);
                                        //         // break;
                                        //         goto guimen_set_fail;
                                        //     }
                                        //     else
                                        //     {
                                        //         DB_PR("-----ok <12gm 12 lock------\r\n");
                                        //     }
                                            
                                        // }

                                        DB_PR("\r\n");


                                        DB_PR("1-------shengyu_all_max_temp=%03d\r\n",shengyu_all_max_temp);
                                        if(shengyu_all_max_temp==0)
                                        {
                                            DB_PR("-----------shengyu_all_max_temp =0--------------\r\n");
                                            send_cmd_to_lcd_pic(0x0052);
                                            break;
                                        }

                                        if((shengyu_all_max_temp>0)
                                            &&(shengyu_all_max_temp<=SHENYU_GEZI_MAX))
                                        {
                                            DB_PR("2-------shengyu_all_max_temp=%03d\r\n",shengyu_all_max_temp);
                                            shengyu_all_max = shengyu_all_max_temp;
                                            // memcpy(guimen_x_gk_max,guimen_x_gk_max_temp,BOARD_GK_MAX);//????????

                                            // DB_PR("2-hang_shu_max=%03d\r\n",hang_shu_max);
                                            // uart0_debug_data_d(guimen_x_gk_max,BOARD_GK_MAX);
                                            DB_PR("3-hang_shu_max=%03d\r\n",hang_shu_max);
                                            uart0_debug_data_d(guimen_x_gk_max_temp,BOARD_GK_MAX);






                                            // u16 buff_temp1[SHENYU_GEZI_MAX]={0};
                                            // u16 buff_temp2[SHENYU_GEZI_MAX]={0};

                                            // u8 buff_temp1_c[400]={0};//char
                                            // u8 buff_temp2_c[400]={0};//150
                                            uint16_t id=0,iz=0,ix=0;
                                            uint16_t id_max=0,iz_max=0,ix_max=0;
                                            uint16_t changqi_num_temp=0;
                                            j=0;

            



                                            for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
                                            {
                                                database_cw.dIndx = i;
                                                // DB_PR("fenpei?=%d, ", database_gz[i].state_fenpei_gz);
                                                
                                                // if(database_gz[i].state_fenpei_gz ==1)
                                                nvs_wr_fenpei_gz(1);//init is above
                                                
                                                
                                                if(database_gz[i].state_fenpei_gz == 1)
                                                {
                                                    j++;
                                                    database_gz[i].dIndx_gz = j;
                                                    database_cw.dIndx = i;
            

                                                    DB_PR("-2-lock index =%03d, ",i);
                                                    DB_PR("-2-xmh index j= %03d",j);
                                                    DB_PR("\r\n");
                                                    nvs_wr_index_gz(1);
                
     

                                                    if(database_gz[i].dzx_mode_gz ==1)
                                                    {
                                                        // buff_temp1[id_max] = database_gz[i].dIndx_gz;
                                                        // DB_PR("b_temp1[id_max]= %03d ",buff_temp1[id_max]);//xiangmenhao
                                                        
                                                        // itoa(buff_temp1[id_max],(char*)(buff_temp1_c+4*(id_max)),10);//+4*(i-1)

                                                        id_max++;
                                                        if((database_gz[i].state_gz ==0)
                                                            &&(database_gz[i].changqi ==0)
                                                            &&(database_gz[i].lock ==0))
                                                        {
                                                            id++;
                                                        }
                                                    }
                                                    else if(database_gz[i].dzx_mode_gz ==2)
                                                    {
                                                        // buff_temp2[iz_max] = database_gz[i].dIndx_gz;
                                                        // DB_PR("b_temp2[iz_max]= %03d ",buff_temp2[iz_max]);//xiangmenhao
                                                        // itoa(buff_temp2[iz_max],(char*)(buff_temp2_c+4*(iz_max)),10);//+4*(i-1)

                                                        iz_max++;
                                                        if((database_gz[i].state_gz ==0)
                                                            &&(database_gz[i].changqi ==0)
                                                            &&(database_gz[i].lock ==0))
                                                        {
                                                            iz++;
                                                        }
                                                    }
                                                    else if(database_gz[i].dzx_mode_gz ==3)
                                                    {
                                                        ix_max++;
                                                        if((database_gz[i].state_gz ==0)
                                                            &&(database_gz[i].changqi ==0)
                                                            &&(database_gz[i].lock ==0))
                                                        {
                                                            ix++;
                                                        }
                                                    }

                                                    //lock
                                                    //changqi
                                                    // if(database_gz[i].changqi ==1)
                                                    // {
                                                    //     changqi_num_temp++;
                                                    // }

                                                    
                                                }
                                                else// if(database_gz[i].state_fenpei_gz == 0)
                                                {
                                                    DB_PR("-----clear other 2----\r\n");
                                                    if(database_gz[database_cw.dIndx].dIndx_gz !=0)
                                                    {
                                                        database_gz[database_cw.dIndx].dIndx_gz =0;
                                                        // database_gz[database_cw.dIndx].state_fenpei_gz =0;
                                                        nvs_wr_index_gz(1);
                                                    }


                                                    if( database_gz[database_cw.dIndx].changqi !=0)
                                                    {
                                                        database_gz[database_cw.dIndx].changqi = 0;
                                                        nvs_wr_glongtime_gz(1);
                                                    }

                                                    if( database_gz[database_cw.dIndx].lock!=0)
                                                    {
                                                        database_gz[database_cw.dIndx].lock = 0;
                                                        nvs_wr_glock_gz(1);
                                                    }

                                                    if( database_gz[database_cw.dIndx].cunwu_mode_gz!=0)
                                                    {
                                                        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                                        nvs_wr_cunwu_mode_gz(1);
                                   
                                                    }

                                                    if( database_gz[database_cw.dIndx].dzx_mode_gz!=3)
                                                    {
                                                        database_gz[database_cw.dIndx].dzx_mode_gz = 3;//todo
                                                        nvs_wr_dzx_mode_gz(1);
                                                    }

                                                    if( (database_gz[database_cw.dIndx].phone_number_nvs_gz!=0)
                                                        ||( database_gz[database_cw.dIndx].mima_number_nvs_gz != 0))
                                                    {
                                                        database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                                        database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                                        nvs_wr_phone_number_nvs_gz(1);
                                                        nvs_wr_mima_number_nvs_gz(1);
                                                    }


                                                    if( database_gz[database_cw.dIndx].state_gz!=0)
                                                    {
                                                        database_gz[database_cw.dIndx].state_gz =0;
                                                        nvs_wr_state_gz(1);
                                                    }



                                                    database_cw.zhiwen_page_id =database_gz[database_cw.dIndx].zhiwen_page_id_gz;
                                                    if(( database_cw.zhiwen_page_id!=0)
                                                        ||(database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id]!=0))
                                                    {
                                                        Del_FR(database_cw.zhiwen_page_id);

                                                        database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id] =0;
                                                        nvs_wr_adm_zwpageid_flag(1,database_cw.zhiwen_page_id);

                                                        database_gz[database_cw.dIndx].zhiwen_page_id_gz =0;
                                                        nvs_wr_zw_pageid_gz(1);
                                                    }


                                                }
                                       
                                                
                                            }




                                            shengyu_all_max =  id_max+ iz_max+ ix_max;
                                            shengyu_xiao_max =ix_max;
                                            shengyu_zhong_max=iz_max;
                                            shengyu_da_max =id_max;

                                            shengyu_all =id+ iz +ix;
                                            shengyu_xiao = ix;
                                            shengyu_zhong = iz;
                                            shengyu_da = id;

                                            DB_PR("---shengyu_all=%d----\n",shengyu_all);

                                            DB_PR("---shengyu_da=%d----\n",shengyu_da);
                                            DB_PR("---shengyu_zhong=%d----\n",shengyu_zhong);
                                            DB_PR("---shengyu_xiao=%d----\n",shengyu_xiao);


                                            //shengyu_all_max = shengyu_da_max + shengyu_zhong_max + shengyu_xiao_max;
                                            DB_PR("-2-shengyu_all_max=%d----\n",shengyu_all_max);
                                            DB_PR("---shengyu_da_max=%d----\n",shengyu_da_max);
                                            DB_PR("---shengyu_zhong_max=%d----\n",shengyu_zhong_max);
                                            DB_PR("---shengyu_xiao_max=%d----\n",shengyu_xiao_max);

                                            nvs_wr_shengyu_all_max(1);
                                            nvs_wr_shengyu_da_max(1);
                                            nvs_wr_shengyu_zhong_max(1);
                                            nvs_wr_shengyu_xiao_max(1);

                                            nvs_wr_shengyu_da(1);
                                            nvs_wr_shengyu_zhong(1);
                                            nvs_wr_shengyu_xiao(1);

                                            tongbu_gekou_shuliang_all(shengyu_all);//
                                            //vTaskDelay(10 / portTICK_PERIOD_MS);
                                            tongbu_gekou_shuliang_d(shengyu_da);
                                            //vTaskDelay(10 / portTICK_PERIOD_MS);
                                            tongbu_gekou_shuliang_z(shengyu_zhong);
                                            //vTaskDelay(10 / portTICK_PERIOD_MS);
                                            tongbu_gekou_shuliang_x(shengyu_xiao);
                                            //vTaskDelay(10 / portTICK_PERIOD_MS);

                                            tongbu_da();
                                            //vTaskDelay(1530 / portTICK_PERIOD_MS);
                                            tongbu_zh();

                                            tongbu_changqi();

                                            tongbu_locked();   
                                            send_cmd_to_lcd_pic(GUIMEN_OK_PIC);

                                            
                                        }
                                        else
                                        {
guimen_set_fail:
                                            send_cmd_to_lcd_pic(0x004F);
                                            DB_PR("--input shuliang =err--.\r\n");   
                                        }
                                        
                                        //save

                                        //memset(tx_Buffer2,0,200);
                                        //send_cmd_to_lcd_bl_len(BL_XM_SZ,tx_Buffer2,data_rx_t[2]-1);//clear

                                        //send_cmd_to_lcd_pic(GEKOU_PIC);


                                    }
                                    else
                                    {
                                        DB_PR("--input shuliang =0--.\r\n");   
                                    }
                                    
                                    
                                }
                                else
                                {
                                    DB_PR("--hangshu cuowu--.\r\n");   
                                }
                                
                                
                                
                                //todo 每次设置不是all

                                break;





                            //-------2------格口设置 dazhongxiao
                            //da
                            case BL_GK_SZ_D://
                                
                                DB_PR("--da gekou--.\r\n");   
                                j=0;
                                for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                    DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                    tx_Buffer2[j]=data_rx_t[i];
                                    // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                    // if(tx_Buffer2[j] != 0x2D)
                                    j++;
                                }
                                DB_PR("\r\n");
                                hang_shu_max =0;
                                if(data_rx_t[2] == 0xCE)//0xFC)//0xCE)//0x9c)
                                {
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;
                                    while('-' == *p)    
                                    {
                                        *p =0;
                                        p=p+1;
                                        q=p;
                                        //DB_PR("0-a *p=%02x\r\n",*p);//0x2d
                                    }
                                    DB_PR("0-b *p=%02x\r\n\r\n",*p);//0x2d

                                    while( NULL != (p = strchr(p,'-')))    
                                    {
                                        
                                        if(*(p-1) == '-')
                                        {
                                            DB_PR("0-c\r\n\r\n");//0x2d
                                            *(p-1)='\0';
                                            p=p+1;
                                            q=p;
                                            continue;
                                        }
                                        strncpy(show[i],q,p-q);
                                        show[i][p-q] = '\0';

                                        //DB_PR("%02x\r\n",*p);//0x2d
                                        p = p+1;
                                        q = p;
                                        i ++;
                                    }

                                    if (p == NULL)
                                    {
                                        strncpy(show[i],q,strlen(q));
                                        show[i][strlen(q)] = '\0';
                                    }

                                    hang_shu_max =i;//hang shu
                                    DB_PR("show1=");
                                    for(j = 0; j <= hang_shu_max; j++)
                                    {
                                        DB_PR("%s\r\n",show[j]);
                                    }

                                    for(j = 0; j <= strlen(show[hang_shu_max]); j++)//最后一行
                                    {
                                        if(0xff == show[hang_shu_max][j])
                                            show[hang_shu_max][j] = '\0';
                                        DB_PR("%02x-",show[hang_shu_max][j]);
                                    }
                                    DB_PR("\r\n");


                                    DB_PR("*show[0]=%d\r\n",* show[0]);
                                    if(*show[0] ==0)
                                    {
                                        DB_PR("da_set_over\r\n");
                                        send_cmd_to_lcd_pic(GEKOU_Z_PIC);
                                        break;
                                    }

                                    bool over_flag=0;//<=0 or>shengyu_all_max
                                    DB_PR("hang_shu_max=%03d\r\n",hang_shu_max);
                                    if((hang_shu_max<= 0x96))//
                                    {
                                        DB_PR("show2=");
                                        for(j = 0; j <= hang_shu_max; j++)//i 个柜子
                                        {

                                            guimen_gk_temp = atoi((const char*)show[j]);
                                            DB_PR("---1----guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 
                          
                                            if((guimen_gk_temp>shengyu_all_max)
                                                ||(guimen_gk_temp<=0))
                                            {
                                                continue;
                                                DB_PR("--gekou overflow ,tiaoguo--.\r\n");   
                                                
                                            }

                                            DB_PR("---2----guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 


                                            // if((guimen_gk_temp >=SHENYU_GEZI_MAX)||(0==guimen_gk_temp))
                                            // {
                                            //     DB_PR("--gekou set fail--.\r\n");   
                                            //     goto gekou_fail;

                                            // }
                                            database_cw.dIndx =0;
                                            database_cw.dIndx = find_lock_index(guimen_gk_temp);

                                            DB_PR("---2----lock--dIndx=%d--.\r\n",database_cw.dIndx); 
                                            if(database_cw.dIndx !=0)
                                            {
                                                DB_PR("--gekou set one ok--.\r\n");   
                                                switch(database_gz[database_cw.dIndx].dzx_mode_gz)
                                                {
                                                case 1:
                                                    // no gai
                                                //     shengyu_da_max++;
                                                //     shengyu_zhong_max--;
                                                //     //shengyu_xiao_max--;
                                                //     shengyu_da ++;
                                                //     shengyu_zhong--;
                                                //     //shengyu_xiao --;

                                                    break;
                                                case 2:
                                                    shengyu_da_max++;
                                                    shengyu_zhong_max--;
                                                    //shengyu_xiao_max--;

                                                    if((database_gz[database_cw.dIndx].state_gz ==0) //no use
                                                        &&(database_gz[database_cw.dIndx].lock == 0)
                                                        &&(database_gz[database_cw.dIndx].changqi == 0))
                                                    {
                                                        DB_PR("--no use--.\r\n");   
                                                        shengyu_da ++;
                                                        shengyu_zhong--;
                                                    }
                                                    else
                                                    {
                                                        DB_PR("--on use--.\r\n");   
                                                    }
                                                    
                                                    

                                                    //shengyu_xiao --;
                                                    break;
                                                case 3:
                                                    shengyu_da_max++;
                                                    //shengyu_zhong_max--;
                                                    shengyu_xiao_max--;

                                                    if((database_gz[database_cw.dIndx].state_gz ==0) //no use
                                                        &&(database_gz[database_cw.dIndx].lock == 0)
                                                        &&(database_gz[database_cw.dIndx].changqi == 0))
                                                    {
                                                        DB_PR("--no use--.\r\n");   

                                                        shengyu_da ++;
                                                        //shengyu_zhong--;
                                                        shengyu_xiao --;
                                                    }
                                                    else
                                                    {
                                                        DB_PR("--on use--.\r\n");   
                                                    }

                                                    break;
        
                                                default:
                                                    DB_PR("--other--.\r\n");   
                                                }
                                                DB_PR("---shengyu_all=%d----\n",shengyu_all);

                                                DB_PR("---shengyu_da=%d----\n",shengyu_da);
                                                DB_PR("---shengyu_zhong=%d----\n",shengyu_zhong);
                                                DB_PR("---shengyu_xiao=%d----\n",shengyu_xiao);


                                                //shengyu_all_max = shengyu_da_max + shengyu_zhong_max + shengyu_xiao_max;
                                                DB_PR("-2-shengyu_all_max=%d----\n",shengyu_all_max);
                                                DB_PR("---shengyu_da_max=%d----\n",shengyu_da_max);
                                                DB_PR("---shengyu_zhong_max=%d----\n",shengyu_zhong_max);
                                                DB_PR("---shengyu_xiao_max=%d----\n",shengyu_xiao_max);





                                                database_gz[database_cw.dIndx].dzx_mode_gz = 1;
                                                nvs_wr_dzx_mode_gz(1);
                                                
                                                DB_PR("database_cw.dIndx = %03d\r\n\r\n",database_cw.dIndx);
                                            }
                                            else
                                            {
                                                DB_PR("--gekou set one fail2 d--.\r\n");   
                                                //goto gekou_fail;
                                            }
                                            
                                        }
                                        DB_PR("-------ts1-------\r\n");
                                        DB_PR("shengyu_all_max=%03d\r\n",shengyu_all_max);

     
                                        nvs_wr_shengyu_da_max(1);
                                        nvs_wr_shengyu_zhong_max(1);
                                        nvs_wr_shengyu_xiao_max(1);
                                        //nvs_wr_shengyu_all_max(1);

                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);

                                        tongbu_gekou_shuliang_all(shengyu_all);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_d(shengyu_da);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_z(shengyu_zhong);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_x(shengyu_xiao);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);


                                        tongbu_da();
                                        tongbu_zh();
                                        
                                    }


                                    //save

                                    memset(tx_Buffer2,0,200);
                                    //send_cmd_to_lcd_bl_len(BL_GK_SZ_D,tx_Buffer2,data_rx_t[2]-1);//clear


                                    send_cmd_to_lcd_pic(GEKOU_Z_PIC);

                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_D,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
                                }
                                else
                                {
//da
// gekou_fail:
                                    //send_cmd_to_lcd_pic(GEKOU_PIC);
                                    DB_PR("-2-gekou fail--.\r\n");   
                                }
                                
                                break;







                            //-------------格口设置 dazhongxiao
                            //zh
                            case BL_GK_SZ_Z://
                                DB_PR("--zh gekou--.\r\n");   
                                
                                j=0;
                                for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                    DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                    tx_Buffer2[j]=data_rx_t[i];
                                    // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                    // if(tx_Buffer2[j] != 0x2D)
                                    j++;
                                }
                                DB_PR("\r\n");

                                if(data_rx_t[2] == 0xCE)//0x9c)
                                {
                                    
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;
                                    while('-' == *p)    
                                    {
                                        *p =0;
                                        p=p+1;
                                        q=p;
                                        //DB_PR("0-a *p=%02x\r\n",*p);//0x2d
                                    }
                                    DB_PR("0-b *p=%02x\r\n\r\n",*p);//0x2d

                                    while( NULL != (p = strchr(p,'-')))    
                                    {
                                        
                                        if(*(p-1) == '-')
                                        {
                                            DB_PR("0-c\r\n\r\n");//0x2d
                                            *(p-1)='\0';
                                            p=p+1;
                                            q=p;
                                            continue;
                                        }
                                        strncpy(show[i],q,p-q);
                                        show[i][p-q] = '\0';

                                        //DB_PR("%02x\r\n",*p);//0x2d
                                        p = p+1;
                                        q = p;
                                        i ++;
                                    }

                                    if (p == NULL)
                                    {
                                        strncpy(show[i],q,strlen(q));
                                        show[i][strlen(q)] = '\0';
                                    }

                                    hang_shu_max =i;//hang shu
                                    DB_PR("show1=");
                                    for(j = 0; j <= hang_shu_max; j++)
                                    {
                                        DB_PR("%s\r\n",show[j]);
                                    }

                                    for(j = 0; j <= strlen(show[hang_shu_max]); j++)//最后一行
                                    {
                                        if(0xff == show[hang_shu_max][j])
                                            show[hang_shu_max][j] = '\0';
                                        DB_PR("%02x-",show[hang_shu_max][j]);
                                    }
                                    DB_PR("\r\n");

                                    DB_PR("*show[0]=%d\r\n",* show[0]);
                                    if(*show[0] ==0)
                                    {
                                        DB_PR("da_set_over\r\n");
                                        send_cmd_to_lcd_pic(GEKOU_X_PIC);
                                        break;
                                    }



                                    DB_PR("hang_shu_max=%03d\r\n",hang_shu_max);
                                    if((hang_shu_max<= 0x96))//
                                    {
                                        DB_PR("show2=");
                                        for(j = 0; j <= hang_shu_max; j++)//i 个柜子
                                        {

                                            guimen_gk_temp = atoi((const char*)show[j]);
                                            DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                            if((guimen_gk_temp>shengyu_all_max)
                                                ||(guimen_gk_temp<=0))
                                            {
                                                continue;
                                                DB_PR("--gekou overflow ,tiaoguo--.\r\n");   
                                                
                                            }
                                            DB_PR("---2----guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                            // if((guimen_gk_temp >=SHENYU_GEZI_MAX)||(0==guimen_gk_temp))
                                            // {
                                            //     DB_PR("--gekou set fail--.\r\n");   
                                            //     goto gekou_fail_z;

                                            // }
                                            database_cw.dIndx =0;
                                            database_cw.dIndx = find_lock_index(guimen_gk_temp);

                                            DB_PR("---2----lock--dIndx=%d--.\r\n",database_cw.dIndx); 
                                            // if(database_cw.dIndx ==0)
                                            // {
                                            //     DB_PR("--gekou set fail2--.\r\n");   
                                            //     goto gekou_fail_z;
                                            // }
                                            
                                            if(database_cw.dIndx !=0)
                                            {
                                                DB_PR("--gekou set one ok--.\r\n");   
                                                switch(database_gz[database_cw.dIndx].dzx_mode_gz)
                                                {
                                                case 1:
                                                    // no gai
                                                    shengyu_da_max--;
                                                    shengyu_zhong_max++;
                                                    //shengyu_xiao_max--;


                                                    if((database_gz[database_cw.dIndx].state_gz ==0) //no use
                                                        &&(database_gz[database_cw.dIndx].lock == 0)
                                                        &&(database_gz[database_cw.dIndx].changqi == 0))
                                                    {
                                                        DB_PR("--no use--.\r\n");   
                                                        shengyu_da --;
                                                        shengyu_zhong++;
                                                        //shengyu_xiao --;
                                                    }
                                                    else
                                                    {
                                                        DB_PR("--on use--.\r\n");   
                                                    }


                                                    break;
                                                case 2:
                                                    // shengyu_da_max++;
                                                    // shengyu_zhong_max--;
                                                    // //shengyu_xiao_max--;
                                                    // shengyu_da ++;
                                                    // shengyu_zhong--;
                                                    // //shengyu_xiao --;
                                                    break;
                                                case 3:
                                                    shengyu_zhong_max++;
                                                    //shengyu_zhong_max--;
                                                    shengyu_xiao_max--;

                                                    if((database_gz[database_cw.dIndx].state_gz ==0) //no use
                                                        &&(database_gz[database_cw.dIndx].lock == 0)
                                                        &&(database_gz[database_cw.dIndx].changqi == 0))
                                                    {
                                                        shengyu_zhong ++;
                                                        //shengyu_zhong--;
                                                        shengyu_xiao --;
                                                    }
                                                    else
                                                    {
                                                        DB_PR("--on use--.\r\n");   
                                                    }

                                                    break;
        
                                                default:
                                                    DB_PR("--other--.\r\n");   
                                                }
                                                DB_PR("---shengyu_all=%d----\n",shengyu_all);

                                                DB_PR("---shengyu_da=%d----\n",shengyu_da);
                                                DB_PR("---shengyu_zhong=%d----\n",shengyu_zhong);
                                                DB_PR("---shengyu_xiao=%d----\n",shengyu_xiao);


                                                //shengyu_all_max = shengyu_da_max + shengyu_zhong_max + shengyu_xiao_max;
                                                DB_PR("-2-shengyu_all_max=%d----\n",shengyu_all_max);
                                                DB_PR("---shengyu_da_max=%d----\n",shengyu_da_max);
                                                DB_PR("---shengyu_zhong_max=%d----\n",shengyu_zhong_max);
                                                DB_PR("---shengyu_xiao_max=%d----\n",shengyu_xiao_max);





                                                database_gz[database_cw.dIndx].dzx_mode_gz = 2;
                                                nvs_wr_dzx_mode_gz(1);
                                                
                                                DB_PR("database_cw.dIndx = %03d\r\n\r\n",database_cw.dIndx);

                                            }
                                            else
                                            {
                                                DB_PR("--gekou set one fail2 z--.\r\n");   
                                                //goto gekou_fail_z;
                                            }
                                            
                                        }
                                        DB_PR("-------ts1-------\r\n");
                                        DB_PR("\r\n");
                                        DB_PR("shengyu_all_max=%03d\r\n",shengyu_all_max);

     
                                        nvs_wr_shengyu_da_max(1);
                                        nvs_wr_shengyu_zhong_max(1);
                                        nvs_wr_shengyu_xiao_max(1);
                                        //nvs_wr_shengyu_all_max(1);

                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);

                                        tongbu_gekou_shuliang_all(shengyu_all);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_d(shengyu_da);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_z(shengyu_zhong);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_x(shengyu_xiao);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);


                                        tongbu_da();
                                        tongbu_zh();
                                        
                                    }


                                    //save

                                    memset(tx_Buffer2,0,200);
                                    //send_cmd_to_lcd_bl_len(BL_GK_SZ_Z,tx_Buffer2,data_rx_t[2]-1);//clear


                                    send_cmd_to_lcd_pic(GEKOU_X_PIC);

                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_Z,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
                                }
                                else
                                {
//z
// gekou_fail_z:
                                    //send_cmd_to_lcd_pic(GEKOU_PIC);
                                    DB_PR("-2-gekou fail--.\r\n");   
                                }
                                
                                break;




                            //-------------格口设置 dazhongxiao
                            //xiao
                            case BL_GK_SZ_X://
                                DB_PR("--xiao gekou--.\r\n");   
                                
                                j=0;
                                for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                    DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                    tx_Buffer2[j]=data_rx_t[i];
                                    // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                    // if(tx_Buffer2[j] != 0x2D)
                                    j++;
                                }
                                DB_PR("\r\n");

                                if(data_rx_t[2] == 0xCE)//0x9c)
                                {
                                    
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;
                                    while('-' == *p)    
                                    {
                                        *p =0;
                                        p=p+1;
                                        q=p;
                                        //DB_PR("0-a *p=%02x\r\n",*p);//0x2d
                                    }
                                    DB_PR("0-b *p=%02x\r\n\r\n",*p);//0x2d

                                    while( NULL != (p = strchr(p,'-')))    
                                    {
                                        
                                        if(*(p-1) == '-')
                                        {
                                            DB_PR("0-c\r\n\r\n");//0x2d
                                            *(p-1)='\0';
                                            p=p+1;
                                            q=p;
                                            continue;
                                        }
                                        strncpy(show[i],q,p-q);
                                        show[i][p-q] = '\0';

                                        //DB_PR("%02x\r\n",*p);//0x2d
                                        p = p+1;
                                        q = p;
                                        i ++;
                                    }

                                    if (p == NULL)
                                    {
                                        strncpy(show[i],q,strlen(q));
                                        show[i][strlen(q)] = '\0';
                                    }

                                    hang_shu_max =i;//hang shu
                                    DB_PR("show1=");
                                    for(j = 0; j <= hang_shu_max; j++)
                                    {
                                        DB_PR("%s\r\n",show[j]);
                                    }

                                    for(j = 0; j <= strlen(show[hang_shu_max]); j++)//最后一行
                                    {
                                        if(0xff == show[hang_shu_max][j])
                                            show[hang_shu_max][j] = '\0';
                                        DB_PR("%02x-",show[hang_shu_max][j]);
                                    }
                                    DB_PR("\r\n");

                                    DB_PR("*show[0]=%d\r\n",* show[0]);
                                    if(*show[0] ==0)
                                    {
                                        DB_PR("da_set_over\r\n");
                                        send_cmd_to_lcd_pic(GEKOU_OK_PIC);
                                        break;
                                    }



                                    DB_PR("hang_shu_max=%03d\r\n",hang_shu_max);
                                    if((hang_shu_max<= 0x96))//
                                    {
                                        DB_PR("show2=");
                                        for(j = 0; j <= hang_shu_max; j++)//i 个柜子
                                        {

                                            guimen_gk_temp = atoi((const char*)show[j]);
                                            DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                            if((guimen_gk_temp>shengyu_all_max)
                                                ||(guimen_gk_temp<=0))
                                            {
                                                continue;
                                                DB_PR("--gekou overflow ,tiaoguo--.\r\n");   
                                                
                                            }
                                            DB_PR("---2----guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                            // if((guimen_gk_temp >=SHENYU_GEZI_MAX)||(0==guimen_gk_temp))
                                            // {
                                            //     DB_PR("--gekou set fail--.\r\n");   
                                            //     goto gekou_fail_x;
                                            // }
                                            database_cw.dIndx =0;
                                            database_cw.dIndx = find_lock_index(guimen_gk_temp);

                                            DB_PR("---2----lock--dIndx=%d--.\r\n",database_cw.dIndx); 
                                            // if(database_cw.dIndx ==0)
                                            // {
                                            //     DB_PR("--gekou set fail2--.\r\n");   
                                            //     goto gekou_fail_x;
                                            // }
                                            
                                            if(database_cw.dIndx !=0)
                                            {
                                                DB_PR("--gekou set one ok--.\r\n");   
                                                switch(database_gz[database_cw.dIndx].dzx_mode_gz)
                                                {
                                                case 1:
                                                    // no gai
                                                    shengyu_da_max--;
                                                    //shengyu_zhong_max++;
                                                    shengyu_xiao_max++;


                                                    if((database_gz[database_cw.dIndx].state_gz ==0) //no use
                                                        &&(database_gz[database_cw.dIndx].lock == 0)
                                                        &&(database_gz[database_cw.dIndx].changqi == 0))
                                                    {
                                                        shengyu_da --;
                                                        //shengyu_zhong++;
                                                        shengyu_xiao ++;
                                                    }
                                                    else
                                                    {
                                                        DB_PR("--on use--.\r\n");   
                                                    }


                                                    break;
                                                case 2:
                                                    //shengyu_da_max++;
                                                    shengyu_zhong_max--;
                                                    shengyu_xiao_max++;


                                                    if((database_gz[database_cw.dIndx].state_gz ==0) //no use
                                                        &&(database_gz[database_cw.dIndx].lock == 0)
                                                        &&(database_gz[database_cw.dIndx].changqi == 0))
                                                    {
                                                        //shengyu_da ++;
                                                        shengyu_zhong--;
                                                        shengyu_xiao ++;
                                                    }
                                                    else
                                                    {
                                                        DB_PR("--on use--.\r\n");   
                                                    }

                                                    break;
                                                case 3:
                                                    // shengyu_zhong_max++;
                                                    // //shengyu_zhong_max--;
                                                    // shengyu_xiao_max--;

                                                    // shengyu_zhong ++;
                                                    // //shengyu_zhong--;
                                                    // shengyu_xiao --;
                                                    break;
        
                                                default:
                                                    DB_PR("--other--.\r\n");   
                                                }
         

                                                database_gz[database_cw.dIndx].dzx_mode_gz = 3;
                                                nvs_wr_dzx_mode_gz(1);
                                                
                                                DB_PR("database_cw.dIndx = %03d\r\n\r\n",database_cw.dIndx);

                                            }
                                            else
                                            {
                                                DB_PR("--gekou set one fail2 x--.\r\n");   
                                                //goto gekou_fail_x;
                                            }
                                            
                                        }
                                        DB_PR("-------ts1-------\r\n");

                                        DB_PR("\r\n");
                                        DB_PR("shengyu_all_max=%03d\r\n",shengyu_all_max);

     
                                        nvs_wr_shengyu_da_max(1);
                                        nvs_wr_shengyu_zhong_max(1);
                                        nvs_wr_shengyu_xiao_max(1);
                                        //nvs_wr_shengyu_all_max(1);

                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);

                                        tongbu_gekou_shuliang_all(shengyu_all);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_d(shengyu_da);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_z(shengyu_zhong);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);
                                        tongbu_gekou_shuliang_x(shengyu_xiao);
                                        //vTaskDelay(10 / portTICK_PERIOD_MS);


                                        tongbu_da();
                                        tongbu_zh();//add
                                        
                                    }


                                    //save

                                    memset(tx_Buffer2,0,200);
                                    //send_cmd_to_lcd_bl_len(BL_GK_SZ_X,tx_Buffer2,data_rx_t[2]-1);//clear


                                    send_cmd_to_lcd_pic(GEKOU_OK_PIC);

                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_X,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
                                }
                                else
                                {
//x
// gekou_fail_x:
                                    //send_cmd_to_lcd_pic(GEKOU_PIC);
                                    DB_PR("-2-gekou fail--.\r\n");   
                                }
                                
                                break;









                           case 0x2040://
                                DB_PR("--quankai--.\r\n");   
                                // j=0;
                                // for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                //     tx_Buffer2[j]=data_rx_t[i];
                                //     // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                //     // if(tx_Buffer2[j] != 0x2D)
                                //     j++;
                                // }
                                // DB_PR("\r\n");

                                // DB_PR("shengyu_all_max= %3d\r\n", shengyu_all_max);
                                // guimen_gk_temp = shengyu_all_max ;
                                // j=0;
                                uint16_t k=0;


                                xTaskCreate(lock_all_open_task, "lk_all_open_task", 2* 1024, NULL, 2, NULL);//1024 10

                                //send_cmd_to_lcd_pic(KAIJI_PIC);
                                break;






                           case 0x2060://
                                DB_PR("--quan qing--.\r\n");   
                                // j=0;
                                // for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                //     tx_Buffer2[j]=data_rx_t[i];
                                //     // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                //     // if(tx_Buffer2[j] != 0x2D)
                                //     j++;
                                // }
                                // DB_PR("\r\n");

                                // DB_PR("shengyu_all_max= %3d\r\n", shengyu_all_max);
                                // guimen_gk_temp = shengyu_all_max ;
                                // j=0;
                                //uint16_t k=0;

                                xTaskCreate(lock_all_clear_task, "lk_all_clear_open_task", 4* 1024, NULL, 2, NULL);//1024 10

                                //send_cmd_to_lcd_pic(KAIJI_PIC);
                                break;

                           case 0x1270://
                                DB_PR("--xm fail return_cause =%d--.\r\n",return_cause);   

                                switch (return_cause)
                                {
                                    case 1:
                                        send_cmd_to_lcd_pic(0x0013);
                                        break;
                                    case 2:
                                        send_cmd_to_lcd_pic(0x0023);
                                        break;

                                    case 3:
                                        send_cmd_to_lcd_pic(0x0028);
                                        break;
                                    case 4:
                                        send_cmd_to_lcd_pic(0x002b);
                                        break;

                                    case 5:
                                        send_cmd_to_lcd_pic(0x0018);
                                        break;
                                    case 6:
                                        send_cmd_to_lcd_pic(0x0021);
                                        break;

                                    default:

                                        break;
                                }
                                    
                                return_cause =0;
                                break;



                            case 0x10c0://xiangmenhao   kaixiang
                                DB_PR("----admin --xmh open-----.\r\n");
                                // TaskHandle_t TaskHandle;	
                                // // TaskStatus_t TaskStatus;
                                // eTaskState TaskState;

                                // TaskHandle=xTaskGetHandle("gpio_task_example1");		//获取任务句柄（返回值任务句柄）（参数为任务名：query_task）
                                // TaskState=eTaskGetState(TaskHandle);			//获取任务状态函数（返回值任务状态）（参数为任务句柄：TaskHandle）
                                // DB_PR("-------TaskState=%d--.\r\n",TaskState); 

                                // vTaskDelete(TaskHandle);
                                // TaskState=eTaskGetState(TaskHandle);			//获取任务状态函数（返回值任务状态）（参数为任务句柄：TaskHandle）
                                // DB_PR("-------TaskState=%d--.\r\n",TaskState); 


                                
                                uint8_t temp_xiangmen[4]={0}; 
                                u8 buff_temp1_c[32]={0};
                                memset(temp_xiangmen,0,4);
                                j=0;
                                k=0;//k:board  j:lock
                                //uint8_t temp_xiangmen_uint=0; //16
                                memcpy(temp_xiangmen,data_rx_t+7,3);//len todo 
                                if(02== data_rx_t[6])//-------------------------
                                {
                                    guimen_gk_temp = atoi((const char*)temp_xiangmen);
                                    DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                    database_cw.dIndx =0;
                                    // for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
                                    // {
                                    //     if(database_gz[i].dIndx_gz == guimen_gk_temp)
                                    //     {
                                    //         database_cw.dIndx = i;
                                    //         DB_PR("-------lock--dIndx=%d--.\r\n",database_cw.dIndx); 
                                    //     }
                                       
                                    // }


                                    database_cw.dIndx = find_lock_index(guimen_gk_temp);
  
 
                                    // if(database_gz[database_cw.dIndx].state_fenpei_gz == 0)
                                    if(database_cw.dIndx == 0)
                                    {
                                        goto wuci_xmh;
                                    }

                                    DB_PR("------lock open--dIndx=%d--.\r\n",database_cw.dIndx); 
                                    guimen_gk_temp = database_cw.dIndx ;


      
                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);


                                    uint16_t j=0,k=0;

                                    k = guimen_gk_temp/24;
                                    j = guimen_gk_temp%24;

                                    if(guimen_gk_temp%24 ==0)
                                    {
                                        k = guimen_gk_temp/24 -1;
                                        j = 24;
                                    }

                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);


                                    // send_cmd_to_lcd_bl_len(0x10c0,(uint8_t*)buff_t,2*2+5);//key

                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    send_cmd_to_lock(k+1,j);

                                    memset(buff_temp1_c,0,32);
                                    buff_temp1_c[0] = database_gz[database_cw.dIndx].dIndx_gz/256;
                                    buff_temp1_c[1] = database_gz[database_cw.dIndx].dIndx_gz%256;
                                    send_cmd_to_lcd_bl_len(0x10c0,buff_temp1_c,32+3);
                                    // send_cmd_to_lcd_bl(0x10c0,database_gz[database_cw.dIndx].dIndx_gz);
            
                                    send_cmd_to_lcd_pic(0x0014);




                                    DB_PR("-----2 1-----[ * ] Starting audio pipeline\r\n");


                                    // if(taskhandle_mp3!=NULL)

                                    if(audio_play_mp3_task!=0)
                                    {
                                        audio_play_mp3_task =0;
                                        vTaskDelay(20 / portTICK_PERIOD_MS);
                                        DB_PR("----111111 -a-----.\r\n");
                                        vTaskDelete(taskhandle_mp3);
                                        // taskhandle_mp3 =NULL;
                                        DB_PR("----111111 -b-----.\r\n");
                                        // vTaskDelay(500 / portTICK_PERIOD_MS);
                                    }
                                    else
                                    {
                                        DB_PR("----222222 =NULL-----.\r\n");
                                    }
                                    
                                    xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, (TaskHandle_t* )&taskhandle_mp3);


                                }
                                else
                                {
wuci_xmh:
                                    send_cmd_to_lcd_bl_len(0x10c0,(uint8_t*)buff_t,2*2+5);//key
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
                                    return_cause = 1;
                                    DB_PR("----admin --wu ci xiangmenhao-----.\r\n");
                                }
                                //send_cmd_to_lcd_bl(0x10C0,database_gz[database_cw.dIndx].dIndx_gz);//xiangmen------------
                                
                                break;


                           case 0x1130://
                                DB_PR("--qingxiang--.\r\n");   
                                // send_cmd_to_lcd_bl_len(0x1130,(uint8_t*)buff_t,2*2+5);//
                                // j=0;
                                // for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                //     tx_Buffer2[j]=data_rx_t[i];
                                //     // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                //     // if(tx_Buffer2[j] != 0x2D)
                                //     j++;
                                // }
                                // DB_PR("\r\n");


                                //xTaskCreate(lock_all_open_task, "lk_all_open_task", 2* 1024, NULL, 2, NULL);//1024 10
                                
                                

                                memset(temp_xiangmen,0,4);
                                j=0;
                                k=0;//k:board  j:lock
                                bool lock_flag =0;
                                //uint8_t temp_xiangmen_uint=0; //16
                                memcpy(temp_xiangmen,data_rx_t+7,3);//len todo 
                                if((02== data_rx_t[6])||(03== data_rx_t[6]))//-------------------------
                                {
                                    guimen_gk_temp = atoi((const char*)temp_xiangmen);
                                    DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                    database_cw.dIndx =0;
                                    database_cw.dIndx = find_lock_index(guimen_gk_temp);

                                    
 
                                    if((database_gz[database_cw.dIndx].state_fenpei_gz == 0)
                                        ||(database_cw.dIndx ==0))
                                    {
                                        goto wuci_xmh_q;
                                    }

                                    DB_PR("------lock open--dIndx=%d--.\r\n",database_cw.dIndx); 
                                    guimen_gk_temp = database_cw.dIndx ;

                                    uint16_t j=0,k=0;

                                    k = guimen_gk_temp/24;
                                    j = guimen_gk_temp%24;

                                    if(guimen_gk_temp%24 ==0)
                                    {
                                        k = guimen_gk_temp/24 -1;
                                        j = 24;
                                    }
                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    send_cmd_to_lock(k+1,j);

                                    memset(buff_temp1_c,0,32);
                                    buff_temp1_c[0] = database_gz[database_cw.dIndx].dIndx_gz/256;
                                    buff_temp1_c[1] = database_gz[database_cw.dIndx].dIndx_gz%256;
                                    send_cmd_to_lcd_bl_len(0x1130,buff_temp1_c,32+3);
                                    // send_cmd_to_lcd_bl(0x1130,database_gz[database_cw.dIndx].dIndx_gz);
            
                                    //send_cmd_to_lcd_pic(0x0024);
                                    send_cmd_to_lcd_pic(CLEAR_ONE_OK_PIC);



                                    
                         



                                    if((0 != database_gz[database_cw.dIndx].state_gz)
                                        ||(0 != database_gz[database_cw.dIndx].changqi))
                                    {
                                        switch (database_gz[database_cw.dIndx].dzx_mode_gz)
                                        {
                                        case 1:
                                            //d
                                            shengyu_da ++;
                                            tongbu_gekou_shuliang_d(shengyu_da); 
                                            break;
                                        case 2:
                                            //z
                                            shengyu_zhong ++;
                                            tongbu_gekou_shuliang_z(shengyu_zhong); 
                                            break;
                                        case 3:
                                            //x
                                            shengyu_xiao ++;
                                            tongbu_gekou_shuliang_x(shengyu_xiao); 
                                            break;

                                        default:
                                            break;
                                        }

                                        DB_PR("----test--.\r\n");  
                                        

                                        shengyu_all ++ ;
                                        tongbu_gekou_shuliang_all(shengyu_all);


                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);
                                    }

                                    
                                    DB_PR( "database_gz[database_cw.dIndx].state_gz = %d\r\n", database_gz[database_cw.dIndx].state_gz);
                                    
                                    



                                    //if(0 != database_gz[database_cw.dIndx].state_gz)
                                    {

                                        DB_PR( "---database_gz[database_cw.dIndx].cunwu_mode_gz = %d\r\n", database_gz[database_cw.dIndx].cunwu_mode_gz);
                                        //if(database_gz[database_cw.dIndx].cunwu_mode_gz ==1)
                                        {
                                            Del_FR(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                                            //del_zw_database(database_gz[database_cw.dIndx].zhiwen_page_id_gz);

                                            database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id] =0;

                                            database_cw.zhiwen_page_id =database_gz[database_cw.dIndx].zhiwen_page_id_gz;
                                            nvs_wr_adm_zwpageid_flag(1,database_cw.zhiwen_page_id);
                                            database_gz[database_cw.dIndx].zhiwen_page_id_gz = 0;
                                            nvs_wr_zw_pageid_gz(1);
                                            
                                        }
                                        //else  if(database_gz[database_cw.dIndx].cunwu_mode_gz ==2)
                                        {
                                            database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                            database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                            nvs_wr_phone_number_nvs_gz(1);
                                            nvs_wr_mima_number_nvs_gz(1);
                                        }

                                        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                        //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
                                        database_gz[database_cw.dIndx].state_gz =0;
                                        database_gz[database_cw.dIndx].changqi =0;
                                       
                                        //nvs_wr_dzx_mode_gz(1);


                                        nvs_wr_cunwu_mode_gz(1);
                                        nvs_wr_state_gz(1);
                                        nvs_wr_glongtime_gz(1);



                                        DB_PR("-----2-----[ * ] Starting audio pipeline");


                                        if(audio_play_mp3_task!=0)
                                        {
                                            audio_play_mp3_task =0;
                                            vTaskDelay(20 / portTICK_PERIOD_MS);
                                            DB_PR("----111111 -a-----.\r\n");
                                            vTaskDelete(taskhandle_mp3);
                                            // taskhandle_mp3 =NULL;
                                            DB_PR("----111111 -b-----.\r\n");
                                            // vTaskDelay(500 / portTICK_PERIOD_MS);
                                        }
                                        else
                                        {
                                            DB_PR("----222222 =NULL-----.\r\n");
                                        }
                                        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);
                                        //update xianshi todo
                                        tongbu_changqi();
                                        //tongbu_locked();    
                                    }

                                }
                                else
                                {
wuci_xmh_q:
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
                                    send_cmd_to_lcd_bl_len(0x1130,(uint8_t*)buff_t,2*2+5);//
                                    return_cause = 2;
                                    DB_PR("----admin --wu ci xiangmenhao-----.\r\n");
                                }
                                //send_cmd_to_lcd_bl(0x1130,database_gz[database_cw.dIndx].dIndx_gz);//xiangmen------------


                                
                                break;









                            case 0x1280://mima? zanwukongxiang close
                                DB_PR("---return_cause_has_be_lock=%d---.\r\n",return_cause_has_be_lock);   
                                //if -> huise tupian?
                                if(return_cause_has_be_lock == 1)
                                {
                                    DB_PR("--lock lock--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0028);
                                }
                                else if(return_cause_has_be_lock == 2)
                                {
                                    DB_PR("--changqi lock 111 --.\r\n");  
                                    //baocun 1
                                    send_cmd_to_lcd_pic(0x0017);
                                }
                                else
                                {
                                    DB_PR("--other 111 --.\r\n");  
                                    //baocun 1
                                    send_cmd_to_lcd_pic(0x0011);
                                }
                                return_cause_has_be_lock =0;

                                break;






                           case 0x1150://
                                DB_PR("--lock ding--.\r\n");   
                                // j=0;
                                // for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                //     tx_Buffer2[j]=data_rx_t[i];
                                //     // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                //     // if(tx_Buffer2[j] != 0x2D)
                                //     j++;
                                // }
                                // DB_PR("\r\n");

                                send_cmd_to_lcd_bl_len(0x1150,(uint8_t*)buff_t,2*2+5);//
                                //uint8_t temp_xiangmen[4]={0}; 
                                memset(temp_xiangmen,0,4);
                                j=0;
                                k=0;//k:board  j:lock
                                //uint8_t temp_xiangmen_uint=0; //16
                                memcpy(temp_xiangmen,data_rx_t+7,3);//len todo 
                                if((02== data_rx_t[6])||(03== data_rx_t[6]))//-------------------------
                                {
                                    guimen_gk_temp = atoi((const char*)temp_xiangmen);
                                    DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                    database_cw.dIndx =0;
                                    database_cw.dIndx = find_lock_index(guimen_gk_temp);


 
                                    if(database_gz[database_cw.dIndx].lock == 1)
                                    {
                                        DB_PR("--break, have been be lock--.\r\n");   
                                        send_cmd_to_lcd_pic(0x004a);
                                        return_cause_has_be_lock = 1;
                                        break;
                                        //goto wuci_xmh_lk;
                                    }
                                    
 
                                    if((database_gz[database_cw.dIndx].state_fenpei_gz == 0)
                                       ||(database_cw.dIndx ==0))
                                    {
                                        goto wuci_xmh_lk;
                                    }

                                    DB_PR("------lock open--dIndx=%d--.\r\n",database_cw.dIndx); 
                                    guimen_gk_temp = database_cw.dIndx ;

                                    uint16_t j=0,k=0;

                                    k = guimen_gk_temp/24;
                                    j = guimen_gk_temp%24;

                                    if(guimen_gk_temp%24 ==0)
                                    {
                                        k = guimen_gk_temp/24 -1;
                                        j = 24;
                                    }
                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    // send_cmd_to_lock(k+1,j);
                                    send_cmd_to_lcd_bl(0x1150,database_gz[database_cw.dIndx].dIndx_gz);

                                    // uint8_t buff_temp1[6]={0x01,0x64,0x31};
                                    // send_cmd_to_lcd_bl_len(0x1150,buff_temp1,6+5);
            
                                    //send_cmd_to_lcd_pic(0x0024);
                                    send_cmd_to_lcd_pic(LK_OK_PIC);



                                    DB_PR("-----2-----[ * ] Starting audio pipeline");


                                    DB_PR( "database_gz[database_cw.dIndx].state_gz = %d", database_gz[database_cw.dIndx].state_gz);
                                    if((0 == database_gz[database_cw.dIndx].state_gz)
                                        &&(0 == database_gz[database_cw.dIndx].changqi))
                                    {
                                        switch (database_gz[database_cw.dIndx].dzx_mode_gz)
                                        {
                                        case 1:
                                            //d
                                            shengyu_da --;
                                            tongbu_gekou_shuliang_d(shengyu_da); 
                                            break;
                                        case 2:
                                            //z
                                            shengyu_zhong --;
                                            tongbu_gekou_shuliang_z(shengyu_zhong); 
                                            break;
                                        case 3:
                                            //x
                                            shengyu_xiao --;
                                            tongbu_gekou_shuliang_x(shengyu_xiao); 
                                            break;

                                        default:
                                            break;
                                        }

                                        DB_PR("----test--.\r\n");  
                                        

                                        shengyu_all -- ;
                                        tongbu_gekou_shuliang_all(shengyu_all);


                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);

                                    }
 

                                    //if(0 != database_gz[database_cw.dIndx].state_gz)
                                    {

                                        // char key_name[15];//15
                                        // esp_err_t err;

                                        //database_gz[database_cw.dIndx].cunwu_mode_gz = 0;//del
                                        //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
  
                                        //database_gz[database_cw.dIndx].state_gz =0;

                                        if(database_gz[database_cw.dIndx].changqi ==1)
                                            database_gz[database_cw.dIndx].changqi =2;
                                        DB_PR("--database_gz[database_cw.dIndx].changqi=%d\r\n",database_gz[database_cw.dIndx].changqi);
                                        // else if(database_gz[database_cw.dIndx].changqi ==0)
                                        //     database_gz[database_cw.dIndx].changqi =0;

                                        database_gz[database_cw.dIndx].lock =1;
                                        //nvs_wr_cunwu_mode_gz(1);
                                        //nvs_wr_dzx_mode_gz(1);
         
                                        //nvs_wr_state_gz(1);
                                        nvs_wr_glongtime_gz(1);
                                        nvs_wr_glock_gz(1);

                                        // if(0 == database_gz[database_cw.dIndx].state_gz)
                                        // {
                                        //     database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                        //     database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                        //     nvs_wr_phone_number_nvs_gz(1);
                                        //     nvs_wr_mima_number_nvs_gz(1);
                                        // }

                                    }
                                    xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8192, (void*)TONE_TYPE_SUODING, 10, NULL);

                                    //if(database_gz[database_cw.dIndx].changqi == 1)
                                    {
                                        //update xianshi todo
                                        tongbu_changqi();
                                        tongbu_locked();   
                                    } 


                                }
                                else
                                {
wuci_xmh_lk:
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
                                    return_cause = 3;
                                    DB_PR("----admin --wu ci xiangmenhao-----.\r\n");
                                }
                                //send_cmd_to_lcd_bl(0x1130,database_gz[database_cw.dIndx].dIndx_gz);//xiangmen------------


                                
                                break;













                           case 0x1180://
                                DB_PR("--lock jie suo--.\r\n");   
                                // j=0;
                                // for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                //     tx_Buffer2[j]=data_rx_t[i];
                                //     // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                //     // if(tx_Buffer2[j] != 0x2D)
                                //     j++;
                                // }
                                // DB_PR("\r\n");

                                send_cmd_to_lcd_bl_len(0x1180,(uint8_t*)buff_t,2*2+5);//
                                //uint8_t temp_xiangmen[4]={0}; 
                                memset(temp_xiangmen,0,4);
                                j=0;
                                k=0;//k:board  j:lock
                                //uint8_t temp_xiangmen_uint=0; //16
                                memcpy(temp_xiangmen,data_rx_t+7,3);//len todo 
                                if((02== data_rx_t[6])||(03== data_rx_t[6]))//-------------------------
                                {
                                    guimen_gk_temp = atoi((const char*)temp_xiangmen);
                                    DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                    database_cw.dIndx =0;
                                    database_cw.dIndx = find_lock_index(guimen_gk_temp);



 
                                    if((database_gz[database_cw.dIndx].state_fenpei_gz == 0)
                                       ||(database_cw.dIndx ==0))
                                    {
                                        DB_PR("------no fenpei---------.\r\n");
                                        goto wuci_xmh_unlk;
                                    }

                                    if(database_gz[database_cw.dIndx].lock == 0)
                                    {
                                        DB_PR("------no lock------.\r\n");
                                        send_cmd_to_lcd_pic(0x0050);
                                        break;
                                        //goto wuci_xmh_unlk;//todo--------------
                                    }
                                    
                                    DB_PR("------lock open--dIndx=%d--.\r\n",database_cw.dIndx); 
                                    guimen_gk_temp = database_cw.dIndx ;

                                    uint16_t j=0,k=0;

                                    k = guimen_gk_temp/24;
                                    j = guimen_gk_temp%24;

                                    if(guimen_gk_temp%24 ==0)
                                    {
                                        k = guimen_gk_temp/24 -1;
                                        j = 24;
                                    }
                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    // send_cmd_to_lock(k+1,j);
                                    send_cmd_to_lcd_bl(0x1180,database_gz[database_cw.dIndx].dIndx_gz);
            
                                    //send_cmd_to_lcd_pic(0x0024);
                                    send_cmd_to_lcd_pic(UNLK_OK_PIC);



                                    DB_PR("-----2-----[ * ] Starting audio pipeline\r\n");


                                    DB_PR( "database_gz[database_cw.dIndx].state_gz = %d\r\n", database_gz[database_cw.dIndx].state_gz);
                                    DB_PR( "database_gz[database_cw.dIndx].dIndx_gz = %d\r\n", database_gz[database_cw.dIndx].dIndx_gz);
                                    if((0 == database_gz[database_cw.dIndx].state_gz)
                                        &&(0 == database_gz[database_cw.dIndx].changqi))
                                    {
                                        switch (database_gz[database_cw.dIndx].dzx_mode_gz)
                                        {
                                        case 1:
                                            //d
                                            shengyu_da ++;
                                            tongbu_gekou_shuliang_d(shengyu_da); 
                                            break;
                                        case 2:
                                            //z
                                            shengyu_zhong ++;
                                            tongbu_gekou_shuliang_z(shengyu_zhong); 
                                            break;
                                        case 3:
                                            //x
                                            shengyu_xiao ++;
                                            tongbu_gekou_shuliang_x(shengyu_xiao); 
                                            break;

                                        default:
                                            break;
                                        }

                                        DB_PR("----test--.\r\n");  
                                        

                                        shengyu_all ++ ;
                                        tongbu_gekou_shuliang_all(shengyu_all);


                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);

                                    }
                                    // if(database_gz[database_cw.dIndx].changqi == 1)
                                    // {
                                    //     //update xianshi todo
                                    //     ;
                                    // }  

                                    //if(0 != database_gz[database_cw.dIndx].state_gz)
                                    {

                                        // char key_name[15];//15
                                        // esp_err_t err;

                                        // database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                        //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
  
                                        //database_gz[database_cw.dIndx].state_gz =0;

                                       
                                        database_gz[database_cw.dIndx].lock =0;
                                        nvs_wr_glock_gz(1);
                                        //nvs_wr_cunwu_mode_gz(1);
                                        //nvs_wr_dzx_mode_gz(1);
         
                                        //nvs_wr_state_gz(1);



                                        DB_PR("-1-database_gz[database_cw.dIndx].changqi=%d\r\n",database_gz[database_cw.dIndx].changqi);
                                        if(2==database_gz[database_cw.dIndx].changqi)
                                        {
                                            database_gz[database_cw.dIndx].changqi =1;
                                            nvs_wr_glongtime_gz(1);
                                        }
                                        DB_PR("-2-database_gz[database_cw.dIndx].changqi=%d\r\n",database_gz[database_cw.dIndx].changqi);
                                        // else
                                        // {
                                        //     database_gz[database_cw.dIndx].changqi =0;
                                        // }

                                        // database_gz[database_cw.dIndx].changqi =0;
                                        
                                    



                                        

                                        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8192, (void*)TONE_TYPE_JIESUO, 10, NULL);
                                        
                                        tongbu_changqi();
                                        tongbu_locked();   
                                        
                                        // //if(0 == database_gz[database_cw.dIndx].state_gz)
                                        // {
                                        //     database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                        //     database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                        //     nvs_wr_phone_number_nvs_gz(1);
                                        //     nvs_wr_mima_number_nvs_gz(1);
                                        // }

                                    }

                                }
                                else
                                {
wuci_xmh_unlk:
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
                                    return_cause = 4;
                                    DB_PR("----admin --wu ci xiangmenhao-----.\r\n");
                                }
                                //send_cmd_to_lcd_bl(0x1130,database_gz[database_cw.dIndx].dIndx_gz);//xiangmen------------


                                
                                break;














                            case 0x10e0://
                                DB_PR("--xinzeng changqi--.\r\n");   
                                // j=0;
                                // for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                //     tx_Buffer2[j]=data_rx_t[i];
                                //     // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                //     // if(tx_Buffer2[j] != 0x2D)
                                //     j++;
                                // }
                                // DB_PR("\r\n");

                                database_cw_adm.changqi_tmp =1;


                                
                                //uint8_t temp_xiangmen[4]={0}; 
                                memset(temp_xiangmen,0,4);
                                j=0;
                                k=0;//k:board  j:lock
                                //bool lock_flag =0;
                                //uint8_t temp_xiangmen_uint=0; //16
                                memcpy(temp_xiangmen,data_rx_t+7,3);//len todo 
                                if(02== data_rx_t[6])//-------------------------
                                {
                                    send_cmd_to_lcd_bl_len(0x10e0,(uint8_t*)buff_t,2*2+5);//
                                    guimen_gk_temp = atoi((const char*)temp_xiangmen);
                                    DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                    database_cw_adm.dIndx =0;
                                    database_cw_adm.dIndx = find_lock_index(guimen_gk_temp);

                                    DB_PR("1------lock open--dIndx=%d--.\r\n",database_cw_adm.dIndx); 
                                    DB_PR("database_gz[database_cw.dIndx].dIndx_gz=%d--.\r\n",database_gz[database_cw_adm.dIndx].dIndx_gz); 

                                    if((database_gz[database_cw_adm.dIndx].changqi == 1)
                                        ||(database_gz[database_cw_adm.dIndx].state_gz == 1))
                                    {
                                        send_cmd_to_lcd_bl(0x10e0,database_gz[database_cw_adm.dIndx].dIndx_gz);
                                        send_cmd_to_lcd_pic(0x0029);
                                        break;
                                    }
                                    
                                    if(database_gz[database_cw_adm.dIndx].lock == 1)
                                    {
                                        DB_PR("--break, have been be lock--.\r\n");   
                                        send_cmd_to_lcd_pic(0x004a);
                                        return_cause_has_be_lock = 2;
                                        break;
                                        //goto wuci_xmh_lk;
                                    }

                                    if((database_gz[database_cw_adm.dIndx].state_fenpei_gz == 0)
                                        ||(database_cw_adm.dIndx ==0))
                                    {
                                        goto wuci_xmh_xinz;
                                    }

                                    

                                    DB_PR("------lock open--dIndx=%d--.\r\n",database_cw_adm.dIndx); 
                                    guimen_gk_temp = database_cw_adm.dIndx ;

                                    // database_cw_adm.tmp_dIndx_gz = find_lock_index(database_cw_adm.dIndx);



                                    uint16_t j=0,k=0;

                                    k = guimen_gk_temp/24;
                                    j = guimen_gk_temp%24;

                                    if(guimen_gk_temp%24 ==0)
                                    {
                                        k = guimen_gk_temp/24 -1;
                                        j = 24;
                                    }
                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    //send_cmd_to_lock(k+1,j);
            
                                    //send_cmd_to_lcd_pic(0x001c);


                                    // if((0== shengyu_da)
                                    //     &&(0== shengyu_zhong)
                                    //     &&(0== shengyu_xiao))
                                    // {
                                    //     DB_PR("--zanwu kongxiang--.\r\n"); // houbian sheng  
                                    //     send_cmd_to_lcd_pic(0x0001);
                                    //     return_cause_zanwu_kx =1;
                                    //     //     /* Start the timers */
                                    //     // ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 3000000));//3s
                                    //     // DB_PR("Started timers, time since boot: %lld us", esp_timer_get_time());

                                    // }
                                    // else//
                                    // {
                                    //     send_cmd_to_lcd_pic(CHANQI_CW_MODE_PIC);
                                    // }

                                    send_cmd_to_lcd_pic(CHANQI_CW_MODE_PIC);

                                    DB_PR("-----2-----[ * ] Starting audio pipeline");
                                    DB_PR( "database_gz[database_cw_adm.dIndx].state_gz = %d", database_gz[database_cw_adm.dIndx].state_gz);


                                }
                                else
                                {
wuci_xmh_xinz:
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
                                    return_cause = 5;
                                    //send_cmd_to_lcd_pic(FAIL_CHANGQI_PIC);
                                    DB_PR("----admin --wu ci xiangmenhao-----.\r\n");
                                }
                                //send_cmd_to_lcd_bl(0x1130,database_gz[database_cw_adm.dIndx].dIndx_gz);//xiangmen------------


                                
                                break;





                            case 0x2050://zhiwen  or   mima changqi
                                DB_PR("--a-zhiwen or mima---.\r\n");   
                                //if -> huise tupian?
                                if(01== data_rx_t[8])//zhiwen cun
                                {
                                    DB_PR("--zhiwen--.\r\n");  
                                    //baocun 1
                                    database_cw_adm.cunwu_mode = 1;
									send_cmd_to_lcd_pic(0x0004);//0x001b
                                    xTaskCreate(Add_FR_First, "add_zhiwen_task", 8* 1024, NULL, 2, NULL);//1024 10
                                }
                                else if(02== data_rx_t[8])//mi ma
                                {
                                    DB_PR("--mima--.\r\n");  
                                    //baocun 2
									send_cmd_to_lcd_pic(0x001d);
                                    database_cw_adm.cunwu_mode = 2;

                                    send_cmd_to_lcd_bl_len(0x1100,(uint8_t*)buff_t,30+5);//phone
                                    send_cmd_to_lcd_bl_len(0x1110,(uint8_t*)buff_t,30+5);//key
                                }


                                break;



                            case 0x1100:
                                //5A A5 10 83   10 50   06    31 32 33 34 35 36 37 38 39 30  31 00
                                DB_PR("-a-phone number--.\r\n");
                                //panduan   -  zan cun quanju
                                //if has, todo  -----------------------------------

                                //DB_PR("data_rx_t[6]=%d---.\r\n",data_rx_t[6]);
                                if((06== data_rx_t[6])
                                    &&(0xFF== data_rx_t[len_rx_t -3]))//12
                                {
                                    //zancun
                                    phone_weishu_ok_a =1;
                                    memcpy( phone_number_a,data_rx_t+7 ,11);
                                    //phone_number[11] = '\0';//no use
                                    

                                    for (int i = 7; i < 7+ data_rx_t[6] *2 -1; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            phone_weishu_ok_a =0;
                                            DB_PR("--no--phone_weishu_ok_a=%d---.\r\n",phone_weishu_ok_a);
                                        }
                                    }
                                    DB_PR("\r\n");

                                    if(phone_weishu_ok_a == 1)
                                        DB_PR("-yes-phone_weishu_ok_a=%d---.\r\n",phone_weishu_ok_a);

                                }
                                else
                                {
                                    DB_PR("-------1 - shoujihao weishu err--------.\r\n");
                                }
                                
                                break;

                            case 0x1110:
                                
                                //5A A5 0A 83   10 60   03   31 32 33 34 35 36 
                                DB_PR("-a-CQ-password--.\r\n");
                                //DB_PR("---phone_weishu_ok=%d---.\r\n",phone_weishu_ok);
                                // send_cmd_to_lcd_bl_len(0x1100,(uint8_t*)buff_t,12+5);//phone
                                // send_cmd_to_lcd_bl_len(0x1110,(uint8_t*)buff_t,6+5);//key


                                //存物的格口编号（123）、格口类型（1：小，2：中，3：大）
                                //存物手机号（11位）密码（6位）            或者指纹(----)   


                                if((1 == phone_weishu_ok_a)&&(03== data_rx_t[6]))//6   ok todo shoujihao yiyou
                                {
                                    
                                    phone_weishu_ok_a =0;
                                    memcpy( mima_number_a,data_rx_t+7 ,6);

                                    DB_PR("phone_number=");
                                    uart0_debug_str(phone_number_a,11);

                                    DB_PR("mima_number=");
                                    uart0_debug_str(mima_number_a,6);


                                    if(0==memcmp(mima_number_a,buff_t,6))
                                    {
                                        DB_PR("------mima_number_a all=0-------.\r\n");
                                        goto done_longtime;
                                    }

                                    for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            DB_PR("--no--mima_weishu_ok---.\r\n");
                                            goto done_longtime;
                                        }
                                    }
                                    DB_PR("\r\n");


                                    uint16_t j=0,k=0;
   

                                    database_cw_adm.phone_number_nvs = atoll((const char*)phone_number_a);
                                    database_cw_adm.mima_number_nvs = atoi((const char*)mima_number_a);
                                    DB_PR("phone?=%11llu,mima?=%6u,", database_cw_adm.phone_number_nvs, database_cw_adm.mima_number_nvs);
                                    

                                    
                                    for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                                    {

                                        if(database_gz[i].phone_number_nvs_gz == database_cw_adm.phone_number_nvs)
                                        {
                                            if(database_gz[i].mima_number_nvs_gz == database_cw_adm.mima_number_nvs) 
                                            {
                                                //chongfu_flag =1;
                                                DB_PR("---phone_number and key_number has in database\r\n");
                                                send_cmd_to_lcd_pic(0x001F);
                                                goto done_longtime_2;
                                            }
                                            else
                                            {
                                            
                                                DB_PR("---only mima_number will xin, phone_number has in database\r\n");
                                            }


                                        }
                                        else//ok
                                        {
                                            //DB_PR("---phone_number and mima will xin\r\n");
                                        }
                                        
                                    }
                                    DB_PR("---phone_number and mima will xin\r\n");



                                    

                                    database_cw.dIndx = database_cw_adm.dIndx;//随机获取哪个门没用

                                    // database_cw.state=1;
                                    // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                    DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                                    DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);

                                    //database_cw.dIndx = 43;//-------14--------


                                    guimen_gk_temp = database_cw.dIndx ;

                                    j=0,k=0;

                                    k = guimen_gk_temp/24;
                                    j = guimen_gk_temp%24;

                                    if(guimen_gk_temp%24 ==0)
                                    {
                                        k = guimen_gk_temp/24 -1;
                                        j = 24;
                                    }
                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    send_cmd_to_lock(k+1,j);
                                    

                                    send_cmd_to_lcd_bl(0x10e0,database_gz[database_cw.dIndx].dIndx_gz);
                                    send_cmd_to_lcd_pic(XIN_CHANGQI_OK_PIC);



                                    



                                    DB_PR("-----2-----[ * ] Starting audio pipeline");


                                    DB_PR( "database_gz[database_cw_adm.dIndx].state_gz = %d", database_gz[database_cw_adm.dIndx].state_gz);
                                    if(0 == database_gz[database_cw_adm.dIndx].state_gz)
                                    {
                                        switch (database_gz[database_cw_adm.dIndx].dzx_mode_gz)
                                        {
                                        case 1:
                                            //d
                                            shengyu_da --;
                                            tongbu_gekou_shuliang_d(shengyu_da); 
                                            break;
                                        case 2:
                                            //z
                                            shengyu_zhong --;
                                            tongbu_gekou_shuliang_z(shengyu_zhong); 
                                            break;
                                        case 3:
                                            //x
                                            shengyu_xiao --;
                                            tongbu_gekou_shuliang_x(shengyu_xiao); 
                                            break;

                                        default:
                                            break;
                                        }

                                        DB_PR("----test--.\r\n");  
                                        

                                        shengyu_all -- ;
                                        tongbu_gekou_shuliang_all(shengyu_all);


                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);

                                    }


                                    //if(0 != database_gz[database_cw_adm.dIndx].state_gz)
                                    {

                                        // char key_name[15];//15
                                        // esp_err_t err;

                                        database_gz[database_cw_adm.dIndx].cunwu_mode_gz = database_cw_adm.cunwu_mode;
                                        //database_gz[database_cw_adm.dIndx].dzx_mode_gz = 0;
  
                                        //database_gz[database_cw_adm.dIndx].state_gz =0;

                                        database_gz[database_cw_adm.dIndx].changqi =1;
                                        //database_gz[database_cw_adm.dIndx].lock =0;
                                        nvs_wr_cunwu_mode_gz(1);
                                        //nvs_wr_dzx_mode_gz(1);
         
                                        //nvs_wr_state_gz(1);
                                        nvs_wr_glongtime_gz(1);
                                        //nvs_wr_glock_gz(1);

                                        //if(0 == database_gz[database_cw_adm.dIndx].state_gz)
                                        {
                                            database_gz[database_cw_adm.dIndx].phone_number_nvs_gz = database_cw_adm.phone_number_nvs;
                                            database_gz[database_cw_adm.dIndx].mima_number_nvs_gz = database_cw_adm.mima_number_nvs;
                                            nvs_wr_phone_number_nvs_gz(1);
                                            nvs_wr_mima_number_nvs_gz(1);
                                        }

                                    }

                                    DB_PR("-----2-----[ * ] Starting audio pipeline");
                                    xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);
                                    //if(database_gz[database_cw_adm.dIndx].changqi == 0)
                                    {
                                        //update xianshi todo
                                        tongbu_changqi();
                                    }  
                                        //custumer   mingming kongjian   todo
                                        // esp_err_t err = save_u16_value(STORAGE_NAMESPACE,"dw_dIndx",database_cw.dIndx);
                                        // if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
                                        // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                                        //unique number

                                    database_cw_adm.cunwu_mode =0;
                                    database_cw_adm.dzx_mode = 0 ;

                                    send_cmd_to_lcd_bl_len(0x1100,(uint8_t*)buff_t,30+5);//phone
                                    send_cmd_to_lcd_bl_len(0x1110,(uint8_t*)buff_t,30+5);//key
                                }
                                else
                                {
                                    if(03!= data_rx_t[6])
                                    {
                                      DB_PR("----------------2 - mima weishu err---------------.\r\n");  
                                    }
done_longtime:
                                    DB_PR("----test2-error--.\r\n");  
                                    send_cmd_to_lcd_pic(0x001e);//0x0007
                                }
done_longtime_2:
                                DB_PR("----test3-done--.\r\n");  


                                // send_cmd_to_lcd_bl(0x1100,0);//phone
                                // send_cmd_to_lcd_bl(0x1110,0);//key
                                


                                


                                // phone_weishu_ok =0;


                                break;








                           case 0x1120://
                                DB_PR("--changqi clear exe--.\r\n");   
                                // j=0;
                                // for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                //     tx_Buffer2[j]=data_rx_t[i];
                                //     // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                //     // if(tx_Buffer2[j] != 0x2D)
                                //     j++;
                                // }
                                // DB_PR("\r\n");

                                
                                //uint8_t temp_xiangmen[4]={0}; 
                                memset(temp_xiangmen,0,4);
                                j=0;
                                k=0;//k:board  j:lock
                                //uint8_t temp_xiangmen_uint=0; //16
                                memcpy(temp_xiangmen,data_rx_t+7,3);//len todo 
                                if(02== data_rx_t[6])//-----------  --------------
                                {
                                    send_cmd_to_lcd_bl_len(0x1120,(uint8_t*)buff_t,2*2+5);//
                                    guimen_gk_temp = atoi((const char*)temp_xiangmen);
                                    DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 

                                    database_cw.dIndx =0;
                                    database_cw.dIndx = find_lock_index(guimen_gk_temp);



                                    
 
                                    if((database_gz[database_cw.dIndx].state_fenpei_gz == 0)
                                        ||(database_gz[database_cw.dIndx].changqi == 0)
                                       ||(database_cw.dIndx ==0))
                                    {
                                        goto wuci_xmh_unchangqi;
                                    }

                                    DB_PR("------lock open--dIndx=%d--.\r\n",database_cw.dIndx); 
                                    guimen_gk_temp = database_cw.dIndx ;

                                    uint16_t j=0,k=0;

                                    k = guimen_gk_temp/24;
                                    j = guimen_gk_temp%24;

                                    if(guimen_gk_temp%24 ==0)
                                    {
                                        k = guimen_gk_temp/24 -1;
                                        j = 24;
                                    }
                                    DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    send_cmd_to_lock(k+1,j);
                                    send_cmd_to_lcd_bl(0x1120,database_gz[database_cw.dIndx].dIndx_gz);

                                    DB_PR("-----2-----[ * ] Starting audio pipeline");
                                    xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);
                                    //send_cmd_to_lcd_pic(0x0024);
                                    send_cmd_to_lcd_pic(UN_CHANGQI_OK_PIC);



                                    DB_PR("-----2-----[ * ] Starting audio pipeline");


                                    DB_PR( "database_gz[database_cw.dIndx].state_gz = %d", database_gz[database_cw.dIndx].state_gz);
                                    DB_PR( "database_gz[database_cw.dIndx].dIndx_gz = %d", database_gz[database_cw.dIndx].dIndx_gz);
                                    if(0 == database_gz[database_cw.dIndx].state_gz)
                                    {
                                        switch (database_gz[database_cw.dIndx].dzx_mode_gz)
                                        {
                                        case 1:
                                            //d
                                            shengyu_da ++;
                                            tongbu_gekou_shuliang_d(shengyu_da); 
                                            break;
                                        case 2:
                                            //z
                                            shengyu_zhong ++;
                                            tongbu_gekou_shuliang_z(shengyu_zhong); 
                                            break;
                                        case 3:
                                            //x
                                            shengyu_xiao ++;
                                            tongbu_gekou_shuliang_x(shengyu_xiao); 
                                            break;

                                        default:
                                            break;
                                        }

                                        DB_PR("----test--.\r\n");  
                                        

                                        shengyu_all ++ ;
                                        tongbu_gekou_shuliang_all(shengyu_all);


                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);

                                    }


                                    //if(0 != database_gz[database_cw.dIndx].state_gz)
                                    {

                                        // char key_name[15];//15
                                        // esp_err_t err;


                                        // //if(0 == database_gz[database_cw.dIndx].state_gz)

                                        //if(database_gz[database_cw.dIndx].cunwu_mode_gz ==1)
                                        {
                                            DB_PR("----test--.\r\n");  
                                            
                                            Del_FR(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                                            //del_zw_database(database_gz[database_cw.dIndx].zhiwen_page_id_gz);

                                            database_cw.zhiwen_page_id =database_gz[database_cw.dIndx].zhiwen_page_id_gz;
                                            database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id] =0;
                                            nvs_wr_adm_zwpageid_flag(1,database_cw.zhiwen_page_id);
                                            database_gz[database_cw.dIndx].zhiwen_page_id_gz = 0;
                                            nvs_wr_zw_pageid_gz(1);
                                            
                                        }
                                        //else  if(database_gz[database_cw.dIndx].cunwu_mode_gz ==2)
                                        {
                                            DB_PR("----test--.\r\n");  
                                            database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                            database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                            nvs_wr_phone_number_nvs_gz(1);
                                            nvs_wr_mima_number_nvs_gz(1);
                                        }

                                        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                        //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
  
                                        //database_gz[database_cw.dIndx].state_gz =0;

                                        database_gz[database_cw.dIndx].changqi =0;
                                        //database_gz[database_cw.dIndx].lock =0;
                                        nvs_wr_cunwu_mode_gz(1);
                                        //nvs_wr_dzx_mode_gz(1);
         
                                        //nvs_wr_state_gz(1);
                                        nvs_wr_glongtime_gz(1);
                                        //nvs_wr_glock_gz(1);
                                        
                                        //if(database_gz[database_cw.dIndx].changqi == 1)
                                        {
                                            //update xianshi todo
                                            tongbu_changqi();
                                        }  

                                    }

                                }
                                else
                                {
wuci_xmh_unchangqi:
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
                                    return_cause = 6;
                                    DB_PR("----admin --wu ci xiangmenhao-----.\r\n");
                                }
                                //send_cmd_to_lcd_bl(0x1130,database_gz[database_cw.dIndx].dIndx_gz);//xiangmen------------


                                
                                break;













                            case 0x11e0:
                                //5A A5 10 83   10 50   06    31 32 33 34 35 36 37 38 39 30  31 00
                                DB_PR("-admin1-mima1 number--.\r\n");
                                //panduan   -  zan cun quanju
                                //if has, todo  -----------------------------------

                                //DB_PR("data_rx_t[6]=%d---.\r\n",data_rx_t[6]);
                                if(03== data_rx_t[6])//12 04
                                {
                                    //zancun
                                    mima_weishu_ok_a1 =1;
                                    
                                    memcpy( mima_number_a1,data_rx_t+7 ,6);




                                    DB_PR("mima_number=");
                                    uart0_debug_str(mima_number_a1,6);

                                    // if(0==memcmp(mima_number_a1,buff_t,6))
                                    // {
                                    //     mima_weishu_ok_a1 =0;
                                    //     DB_PR("------mima_number_a1 all=0-------.\r\n");
                                    //     send_cmd_to_lcd_pic(0x0053);
                                    //     break;
                                    // }


                                    for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if((data_rx_t[i] == 0xFF)
                                            ||(data_rx_t[i] == 0x00))
                                        {
                                            mima_weishu_ok_a1 =0;
                                            send_cmd_to_lcd_pic(0x0053);
                                            DB_PR("--no--mima_weishu 1---.\r\n");
                                            break;
                                        }
                                    }
                                    DB_PR("\r\n");
                                    // for (int i = 7; i < 7+ data_rx_t[6] *2 -2 ; i++) {
                                    //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                    //     if(data_rx_t[i] == 0xFF)
                                    //     {
                                    //         mima_weishu_ok_a1 =0;
                                    //         DB_PR("--no--mima_weishu_ok_a1=%d---.\r\n",mima_weishu_ok_a1);
                                    //     }
                                    // }
                                    // DB_PR("\r\n");

                                    if(mima_weishu_ok_a1 == 1)
                                        DB_PR("-yes-mima_weishu_ok_a1=%d---.\r\n",mima_weishu_ok_a1);

                                }
                                else
                                {
                                    DB_PR("-------1 - mima weishu err--------.\r\n");
                                }
                                // send_cmd_to_lcd_bl_len(0x11e0,(uint8_t*)buff_t,30+5);//key
                                break;

                            case 0x11f0:
                                
                                //5A A5 0A 83   10 60   03   31 32 33 34 35 36 
                                DB_PR("-admin2-mima2 number--.\r\n");
                                //DB_PR("---phone_weishu_ok=%d---.\r\n",phone_weishu_ok);



                                //存物的格口编号（123）、格口类型（1：小，2：中，3：大）
                                //存物手机号（11位）密码（6位）            或者指纹(----)   


                                if((1 == mima_weishu_ok_a1)&&(03== data_rx_t[6]))//6  04   ok todo shoujihao yiyou
                                {
                                    
                                    mima_weishu_ok_a1 =0;
                                    memcpy( mima_number_a2,data_rx_t+7 ,6);

                                    for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if((data_rx_t[i] == 0xFF)
                                            ||(data_rx_t[i] == 0x00))
                                        {
                                            send_cmd_to_lcd_pic(0x0053);
                                            DB_PR("--no--mima_weishu 2---.\r\n");
                                            break;
                                        }
                                    }
                                    DB_PR("\r\n");

                                    DB_PR("mima_number1=");
                                    uart0_debug_str(mima_number_a1,6);

                                    DB_PR("mima_number2=");
                                    uart0_debug_str(mima_number_a2,6);



                                    // for (int i = 7; i < 7+ data_rx_t[6] *2 -2 ; i++) {
                                    //     DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                    //     if(data_rx_t[i] == 0xFF)
                                    //     {
                                    //         DB_PR("--no--mima_weishu_ok---.\r\n");
                                    //         goto done_mima_nosame;
                                    //     }
                                    // }
                                    // DB_PR("\r\n");


                                    uint16_t j=0,k=0;
                                    uint32_t mima_number_adm_temp1;  //
                                    uint32_t mima_number_adm_temp2;  //
   

                                    mima_number_adm_temp1 = atoi((const char*)mima_number_a1);
                                    mima_number_adm_temp2 = atoi((const char*)mima_number_a2);

                                    DB_PR("mima1?=%6u,mima2?=%6u,", mima_number_adm_temp1, mima_number_adm_temp2);
                                    if(mima_number_adm_temp1 == mima_number_adm_temp2)
                                    {
                                        database_ad.mima_number_adm = mima_number_adm_temp2;
                                    }
                                    else
                                    {
                                        goto done_mima_nosame;
                                    }

                                    send_cmd_to_lcd_bl(0x10e0,database_gz[database_cw.dIndx].dIndx_gz);
                                    send_cmd_to_lcd_pic(XIN_MIMA_ADMIN_OK_PIC);
                                    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 3000000));//3s
                                    DB_PR("Started timers, time since boot: %lld us", esp_timer_get_time());

                                    DB_PR("-----2-----[ * ] gai -mima success");
                                    nvs_wr_mima_number_adm(1);

                                    send_cmd_to_lcd_bl_len(0x11e0,(uint8_t*)buff_t,30+5);//key
                                    send_cmd_to_lcd_bl_len(0x11f0,(uint8_t*)buff_t,30+5);//key2

                                }
                                else
                                {
                                    if(03!= data_rx_t[6])
                                    {
                                      DB_PR("----------------2 - mima weishu err---------------.\r\n");  
                                    }
done_mima_nosame:
                                    DB_PR("----test2-error--.\r\n");  
                                    send_cmd_to_lcd_pic(FAIL_MIMA_ADMIN_OK_PIC);
                                }

                                DB_PR("----test3-done--.\r\n");  

                                // send_cmd_to_lcd_bl(0x11e0,0);//key
                                // send_cmd_to_lcd_bl(0x11f0,0);//key2

                                break;







                            case 0x2070:
                                DB_PR("--will default--.\r\n");

                                send_cmd_to_lcd_pic(DEFAULT_ADMIN_OK_PIC);
                                default_factory_set();
                                send_cmd_to_lcd_pic(BOOT_PIC);

                                


                                break;


                            case 0x1200:
                                DB_PR("--sound setting--.\r\n");
                                if(02== data_rx_t[8])
                                {
                                    DB_PR("-------yuyin off-----------.\r\n");
                                    audio_play_mp3_stop =1;
                                    send_cmd_to_lcd_pic(MUSIC_OFF_PIC);//default
                                    //close_mp3();
                                    //xTaskCreate(audio_init, "audio_init1", 2048, (void*)TONE_TYPE_OPEN, 10, NULL);   
                                }
                                if(01== data_rx_t[8])
                                {
                                    audio_play_mp3_stop =0;
                                    DB_PR("-------yuyin on------------.\r\n");
                                    send_cmd_to_lcd_pic(MUSIC_ON_PIC);
                                    //esp_restart();
                                    //xTaskCreate(audio_init, "audio_init2", 2048, NULL, 3, NULL);   

                                }
                                nvs_wr_mp3_ctl(1);
                                DB_PR("00---audio_play_mp3_stop=%d----\r\n",audio_play_mp3_stop);
                                break;


                            case 0x12f0://mp3 switch
                                DB_PR("---mp3 switch---.\r\n");   
                                // send_cmd_to_lcd_bl_len(0x10c0,(uint8_t*)buff_t,2*2+5);//key
                                if(audio_play_mp3_stop ==0)
                                {
                                    DB_PR("--2 mp3 on--.\r\n");  
                                    send_cmd_to_lcd_pic(MUSIC_ON_PIC);

                
                                }
                                else if(audio_play_mp3_stop ==1)
                                {
                                    DB_PR("--2 mp3 off--.\r\n");  
                                    send_cmd_to_lcd_pic(MUSIC_OFF_PIC);
                                }

                                break;




                            case 0x1290://zw luru fail   todo
                                DB_PR("---return_cause_zw_fail=%d---.\r\n",return_cause_zw_fail);   
                                DB_PR("--- 1 zw luru fail ---.\r\n");   
                                if(return_cause_zw_fail == 2)//changqi
                                {
                                    DB_PR("--2 changqi--.\r\n");  
                                    send_cmd_to_lcd_pic(CHANQI_CW_MODE_PIC);
                                }
                                else if(return_cause_zw_fail == 3)//qu
                                {
                                    DB_PR("--2 qu  zw--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0009);
                                }
                                else
                                {
                                    DB_PR("--other cunwu --.\r\n");  
                                    //baocun 1
                                    send_cmd_to_lcd_pic(0x0002);
                                }
                                return_cause_zw_fail =0;
                                // send_cmd_to_lcd_pic(0x0002);
                                // DB_PR("2-vTask: delete vTask1.\r\n");
                                // return_cause_zw =1;
                                // vTaskDelete(taskhandle1);
                                
                                break;


                            case 0x12a0:// 0x004d
                                DB_PR("--- 2 zw handshake fail ---.\r\n");    

                                if(return_cause_zw_handshake_fail == 2)//quwu zw
                                {
                                    DB_PR("--2 quwu zw--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0009);//0x000c
                                }
                                else if(return_cause_zw_handshake_fail == 1)
                                {
                                    if(database_cw_adm.changqi_tmp == 1)
                                    {
                                        DB_PR("--2 changqi--.\r\n");  
                                        send_cmd_to_lcd_pic(CHANQI_CW_MODE_PIC);
                                    }
                                    else if(database_cw_adm.changqi_tmp == 0)
                                    {
                                        DB_PR("--2 zw cun--.\r\n");  
                                        send_cmd_to_lcd_pic(0x0002);
                                    }
                                }
  
                                return_cause_zw_handshake_fail =0;

                                break;

                            case 0x12b0://   0x004e
                                DB_PR("--- 3 zw yikaishouzi fanhui ---.\r\n");    
                                if(database_cw_adm.changqi_tmp == 0)
                                {
                                    send_cmd_to_lcd_pic(0x0002);
                                }
                                else
                                {
                                    send_cmd_to_lcd_pic(CHANQI_CW_MODE_PIC);
                                }
                                return_cause_zw =1;//add

                                break;


                            case 0x12d0://zw chongfu
                                DB_PR("--- 3 zw congfu ---.\r\n"); 
                                DB_PR("---database_cw_adm.changqi_tmp=%d.\r\n",database_cw_adm.changqi_tmp);      
                                if(database_cw_adm.changqi_tmp == 1)
                                {
                                    send_cmd_to_lcd_pic(CHANQI_CW_MODE_PIC);
                                    DB_PR("--2 changqi--.\r\n");  
                                }
                                else
                                {
    
                                    if(((shengyu_xiao==0) && (shengyu_zhong==0))
                                        ||((shengyu_da==0) && (shengyu_xiao==0))
                                        ||((shengyu_da==0) && (shengyu_zhong==0)))
                                    {
                                        DB_PR("--2 cunwu--.\r\n");  
                                        //baocun 1
                                        send_cmd_to_lcd_pic(0x0002);
                    
                                    }
                                    else
                                    {
                                        DB_PR("--2 gekou--.\r\n");  
                                        send_cmd_to_lcd_pic(0x0003);
                                    }
                                }
                                
                                
                                
                                return_cause_zw =1;
                                //DB_PR("1-vTask: delete vTask1. taskhandle1=%d\r\n"(int)*taskhandle1);
                                // vTaskDelete(taskhandle1);


                                break;




                            case 0x1260://zhiwen  close
                                DB_PR("---zhiwen close---database_cw_adm.changqi_tmp=%d.\r\n",database_cw_adm.changqi_tmp);   
                                //if -> huise tupian?
                                if(database_cw_adm.changqi_tmp == 1)
                                {
                                    send_cmd_to_lcd_pic(CHANQI_CW_MODE_PIC);
                                    DB_PR("--2 changqi--.\r\n");  
                                }
                                else
                                {
    
                                    if(((shengyu_xiao==0) && (shengyu_zhong==0))
                                        ||((shengyu_da==0) && (shengyu_xiao==0))
                                        ||((shengyu_da==0) && (shengyu_zhong==0)))
                                    {
                                        DB_PR("--2 cunwu--.\r\n");  
                                        //baocun 1
                                        send_cmd_to_lcd_pic(0x0002);
                    
                                    }
                                    else
                                    {
                                        DB_PR("--2 gekou--.\r\n");  
                                        send_cmd_to_lcd_pic(0x0003);
                                    }
                                }
                                return_cause_zw =1;

                                // if( taskhandle1 != NULL ) //todo
                                // {
                                //     //DB_PR("1-vTask: delete vTask1. taskhandle1=%d\r\n"(int)*taskhandle1);
                                //     vTaskDelete(taskhandle1);
                                // }
                                // else
                                // {
                                //     DB_PR("taskhandle1 is NULL.\r\n");
                                // }

                                break;




                            case 0x12c0://zw qu 
                                DB_PR("--- 3 zw qu ---.\r\n");    

                                send_cmd_to_lcd_pic(0x0009);
                                DB_PR("3-vTask: delete shua_zhiwen_task.\r\n");
                                return_cause_zw =1;
                                break;



                            case 0x12e0://shoujihao close
                                DB_PR("---shoujihao cun close---.\r\n");   

                                if(((shengyu_xiao==0) && (shengyu_zhong==0))
                                    ||((shengyu_da==0) && (shengyu_xiao==0))
                                    ||((shengyu_da==0) && (shengyu_zhong==0)))
                                {
                                    DB_PR("--2 cunwu--.\r\n");  
                                    //baocun 1
                                    send_cmd_to_lcd_pic(0x0002);
                
                                }
                                else
                                {
                                    DB_PR("--2 gekou--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0003);
                                }
                                
                                return_cause_phone =1;
                                break;

                            case 0x2090://mima chongfu close
                                DB_PR("---mima chongfu close---.\r\n");   

                                if(database_cw_adm.changqi_tmp == 0)
                                {
                                    DB_PR("--2 cunwu mima--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0006);
                                }
                                else
                                {
                                    send_cmd_to_lcd_pic(0x001d);
                                    DB_PR("--2 changqi--.\r\n");  
                                }
                                break;


                            case 0x20A0://
                                DB_PR("---firmware shengji request---.\r\n");   
                                if(1==wifi_connected_flag)
                                {
                                    xTaskCreate(&simple_ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);//8192
                                }
                                else
                                {
                                    DB_PR("---wifi discon---.\r\n");   
                                    send_cmd_to_lcd_pic(0x005c);//0x0054
                                }
                                
                                
                                
                                break;


//-----------------------------------------cun-----------------------------------------------------
                            case 0x2080://
                                DB_PR("--cunwu--.\r\n");   

                                if((0== shengyu_da)
                                    &&(0== shengyu_zhong)
                                    &&(0== shengyu_xiao))
                                {
                                    DB_PR("--zanwu kongxiang--.\r\n"); // houbian sheng  
                                    send_cmd_to_lcd_pic(0x0001);
                                    return_cause_zanwu_kx =1;
                                    //     /* Start the timers */
                                    // ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 3000000));//3s
                                    // DB_PR("Started timers, time since boot: %lld us", esp_timer_get_time());

                                }
                                else//
                                {
                                    send_cmd_to_lcd_pic(0x0002);
                                }

                                break;

                            case 0x2010://zhiwen  or   mima
                                DB_PR("---zhiwen or mima---.\r\n");   
                                //if -> huise tupian?
                                if(01== data_rx_t[8])//zhiwen cun
                                {
                                    DB_PR("--zhiwen--.\r\n");  
                                    //baocun 1
                                    database_cw.cunwu_mode = 1;
                                    database_cw_adm.changqi_tmp = 0;
                                }
                                else if(02== data_rx_t[8])//mi ma
                                {
                                    DB_PR("--mima--.\r\n");  
                                    //baocun 2
                                    database_cw.cunwu_mode = 2;
                                }


                                send_cmd_to_lcd_bl_len(0x1050,(uint8_t*)buff_t,30+5);//phone
                                send_cmd_to_lcd_bl_len(0x1060,(uint8_t*)buff_t,30+5);//key
                                // send_cmd_to_lcd_bl(0x1050,0);//phone
                                // send_cmd_to_lcd_bl(0x1060,0);//key


                                if(((shengyu_xiao==0) && (shengyu_zhong==0))
                                    ||((shengyu_da==0) && (shengyu_xiao==0))
                                    ||((shengyu_da==0) && (shengyu_zhong==0)))
                                {
                                    if((shengyu_xiao==0) && (shengyu_zhong==0))
                                    {
                                        database_cw.dzx_mode = 1;
                                    }
                                    else if((shengyu_da==0) && (shengyu_xiao==0))
                                    {
                                        database_cw.dzx_mode = 2;
                                    }
                                    else if((shengyu_da==0) && (shengyu_zhong==0))
                                    {
                                        database_cw.dzx_mode = 3;
                                    }
                                    DB_PR("-----debug------database_cw.dzx_mode=%d\r\n",database_cw.dzx_mode);//j=0  

                                    if(01== data_rx_t[8])//zhiwen cun
                                    {
                                        DB_PR("--2 zhiwen--.\r\n");  
                                        //baocun 1
                                        send_cmd_to_lcd_pic(0x0004);
                                        xTaskCreate(Add_FR_First, "add_zhiwen_task1", 8* 1024, NULL, 2, &taskhandle1);//1024 10
                                    }
                                    else if(02== data_rx_t[8])//mi ma
                                    {
                                        DB_PR("--2 mima--.\r\n");  
                                        //baocun 2
                                        send_cmd_to_lcd_pic(0x0006);
                                    }
                                    // DB_PR("---2 da---.\r\n");  
                                }
                                else
                                {
                                    send_cmd_to_lcd_pic(0x0003);

                                }

                                break;

                            case 0x1250://mima? zanwukongxiang close
                                DB_PR("---return_cause_zanwu_kx=%d---.\r\n",return_cause_zanwu_kx);   
                                //if -> huise tupian?

                                // if(((shengyu_xiao==0) && (shengyu_zhong==0))
                                //     ||((shengyu_da==0) && (shengyu_xiao==0))
                                //     ||((shengyu_da==0) && (shengyu_zhong==0)))
                                if((return_cause_zanwu_kx == 3)||
                                    (return_cause_zanwu_kx == 4))
                                {
                                    DB_PR("--2 gekou--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0003);
                                }
                                else if(return_cause_zanwu_kx == 1)
                                {
                                    DB_PR("--2 zhuye--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0026);
                                }
                                else
                                {
                                    DB_PR("--other cunwu --.\r\n");  
                                    //baocun 1
                                    send_cmd_to_lcd_pic(0x0002);
                                }
                                return_cause_zanwu_kx =0;

                                break;





                            case 0x2020://da zhong xiao    ke sheng
                                DB_PR("----------------dazhxiao---------------.\r\n");   

                                bool flag_temp=1;
                                //todo
                                if((01== data_rx_t[8])&&(0!=shengyu_da))//da
                                {
                                    DB_PR("---da---.\r\n");  
                                    //cun qi lai   quanju yihuiyong
                                    database_cw.dzx_mode = 1;
                                }
                                else if((02== data_rx_t[8])&&(0!=shengyu_zhong))//zhong
                                {
                                    DB_PR("---zhong---.\r\n");  
                                    database_cw.dzx_mode = 2;
                                }
                                else if((03== data_rx_t[8])&&(0!=shengyu_xiao))//xiao
                                {
                                    DB_PR("---xiao---.\r\n");  
                                    database_cw.dzx_mode = 3;
                                }
                                else
                                {
                                    DB_PR("---zanwu kongxiang---.\r\n");  
                                    // tx_Buffer[8] = 0x00;
                                    // tx_Buffer[9] = 0x01;
                                    send_cmd_to_lcd_pic(0x0001);
                                    
                                    flag_temp =0 ;

                                }


                                if(flag_temp == 1)
                                {
                                    return_cause_zanwu_kx =4;
                                    if(02 == database_cw.cunwu_mode)//2010 密码
                                    {
                                        send_cmd_to_lcd_pic(0x0006);
                                    }
                                    else if(01 == database_cw.cunwu_mode)//2010 指纹 ->指纹判断 0005 todo
                                    {
                                        send_cmd_to_lcd_pic(0x0004);
                                        xTaskCreate(Add_FR_First, "add_zhiwen_task1", 8* 1024, NULL, 2, &taskhandle1);//1024 10
                                    } 
                                }
                                else
                                {
                                    return_cause_zanwu_kx =3;
                                }
                                
 
                                // send_cmd_to_lcd_bl(0x1050,0);//phone
                                // send_cmd_to_lcd_bl(0x1060,0);//key
                                send_cmd_to_lcd_bl_len(0x1050,(uint8_t*)buff_t,30+5);//phone
                                send_cmd_to_lcd_bl_len(0x1060,(uint8_t*)buff_t,30+5);//key
                                

                                break;



                            case 0x1050:
                                //5A A5 10 83   10 50   06    31 32 33 34 35 36 37 38 39 30  31 00
                                DB_PR("--phone number--.\r\n");
                                //panduan   -  zan cun quanju
                                //if has, todo  -----------------------------------

                                //DB_PR("data_rx_t[6]=%d---.\r\n",data_rx_t[6]);
                                if((06== data_rx_t[6])
                                    &&(0xFF== data_rx_t[len_rx_t -3]))//12
                                {
                                    //zancun
                                    phone_weishu_ok =1;
                                    memcpy( phone_number,data_rx_t+7 ,11);
                                    //phone_number[11] = '\0';//no use
                                    

                                    for (int i = 7; i < 7+ data_rx_t[6] *2 -1; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            phone_weishu_ok =0;
                                            DB_PR("--no--phone_weishu_ok=%d---.\r\n",phone_weishu_ok);
                                        }
                                    }
                                    DB_PR("\r\n");

                                    if(phone_weishu_ok == 1)
                                        DB_PR("-yes-phone_weishu_ok=%d---.\r\n",phone_weishu_ok);

                                }
                                else
                                {
                                    DB_PR("-------1 - shoujihao weishu err--------.\r\n");
                                }
                                
                                break;

                            case 0x1060:
                                
                                
                                //5A A5 0A 83   10 60   03   31 32 33 34 35 36 
                                DB_PR("--password--.\r\n");
                                //DB_PR("---phone_weishu_ok=%d---.\r\n",phone_weishu_ok);

                                // send_cmd_to_lcd_bl_len(0x1050,(uint8_t*)buff_t,12+5);//phone
                                // send_cmd_to_lcd_bl_len(0x1060,(uint8_t*)buff_t,6+5);//key

                                //存物的格口编号（123）、格口类型（1：小，2：中，3：大）
                                //存物手机号（11位）密码（6位）            或者指纹(----)   


                                if((1 == phone_weishu_ok)&&(03== data_rx_t[6]))//6   ok todo shoujihao yiyou
                                {

                                    memcpy( mima_number,data_rx_t+7 ,6);

                                    DB_PR("phone_number=");
                                    uart0_debug_str(phone_number,11);

                                    DB_PR("mima_number=");
                                    uart0_debug_str(mima_number,6);

                                    if(0==memcmp(mima_number,buff_t,6))
                                    {
                                        DB_PR("------mima all=0-------.\r\n");
                                        goto done;
                                    }

                                    for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            DB_PR("--no--mima_weishu_ok---.\r\n");
                                            goto done;
                                        }
                                    }
                                    DB_PR("\r\n");



            
                                    database_cw.phone_number_nvs = atoll((const char*)phone_number);
                                    database_cw.mima_number_nvs = atoi((const char*)mima_number);
                                    DB_PR("phone?=%11llu,mima?=%6u,", database_cw.phone_number_nvs, database_cw.mima_number_nvs);
                                    

                                    
                                    for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                                    {

                                        if(database_gz[i].phone_number_nvs_gz == database_cw.phone_number_nvs)
                                        {
                                            if(database_gz[i].mima_number_nvs_gz == database_cw.mima_number_nvs) 
                                            {
                                                //chongfu_flag =1;
                                                DB_PR("---phone_number and key_number has in database\r\n");
                                                send_cmd_to_lcd_pic(0x001F);//todo
                                                goto done_2;
                                            }
                                            else
                                            {
                                            
                                                DB_PR("---only mima_number will xin, phone_number has in database\r\n");
                                            }


                                        }
                                        else//ok
                                        {
                                            //DB_PR("---phone_number and mima will xin\r\n");
                                        }
                                        
                                    }
                                    DB_PR("---phone_number and mima will xin\r\n");







                                    uint16_t j=0,k=0;
                                    uint16_t database_gz_temp[SHENYU_GEZI_MAX]={0};
                                    uint16_t database_gz_temp_onuse[SHENYU_GEZI_MAX]={0};

                                    // start ----------------------------------------
                                    if((int16_t)shengyu_all<=0)
                                    {
                                        send_cmd_to_lcd_pic(0x0001);
                                        return_cause_zanwu_kx =2;//del
                                        goto done_2;
                                    }

                                    if(1 == database_cw.dzx_mode)
                                    {
                                        
                                        for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                                        {
                                            if((database_gz[i].state_gz ==0) //no use
                                                &&(database_gz[i].state_fenpei_gz == 1)
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==1)
                                                &&(database_gz[i].cunwu_mode_gz==0))//d   2 is shouji
                                            {
                                                database_gz_temp[j++] =i;
                                            }
                                            else if((database_gz[i].state_fenpei_gz == 1)
                                                &&(database_gz[i].dzx_mode_gz ==1) )//d
                                            {
                                                database_gz_temp_onuse[k++] =i;
                                            }
                                            // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                            //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                                        }
                                        DB_PR("shengyu j=%d, onuse k=%d\r\n",j,k);
                                        uart0_debug_data_dec(database_gz_temp,j);
                                        uart0_debug_data_dec(database_gz_temp_onuse,k);//no

                                        if(j>0)
                                        {
                                            srand((unsigned int) time(NULL));
                                            database_cw.dIndx = database_gz_temp[rand() %j];//随机获取哪个门没用

                                            database_cw.state=1;
                                            // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                            DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);

                                            DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);



                                            guimen_gk_temp = database_cw.dIndx ;
                                            uint16_t j=0,k=0;

                                            k = guimen_gk_temp/24;
                                            j = guimen_gk_temp%24;

                                            if(guimen_gk_temp%24 ==0)
                                            {
                                                k = guimen_gk_temp/24 -1;
                                                j = 24;
                                            }

                                            DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);


    

                                            // if(shengyu_da>0)
                                            {
                                                //old
                                                shengyu_da --;
                                                tongbu_gekou_shuliang_d(shengyu_da);


                                                DB_PR("-da-lock:%d ok--.\r\n",j);

                                                send_cmd_to_lock(k+1,j);
                                                send_cmd_to_lcd_bl(0x1070,database_gz[database_cw.dIndx].dIndx_gz);//-----------
                                            }
                                        
                                        }    
                                        else
                                        {
                                            DB_PR("---1----state d have no-----.\r\n");
                                            goto done;
                                        }
                                        

                                    }
                                    else if(2 == database_cw.dzx_mode)
                                    {
                                        j=0;
                                        for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                                        {
                                            if((database_gz[i].state_gz ==0) //no use
                                                &&(database_gz[i].state_fenpei_gz == 1)
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==2)
                                                &&(database_gz[i].cunwu_mode_gz==0))//z
                                            {
                                                database_gz_temp[j++] =i;
                                            }
                                            else if((database_gz[i].state_fenpei_gz == 1)
                                                &&(database_gz[i].dzx_mode_gz ==2))//z
                                            {
                                                database_gz_temp_onuse[k++] =i;
                                            }
                                            // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                            //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                                        }
                                        DB_PR("shengyu j=%d, onuse k=%d\r\n",j,k);
                                        uart0_debug_data_dec(database_gz_temp,j);
                                        uart0_debug_data_dec(database_gz_temp_onuse,k);

                                        if(j>0)
                                        {
                                            srand((unsigned int) time(NULL));
                                            database_cw.dIndx = database_gz_temp[rand() %j];//随机获取哪个门没用

                                            database_cw.state=1;
                                            // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                            DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                                            DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);



                                            guimen_gk_temp = database_cw.dIndx ;
                                            uint16_t j=0,k=0;

                                            k = guimen_gk_temp/24;
                                            j = guimen_gk_temp%24;

                                            if(guimen_gk_temp%24 ==0)
                                            {
                                                k = guimen_gk_temp/24 -1;
                                                j = 24;
                                            }

                                            DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                            //if(shengyu_zhong >0)
                                            {
                                                shengyu_zhong --;
                                                tongbu_gekou_shuliang_z(shengyu_zhong);

                                                //DB_PR("--lock2 ok--.\r\n");

                                                DB_PR("-zhong-lock:%d ok--.\r\n",j);
                                                send_cmd_to_lock(k+1,j);
                                                send_cmd_to_lcd_bl(0x1070,database_gz[database_cw.dIndx].dIndx_gz);
                                                
                                            }

                                        }
                                        else
                                        {
                                            DB_PR("---2----state z have no-----.\r\n");
                                            goto done;
                                        }
                                        

                                    }
                                    else if(3 == database_cw.dzx_mode)
                                    {

                                        for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                                        {
                                            if((database_gz[i].state_gz ==0) //no use
                                                &&(database_gz[i].state_fenpei_gz == 1)
                                                &&(database_gz[i].lock == 0)
                                                &&(database_gz[i].changqi == 0)
                                                &&(database_gz[i].dzx_mode_gz ==3)
                                                &&(database_gz[i].cunwu_mode_gz==0))//x
                                            {
                                                database_gz_temp[j++] =i;
                                            }
                                            else if((database_gz[i].state_fenpei_gz == 1)
                                                &&(database_gz[i].dzx_mode_gz ==3))//x
                                            {
                                                database_gz_temp_onuse[k++] =i;
                                            }
                                            // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                            //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                                        }
                                        DB_PR("\r\nshengyu j=%d, onuse k=%d\r\n",j,k);
                                        uart0_debug_data_dec(database_gz_temp,j);
                                        uart0_debug_data_dec(database_gz_temp_onuse,k);

                                        if(j>0)
                                        {
                                            srand((unsigned int) time(NULL));
                                            database_cw.dIndx = database_gz_temp[rand() %j];//随机获取哪个门没用



                                            //database_cw.dIndx = database_gz_temp[0%j];//随机获取哪个门没用

                                            for (uint16_t i = 0; i < j+5; i++)
                                            {
                                                DB_PR("---database_gz_temp[%d]=%u\r\n",i,database_gz_temp[i%j]);
                                            }


                                            database_cw.state=1;
                                            // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                            DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                                            DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);

                                            //database_cw.dIndx = 43;//-------14--------


                                            guimen_gk_temp = database_cw.dIndx ;
                                                uint16_t j=0,k=0;

                                            k = guimen_gk_temp/24;
                                            j = guimen_gk_temp%24;

                                            if(guimen_gk_temp%24 ==0)
                                            {
                                                k = guimen_gk_temp/24 -1;
                                                j = 24;
                                            }

                                            DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);


                                            
                                            //if(shengyu_xiao >0)
                                            {
                                                shengyu_xiao --;
                                                tongbu_gekou_shuliang_x(shengyu_xiao);
                                                //DB_PR("----------------lock3 ts---------------.\r\n");
                                                
                                                DB_PR("-xiao-lock:%d ok--.\r\n",j);

                                                send_cmd_to_lock(k+1,j);
                                                send_cmd_to_lcd_bl(0x1070,database_gz[database_cw.dIndx].dIndx_gz);

                                            }
    


                                        }
                                        else
                                        {
                                            DB_PR("----3---state x have no-----.\r\n");
                                            goto done;
                                        }
                                            
                                    }

                                    DB_PR("-----2-----[ * ] Starting audio pipeline");
                                    xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);

                                    DB_PR("----test--.\r\n");  
                                    if(0 != database_cw.dzx_mode)
                                    {
                                        shengyu_all -- ;
                                        tongbu_gekou_shuliang_all(shengyu_all);



                                        //guizi     ------------save-------------------
                                        database_gz[database_cw.dIndx].cunwu_mode_gz = database_cw.cunwu_mode;
                                        nvs_wr_cunwu_mode_gz(1);


                                        // database_gz[database_cw.dIndx].dzx_mode_gz = database_cw.dzx_mode;
                                        // nvs_wr_dzx_mode_gz(1);


                                        //database_cw.phone_number_nvs = atoll((const char*)phone_number);
                                        database_gz[database_cw.dIndx].phone_number_nvs_gz = database_cw.phone_number_nvs;
                                        nvs_wr_phone_number_nvs_gz(1);


                                        //database_cw.mima_number_nvs = atoi((const char*)mima_number);
                                        database_gz[database_cw.dIndx].mima_number_nvs_gz = database_cw.mima_number_nvs;
                                        nvs_wr_mima_number_nvs_gz(1);


                                        database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                        nvs_wr_state_gz(1);


                                        //admin   ------------save-------------------
                                        nvs_wr_shengyu_da(1);
                                        nvs_wr_shengyu_zhong(1);
                                        nvs_wr_shengyu_xiao(1);




                                        //custumer   mingming kongjian   todo
                                        // esp_err_t err = save_u16_value(STORAGE_NAMESPACE,"dw_dIndx",database_cw.dIndx);
                                        // if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                        // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
                                        // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                                        //unique number


                                        send_cmd_to_lcd_pic(0x0008);



                                    }
                                    // uart0_debug_data_dec(database_gz_temp,j);
                                    // uart0_debug_data_dec(database_gz_temp_onuse,k);
                                    //cun
                                    
                                    


                                    //log debug
                                    j=0;k=0;
                                    for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                                    {
                                        if((database_gz[i].state_gz ==0) //no use
                                            &&(database_gz[i].state_fenpei_gz == 1)
                                            &&(database_gz[i].lock == 0)
                                            &&(database_gz[i].changqi == 0)                                            
                                            &&(database_gz[i].cunwu_mode_gz==database_cw.cunwu_mode)
                                            &&(database_gz[i].dzx_mode_gz ==database_cw.dzx_mode))//d
                                        {
                                            database_gz_temp[j++] =i;
                                        }
                                        else if((database_gz[i].state_fenpei_gz == 1)
                                            &&(database_gz[i].dzx_mode_gz ==database_cw.dzx_mode))//d
                                        {
                                            database_gz_temp_onuse[k++] =i;
                                        }
                                        // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                        //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                                    }
                                    DB_PR("-------------shengyu j=%d, onuse k=%d\r\n",j,k);
                                    vTaskDelay(1 / portTICK_PERIOD_MS);
                                    uart0_debug_data_dec(database_gz_temp,j);
                                    uart0_debug_data_dec(database_gz_temp_onuse,k);//no
                                    DB_PR("---lock idx---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                                    DB_PR("---xiangmen---guimen_gk_temp=%u\r\n",guimen_gk_temp);
                                
                                
                                    database_cw.dzx_mode = 0 ;
                                    database_cw.cunwu_mode =0;
                                    // database_cw.dzx_mode = 0 ;
                                    database_cw.state=0;

                                    send_cmd_to_lcd_bl_len(0x1050,(uint8_t*)buff_t,12+5);//phone
                                    send_cmd_to_lcd_bl_len(0x1060,(uint8_t*)buff_t,6+5);//key
                                
                                }
                                else
                                {

                                    if(03!= data_rx_t[6])
                                    {
                                        DB_PR("-------2 - mima weishu err---------------.\r\n");  
                                    }
done:
                                    if(0==phone_weishu_ok)
                                    {
                                        DB_PR("-------2 - phone number weishu err--.\r\n");  
                                        
                                    }
                                    send_cmd_to_lcd_pic(0x0007);
                                    DB_PR("----test2-error--.\r\n");  

                                }
done_2:                             
                                phone_weishu_ok =0;
                                DB_PR("----test3-done--.\r\n");  

                                // send_cmd_to_lcd_bl(0x1050,0);//phone
                                // send_cmd_to_lcd_bl(0x1060,0);//key
                                



                                // phone_weishu_ok =0;


                                break;










//---------------------------------取物---------------------------------------------------
                            case 0x2030:
                                DB_PR("-----qu-----.\r\n");
                                if(02== data_rx_t[8])//12   mima
                                {
                                    database_cw.cunwu_mode = 2;//no use?
                                    send_cmd_to_lcd_pic(0x000a);
                                }
                                if(01== data_rx_t[8])//12 zhiwen
                                {
                                    database_cw.cunwu_mode = 1;
                                    send_cmd_to_lcd_pic(0x000c);   

                                    xTaskCreate(press_FR, "shua_zhiwen_task", 6* 1024, NULL, 2, NULL);//1024 10  
                                }
                                send_cmd_to_lcd_bl_len(0x1080,(uint8_t*)buff_t,30+5);//phone
                                send_cmd_to_lcd_bl_len(0x1090,(uint8_t*)buff_t,30+5);//key

                                break;

                            case 0x1080:
                                DB_PR("---q--phone-----.\r\n");
                                //if(05== data_rx_t[6])//12
                                if((06== data_rx_t[6])
                                    &&(0xFF== data_rx_t[len_rx_t -3]))//12
                                {
                                    //zancun
                                    phone_weishu_ok =1;
                                    memcpy( phone_number,data_rx_t+7 ,11);
                                    

                                    for (int i = 7; i < 7+ data_rx_t[6] *2 -1; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            phone_weishu_ok =0;
                                            DB_PR("q--no--phone_weishu_ok=%d---.\r\n",phone_weishu_ok);
                                        }
                                    }
                                    if(phone_weishu_ok == 1)
                                        DB_PR("q-yes-phone_weishu_ok=%d---.\r\n",phone_weishu_ok);
                                }
                                else
                                {
                                    DB_PR("q-------1 - shoujihao weishu err--------.\r\n");
                                }
                                break;
                            case 0x1090:
                                DB_PR("---q--mima-----.\r\n");
                                // send_cmd_to_lcd_bl_len(0x1080,(uint8_t*)buff_t,30+5);//phone
                                // send_cmd_to_lcd_bl_len(0x1090,(uint8_t*)buff_t,30+5);//key

                                //DB_PR("---phone_weishu_ok=%d---.\r\n",phone_weishu_ok);

                                //存物的格口编号（123）、格口类型（1：小，2：中，3：大）
                                //存物手机号（11位）密码（6位）            或者指纹(----)   


                                //if((1 == phone_weishu_ok)&&(03== data_rx_t[6]))//6   ok todo shoujihao yiyou
                                if((1 == phone_weishu_ok)&&(03== data_rx_t[6]))//03
                                {
                                    phone_weishu_ok =0;
                                    memcpy( mima_number,data_rx_t+7 ,6);

                                    DB_PR("phone_number=");
                                    uart0_debug_str(phone_number,11);

                                    DB_PR("mima_number=");
                                    uart0_debug_str(mima_number,6);


                                    if(0==memcmp(mima_number,buff_t,6))
                                    {
                                        DB_PR("------q mima all=0-------.\r\n");
                                        goto done_qu;
                                    }

                                    for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            DB_PR("--no--mima_weishu_ok---.\r\n");
                                            goto done_qu;
                                        }
                                    }
                                    DB_PR("\r\n");



            
                                    uint64_t phone_number_nvs_i;  //
                                    uint32_t mima_number_nvs_i;  //
                                    //uint16_t duiying_index;  //

                                    phone_number_nvs_i = atoll((const char*)phone_number);
                                    mima_number_nvs_i = atoi((const char*)mima_number);
                                    DB_PR("phone?=%11llu,mima?=%6u,", phone_number_nvs_i, mima_number_nvs_i);
                                    
        
                                    database_cw.dIndx = 0;
                                    for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                                    {

                                        if(database_gz[i].phone_number_nvs_gz == phone_number_nvs_i)
                                        {
                                            if(database_gz[i].mima_number_nvs_gz == mima_number_nvs_i) 
                                            {

                                                //chongfu_flag =1;
                                                database_cw.dIndx = i;//ok
                                                DB_PR("---yes- phone_number and key_number has in database\r\n");
                                            }

                                        }
                                        else//
                                        {
                                            //DB_PR("---phone_number and mima will xin\r\n");
                                        }
                                        
                                    }
                                    DB_PR("---add---xmh =%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);
                                    if((database_cw.dIndx == 0)||(database_gz[database_cw.dIndx].dIndx_gz > shengyu_all_max))
                                    {
                                        send_cmd_to_lcd_pic(0x000b); //0x000d
                                        DB_PR("---no find phone\r\n");
                                        // database_cw.dIndx = atoi((const char*)mima_number);//data_rx_t[7] - 0x30;
                                        //goto done_qu_zanwu;//todo
                                    }
                                    else
                                    {

                                        DB_PR("---phone_number and mima etc... will be deleted\r\n");

                                        // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                        DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);



                                        guimen_gk_temp = database_cw.dIndx ;

                                        uint16_t j=0,k=0;

                                        k = guimen_gk_temp/24;
                                        j = guimen_gk_temp%24;

                                        if(guimen_gk_temp%24 ==0)
                                        {
                                            k = guimen_gk_temp/24 -1;
                                            j = 24;
                                        }
                                        DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);

                                    
                

                                        DB_PR("--lock:%d ok--.\r\n",j);
                                        send_cmd_to_lock(k+1,j);
                                        send_cmd_to_lcd_bl(0x10a0,database_gz[database_cw.dIndx].dIndx_gz);//weishu bugou
                
                                        DB_PR("-----2-----[ * ] Starting audio pipeline\r\n");
                                        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);   





                                        if((database_gz[database_cw.dIndx].lock == 0)
                                            &&(database_gz[database_cw.dIndx].changqi == 0))
                                        {
                                            switch (database_gz[database_cw.dIndx].dzx_mode_gz)
                                            {
                                            case 1:
                                                //d
                                                shengyu_da ++;
                                                tongbu_gekou_shuliang_d(shengyu_da); 
                                                break;
                                            case 2:
                                                //z
                                                shengyu_zhong ++;
                                                tongbu_gekou_shuliang_z(shengyu_zhong); 
                                                break;
                                            case 3:
                                                //x
                                                shengyu_xiao ++;
                                                tongbu_gekou_shuliang_x(shengyu_xiao); 
                                                break;

                                            default:
                                                break;
                                            }

                                            shengyu_all ++ ;
                                            tongbu_gekou_shuliang_all(shengyu_all);
    
                                            nvs_wr_shengyu_da(1);
                                            nvs_wr_shengyu_zhong(1);
                                            nvs_wr_shengyu_xiao(1);

                                        }
           



                                        DB_PR("----test--.\r\n");  
                                        

                                        //if(0 != database_cw.dzx_mode)
                                        {

                                            // char key_name[15];//15
                                            // esp_err_t err;



                                            if((database_gz[database_cw.dIndx].changqi == 0)
                                                ||(database_gz[database_cw.dIndx].changqi == 2))
                                            {

    
                                                {
                                                    database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                                    database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                                    nvs_wr_phone_number_nvs_gz(1);
                                                    nvs_wr_mima_number_nvs_gz(1);
                                                }




                                                database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                                //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
                                                nvs_wr_cunwu_mode_gz(1);
                                                //nvs_wr_dzx_mode_gz(1);

                            
                                                database_gz[database_cw.dIndx].state_gz =0;
                                                nvs_wr_state_gz(1);

                                                DB_PR("--database_gz[database_cw.dIndx].changqi=%d\r\n",database_gz[database_cw.dIndx].changqi);
                                                if(database_gz[database_cw.dIndx].changqi == 2)
                                                {
                                                    database_gz[database_cw.dIndx].changqi =0;
                                                    nvs_wr_glongtime_gz(1);
                                                    tongbu_changqi();
                                                }
                                                

                                            }






                                            //custumer   mingming kongjian   todo
                                            // esp_err_t err = save_u16_value(STORAGE_NAMESPACE,"dw_dIndx",database_cw.dIndx);
                                            // if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                                            // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
                                            // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                                            //unique number

                                        }

                                        send_cmd_to_lcd_pic(0x000e); 



                                        database_cw.cunwu_mode =0;
                                        database_cw.state=0;
                                        database_cw.dzx_mode = 0 ;//?

                                        // send_cmd_to_lcd_bl(0x1080,0);//phone
                                        // send_cmd_to_lcd_bl(0x1090,0);//key
                                        send_cmd_to_lcd_bl_len(0x1080,(uint8_t*)buff_t,30+5);//phone
                                        send_cmd_to_lcd_bl_len(0x1090,(uint8_t*)buff_t,30+5);//key
                                    }

                                }
                                else
                                {
                                    if(03!= data_rx_t[6])
                                    {
                                      DB_PR("----------------2 - mima weishu err---------------.\r\n");  
                                    }
done_qu:
                                    DB_PR("----test2-error--.\r\n");  

                                    send_cmd_to_lcd_pic(0x000b); //

                                }
                                DB_PR("----test3-done--.\r\n");  

                                




                                
                                break;




//---------------------------------admin----------------------------------------------------
                            case 0x10b0://mima
                                DB_PR("----admin --mima-----.\r\n");

                                //mima 666888 todo----------------

                                if(03== data_rx_t[6])
                                {   
                                    memcpy( mima_number,data_rx_t+7 ,6);

                                    DB_PR("mima_number=");
                                    uart0_debug_str(mima_number,6);
                                    for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            DB_PR("--no--mima_weishu_ok---.\r\n");
                                            goto done_kai_admin;
                                        }
                                    }
                                    DB_PR("\r\n");

                                    uint32_t mima_number_nvs_i;  //
                                    //uint16_t duiying_index;  //

                                    mima_number_nvs_i = atoi((const char*)mima_number);
                                    DB_PR("mima?=%6u,", mima_number_nvs_i);


                                    // if((mima_number_nvs_i == 666888)
                                    //     ||(mima_number_nvs_i == database_ad.mima_number_adm))//read


                                    if((mima_number_nvs_i == database_ad.mima_number_adm))//read  todo
                                    //if(1)
                                    {
                                        send_cmd_to_lcd_pic(0x0011);
                                    }
                                    else
                                    {
                                        DB_PR("--no--mima_meiyou_ok---.\r\n");
                                        goto done_kai_admin;
                                    }
                                    
                                    
                                }
                                else
                                {
done_kai_admin:
                                    send_cmd_to_lcd_pic(0x0010);
                                    DB_PR("----admin --mima weisu err-----.\r\n");
                                }
                                
                                send_cmd_to_lcd_bl_len(0x10B0,(uint8_t*)buff_t,30+5);//key
                                // send_cmd_to_lcd_bl(0x10B0,0);//key
  
                                break;




                            default:
                                DB_PR("----------------83 default---------------.\r\n");
                                break;
                            }

                        }
                        else
                        {
                            DB_PR("----------------len2 err---------------.\r\n");
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


// ///command struct
// typedef struct
// {
// 	//uint8_t type;
// 	uint8_t len;
// 	uint8_t opcode;
// 	uint16_t sum;
// 	uint8_t dIndx;
// 	uint8_t data[256];
	
// }command_struct;

// command_struct g_data;


// #define CMD_SUCCESS 0x01
// #define CMD_FAIL	     0x00

// /////start process the data in
// uint8_t uState = 0;
// // static void user_cmd_buffer_clear(void)
// // {
// // 	uState = 0;
// // 	g_data.len = 0;
// // 	g_data.type = 0;
// // 	g_data.opcode = 0;
// // 	g_data.sum = 0;
// // 	g_data.dIndx = 0;
// // }

// ///command enum
// typedef enum
// {
// 	UART_IDLE =0,
// 	UART_HEADER,
// 	UART_HEADER2,
// 	// UART_TYPE,
// 	UART_LENGTH,
// 	UART_OPCODE,
// 	UART_PAYLOAD,
// 	UART_CHECKSUM,
// 	UART_RESERVE,
// }UART_STATE_ENUM;

// void user_send_cmd_response(uint8_t opCode, uint8_t rsp)
// {
// 	// uint8_t sum = 0;
// 	// uint8_t responseBuffer[32] = {0};
// 	// responseBuffer[0] = 0x55;
// 	// responseBuffer[1] = 0x02;
// 	// responseBuffer[2] = 4;
// 	// responseBuffer[3] = opCode;
// 	// responseBuffer[4] = rsp;

// 	// sum += responseBuffer[2];
// 	// sum += responseBuffer[3];
// 	// sum += responseBuffer[4];

// 	// responseBuffer[5] = sum;

//     //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);
// 	//spear_uart_send_datas(responseBuffer, 6);
// }
// bool spear_uart_process_data(uint8_t byt)
// {
// 	bool r = false;
//     uint8_t data_t[128];
//     // uint8_t data_crc1=0;
//     // uint8_t data_crc2=0;
// 	DB_PR(" %x: ", byt);
// 	switch(uState)
// 	{
// 		case UART_IDLE:
// 		case UART_HEADER:
// 			if(byt == 0x5A)
// 			{
// 				uState = UART_HEADER2;
// 				//user_start_cmd_receive(true);
// 			}
// 			//DB_PR("header = %02x\r\n", byt);
// 			break;

// 		case UART_HEADER2:
// 			if(byt == 0xA5)
// 			{
// 				uState = UART_LENGTH;
// 				//user_start_cmd_receive(true);
// 			}
// 			//DB_PR("header = %02x\r\n", byt);
// 			break;

// 		case UART_LENGTH:
// 			g_data.len = byt;//-3   no jiaoyan len
// 			g_data.dIndx = 0;
			
// 			uState++;
// 			//DB_PR("length = %02x\r\n", byt);
// 			break;
// 		case UART_OPCODE:
// 			g_data.opcode = byt;
// 			//g_data.sum += byt;
//             //g_data.sum = byt;
// 			uState++;
// 			//DB_PR("opcode = %02x\r\n", byt);
// 			break;
// 		case UART_PAYLOAD:



//             if(g_data.dIndx == g_data.len-3 )//crc1   2
//             {
//                 g_data.data[g_data.dIndx] = byt;
//                 //DB_PR("crc data[%d] = %02x\r\n",g_data.dIndx, byt);
//                 g_data.dIndx++;
//             }
// 			else if(g_data.dIndx < g_data.len-3 )//<   1
// 			{
// 				g_data.data[g_data.dIndx] = byt;
// 				//g_data.sum += byt;
// 				//DB_PR("data[%d] = %02x\r\n",g_data.dIndx, byt);
//                 g_data.dIndx++;
// 			}
// 			else//crc2
// 			{
//                 g_data.data[g_data.dIndx] = byt;
//                 // DB_PR("crc data[%d] = %02x\r\n",g_data.dIndx, byt);
//                 g_data.dIndx++;
// 				//user_start_cmd_receive(false);
// 				uState = 0;
//                 data_t[0] = g_data.opcode;
//                 memcpy(data_t+1,g_data.data,g_data.len -3);

//                 g_data.sum = CRC16(data_t, g_data.len -3 +1);
// 				// DB_PR("sum = %04x\r\n",g_data.sum);

//                 // DB_PR("g_data.data[g_data.dIndx -2] = %02x, g_data.data[g_data.dIndx-1] = %02x\r\n",g_data.data[g_data.dIndx -2], g_data.data[g_data.dIndx-1]);
//                 // DB_PR("g_data.sum &0xff = %02x, (g_data.sum>>8)&0xff = %02x\r\n",g_data.sum &0xff, (g_data.sum>>8)&0xff);


// 				if( ((g_data.sum &0xff) == g_data.data[g_data.dIndx -2])
//                     &&(((g_data.sum>>8)&0xff) == g_data.data[g_data.dIndx-1]) )//byt
// 				{
//                     DB_PR("处理数据1-pass\r\n");
// 					r = true;
// 				}
// 				else
// 				{
//                     DB_PR("处理数据1-fail\r\n");
// 					user_send_cmd_response(g_data.opcode, CMD_FAIL);
// 				}
// 			}
// 			break;
// 		case UART_RESERVE:
// 		default:
// 			break;
// 	}

// 	return r;
// }



                                    


static void lock_all_open_task()
{
    uint16_t j=0,k=0;

    send_cmd_to_lcd_pic(0x0049);
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        vTaskDelay(1);
        if(1== database_gz[i].state_fenpei_gz)
        {
            k = i/24;
            j = i%24;

            if(i%24 ==0)
            {
                k = i/24 -1;
                j = 24;
            }
            send_cmd_to_lock(k+1, j);
            //vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    }
    send_cmd_to_lcd_pic(ALLOPEN_OK_PIC);

    vTaskDelay(1);
    vTaskDelete(NULL);
}


static void lock_all_clear_task()
{

    int j=0,k=0,l=0;
    uint16_t m=0,n=0;

    send_cmd_to_lcd_pic(0x004b);


    u8  ensure;
	ensure=PS_Empty();//清空指纹库

	if(ensure==0)
	{
		//LCD_Fill(0,120,lcddev.width,160,WHITE);
		DB_PR("---del all zhiwen ok -----删除指纹成功 \r\n");		
	}
    else
		ShowErrMessage(ensure);	


    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        //vTaskDelay(1);
        database_cw.dIndx =i;
        nvs_wr_fenpei_gz(0);//2  _dz_fp
       
        if(1== database_gz[i].state_fenpei_gz)
        {

            if(0== database_gz[database_cw.dIndx].lock)
            {
                if(1== database_gz[database_cw.dIndx].dzx_mode_gz)
                {
                    j++;
                }
                else if(2== database_gz[database_cw.dIndx].dzx_mode_gz)
                {
                    k++;
                }
                else if(3== database_gz[database_cw.dIndx].dzx_mode_gz)
                {
                    l++; 
                }

            }


            //if(database_gz[database_cw.dIndx].cunwu_mode_gz ==1)
            // Del_FR(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
            nvs_wr_adm_zwpageid_flag(1,database_gz[database_cw.dIndx].zhiwen_page_id_gz);

            database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
            //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
            database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
            database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
            database_gz[database_cw.dIndx].state_gz =0;
            database_gz[database_cw.dIndx].changqi =0;
            database_gz[database_cw.dIndx].zhiwen_page_id_gz =0;
            nvs_wr_cunwu_mode_gz(1);
            //nvs_wr_dzx_mode_gz(1);
            nvs_wr_phone_number_nvs_gz(1);
            nvs_wr_mima_number_nvs_gz(1);
            nvs_wr_state_gz(1);
            nvs_wr_glongtime_gz(1);
            nvs_wr_zw_pageid_gz(1);




            m = i/24;
            n = i%24;

            if(i%24 ==0)
            {
                m = i/24 -1;
                n = 24;
            }
            send_cmd_to_lock(m+1, n);
            //send_cmd_to_lock(i/24 +1, i%24);
            //vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    }




    tongbu_changqi();

    shengyu_da = j;
    shengyu_zhong = k;
    shengyu_xiao = l;

    shengyu_all = shengyu_da + shengyu_zhong + shengyu_xiao;

    DB_PR("---shengyu_all=%d----\n",shengyu_all);

    DB_PR("---shengyu_da=%d----\n",shengyu_da);
    DB_PR("---shengyu_zhong=%d----\n",shengyu_zhong);
    DB_PR("---shengyu_xiao=%d----\n",shengyu_xiao);

    tongbu_gekou_shuliang_all(shengyu_all);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_d(shengyu_da);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_z(shengyu_zhong);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_x(shengyu_xiao);
    //vTaskDelay(10 / portTICK_PERIOD_MS);


    nvs_wr_shengyu_da(1);
    nvs_wr_shengyu_zhong(1);
    nvs_wr_shengyu_xiao(1);

    send_cmd_to_lcd_pic(CLEAR_ALL_OK_PIC);

    vTaskDelay(1);
    
    vTaskDelete(NULL);
}

static void echo_task0()//zhiwen
{
    DB_PR("--echo_task0-- \n");


    // vTaskDelay(1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}

static void echo_task3()//zhiwen
{
    DB_PR("--echo_task3-- \n");


    // vTaskDelay(1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}

static void echo_task()
{
    
    // Configure a temporary buffer for the incoming data
    //uint8_t *data_rx = (uint8_t *) malloc(BUF_SIZE);

    //DB_PR("UART1 start recieve loop.\r\n");

    while (1) {
        //vTaskDelay(10 / portTICK_PERIOD_MS);
        // Read data from the UART
        len_rx0 = uart_read_bytes(UART_NUM_0, data_rx0, BUF_SIZE, 20 / portTICK_RATE_MS);
        len_rx = uart_read_bytes(UART_NUM_1, data_rx, BUF_SIZE, 20 / portTICK_RATE_MS);
        len_rx2_m = uart_read_bytes(UART_NUM_2, data_rx2_m, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        
        //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);//len =0 budayin
        //uart_write_bytes(UART_NUM_2, (const char *) data_rx, len_rx);


        if ((len_rx2_m > 0) ) {
            
            // len_rx2_m = len_rx2_m;
            // memcpy(data_rx2_m,data_rx2_m,len_rx2_m);
            DB_PR("2rcv_zhiwen_uart2-Received %u bytes:", len_rx2_m);//zhiwen
            for (int i = 0; i < len_rx2_m; i++) {
                DB_PR("0x%.2X ", (uint8_t)data_rx2_m[i]);
            }
            DB_PR("] \n");
            
            flag_rx2 =1;
            DB_PR("-----2----flag_rx2=%u\r\n", flag_rx2);
            
            // if(taskhandle_uart2!=NULL)
            // {
            //     DB_PR("--uart2--111111-----.\r\n");
            //     vTaskDelete(taskhandle_uart2);
            //     taskhandle_uart2 =NULL;
            // }
            // else
            // {
            //     DB_PR("--uart2--222222 =NULL-----.\r\n");
            // }
            
            // xTaskCreate(echo_task3, "uart_echo_task3",2* 1024, NULL, 2, &taskhandle_uart2);//uart2
            //uart_write_bytes(UART_NUM_2, (const char *) data_rx2_m, len_rx2_m);
        }

								
		


        if (len_rx > 0) {

            DB_PR("1rcv_lcd_uart1-Received %u bytes:", len_rx);//lcd
            for (int i = 0; i < len_rx; i++) {
                DB_PR("0x%.2X ", (uint8_t)data_rx[i]);
                // if(spear_uart_process_data(data_rx[i]))
                // {
                //     ////send event to 
                //     //spear_sched_set_evt(NULL, 0,user_cmd_process_event_handle);

                //     DB_PR("处理数据2-start\r\n");
                //     xTaskCreate(echo_task2, "uart_echo_task2",4* 1024, NULL, 2, NULL);
                // }
            }
            DB_PR("] \n");

            //vTaskDelay(2 / portTICK_PERIOD_MS);


            xTaskCreate(echo_task2, "uart_echo_task2",6* 1024, NULL, 2, NULL);//uart1

        }
	

        if (len_rx0 > 0) {

            DB_PR("0rcv_lcd_uart1-Received %u bytes:", len_rx0);//485  DTU
            for (int i = 0; i < len_rx0; i++) {
                DB_PR("0x%.2X ", (uint8_t)data_rx0[i]);
            }
            DB_PR("] \n");

            //vTaskDelay(2 / portTICK_PERIOD_MS);
            xTaskCreate(echo_task0, "uart_echo_task0",2* 1024, NULL, 2, NULL);//uart1
            // uart_write_bytes(UART_NUM_0, (const char *) data_rx0, len_rx0);//debug---------

        }



    }
    //vTaskDelay(1);
    
    vTaskDelete(NULL);
}








//显示确认码错误信息
void ShowErrMessage(u8 ensure)
{
		DB_PR("err=%s\r\n",(u8*)EnsureMessage(ensure));
}
// //录指纹
// void Add_FR(void)
// {
// 	u8 i=0,ensure ,processnum=0;
// 	//u16 ID;
// 	while(1)
// 	{
//         //vTaskDelay(100 / portTICK_PERIOD_MS);
// 		switch (processnum)
// 		{
// 			case 0:
// 				i++;
// 				//LCD_Fill(0,100,lcddev.width,160,WHITE);
// 				//DB_PR("请按指纹");
//                 DB_PR("---0--an-请按指纹");
// 				ensure=PS_GetImage();
// 				if(ensure==0x00) 
// 				{
// 					//BEEP=1;------------------------
// 					ensure=PS_GenChar(CharBuffer1);//生成特征
// 					//BEEP=0;
// 					if(ensure==0x00)
// 					{
// 						//LCD_Fill(0,120,lcddev.width,160,WHITE);
// 						DB_PR("--0-ok--指纹正常");
// 						i=0;
// 						processnum=1;//跳到第二步						
// 					}
//                     else 
//                     {
//                         DB_PR("-0-no-1-ensure=%d",ensure);
//                         ShowErrMessage(ensure);			
//                     }
                        	
// 				}
//                 else 
//                 {
//                     DB_PR("-0-no-2-ensure=%d",ensure);
//                     ShowErrMessage(ensure);	
//                 }
                    					
// 				break;
			
// 			case 1:
// 				i++;
// 				////LCD_Fill(0,100,lcddev.width,160,WHITE);
// 				DB_PR("---1-zaian--请按再按一次指纹");
// 				ensure=PS_GetImage();
// 				if(ensure==0x00) 
// 				{
// 					//BEEP=1;
// 					ensure=PS_GenChar(CharBuffer2);//生成特征
// 					//BEEP=0;
// 					if(ensure==0x00)
// 					{
// 						////LCD_Fill(0,120,lcddev.width,160,WHITE);
// 						DB_PR("--1-ok--指纹正常 ");
// 						i=0;
// 						processnum=2;//跳到第三步
// 					}else ShowErrMessage(ensure);	
// 				}else ShowErrMessage(ensure);		
// 				break;

// 			case 2:
// 				////LCD_Fill(0,100,lcddev.width,160,WHITE);
// 				DB_PR("--2-duibi--对比两次指纹 ");
// 				//ensure=PS_Match();
//                 SearchResult *p_rsp=NULL;
//                 //ensure = PS_Search(0x02, 0x0000, ZHIWEN_PAGE_ID_MAX, p_rsp);
//                 ensure =0;
//                 //DB_PR("--2-pageID=%d, mathscore=%d",p_rsp->pageID,p_rsp->mathscore);
// 				if(ensure==0x00) 
// 				{
// 					//LCD_Fill(0,120,lcddev.width,160,WHITE);
// 					DB_PR("--2-ok对比成功,两次指纹一样 ");
// 					processnum=3;//跳到第四步
// 				}
// 				else 
// 				{
// 					//LCD_Fill(0,100,lcddev.width,160,WHITE);
// 					DB_PR("--2-no对比失败，请重新录入指纹 ");
// 					ShowErrMessage(ensure);
// 					i=0;
// 					processnum=0;//跳回第一步		
// 				}
// 				delay_ms(1200);
// 				break;

// 			case 3:
// 				//LCD_Fill(0,100,lcddev.width,160,WHITE);
// 				DB_PR("----3 shengcheng-----生成指纹模板 ");
// 				ensure=PS_RegModel();
// 				if(ensure==0x00) 
// 				{
// 					//LCD_Fill(0,120,lcddev.width,160,WHITE);
// 					DB_PR("--3-ok生成指纹模板成功 ");
// 					processnum=4;//跳到第五步
// 				}else {processnum=0;ShowErrMessage(ensure);}
// 				delay_ms(1200);
// 				break;
				
// 			case 4:	
// 				//LCD_Fill(0,100,lcddev.width,160,WHITE);
// 				DB_PR("----4 input id-----请输入储存ID,按Enter保存 ");
// 				DB_PR("0=< ID <=299 ");// 0 - 100---------------------

                
// 				// do
// 				// 	ID=GET_NUM();
// 				while(!(database_cw.zhiwen_page_id<AS608Para.PS_max));//输入ID必须小于最大存储数值



// 				ensure=PS_StoreChar(CharBuffer2,database_cw.zhiwen_page_id);//储存模板
// 				if(ensure==0x00) 
// 				{			
// 					//LCD_Fill(0,100,lcddev.width,160,WHITE);					
// 					DB_PR("--4-ok录入指纹成功 ");
// 					PS_ValidTempleteNum(&ValidN);//读库指纹个数
// 					DB_PR("zhiwen number=%d ",AS608Para.PS_max-ValidN);
// 					delay_ms(1500);
// 					//LCD_Fill(0,100,240,160,WHITE);
// 					return ;
// 				}else {processnum=0;ShowErrMessage(ensure);}					
// 				break;				
// 		}
// 		delay_ms(400);
// 		if(i==5)//超过5次没有按手指则退出
// 		{
//             DB_PR("---->5 input id-----请输入储存ID,按Enter保存 ");
// 			//LCD_Fill(0,100,lcddev.width,160,WHITE);
// 			break;	
// 		}				
// 	}
// }


//第三次
bool processnum_first_ok=0;
//录指纹
void Add_FR_First()
{
    return_cause_zw=0;
    DB_PR("---HandShakeFlag=%d\r\n",HandShakeFlag);
    if(HandShakeFlag ==1)
    {
        return_cause_zw_handshake_fail =1;
        send_cmd_to_lcd_pic(0x004D);
        DB_PR("---zhiwen connect fail\r\n");
        vTaskDelete(NULL);
    }
	u8 i=0,ensure=0 ,processnum=0;
    // u8 ensure_2=0,ensure_3=0;
    SearchResult p_rsp;
	//u16 ID;
    delay_ms(200);
	while(1)
	{
        vTaskDelay(10 / portTICK_PERIOD_MS);
		switch (processnum)
		{
			case 0:
				i++;
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//DB_PR("请按指纹");
                DB_PR("---0--an-请按指纹");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//BEEP=1;------------------------
                    delay_ms(200);//这里需要延时一下，模块内部处理图像需要时间  up
					ensure=PS_GenChar(CharBuffer1);//生成特征
					//BEEP=0;
					if(ensure==0x00)
					{
                        delay_ms(120);
                        DB_PR("--0-ok1--指纹正常\r\n");
                        ensure = PS_Search(CharBuffer1, 0x0000,AS608Para.PS_max , &p_rsp);//0x02 0x00AA ZHIWEN_PAGE_ID_MAX
                        //delay_ms(50);
                        DB_PR("--0--ensure=%d\r\n",ensure);
                        if(ensure==0x00)
                        {
                            DB_PR("-------p_rsp->pageID=%02x----\r\n",p_rsp.pageID);
                            //LCD_Fill(0,120,lcddev.width,160,WHITE);
                            DB_PR("--0-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            i=0;
	                        processnum=0;//跳回第一步	
                            DB_PR("--0-no-3-ensure=%d\r\n",ensure);
                            ShowErrMessage(ensure);	

                            vTaskDelete(NULL);
                        }
                        else 
                        {
                            
                            DB_PR("--0-ok2-对比成功,新的指纹\r\n");
                            i=0;//del?
                            //processnum=1;//跳到第二步	

                            processnum=0;	
                            //processnum_first_ok =1;	
                            send_cmd_to_lcd_pic(0x004E);
                            delay_ms(200);

                            DB_PR("database_cw_adm.changqi_tmp= %d \r\n",database_cw_adm.changqi_tmp);
                            if(database_cw_adm.changqi_tmp == 0)
                            {
                                xTaskCreate(Add_FR, "add_zhiwen_task2", 4* 1024, NULL, 2, NULL);//1024 10
                            }
                            else
                            {
                                xTaskCreate(Add_FR_CQ, "add_zhiwen_task2", 4* 1024, NULL, 2, NULL);//1024 10
                            }
                            xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_CL, 10, NULL);
                            
                            vTaskDelete(NULL);


                        }
                        delay_ms(200);
				
					}
                    else 
                    {
                        DB_PR("-0-no-2-ensure=%d\r\n",ensure);
                        ShowErrMessage(ensure);			
                    }
                        	
				}
                else 
                {
                    DB_PR("-0-no-1-ensure=%d\r\n",ensure);
                    ShowErrMessage(ensure);	//no shouzhi
                    //send_cmd_to_lcd_pic(0x004C);
                }
                    					
				break;
			


            default:
                DB_PR("--default-zhiwen add1 \r\n");
                break;		
		}
		delay_ms(400);
        DB_PR("-1-hearbeat...return_cause_zw=%d \r\n",return_cause_zw);
        if(return_cause_zw ==1)
        {
            DB_PR("--zw task 1 over \r\n");
            return_cause_zw =0;
            vTaskDelete(NULL);
        }

		if(i==40)//超过5次没有按手指则退出
		{
            if(database_cw_adm.changqi_tmp == 0)
            {
                return_cause_zw_fail =1;
            }
            else
            {
                return_cause_zw_fail =2;
            }
            DB_PR("-1-return_cause_zw_fail=%d---.\r\n",return_cause_zw_fail);   
            
            DB_PR("---->5a---- 超过5次没有按手指则退出 ");
			//LCD_Fill(0,100,lcddev.width,160,WHITE);
            send_cmd_to_lcd_pic(0x004C);
            
			break;	
		}				
	}
    return_cause_zw =0;
    vTaskDelay(1);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);

}



//录指纹  2
void Add_FR()
{

    
    // audio_pipeline_stop(pipeline);
    // audio_pipeline_wait_for_stop(pipeline);
    // audio_pipeline_terminate(pipeline);
    // audio_pipeline_reset_ringbuffer(pipeline);
    // audio_pipeline_reset_elements(pipeline);

    // DB_PR("[2.6-d-zw] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)\r\n");
    // audio_element_set_uri(tone_stream_reader, tone_uri[TONE_TYPE_CL]);
    // audio_pipeline_run(pipeline);



    //return_cause_zw=0;
//     u8 zw_likai_flag=0;
//     for(uint16_t i=0;i<50;i++)//
// 	{
//         DB_PR("---------gpio_get_level(GPIO_INPUT_IO_ZW_JC)=%d---------\r\n",gpio_get_level(GPIO_INPUT_IO_ZW_2));
//         if((gpio_get_level(GPIO_INPUT_IO_ZW_2)==0) &&(i==2))//
//         {
//             DB_PR("------------finger move ok----------\r\n");
//             //io_shouzhi_down_flag =0;
//             zw_likai_flag =1;
//             break;
//         }
//         delay_ms(200);
//         DB_PR("------------wait for finger move----------\r\n");

//     }
//     if(zw_likai_flag == 0)
//     {
//         vTaskDelay(1);
//         DB_PR("----a----shouzhi move timeout--------\r\n");
//         vTaskDelete(NULL);
//     }
//     else
//     {
//         DB_PR("-----b------shouzhi move ok, continue-----------\r\n");
//         /* code */
//     }
    
    
    // if(processnum_first_ok ==1)
    // {
    //     send_cmd_to_lcd_pic(0x0005);
    //     DB_PR("---zhiwen connect fail\r\n");
    //     return;
    // }
	u8 i=0,ensure=0 ,processnum=1;
    // u8 ensure_2=0,ensure_3=0;
    SearchResult p_rsp;
	//u16 ID;
    //delay_ms(1000);
	while(1)
	{
        vTaskDelay(10 / portTICK_PERIOD_MS);

        DB_PR("----1---i=%d----\r\n",i);

		switch (processnum)
		{

			case 1:
				i++;
				////LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("---1-zaian--请按再按一次指纹\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//BEEP=1;
                    delay_ms(200);//这里需要延时一下，模块内部处理图像需要时间
					ensure=PS_GenChar(CharBuffer2);//生成特征
					//BEEP=0;
					if(ensure==0x00)
					{
                        delay_ms(120);
                        DB_PR("--1-ok1--指纹正常 \r\n");
                        ensure = PS_Search(CharBuffer2, 0x0000, AS608Para.PS_max, &p_rsp);//0x02ZHIWEN_PAGE_ID_MAX
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--1-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=1;//跳回第一步	
                            vTaskDelete(NULL);
                        }
                        else 
                        {
                            DB_PR("--1-ok2-对比成功,新的指纹\r\n");
                            i=0;
                            processnum=2;//跳到第三步	

                        }
                        delay_ms(200);

					}
                    else 
                    {
                        ShowErrMessage(ensure);	
                    }   
				}
                else 
                {
                    ShowErrMessage(ensure);		
                    //send_cmd_to_lcd_pic(0x004C);
                }
                    
				break;


            case 2:
				i++;
				////LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("---2-zaian--请按再按一次指纹\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//BEEP=1;
                    delay_ms(200);//这里需要延时一下，模块内部处理图像需要时间
					ensure=PS_GenChar(CharBuffer3);//生成特征
					//BEEP=0;
					if(ensure==0x00)
					{
                        delay_ms(120);
                        DB_PR("--2-ok--指纹正常 \r\n");
                        ensure = PS_Search(CharBuffer3, 0x0000, AS608Para.PS_max, &p_rsp);//0x02ZHIWEN_PAGE_ID_MAX
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--2-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=1;//跳回第一步	

                            vTaskDelete(NULL);
                        }
                        else 
                        {
                            DB_PR("--2-ok-对比成功,新的指纹\r\n");
                            i=0;
                            processnum=3;//跳到第三步			

                        }
                        delay_ms(200);

					}
                    else 
                    {
                        ShowErrMessage(ensure);	
                    }   
				}
                else 
                {
                    ShowErrMessage(ensure);		
                    //send_cmd_to_lcd_pic(0x004C);
                }
                    
				break;


			case 3:
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("----3 shengcheng-----生成指纹模板 \r\n");
				ensure=PS_RegModel();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					DB_PR("--3-ok生成指纹模板成功 \r\n");
					processnum=4;//跳到第五步
				}
                else 
                {
                    processnum=1;
                    ShowErrMessage(ensure);
                    vTaskDelete(NULL);
                }
				delay_ms(200);
				break;
				
			case 4:	
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("----4 input id-----请输入储存ID,按Enter保存 \r\n");
				DB_PR("0=< ID <=299 \r\n");// 0 - 100---------------------

                uint16_t page_id_temp1[ZHIWEN_PAGE_ID_MAX]={0};
                uint16_t page_id_temp2[ZHIWEN_PAGE_ID_MAX]={0};
                int16_t guimen_gk_temp =0;


                DB_PR("AS608Para.PS_max=%d, ValidN =%d \r\n",AS608Para.PS_max, ValidN);
                DB_PR("库容量:%d     对比等级: %d\r\n",AS608Para.PS_max-ValidN,AS608Para.PS_level);

                if(AS608Para.PS_max == 0)
                {
                    send_cmd_to_lcd_pic(0x004C);
                    DB_PR("--fail--PS_max ==0 \r\n");
                    vTaskDelete(NULL);
                }
                else
                {
                    DB_PR("--ok--PS_max !=0 \r\n");
                }
                


                //AS608Para.PS_max-ValidN
                uint16_t j =0, k=0;
                int rand_temp=0;
                for(uint16_t i=1;i<AS608Para.PS_max;i++)//
                {
                    if(database_ad.zhiwen_page_id_adm[i] ==0)//weiyong
                    {
                        //DB_PR("1-i=%d, ",i);
                        page_id_temp1[j++] = i;
                    }
                    else//yi yong
                    {
                        //DB_PR("2-i=%d, ",i);
                        page_id_temp2[k++] = i;
                    }
                    //DB_PR("---i=%d, ",i);
                }
                DB_PR("---pageid--shengyu j=%d, onuse k=%d\r\n",j,k);
                DB_PR("--ok--no use  zhiwen idx\r\n");
                uart0_debug_data_dec(page_id_temp1,j);
                DB_PR("--fail--has use zhiwen idx\r\n");
                uart0_debug_data_dec(page_id_temp2,k);
                
                
                if(j>0)
                {
                    srand((unsigned int) time(NULL));
                    rand_temp = rand();
                    DB_PR("rand_temp=%d\r\n",rand_temp);
                    database_cw.zhiwen_page_id = page_id_temp1[rand_temp%j];//随机获取哪个门没用
                    

                    // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                    //DB_PR("-------add---database_cw.zhiwen_page_id=%d\r\n",database_cw.zhiwen_page_id);
                    DB_PR("******add******database_cw.zhiwen_page_id=%d\r\n",database_cw.zhiwen_page_id);

                    // do
                    // 	ID=GET_NUM();
                    //while(!(database_cw.zhiwen_page_id<AS608Para.PS_max));//输入ID必须小于最大存储数值


                    //database_cw.zhiwen_page_id = 0x0c;//todo
                    //ensure=PS_StoreChar(CharBuffer3,database_cw.zhiwen_page_id);//储存模板
                    ensure=PS_StoreChar(CharBuffer1,database_cw.zhiwen_page_id);//储存模板
                    DB_PR("----4------ensure=%d",ensure);
                    if(ensure==0x00) 
                    {			
                        //LCD_Fill(0,100,lcddev.width,160,WHITE);					
                        DB_PR("--4-ok录入指纹成功 ");
                        // PS_ReadSysPara(&AS608Para);  //读参数 
                        PS_ValidTempleteNum(&ValidN);//读库指纹个数
                        DB_PR("AS608Para.PS_max=%d, ValidN =%d ",AS608Para.PS_max, ValidN);
                        DB_PR("zhiwen shengyu number=%d ",AS608Para.PS_max-ValidN);
                        delay_ms(150);//1500









                        uint16_t j=0,k=0;
                        uint16_t database_gz_temp[SHENYU_GEZI_MAX]={0};
                        uint16_t database_gz_temp_onuse[SHENYU_GEZI_MAX]={0};

                        DB_PR("-----debug------database_cw.dzx_mode=%d\r\n",database_cw.dzx_mode);//j=0  
// start ----------------------------------------
                        if(1 == database_cw.dzx_mode)
                        {
                            
                            for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                            {
                                if((database_gz[i].state_gz ==0) //no use
                                    &&(database_gz[i].state_fenpei_gz == 1)
                                    &&(database_gz[i].lock == 0)
                                    &&(database_gz[i].changqi == 0)
                                    &&(database_gz[i].cunwu_mode_gz==0)
                                    &&(database_gz[i].dzx_mode_gz ==1))//d
                                {
                                    database_gz_temp[j++] =i;
                                }
                                else if((database_gz[i].state_fenpei_gz == 1)
                                    &&(database_gz[i].dzx_mode_gz ==1) )//d
                                {
                                    database_gz_temp_onuse[k++] =i;
                                }
                                // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                            }
                            DB_PR("1--shengyu j=%d, onuse k=%d\r\n",j,k);//j=0  
                            uart0_debug_data_dec(database_gz_temp,j);
                            uart0_debug_data_dec(database_gz_temp_onuse,k);//no

                            if(j>0)
                            {
                                srand((unsigned int) time(NULL));
                                database_cw.dIndx = database_gz_temp[rand() %j];//随机获取哪个门没用

                                database_cw.state=1;
                                // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);

                                DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);



                                guimen_gk_temp = database_cw.dIndx ;
                                uint16_t j=0,k=0;

                                k = guimen_gk_temp/24;
                                j = guimen_gk_temp%24;

                                if(guimen_gk_temp%24 ==0)
                                {
                                    k = guimen_gk_temp/24 -1;
                                    j = 24;
                                }

                                DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);




                                // if(shengyu_da>0)
                                {
                                    //old
                                    shengyu_da --;
                                    tongbu_gekou_shuliang_d(shengyu_da);


                                    DB_PR("-da-lock:%d ok--.\r\n",j);

                                    send_cmd_to_lock(k+1,j);
                                    send_cmd_to_lcd_bl(0x1070,database_gz[database_cw.dIndx].dIndx_gz);//-----------
                                }
                            
                            }    
                            else
                            {
                                DB_PR("---1----state d have no-----.\r\n");
                                goto done_zwc_fail;
                            }
                            

                        }
                        else if(2 == database_cw.dzx_mode)
                        {
                            j=0;
                            for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                            {
                                if((database_gz[i].state_gz ==0) //no use
                                    &&(database_gz[i].state_fenpei_gz == 1)
                                    &&(database_gz[i].lock == 0)
                                    &&(database_gz[i].changqi == 0)
                                    &&(database_gz[i].cunwu_mode_gz==0)
                                    &&(database_gz[i].dzx_mode_gz ==2))//z
                                {
                                    database_gz_temp[j++] =i;
                                }
                                else if((database_gz[i].state_fenpei_gz == 1)
                                    &&(database_gz[i].dzx_mode_gz ==2))//z
                                {
                                    database_gz_temp_onuse[k++] =i;
                                }
                                // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                            }
                            DB_PR("2-shengyu j=%d, onuse k=%d\r\n",j,k);
                            uart0_debug_data_dec(database_gz_temp,j);
                            uart0_debug_data_dec(database_gz_temp_onuse,k);

                            if(j>0)
                            {
                                srand((unsigned int) time(NULL));
                                database_cw.dIndx = database_gz_temp[rand() %j];//随机获取哪个门没用

                                database_cw.state=1;
                                // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                                DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);



                                guimen_gk_temp = database_cw.dIndx ;
                                uint16_t j=0,k=0;

                                k = guimen_gk_temp/24;
                                j = guimen_gk_temp%24;

                                if(guimen_gk_temp%24 ==0)
                                {
                                    k = guimen_gk_temp/24 -1;
                                    j = 24;
                                }

                                DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                                //if(shengyu_zhong >0)
                                {
                                    shengyu_zhong --;
                                    tongbu_gekou_shuliang_z(shengyu_zhong);

                                    //DB_PR("--lock2 ok--.\r\n");

                                    DB_PR("-zhong-lock:%d ok--.\r\n",j);
                                    send_cmd_to_lock(k+1,j);
                                    send_cmd_to_lcd_bl(0x1070,database_gz[database_cw.dIndx].dIndx_gz);
                                    
                                }

                            }
                            else
                            {
                                DB_PR("---2----state z have no-----.\r\n");
                                goto done_zwc_fail;
                            }
                            

                        }
                        else if(3 == database_cw.dzx_mode)
                        {

                            for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                            {
                                if((database_gz[i].state_gz ==0) //no use
                                    &&(database_gz[i].state_fenpei_gz == 1)
                                    &&(database_gz[i].lock == 0)
                                    &&(database_gz[i].changqi == 0)
                                    &&(database_gz[i].cunwu_mode_gz==0)
                                    &&(database_gz[i].dzx_mode_gz ==3))//x
                                {
                                    database_gz_temp[j++] =i;
                                }
                                else if((database_gz[i].state_fenpei_gz == 1)
                                    &&(database_gz[i].dzx_mode_gz ==3))//x
                                {
                                    database_gz_temp_onuse[k++] =i;
                                }
                                // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                                //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                            }
                            DB_PR("3-shengyu j=%d, onuse k=%d\r\n",j,k);
                            uart0_debug_data_dec(database_gz_temp,j);
                            uart0_debug_data_dec(database_gz_temp_onuse,k);

                            if(j>0)
                            {
                                srand((unsigned int) time(NULL));
                                database_cw.dIndx = database_gz_temp[rand() %j];//随机获取哪个门没用

                                database_cw.state=1;
                                // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                                DB_PR("--add---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                                DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);

                                //database_cw.dIndx = 43;//-------14--------


                                guimen_gk_temp = database_cw.dIndx ;
                                uint16_t j=0,k=0;

                                k = guimen_gk_temp/24;
                                j = guimen_gk_temp%24;

                                if(guimen_gk_temp%24 ==0)
                                {
                                    k = guimen_gk_temp/24 -1;
                                    j = 24;
                                }

                                DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);


                                
                                //if(shengyu_xiao >0)
                                {
                                    shengyu_xiao --;
                                    tongbu_gekou_shuliang_x(shengyu_xiao);
                                    //DB_PR("----------------lock3 ts---------------.\r\n");
                                    
                                    DB_PR("-xiao-lock:%d ok--.\r\n",j);

                                    send_cmd_to_lock(k+1,j);
                                    send_cmd_to_lcd_bl(0x1070,database_gz[database_cw.dIndx].dIndx_gz);

                                }



                            }
                            else
                            {
                                DB_PR("----3---state x have no-----.\r\n");
                                goto done_zwc_fail;
                            }
                                
                        }

                        DB_PR("-----2-----[ * ] Starting audio pipeline");
                        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);

                        DB_PR("--11111111111--test--.\r\n");  
                        if(0 != database_cw.dzx_mode)
                        {
                            shengyu_all -- ;
                            tongbu_gekou_shuliang_all(shengyu_all);



                            //guizi     ------------save-------------------
                            database_gz[database_cw.dIndx].cunwu_mode_gz = database_cw.cunwu_mode;
                            nvs_wr_cunwu_mode_gz(1);


                            // database_gz[database_cw.dIndx].dzx_mode_gz = database_cw.dzx_mode;
                            // nvs_wr_dzx_mode_gz(1);


                            DB_PR("******add******database_cw.zhiwen_page_id=%d\r\n",database_cw.zhiwen_page_id);
                            database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id] =1;
                            nvs_wr_adm_zwpageid_flag(1,database_cw.zhiwen_page_id);
                            database_gz[database_cw.dIndx].zhiwen_page_id_gz = database_cw.zhiwen_page_id;
                            nvs_wr_zw_pageid_gz(1);

                            // //database_cw.phone_number_nvs = atoll((const char*)phone_number);
                            // database_gz[database_cw.dIndx].phone_number_nvs_gz = database_cw.phone_number_nvs;
                            // nvs_wr_phone_number_nvs_gz(1);


                            // //database_cw.mima_number_nvs = atoi((const char*)mima_number);
                            // database_gz[database_cw.dIndx].mima_number_nvs_gz = database_cw.mima_number_nvs;
                            // nvs_wr_mima_number_nvs_gz(1);


                            database_gz[database_cw.dIndx].state_gz =database_cw.state;
                            nvs_wr_state_gz(1);


                            //admin   ------------save-------------------
                            nvs_wr_shengyu_da(1);
                            nvs_wr_shengyu_zhong(1);
                            nvs_wr_shengyu_xiao(1);




                            //custumer   mingming kongjian   todo
                            // esp_err_t err = save_u16_value(STORAGE_NAMESPACE,"dw_dIndx",database_cw.dIndx);
                            // if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                            // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
                            // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                            //unique number

                            send_cmd_to_lcd_pic(0x0008);


                        }
                        // uart0_debug_data_dec(database_gz_temp,j);
                        // uart0_debug_data_dec(database_gz_temp_onuse,k);
                        //cun
                        
                        


                        //log debug
                        j=0;k=0;
                        for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//todo changqi and suoding
                        {
                            if((database_gz[i].state_gz ==0) //no use
                                &&(database_gz[i].state_fenpei_gz == 1)
                                &&(database_gz[i].lock == 0)
                                &&(database_gz[i].changqi == 0)
                                &&(database_gz[i].cunwu_mode_gz==database_cw.cunwu_mode)
                                &&(database_gz[i].dzx_mode_gz ==database_cw.dzx_mode))//d
                            {
                                database_gz_temp[j++] =i;
                            }
                            else if((database_gz[i].state_fenpei_gz == 1)
                                &&(database_gz[i].dzx_mode_gz ==database_cw.dzx_mode))//d
                            {
                                database_gz_temp_onuse[k++] =i;
                            }
                            // DB_PR("shengyu index=%03d, database_gz[i].dzx_mode_gz=%d, state =%d\r\n",
                            //         i, database_gz[i].dzx_mode_gz, database_gz[i].state_gz);
                        }
                        DB_PR("-------------shengyu j=%d, onuse k=%d\r\n",j,k);
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                        uart0_debug_data_dec(database_gz_temp,j);
                        uart0_debug_data_dec(database_gz_temp_onuse,k);//no
                        DB_PR("---lock idx---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                        DB_PR("---xiangmen---guimen_gk_temp=%u\r\n",guimen_gk_temp);

                        database_cw.cunwu_mode =0;

                        database_cw.state=0;

                        database_cw.dzx_mode = 0 ;//add

                        //send_cmd_to_lcd_pic(0x0008);
                        //LCD_Fill(0,100,240,160,WHITE);
                        //return ;
                        vTaskDelete(NULL);

                        
                    }
                    else 
                    {
done_zwc_fail:
                        processnum=1;
                        DB_PR("---done_zwc_fail----\r\n");
                        // database_cw.cunwu_mode =0;
                        // // database_cw.dzx_mode = 0 ;
                        // database_cw.state=0;
                        send_cmd_to_lcd_pic(0x0001);
                        return_cause_zanwu_kx =8;
                        ShowErrMessage(ensure);
                        vTaskDelete(NULL);
                    }		

                }			
				break;	

            default:
                DB_PR("--default-zhiwen add2 \r\n");
                break;		
		}
		delay_ms(400);
        DB_PR("-2-hearbeat...return_cause_zw=%d \r\n",return_cause_zw);
        if(return_cause_zw ==1)
        {
            DB_PR("--zw task 2 over \r\n");
            return_cause_zw =0;
            vTaskDelete(NULL);
        }
		if(i==40)//超过5次没有按手指则退出
		{
            return_cause_zw_fail =1;
            DB_PR("---->5a---- 超过5次没有按手指则退出 \r\n");
            send_cmd_to_lcd_pic(0x004C);
			//LCD_Fill(0,100,lcddev.width,160,WHITE);
			break;	
		}				
	}
    vTaskDelay(1);
    return_cause_zw =0;
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);

}


//录指纹
void Add_FR_CQ()
{
    //return_cause_zw =0;
    // if(HandShakeFlag ==1)
    // {
    //     send_cmd_to_lcd_pic(0x004D);
    //     DB_PR("---zhiwen connect fail\r\n");
    //     vTaskDelete(NULL);
    // }
	u8 i=0,ensure=0 ,processnum=1;
    // u8 ensure_2=0,ensure_3=0;
    SearchResult p_rsp;
	//u16 ID;
	while(1)
	{
        vTaskDelay(10 / portTICK_PERIOD_MS);
		switch (processnum)
		{

			case 1:
				i++;
				////LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("---1-zaian--请按再按一次指纹\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//BEEP=1;
                    delay_ms(200);//这里需要延时一下，模块内部处理图像需要时间
					ensure=PS_GenChar(CharBuffer2);//生成特征
					//BEEP=0;
					if(ensure==0x00)
					{
                        delay_ms(120);
                        DB_PR("--1-ok1--指纹正常 \r\n");
                        ensure = PS_Search(CharBuffer2, 0x0000, AS608Para.PS_max, &p_rsp);//0x02ZHIWEN_PAGE_ID_MAX
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--1-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=1;//跳回第一步	
                            vTaskDelete(NULL);
                        }
                        else 
                        {
                            DB_PR("--1-ok2-对比成功,新的指纹\r\n");
                            i=0;
                            processnum=2;//跳到第三步			

                        }
                        delay_ms(200);

					}
                    else 
                    {
                        ShowErrMessage(ensure);	
                    }   
				}
                else 
                {
                    ShowErrMessage(ensure);		
                }
                    
				break;

            case 2:
				i++;
				////LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("---2-zaian--请按再按一次指纹\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					//BEEP=1;
                    delay_ms(200);//这里需要延时一下，模块内部处理图像需要时间
					ensure=PS_GenChar(CharBuffer3);//生成特征
					//BEEP=0;
					if(ensure==0x00)
					{
                        delay_ms(120);
                        DB_PR("--2-ok--指纹正常 \r\n");
                        ensure = PS_Search(CharBuffer3, 0x0000,AS608Para.PS_max , &p_rsp);//0x02ZHIWEN_PAGE_ID_MAX
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--2-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=1;//跳回第一步	
                            vTaskDelete(NULL);
                            	
                        }
                        else 
                        {
                            DB_PR("--2-ok-对比成功,新的指纹\r\n");
                            i=0;
                            processnum=3;//跳到第三步			

                        }
                        delay_ms(200);

					}
                    else 
                    {
                        ShowErrMessage(ensure);	
                    }   
				}
                else 
                {
                    ShowErrMessage(ensure);		
                }
                    
				break;


			case 3:
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("----3 shengcheng-----生成指纹模板 \r\n");
				ensure=PS_RegModel();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					DB_PR("--3-ok生成指纹模板成功 \r\n");
					processnum=4;//跳到第五步
				}else 
                {
                    processnum=1;
                    ShowErrMessage(ensure);
                    vTaskDelete(NULL);
                }
				delay_ms(200);
				break;
				
			case 4:	
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				DB_PR("----4 input id-----请输入储存ID,按Enter保存 \r\n");
				DB_PR("0=< ID <=299 \r\n");// 0 - 100---------------------

                uint16_t page_id_temp1[ZHIWEN_PAGE_ID_MAX]={0};
                uint16_t page_id_temp2[ZHIWEN_PAGE_ID_MAX]={0};
                int16_t guimen_gk_temp =0;


                DB_PR("AS608Para.PS_max=%d, ValidN =%d \r\n",AS608Para.PS_max, ValidN);
                DB_PR("库容量:%d     对比等级: %d\r\n",AS608Para.PS_max-ValidN,AS608Para.PS_level);

                if(AS608Para.PS_max == 0)
                {
                    send_cmd_to_lcd_pic(0x004C);
                    DB_PR("--fail--PS_max ==0 \r\n");
                    vTaskDelete(NULL);
                }
                else
                {
                    DB_PR("--ok--PS_max !=0 \r\n");
                }
                


                //AS608Para.PS_max-ValidN
                uint16_t j =0, k=0;
                int rand_temp=0;
                for(uint16_t i=1;i<AS608Para.PS_max;i++)//1 - 119
                {
                    if(database_ad.zhiwen_page_id_adm[i] ==0)//weiyong
                    {
                        //DB_PR("1-i=%d, ",i);
                        page_id_temp1[j++] = i;
                    }
                    else//yi yong
                    {
                        //DB_PR("2-i=%d, ",i);
                        page_id_temp2[k++] = i;
                    }
                    //DB_PR("---i=%d, ",i);
                }
                DB_PR("---pageid--shengyu j=%d, onuse k=%d\r\n",j,k);
                DB_PR("--ok--no use  zhiwen idx\r\n");
                uart0_debug_data_dec(page_id_temp1,j);
                DB_PR("--fail--has use zhiwen idx\r\n");
                uart0_debug_data_dec(page_id_temp2,k);
                
                
                if(j>0)
                {
                    srand((unsigned int) time(NULL));
                    rand_temp = rand();
                    DB_PR("rand_temp=%d\r\n",rand_temp);
                    database_cw_adm.zhiwen_page_id = page_id_temp1[rand_temp%j];//随机获取哪个门没用
                    

                    // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                    //DB_PR("-------add---database_cw.zhiwen_page_id=%d\r\n",database_cw.zhiwen_page_id);
                    DB_PR("******add******database_cw.zhiwen_page_id=%d\r\n",database_cw_adm.zhiwen_page_id);

                    // do
                    // 	ID=GET_NUM();
                    //while(!(database_cw.zhiwen_page_id<AS608Para.PS_max));//输入ID必须小于最大存储数值


                    //database_cw.zhiwen_page_id = 0x0c;//todo
                    //ensure=PS_StoreChar(CharBuffer3,database_cw.zhiwen_page_id);//储存模板
                    ensure=PS_StoreChar(CharBuffer1,database_cw_adm.zhiwen_page_id);//储存模板
                    DB_PR("----4------ensure=%d",ensure);
                    if(ensure==0x00) 
                    {			
                        //LCD_Fill(0,100,lcddev.width,160,WHITE);					
                        DB_PR("--4-ok录入指纹成功 ");
                        // PS_ReadSysPara(&AS608Para);  //读参数 
                        PS_ValidTempleteNum(&ValidN);//读库指纹个数
                        DB_PR("AS608Para.PS_max=%d, ValidN =%d ",AS608Para.PS_max, ValidN);
                        DB_PR("zhiwen shengyu number=%d ",AS608Para.PS_max-ValidN);
                        delay_ms(150);



                        uint16_t j=0,k=0;

                        DB_PR("-----debug------database_cw.dzx_mode=%d\r\n",database_cw_adm.dzx_mode);//j=0  
// start ----------------------------------------
                        




                        database_cw.dIndx = database_cw_adm.dIndx;//3703 nvs use

                        // database_cw.state=1;
                        // database_gz[database_cw.dIndx].state_gz =database_cw.state;
                        DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                        DB_PR("-------------add--xmh=%u\r\n",database_gz[database_cw_adm.dIndx].dIndx_gz);

                        //database_cw.dIndx = 43;//-------14--------


                        guimen_gk_temp = database_cw.dIndx ;

                        j=0,k=0;

                        k = guimen_gk_temp/24;
                        j = guimen_gk_temp%24;

                        if(guimen_gk_temp%24 ==0)
                        {
                            k = guimen_gk_temp/24 -1;
                            j = 24;
                        }
                        DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);



                        DB_PR("-da-lock:%d ok--.\r\n",j);
                        send_cmd_to_lock(k+1,j);
                        

                        send_cmd_to_lcd_bl(0x10e0,database_gz[database_cw_adm.dIndx].dIndx_gz);
                        send_cmd_to_lcd_pic(XIN_CHANGQI_OK_PIC);



                        
                        DB_PR("-----2-----[ * ] Starting audio pipeline");
                        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);

                        DB_PR( "database_gz[database_cw_adm.dIndx].state_gz = %d", database_gz[database_cw_adm.dIndx].state_gz);
                        if(0 == database_gz[database_cw_adm.dIndx].state_gz)
                        {
                            switch (database_gz[database_cw_adm.dIndx].dzx_mode_gz)
                            {
                            case 1:
                                //d
                                shengyu_da --;
                                tongbu_gekou_shuliang_d(shengyu_da); 
                                break;
                            case 2:
                                //z
                                shengyu_zhong --;
                                tongbu_gekou_shuliang_z(shengyu_zhong); 
                                break;
                            case 3:
                                //x
                                shengyu_xiao --;
                                tongbu_gekou_shuliang_x(shengyu_xiao); 
                                break;

                            default:
                                break;
                            }

                            DB_PR("----test--.\r\n");  
                            

                            shengyu_all -- ;
                            tongbu_gekou_shuliang_all(shengyu_all);


                            nvs_wr_shengyu_da(1);
                            nvs_wr_shengyu_zhong(1);
                            nvs_wr_shengyu_xiao(1);

                        }


                        //if(0 != database_gz[database_cw_adm.dIndx].state_gz)
                        {

                            // char key_name[15];//15
                            // esp_err_t err;

                            database_gz[database_cw_adm.dIndx].cunwu_mode_gz = database_cw_adm.cunwu_mode;
                            //database_gz[database_cw_adm.dIndx].dzx_mode_gz = 0;

                            //database_gz[database_cw_adm.dIndx].state_gz =0;

                            database_gz[database_cw_adm.dIndx].changqi =1;
                            //database_gz[database_cw_adm.dIndx].lock =0;
                            nvs_wr_cunwu_mode_gz(1);
                            //nvs_wr_dzx_mode_gz(1);

                            //nvs_wr_state_gz(1);
                            nvs_wr_glongtime_gz(1);
                            //nvs_wr_glock_gz(1);

                            //if(0 == database_gz[database_cw_adm.dIndx].state_gz)
                            {
                                DB_PR("******add******database_cw.zhiwen_page_id=%d\r\n",database_cw_adm.zhiwen_page_id);
                                database_gz[database_cw_adm.dIndx].zhiwen_page_id_gz = database_cw_adm.zhiwen_page_id;
                                nvs_wr_zw_pageid_gz(1);

                                database_ad.zhiwen_page_id_adm[database_cw_adm.zhiwen_page_id] =1;
                                nvs_wr_adm_zwpageid_flag(1,database_cw_adm.zhiwen_page_id);
                                //nvs_wr_adm_zwpageid_flag(1,database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                            }

                        }


                        //if(database_gz[database_cw_adm.dIndx].changqi == 0)
                        {
                            //update xianshi todo
                            tongbu_changqi();
                        }  
                            //custumer   mingming kongjian   todo
                            // esp_err_t err = save_u16_value(STORAGE_NAMESPACE,"dw_dIndx",database_cw.dIndx);
                            // if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                            // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
                            // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                            //unique number


                        DB_PR("----test3-done--.\r\n");  

                        // send_cmd_to_lcd_bl_len(0x1100,(uint8_t*)buff_t,30+5);//phone
                        // send_cmd_to_lcd_bl_len(0x1110,(uint8_t*)buff_t,30+5);//key
                        // // send_cmd_to_lcd_bl(0x1100,0);//phone
                        // // send_cmd_to_lcd_bl(0x1110,0);//key
                        
                        // database_cw.cunwu_mode =0;
                        // database_cw.dzx_mode = 0 ;






                        //send_cmd_to_lcd_pic(0x0008);
                        //LCD_Fill(0,100,240,160,WHITE);
                        //return ;
                        vTaskDelete(NULL);

                        
                    }
                    else 
                    {
                        DB_PR("---done_zwc_fail_cq1----\r\n");
                        goto done_zwc_fail_cq;

                    }		

                }
                else
                {
done_zwc_fail_cq:
                    processnum=1;
                    DB_PR("---done_zwc_fail_cq2----\r\n");
                    database_cw_adm.cunwu_mode =0;
                    database_cw_adm.dzx_mode = 0 ;
                    database_cw_adm.state=0;
                    send_cmd_to_lcd_pic(0x0001);
                    return_cause_zanwu_kx =9;
                    ShowErrMessage(ensure);
                    vTaskDelete(NULL);
                }
                			
				break;	

            default:
                DB_PR("--default-zhiwen add \r\n");
                break;		
		}
		delay_ms(400);
        DB_PR("-3-hearbeat...return_cause_zw=%d \r\n",return_cause_zw);
        if(return_cause_zw ==1)
        {
            DB_PR("--zw task 2 over \r\n");
            return_cause_zw =0;
            vTaskDelete(NULL);
        }

		if(i==40)//超过5次没有按手指则退出   22s
		{
            return_cause_zw_fail =2;
            send_cmd_to_lcd_pic(0x004C);//todo bl
            DB_PR("---->5a---- 超过5次没有按手指则退出 ");
			//LCD_Fill(0,100,lcddev.width,160,WHITE);
			break;	
		}				
	}
    vTaskDelay(1);
    return_cause_zw =0;
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);

}


//刷指纹
void press_FR(void)
{
    return_cause_zw =0;//add
    if(HandShakeFlag ==1)
    {
        return_cause_zw_handshake_fail =2;
        send_cmd_to_lcd_pic(0x004D);
        DB_PR("---zhiwen connect fail\r\n");
        vTaskDelete(NULL);
    }
	for(u8 i=0;i<30;i++)
	{
		vTaskDelay(1);
        SearchResult seach;
        u8 ensure;
        //char *str;
        delay_ms(500);
        ensure=PS_GetImage();
        delay_ms(200);
        DB_PR("-----11111111------\r\n");
        if(ensure==0x00)//获取图像成功 
        {	
            //BEEP=1;//打开蜂鸣器	---------------
            delay_ms(500);
            ensure=PS_GenChar(CharBuffer1);
            delay_ms(200);
            DB_PR("-----2222-----\r\n");
            if(ensure==0x00) //生成特征成功
            {		
                //BEEP=0;//关闭蜂鸣器	-------------
                //ensure=PS_HighSpeedSearch(CharBuffer1,0,AS608Para.PS_max,&seach);
                DB_PR("-----333--tezheng------\r\n");
                delay_ms(200);
                DB_PR("-----AS608Para.PS_max=%d\r\n",AS608Para.PS_max);
                ensure=PS_Search(CharBuffer1,0,0xAA,&seach);//AS608Para.PS_max
                delay_ms(200);
                if(ensure==0x00)//搜索成功
                {				
                    //LCD_Fill(0,100,lcddev.width,160,WHITE);
                    DB_PR("-----4444 ok-----刷指纹成功 \r\n");				

                    DB_PR("---------确有此人,ID:%d  匹配得分:%d \r\n",seach.pageID,seach.mathscore);
                    database_cw.dIndx = find_pid_lock_idx(seach.pageID);
                    DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);
                    //DB_PR("---add---database_gz[database_cw.dIndx].changqi=%u\r\n",database_gz[database_cw.dIndx].changqi);
                    
                    DB_PR("---add 1---xmh =%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);
                    if((database_cw.dIndx == 0)||(database_gz[database_cw.dIndx].dIndx_gz > shengyu_all_max))
                    {
                        return_cause_zw_fail =3;
                        send_cmd_to_lcd_pic(0x004C);//todo bl
                    }
                    
                    if((database_gz[database_cw.dIndx].changqi == 0)
                        ||(database_gz[database_cw.dIndx].changqi == 2))
                    {
                        Del_FR(seach.pageID);
                        DB_PR("-----normal or changqi lock temp cunwu \r\n");	
                    }
                    else
                    {
                        DB_PR("-----changqi cunwu \r\n");			
                    }
                    
                    del_zw_database(seach.pageID);
                    break;
                    //myfree(SRAMIN,str);
                }
                else 
                {
                    send_cmd_to_lcd_pic(0x000d);
                    DB_PR("-----4444----f----- \r\n");				
                    ShowErrMessage(ensure);			
                    break;
                }
                    		
            }
            else
            {
                DB_PR("-----333--tezheng  f------\r\n");
                ShowErrMessage(ensure);			
                break;     
            }
            //BEEP=0;//关闭蜂鸣器-----------
            delay_ms(600);
            //LCD_Fill(0,100,lcddev.width,160,WHITE);
        }
        else
        {
            DB_PR("-----2222   f-----\r\n");
            ShowErrMessage(ensure);		
            //break;//add
        }
        DB_PR("-4-hearbeat...return_cause_zw=%d \r\n",return_cause_zw);
        if(return_cause_zw ==1)//todo
        {
            DB_PR("--zw task 2 over \r\n");
            return_cause_zw =0;
            vTaskDelete(NULL);
        }

        if(i==29)
        {
            DB_PR("--zw qu time out \r\n");
            return_cause_zw_fail =3;
            send_cmd_to_lcd_pic(0x004C);//todo bl
        }
    }
    return_cause_zw =0;
    vTaskDelay(1);
    vTaskDelete(NULL);
		
}

//删除指纹
void Del_FR(u16 num)
{

    DB_PR("1******del***pageid hex**********num=%x \r\n",num);
    DB_PR("2******del***pageid dec**********num=%d \r\n",num);

    // if(HandShakeFlag ==1)
    // {
    //     return_cause_zw_handshake_fail =3;
    //     send_cmd_to_lcd_pic(0x004D);
    //     DB_PR("---zhiwen connect fail\r\n");
    //     return;
    // }
	u8  ensure;
	//u16 num;
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	DB_PR("删除指纹 \r\n");
	DB_PR("请输入指纹ID按Enter发送 \r\n");
	DB_PR("0=< ID <=299 \r\n");

	delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_delFR);
	//num=GET_NUM();//获取返回的数值

	if(num==0xFFFF)
        return ;
		//goto MENU ; //返回主页面-------------
	// else if(num==0xFF00)
	// 	ensure=PS_Empty();//清空指纹库
	else 
		ensure=PS_DeletChar(num,1);//删除单个指纹
	if(ensure==0)
	{
		//LCD_Fill(0,120,lcddev.width,160,WHITE);
		DB_PR("---del ok -----删除指纹成功 \r\n");		
	}
    else
		ShowErrMessage(ensure);	
	delay_ms(1200);
	PS_ValidTempleteNum(&ValidN);//读库指纹个数
	//LCD_ShowNum(56,80,AS608Para.PS_max-ValidN,3,16);
    DB_PR("zhiwen number =%d \r\n",AS608Para.PS_max-ValidN);
//MENU:	
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_menu);

    DB_PR("AS608Para.PS_max=%d, ValidN =%d \r\n",AS608Para.PS_max, ValidN);
    DB_PR("库容量:%d     对比等级: %d\r\n",AS608Para.PS_max-ValidN,AS608Para.PS_level);

}

void del_zw_database(u16 num)
{
    // if(HandShakeFlag ==1)
    // {
    //     return_cause_zw_handshake_fail =4;
    //     send_cmd_to_lcd_pic(0x004D);
    //     DB_PR("---zhiwen connect fail\r\n");
    //     return;
    // }
    DB_PR("-------database_ad.zhiwen_page_id_adm[num]=%d\r\n",database_ad.zhiwen_page_id_adm[num]);
    if(database_ad.zhiwen_page_id_adm[num] ==1) //if(1)
    {

        // database_cw.dIndx = find_pid_lock_idx(num);

        DB_PR("---add 2---xmh =%u\r\n",database_gz[database_cw.dIndx].dIndx_gz);
        if((database_cw.dIndx == 0)||(database_gz[database_cw.dIndx].dIndx_gz > shengyu_all_max))
        {
            DB_PR("---no find zhiwen\r\n");
            goto del_zw_fail;
        }
        else
        {

            DB_PR("---zhiwen etc... will be deleted\r\n");

            // database_gz[database_cw.dIndx].state_gz =database_cw.state;
            DB_PR("---add---database_cw.dIndx=%u\r\n",database_cw.dIndx);


            int16_t guimen_gk_temp =0;

            guimen_gk_temp = database_cw.dIndx ;

            uint16_t j=0,k=0;

            k = guimen_gk_temp/24;
            j = guimen_gk_temp%24;

            if(guimen_gk_temp%24 ==0)
            {
                k = guimen_gk_temp/24 -1;
                j = 24;
            }
            DB_PR("------open------ board-addr k+1=%d, lock-addr j=%d--\r\n",k+1,j);




            DB_PR("--lock:%d ok--.\r\n",j);
            send_cmd_to_lock(k+1,j);
            send_cmd_to_lcd_bl(0x10a0,database_gz[database_cw.dIndx].dIndx_gz);//weishu bugou


            DB_PR("-----2-----[ * ] Starting audio pipeline");
            xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_OPEN, 10, NULL);




            if((database_gz[database_cw.dIndx].lock == 0)
                &&(database_gz[database_cw.dIndx].changqi == 0))
            {
                switch (database_gz[database_cw.dIndx].dzx_mode_gz)
                {
                case 1:
                    //d
                    shengyu_da ++;
                    tongbu_gekou_shuliang_d(shengyu_da); 
                    break;
                case 2:
                    //z
                    shengyu_zhong ++;
                    tongbu_gekou_shuliang_z(shengyu_zhong); 
                    break;
                case 3:
                    //x
                    shengyu_xiao ++;
                    tongbu_gekou_shuliang_x(shengyu_xiao); 
                    break;

                default:
                    break;
                }

                shengyu_all ++ ;
                tongbu_gekou_shuliang_all(shengyu_all);

                nvs_wr_shengyu_da(1);
                nvs_wr_shengyu_zhong(1);
                nvs_wr_shengyu_xiao(1);

            }




            DB_PR("----test--.\r\n");  
            

            //if(0 != database_cw.dzx_mode)
            {

                // char key_name[15];//15
                // esp_err_t err;


                DB_PR("--database_gz[database_cw.dIndx].changqi=%d\r\n",database_gz[database_cw.dIndx].changqi);
                if((database_gz[database_cw.dIndx].changqi == 0)
                    ||(database_gz[database_cw.dIndx].changqi == 2))
                {
                    database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                    //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
                    nvs_wr_cunwu_mode_gz(1);
                    //nvs_wr_dzx_mode_gz(1);

                    // database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                    // database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                    // nvs_wr_phone_number_nvs_gz(1);
                    // nvs_wr_mima_number_nvs_gz(1);
                    database_gz[database_cw.dIndx].state_gz =0;
                    nvs_wr_state_gz(1);

                    DB_PR("--database_gz[database_cw.dIndx].changqi=%d\r\n",database_gz[database_cw.dIndx].changqi);
                    if(database_gz[database_cw.dIndx].changqi == 2)
                    {
                        database_gz[database_cw.dIndx].changqi =0;
                        nvs_wr_glongtime_gz(1);
                        tongbu_changqi();
                    }

                    database_cw.zhiwen_page_id =num;
                    database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id] =0;
                    nvs_wr_adm_zwpageid_flag(1,database_cw.zhiwen_page_id);
                    database_gz[database_cw.dIndx].zhiwen_page_id_gz = 0;
                    nvs_wr_zw_pageid_gz(1);


                }






                //custumer   mingming kongjian   todo
                // esp_err_t err = save_u16_value(STORAGE_NAMESPACE,"dw_dIndx",database_cw.dIndx);
                // if (err != ESP_OK) DB_PR("Error (%s) write data from NVS!\n", esp_err_to_name(err));

                // err = read_u16_value(STORAGE_NAMESPACE,"dw_dIndx", (uint16_t*)(&database_cw.dIndx));
                // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

                //unique number

            }

            send_cmd_to_lcd_pic(0x000e); 
        }

    }
    else
    {
del_zw_fail:
        send_cmd_to_lcd_pic(0x000d); //
        DB_PR("---del_zw_fail--1--\r\n");
    }
    

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
        uart_config_t uart_config0 = {
            .baud_rate = 9600,// lock
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };

    uart_config_t uart_config2 = {
        .baud_rate = 57600,// zhiwen
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };



#if _DEBUG_
    uart_config0.baud_rate = 115200;
#else
    uart_config0.baud_rate = 9600;
#endif
        // Set UART log level
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_param_config(UART_NUM_0, &uart_config0);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // uart_set_pin(UART_NUM_0, ECHO_TEST3_TXD, ECHO_TEST3_RXD, ECHO_TEST3_RTS, ECHO_TEST3_CTS);
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // DB_PR("Start ttl application test and configure UART2.\r\n");

    //1
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // //2
    // uart_param_config(UART_NUM_2, &uart_config2);
    // uart_set_pin(UART_NUM_2, ECHO_TEST2_TXD, ECHO_TEST2_RXD, ECHO_TEST2_RTS, ECHO_TEST2_CTS);
    // uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);


    //3 io moni
    uart_param_config(UART_NUM_2, &uart_config2);
    uart_set_pin(UART_NUM_2,  ECHO_TEST2_TXD, ECHO_TEST2_RXD, ECHO_TEST2_RTS, ECHO_TEST2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);

    // //3 io moni
    // uart_param_config(UART_NUM_3, &uart_config3);
    // uart_set_pin(UART_NUM_3, ECHO_TEST3_TXD, ECHO_TEST3_RXD, ECHO_TEST3_RTS, ECHO_TEST3_CTS);
    // uart_driver_install(UART_NUM_3, BUF_SIZE * 2, 0, 0, NULL, 0);



}






#define PLAYBACK_RATE       16000
#define PLAYBACK_CHANNEL    1
#define PLAYBACK_BITS       16

void audio_play_one_mp3(int num)
{
    audio_play_mp3_task =1;
    DB_PR("\r\n22---mp3 num=%d----\r\n",num);
    DB_PR("\r\n22---mp3 audio_play_mp3_stop=%d----\r\n",audio_play_mp3_stop);
    // DB_PR("\r\n22---mp3 audio_play_mp3_over=%d----\r\n",audio_play_mp3_over);
    //if(NULL!=&num)
    if(audio_play_mp3_stop==0)//kaiguan
    {
        DB_PR("\r\n33---mp3 num=%d----\r\n",num);
        //while (1) 
        {
            // vTaskDelay(10 / portTICK_PERIOD_MS);
            
            //if(audio_play_mp3_over==1)
            {
                audio_play_mp3_over =0;
                DB_PR("22---audio_play_mp3_over=%d----\r\n",audio_play_mp3_over);

                // audio_pipeline_stop(pipeline);
                // audio_pipeline_wait_for_stop(pipeline);
                // audio_pipeline_terminate(pipeline);
                // audio_pipeline_reset_ringbuffer(pipeline);
                // audio_pipeline_reset_elements(pipeline);
                // DB_PR("[2.6-b2] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)\r\n");
                
                // es7134_pa_power(0);
                // vTaskDelay(100 / portTICK_PERIOD_MS);
                i2s_stream_set_clk(i2s_stream_writer, PLAYBACK_RATE, PLAYBACK_BITS, PLAYBACK_CHANNEL);
                // es7134_pa_power(1);
                // vTaskDelay(100 / portTICK_PERIOD_MS);
                audio_element_set_uri(tone_stream_reader, tone_uri[num]);//TONE_TYPE_OPEN
                audio_pipeline_run(pipeline);  

                //break;
            }
        }
        
        //i=0;

    }
    
    

    vTaskDelay(1);
    audio_play_mp3_task =0;
    vTaskDelete(NULL);//taskhandle_mp3

}


// void audio_play_my_mp3(void)
// {
//     u8 i=1;
//     while (1) {
//         vTaskDelay(10 / portTICK_PERIOD_MS);
        

//         if((audio_play_mp3_over==1)&&(i==2))
//         {
//             // audio_pipeline_stop(pipeline);
//             // audio_pipeline_wait_for_stop(pipeline);
//             // audio_pipeline_terminate(pipeline);
//             // audio_pipeline_reset_ringbuffer(pipeline);
//             // audio_pipeline_reset_elements(pipeline);

//             audio_play_mp3_over =0;
//             i++;
//             DB_PR("22---audio_play_mp3_over=%d----\r\n",audio_play_mp3_over);

//             DB_PR("[2.6-b2] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)\r\n");
//             audio_element_set_uri(tone_stream_reader, tone_uri[TONE_TYPE_OPEN]);//TONE_TYPE_OPEN
//             audio_pipeline_run(pipeline);  
//             break;
//         }
//         else if((audio_play_mp3_over==1)&&(i==1))
//         {
//             // audio_pipeline_stop(pipeline);
//             // audio_pipeline_wait_for_stop(pipeline);
//             // audio_pipeline_terminate(pipeline);
//             // audio_pipeline_reset_ringbuffer(pipeline);
//             // audio_pipeline_reset_elements(pipeline);

//             audio_play_mp3_over=0;
//             i++;
//             DB_PR("11---audio_play_mp3_over=%d----\r\n",audio_play_mp3_over);

//             DB_PR("[2.6-b1] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)\r\n");
//             audio_element_set_uri(tone_stream_reader, tone_uri[database_gz[database_cw.dIndx].dIndx_gz]);//
//             audio_pipeline_run(pipeline);
//             continue;
      
//         }


//     }
//     audio_play_mp3_over =0;
//     //i=0;

//     vTaskDelay(1);
//     vTaskDelete(NULL);

// }



// 8388
void es7134_pa_power(bool enable)
{
    gpio_config_t  io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT64(GPIO_NUM_4);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    if (enable) {
        gpio_set_level(GPIO_NUM_4, 1);
    } else {
        gpio_set_level(GPIO_NUM_4, 0);
    }
}



void audio_init(void)
{

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // DB_PR("[ 1 ] Start codec chip");
    // audio_board_handle_t board_handle = audio_board_init();
    // audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);




    DB_PR("[2.0] Create audio pipeline for playback\r\n");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, pipeline, return);

    DB_PR("[2.1] Create tone stream to read data from flash\r\n");
    tone_stream_cfg_t tone_cfg = TONE_STREAM_CFG_DEFAULT();
    tone_cfg.type = AUDIO_STREAM_READER;
    tone_stream_reader = tone_stream_init(&tone_cfg);

    DB_PR("[2.2] Create i2s stream to write data to codec chip\r\n");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    DB_PR("[2.3] Create mp3 decoder to decode mp3 file\r\n");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);

    DB_PR("[2.4] Register all elements to audio pipeline\r\n");
    audio_pipeline_register(pipeline, tone_stream_reader, "tone");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    DB_PR("[2.5] Link it together [flash]-->tone_stream-->mp3_decoder-->i2s_stream-->[codec_chip]\r\n");
    const char *link_tag[3] = {"tone", "mp3", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    es7134_pa_power(0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    i2s_stream_set_clk(i2s_stream_writer, PLAYBACK_RATE, PLAYBACK_BITS, PLAYBACK_CHANNEL);
    es7134_pa_power(1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if(audio_play_mp3_stop ==0)
    {
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        DB_PR("[2.6] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)\r\n");
        audio_element_set_uri(tone_stream_reader, tone_uri[TONE_TYPE_KAIJI]);//kaji
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    DB_PR("[ 3 ] Set up event listener\r\n");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    // audio_event_iface_handle_t m_audio_evt;
    m_audio_evt = audio_event_iface_init(&evt_cfg);

    DB_PR("[3.1] Listening event from all elements of pipeline\r\n");
    audio_pipeline_set_listener(pipeline, m_audio_evt);



    if(audio_play_mp3_stop ==0)
    {
        DB_PR("[ 4 ] Start audio_pipeline\r\n");
        audio_pipeline_run(pipeline);
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    DB_PR("[ 4 ] Listen for all pipeline events\r\n");

    int j=0;
    while (1) {
        // j++;
        // // DB_PR("-------- heart1---------\r\n");
        // if(j%50 ==0)
        // {
        //     DB_PR("-------- heart2---------\r\n");
        // }
        // vTaskDelay(10 / portTICK_PERIOD_MS);
        audio_event_iface_msg_t msg = { 0 };
        esp_err_t ret = audio_event_iface_listen(m_audio_evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            DB_PR( "[ * ] Event interface error : %d\r\n", ret);
            continue;
        }
        // DB_PR("-------- heart3---------\r\n");

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) mp3_decoder
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(mp3_decoder, &music_info);

            DB_PR("\n[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d\n\n",
                    music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_setinfo(i2s_stream_writer, &music_info);
            // i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
            continue;
        }


        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            DB_PR( "[ * ] Stop event received\r\n");



            audio_pipeline_stop(pipeline);
            audio_pipeline_wait_for_stop(pipeline);
            audio_pipeline_terminate(pipeline);
            audio_pipeline_reset_ringbuffer(pipeline);
            audio_pipeline_reset_elements(pipeline);
            audio_play_mp3_over=1;
            DB_PR("00---audio_play_mp3_over=%d----\r\n",audio_play_mp3_over);

            // //set_next_file_marker();
            // DB_PR("[2.6-b] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)");
            // audio_element_set_uri(tone_stream_reader, tone_uri[test_i]);
            // audio_pipeline_run(pipeline);

            // test_i++;
            // if(test_i == 10)
            // {
            //     test_i=0;
            // }



        }



    }
    
    DB_PR("[ 5 ] Stop audio_pipeline\r\n");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, tone_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, mp3_decoder);

    /* Terminal the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(m_audio_evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(tone_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(mp3_decoder);

    DB_PR("[ 6 ] Stop over \r\n");
    vTaskDelay(1);
    vTaskDelete(NULL);

}


void close_mp3(void)
{
    DB_PR("[ 5 ] Stop audio_pipeline\r\n");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, tone_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, mp3_decoder);

    /* Terminal the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);


    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(m_audio_evt);


    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(tone_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(mp3_decoder);

}





// #define DZ_CW_MD "_dz_cw_md"//guizi    _dz_cw_md
// #define DZ_DZXMD "_dz_dzxmd"
// #define DZ_PHONE "_dz_phone"
// #define DZ_MIMA  "_dz_mima"
// #define DZ_ST    "_dz_st"

void read_nvs_guizi_all()
{
    char key_num[3];//65536
    //todo
    char key_name[15];
    esp_err_t err;
    uint16_t page_id_temp1[ZHIWEN_PAGE_ID_MAX]={0};
    uint16_t page_id_temp2[ZHIWEN_PAGE_ID_MAX]={0};


    nvs_wr_mp3_ctl(0);
    DB_PR("audio_play_mp3_stop=%d\r\n",audio_play_mp3_stop);


    DB_PR("AS608Para.PS_max=%d, ValidN =%d ",AS608Para.PS_max, ValidN);
    for(uint16_t i=0;i<AS608Para.PS_max;i++)
    {
        // database_ad.zhiwen_page_id_adm[i] =0;
        nvs_wr_adm_zwpageid_flag(0,i);
    }


    uint16_t j =0, k=0;
    int rand_temp=0;
    for(uint16_t i=0;i<AS608Para.PS_max;i++)
    {
        if(database_ad.zhiwen_page_id_adm[i] ==0)//weiyong
        {
            page_id_temp1[j++] = i;
        }
        else//yi yong
        {
            page_id_temp2[k++] = i;
        }
    }
    DB_PR("shengyu j=%d, onuse k=%d\r\n",j,k);
    DB_PR("--ok--no use  zhiwen idx\r\n");
    uart0_debug_data_dec(page_id_temp1,j);
    DB_PR("--fail--has use zhiwen idx\r\n");
    uart0_debug_data_dec(page_id_temp2,k);
    




    //shengyu_all_max = shengyu_da_max+ shengyu_zhong_max + shengyu_xiao_max;

    //shengyu_xiao_max = shengyu_all_max - (shengyu_da_max+ shengyu_zhong_max);



    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        database_cw.dIndx =i;
        nvs_wr_fenpei_gz(0);//2
        if(1== database_gz[database_cw.dIndx].state_fenpei_gz)
        {
            //guizi
            nvs_wr_cunwu_mode_gz(0);
            nvs_wr_dzx_mode_gz(0);
            nvs_wr_phone_number_nvs_gz(0);
            nvs_wr_mima_number_nvs_gz(0);
            nvs_wr_state_gz(0);
            
            nvs_wr_index_gz(0);//3

            nvs_wr_glock_gz(0);//3
            nvs_wr_glongtime_gz(0);//3

            nvs_wr_zw_pageid_gz(0);
            

            // nvs_wr_fenpei_gz(0);
            // //admin
            // nvs_wr_shengyu_da(0);
            // nvs_wr_shengyu_zhong(0);
            // nvs_wr_shengyu_xiao(0);
        }
        
        DB_PR("\r\n");
    }


  //admin
    nvs_wr_shengyu_da(0);
    nvs_wr_shengyu_zhong(0);
    nvs_wr_shengyu_xiao(0);

    nvs_wr_mima_number_adm(0);//key

    DB_PR("---database_ad.mima_number_adm=%d----\n",database_ad.mima_number_adm);
    

    shengyu_all = shengyu_da + shengyu_zhong + shengyu_xiao;

    DB_PR("---shengyu_all=%d----\n",shengyu_all);

    DB_PR("---shengyu_da=%d----\n",shengyu_da);
    DB_PR("---shengyu_zhong=%d----\n",shengyu_zhong);
    DB_PR("---shengyu_xiao=%d----\n",shengyu_xiao);



    nvs_wr_shengyu_all_max(0);//duoyu
    nvs_wr_shengyu_da_max(0);
    nvs_wr_shengyu_zhong_max(0);
    nvs_wr_shengyu_xiao_max(0);

    //shengyu_all_max = shengyu_da_max + shengyu_zhong_max + shengyu_xiao_max;
    DB_PR("-2-shengyu_all_max=%d----\n",shengyu_all_max);
    DB_PR("---shengyu_da_max=%d----\n",shengyu_da_max);
    DB_PR("---shengyu_zhong_max=%d----\n",shengyu_zhong_max);
    DB_PR("---shengyu_xiao_max=%d----\n",shengyu_xiao_max);

/*
    for(uint16_t i=1;i<=shengyu_all_max;i++)
    {
        DB_PR("shengyu index =%03d,cunwu_mode_gz =%d,dzx_mode_gz =%d,",\
                i, database_gz[i].cunwu_mode_gz,database_gz[i].dzx_mode_gz);


        DB_PR("phone?=%11llu,mima?=%6u,", database_gz[i].phone_number_nvs_gz, database_gz[i].mima_number_nvs_gz);

        DB_PR("state_fenpei_gz?=%d, state?=%d,lock?=%d,changqi?=%d\r\n",\
            database_gz[i].state_fenpei_gz,\
            database_gz[i].state_gz,\
            database_gz[i].lock,\
            database_gz[i].changqi);

    }
*/


}


#if 0
void log_debug(void)
{
    
    uint16_t j=0,k=0,l=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        if(1== database_gz[i].state_fenpei_gz)
        {

            DB_PR("index =%03d,cunwu_mode =%d,dzx_mode =%d,",\
                    i, database_gz[i].cunwu_mode_gz,database_gz[i].dzx_mode_gz);


            DB_PR("phone?=%11llu,mima?=%6u,", database_gz[i].phone_number_nvs_gz, database_gz[i].mima_number_nvs_gz);

            DB_PR("fenpei?=%d, state?=%d,lock?=%d,changqi?=%d, ",\
                    database_gz[i].state_fenpei_gz,\
                    database_gz[i].state_gz,\
                    database_gz[i].lock,\
                    database_gz[i].changqi);
            
            // if(database_gz[i].state_fenpei_gz == 1)
            // {
                j++;
                DB_PR("xmh dIndx= %03d, ",database_gz[i].dIndx_gz);
                DB_PR("xm j= %03d, ",j);//xiangmenhao
            // }




            if(1== database_gz[i].dzx_mode_gz)
            {
                //char *	_EXFUN(itoa,(int, char *, int));

                buff_temp1[k] = database_gz[i].dIndx_gz;
                DB_PR("b_temp1[k]= %03d ",buff_temp1[k]);//xiangmenhao
                
                itoa(buff_temp1[k],(char*)(buff_temp1_c+4*(k)),10);//+4*(i-1)
                k++;
            }
            if(2== database_gz[i].dzx_mode_gz)
            {
                buff_temp2[l] = database_gz[i].dIndx_gz;
                DB_PR("b_temp2[l]= %03d ",buff_temp2[l]);//xiangmenhao
                itoa(buff_temp2[l],(char*)(buff_temp2_c+4*(l)),10);//+4*(i-1)
                l++;
            }





            DB_PR("\r\n");
        }

        //DB_PR("---i=%d\r\n",i);

    }
}
#endif


static void oneshot_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    DB_PR("One-shot timer called, time since boot: %lld us", time_since_boot);
    //send_cmd_to_lcd_pic(KAIJI_PIC);
    send_cmd_to_lcd_pic(ADMIN_LOGIN_PIC);
}

void zhiwen_init(void )
{
    u8 ensure;
    DB_PR("与AS608模块握手....\r\n");
	//while(PS_HandShake(&AS608Addr))//与AS608模块握手
    for(uint16_t i=0;i<2;i++)
	{
        if(PS_HandShake(&AS608Addr))
        {
            delay_ms(400);
            DB_PR("--1--PS_HandShake no-----未检测到模块!!!\r\n");
            delay_ms(800);
            DB_PR("---1---尝试连接模块...\r\n");	

            if(i==2)
            {
                DB_PR("---1---zhiwen HandShake fail...\r\n");	
                HandShakeFlag =1;
                break;
            }
        }
        else
        {
            break;
        }
        

        // u8 data = 0x35;
        // uart_write_bytes(UART_NUM_0, (const char *) &data, 1);//------UART_NUM_2------	  
        // uart_write_bytes(UART_NUM_1, (const char *) &data, 1);//------UART_NUM_2------	  
	}
    delay_ms(100);
    DB_PR("2y-通讯成功!!!\r\n");
    DB_PR("2-波特率:%d   地址:%x\r\n",usart2_baund,AS608Addr);


	ensure=PS_ValidTempleteNum(&ValidN);//读库指纹个数
	if(ensure!=0x00)
    {
        DB_PR("1-ensure = %d\r\n",ensure);
        ShowErrMessage(ensure);//显示确认码错误信息	
    }
		
    delay_ms(100);

    DB_PR("------------3y start-----------\r\n");
	ensure=PS_ReadSysPara(&AS608Para);  //读参数 
	if(ensure==0x00)
	{
			// mymemset(str,0,50);
			// sprintf(str,"库容量:%d     对比等级: %d",AS608Para.PS_max-ValidN,AS608Para.PS_level);
			// Show_Str(0,80,240,16,(u8*)str,16,0);
        DB_PR("3-AS608Para.PS_max=%d, ValidN =%d ",AS608Para.PS_max, ValidN);
        DB_PR("3-库容量:%d     对比等级: %d",AS608Para.PS_max-ValidN,AS608Para.PS_level);
        if(0==AS608Para.PS_max)
        {
            DB_PR("------------fail   AS608Para.PS_max ==0-----------\r\n");
        }
	}
	else
    {
        DB_PR("2-ensure = %d\r\n",ensure);
        ShowErrMessage(ensure);	
    }


}






typedef enum {
	KEY_SHORT_PRESS = 1, KEY_LONG_PRESS,
} alink_key_t;

//宏定义一个GPIO
#define KEY_GPIO    39//14 4

static xQueueHandle gpio_evt_queue = NULL;

void IRAM_ATTR gpio_isr_handler(void *arg) {
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void KeyInit(uint32_t key_gpio_pin) {

	// //配置GPIO，下降沿和上升沿触发中断
	// gpio_config_t io_conf;
	// io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
	// io_conf.pin_bit_mask = 1 << key_gpio_pin;
	// io_conf.mode = GPIO_MODE_INPUT;
	// io_conf.pull_up_en = 1;//1;
	// gpio_config(&io_conf);

	// gpio_set_intr_type(key_gpio_pin, GPIO_INTR_NEGEDGE);
	// gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));//10

	// gpio_install_isr_service(0);
	// gpio_isr_handler_add(key_gpio_pin, gpio_isr_handler, (void *) key_gpio_pin);

    // //remove isr handler for gpio number.
    // gpio_isr_handler_remove(GPIO_INPUT_IO_ADMIN);
    // //hook isr handler for specific gpio pin again
    // gpio_isr_handler_add(GPIO_INPUT_IO_ADMIN, gpio_isr_handler, (void*) GPIO_INPUT_IO_ADMIN);



    gpio_config_t io_conf;
    // //disable interrupt
    // io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // //set as output mode
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // //bit mask of the pins that you want to set,e.g.GPIO18/19
    // io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // //disable pull-down mode
    // io_conf.pull_down_en = 0;
    // //disable pull-up mode
    // io_conf.pull_up_en = 0;
    // //configure GPIO with the given settings
    // gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;//GPIO_PIN_INTR_NEGEDGE;//GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO39/35 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;//1;//wai jie?
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_ADMIN,GPIO_INTR_NEGEDGE );//GPIO_INTR_ANYEDGE  GPIO_PIN_INTR_NEGEDGE no

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    // xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
	

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_ADMIN, gpio_isr_handler, (void*) GPIO_INPUT_IO_ADMIN);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(GPIO_INPUT_IO_ZW_JC, gpio_isr_handler, (void*) GPIO_INPUT_IO_ZW_JC);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_ADMIN);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_ADMIN, gpio_isr_handler, (void*) GPIO_INPUT_IO_ADMIN);


}

esp_err_t alink_key_scan(TickType_t ticks_to_wait) {

	uint32_t io_num;
	BaseType_t press_key = pdFALSE;
	BaseType_t lift_key = pdFALSE;
	int backup_time = 0;

    DB_PR("------------ alink_key_scan  -----------\r\n");
	while (1) {

		//接收从消息队列发来的消息
		xQueueReceive(gpio_evt_queue, &io_num, ticks_to_wait);

		//记录下用户按下按键的时间点
		if (gpio_get_level(io_num) == 0) {
			press_key = pdTRUE;
			backup_time = esp_timer_get_time();
			//如果当前GPIO口的电平已经记录为按下，则开始减去上次按下按键的时间点
		} else if (press_key) {
			//记录抬升时间点
			lift_key = pdTRUE;
			backup_time = esp_timer_get_time() - backup_time;
		}

		//近当按下标志位和按键弹起标志位都为1时候，才执行回调
		if (press_key & lift_key) {
			press_key = pdFALSE;
			lift_key = pdFALSE;

			//如果大于1s则回调长按，否则就短按回调
			if (backup_time > 2000000) {
				return KEY_LONG_PRESS;
			} else {
				return KEY_SHORT_PRESS;
			}
		}
	}
}



#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"//sc_event
void key_trigger(void *arg) {
	esp_err_t ret = 0;
	KeyInit(KEY_GPIO);
    DB_PR("------------ key_trigger  -----------\r\n");
	while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
		ret = alink_key_scan(portMAX_DELAY);
		if (ret == -1)
			vTaskDelete(NULL);

		switch (ret) {
		case KEY_SHORT_PRESS:
			DB_PR("----short-------短按触发回调 ... \r\n");
            DB_PR("------------admin mode-----------\r\n");// no print???
            send_cmd_to_lcd_pic(0x0011);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED_BLUE, 0);
            gpio_set_level(LED_GRREN, 0);
            gpio_set_level(LED_RED, 0);


            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED_BLUE, 1);
            gpio_set_level(LED_GRREN, 1);
            gpio_set_level(LED_RED, 1);
			break;

		case KEY_LONG_PRESS:
			DB_PR("----long--------长按触发回调 ... \r\n");
            DB_PR("------------peiwang gai wifimima-----------\r\n");
            if(0==wifi_peiwang_over_flag)
            {
                // ESP_ERROR_CHECK( esp_wifi_disconnect() );
                // ESP_ERROR_CHECK(esp_wifi_stop());



                char ssid[33] = { 0 };
                char password[65] = { 0 };//wifi_passwd
                wifi_config_t wifi_config;
                bzero(&wifi_config, sizeof(wifi_config_t)); /* 将结构体数据清零 */
                memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
                memcpy(wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
                esp_wifi_disconnect() ;//ESP_ERROR_CHECK
                esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) ;

                // esp_wifi_stop();
                // esp_wifi_restore();
                // initialise_wifi();
                // esp_wifi_deinit();
                // xEventGroupWaitBits(wifi_event_group, DISCONNECTED_BIT, 0, 1, portTICK_RATE_MS);
                // esp_wifi_stop
                //esp_event_handler_unregister
                //ESP_ERROR_CHECK( esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler) );
                
                xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);

                ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
                ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
                ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );


                // ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
                xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
            }
			else
            {
                send_cmd_to_lcd_pic(0x005c);//todo
                DB_PR("-------peiwang wei jiesu -------- \r\n");
            }
            
            break;

		default:
			break;
		}
	}

	vTaskDelete(NULL);
}

// static xQueueHandle gpio_evt_queue = NULL;

// static void IRAM_ATTR gpio_isr_handler(void* arg)
// {
//     uint32_t gpio_num = (uint32_t) arg;
//     xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
// }

// static void gpio_task_example(void* arg)
// {
//     uint32_t io_num;
//     for(;;) {
//         if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
//             DB_PR("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));//yizhi dayin????

//             if((io_num==GPIO_INPUT_IO_ADMIN)
//                 &&(gpio_get_level(io_num)==0))
//             {
//                 DB_PR("------------admin mode-----------\r\n");// no print???
//                 send_cmd_to_lcd_pic(0x0011);
//                 vTaskDelay(1000 / portTICK_PERIOD_MS);
//                 gpio_set_level(LED_BLUE, 0);
//                 gpio_set_level(LED_GRREN, 0);
//                 gpio_set_level(LED_RED, 0);


//                 vTaskDelay(1000 / portTICK_PERIOD_MS);
//                 gpio_set_level(LED_BLUE, 1);
//                 gpio_set_level(LED_GRREN, 1);
//                 gpio_set_level(LED_RED, 1);

//             }
//             if((io_num==GPIO_INPUT_IO_ZW_JC)
//                 &&(gpio_get_level(io_num)==0))
//             {
//                 io_shouzhi_down_flag=1;
//                  DB_PR("------------zw chong an-----------\r\n");
//                 //send_cmd_to_lcd_pic(0x0011);


//                 // vTaskDelay(1000 / portTICK_PERIOD_MS);
//                 // gpio_set_level(LED_BLUE, 0);
//                 // gpio_set_level(LED_GRREN, 0);
//                 // gpio_set_level(LED_RED, 0);


//                 // vTaskDelay(1000 / portTICK_PERIOD_MS);
//                 // gpio_set_level(LED_BLUE, 1);
//                 // gpio_set_level(LED_GRREN, 1);
//                 // gpio_set_level(LED_RED, 1);

//             }

//         }


//     }
//     vTaskDelete(NULL);
// }


static void gpio_task_example1(void* arg)
{
    uint32_t io_num;
    uint32_t tick_times=0;
    bool led_green_state=0;
    bool led_blue_state=0;
    for(;;) 
    {
        
        // DB_PR("------------system heart-----------\r\n");
        tick_times++;
        vTaskDelay(10 / portTICK_PERIOD_MS);//on 
		if(tick_times%100==0)
		{
            DB_PR("------------system heart-----------\r\n");
            if(led_green_state == 0)
            {
                // DB_PR("------------gr led on-----------\r\n");
                led_green_state =1;
                //---------system state----------
                // vTaskDelay(1000 / portTICK_PERIOD_MS);//on 
                // gpio_set_level(LED_BLUE, 0);
                gpio_set_level(LED_GRREN, 0);
                // gpio_set_level(LED_RED, 0);
            }
            else if(led_green_state == 1)
            {
                // DB_PR("------------gr led off-----------\r\n");
                led_green_state =0;
                // vTaskDelay(1000 / portTICK_PERIOD_MS);//off del
                // gpio_set_level(LED_BLUE, 1);
                gpio_set_level(LED_GRREN, 1);
                // gpio_set_level(LED_RED, 1);
            }
            // DB_PR("\r\n");

		}


		if(tick_times%wifi_led_duration_time==0)
        {
            if(led_blue_state == 0)
            {
                // DB_PR("------------bl led on-----------\r\n");
                led_blue_state =1;
                //---------wifi state----------
                // vTaskDelay(wifi_led_duration_time / portTICK_PERIOD_MS);//on 
                gpio_set_level(LED_BLUE, 0);
                // gpio_set_level(LED_GRREN, 0);
                // gpio_set_level(LED_RED, 0);
            }
            else if(led_blue_state == 1)
            {
                // DB_PR("------------bl led off-----------\r\n");
                led_blue_state =0;
                // vTaskDelay(wifi_led_duration_time / portTICK_PERIOD_MS);//off del
                gpio_set_level(LED_BLUE, 1);
                // gpio_set_level(LED_GRREN, 1);
                // gpio_set_level(LED_RED, 1);
            }
            // DB_PR("\r\n");

        }


    }
    vTaskDelete(NULL);
}




/* Esptouch example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// #include <string.h>
// #include <stdlib.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "esp_wifi.h"
// #include "esp_wpa2.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "tcpip_adapter.h"
// #include "esp_smartconfig.h"





// /* FreeRTOS event group to signal when we are connected & ready to make a request */
// static EventGroupHandle_t s_wifi_event_group;

// /* The event group allows multiple bits for each event,
//    but we only care about one event - are we connected
//    to the AP with an IP? */
// static const int CONNECTED_BIT = BIT0;
// static const int ESPTOUCH_DONE_BIT = BIT1;
// static const int WIFI_FAIL_BIT =    BIT2;
//static const char *TAG = "smartconfig_example";

static int retry_num = 0; 
static void event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data)
{
    // static int retry_num = 0;           /* 记录wifi重连次数 */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // if(wifi_peiwang_over_flag==0)
        {
            esp_wifi_connect();
            DB_PR( "wifi start connect\r\n");
        }
        DB_PR( "wifi start-2\r\n");
        // xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_led_duration_time =500;
        wifi_connected_flag =0;
        DB_PR("-2-wifi_connected_flag =%d-----.\r\n",wifi_connected_flag);

		retry_num++;
        DB_PR("retry to connect to the AP %d times. \n",retry_num);

        if (retry_num < 5)  /* WiFi重连次数小于5 */
        {
            esp_wifi_connect();
        }
        else  /* WiFi重连次数大于10 */
        {
            //wifi have disconnected mp3
            if(audio_play_mp3_task!=0)
            {
                audio_play_mp3_task =0;
                vTaskDelay(20 / portTICK_PERIOD_MS);
                DB_PR("----111111 -a-----.\r\n");
                vTaskDelete(taskhandle_mp3);
                // taskhandle_mp3 =NULL;
                DB_PR("----111111 -b-----.\r\n");
                // vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            else
            {
                DB_PR("----222222 =NULL-----.\r\n");
            }
            xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_WIFI_DISCON, 10, (TaskHandle_t* )&taskhandle_mp3);
        

            // retry_num =0;//todo----------------
            /* 将WiFi连接事件标志组的WiFi连接失败事件位置1 */
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);//add
        }
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data; /* 获取IP地址信息*/
        DB_PR("--------------got ip:%d.%d.%d.%d--------------\n\n\n" , IP2STR(&event->ip_info.ip));  /* 打印ip地址*/
        retry_num = 0;                                              /* WiFi重连次数清零 */
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);

        wifi_led_duration_time =20;
        wifi_connected_flag =1;
        DB_PR("-1-wifi_connected_flag =%d-----.\r\n",wifi_connected_flag);
        //todo pic
        
        // vTaskDelay(3000 / portTICK_PERIOD_MS);//on 
        if(audio_play_mp3_task!=0)
        {
            audio_play_mp3_task =0;
            vTaskDelay(20 / portTICK_PERIOD_MS);
            DB_PR("----111111 -a-----.\r\n");
            vTaskDelete(taskhandle_mp3);
            // taskhandle_mp3 =NULL;
            DB_PR("----111111 -b-----.\r\n");
            // vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else
        {
            DB_PR("----222222 =NULL-----.\r\n");
        }
        
        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_WIFI_CON, 10, (TaskHandle_t* )&taskhandle_mp3);
        

    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        DB_PR( "Scan done\r\n");
        //todo yuyin  app cando     扫描wifi完毕
        if(audio_play_mp3_task!=0)
        {
            audio_play_mp3_task =0;
            vTaskDelay(20 / portTICK_PERIOD_MS);
            DB_PR("----111111 -a-----.\r\n");
            vTaskDelete(taskhandle_mp3);
            // taskhandle_mp3 =NULL;
            DB_PR("----111111 -b-----.\r\n");
            // vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else
        {
            DB_PR("----222222 =NULL-----.\r\n");
        }
        
        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_WIFI_SCPASS, 10, (TaskHandle_t* )&taskhandle_mp3);
        
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        DB_PR( "Found channel\r\n");
        send_cmd_to_lcd_pic(0x005A);//
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        DB_PR( "Got SSID and password\r\n");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        //uint8_t ssid[33] = { 0 };
        //uint8_t password[65] = { 0 };
        // char ssid[33] = { 0 };
        // char password[65] = { 0 };//wifi_passwd

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }


        bzero(wifi_ssid, sizeof(wifi_ssid));
        bzero(wifi_passwd, sizeof(wifi_passwd));

        memcpy(wifi_ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(wifi_passwd, evt->password, sizeof(evt->password));
        DB_PR( "peiwang-w-SSID:%s\r\n", wifi_ssid);
        DB_PR( "peiwang-w-PASSWORD:%s\r\n", wifi_passwd);


        /* 将得到的WiFi名称和密码存入NVS*/


        esp_err_t err = save_str_value(STORAGE_NAMESPACE,"wifi_ssid",wifi_ssid );
        if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

        err = save_str_value(STORAGE_NAMESPACE,"wifi_passwd",wifi_passwd );
        if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

        
        // //todo del
        // size_t len;
        // err = read_str_value(STORAGE_NAMESPACE,"wifi_ssid",wifi_ssid , &len);
        // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

        // err = read_str_value(STORAGE_NAMESPACE,"wifi_passwd",wifi_passwd , &len);
        // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

        // nvs_handle_t wificfg_nvs_handler;
        // ESP_ERROR_CHECK( nvs_open(STORAGE_NAMESPACE_ADM, NVS_READWRITE, &wificfg_nvs_handler) );
        // ESP_ERROR_CHECK( nvs_set_str(wificfg_nvs_handler,"wifi_ssid",ssid) );
        // ESP_ERROR_CHECK( nvs_set_str(wificfg_nvs_handler,"wifi_passwd",password) );
        // ESP_ERROR_CHECK( nvs_commit(wificfg_nvs_handler) ); /* 提交 */
        // nvs_close(wificfg_nvs_handler);                     /* 关闭 */ 
        DB_PR("smartconfig save wifi_cfg to NVS .\n");



        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK( esp_wifi_connect() );
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}


static void initialise_wifi(void)
{
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    // ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    // ESP_ERROR_CHECK( esp_wifi_start() );


    nvs_handle_t wificfg_nvs_handler; /* 定义一个NVS操作句柄 */
    //char wifi_ssid[32] = { 0 };     /* 定义一个数组用来存储ssid*/
    //char wifi_passwd[64] = { 0 };   /* 定义一个数组用来存储passwd */


    size_t len;
    // esp_err_t err;

    // err = read_str_value(STORAGE_NAMESPACE,"wifi_ssid",wifi_ssid , &len);
    // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

    // err = read_str_value(STORAGE_NAMESPACE,"wifi_passwd",wifi_passwd , &len);
    // if (err != ESP_OK) DB_PR("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

    DB_PR( "---r SSID:%s\r\n", wifi_ssid);
    DB_PR( "---r PASSWORD:%s\r\n", wifi_passwd);
    /* 打开一个NVS命名空间 */
    nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &wificfg_nvs_handler) ;
    len = sizeof(wifi_ssid);    /* 从NVS中获取ssid */
    nvs_get_str(wificfg_nvs_handler,"wifi_ssid",wifi_ssid,&len) ;
    len = sizeof(wifi_passwd);      /* 从NVS中获取ssid */
    nvs_get_str(wificfg_nvs_handler,"wifi_passwd",wifi_passwd,&len) ;
    nvs_commit(wificfg_nvs_handler) ; /* 提交 */
    nvs_close(wificfg_nvs_handler);                     /* 关闭 */


    DB_PR( "2---r SSID:%s\r\n", wifi_ssid);
    DB_PR( "2---r PASSWORD:%s\r\n", wifi_passwd);
    /* 设置WiFi连接的ssid和password参数 */
    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config_t)); /* 将结构体数据清零 */
    memcpy(wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, wifi_passwd, sizeof(wifi_config.sta.password));

	DB_PR("1-initialise_wifi. \n");
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    /* 设置WiFi连接的参数，主要是ssid和password */
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));//add
	
    ESP_ERROR_CHECK( esp_wifi_start() );
	
	DB_PR("2-initialise_wifi finished. \n");


    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);//WIFI_CONNECTED_BIT |

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_ssid, wifi_passwd);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

 
    // ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    // ESP_ERROR_CHECK(esp_event_handler_unregister(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler) );
    // vEventGroupDelete(s_wifi_event_group);
    DB_PR("3-initialise_wifi finished---. \n");
}

static void smartconfig_example_task(void * parm)
{
    send_cmd_to_lcd_pic(0x0059);//
    wifi_peiwang_over_flag =1;
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, 
                CONNECTED_BIT | ESPTOUCH_DONE_BIT |WIFI_FAIL_BIT, 
                true, false, portMAX_DELAY); 
        if(uxBits & CONNECTED_BIT) {
            DB_PR( "WiFi Connected to ap\r\n");

            // wifi_connected_flag =1;
            // DB_PR("-1-wifi_connected_flag =%d-----.\r\n",wifi_connected_flag);

            // xTaskCreate(&simple_ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            DB_PR( "smartconfig over\r\n");
            wifi_peiwang_over_flag =0;
            send_cmd_to_lcd_pic(0x005B);//

            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }

        if(uxBits & WIFI_FAIL_BIT) {
            DB_PR( "----WIFI_FAIL_BIT-------smartconfig over\r\n");
            wifi_peiwang_over_flag =0;
            send_cmd_to_lcd_pic(0x005c);//todo

            esp_smartconfig_stop();


            // ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
            ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
            // ESP_ERROR_CHECK(esp_event_handler_unregister(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler) );
            // vEventGroupDelete(s_wifi_event_group);

            vTaskDelete(NULL);
        }
    }
    retry_num =0;
}


#include "esp_http_client.h"
esp_err_t _http_event_handler(esp_http_client_event_t *evt);

// static void http_rest_with_hostname_path()
// {
//     esp_http_client_config_t config = {
//         // .host = "express.admin.modoubox.com",
//         // .path = "/api_cabinet/order/checkPaid",
//         .host = "express.admin.modoubox.com",
//         .path = "/mission/Aptest/test",
//         .transport_type = HTTP_TRANSPORT_OVER_TCP,
//         .event_handler = _http_event_handler,
//     };
//     esp_http_client_handle_t client = esp_http_client_init(&config);

// 	DB_PR("\r\n\r\n\r\n");
//     esp_err_t err;





//     uint32_t flash_id;
//     esp_flash_t* chip=NULL;
//     esp_err_t ret = esp_flash_read_id(chip, &flash_id);
//     // TEST_ESP_OK(ret);
//     DB_PR("ret=%X \n",ret);
//     DB_PR("CHIP_ID=%08X\n",flash_id);
//     // if ((flash_id >> 16) == 0xEF) {
//     //     DB_PR("111111111111 \n");
//     //     // return true;
//     // } else {
//     //     DB_PR("222222222222 \n");
//     //     // return false;
//     // }

//     // DB_PR("esp_read_mac(mac, ESP_MAC_WIFI_STA) =%s \n",platform_create_id_string());

//     uint8_t mac[6];
//     esp_read_mac(mac, ESP_MAC_WIFI_STA);
//     DB_PR("MAC_ADDR=");
//     for (uint16_t i = 0; i < 6; i++)//15
//     {
//         DB_PR("%02X",mac[i]);
//     }
//     DB_PR(",MAC_TYPE=%d,CHIP_TYPE=esp32",ESP_MAC_WIFI_STA);
//     DB_PR("\n");
    
//     const esp_partition_t *running = esp_ota_get_running_partition();
//     esp_app_desc_t running_app_info;
//     if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
//         // DB_PR( "Running firmware version: %s", running_app_info.version);
//         DB_PR("RUN_FIRM=%s\n", running_app_info.version);
//     }
//     else
//     {
//         DB_PR("get firmware version err\n");
//     }
    



// //RUN_FIRM=%s&CHIP_ID=%08X&MAC_ADDR=%02X%02X%02X%02X%02X%02X&MAC_TYPE=%02d&CHIP_TYPE=ESP32&type=%d
//     char post_data[500]={0};//15
//     // esp_err_t err;
//     sprintf(post_data, "MAC_TYPE=%02d&CHIP_TYPE=ESP32&type=%d&RUN_FIRM=%s&CHIP_ID=%08X&MAC_ADDR=%02X%02X%02X%02X%02X%02X",
//             ESP_MAC_WIFI_STA,
//             audio_play_mp3_stop,
//             running_app_info.version,
//             flash_id,
//             mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);//audio_play_mp3_stop

//     DB_PR("----------post_data=%s---------------",post_data);


//     // POST
//     //const char *post_data = "field1=value1&field2=value2";
// 	// const char *post_data = "order_code=8268780-1809-32834373";
//     // const char *post_data = "field1=value1&field2=value2";
//     esp_http_client_set_url(client, "/mission/Aptest/test");///post
//     esp_http_client_set_method(client, HTTP_METHOD_POST);
//     esp_http_client_set_post_field(client, post_data, strlen(post_data));
//     err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         DB_PR( "HTTP POST Status = %d, content_length = %d",
//                 esp_http_client_get_status_code(client),
//                 esp_http_client_get_content_length(client));
//         int len =  esp_http_client_get_content_length(client);
//         int read_len = 0;
//         char buf[2048] = {0};
//         read_len = esp_http_client_read(client, buf, 2000);
//         DB_PR("----2----recv data len:%d,content_length: %d,\r\n---buf=%s\r\n",read_len,len,buf);

//         DB_PR("rcv_buf=");
//         for (uint16_t i = 0; i < 500; i++)//15
//         {
//             DB_PR("%c",buf[i]);
//         }
//         DB_PR("\r\n");
//     } else {
//         ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
//     }

//     DB_PR("\r\n");
//     esp_http_client_cleanup(client);
// }









// static void http_test_task(void *pvParameters)
// {
//             // http_rest_with_url();
//     http_rest_with_hostname_path();


//     DB_PR( "Finish http example");
//     vTaskDelete(NULL);
// }






#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "express.admin.modoubox.com"
// #define WEB_SERVER "example.com"
#define WEB_PORT 80
// #define WEB_URL "api_cabinet/order/checkPaid" http://express.admin.modoubox.com/mission/Aptest/test
// #define WEB_URL "http://express.admin.modoubox.com/api_cabinet/order/checkPaid"
// http://express.admin.modoubox.com/mission/Aptest/test
#define WEB_URL "/mission/Aptest/test"

// static const char *TAG = "example";


// static const char *REQUEST = "GET " WEB_URL " HTTP/1.0\r\n"
//     "Host: "WEB_SERVER"\r\n"
//     "User-Agent: esp-idf/1.0 esp32\r\n"
//     "\r\n";



#define http_upload_head "POST /mission/Aptest/test HTTP/1.0\r\n"\
    "Host:"WEB_SERVER"\r\n"\
    "Content-Length:%d\r\n\r\n"\

// RUN_FIRM=%s&CHIP_ID=%08X&MAC_ADDR=%02X%02X%02X%02X%02X%02X&MAC_TYPE=%02d&CHIP_TYPE=ESP32&type=%d
#define http_upload_data     "{\"type\":%d, \
    \"firm_run_version\":\"%s\", \
    \"CHIP_ID\":\"%08X\", \
    \"MAC_ADDR\":\"%02X%02X%02X%02X%02X%02X\", \
    \"MAC_TYPE\":%d, \
    \"CHIP_TYPE\":\"esp32\", \
    \"PARTITION_MODE\":1, \
    \"GUIZI_TYPE\":\"chuwugui\"}"

static char REQUEST[1500]= {0};

char mid_buf[1000];
void send_packetto_server()
{
    uint32_t flash_id;
    esp_flash_t* chip=NULL;
    esp_err_t ret = esp_flash_read_id(chip, &flash_id);
    // TEST_ESP_OK(ret);
    DB_PR("ret=%X \n",ret);
    DB_PR("CHIP_ID=%08X\n",flash_id);
    // if ((flash_id >> 16) == 0xEF) {
    //     DB_PR("111111111111 \n");
    //     // return true;
    // } else {
    //     DB_PR("222222222222 \n");
    //     // return false;
    // }

    // DB_PR("esp_read_mac(mac, ESP_MAC_WIFI_STA) =%s \n",platform_create_id_string());

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    DB_PR("MAC_ADDR=");
    for (uint16_t i = 0; i < 6; i++)//15
    {
        DB_PR("%02X",mac[i]);
    }
    DB_PR(",MAC_TYPE=%d,CHIP_TYPE=esp32",ESP_MAC_WIFI_STA);
    DB_PR("\n");
    
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        // DB_PR( "Running firmware version: %s", running_app_info.version);
        DB_PR("RUN_FIRM=%s\n", running_app_info.version);
    }
    else
    {
        DB_PR("get firmware version err\n");
    }
    


    int len=0;
    int type_t= 1 ;//type--------------




    // char buf[512] = {0};
    char buf_data[1200]={0};

    //5 zhong canshu
    sprintf(buf_data,http_upload_data,
        type_t,
        running_app_info.version,
        flash_id,
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
        ESP_MAC_WIFI_STA);
    len = strlen(buf_data);
    DB_PR("--------len=%d\n",len);
    sprintf(REQUEST,http_upload_head,len);
    strcat(REQUEST,buf_data);
    DB_PR("--------buf=%s\n",REQUEST);

}

// static void http_get_task(void *pvParameters)//
static void http_get_task()//
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    vTaskDelay(10 / portTICK_PERIOD_MS);
    // while(1) 
    {
        int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            // continue;
            // vTaskDelete(NULL);
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        DB_PR( "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            // continue;
            // vTaskDelete(NULL);
        }
        DB_PR( "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            // continue;
            // vTaskDelete(NULL);
        }

        DB_PR( "... connected");
        freeaddrinfo(res);


        //-------------------------------------------------
        send_packetto_server();
        DB_PR( "-------------REQUEST=\r\n%s\r\n\r\n",REQUEST);


        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            // continue;
            // vTaskDelete(NULL);
        }
        DB_PR( "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            // continue;
            // vTaskDelete(NULL);
        }
        DB_PR( "... set socket receiving timeout success");


        memset(mid_buf,0,sizeof(mid_buf));
        DB_PR("---------rev_data=\r\n");
        /* Read HTTP response */
        do {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            DB_PR("\n------------r=%d------------\n",r);
            strcat(mid_buf,recv_buf);
            // vTaskDelay(1000 / portTICK_PERIOD_MS);
            // for(int i = 0; i < r; i++) {
            //     vTaskDelay(10 / portTICK_PERIOD_MS);
            //     putchar(recv_buf[i]);//-------
            // }
        } while(r > 0);
        
        DB_PR("\n\n");

        DB_PR( "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        // for(int countdown = 10; countdown >= 0; countdown--) {
        //     DB_PR( "%d... ", countdown);//idx-----------
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // }
        DB_PR( "Starting again!");
    }

    // vTaskDelete(NULL);
}


#include "cJSON.h"
void printJson(cJSON * root)//以递归的方式打印json的最内层键值对
{
    for(int i=0; i<cJSON_GetArraySize(root); i++)   //遍历最外层json键值对
    {
        cJSON * item = cJSON_GetArrayItem(root, i);        
        if(cJSON_Object == item->type)      //如果对应键的值仍为cJSON_Object就递归调用printJson
            printJson(item);
        else                                //值不为json对象就直接打印出键和值
        {
            DB_PR("%s->", item->string);
            DB_PR("%s\n", cJSON_Print(item));
        }
    }
}

u16 cjson_to_struct_info(char *update_ip_ret,char *text)
{

    if(update_ip_ret == NULL || text == NULL)
    {
        DB_PR("\n----1 err----text=\n%s\n",text);
        return 0;
    }
    // cJSON *root,*psub;

    // cJSON *arrayItem;

    //截取有效json
    // DB_PR("\n----1----text=\n%s\n",text);
    char *index=strchr(text,'{');
    // char *index=strstr(text,"{\"post_data\":{");
    // bzero(text, sizeof(text));
    strcpy(text,index);

    DB_PR("\n----2----text=\n%s\n",text);


    u16 update_status=0;

    cJSON * root = NULL;
    cJSON * item = NULL;//cjson对象

    root = cJSON_Parse(text);     
    if (!root) 
    {
        DB_PR("Error before: [%s]\n",cJSON_GetErrorPtr());
    }
    else
    {
        DB_PR("%s\n", "有格式的方式打印Json:");           
        DB_PR("%s\n\n", cJSON_Print(root));
        DB_PR("%s\n", "无格式方式打印json：");
        DB_PR("%s\n\n", cJSON_PrintUnformatted(root));

        //---------------------
        DB_PR("\n%s\n", "--1--一步一步的获取firm_run_version 键值对:");
        DB_PR("%s\n", "获取result下的cjson对象:");
        item = cJSON_GetObjectItem(root, "result");//
        DB_PR("%s\n", cJSON_Print(item));

        DB_PR("%s\n", "获取post_data下的cjson对象");
        item = cJSON_GetObjectItem(item, "post_data");
        DB_PR("%s\n", cJSON_Print(item));

        DB_PR("%s\n", "获取firm_run_version下的cjson对象");
        item = cJSON_GetObjectItem(item, "firm_run_version");
        DB_PR("%s\n", cJSON_Print(item));

        DB_PR("%s:", item->string);   //看一下cjson对象的结构体中这两个成员的意思
        DB_PR("%s\n", item->valuestring);
                        

        //---------------------
        DB_PR("\n%s\n", "--2--一步一步的获取status 键值对:");
        DB_PR("%s\n", "获取result下的cjson对象:");
        item = cJSON_GetObjectItem(root, "result");//
        DB_PR("%s\n", cJSON_Print(item));

        DB_PR("%s\n", "获取post_data下的cjson对象");
        item = cJSON_GetObjectItem(item, "status");
        DB_PR("%s\n", cJSON_Print(item));
        DB_PR("%s:", item->string);   //看一下cjson对象的结构体中这两个成员的意思
        DB_PR("%d\n", item->valueint);
        update_status = item->valueint;
        DB_PR("update_status=%d\n", update_status);



        if(update_status !=0)
        {
            //---------------------
            DB_PR("\n%s\n", "--3--一步一步的获取url 键值对:");
            DB_PR("%s\n", "获取result下的cjson对象:");
            item = cJSON_GetObjectItem(root, "result");//
            DB_PR("%s\n", cJSON_Print(item));

            DB_PR("%s\n", "获取post_data下的cjson对象");
            item = cJSON_GetObjectItem(item, "url");
            DB_PR("%s\n", cJSON_Print(item));
            DB_PR("%s:", item->string);   //看一下cjson对象的结构体中这两个成员的意思
            DB_PR("%s\n", item->valuestring);

            memcpy(update_ip_ret,item->valuestring,strlen(item->valuestring));
            DB_PR("update_ip_ret=%s\n", update_ip_ret);
        }
        else
        {
            DB_PR("---noip-----update_status=%d\n", update_status);
        }
        



        // DB_PR("\n%s\n", "打印json所有最内层键值对:");
        // printJson(root);
    }



    cJSON_Delete(root);
    return update_status;

}

















/* OTA example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "protocol_examples_common.h"
#include "string.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"


// #define CONFIG_EXAMPLE_CONNECT_WIFI 1
#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

// static const char *TAG = "simple_ota_example";
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256 

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        DB_PR(  "HTTP_EVENT_ERROR\r\n");
        send_cmd_to_lcd_pic(0x0054);//todo
        //语音 服务器连接失败 声音
        if(audio_play_mp3_task!=0)
        {
            audio_play_mp3_task =0;
            vTaskDelay(20 / portTICK_PERIOD_MS);
            DB_PR("----111111 -a-----.\r\n");
            vTaskDelete(taskhandle_mp3);
            // taskhandle_mp3 =NULL;
            DB_PR("----111111 -b-----.\r\n");
            // vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else
        {
            DB_PR("----222222 =NULL-----.\r\n");
        }
        
        xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_SERVER_CONFAIL, 10, (TaskHandle_t* )&taskhandle_mp3);
        
        break;
    case HTTP_EVENT_ON_CONNECTED:
        DB_PR(  "HTTP_EVENT_ON_CONNECTED\r\n");
        break;
    case HTTP_EVENT_HEADER_SENT:
        DB_PR(  "HTTP_EVENT_HEADER_SENT\r\n");
        break;
    case HTTP_EVENT_ON_HEADER:
        DB_PR(  "HTTP_EVENT_ON_HEADER, key=%s, value=%s\r\n", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        DB_PR(  "HTTP_EVENT_ON_DATA, len=%d\r\n", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        DB_PR(  "HTTP_EVENT_ON_FINISH\r\n");
        break;
    case HTTP_EVENT_DISCONNECTED:
        DB_PR(  "HTTP_EVENT_DISCONNECTED\r\n");
        //语音 服务器已断开 声音
        break;
    }
    return ESP_OK;
}

void simple_ota_example_task(void *pvParameter)
{
    char ip_buff_dst[500]={0};
    u16 update_sta=0;
    // xTaskCreate(&http_test_task, "http_test_task", 8192, NULL, 5, NULL);
    // xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
    http_get_task();
    // vTaskDelay(4000 / portTICK_PERIOD_MS);
    update_sta = cjson_to_struct_info(ip_buff_dst,mid_buf);
    DB_PR("---------update_sta=%d\n", update_sta);
    DB_PR("---------ip_buff_dst=%s\n", ip_buff_dst);

    // if(audio_play_mp3_stop == 0)//audio_play_mp3_stop debug
    if(update_sta == 1)//audio_play_mp3_stop
    {
        wifi_led_duration_time =8;
        DB_PR(  "Starting OTA example\r\n");
        send_cmd_to_lcd_pic(0x0057);

        esp_http_client_config_t config = {
            // .url = "http://192.168.10.101:7800/hello-world.bin",//CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL,//"192.168.10.108",//
            .url = ip_buff_dst,//"http://express.admin.modoubox.com/play_mp3.bin",
            .cert_pem = (char *)server_cert_pem_start,
            .event_handler = _http_event_handler,
        };
        memcpy(config.url,ip_buff_dst,strlen(ip_buff_dst));
        DB_PR("config.url=%s\n", config.url);

        DB_PR("----CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL=%s\r\n",CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL);

    #ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
        char url_buf[OTA_URL_SIZE];
        if (strcmp(config.url, "FROM_STDIN") == 0) {
            example_configure_stdin_stdout();
            fgets(url_buf, OTA_URL_SIZE, stdin);
            int len = strlen(url_buf);
            url_buf[len - 1] = '\0';
            config.url = url_buf;
        } else {
            DB_PR(  "Configuration mismatch: wrong firmware upgrade image url\r\n");
            abort();
        }
    #endif

    #ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
        config.skip_cert_common_name_check = true;
    #endif

        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK) {
            send_cmd_to_lcd_pic(0x0058);
            if(audio_play_mp3_task!=0)
            {
                audio_play_mp3_task =0;
                vTaskDelay(20 / portTICK_PERIOD_MS);
                DB_PR("----111111 -a-----.\r\n");
                vTaskDelete(taskhandle_mp3);
                // taskhandle_mp3 =NULL;
                DB_PR("----111111 -b-----.\r\n");
                // vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            else
            {
                DB_PR("----222222 =NULL-----.\r\n");
            }
            
            xTaskCreate(audio_play_one_mp3, "audio_play_my_mp3", 8196, (void*)TONE_TYPE_FIRM_UPOK, 10, (TaskHandle_t* )&taskhandle_mp3);
            
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            DB_PR(  "Firmware upgrade ok\r\n");
            esp_restart();
        } else {
            send_cmd_to_lcd_pic(0x0054);
            DB_PR(  "Firmware upgrade failed\r\n");
        }
        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            DB_PR(  "---ota_heart---\r\n");
        }


    }
    else
    {
        send_cmd_to_lcd_pic(0x0056);//version cant be updated
        DB_PR( "--------http ota reject-------------\n\n");
        vTaskDelete(NULL);
    }
    
    
}


void app_main(void)
{
    // u16 buff_temp1[SHENYU_GEZI_MAX]={0};
    // u16 buff_temp2[SHENYU_GEZI_MAX]={0};

    // u8 buff_temp1_c[400]={0};//char
    // u8 buff_temp2_c[400]={0};//150




    uart_init_all();
    //vTaskDelay(500 / portTICK_PERIOD_MS);

    send_cmd_to_lcd_pic(0x0000);

    //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
    xTaskCreate(echo_task, "uart_echo_task", 2* 1024, NULL, 1, NULL);//1024 10

	
    vTaskDelay(100 / portTICK_PERIOD_MS);



    //---------------zhiwen-----------------
    zhiwen_init();







    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );







  //debug
    for (uint16_t i = 1; i <= SHENYU_GEZI_MAX; i++)//15
    {
        database_gz[i].dzx_mode_gz =3;
    }

    nvs_wr_shengyu_all_max(0);//1
    DB_PR("-1-shengyu_all_max=%d----\n",shengyu_all_max);
    if(shengyu_all_max == 0)
    {
        default_factory_set_first();
        //send_cmd_to_lcd_pic(BOOT_PIC);
    }
    else
    {
        read_nvs_guizi_all();
        //send_cmd_to_lcd_pic(KAIJI_PIC);
    }
    
    DB_PR("切换到开机画面!!!\r\n");

    gpio_pad_select_gpio(RE_485_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RE_485_GPIO, GPIO_MODE_OUTPUT);
    
    

    //todo 4G DTU
    gpio_pad_select_gpio(ECHO_TEST3_TXD);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(ECHO_TEST3_TXD, GPIO_MODE_DISABLE);

    gpio_pad_select_gpio(ECHO_TEST3_RXD);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(ECHO_TEST3_RXD, GPIO_MODE_DISABLE);




    RS485_RX_EN();
    //RS485_TX_EN();

    gpio_pad_select_gpio(LED_BLUE);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_BLUE, 1);


    gpio_pad_select_gpio(LED_GRREN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GRREN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GRREN, 1);


    gpio_pad_select_gpio(LED_RED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_RED, 1);

    xTaskCreate(gpio_task_example1, "gpio_task_example1", 2048, NULL, 10, &taskhandle_temp);

    // gpio_pad_select_gpio(emac->int_gpio_num);
    // gpio_set_direction(emac->int_gpio_num, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(emac->int_gpio_num, GPIO_PULLDOWN_ONLY);
    // gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_POSEDGE);

    // gpio_pad_select_gpio(GPIO_INPUT_IO_ZW_2);
    // /* Set the GPIO as a push/pull output */
    // gpio_set_direction(GPIO_INPUT_IO_ZW_2, GPIO_MODE_INPUT);
    // //gpio_pullup_en(GPIO_INPUT_IO_ZW_2);
    // //gpio_pullup_dis(GPIO_INPUT_IO_ZW_2);
    // gpio_set_pull_mode(GPIO_INPUT_IO_ZW_2,GPIO_PULLUP_ONLY);//GPIO_PULLUP_ONLY




    DB_PR("-----gpio init----- ... \r\n");

    xTaskCreate(key_trigger, "key_trigger", 1024 * 2, NULL, 10,NULL);

    //log_debug();
    if(1)
    {
        uint16_t j=0,k=0,l=0;
        for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
        {
            if(1== database_gz[i].state_fenpei_gz)
            {

                DB_PR("index =%03d,cunwu_mode =%d,dzx_mode =%d,",\
                        i, database_gz[i].cunwu_mode_gz,database_gz[i].dzx_mode_gz);


                DB_PR("phone?=%11llu,mima?=%6u,", database_gz[i].phone_number_nvs_gz, database_gz[i].mima_number_nvs_gz);

                DB_PR("fenpei?=%d, state?=%d,lock?=%d,changqi?=%d, ",\
                        database_gz[i].state_fenpei_gz,\
                        database_gz[i].state_gz,\
                        database_gz[i].lock,\
                        database_gz[i].changqi);
                
                // if(database_gz[i].state_fenpei_gz == 1)
                // {
                    j++;
                    DB_PR("xmh dIndx= %03d, ",database_gz[i].dIndx_gz);
                    DB_PR("xm j= %03d, ",j);//xiangmenhao
                // }

                DB_PR("zw_page_id= %03d, ",database_gz[i].zhiwen_page_id_gz);


                DB_PR("\r\n");
            }

            //DB_PR("---i=%d\r\n",i);

        }
    }

    DB_PR("database_gz[database_cw.dIndx].dIndx_gz=%d--.\r\n",database_gz[1].dIndx_gz); 

    vTaskDelay(30 / portTICK_PERIOD_MS);

    // uart0_debug_data_d(buff_temp1,0x9b);

    tongbu_da();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_zh();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_changqi();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_locked();    

    // err = save_restart_counter();
    //  if (err != ESP_OK) DB_PR("Error (%s) saving restart counter to NVS!\n", esp_err_to_name(err));

    // shengyu_da=shengyu_da_max;//read
    // shengyu_zhong=shengyu_zhong_max;
    // shengyu_xiao=shengyu_xiao_max;


    tongbu_gekou_shuliang_d(shengyu_da);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_z(shengyu_zhong);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    tongbu_gekou_shuliang_x(shengyu_xiao);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    tongbu_gekou_shuliang_all(shengyu_all);
    vTaskDelay(10 / portTICK_PERIOD_MS);


  


    //---------------timer-----------------------
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &oneshot_timer_callback,
            /* argument specified here will be passed to timer callback function */
            // .arg = (void*) periodic_timer,
            .name = "one-shot"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));

    // /* Start the timers */
    // ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 3000000));//5s
    // DB_PR("Started timers, time since boot: %lld us", esp_timer_get_time());

    //DB_PR("111111111111111111  de  11111111111111111111111\r\n");
    DB_PR("-2-shengyu_all_max=%d----\n",shengyu_all_max);
    if(shengyu_all_max == 0)
    {
        send_cmd_to_lcd_pic(BOOT_PIC);
    }
    else
    {
        send_cmd_to_lcd_pic(KAIJI_PIC);
    }




    // for(uint16_t i=1;i<=3;i++)
    // {
    //     //vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     DB_PR("i=%d----\n",i);

    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     gpio_set_level(LED_BLUE, 0);
    //     gpio_set_level(LED_GRREN, 0);
    //     gpio_set_level(LED_RED, 0);


    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     gpio_set_level(LED_BLUE, 1);
    //     gpio_set_level(LED_GRREN, 1);
    //     gpio_set_level(LED_RED, 1);

    // }







    // audio_init();
    xTaskCreate(audio_init, "audio_init0", 2*1024, NULL, 3, NULL);   

    vTaskDelay(3000 / portTICK_PERIOD_MS);//on 



    initialise_wifi();


    // tcpip_adapter_init();
    // ESP_ERROR_CHECK(esp_event_loop_create_default());//duoyu

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // ESP_ERROR_CHECK(example_connect());

#if CONFIG_EXAMPLE_CONNECT_WIFI
    /* Ensure to disable any WiFi power save mode, this allows best throughput
     * and hence timings for overall OTA operation.
     */
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif // CONFIG_EXAMPLE_CONNECT_WIFI

    // xTaskCreate(&simple_ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);

    // vTaskDelay(4000 / portTICK_PERIOD_MS);
    // xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);




}
