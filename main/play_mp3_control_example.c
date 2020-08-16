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

#if __has_include("audio_tone_uri.h")
    #include "audio_tone_uri.h"
#else
    #error "please refer the README, and then make the tone file"
#endif



#include "esp_timer.h"





static void lock_all_open_task();
static void lock_all_clear_task();


esp_timer_handle_t oneshot_timer;
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
// extern const uint8_t lr_mp3_start[] asm("_binary_16b_2c_8000hz_mp3_start");
// extern const uint8_t lr_mp3_end[]   asm("_binary_16b_2c_8000hz_mp3_end");

// // medium rate mp3 audio
// extern const uint8_t mr_mp3_start[] asm("_binary_16b_2c_22050hz_mp3_start");
// extern const uint8_t mr_mp3_end[]   asm("_binary_16b_2c_22050hz_mp3_end");

// // high rate mp3 audio
// extern const uint8_t hr_mp3_start[] asm("_binary_16b_2c_44100hz_mp3_start");
// extern const uint8_t hr_mp3_end[]   asm("_binary_16b_2c_44100hz_mp3_end");



// extern const uint8_t adf_music_mp3_start[] asm("_binary_adf_music_mp3_start");
// extern const uint8_t adf_music_mp3_end[]   asm("_binary_adf_music_mp3_end");
// static int adf_music_mp3_pos;


// const uint8_t * m_mp3_start ;
// const uint8_t * m_mp3_end;
// static int m_mp3_pos;




// // static void set_next_file_marker(int midx)
// static void set_next_file_marker(void)
// {
//     static int midx = 0;

//     switch (midx) {
//         case 0:
//             m_mp3_start = lr_mp3_start;
//             m_mp3_end   = lr_mp3_end;
//             break;
//         case 1:
//             m_mp3_start = mr_mp3_start;
//             m_mp3_end   = mr_mp3_end;
//             break;
//         case 2:
//             m_mp3_start = hr_mp3_start;
//             m_mp3_end   = hr_mp3_end;
//             break;
//         case 3:
//             m_mp3_start = adf_music_mp3_start;
//             m_mp3_end   = adf_music_mp3_end;
//             break;
//         default:
//             m_mp3_start = adf_music_mp3_start;
//             m_mp3_end   = adf_music_mp3_end;
//             DB_PR( "[ * ] Not supported index = %d", midx);
//     }
//     // if (++idx > 2) {
//     //     idx = 0;
//     // }
//     m_mp3_pos = 0;

//     return;
// }



// int mp3_music_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
// {
//     int read_size = m_mp3_end - m_mp3_start - m_mp3_pos;
//     if (read_size == 0) {
//         return AEL_IO_DONE;
//     } else if (len < read_size) {
//         read_size = len;
//     }
//     memcpy(buf, m_mp3_start + m_mp3_pos, read_size);
//     m_mp3_pos += read_size;
//     return read_size;
// }









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



#define ECHO_TEST_TXD  GPIO_NUM_32//32(GPIO_NUM_33)//GPIO_NUM_4
#define ECHO_TEST_RXD  GPIO_NUM_33//33(GPIO_NUM_32)//GPIO_NUM_5
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
#define LED_RED     (GPIO_NUM_27)

#define GPIO_INPUT_IO_0     39//4
// #define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))// | (1ULL<<GPIO_INPUT_IO_1)
#define ESP_INTR_FLAG_DEFAULT 0


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



#define SHENYU_GEZI_MAX 150//310//all kong

#define ZHIWEN_PAGE_ID_MAX 300//all kong


//admin   need save   实时更新
//shuliang
uint16_t shengyu_all=0;//
uint16_t shengyu_da=1;
uint16_t shengyu_zhong=15;
uint16_t shengyu_xiao=100;



//admin 
//and cun de yong
uint16_t shengyu_all_max=0;//shengyu max admin, guding

    uint16_t shengyu_da_max=0;//
    uint16_t shengyu_zhong_max=0;
    uint16_t shengyu_xiao_max=0;


#define BOARD_GK_MAX 35//25//all kong
int16_t hang_shu_max;

#define GUIMENX_GK_MAX 24//all kong

    int16_t guimen1_gk_max=12;
    int16_t guimen2_gk_max=10;
    int16_t guimen3_gk_max=10;//8  guding

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


// int16_t guimen_x_gk_max[25]={12,10,10,16,16,16,16,16,16,16,
//                            16,16,16,16,16,16,16,16,16};//600

int16_t guimen_x_gk_max[BOARD_GK_MAX];//600=24*25   need save?



//1 admin
typedef struct
{
    uint32_t mima_number_adm;  //
    
    bool zhiwen_page_id_adm[300];//flag
    // uint16_t zhiwen_gz_index[300];//gz

    uint8_t shengyu1;
    uint8_t shengyu2;


}shujuku_struct_admin;

shujuku_struct_admin database_ad=
{
    .mima_number_adm = 666888,
    .shengyu1 = 0,
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

shujuku_struct_gz database_gz[SHENYU_GEZI_MAX];//i/lock  idx
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
	bool state;




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
        if(database_gz[i].dIndx_gz == guimen_index)
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







void Add_FR_CQ();
void del_zw_database(u16 num);
#define usart2_baund  57600//串口2波特率，根据指纹模块波特率更改

SysPara AS608Para;//指纹模块AS608参数
u16 ValidN;//模块内有效指纹个数
u8** kbd_tbl;
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
    DB_PR("--1---debug_str:");
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


#define  BL_GK_BH_MAX_LEN 0xFC//


#define  BL_GK_BH_D_LEN 0xFC//0xCE//0x23//(0xCD)
//0x132//306

#define  BL_GK_BH_Z_LEN 0xFC//0x23

#define  BL_GK_BH_CHANGQI_LEN 0xFC//(0xCD)
//0xCD//0xCE

//数组
void send_cmd_to_lcd_bl_len(uint16_t opCode, uint8_t* buff_temp,uint16_t data_len)//变量
{
    uint8_t tx_Buffer[300]={0};  
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
    DB_PR("temp-pic:0x%04d\r\n",temp);

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




void default_factory_set(void)
{

    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        database_cw.dIndx = i;


        database_gz[database_cw.dIndx].dIndx_gz =0;
        database_gz[database_cw.dIndx].state_fenpei_gz =0;

        nvs_wr_index_gz(1);
        nvs_wr_fenpei_gz(1);


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

    
}






void tongbu_da(void)
{
    u16 buff_temp1[300]={0};

    u8 buff_temp1_c[300]={0};//char


    uint16_t j=0,k=0,l=0;
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


    for(uint16_t i=1;i<=300;i++)
    {
        if(buff_temp1_c[i]==0)
        {
            buff_temp1_c[i]=0x20;
            DB_PR("kongd ");
        }
    }


    vTaskDelay(5 / portTICK_PERIOD_MS);
    DB_PR("-----gekouleixing-d-----\r\n");
    send_cmd_to_lcd_bl_len(BL_GK_BH_D,buff_temp1_c,BL_GK_BH_D_LEN);//300
    vTaskDelay(1 / portTICK_PERIOD_MS);

}

void tongbu_zh(void)
{
    u16 buff_temp2[300]={0};
    u8 buff_temp2_c[300]={0};//150

    uint16_t j=0,k=0,l=0;
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


    for(uint16_t i=1;i<=300;i++)
    {
        if(buff_temp2_c[i]==0)
        {
            DB_PR("kongz ");
            buff_temp2_c[i]=0x20;
        }
    }


    vTaskDelay(5 / portTICK_PERIOD_MS);
    DB_PR("-----gekouleixing-z-----\r\n");
    send_cmd_to_lcd_bl_len(BL_GK_BH_Z,buff_temp2_c,BL_GK_BH_Z_LEN);//300 0x23 30
    vTaskDelay(1 / portTICK_PERIOD_MS);


}


void tongbu_changqi(void)
{
    u16 buff_temp2[300]={0};
    u8 buff_temp2_c[300]={0};//150

    uint16_t j=0,k=0,l=0;
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        //vTaskDelay(1);
        if(1== database_gz[i].state_fenpei_gz)
        {

            if(1== database_gz[i].changqi)
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


    for(uint16_t i=1;i<=300;i++)
    {
        //vTaskDelay(1);
        if(buff_temp2_c[i]==0)
        {
            buff_temp2_c[i]=0x20;
            DB_PR("kong ");
        }
    }


    vTaskDelay(10 / portTICK_PERIOD_MS);
    DB_PR("-----changqi--sync----\r\n");
    send_cmd_to_lcd_bl_len(BL_GK_BH_CHANGQI,buff_temp2_c,BL_GK_BH_CHANGQI_LEN);//300 0x23 30
    vTaskDelay(1);


}




uint8_t phone_number[11]={0};  
uint8_t mima_number[6]={0xff, 0xff,0xff,0xff,0xff,0xff};  


uint8_t phone_number_a[11]={0};  
uint8_t mima_number_a[6]={0};  

uint8_t mima_number_a1[6]={0};  
uint8_t mima_number_a2[6]={0};  
uint8_t buff_t[100]={0};

uint8_t return_cause;//xiangmen fail

uint8_t return_cause_zanwu_kx;
uint8_t return_cause_has_be_lock;

static void echo_task2()//lcd
{
    uint16_t bl_addr=0;//bianliang lcd
    uint16_t crc16_temp=0;
    
    uint8_t data_rx_t[BUF_SIZE] = {0};
    uint16_t len_rx_t= len_rx;
    int16_t guimen_gk_temp =0;


    memcpy(data_rx_t,data_rx,len_rx_t);
    //while(1)
    {
        //vTaskDelay(40 / portTICK_PERIOD_MS);
                //&&(flag_rx2 ==0)
        // if ((len_rx2 > 0) ) {
        //     flag_rx2 =1;
        //     len_rx2_m = len_rx2;
        //     memcpy(data_rx2_m,data_rx2,len_rx2_m);
        //     DB_PR("uart2-Received %u bytes:", len_rx2_m);
        //     for (int i = 0; i < len_rx2_m; i++) {
        //         DB_PR("0x%.2X ", (uint8_t)data_rx2_m[i]);
        //     }
        //     DB_PR("] \n");
        // }

								
		


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
                    switch (data_rx_t[3])
                    {
                    case 0x82:
                        //uart_write_bytes(UART_NUM_2, (const char *) (data_rx_t+4), len_rx_t-4);
                        
                        bl_addr = (data_rx_t[4]<<8) + data_rx_t[5];
                        DB_PR("--0x82--.bl_addr=%04x\r\n",bl_addr);

                        uint8_t tx_Buffer[50]={0};  
                        uint8_t bcc_temp=0;
                        switch (bl_addr)
                        {

                        case 0x4F4B:
                            DB_PR("---lcd---ok-----.\r\n");
                            //todo
                            break;

                        default:
                            DB_PR("----------------83 default---------------.\r\n");
                            break;
                        }

                        break;

                    case 0x83:
                        if( data_rx_t[6] == (len_rx_t-7 -2)/2)
                        {
                            //uart_write_bytes(UART_NUM_2, (const char *) (data_rx_t+4), len_rx_t-4);
                            bl_addr = (data_rx_t[4]<<8) + data_rx_t[5];
                            DB_PR("--0x83--.bl_addr=%04x\r\n",bl_addr);
                            DB_PR("--0x83--.bl_addr=%04x\r\n",bl_addr);

                            //uint8_t tx_Buffer[50]={0};  
                            uint8_t tx_Buffer2[200]={0};  
                            uint8_t Buffer2_ok[200]={0};  
                            uint8_t bcc_temp=0;
                            switch (bl_addr)
                            {




//-------------------admin--------------guimenshezhi-----------2次------------------------------------------
                            case 0x11A0://
                                DB_PR("--kajishezhi--.\r\n");   
                                int j=0;
                                for (int i = 7; i < 7+ data_rx_t[6] *2 ; i++) {
                                    DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                    tx_Buffer2[j]=data_rx_t[i];
                                    // uart0_debug_data( (const char *) tx_Buffer, 3+ data_len);
                                    // if(tx_Buffer2[j] != 0x2D)
                                    j++;
                                }
                                DB_PR("\r\n");
                                

                                if(data_rx_t[2] == 0x6A)//0x9c
                                {
                                    send_cmd_to_lcd_bl_len(0x11A0,(uint8_t*)buff_t,30*4+5);//
                                    // send_cmd_to_lcd_bl(0x11A0,0);
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;
                                    while( NULL != (p = strchr(p,'-')))    
                                    {
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




                                    
                                    //shengyu_all_max =0 ;
                                    // nvs_wr_mima_number_adm(1);
                                    // for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
                                    // {
                                    //     database_cw.dIndx = i;

                                    //     database_gz[i].dIndx_gz =0;
                                    //     database_gz[i].state_fenpei_gz =0;
                                    //     // database_gz[i].dzx_mode_gz =3;
                                    //     // database_gz[i].state_gz =0;
                                        
                                    //     nvs_wr_index_gz(1);
                                    //     nvs_wr_fenpei_gz(1);
                                    //     // nvs_wr_dzx_mode_gz(1);
                                    //     // nvs_wr_state_gz(1);

                                    //     //lock----------------todo---------------------
                                    //     //changqi
                                    // }
                                    
                                    uint16_t shengyu_all_max_temp=0;//shengyu max admin, guding
                                    int16_t guimen_x_gk_max_temp[BOARD_GK_MAX]={0};

                                    DB_PR("hang_shu_max+1=%03d\r\n",hang_shu_max+1);//0
                                    if(hang_shu_max<=BOARD_GK_MAX)
                                    {
                                        DB_PR("show2=");
                                        for(j = 0; j <= hang_shu_max; j++)//i 个柜子
                                        {

                                            guimen_x_gk_max_temp[j] = (int16_t)atoi((const char*)show[j]);
                                            if(guimen_x_gk_max_temp[j] >=24)
                                                guimen_x_gk_max_temp[j] =24;
                                            DB_PR("guimen_x_gk_max_temp[j] = %03d\r\n\r\n",guimen_x_gk_max_temp[j]);

                                            for(int k=1; k<= guimen_x_gk_max_temp[j]; k++)//列
                                            {
                                                database_gz[j*24 + k].state_fenpei_gz = 1;
                                                DB_PR("-1-lock index=%03d\r\n",j*24 + k);
                                            }
                                            if(guimen_x_gk_max_temp[j] <24)
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

                                        for(j = hang_shu_max+1; j <= BOARD_GK_MAX; j++)//i 个柜子
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

                                        DB_PR("\r\n");


                                        DB_PR("1-------shengyu_all_max_temp=%03d\r\n",shengyu_all_max_temp);
                                        if(shengyu_all_max_temp>0)
                                        {
                                            DB_PR("2-------shengyu_all_max_temp=%03d\r\n",shengyu_all_max_temp);
                                            shengyu_all_max = shengyu_all_max_temp;
                                            memcpy(guimen_x_gk_max,guimen_x_gk_max_temp,BOARD_GK_MAX);//????????

                                            DB_PR("2-hang_shu_max=%03d\r\n",hang_shu_max);
                                            uart0_debug_data_d(guimen_x_gk_max,BOARD_GK_MAX);
                                            DB_PR("3-hang_shu_max=%03d\r\n",hang_shu_max);
                                            uart0_debug_data_d(guimen_x_gk_max_temp,BOARD_GK_MAX);


                                            // nvs_wr_shengyu_all_max(0);//1
                                            // DB_PR("-1-shengyu_all_max=%d----\n",shengyu_all_max);
                                            
                                            // if((shengyu_all_max == 0) )
                                            // {
                                            //     //shengyu_all_max
                                            //     memcpy(guimen_x_gk_max,guimen_x_gk_max_temp,BOARD_GK_MAX);
                                            //     shengyu_xiao_max =shengyu_all_max;
                                            //     shengyu_zhong_max=0;
                                            //     shengyu_da_max =0;

                                            //     shengyu_all = shengyu_all_max;
                                            //     shengyu_xiao = shengyu_xiao_max;
                                            //     shengyu_zhong = shengyu_zhong_max;
                                            //     shengyu_da = shengyu_da_max;

                                            // }
                                            // else
                                            // {
                                            //     /* todo    add del */
                                            //     for(i = 0; i <= hang_shu_max; i++)//i 个柜子
                                            //     {
                                            //         if(guimen_x_gk_max_temp[i]>guimen_x_gk_max[i])
                                            //         {
                                            //             guimen_x_gk_max[i]

                                            //             if(1==database_gz[i].dzx_mode_gz)
                                            //             {

                                            //             }
                                            //         }
                                            //         // else if(guimen_x_gk_max_temp[i]<guimen_x_gk_max[i])
                                            //         // {
                                            //         //     /* code */
                                            //         // }
                                                    
                                            //     }
                                            // }
                                            






                                            u16 buff_temp1[300]={0};
                                            u16 buff_temp2[300]={0};

                                            u8 buff_temp1_c[400]={0};//char
                                            u8 buff_temp2_c[400]={0};//150
                                            uint16_t id=0,iz=0,ix=0;
                                            uint16_t id_max=0,iz_max=0,ix_max=0;
                                            uint16_t changqi_num_temp=0;
                                            j=0;
                                            for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
                                            {
                                                database_cw.dIndx = i;
                                                // DB_PR("fenpei?=%d, ", database_gz[i].state_fenpei_gz);
                                                nvs_wr_fenpei_gz(1);
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
                                                else
                                                {
                                                    database_gz[i].dIndx_gz = 0;
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

                                        }
                                        //save

                                        //memset(tx_Buffer2,0,200);
                                        //send_cmd_to_lcd_bl_len(BL_XM_SZ,tx_Buffer2,data_rx_t[2]-1);//clear
                                        send_cmd_to_lcd_pic(GUIMEN_OK_PIC);
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





                            //-------------格口设置 dazhongxiao
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
                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_D,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;
                                    while( NULL != (p = strchr(p,'-')))    
                                    {
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

                                    DB_PR("hang_shu_max=%03d\r\n",hang_shu_max);
                                    if((hang_shu_max<= 0x96))//
                                    {
                                        DB_PR("show2=");
                                        for(j = 0; j <= hang_shu_max; j++)//i 个柜子
                                        {

                                            guimen_gk_temp = atoi((const char*)show[j]);
                                            DB_PR("-------guimen_gk_temp--dIndx=%d--.\r\n",guimen_gk_temp); 


                                            if((guimen_gk_temp >=SHENYU_GEZI_MAX)||(0==guimen_gk_temp))
                                            {
                                                DB_PR("--gekou set fail--.\r\n");   
                                                goto gekou_fail;

                                            }
                                            database_cw.dIndx =0;
                                            database_cw.dIndx = find_lock_index(guimen_gk_temp);

                                            DB_PR("---2----lock--dIndx=%d--.\r\n",database_cw.dIndx); 
                                            if(database_cw.dIndx ==0)
                                            {
                                                DB_PR("--gekou set fail2--.\r\n");   
                                                goto gekou_fail;
                                            }
                                            

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
                                                shengyu_da ++;
                                                shengyu_zhong--;
                                                //shengyu_xiao --;
                                                break;
                                            case 3:
                                                shengyu_da_max++;
                                                //shengyu_zhong_max--;
                                                shengyu_xiao_max--;
                                                shengyu_da ++;
                                                //shengyu_zhong--;
                                                shengyu_xiao --;
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
                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_D,tx_Buffer2,data_rx_t[2]-1);//clear


                                    send_cmd_to_lcd_pic(GEKOU_Z_PIC);

                                    
                                }
                                else
                                {
//da
gekou_fail:
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
                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_Z,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;
                                    while( NULL != (p = strchr(p,'-')))    
                                    {
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


                                            if((guimen_gk_temp >=SHENYU_GEZI_MAX)||(0==guimen_gk_temp))
                                            {
                                                DB_PR("--gekou set fail--.\r\n");   
                                                goto gekou_fail_z;

                                            }
                                            database_cw.dIndx =0;
                                            database_cw.dIndx = find_lock_index(guimen_gk_temp);

                                            DB_PR("---2----lock--dIndx=%d--.\r\n",database_cw.dIndx); 
                                            if(database_cw.dIndx ==0)
                                            {
                                                DB_PR("--gekou set fail2--.\r\n");   
                                                goto gekou_fail_z;
                                            }
                                            

                                            switch(database_gz[database_cw.dIndx].dzx_mode_gz)
                                            {
                                            case 1:
                                                // no gai
                                                shengyu_da_max--;
                                                shengyu_zhong_max++;
                                                //shengyu_xiao_max--;
                                                shengyu_da --;
                                                shengyu_zhong++;
                                                //shengyu_xiao --;

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

                                                shengyu_zhong ++;
                                                //shengyu_zhong--;
                                                shengyu_xiao --;
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
                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_Z,tx_Buffer2,data_rx_t[2]-1);//clear


                                    send_cmd_to_lcd_pic(GEKOU_X_PIC);

                                    
                                }
                                else
                                {
//z
gekou_fail_z:
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
                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_X,(uint8_t*)buff_t,BL_GK_BH_MAX_LEN);//
                                    char show[156][10];
                                    char *p = NULL;
                                    char *q = NULL;
                                    int i = 0;
                                    // int j;
                                    // DB_PR("请输入字符串shou:");
                                    // gets(shou);
                                    p = q = (char*)tx_Buffer2;
                                    while( NULL != (p = strchr(p,'-')))    
                                    {
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


                                            if((guimen_gk_temp >=SHENYU_GEZI_MAX)||(0==guimen_gk_temp))
                                            {
                                                DB_PR("--gekou set fail--.\r\n");   
                                                goto gekou_fail_x;

                                            }
                                            database_cw.dIndx =0;
                                            database_cw.dIndx = find_lock_index(guimen_gk_temp);

                                            DB_PR("---2----lock--dIndx=%d--.\r\n",database_cw.dIndx); 
                                            if(database_cw.dIndx ==0)
                                            {
                                                DB_PR("--gekou set fail2--.\r\n");   
                                                goto gekou_fail_x;
                                            }
                                            

                                            switch(database_gz[database_cw.dIndx].dzx_mode_gz)
                                            {
                                            case 1:
                                                // no gai
                                                shengyu_da_max--;
                                                //shengyu_zhong_max++;
                                                shengyu_xiao_max++;
                                                shengyu_da --;
                                                //shengyu_zhong++;
                                                shengyu_xiao ++;

                                                break;
                                            case 2:
                                                //shengyu_da_max++;
                                                shengyu_zhong_max--;
                                                shengyu_xiao_max++;
                                                //shengyu_da ++;
                                                shengyu_zhong--;
                                                shengyu_xiao ++;
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


                                        //tongbu_zh();
                                        
                                    }


                                    //save

                                    memset(tx_Buffer2,0,200);
                                    send_cmd_to_lcd_bl_len(BL_GK_SZ_X,tx_Buffer2,data_rx_t[2]-1);//clear


                                    send_cmd_to_lcd_pic(GEKOU_OK_PIC);

                                    
                                }
                                else
                                {
//x
gekou_fail_x:
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

                           case 0x1130://
                                DB_PR("--qingxiang--.\r\n");   
                                send_cmd_to_lcd_bl_len(0x1130,(uint8_t*)buff_t,2*2+5);//
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
                                
                                
                                uint8_t temp_xiangmen[4]={0}; 
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


                                    if(database_gz[database_cw.dIndx].cunwu_mode_gz ==1)
                                    {
                                        Del_FR(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                                        //del_zw_database(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                                        
                                    }
                                    
 
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
                                    send_cmd_to_lcd_bl(0x1130,database_gz[database_cw.dIndx].dIndx_gz);
            
                                    //send_cmd_to_lcd_pic(0x0024);
                                    send_cmd_to_lcd_pic(CLEAR_ONE_OK_PIC);



                                    DB_PR("-----2-----[ * ] Starting audio pipeline");
                                    // audio_pipeline_stop(pipeline);
                                    // audio_pipeline_wait_for_stop(pipeline);
                                    // audio_pipeline_terminate(pipeline);
                                    // audio_pipeline_reset_ringbuffer(pipeline);
                                    // audio_pipeline_reset_elements(pipeline);

                                    // // set_next_file_marker((int)(database_cw.dIndx));
                                    // //set_next_file_marker();
                                    // //static int idx = 0;

                                    // //static int midx = 0;
                                    // DB_PR( "--sound-- index = %d", database_cw.dIndx);
                                    // switch (1) {
                                    //    case 0:
                                    //         m_mp3_start = lr_mp3_start;
                                    //         m_mp3_end   = lr_mp3_end;
                                    //         break;

                                    //     case 1:
                                    //         m_mp3_start = adf_music_mp3_start;
                                    //         m_mp3_end   = adf_music_mp3_end;
                                    //         break;

  
                                    //     case 2:
                                    //         m_mp3_start = mr_mp3_start;
                                    //         m_mp3_end   = mr_mp3_end;
                                    //         break;
                                    //     case 3:
                                    //         m_mp3_start = hr_mp3_start;
                                    //         m_mp3_end   = hr_mp3_end;
                                    //         break;
  
                                    //     default:
                                    //         m_mp3_start = adf_music_mp3_start;
                                    //         m_mp3_end   = adf_music_mp3_end;
                                    //         DB_PR( "[ * ] Not supported index = %d", database_cw.dIndx);
                                    // }
                                    // audio_pipeline_run(pipeline);






                                    if(lock_flag == 0)
                                    {
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

                                    }
                                    DB_PR( "database_gz[database_cw.dIndx].state_gz = %d", database_gz[database_cw.dIndx].state_gz);
                                    
                                    

                                    //if(0 != database_gz[database_cw.dIndx].state_gz)
                                    {

                                        // char key_name[15];//15
                                        // esp_err_t err;


         


                                        if(database_gz[database_cw.dIndx].cunwu_mode_gz ==1)
                                        {
                                            Del_FR(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                                            //del_zw_database(database_gz[database_cw.dIndx].zhiwen_page_id_gz);

                                            database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id] =0;
                                            nvs_wr_adm_zwpageid_flag(1,database_cw.zhiwen_page_id);
                                            database_gz[database_cw.dIndx].zhiwen_page_id_gz = 0;
                                            nvs_wr_zw_pageid_gz(1);
                                            
                                        }
                                        else  if(database_gz[database_cw.dIndx].cunwu_mode_gz ==2)
                                        {
                                            database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                            database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                            nvs_wr_phone_number_nvs_gz(1);
                                            nvs_wr_mima_number_nvs_gz(1);
                                        }

                                        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                        //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
                                        // database_gz[database_cw.dIndx].phone_number_nvs_gz = 0;
                                        // database_gz[database_cw.dIndx].mima_number_nvs_gz = 0;
                                        database_gz[database_cw.dIndx].state_gz =0;
                                        database_gz[database_cw.dIndx].changqi =0;
                                       
                                        //nvs_wr_dzx_mode_gz(1);


                                        nvs_wr_cunwu_mode_gz(1);
                                        nvs_wr_state_gz(1);
                                        nvs_wr_glongtime_gz(1);

                                        //update xianshi todo
                                        tongbu_changqi();

                                    }

                                }
                                else
                                {
wuci_xmh_q:
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
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

                                        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                        //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
  
                                        //database_gz[database_cw.dIndx].state_gz =0;

                                        if(database_gz[database_cw.dIndx].changqi !=0)
                                            database_gz[database_cw.dIndx].changqi =2;
                                        else
                                            database_gz[database_cw.dIndx].changqi =0;

                                        database_gz[database_cw.dIndx].lock =1;
                                        nvs_wr_cunwu_mode_gz(1);
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
                                    //if(database_gz[database_cw.dIndx].changqi == 1)
                                    {
                                        //update xianshi todo
                                        tongbu_changqi();
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
                                        goto wuci_xmh_unlk;
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



                                    DB_PR("-----2-----[ * ] Starting audio pipeline");


                                    DB_PR( "database_gz[database_cw.dIndx].state_gz = %d", database_gz[database_cw.dIndx].state_gz);
                                    DB_PR( "database_gz[database_cw.dIndx].dIndx_gz = %d", database_gz[database_cw.dIndx].dIndx_gz);
                                    if((2!=database_gz[database_cw.dIndx].changqi)
                                        &&((0 == database_gz[database_cw.dIndx].state_gz)
                                       ||(0==database_gz[database_cw.dIndx].lock)))
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

                                        database_gz[database_cw.dIndx].cunwu_mode_gz = 0;
                                        //database_gz[database_cw.dIndx].dzx_mode_gz = 0;
  
                                        //database_gz[database_cw.dIndx].state_gz =0;

                                       
                                        database_gz[database_cw.dIndx].lock =0;
                                        nvs_wr_cunwu_mode_gz(1);
                                        //nvs_wr_dzx_mode_gz(1);
         
                                        //nvs_wr_state_gz(1);

                                        if(2==database_gz[database_cw.dIndx].changqi)
                                        {
                                            database_gz[database_cw.dIndx].changqi =1;
                                        }
                                        else
                                        {
                                            database_gz[database_cw.dIndx].changqi =0;
                                        }

                                        nvs_wr_glongtime_gz(1);
                                        nvs_wr_glock_gz(1);

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





                            case 0x2050://zhiwen  or   mima
                                DB_PR("--a-zhiwen or mima---.\r\n");   
                                //if -> huise tupian?
                                if(01== data_rx_t[8])//zhiwen cun
                                {
                                    DB_PR("--zhiwen--.\r\n");  
                                    //baocun 1
                                    database_cw_adm.cunwu_mode = 1;
									send_cmd_to_lcd_pic(0x001b);
                                    xTaskCreate(Add_FR_CQ, "add_zhiwen_task", 6* 1024, NULL, 2, NULL);//1024 10
                                }
                                else if(02== data_rx_t[8])//mi ma
                                {
                                    DB_PR("--mima--.\r\n");  
                                    //baocun 2
									send_cmd_to_lcd_pic(0x001d);
                                    database_cw_adm.cunwu_mode = 2;
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
                                DB_PR("-a-password--.\r\n");
                                //DB_PR("---phone_weishu_ok=%d---.\r\n",phone_weishu_ok);
                                send_cmd_to_lcd_bl_len(0x1100,(uint8_t*)buff_t,12+5);//phone
                                send_cmd_to_lcd_bl_len(0x1110,(uint8_t*)buff_t,6+5);//key


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

                                    if(0==memcmp(mima_number,buff_t,6))
                                    {
                                        DB_PR("------mima all=0-------.\r\n");
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


                                }
                                else
                                {
                                    if(03!= data_rx_t[6])
                                    {
                                      DB_PR("----------------2 - mima weishu err---------------.\r\n");  
                                    }
done_longtime:
                                    DB_PR("----test2-error--.\r\n");  
                                    send_cmd_to_lcd_pic(0x0007);
                                }
done_longtime_2:
                                DB_PR("----test3-done--.\r\n");  

                                send_cmd_to_lcd_bl_len(0x1100,(uint8_t*)buff_t,30+5);//phone
                                send_cmd_to_lcd_bl_len(0x1110,(uint8_t*)buff_t,30+5);//key
                                // send_cmd_to_lcd_bl(0x1100,0);//phone
                                // send_cmd_to_lcd_bl(0x1110,0);//key
                                
                                database_cw.cunwu_mode =0;
                                database_cw.dzx_mode = 0 ;


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

                                        if(database_gz[database_cw.dIndx].cunwu_mode_gz ==1)
                                        {
                                            Del_FR(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                                            //del_zw_database(database_gz[database_cw.dIndx].zhiwen_page_id_gz);

                                            database_ad.zhiwen_page_id_adm[database_cw.zhiwen_page_id] =0;
                                            nvs_wr_adm_zwpageid_flag(1,database_cw.zhiwen_page_id);
                                            database_gz[database_cw.dIndx].zhiwen_page_id_gz = 0;
                                            nvs_wr_zw_pageid_gz(1);
                                            
                                        }
                                        else  if(database_gz[database_cw.dIndx].cunwu_mode_gz ==2)
                                        {
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



                                    for (int i = 7; i < 7+ data_rx_t[6] *2 -2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            mima_weishu_ok_a1 =0;
                                            DB_PR("--no--mima_weishu_ok_a1=%d---.\r\n",mima_weishu_ok_a1);
                                        }
                                    }
                                    DB_PR("\r\n");

                                    if(mima_weishu_ok_a1 == 1)
                                        DB_PR("-yes-mima_weishu_ok_a1=%d---.\r\n",mima_weishu_ok_a1);

                                }
                                else
                                {
                                    DB_PR("-------1 - mima weishu err--------.\r\n");
                                }
                                
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

                                    DB_PR("mima_number1=");
                                    uart0_debug_str(mima_number_a1,6);

                                    DB_PR("mima_number2=");
                                    uart0_debug_str(mima_number_a2,6);



                                    for (int i = 7; i < 7+ data_rx_t[6] *2 -2 ; i++) {
                                        DB_PR("0x%.2X ", (uint8_t)data_rx_t[i]);
                                        if(data_rx_t[i] == 0xFF)
                                        {
                                            DB_PR("--no--mima_weishu_ok---.\r\n");
                                            goto done_mima_nosame;
                                        }
                                    }
                                    DB_PR("\r\n");


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
                                send_cmd_to_lcd_bl_len(0x11e0,(uint8_t*)buff_t,30+5);//key
                                send_cmd_to_lcd_bl_len(0x11f0,(uint8_t*)buff_t,30+5);//key2

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
                                    DB_PR("--yuyin off--.\r\n");
                                    send_cmd_to_lcd_pic(MUSIC_OFF_PIC);//default
                                }
                                if(01== data_rx_t[8])
                                {
                                    DB_PR("--yuyin on--.\r\n");
                                    send_cmd_to_lcd_pic(MUSIC_ON_PIC);
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
                                        xTaskCreate(Add_FR, "add_zhiwen_task", 6* 1024, NULL, 2, NULL);//1024 10
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
                                if(return_cause_zanwu_kx == 3)
                                {
                                    DB_PR("--2 cunwu--.\r\n");  
                                    //baocun 1
                                    send_cmd_to_lcd_pic(0x0002);
                                }
                                else if(return_cause_zanwu_kx == 4)
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

                            case 0x1260://zhiwen  close
                                DB_PR("---zhiwen close---.\r\n");   
                                //if -> huise tupian?

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

                                break;



                            case 0x2020://da zhong xiao    ke sheng
                                DB_PR("----------------daxiao---------------.\r\n");   

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
                                        xTaskCreate(Add_FR, "add_zhiwen_task", 6* 1024, NULL, 2, NULL);//1024 10
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

                                send_cmd_to_lcd_bl_len(0x1050,(uint8_t*)buff_t,12+5);//phone
                                send_cmd_to_lcd_bl_len(0x1060,(uint8_t*)buff_t,6+5);//key

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
                                            // send_cmd_to_lcd_pic(0x001F);
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
                                                send_cmd_to_lcd_pic(0x001F);
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
                                            database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

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
                                            database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

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
                                            database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用



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
                                

                                send_cmd_to_lcd_bl_len(0x1050,(uint8_t*)buff_t,12+5);//phone
                                send_cmd_to_lcd_bl_len(0x1060,(uint8_t*)buff_t,6+5);//key
                                
                                database_cw.cunwu_mode =0;
                                database_cw.dzx_mode = 0 ;
                                database_cw.state=0;

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
                                send_cmd_to_lcd_bl_len(0x1080,(uint8_t*)buff_t,30+5);//phone
                                send_cmd_to_lcd_bl_len(0x1090,(uint8_t*)buff_t,30+5);//key
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
                                    if(database_cw.dIndx == 0)//
                                    {
                                        send_cmd_to_lcd_pic(0x000d); //
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



                                            if(database_gz[database_cw.dIndx].changqi == 0)
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
                                    if(03!= data_rx_t[6])
                                    {
                                      DB_PR("----------------2 - mima weishu err---------------.\r\n");  
                                    }
done_qu:
                                    DB_PR("----test2-error--.\r\n");  

                                    send_cmd_to_lcd_pic(0x000b); //

                                }
                                DB_PR("----test3-done--.\r\n");  

                                
                                // send_cmd_to_lcd_bl(0x1080,0);//phone
                                // send_cmd_to_lcd_bl(0x1090,0);//key
                                send_cmd_to_lcd_bl_len(0x1080,(uint8_t*)buff_t,30+5);//phone
                                send_cmd_to_lcd_bl_len(0x1090,(uint8_t*)buff_t,30+5);//key

                                database_cw.cunwu_mode =0;
                                database_cw.dzx_mode = 0 ;
                                database_cw.state=0;

                                
                                break;




//---------------------------------admin----------------------------------------------------
                            case 0x10b0://mima
                                DB_PR("----admin --mima-----.\r\n");

                                //mima 666888 todo----------------

                                if(03== data_rx_t[6])
                                //if(1)
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


                            case 0x10c0://xiangmenhao   kaixiang
                                send_cmd_to_lcd_bl_len(0x10c0,(uint8_t*)buff_t,2*2+5);//key
                                
                                DB_PR("----admin --mima-----.\r\n");
                                //uint8_t temp_xiangmen[4]={0}; 
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
  
 
                                    if(database_gz[database_cw.dIndx].state_fenpei_gz == 0)
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



                                    DB_PR("-da-lock:%d ok--.\r\n",j);
                                    send_cmd_to_lock(k+1,j);
                                    send_cmd_to_lcd_bl(0x10c0,database_gz[database_cw.dIndx].dIndx_gz);
            
                                    send_cmd_to_lcd_pic(0x0014);




                                    DB_PR("-----2-----[ * ] Starting audio pipeline");

                                    audio_pipeline_stop(pipeline);
                                    audio_pipeline_wait_for_stop(pipeline);
                                    audio_pipeline_terminate(pipeline);
                                    audio_pipeline_reset_ringbuffer(pipeline);
                                    audio_pipeline_reset_elements(pipeline);

                                    //set_next_file_marker();
                                    DB_PR("[2.6-b] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)");
                                    audio_element_set_uri(tone_stream_reader, tone_uri[database_gz[database_cw.dIndx].dIndx_gz]);
                                    audio_pipeline_run(pipeline);

                                    delay_ms(5000);
                                    audio_pipeline_stop(pipeline);
                                    audio_pipeline_wait_for_stop(pipeline);
                                    audio_pipeline_terminate(pipeline);
                                    audio_pipeline_reset_ringbuffer(pipeline);
                                    audio_pipeline_reset_elements(pipeline);

                                    //set_next_file_marker();
                                    DB_PR("[2.6-b] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)");
                                    audio_element_set_uri(tone_stream_reader, tone_uri[1]);
                                    audio_pipeline_run(pipeline);

                                }
                                else
                                {
wuci_xmh:
                                    send_cmd_to_lcd_pic(FAIL_XMH_PIC);
                                    return_cause = 1;
                                    DB_PR("----admin --wu ci xiangmenhao-----.\r\n");
                                }
                                //send_cmd_to_lcd_bl(0x10C0,database_gz[database_cw.dIndx].dIndx_gz);//xiangmen------------
                                
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
	DB_PR(" %x: ", byt);
	switch(uState)
	{
		case UART_IDLE:
		case UART_HEADER:
			if(byt == 0x5A)
			{
				uState = UART_HEADER2;
				//user_start_cmd_receive(true);
			}
			//DB_PR("header = %02x\r\n", byt);
			break;

		case UART_HEADER2:
			if(byt == 0xA5)
			{
				uState = UART_LENGTH;
				//user_start_cmd_receive(true);
			}
			//DB_PR("header = %02x\r\n", byt);
			break;

		case UART_LENGTH:
			g_data.len = byt;//-3   no jiaoyan len
			g_data.dIndx = 0;
			
			uState++;
			//DB_PR("length = %02x\r\n", byt);
			break;
		case UART_OPCODE:
			g_data.opcode = byt;
			//g_data.sum += byt;
            //g_data.sum = byt;
			uState++;
			//DB_PR("opcode = %02x\r\n", byt);
			break;
		case UART_PAYLOAD:



            if(g_data.dIndx == g_data.len-3 )//crc1   2
            {
                g_data.data[g_data.dIndx] = byt;
                //DB_PR("crc data[%d] = %02x\r\n",g_data.dIndx, byt);
                g_data.dIndx++;
            }
			else if(g_data.dIndx < g_data.len-3 )//<   1
			{
				g_data.data[g_data.dIndx] = byt;
				//g_data.sum += byt;
				//DB_PR("data[%d] = %02x\r\n",g_data.dIndx, byt);
                g_data.dIndx++;
			}
			else//crc2
			{
                g_data.data[g_data.dIndx] = byt;
                // DB_PR("crc data[%d] = %02x\r\n",g_data.dIndx, byt);
                g_data.dIndx++;
				//user_start_cmd_receive(false);
				uState = 0;
                data_t[0] = g_data.opcode;
                memcpy(data_t+1,g_data.data,g_data.len -3);

                g_data.sum = CRC16(data_t, g_data.len -3 +1);
				// DB_PR("sum = %04x\r\n",g_data.sum);

                // DB_PR("g_data.data[g_data.dIndx -2] = %02x, g_data.data[g_data.dIndx-1] = %02x\r\n",g_data.data[g_data.dIndx -2], g_data.data[g_data.dIndx-1]);
                // DB_PR("g_data.sum &0xff = %02x, (g_data.sum>>8)&0xff = %02x\r\n",g_data.sum &0xff, (g_data.sum>>8)&0xff);


				if( ((g_data.sum &0xff) == g_data.data[g_data.dIndx -2])
                    &&(((g_data.sum>>8)&0xff) == g_data.data[g_data.dIndx-1]) )//byt
				{
                    DB_PR("处理数据1-pass\r\n");
					r = true;
				}
				else
				{
                    DB_PR("处理数据1-fail\r\n");
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
    for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    {
        //vTaskDelay(1);
        database_cw.dIndx =i;
        nvs_wr_fenpei_gz(0);//2  _dz_fp
       
        if(1== database_gz[i].state_fenpei_gz)
        {

            if(0== database_gz[database_cw.dIndx].lock)
            {
                // if((0 == database_gz[database_cw.dIndx].state_gz)
                //     ||(0 == database_gz[database_cw.dIndx].changqi))
                {
                    if(database_gz[database_cw.dIndx].cunwu_mode_gz ==1)
                    {
                        Del_FR(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                        //del_zw_database(database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                        //if(database_ad.zhiwen_page_id_adm[database_gz[database_cw.dIndx].zhiwen_page_id_gz]==1)//AS608Para.PS_max   
                        {
                            nvs_wr_adm_zwpageid_flag(1,database_gz[database_cw.dIndx].zhiwen_page_id_gz);
                        }
                    }

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
                    
                }


            }


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

    shengyu_all = j+k+l;


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



static void echo_task3()//zhiwen
{
    DB_PR("--echo_task3-- \n");


    vTaskDelay(1);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
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
        len_rx = uart_read_bytes(UART_NUM_1, data_rx, BUF_SIZE, 20 / portTICK_RATE_MS);
        len_rx2 = uart_read_bytes(UART_NUM_2, data_rx2, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        
        //uart_write_bytes(UART_NUM_1, (const char *) data_rx, len_rx);//len =0 budayin
        //uart_write_bytes(UART_NUM_2, (const char *) data_rx, len_rx);


        if ((len_rx2 > 0) ) {
            
            len_rx2_m = len_rx2;
            memcpy(data_rx2_m,data_rx2,len_rx2_m);
            DB_PR("2rcv_zhiwen_uart2-Received %u bytes:", len_rx2_m);
            for (int i = 0; i < len_rx2_m; i++) {
                DB_PR("0x%.2X ", (uint8_t)data_rx2_m[i]);
            }
            DB_PR("] \n");
            
            flag_rx2 =1;
            DB_PR("-----2----flag_rx2=%u\r\n", flag_rx2);
            

            xTaskCreate(echo_task3, "uart_echo_task2",2* 1024, NULL, 2, NULL);//uart2
            //uart_write_bytes(UART_NUM_2, (const char *) data_rx2, len_rx2);
        }

								
		


        if (len_rx > 0) {

            DB_PR("1rcv_lcd_uart1-Received %u bytes:", len_rx);
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
            xTaskCreate(echo_task2, "uart_echo_task2",8* 1024, NULL, 2, NULL);//uart1

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
//                 //ensure = PS_Search(0x02, 0x0000, 0x00AA, p_rsp);
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

//录指纹
void Add_FR()
{
	u8 i=0,ensure=0 ,processnum=0;
    // u8 ensure_2=0,ensure_3=0;
    SearchResult p_rsp;
	//u16 ID;
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
                        ensure = PS_Search(CharBuffer1, 0x0000, 0x00AA, &p_rsp);//0x02
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
                        }
                        else 
                        {
                            
                            DB_PR("--0-ok2-对比成功,新的指纹\r\n");
                            i=0;//del?
                            processnum=1;//跳到第二步	

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
                    ShowErrMessage(ensure);	
                }
                    					
				break;
			
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
                        ensure = PS_Search(CharBuffer2, 0x0000, 0x00AA, &p_rsp);//0x02
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--1-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=0;//跳回第一步	
                            	
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
                        ensure = PS_Search(CharBuffer3, 0x0000, 0x00AA, &p_rsp);//0x02
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--2-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=0;//跳回第一步	
                            	
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
				}else {processnum=0;ShowErrMessage(ensure);}
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




                //AS608Para.PS_max-ValidN
                uint16_t j =0, k=0;
                int rand_temp=0;
                for(uint16_t i=0;i<AS608Para.PS_max;i++)//
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
                                database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

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
                                database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

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
                                database_cw.dIndx = database_gz_temp[rand()%j];//随机获取哪个门没用

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
                        database_cw.dzx_mode = 0 ;
                        database_cw.state=0;




                        //send_cmd_to_lcd_pic(0x0008);
                        //LCD_Fill(0,100,240,160,WHITE);
                        //return ;
                        vTaskDelete(NULL);

                        
                    }
                    else 
                    {
done_zwc_fail:
                        processnum=0;
                        DB_PR("---done_zwc_fail----\r\n");
                        database_cw.cunwu_mode =0;
                        database_cw.dzx_mode = 0 ;
                        database_cw.state=0;
                        send_cmd_to_lcd_pic(0x0001);
                        return_cause_zanwu_kx =8;
                        ShowErrMessage(ensure);
                    }		

                }			
				break;	

            default:
                DB_PR("--default-zhiwen add ");
                break;		
		}
		delay_ms(400);
		if(i==5)//超过5次没有按手指则退出
		{
            DB_PR("---->5a---- 超过5次没有按手指则退出 ");
			//LCD_Fill(0,100,lcddev.width,160,WHITE);
			break;	
		}				
	}
    vTaskDelay(1);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);

}


//录指纹
void Add_FR_CQ()
{
	u8 i=0,ensure=0 ,processnum=0;
    // u8 ensure_2=0,ensure_3=0;
    SearchResult p_rsp;
	//u16 ID;
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
                        ensure = PS_Search(CharBuffer1, 0x0000, 0x00AA, &p_rsp);//0x02
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
                        }
                        else 
                        {
                            
                            DB_PR("--0-ok2-对比成功,新的指纹\r\n");
                            i=0;//del?
                            processnum=1;//跳到第二步	

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
                    ShowErrMessage(ensure);	
                }
                    					
				break;
			
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
                        ensure = PS_Search(CharBuffer2, 0x0000, 0x00AA, &p_rsp);//0x02
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--1-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=0;//跳回第一步	
                            	
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
                        ensure = PS_Search(CharBuffer3, 0x0000, 0x00AA, &p_rsp);//0x02
                        if(ensure==0x00)
                        {
                            ////LCD_Fill(0,120,lcddev.width,160,WHITE);
                            
                            DB_PR("--2-no-对比完成,指纹已存在 \r\n");
                            send_cmd_to_lcd_pic(0x0005);
                            ShowErrMessage(ensure);	
                            i=0;
                            processnum=0;//跳回第一步	
                            	
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
				}else {processnum=0;ShowErrMessage(ensure);}
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




                //AS608Para.PS_max-ValidN
                uint16_t j =0, k=0;
                int rand_temp=0;
                for(uint16_t i=0;i<AS608Para.PS_max;i++)//
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
                    processnum=0;
                    DB_PR("---done_zwc_fail_cq2----\r\n");
                    database_cw_adm.cunwu_mode =0;
                    database_cw_adm.dzx_mode = 0 ;
                    database_cw_adm.state=0;
                    send_cmd_to_lcd_pic(0x0001);
                    return_cause_zanwu_kx =9;
                    ShowErrMessage(ensure);
                }
                			
				break;	

            default:
                DB_PR("--default-zhiwen add ");
                break;		
		}
		delay_ms(400);
		if(i==5)//超过5次没有按手指则退出
		{
            DB_PR("---->5a---- 超过5次没有按手指则退出 ");
			//LCD_Fill(0,100,lcddev.width,160,WHITE);
			break;	
		}				
	}
    vTaskDelay(1);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);

}


//刷指纹
void press_FR(void)
{
	for(u8 i=0;i<3;i++)
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
                    if(database_gz[database_cw.dIndx].changqi == 0)
                    {
                        Del_FR(seach.pageID);
                        DB_PR("-----changqi cunwu \r\n");	
                    }
                    else
                    {
                        DB_PR("-----normal cunwu \r\n");			
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
            break;//add
        }
        
    }
    vTaskDelay(1);
    vTaskDelete(NULL);
		
}

//删除指纹
void Del_FR(u16 num)
{
	u8  ensure;
	//u16 num;
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	DB_PR("删除指纹 \r\n");
	DB_PR("请输入指纹ID按Enter发送 \r\n");
	DB_PR("0=< ID <=299 \r\n");

    DB_PR("1*********pageid hex**********num=%x \r\n",num);
    DB_PR("2*********pageid dec**********num=%d \r\n",num);
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
    DB_PR("-------database_ad.zhiwen_page_id_adm[num]=%d\r\n",database_ad.zhiwen_page_id_adm[num]);
    if(database_ad.zhiwen_page_id_adm[num] ==1)
    //if(1)
    {

        // database_cw.dIndx = find_pid_lock_idx(num);

        if(database_cw.dIndx == 0)
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



                if(database_gz[database_cw.dIndx].changqi == 0)
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




void audio_init(void)
{


    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    DB_PR("[ 1 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    DB_PR("[2.0] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, pipeline, return);

    DB_PR("[2.1] Create tone stream to read data from flash");
    tone_stream_cfg_t tone_cfg = TONE_STREAM_CFG_DEFAULT();
    tone_cfg.type = AUDIO_STREAM_READER;
    tone_stream_reader = tone_stream_init(&tone_cfg);

    DB_PR("[2.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    DB_PR("[2.3] Create mp3 decoder to decode mp3 file");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);

    DB_PR("[2.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, tone_stream_reader, "tone");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    DB_PR("[2.5] Link it together [flash]-->tone_stream-->mp3_decoder-->i2s_stream-->[codec_chip]");
    const char *link_tag[3] = {"tone", "mp3", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    DB_PR("[2.6] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)");
    audio_element_set_uri(tone_stream_reader, tone_uri[TONE_TYPE_HELLO]);

    DB_PR("[ 3 ] Set up event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    DB_PR("[3.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    DB_PR("[ 4 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    DB_PR("[ 4 ] Listen for all pipeline events");
    while (1) {
        audio_event_iface_msg_t msg = { 0 };
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) mp3_decoder
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(mp3_decoder, &music_info);

            DB_PR("[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_setinfo(i2s_stream_writer, &music_info);
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
            continue;
        }

        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            ESP_LOGW(TAG, "[ * ] Stop event received");


            // audio_pipeline_stop(pipeline);
            // audio_pipeline_wait_for_stop(pipeline);
            // audio_pipeline_terminate(pipeline);
            // audio_pipeline_reset_ringbuffer(pipeline);
            // audio_pipeline_reset_elements(pipeline);

            // //set_next_file_marker();
            // DB_PR("[2.6-b] Set up  uri (file as tone_stream, mp3 as mp3 decoder, and default output is i2s)");
            // audio_element_set_uri(tone_stream_reader, tone_uri[test_i]);
            // audio_pipeline_run(pipeline);

            // test_i++;
            // if(test_i == 22)
            // {
            //     test_i=0;
            // }
            

            // break;
        }
    }

    DB_PR("[ 5 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, tone_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, mp3_decoder);

    /* Terminal the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

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
	while(PS_HandShake(&AS608Addr))//与AS608模块握手
	{
		delay_ms(400);
        DB_PR("--1--PS_HandShake no-----未检测到模块!!!\r\n");
        delay_ms(800);
        DB_PR("---1---尝试连接模块...\r\n");	

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
	}
	else
    {
        DB_PR("2-ensure = %d\r\n",ensure);
        ShowErrMessage(ensure);	
    }


}


static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            DB_PR("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));

            if((io_num==39)
                &&(gpio_get_level(io_num)==0))
            {
                send_cmd_to_lcd_pic(0x0011);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(LED_BLUE, 0);
                gpio_set_level(LED_GRREN, 0);
                gpio_set_level(LED_RED, 0);


                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(LED_BLUE, 1);
                gpio_set_level(LED_GRREN, 1);
                gpio_set_level(LED_RED, 1);

            }


        }


    }
    vTaskDelete(NULL);
}



void app_main(void)
{
    u16 buff_temp1[300]={0};
    u16 buff_temp2[300]={0};

    u8 buff_temp1_c[400]={0};//char
    u8 buff_temp2_c[400]={0};//150




    uart_init_all();
    //vTaskDelay(500 / portTICK_PERIOD_MS);

    //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
    xTaskCreate(echo_task, "uart_echo_task", 2* 1024, NULL, 1, NULL);//1024 10

	
    vTaskDelay(500 / portTICK_PERIOD_MS);



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
        default_factory_set();
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
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO39/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    // //hook isr handler for specific gpio pin
    // gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);




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




                // if(1== database_gz[i].dzx_mode_gz)
                // {
                //     //char *	_EXFUN(itoa,(int, char *, int));

                //     buff_temp1[k] = database_gz[i].dIndx_gz;
                //     DB_PR("b_temp1[k]= %03d ",buff_temp1[k]);//xiangmenhao
                    
                //     itoa(buff_temp1[k],(char*)(buff_temp1_c+4*(k)),10);//+4*(i-1)
                //     k++;
                // }
                // else if(2== database_gz[i].dzx_mode_gz)
                // {
                //     buff_temp2[l] = database_gz[i].dIndx_gz;
                //     DB_PR("b_temp2[l]= %03d ",buff_temp2[l]);//xiangmenhao
                //     itoa(buff_temp2[l],(char*)(buff_temp2_c+4*(l)),10);//+4*(i-1)
                //     l++;
                // }





                DB_PR("\r\n");
            }

            //DB_PR("---i=%d\r\n",i);

        }
    }

    DB_PR("database_gz[database_cw.dIndx].dIndx_gz=%d--.\r\n",database_gz[1].dIndx_gz); 

    vTaskDelay(30 / portTICK_PERIOD_MS);

    // uart0_debug_data_d(buff_temp1,0x9b);
    // uart0_debug_data_d(buff_temp1,0x9b);


    // for(uint16_t i=1;i<=SHENYU_GEZI_MAX;i++)
    // {
    //     if(buff_temp2_c[i]==0)
    //     {
    //         buff_temp2_c[i]=0x20;
    //     }
    //     if(buff_temp1_c[i]==0)
    //     {
    //         buff_temp1_c[i]=0x20;
    //     }
    // }


    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // DB_PR("-----gekouleixing-----\r\n");
    // send_cmd_to_lcd_bl_len(BL_GK_BH_Z,buff_temp2_c,BL_GK_BH_Z_LEN);//300 0x23 30
    // vTaskDelay(1 / portTICK_PERIOD_MS);
    // send_cmd_to_lcd_bl_len(BL_GK_BH_D,buff_temp1_c,BL_GK_BH_D_LEN);//300   空格
    // vTaskDelay(30 / portTICK_PERIOD_MS);
    tongbu_da();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_zh();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);
    tongbu_changqi();
    //vTaskDelay(1530 / portTICK_PERIOD_MS);


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
    audio_init();



}
