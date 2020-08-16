#ifndef __AUDIO_TONEURI_H__
#define __AUDIO_TONEURI_H__

extern const char* tone_uri[];

typedef enum {
    TONE_TYPE_BT_RECONNECT,
    TONE_TYPE_WECHAT,
    TONE_TYPE_WELCOME_TO_WIFI,
    TONE_TYPE_NEW_VERSION_AVAILABLE,
    TONE_TYPE_BT_SUCCESS,
    TONE_TYPE_FREETALK,
    TONE_TYPE_UPGRADE_DONE,
    TONE_TYPE_SHUTDOWN,
    TONE_TYPE_ALARM,
    TONE_TYPE_WIFI_SUCCESS,
    TONE_TYPE_UNDER_SMARTCONFIG,
    TONE_TYPE_OUT_OF_POWER,
    TONE_TYPE_SERVER_CONNECT,
    TONE_TYPE_HELLO,
    TONE_TYPE_NEW_MESSAGE,
    TONE_TYPE_PLEASE_RETRY_WIFI,
    TONE_TYPE_PLEASE_SETTING_WIFI,
    TONE_TYPE_WELCOME_TO_BT,
    TONE_TYPE_WIFI_TIME_OUT,
    TONE_TYPE_WIFI_RECONNECT,
    TONE_TYPE_SERVER_DISCONNECT,
    TONE_TYPE_MAX,
} tone_type_t;

int get_tone_uri_num();

#endif
