#ifndef __AUDIO_TONEURI_H__
#define __AUDIO_TONEURI_H__

extern const char* tone_uri[];

typedef enum {
    TONE_TYPE_JIESUO_1,
    TONE_TYPE_KAIJI,
    TONE_TYPE_OPEN_1,
    TONE_TYPE_OPEN_2,
    TONE_TYPE_OPEN_3,
    TONE_TYPE_OPEN_4,
    TONE_TYPE_OPEN_5,
    TONE_TYPE_OPEN_6,
    TONE_TYPE_OPEN,
    TONE_TYPE_SUODING_1,
    TONE_TYPE_MAX,
} tone_type_t;

int get_tone_uri_num();

#endif
