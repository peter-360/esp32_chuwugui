#ifndef __AUDIO_TONEURI_H__
#define __AUDIO_TONEURI_H__

extern const char* tone_uri[];

typedef enum {
    TONE_TYPE_CL,
    TONE_TYPE_JIESUO,
    TONE_TYPE_KAIJI,
    TONE_TYPE_OPEN,
    TONE_TYPE_SUODING,
    TONE_TYPE_MAX,
} tone_type_t;

int get_tone_uri_num();

#endif
