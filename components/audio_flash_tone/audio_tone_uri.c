/*This is tone file*/

const char* tone_uri[] = {
   "flash://tone/0_jiesuo-1.mp3",
   "flash://tone/1_kaiji.mp3",
   "flash://tone/2_open-1.mp3",
   "flash://tone/3_open-2.mp3",
   "flash://tone/4_open-3.mp3",
   "flash://tone/5_open-4.mp3",
   "flash://tone/6_open-5.mp3",
   "flash://tone/7_open-6.mp3",
   "flash://tone/8_open.mp3",
   "flash://tone/9_suoding-1.mp3",
};

int get_tone_uri_num()
{
    return sizeof(tone_uri) / sizeof(char *) - 1;
}
