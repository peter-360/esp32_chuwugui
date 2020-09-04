/*This is tone file*/

const char* tone_uri[] = {
   "flash://tone/0_cl.mp3",
   "flash://tone/1_jiesuo.mp3",
   "flash://tone/2_kaiji.mp3",
   "flash://tone/3_open.mp3",
   "flash://tone/4_suoding.mp3",
   "flash://tone/5_wifi_con.mp3",
   "flash://tone/6_wifi_discon.mp3",
};

int get_tone_uri_num()
{
    return sizeof(tone_uri) / sizeof(char *) - 1;
}
