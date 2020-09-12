/*This is tone file*/

const char* tone_uri[] = {
   "flash://tone/0_cl.mp3",
   "flash://tone/1_firm_upok.mp3",
   "flash://tone/2_jiesuo.mp3",
   "flash://tone/3_kaiji.mp3",
   "flash://tone/4_open.mp3",
   "flash://tone/5_server_confail.mp3",
   "flash://tone/6_suoding.mp3",
   "flash://tone/7_wifi_con.mp3",
   "flash://tone/8_wifi_discon.mp3",
   "flash://tone/9_wifi_scpass.mp3",
};

int get_tone_uri_num()
{
    return sizeof(tone_uri) / sizeof(char *) - 1;
}
