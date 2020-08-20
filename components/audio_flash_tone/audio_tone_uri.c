/*This is tone file*/

const char* tone_uri[] = {
   "flash://tone/0_akaiji.mp3",
   "flash://tone/1_axm_101.mp3",
   "flash://tone/2_axm_102.mp3",
   "flash://tone/3_axm_103.mp3",
   "flash://tone/4_axm_104.mp3",
   "flash://tone/5_axm_105.mp3",
   "flash://tone/6_axm_106.mp3",
   "flash://tone/7_axm_107.mp3",
   "flash://tone/8_axm_108.mp3",
   "flash://tone/9_axm_109.mp3",
   "flash://tone/10_axm_110.mp3",
   "flash://tone/11_axm_111.mp3",
   "flash://tone/12_axm_112.mp3",
   "flash://tone/13_axm_113.mp3",
   "flash://tone/14_axm_114.mp3",
   "flash://tone/15_axm_115.mp3",
   "flash://tone/16_axm_116.mp3",
   "flash://tone/17_axm_117.mp3",
   "flash://tone/18_axm_118.mp3",
   "flash://tone/19_axm_119.mp3",
   "flash://tone/20_axm_120.mp3",
   "flash://tone/21_cl.mp3",
   "flash://tone/22_jeisuo.mp3",
   "flash://tone/23_open.mp3",
   "flash://tone/24_suoding.mp3",
};

int get_tone_uri_num()
{
    return sizeof(tone_uri) / sizeof(char *) - 1;
}
