.section ".enc_dat", #alloc
.balign 32
.global g_a_as_config_dat
g_a_as_config_dat:
.incbin "/Users/alexanderismailov/WORK/david/MESC_Firmware/MESC_RZT2M/dat/Config_AFormat_v1.0.dat"
.balign 32
.global g_biss_config_dat
g_biss_config_dat:
.incbin "/Users/alexanderismailov/WORK/david/MESC_Firmware/MESC_RZT2M/dat/Config_Biss_v1.0.dat"
.balign 32
.global g_endat_config_dat
g_endat_config_dat:
.incbin "/Users/alexanderismailov/WORK/david/MESC_Firmware/MESC_RZT2M/dat/Config_Endat22_v1.0.dat"
.balign 32
.global g_fac_config_dat
g_fac_config_dat:
.incbin "/Users/alexanderismailov/WORK/david/MESC_Firmware/MESC_RZT2M/dat/Config_Fa_Coder_V1.0.dat"
.balign 32
.global g_hfdsl_config_dat
g_hfdsl_config_dat:
.incbin "/Users/alexanderismailov/WORK/david/MESC_Firmware/MESC_RZT2M/dat/Config_hfdsl_V1.0.dat"
.balign 32
.global g_pinmux_config
g_pinmux_config:
.incbin "/Users/alexanderismailov/WORK/david/MESC_Firmware/MESC_RZT2M/lib/ecl/RZT2_pinmux_v1.0.bin"
.end
