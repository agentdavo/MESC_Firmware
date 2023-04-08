```
Use FSP v1.2.0 to configure the peripherals and update the generated bsp

https://github.com/renesas/rzt-fsp/releases/tag/v1.2.0

Standalone FSP Smart Configurator

setup_rztfsp_v1_2_0_rzsc_v2023-01.exe

see Config.cmake to suit your own toolchain directory

You make have to manually change the GeneratedCfg.cmake fsp.ld to fsp_xspi0_boot.ld to compile correctly

```
