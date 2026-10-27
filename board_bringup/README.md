# camRover board bring-up

Arduino ESP32-S3 sketch based on `ov7670_camTest`.

Checks board from one web page: OV7670 RGB565 color at 320x240 and 1 fps, both DRV8871 inputs, quadrature encoder counts, and WS2812B status LED.

## Pin map

| Function | GPIO |
|---|---:|
| OV7670 D0..D7 | 4, 8, 5, 9, 6, 10, 7, 11 |
| OV7670 XCLK/PCLK/VSYNC/HREF | 13 / 12 / 21 / 14 |
| OV7670 SIOD/SIOC | 16 / 17 |
| M1 DRV8871 IN1/IN2 | 18 / 15 |
| M2 DRV8871 IN1/IN2 | 42 / 47 |
| M1 encoder A/B | 41 / 40 |
| M2 encoder A/B | 2 / 1 |
| 5x WS2812B chain DIN | 48 |

Open serial at 115200. Commands: `m -255..255 -255..255`, `s` stop, `l R G B`, `e` read counts. Web buttons pulse motors for 750 ms. Firmware watchdog stops any motor command after at most 2 seconds.

Motor 2 driver inputs are reversed in software so the same positive command drives both motors in the same rover direction. RGB commands update all five daisy-chained LEDs together.

Set `CAMERA_ROTATION` near the top of the sketch to `0`, `90`, `180`, or `270`. `CAMERA_CROP_LEFT` defaults to 6 pixels to remove the OV7670 left-edge black strip; increase it slightly if your sensor needs more cropping.

Build tested with Arduino-ESP32 3.3.12. Board settings: ESP32S3 Dev Module, Flash Size `16MB (128Mb)`, PSRAM `OPI PSRAM`, USB CDC on Boot `Enabled`.
