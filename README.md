# TEMS_Arduino

## Hardware

This repository contains the codes for esp32 and sht20 sensor.
You need to compile and upload the sketch to the esp32 board.
Before upload the sketch, you should connect esp32 board and sht20 sensor correctly.

The esp32 board and sht20 uses I2C communication.

Please refer to the table below when connecting the esp32 and sht20.

| ESP32   |  sht20  |
| ------- |:-------:|
| GND     |   GND   |
| 3V3     |   VCC   |
| GPIO 21 |   SDA   |
| GPIO 22 |   SCL   |


## ESP32 library

While implementing this, I found that the published esp32 ble library has some problem.
And I realised that there are several repositories for the esp32 ble library.

Please download and include the codes from [this repository](https://github.com/nkolban/esp32-snippets).

If you go to the "cpp_utils" directory in that repository, you could see the README file that contains the relevant instruction.


### esp32 ble - "sketch too big" issue

When you try to compile the sketch, some of you would fail to compile the sketch since the memory usage of the BLEDevice object exceeds the limit.

To overcome this problem, you need to modify the partition of the esp32 board.

I found an useful link from google - please click [here](https://github.com/nkolban/esp32-snippets/issues/441).


## Contribution

The codes in this repository are written for TEMS project of Technonia, thus, I would recommend you to fork the repository if you want to modify the codes.

However, any questions are welcome - please open a new issue if you find some problem or have some question.


## Copyrights

TEMS stands for Technonia Environment Monitoring System.
As the name of the project depicts, this repository contains the codes for arduino part of the TEMS project.
Thus, it is possible to say that all copyrights of this repository belong to both Yeonwoo Sung (the main programmer of the TEMS project) and Technonia.