# 9roomer

## Hardware

This repository contains the codes for 9room project.
The 9roomer is a device that uses esp32 board with sht20 sensor.
The main aim of 9roomer is to collect the temperature, humidity and radioactive-rays data and send the collected data to the 9roomwe server.


To run the 9roomer, you need to compile and upload the sketch to the esp32 board.
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

Furthermore, according to [this issue](https://github.com/nkolban/esp32-snippets/issues/797), you should delete [this line](https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/BLEAdvertising.cpp#L496). Because, that line prevents the BLEAdverting instance to stop advertising the ble server.

As we need to stop the BLE server when it changes to the BLE client mode, thus, you should delete that line from BLEAdvertising.cpp file before compile the sketch. Otherwise, the BLE server will keep advertise the server even if the server instance is deleted from the memory (This could generate unexpected errors, for example, NullPointerException, which will crash the device).


### esp32 ble - "sketch too big" issue

When you try to compile the sketch, some of you would fail to compile the sketch since the memory usage of the BLEDevice object exceeds the limit.

To overcome this problem, you need to modify the partition of the esp32 board.

I found an useful link from google - please click [here](https://github.com/nkolban/esp32-snippets/issues/441).

