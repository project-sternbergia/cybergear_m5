# cybergear_m5

M5 stack library for cybergear

*"Remember, with great torque comes great responsibility."*

## Supported framework

* Arduino for ESP32

## Supported device (ESP32)

* M5Stack Basic V2.7

## H/W Components (MCP2515)

* [Xiaomi Cybergear](https://www.mi.com/cyber-gear)
* [M5Stack Basic V2.7](https://shop.m5stack.com/collections/m5-controllers/products/esp32-basic-core-lot-development-kit-v2-7)
* [M5Stack Commu module \[M001\]](https://shop.m5stack.com/products/commu-module)
* [XT30(2+2)-F](https://www.china-amass.com/product/contain/1Yf5h7G4u1927079)
* [Grove Cable](https://www.seeedstudio.com/Grove-Universal-4-Pin-Buckled-20cm-Cable-5-PCs-pack.html)

![image](https://github.com/project-sternbergia/cybergear_m5/assets/147309062/c36d82cf-e91a-45da-ac53-a79e8d8fc730)

## H/W Components (ESP32 + CAN Transceiver Unit)

* [Xiaomi Cybergear](https://www.mi.com/cyber-gear)
* [M5Stack Basic V2.7](https://shop.m5stack.com/collections/m5-controllers/products/esp32-basic-core-lot-development-kit-v2-7)
* [LAN Module W5500 with PoE V12](https://shop.m5stack.com/products/lan-module-w5500-with-poe-v12) or [CANBus Unit(CA-IS3050G)](https://shop.m5stack.com/products/canbus-unitca-is3050g)
* [XT30(2+2)-F](https://www.china-amass.com/product/contain/1Yf5h7G4u1927079)
* [Grove Cable](https://www.seeedstudio.com/Grove-Universal-4-Pin-Buckled-20cm-Cable-5-PCs-pack.html)

## How to use Official GUI tool

This software requires a specific CAN to USB module.
The official documentation recommends YourCee's USB to CAN module, which supports the serial protocol with a frame header of 41 54 and frame tail of 0D 0A, but this is not readily available, so we searched Aliexpress for an alternative.
※ Note that this will not work for general modules.

Tested(for reference):
* [CAN to USB module](https://ja.aliexpress.com/item/1005004296661528.html)

## Recommended pre-crimped cables

* [XT30(2+2) Cable](https://ja.aliexpress.com/item/1005006046478152.html)

## How to run sample

### Arduino IDE

1. Clone [MCP_CAN_LIB](https://github.com/coryjfowler/MCP_CAN_lib) and [cygergear_m5](https://github.com/project-sternbergia/cybergear_m5) to Arduino Library directory.

```bash
cd ~/Arduino/libraries
git clone https://github.com/coryjfowler/MCP_CAN_lib.git
git clone https://github.com/Locoduino/RingBuffer.git
git clone git@github.com:project-sternbergia/arduino-CAN.git
git clone https://github.com/project-sternbergia/cybergear_m5.git
```

2. Open [cybergear_m5/examples/control_mode_example.ino](https://github.com/project-sternbergia/cybergear_m5/blob/main/examples/control_mode_example.ino) with Arduino IDE

    ![image](https://github.com/project-sternbergia/cybergear_m5/assets/147309062/8a4edd90-241c-4683-a13d-4a26685e8251)

    Put this file in the same folder as control_mode_example.ino (for Arduino IDE)


4. Build and write firmware to M5Stack

## Sample Code

### control_mode_example.ino

Check cybergear behaviour using M5 stack.

* Middle Button - Change Control Mode (Position Mode -> Speed Mode -> Current Mode)
* Right Button  - Increase control value
* Left Button  - Decrease control value

![CyberGear_M5_Control_Mode](docs/img/cybergear_m5_sample_control_mode.gif)

### cybergear_bilateral.ino

This example use two cybergears for leader and follower.
Before you test this example, please change cybergear can id as follows.
After that write [cybergear_m5/examples/cybergear_bilateral.ino](https://github.com/project-sternbergia/cybergear_m5/blob/main/examples/cybergear_bilateral.ino) to m5 stack throughout Arduino IDE.

* leader cybergear : 0x7F
* follower cybergear : 0x7E

![CyberGear_Bilateral](docs/img/cybergear_bilateral.gif)

## References

* [Xiaomi Cybergear 微电机使用说明书](https://web.vip.miui.com/page/info/mio/mio/detail?postId=40233100)

## LICENSE

MIT
