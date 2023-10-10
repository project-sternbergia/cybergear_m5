# cybergear_m5

M5 stack library for cybergear

## Supported framework

* Arduino for ESP32

## Supported device (ESP32)

* M5Stack Basic V2.7

## H/W Components

* [Xiaomi Cybergear](https://www.mi.com/cyber-gear)
* [M5Stack Basic V2.7](https://shop.m5stack.com/collections/m5-controllers/products/esp32-basic-core-lot-development-kit-v2-7)
* [M5Stack Commu module \[M001\]](https://shop.m5stack.com/products/commu-module)
* [XT30(2+2)-F](https://www.china-amass.com/product/contain/1Yf5h7G4u1927079)
* [Unbuckled Grove Cable](https://shop.m5stack.com/products/4pin-buckled-grove-cable)

## H/W Connection

![CyberGear_M5_Connection](docs/img/cybergear_m5_connection.png)

## How to run sample

### Arduino IDE

1. Clone [MCP_CAN_LIB](https://github.com/coryjfowler/MCP_CAN_lib) and [cygergear_m5](https://github.com/project-sternbergia/cybergear_m5) to Arduino Library directory.

```bash
cd ~/Arduino/libraries
git clone https://github.com/coryjfowler/MCP_CAN_lib.git
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

## References

* Xiaomi Cybergear 微电机使用说明书

## LICENSE

MIT
