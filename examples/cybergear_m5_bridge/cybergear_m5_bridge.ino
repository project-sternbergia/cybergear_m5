#include <Arduino.h>
#include <math.h>
#include <M5Stack.h>
#include "ros2_logo.hh"
#include "cybergear_bridge.hh"
#include "cybergear_controller.hh"
#ifdef USE_ESP32_CAN
#include "cybergear_can_interface_esp32.hh"
#else
#include "cybergear_can_interface_mcp.hh"
#endif

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_LEADER_CAN_ID = 0x7F;
uint8_t MOT_FOLLOWER_CAN_ID = 0x7E;
std::vector<uint8_t> motor_ids = {MOT_LEADER_CAN_ID, MOT_FOLLOWER_CAN_ID};

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);
CybergearBridge bridge = CybergearBridge(&controller, &Serial);
#ifdef USE_ESP32_CAN
CybergearCanInterfaceEsp32 interface;
#else
CybergearCanInterfaceMcp interface;
#endif


void setup()
{
  M5.begin(true, true, false);
  Serial.begin(2000000);
  Serial.flush();

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.drawBitmap((M5.Lcd.width() - imgWidth)/2, (M5.Lcd.height() - imgHeight)/2, imgWidth, imgHeight, (uint16_t *)img);

  // init cybergear driver
  interface.init();
  controller.init(motor_ids, MODE_CURRENT, &interface);

  delay(1000);
  M5.Lcd.fillScreen(BLACK);
}

void loop()
{
  // update m5 satatus
  M5.update();
  bridge.process_request_command();
  bridge.process_motor_response();
}
