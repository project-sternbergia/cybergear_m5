#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include "ros2_logo.hh"
#include "cybergear_bridge.hh"
#include "cybergear_controller.hh"


/**
 * @brief Init can interface
 */
void init_can();
void can_receive_task();

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_LEADER_CAN_ID = 0x7F;
uint8_t MOT_FOLLOWER_CAN_ID = 0x7E;
std::vector<uint8_t> motor_ids = {MOT_LEADER_CAN_ID, MOT_FOLLOWER_CAN_ID};

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);
CybergearBridge bridge = CybergearBridge(&controller, &Serial);


void setup()
{
  M5.begin(true,true,false);
  Serial.begin(2000000);
  Serial.flush();

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.drawBitmap((M5.Lcd.width() - imgWidth)/2, (M5.Lcd.height() - imgHeight)/2, imgWidth, imgHeight, (uint16_t *)img);

  // init cybergear driver
  init_can();
  controller.init(motor_ids, MODE_CURRENT, &CAN0);

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

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
