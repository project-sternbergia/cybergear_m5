#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include "cybergear_controller.hh"

/**
 * @brief Init can interface
 */
void init_can();

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CURRENT_CAN_ID = 0x7F;
uint8_t MOT_NEXT_CAN_ID = 0x7E;

// init cybergeardriver
CybergearDriver driver = CybergearDriver(MASTER_CAN_ID, MOT_CURRENT_CAN_ID);

void setup()
{
  M5.begin();

  // init sprite
  sprite.setColorDepth(8);
  sprite.setTextSize(3);
  sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());

  // init cybergear driver
  init_can();
  driver.init(&CAN0);
  driver.init_motor(MODE_POSITION);
  driver.enable_motor();
  driver.change_motor_can_id(MOT_NEXT_CAN_ID);
}

void loop()
{
  M5.update();
}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
