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
uint8_t MOT_LEADER_CAN_ID = 0x7F;
uint8_t MOT_FOLLOWER_CAN_ID = 0x7E;
const float MAX_CURRENT = 1.0;
const float LEADER_FOLLOWER_GAIN = 30.0;

std::vector<uint8_t> motor_ids = {MOT_LEADER_CAN_ID, MOT_FOLLOWER_CAN_ID};
std::vector<float> currents = {0.0f, 0.0f};

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);


void setup()
{
  M5.begin();

  // init cybergear driver
  init_can();

  // init position offset
  M5.Lcd.print("Init motors ... ");
  controller.init(motor_ids, MODE_POSITION, &CAN0);
  controller.enable_motors();
  M5.Lcd.println("done");

  M5.Lcd.print("move to motor origin ... ");
  controller.send_position_command(motor_ids, {0.0f, 0.0f});
  delay(1000);
  M5.Lcd.println("done");

  // start bilateral mode
  M5.Lcd.print("starting leader follower demo ... ");
  controller.init(motor_ids, MODE_CURRENT, &CAN0);
  controller.enable_motors();
  M5.Lcd.println("done");
}

void loop()
{
  // update m5 satatus
  M5.update();

  controller.send_current_command(motor_ids, currents);

  // update and get motor data
  std::vector<MotorStatus> status_list;
  if ( controller.process_can_packet() ) {
    controller.get_motor_status(status_list);
  }

  currents[0] = (status_list[1].position - status_list[0].position) * LEADER_FOLLOWER_GAIN;
  currents[1] = (status_list[0].position - status_list[1].position) * LEADER_FOLLOWER_GAIN;

  // clamp data range
  currents[0] = min(max(currents[0], -MAX_CURRENT), MAX_CURRENT);
  currents[1] = min(max(currents[1], -MAX_CURRENT), MAX_CURRENT);
}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
