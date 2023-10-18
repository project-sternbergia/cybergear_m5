#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include "cybergear_controller.hh"

#define INC_POSITION  20.0
#define INC_VELOCITY  0.4
#define INC_TORQUE    0.04


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
std::vector<uint8_t> motor_ids = {MOT_LEADER_CAN_ID, MOT_FOLLOWER_CAN_ID};

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);


void setup()
{
  M5.begin();

  // init cybergear driver
  init_can();
  controller.init(motor_ids, MODE_CURRENT, &CAN0);
  controller.enable_motors();
}

std::vector<float> currents = {0.0f, 0.0f};

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

  currents[0] = (status_list[1].position - status_list[0].position) * 1.0f;
  currents[1] = (status_list[0].position - status_list[1].position) * 1.0f;
  delay(1);
}

void init_can()
{
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
