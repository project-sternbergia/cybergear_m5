#include <Arduino.h>
#include <M5Stack.h>
#include <math.h>
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
const float MAX_CURRENT = 1.0;
const float LEADER_FOLLOWER_GAIN = 30.0;

std::vector<uint8_t> motor_ids = {MOT_LEADER_CAN_ID, MOT_FOLLOWER_CAN_ID};
std::vector<float> currents = {0.0f, 0.0f};

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);
#ifdef USE_ESP32_CAN
CybergearCanInterfaceEsp32 interface;
#else
CybergearCanInterfaceMcp interface;
#endif

void setup()
{
  M5.begin();

  // init position offset
  M5.Lcd.print("Init motors ... ");
  interface.init();
  controller.init(motor_ids, MODE_POSITION, &interface);
  controller.enable_motors();
  M5.Lcd.println("done");

  M5.Lcd.print("move to motor origin ... ");
  controller.send_position_command(motor_ids, {0.0f, 0.0f});
  delay(1000);
  M5.Lcd.println("done");

  // start bilateral mode
  M5.Lcd.print("starting leader follower demo ... ");
  controller.init(motor_ids, MODE_CURRENT, &interface);
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
  if ( controller.process_packet() ) {
    controller.get_motor_status(status_list);
  }

  currents[0] = (status_list[1].position - status_list[0].position) * LEADER_FOLLOWER_GAIN;
  currents[1] = (status_list[0].position - status_list[1].position) * LEADER_FOLLOWER_GAIN;

  // clamp data range
  currents[0] = min(max(currents[0], -MAX_CURRENT), MAX_CURRENT);
  currents[1] = min(max(currents[1], -MAX_CURRENT), MAX_CURRENT);
}
