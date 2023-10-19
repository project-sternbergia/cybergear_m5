#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
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
  M5.begin();
  M5.Lcd.begin();             // 画面初期化
  M5.Lcd.setRotation(1);      // 画面向き設定（0～3で設定、4～7は反転)※初期値は1
  M5.Lcd.setTextWrap(false);  // 画面端での改行の有無（true:有り[初期値], false:無し）※print関数のみ有効

  // init cybergear driver
  init_can();
  controller.init(motor_ids, MODE_CURRENT, &CAN0);

  //launch task 0 loop
  // xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 1);

  // launch task 1 loop
  // xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 2, NULL, 1);
}

void loop()
{
  // update m5 satatus
  M5.update();
  bridge.process_request_command();
  bridge.process_motor_response();
  M5.Lcd.setCursor(0, 0);     // 表示座標を指定(x座標, y座標)
}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
