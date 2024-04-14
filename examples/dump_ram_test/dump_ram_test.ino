#include <Arduino.h>
#include <M5Stack.h>
#include "cybergear_driver.hh"

#define USE_ESP32_CAN
#ifdef USE_ESP32_CAN
#include "cybergear_can_interface_esp32.hh"
#else
#include "cybergear_can_interface_mcp.hh"
#endif

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID = 0x7F;

// init cybergeardriver
CybergearDriver driver = CybergearDriver(MASTER_CAN_ID, MOT_CAN_ID);

#ifdef USE_ESP32_CAN
CybergearCanInterfaceEsp32 interface;
#else
CybergearCanInterfaceMcp interface;
#endif

void setup()
{
  M5.begin(true, false, true);

  // init cybergear driver
  M5.Lcd.printf("Start read write motor param test\n");

  interface.init();
  driver.init(&interface);
  driver.init_motor(MODE_POSITION);
  delay(1000);
  driver.enable_motor();
}

void loop()
{
  M5.update();

  // get motor parameter
  driver.set_position_ref(0.0f);
  driver.dump_motor_param();
  driver.process_packet();

  MotorParameter param = driver.get_motor_param();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Received motor param\n");
  M5.Lcd.printf("  stamp    : %u\n", param.stamp_usec);
  M5.Lcd.printf("  run_mode : %d\n", param.run_mode);
  M5.Lcd.printf("  iq_ref : %f\n", param.iq_ref);
  M5.Lcd.printf("  spd_ref : %f\n", param.spd_ref);
  M5.Lcd.printf("  limit_torque : %f\n", param.limit_torque);
  M5.Lcd.printf("  cur_kp : %f\n", param.cur_kp);
  M5.Lcd.printf("  cur_ki : %f\n", param.cur_ki);
  M5.Lcd.printf("  cur_filt_gain : %f\n", param.cur_filt_gain);
  M5.Lcd.printf("  loc_ref : %f\n", param.loc_ref);
  M5.Lcd.printf("  limit_spd : %f\n", param.limit_spd);
  M5.Lcd.printf("  limit_cur : %f\n", param.limit_cur);
  M5.Lcd.printf("  mech_pos : %f\n", param.mech_pos);
  M5.Lcd.printf("  iqf : %f\n", param.iqf);
  M5.Lcd.printf("  mech_vel : %f\n", param.mech_vel);
  M5.Lcd.printf("  vbus : %f\n", param.vbus);
  M5.Lcd.printf("  rotation : %d\n", param.rotation);
  M5.Lcd.printf("  loc_kp : %f\n", param.loc_kp);
  M5.Lcd.printf("  spd_kp : %f\n", param.spd_kp);
  M5.Lcd.printf("  spd_ki : %f\n", param.spd_ki);
  delay(5000);
}
