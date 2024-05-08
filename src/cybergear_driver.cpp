#include "cybergear_driver.hh"

#include <Arduino.h>  // for micros and delay function

#include <vector>

#include "cybergear_driver_defs.hh"
#include "cybergear_driver_utils.hh"

CybergearDriver::CybergearDriver()
: can_(NULL), master_can_id_(0), target_can_id_(0), run_mode_(MODE_MOTION), send_count_(0)
{
}

CybergearDriver::CybergearDriver(uint8_t master_can_id, uint8_t target_can_id)
: can_(NULL),
  master_can_id_(master_can_id),
  target_can_id_(target_can_id),
  run_mode_(MODE_MOTION),
  send_count_(0)
{
}

CybergearDriver::~CybergearDriver() {}

void CybergearDriver::init(CybergearCanInterface * can, uint16_t wait_response_time_usec)
{
  can_ = can;
  wait_response_time_usec_ = wait_response_time_usec;
}

void CybergearDriver::init_motor(uint8_t run_mode)
{
  reset_motor();
  set_run_mode(run_mode);
}

void CybergearDriver::enable_motor()
{
  uint8_t data[8] = {0x00};
  send_command(target_can_id_, CMD_ENABLE, master_can_id_, 8, data);
}

void CybergearDriver::reset_motor()
{
  uint8_t data[8] = {0x00};
  send_command(target_can_id_, CMD_RESET, master_can_id_, 8, data);
}

void CybergearDriver::set_run_mode(uint8_t run_mode)
{
  // set class variable
  run_mode_ = run_mode;
  uint8_t data[8] = {0x00};
  data[0] = ADDR_RUN_MODE & 0x00FF;
  data[1] = ADDR_RUN_MODE >> 8;
  data[4] = run_mode;
  send_command(target_can_id_, CMD_RAM_WRITE, master_can_id_, 8, data);
}

void CybergearDriver::motor_control(float position, float speed, float torque, float kp, float kd)
{
  uint8_t data[8] = {0x00};
  data[0] = float_to_uint(position, P_MIN, P_MAX, 16) >> 8;
  data[1] = float_to_uint(position, P_MIN, P_MAX, 16);
  data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
  data[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
  data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
  data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
  data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
  data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);

  uint16_t data_torque = float_to_uint(torque, T_MIN, T_MAX, 16);
  send_command(target_can_id_, CMD_POSITION, data_torque, 8, data);
}

void CybergearDriver::set_limit_speed(float speed)
{
  write_float_data(target_can_id_, ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
}

void CybergearDriver::set_limit_current(float current)
{
  write_float_data(target_can_id_, ADDR_LIMIT_CURRENT, current, 0.0f, IQ_MAX);
}

void CybergearDriver::set_current_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_CURRENT_KP, kp, 0.0f, KP_MAX);
}

void CybergearDriver::set_current_ki(float ki)
{
  // write_float_data(target_can_id_, ADDR_CURRENT_KI, ki, 0.0f, KI_MAX);
}

void CybergearDriver::set_current_filter_gain(float gain)
{
  write_float_data(
    target_can_id_, ADDR_CURRENT_FILTER_GAIN, gain, CURRENT_FILTER_GAIN_MIN,
    CURRENT_FILTER_GAIN_MAX);
}

void CybergearDriver::set_limit_torque(float torque)
{
  write_float_data(target_can_id_, ADDR_LIMIT_TORQUE, torque, 0.0f, T_MAX);
}

void CybergearDriver::set_position_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_LOC_KP, kp, 0.0f, 200.0f);
}

void CybergearDriver::set_velocity_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_SPD_KP, kp, 0.0f, 200.0f);
}

void CybergearDriver::set_velocity_ki(float ki)
{
  write_float_data(target_can_id_, ADDR_SPD_KI, ki, 0.0f, 200.0f);
}

void CybergearDriver::get_mech_position() { read_ram_data(ADDR_MECH_POS); }

void CybergearDriver::get_mech_velocity() { read_ram_data(ADDR_MECH_VEL); }

void CybergearDriver::get_vbus() { read_ram_data(ADDR_VBUS); }

void CybergearDriver::get_rotation() { read_ram_data(ADDR_ROTATION); }

void CybergearDriver::dump_motor_param()
{
  const std::vector<uint16_t> index_array = {
    ADDR_RUN_MODE,
    ADDR_IQ_REF,
    ADDR_SPEED_REF,
    ADDR_LIMIT_TORQUE,
    ADDR_CURRENT_KP,
    ADDR_CURRENT_KI,
    ADDR_CURRENT_FILTER_GAIN,
    ADDR_LOC_REF,
    ADDR_LIMIT_SPEED,
    ADDR_LIMIT_CURRENT,
    ADDR_MECH_POS,
    ADDR_IQF,
    ADDR_MECH_VEL,
    ADDR_VBUS,
    ADDR_ROTATION,
    ADDR_LOC_KP,
    ADDR_SPD_KP,
    ADDR_SPD_KI};

  for (auto index : index_array) {
    read_ram_data(index);
    delay(1);
  }
}

void CybergearDriver::set_position_ref(float position)
{
  write_float_data(target_can_id_, ADDR_LOC_REF, position, P_MIN, P_MAX);
}

void CybergearDriver::set_speed_ref(float speed)
{
  write_float_data(target_can_id_, ADDR_SPEED_REF, speed, V_MIN, V_MAX);
}

void CybergearDriver::set_current_ref(float current)
{
  write_float_data(target_can_id_, ADDR_IQ_REF, current, IQ_MIN, IQ_MAX);
}

void CybergearDriver::set_mech_position_to_zero()
{
  uint8_t data[8] = {0x00};
  data[0] = 0x01;
  send_command(target_can_id_, CMD_SET_MECH_POSITION_TO_ZERO, master_can_id_, 8, data);
}

void CybergearDriver::change_motor_can_id(uint8_t can_id)
{
  uint8_t data[8] = {0x00};
  uint16_t option = can_id << 8 | master_can_id_;
  send_command(target_can_id_, CMD_CHANGE_CAN_ID, option, 8, data);
}

void CybergearDriver::read_ram_data(uint16_t index)
{
  uint8_t data[8] = {0x00};
  memcpy(&data[0], &index, 2);
  send_command(target_can_id_, CMD_RAM_READ, master_can_id_, 8, data);
}

uint8_t CybergearDriver::get_run_mode() const { return run_mode_; }

uint8_t CybergearDriver::get_motor_id() const { return target_can_id_; }

bool CybergearDriver::process_packet()
{
  CG_DEBUG_FUNC
  bool check_update = false;
  while (can_->available()) {
    if (receive_motor_data(motor_status_)) {
      check_update = true;
    }
  }
  return check_update;
}

bool CybergearDriver::update_motor_status(unsigned long id, const uint8_t * data, unsigned long len)
{
  CG_DEBUG_FUNC
  uint8_t receive_can_id = id & 0xff;
  if (receive_can_id != master_can_id_) {
    return false;
  }

  uint8_t motor_can_id = (id & 0xff00) >> 8;
  if (motor_can_id != target_can_id_) {
    return false;
  }

  // check packet type
  uint8_t packet_type = (id & 0x3F000000) >> 24;
  if (packet_type == CMD_RESPONSE) {
    process_motor_packet(data, len);

  } else if (packet_type == CMD_RAM_READ) {
    process_read_parameter_packet(data, len);

  } else if (packet_type == CMD_GET_MOTOR_FAIL) {
    // NOT IMPLEMENTED

  } else {
    CG_DEBUG_PRINTF("invalid command response [0x%x]\n", packet_type);
    print_can_packet(id, data, len);
    return false;
  }

  return true;
}

void CybergearDriver::write_float_data(
  uint8_t can_id, uint16_t addr, float value, float min, float max)
{
  uint8_t data[8] = {0x00};
  data[0] = addr & 0x00FF;
  data[1] = addr >> 8;

  float val = (max < value) ? max : value;
  val = (min > value) ? min : value;
  memcpy(&data[4], &value, 4);
  send_command(can_id, CMD_RAM_WRITE, master_can_id_, 8, data);
}

int CybergearDriver::float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float CybergearDriver::uint_to_float(uint16_t x, float x_min, float x_max)
{
  uint16_t type_max = 0xFFFF;
  float span = x_max - x_min;
  return (float)x / type_max * span + x_min;
}

void CybergearDriver::send_command(
  uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data)
{
  uint32_t id = cmd_id << 24 | option << 8 | can_id;
  can_->send_message(id, data, len, true);
  delayMicroseconds(wait_response_time_usec_);
  ++send_count_;
}

bool CybergearDriver::receive_motor_data(MotorStatus & mot)
{
  // receive data
  unsigned long id;
  uint8_t len;
  if (!can_->read_message(id, receive_buffer_, len)) {
    CG_DEBUG_PRINTLN("received data is not available");
    return false;
  }

  // if id is not mine
  uint8_t receive_can_id = id & 0xff;
  if (receive_can_id != master_can_id_) {
    CG_DEBUG_PRINTF(
      "Invalid master can id. Expected=[0x%02x] Actual=[0x%02x] Raw=[%x]\n", master_can_id_,
      receive_can_id, id);
    return false;
  }

  uint8_t motor_can_id = (id & 0xff00) >> 8;
  if (motor_can_id != target_can_id_) {
    CG_DEBUG_PRINTF(
      "Invalid target can id. Expected=[0x%02x] Actual=[0x%02x] Raw=[%x]\n", target_can_id_,
      motor_can_id, id);
    return false;
  }

  // parse packet --------------
  return update_motor_status(id, receive_buffer_, len);
}

void CybergearDriver::process_motor_packet(const uint8_t * data, unsigned long len)
{
  motor_status_.raw_position = data[1] | data[0] << 8;
  motor_status_.raw_velocity = data[3] | data[2] << 8;
  motor_status_.raw_effort = data[5] | data[4] << 8;
  motor_status_.raw_temperature = data[7] | data[6] << 8;

  // convert motor data
  motor_status_.stamp_usec = micros();
  motor_status_.motor_id = target_can_id_;
  motor_status_.position = uint_to_float(motor_status_.raw_position, P_MIN, P_MAX);
  motor_status_.velocity = uint_to_float(motor_status_.raw_velocity, V_MIN, V_MAX);
  motor_status_.effort = uint_to_float(motor_status_.raw_effort, T_MIN, T_MAX);
  motor_status_.temperature = motor_status_.raw_temperature;
}

void CybergearDriver::process_read_parameter_packet(const uint8_t * data, unsigned long len)
{
  uint16_t index = data[1] << 8 | data[0];

  uint8_t uint8_data;
  memcpy(&uint8_data, &data[4], sizeof(uint8_t));

  int16_t int16_data;
  memcpy(&int16_data, &data[4], sizeof(int16_t));

  float float_data;
  memcpy(&float_data, &data[4], sizeof(float));

  bool is_updated = true;

  switch (index) {
    case ADDR_RUN_MODE:
      motor_param_.run_mode = uint8_data;
      CG_DEBUG_PRINTF("Receive ADDR_RUN_MODE = [0x%02x]\n", uint8_data);
      break;
    case ADDR_IQ_REF:
      motor_param_.iq_ref = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_IQ_REF = [%f]\n", float_data);
      break;
    case ADDR_SPEED_REF:
      motor_param_.spd_ref = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_SPEED_REF = [%f]\n", float_data);
      break;
    case ADDR_LIMIT_TORQUE:
      motor_param_.limit_torque = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_LIMIT_TORQUE = [%f]\n", float_data);
      break;
    case ADDR_CURRENT_KP:
      motor_param_.cur_kp = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_CURRENT_KP = [%f]\n", float_data);
      break;
    case ADDR_CURRENT_KI:
      motor_param_.cur_ki = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_CURRENT_KI = [%f]\n", float_data);
      break;
    case ADDR_CURRENT_FILTER_GAIN:
      motor_param_.cur_filt_gain = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_CURRENT_FILTER_GAIN = [%f]\n", float_data);
      break;
    case ADDR_LOC_REF:
      motor_param_.loc_ref = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_LOC_REF = [%f]\n", float_data);
      break;
    case ADDR_LIMIT_SPEED:
      motor_param_.limit_spd = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_LIMIT_SPEED = [%f]\n", float_data);
      break;
    case ADDR_LIMIT_CURRENT:
      motor_param_.limit_cur = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_LIMIT_CURRENT = [%f]\n", float_data);
      break;
    case ADDR_MECH_POS:
      motor_param_.mech_pos = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_MECH_POS = [%f]\n", float_data);
      break;
    case ADDR_IQF:
      motor_param_.iqf = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_IQF = [%f]\n", float_data);
      break;
    case ADDR_MECH_VEL:
      motor_param_.mech_vel = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_MECH_VEL = [%f]\n", float_data);
      break;
    case ADDR_VBUS:
      motor_param_.vbus = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_VBUS = [%f]\n", float_data);
      break;
    case ADDR_ROTATION:
      motor_param_.rotation = int16_data;
      CG_DEBUG_PRINTF("Receive ADDR_ROTATION = [%d]\n", int16_data);
      break;
    case ADDR_LOC_KP:
      motor_param_.loc_kp = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_LOC_KP = [%f]\n", float_data);
      break;
    case ADDR_SPD_KP:
      motor_param_.spd_kp = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_SPD_KP = [%f]\n", float_data);
      break;
    case ADDR_SPD_KI:
      motor_param_.spd_ki = float_data;
      CG_DEBUG_PRINTF("Receive ADDR_SPD_KI = [%f]\n", float_data);
      break;
    default:
      CG_DEBUG_PRINTF("Unknown parameter value index=[0x%04x]\n", index);
      is_updated = false;
      break;
  }

  if (is_updated) {
    motor_param_.stamp_usec = micros();
  }
}

void CybergearDriver::print_can_packet(uint32_t id, const uint8_t * data, uint8_t len)
{
  CG_DEBUG_PRINTF("Id: 0x%x ", id);
  CG_DEBUG_PRINTF("Data: ");
  for (uint8_t idx = 0; idx < len; ++idx) {
    CG_DEBUG_PRINTF("%x ", data[idx]);
  }
  CG_DEBUG_PRINTLN("");
}
