#include "cybergear_controller.hh"

#include <Arduino.h>

#include "cybergear_can_interface.hh"
#include "cybergear_driver.hh"
#include "cybergear_driver_defs.hh"
#include "cybergear_driver_utils.hh"

CybergearController::CybergearController(uint8_t master_can_id)
: can_(NULL), master_can_id_(master_can_id), recv_count_(0)
{
}

CybergearController::~CybergearController() {}

bool CybergearController::init(
  const std::vector<uint8_t> & ids, uint8_t mode, CybergearCanInterface * can,
  uint16_t wait_response_time_usec)
{
  std::vector<CybergearSoftwareConfig> configs;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    CybergearSoftwareConfig config;
    config.id = ids[idx];
    configs.push_back(config);
  }

  return init(ids, configs, mode, can, wait_response_time_usec);
}

bool CybergearController::init(
  const std::vector<uint8_t> & ids, const std::vector<CybergearSoftwareConfig> & sw_configs,
  uint8_t mode, CybergearCanInterface * can, uint16_t wait_response_time_usec)
{
  if (ids.size() != sw_configs.size()) {
    return false;
  }

  // setup variables
  can_ = can;
  motor_ids_ = ids;  // for send sequence
  control_mode_ = mode;

  // create motor class
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    CybergearDriver driver = CybergearDriver(master_can_id_, ids[idx]);
    driver.init(can_, wait_response_time_usec);
    driver.init_motor(mode);
    drivers_[ids[idx]] = driver;
    motor_update_flag_[ids[idx]] = false;
  }

  // copy sw configs
  for (uint8_t idx = 0; idx < sw_configs.size(); ++idx) {
    sw_configs_[sw_configs[idx].id] = sw_configs[idx];
  }

  return true;
}

bool CybergearController::set_run_mode(
  const std::vector<uint8_t> & ids, const std::vector<uint8_t> modes)
{
  if (ids.size() != modes.size()) return false;
  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= set_run_mode(ids[idx], modes[idx]);
  }
  return ret;
}

bool CybergearController::set_run_mode(uint8_t mode)
{
  for (uint8_t idx = 0; idx < motor_ids_.size(); ++idx) {
    drivers_[motor_ids_[idx]].set_run_mode(mode);
  }
  return true;
}

bool CybergearController::set_run_mode(uint8_t id, uint8_t mode)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_run_mode(mode);
  return true;
}

bool CybergearController::set_motor_config(const std::vector<CybergearHardwareConfig> & configs)
{
  bool ret = true;
  for (uint8_t idx = 0; idx < configs.size(); ++idx) {
    if (set_motor_config(configs[idx])) {
      hw_configs_[configs[idx].id] = configs[idx];
    } else {
      ret = false;
    }
  }
  return ret;
}

bool CybergearController::set_motor_config(const CybergearHardwareConfig & config)
{
  if (!check_motor_id(config.id)) return false;

  // send config command
  drivers_[config.id].set_limit_speed(config.limit_speed);
  drivers_[config.id].set_limit_torque(config.limit_torque);
  drivers_[config.id].set_limit_current(config.limit_current);
  drivers_[config.id].set_current_kp(config.current_kp);
  drivers_[config.id].set_current_ki(config.current_ki);
  drivers_[config.id].set_current_filter_gain(config.current_filter_gain);
  return true;
}

bool CybergearController::set_speed_limit(uint8_t id, float limit)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_limit_speed(limit);
  return true;
}

bool CybergearController::set_torque_limit(uint8_t id, float limit)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_limit_torque(limit);
  return true;
}

bool CybergearController::set_current_limit(uint8_t id, float limit)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_limit_current(limit);
  return true;
}

bool CybergearController::set_position_control_gain(uint8_t id, float kp)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_position_kp(kp);
  return true;
}

bool CybergearController::set_velocity_control_gain(uint8_t id, float kp, float ki)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_velocity_kp(kp);
  drivers_[id].set_velocity_ki(ki);
  return true;
}

bool CybergearController::set_current_control_param(uint8_t id, float kp, float ki, float gain)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_current_kp(kp);
  drivers_[id].set_current_ki(ki);
  drivers_[id].set_current_filter_gain(gain);
  return true;
}

bool CybergearController::enable_motor(uint8_t id, uint8_t mode)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].init_motor(mode);
  drivers_[id].enable_motor();
  return true;
}

bool CybergearController::enable_motors()
{
  for (uint8_t idx = 0; idx < motor_ids_.size(); ++idx) {
    drivers_[motor_ids_[idx]].enable_motor();
  }
  return true;
}

bool CybergearController::reset_motor(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].reset_motor();
  return true;
}

bool CybergearController::reset_motors()
{
  for (uint8_t idx = 0; idx < motor_ids_.size(); ++idx) {
    drivers_[motor_ids_[idx]].reset_motor();
  }
  return true;
}

bool CybergearController::send_motion_command(
  const std::vector<uint8_t> ids, const std::vector<CybergearMotionCommand> cmds)
{
  // check size
  if (ids.size() != cmds.size()) {
    return false;
  }

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_motion_command(ids[idx], cmds[idx]);
  }
  return ret;
}

bool CybergearController::send_motion_command(uint8_t id, const CybergearMotionCommand & cmd)
{
  if (!check_motor_id(id)) return false;
  float pos = std::max(
    std::min(cmd.position, sw_configs_[id].upper_position_limit),
    sw_configs_[id].lower_position_limit);
  float cmd_pos = (pos - sw_configs_[id].position_offset) * sw_configs_[id].direction;
  float vel = std::max(
    std::min(sw_configs_[id].direction * cmd.velocity, sw_configs_[id].limit_speed),
    -sw_configs_[id].limit_speed);
  float eff = std::max(
    std::min(sw_configs_[id].direction * cmd.effort, sw_configs_[id].limit_torque),
    -sw_configs_[id].limit_torque);
  drivers_[id].motor_control(cmd_pos, vel, eff, cmd.kp, cmd.kd);
  return true;
}

bool CybergearController::send_position_command(
  const std::vector<uint8_t> ids, const std::vector<float> positions)
{
  // check size
  if (ids.size() != positions.size()) return false;

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_position_command(ids[idx], positions[idx]);
  }
  return ret;
}

bool CybergearController::send_position_command(uint8_t id, float position)
{
  if (!check_motor_id(id)) return false;
  float pos = std::max(
    std::min(position, sw_configs_[id].upper_position_limit), sw_configs_[id].lower_position_limit);
  float cmd_pos = (pos - sw_configs_[id].position_offset) * sw_configs_[id].direction;
  drivers_[id].set_position_ref(cmd_pos);
  return true;
}

bool CybergearController::send_speed_command(
  const std::vector<uint8_t> ids, const std::vector<float> speeds)
{
  // check size
  if (ids.size() != speeds.size()) return false;

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_speed_command(ids[idx], speeds[idx]);
  }
  return ret;
}

bool CybergearController::send_speed_command(uint8_t id, float speed)
{
  if (!check_motor_id(id)) return false;
  float vel = std::max(
    std::min(sw_configs_[id].direction * speed, sw_configs_[id].limit_speed),
    -sw_configs_[id].limit_speed);
  drivers_[id].set_speed_ref(vel);
  return true;
}

bool CybergearController::send_current_command(
  const std::vector<uint8_t> ids, const std::vector<float> currents)
{
  // check size
  if (ids.size() != currents.size()) return false;

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_current_command(ids[idx], currents[idx]);
  }
  return ret;
}

bool CybergearController::send_current_command(uint8_t id, float current)
{
  if (!check_motor_id(id)) return false;
  float cur = std::max(
    std::min(sw_configs_[id].direction * current, sw_configs_[id].limit_current),
    -sw_configs_[id].limit_current);
  drivers_[id].set_current_ref(cur);
  return true;
}

bool CybergearController::set_mech_position_to_zero(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_mech_position_to_zero();
  return true;
}

bool CybergearController::get_motor_status(std::vector<MotorStatus> & status)
{
  status.clear();
  for (uint8_t idx = 0; idx < motor_ids_.size(); ++idx) {
    if (drivers_.find(motor_ids_[idx]) == drivers_.end()) return false;
    MotorStatus mot = drivers_[motor_ids_[idx]].get_motor_status();
    get_motor_status(motor_ids_[idx], mot);
    status.push_back(mot);
  }
  return true;
}

bool CybergearController::get_motor_status(uint8_t id, MotorStatus & status)
{
  if (!check_motor_id(id)) return false;
  status = drivers_[id].get_motor_status();
  status.position = sw_configs_[id].direction * status.position + sw_configs_[id].position_offset;
  status.velocity *= sw_configs_[id].direction;
  status.effort *= sw_configs_[id].direction;
  return true;
}

bool CybergearController::get_software_config(uint8_t id, CybergearSoftwareConfig & config)
{
  if (!check_motor_id(id)) return false;
  config = sw_configs_[id];
  return true;
}

bool CybergearController::process_packet()
{
  bool is_updated = false;
  while (can_->available()) {
    unsigned long id;
    uint8_t len;
    if (!can_->read_message(id, receive_buffer_, len)) {
      continue;
    }

    // if id is not mine
    uint8_t receive_can_id = id & 0xff;
    if (receive_can_id != master_can_id_) {
      CG_DEBUG_PRINTF(
        "Invalid master can id. Expected=[0x%02x] Actual=[0x%02x] Raw=[%x]\n", master_can_id_,
        receive_can_id, id);
      continue;
    }

    uint8_t motor_can_id = (id & 0xff00) >> 8;
    if (drivers_.find(motor_can_id) == drivers_.end()) {
      continue;
    }

    // parse packet --------------
    if (drivers_[motor_can_id].update_motor_status(id, receive_buffer_, len)) {
      motor_update_flag_[motor_can_id] = true;
      is_updated = true;
      recv_count_++;
    }
  }

  return is_updated;
}

bool CybergearController::check_update_flag(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  return motor_update_flag_[id];
}

bool CybergearController::reset_update_flag(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  motor_update_flag_[id] = false;
  return true;
}

std::vector<uint8_t> CybergearController::motor_ids() const { return motor_ids_; }

bool CybergearController::check_motor_id(uint8_t id)
{
  if (drivers_.find(id) == drivers_.end()) {
    return false;
  }

  if (sw_configs_.find(id) == sw_configs_.end()) {
    return false;
  }

  return true;
}

unsigned long CybergearController::send_count() const
{
  unsigned long cnt = 0;
  for (uint8_t idx = 0; idx < motor_ids_.size(); ++idx) {
    cnt += drivers_.at(motor_ids_.at(idx)).send_count();
  }
  return cnt;
}
