#ifndef CYBER_GEAR_CONTROLLER_HH
#define CYBER_GEAR_CONTROLLER_HH

#include "cybergear_driver.hh"

#include <cstdint>
#include <sys/types.h>
#include <unordered_map>
#include <mcp_can.h>
#include <vector>

struct CybergearConfig
{
  uint8_t id;
  float limit_speed;
  float limit_current;
  float limit_torque;
  float current_kp;
  float current_ki;
  float current_filter_gain;
};

struct CybergearMotionCommand
{
  float position;
  float velocity;
  float effort;
  float kp;
  float kd;
};

class CybergearController
{
public:
  explicit CybergearController(uint8_t master_can_id);
  virtual ~CybergearController();

  bool init(const std::vector<uint8_t> ids, uint8_t mode, MCP_CAN* can);

  bool set_motor_config(const std::vector<CybergearConfig>& configs);
  bool set_motor_config(const CybergearConfig& config);

  bool set_speed_limit(uint8_t id, float limit);
  bool set_torque_limit(uint8_t id, float limit);
  bool set_current_limit(uint8_t id, float limit);

  bool set_current_control_param(uint8_t id, float kp, float ki, float gain);

  bool enable_motors();
  bool disable_motors();

  bool send_motion_command(const std::vector<uint8_t> ids, const std::vector<CybergearMotionCommand> cmds);
  bool send_motion_command(uint8_t id, const CybergearMotionCommand& cmd);

  bool send_position_command(const std::vector<uint8_t> ids, const std::vector<float> positions);
  bool send_position_command(uint8_t id, float position);

  bool send_speed_command(const std::vector<uint8_t> ids, const std::vector<float> speeds);
  bool send_speed_command(uint8_t id, float speed);

  bool send_current_command(const std::vector<uint8_t> ids, const std::vector<float> currents);
  bool send_current_command(uint8_t id, float current);

  bool get_motor_status(std::vector<MotorStatus> & status);
  bool get_motor_status(uint8_t id, MotorStatus& status);

  bool process_can_packet();

private:
  typedef std::unordered_map<uint8_t, CybergearDriver> CybergearDriverMap;
  typedef std::unordered_map<uint8_t, CybergearConfig> CybergearConfigMap;

  bool check_motor_id(uint8_t id);

  std::vector<uint8_t> motor_ids_;
  CybergearDriverMap drivers_;
  CybergearConfigMap configs_;
  uint8_t control_mode_;

  MCP_CAN *can_;
  uint8_t master_can_id_;
  uint8_t receive_buffer_[64];
};

#endif // !CYBER_GEAR_CONTROLLER_HH
