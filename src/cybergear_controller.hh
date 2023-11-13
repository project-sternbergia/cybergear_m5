#ifndef CYBER_GEAR_CONTROLLER_HH
#define CYBER_GEAR_CONTROLLER_HH

#include "cybergear_driver.hh"

#include <cstdint>
#include <sys/types.h>
#include <unordered_map>
#include <mcp_can.h>
#include <vector>

/**
 * @brief Cybergear config struct for initializing
 */
struct CybergearConfig
{
  uint8_t id;                   //!< motor id
  float limit_speed;            //!< limit speed (rad/s)
  float limit_current;          //!< limit current (A)
  float limit_torque;           //!< limit torque (Nm)
  float current_kp;             //!< control parameter kp for current control
  float current_ki;             //!< control parameter ki for current control
  float current_filter_gain;    //!< current filter gain
};

/**
 * @brief Cybegear motion command struct
 */
struct CybergearMotionCommand
{
  float position;   //!< target position
  float velocity;   //!< target velocity (rad/sec)
  float effort;     //!< target effort
  float kp;         //!< motion control kp
  float kd;         //!< motion control kd
};

/**
 * @brief Multi-Cybergear controller class
 */
class CybergearController
{
public:
  /**
   * @brief Construct a new Cybergear Controller object
   *
   * @param master_can_id master can id
   */
  explicit CybergearController(uint8_t master_can_id);

  /**
   * @brief Destroy the Cybergear Controller object
   */
  virtual ~CybergearController();

  /**
   * @brief Init cybergear controller class
   *
   * @param ids     motor ids
   * @param mode    control mode (motion, position, speed or current)
   * @param can     can control class
   * @return true   succeeded to init this class
   * @return false  failed to init this class
   */
  bool init(const std::vector<uint8_t> ids, uint8_t mode, MCP_CAN* can);

  /**
   * @brief Set cybergear motor configs
   *
   * @param configs   cybergear motor configs. this fnction write config params to cybergear ram via ca.
   * @return true     success to send
   * @return false    failed to send
   */
  bool set_motor_config(const std::vector<CybergearConfig>& configs);

  /**
   * @brief Set cybergear motor config
   *
   * @param config   cybergear motor config. this fnction write config params to cybergear ram via ca.
   * @return true     success to send
   * @return false    failed to send
   */
  bool set_motor_config(const CybergearConfig& config);

  // set motor config
  bool set_speed_limit(uint8_t id, float limit);
  bool set_torque_limit(uint8_t id, float limit);
  bool set_current_limit(uint8_t id, float limit);
  bool set_position_control_gain(uint8_t id, float kp);
  bool set_velocity_control_gain(uint8_t id, float kp, float ki);
  bool set_current_control_param(uint8_t id, float kp, float ki, float gain);

  /**
   * @brief enable motors
   *
   * @return true   success
   * @return false  failed
   */
  bool enable_motors();

  /**
   * @brief enable motor
   *
   * @param id      target motor
   * @param mode    motor mode
   * @return true   success
   * @return false  failed
   */
  bool enable_motor(uint8_t id, uint8_t mode);

  /**
   * @brief disable motors
   *
   * @return true   success
   * @return false  failed
   */
  bool reset_motors();

  /**
   * @brief disable motors
   *
   * @return true   success
   * @return false  failed
   */
  bool reset_motor(uint8_t id);

  /**
   * @brief send motion command for each motors
   *
   * @param ids     target motor ids
   * @param cmds    target commands
   * @return true   success to set (if you want to check execution result, please check packet data)
   * @return false  failed to set (if you want to check execution result, please check packet data)
   */
  bool send_motion_command(const std::vector<uint8_t> ids, const std::vector<CybergearMotionCommand> cmds);

  /**
   * @brief send motion command to target motor
   *
   * @param id      target motor id
   * @param cmd     target command
   * @return true   success to set (if you want to check execution result, please check packet data)
   * @return false  failed to set (if you want to check execution result, please check packet data)
   */
  bool send_motion_command(uint8_t id, const CybergearMotionCommand& cmd);

  /**
   * @brief send position command to target motor
   *
   * @param ids         target motor ids
   * @param positions   target position
   * @return true       success to set (if you want to check execution result, please check packet data)
   * @return false      failed to set (if you want to check execution result, please check packet data)
   */
  bool send_position_command(const std::vector<uint8_t> ids, const std::vector<float> positions);

  /**
   * @brief send position command to target motor
   *
   * @param ids         target motor id
   * @param positions   target position
   * @return true       success to set (if you want to check execution result, please check packet data)
   * @return false      failed to set (if you want to check execution result, please check packet data)
   */
  bool send_position_command(uint8_t id, float position);

  /**
   * @brief send speed commands to target motor
   *
   * @param ids         target motor ids
   * @param speeds      target speeds
   * @return true       success to set (if you want to check execution result, please check packet data)
   * @return false      failed to set (if you want to check execution result, please check packet data)
   */
  bool send_speed_command(const std::vector<uint8_t> ids, const std::vector<float> speeds);

  /**
   * @brief send speed command to target motor
   *
   * @param id          target motor id
   * @param speed       target speeds
   * @return true       success to set (if you want to check execution result, please check packet data)
   * @return false      failed to set (if you want to check execution result, please check packet data)
   */
  bool send_speed_command(uint8_t id, float speed);

  /**
   * @brief send current commands to target motor
   *
   * @param ids         target motor id
   * @param currents    target currents [A]
   * @return true       success to set (if you want to check execution result, please check packet data)
   * @return false      failed to set (if you want to check execution result, please check packet data)
   */
  bool send_current_command(const std::vector<uint8_t> ids, const std::vector<float> currents);

  /**
   * @brief send current command to target motor
   *
   * @param id          target motor id
   * @param current     target currents [A]
   * @return true       success to set (if you want to check execution result, please check packet data)
   * @return false      failed to set (if you want to check execution result, please check packet data)
   */
  bool send_current_command(uint8_t id, float current);

  bool set_mech_position_to_zero(uint8_t id);

  /**
   * @brief Get the motor status stored by processs can packet
   *
   * @param status  motor status
   * @return true   succeed to get motor status
   * @return false  failed to get motor status
   */
  bool get_motor_status(std::vector<MotorStatus> & status);

  /**
   * @brief Get the motor status stored by processs can packet
   *
   * @param id      target motor id
   * @param status  motor status
   * @return true   succeed to get motor status
   * @return false  failed to get motor status
   */
  bool get_motor_status(uint8_t id, MotorStatus& status);

  /**
   * @brief Process can packet
   *
   * @return true   success to update data
   * @return false  failed to update data
   */
  bool process_can_packet();

  bool check_update_flag(uint8_t id);
  bool reset_update_flag(uint8_t id);
  std::vector<uint8_t> motor_ids() const;

  unsigned long send_count() const;
  unsigned long recv_count() const { return recv_count_; }

private:
  typedef std::unordered_map<uint8_t, CybergearDriver> CybergearDriverMap;
  typedef std::unordered_map<uint8_t, CybergearConfig> CybergearConfigMap;

  bool check_motor_id(uint8_t id);

  std::vector<uint8_t> motor_ids_;
  std::unordered_map<uint8_t, bool> motor_update_flag_;
  CybergearDriverMap drivers_;
  CybergearConfigMap configs_;
  uint8_t control_mode_;

  MCP_CAN *can_;
  uint8_t master_can_id_;
  uint8_t receive_buffer_[64];
  unsigned long recv_count_;
};

#endif // !CYBER_GEAR_CONTROLLER_HH
