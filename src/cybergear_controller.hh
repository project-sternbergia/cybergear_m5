#ifndef CYBER_GEAR_CONTROLLER_HH
#define CYBER_GEAR_CONTROLLER_HH

#include <sys/types.h>

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "cybergear_can_interface.hh"
#include "cybergear_driver.hh"

/**
 * @brief Cybergear config for hardware limit
 */
struct CybergearHardwareConfig
{
  CybergearHardwareConfig()
  : id(0xFF),
    limit_speed(2.0f),
    limit_current(27.0f),
    limit_torque(12.0f),
    current_kp(0.025f),
    current_ki(0.0258f),
    current_filter_gain(0.1f)
  {
  }

  uint8_t id;                 //!< motor id
  int8_t direciton;           //!< motor direction (-1: ccw / 1: cw)
  float limit_speed;          //!< limit speed (rad/s)
  float limit_current;        //!< limit current (A)
  float limit_torque;         //!< limit torque (Nm)
  float current_kp;           //!< control parameter kp for current control
  float current_ki;           //!< control parameter ki for current control
  float current_filter_gain;  //!< current filter gain
};

/**
 * @brief Cybergear config for software limit
 */
struct CybergearSoftwareConfig
{
  CybergearSoftwareConfig()
  : id(0xFF),
    direction(CW),
    limit_speed(30.0f),
    limit_current(27.0f),
    limit_torque(12.0f),
    upper_position_limit(4.0 * M_PI),
    lower_position_limit(-4.0 * M_PI),
    calib_direction(CW),
    position_offset(0.0f)
  {
  }

  CybergearSoftwareConfig(
    uint8_t id, int8_t dir, float limit_speed, float limit_current, float limit_torque,
    float upper_position_limit, float lower_position_limit, int8_t calib_direction,
    float position_offset)
  : id(id),
    direction(dir),
    limit_speed(limit_speed),
    limit_current(limit_current),
    limit_torque(limit_torque),
    upper_position_limit(upper_position_limit),
    lower_position_limit(lower_position_limit),
    calib_direction(calib_direction),
    position_offset(position_offset)
  {
  }

  uint8_t id;                  //!< motor id
  int8_t direction;            //!< motor direction
  float limit_speed;           //!< limit speed (rad/s)
  float limit_current;         //!< limit current (Aa)
  float limit_torque;          //!< limit torque (Nm)
  float upper_position_limit;  //!< motor upper limit [rad]
  float lower_position_limit;  //!< motor lower limit [rad]
  int8_t calib_direction;      //!< calibration direction
  float position_offset;       //!< motor offset [rad]
};

/**
 * @brief Cybegear motion command struct
 */
struct CybergearMotionCommand
{
  float position;  //!< target position
  float velocity;  //!< target velocity (rad/sec)
  float effort;    //!< target effort
  float kp;        //!< motion control kp
  float kd;        //!< motion control kd
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
   * @param wait_response_time_usec wait response time after send command
   * @return true   succeeded to init this class
   * @return false  failed to init this class
   */
  bool init(
    const std::vector<uint8_t> & ids, uint8_t mode, CybergearCanInterface * can,
    uint16_t wait_response_time_usec = 0);

  /**
   * @brief Init cybergear controller class

   * @param ids           motor ids
   * @param sw_configs    motor software config (e.g speed_limit, current_limit, position_limit, offset, etc)
   * @param mode          control mode (motion, position, speed or current)
   * @param can           can control class
   * @param wait_response_time_usec wait response time after send command
   * @return true         succeeded to init this class
   * @return false        failed to init this class
   */
  bool init(
    const std::vector<uint8_t> & ids, const std::vector<CybergearSoftwareConfig> & sw_configs,
    uint8_t mode, CybergearCanInterface * can, uint16_t wait_response_time_usec = 0);

  /**
   * @brief Set the run mode
   *
   * @param ids     motor ids
   * @param modes   run mode
   * @return true   succeeded to init this class
   * @return false  failed to init this class
   */
  bool set_run_mode(const std::vector<uint8_t> & ids, const std::vector<uint8_t> modes);

  /**
   * @brief Set the run mode object
   *
   * @param mode    motor id
   * @return true   succeeded to init this class
   * @return false  failed to init this class
   */
  bool set_run_mode(uint8_t mode);

  /**
   * @brief Set the run mode object
   *
   * @param ids     motor ids
   * @param modes   run mode
   * @return true   succeeded to init this class
   * @return false  failed to init this class
   */
  bool set_run_mode(uint8_t id, uint8_t mode);

  /**
   * @brief Set cybergear motor configs
   *
   * @param configs   cybergear motor configs. this fnction write config params to cybergear ram via ca.
   * @return true     success to send
   * @return false    failed to send
   */
  bool set_motor_config(const std::vector<CybergearHardwareConfig> & configs);

  /**
   * @brief Set cybergear motor config
   *
   * @param config   cybergear motor config. this fnction write config params to cybergear ram via ca.
   * @return true     success to send
   * @return false    failed to send
   */
  bool set_motor_config(const CybergearHardwareConfig & config);

  // set motor config for cybergear can apis
  bool set_speed_limit(uint8_t id, float limit);
  bool set_torque_limit(uint8_t id, float limit);
  bool set_current_limit(uint8_t id, float limit);
  bool set_position_control_gain(uint8_t id, float kp);
  bool set_velocity_control_gain(uint8_t id, float kp, float ki);
  bool set_current_control_param(uint8_t id, float kp, float ki, float gain);

  /**
   * @bref MotorStatus enable motors
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
  bool send_motion_command(
    const std::vector<uint8_t> ids, const std::vector<CybergearMotionCommand> cmds);

  /**
   * @brief send motion command to target motor
   *
   * @param id      target motor id
   * @param cmd     target command
   * @return true   success to set (if you want to check execution result, please check packet data)
   * @return false  failed to set (if you want to check execution result, please check packet data)
   */
  bool send_motion_command(uint8_t id, const CybergearMotionCommand & cmd);

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

  /**
   * @brief Set current mech position to zero (cybergear api)
   *
   * @param id        target motor id
   * @return true     success to set (if you want to check execution result, please check packet data)
   * @return false    failed to set (if you want to check execution result, please check packet data)
   */
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
  bool get_motor_status(uint8_t id, MotorStatus & status);

  /**
   * @brief Get the software config object
   *
   * @param id       target motor id
   * @param config   motor config
   * @return true
   * @return false
   */
  bool get_software_config(uint8_t id, CybergearSoftwareConfig & config);

  /**
   * @brief Process can packet
   *
   * @return true   success to update data
   * @return false  failed to update data
   */
  bool process_packet();

  bool check_update_flag(uint8_t id);
  bool reset_update_flag(uint8_t id);
  std::vector<uint8_t> motor_ids() const;

  unsigned long send_count() const;
  unsigned long recv_count() const { return recv_count_; }

private:
  typedef std::unordered_map<uint8_t, CybergearDriver> CybergearDriverMap;
  typedef std::unordered_map<uint8_t, CybergearHardwareConfig> CybergearHardwareConfigMap;
  typedef std::unordered_map<uint8_t, CybergearSoftwareConfig> CybergearSoftwareConfigMap;

  bool check_motor_id(uint8_t id);

  std::vector<uint8_t> motor_ids_;
  std::unordered_map<uint8_t, bool> motor_update_flag_;
  CybergearDriverMap drivers_;
  CybergearHardwareConfigMap hw_configs_;
  CybergearSoftwareConfigMap sw_configs_;
  uint8_t control_mode_;

  CybergearCanInterface * can_;
  uint8_t master_can_id_;
  uint8_t receive_buffer_[64];
  unsigned long recv_count_;
};

#endif  // !CYBER_GEAR_CONTROLLER_HH
