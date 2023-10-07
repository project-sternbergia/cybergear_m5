#ifndef CYBER_GEAR_DRIVER_H
#define CYBER_GEAR_DRIVER_H

#include "cybergear_driver_defs.hh"
#include "mcp_can.h"


/**
 * @brief MotorStatus class
 */
struct MotorStatus
{
  uint8_t motor_id;   //!< motor id
  float position;     //!< encoder position (-4pi to 4pi)
  float velocity;     //!< motor velocity (-30rad/s to 30rad/s)
  float effort;       //!< motor effort (-12Nm - 12Nm)
  float temperature;  //!< temperature
};


/**
 * @brief Cybergear driver class
 */
class CybergearDriver
{
public:
  /**
   * @brief Construct a new Cybergear Driver object
   *
   * @param master_can_id   master_can_id (this host)
   * @param motor_can_id   master_can_id (this host)
   */
  CybergearDriver(uint8_t master_can_id, uint8_t target_can_id);
  virtual ~CybergearDriver();

  /**
   * @brief Init this class
   *
   * @param MCP_CAN mcp_can object for can communication
   */
  void init(MCP_CAN *can);

  /**
   * @brief Init motor
   *        this function call reset motor then call set mode for ready to enable
   *
   * @param run_mode   run mode. please chose from MotorMode enum
   */
  void init_motor(uint8_t run_mode);

  /**
   * @brief Enable motor
   */
  void enable_motor();

  /**
   * @brief Reset motor
   */
  void reset_motor();

  /**
   * @brief Set run mode
   *
   * @param run_mode run mode
   */
  void set_run_mode(uint8_t run_mode);

  /**
   * @brief control motor using motor control mode
   *        this mode control target position, speed, torque with control parameters (kp, kd)
   *        arguments clip by their limit in this function
   *
   * @param position target encoder position (-4pi to 4pi)
   * @param speed target speed (-30rad/s to 30rad/s)
   * @param torque target torque (-12Nm to 12Nm)
   * @param kp control parameter kp (0.0 to 500.0)
   * @param kd control parameter kd (0.0 to 5.0)
   */
  void motor_control(float position, float speed, float torque, float kp, float kd);

  /**
   * @brief Set the limit speed for speed control mode
   *
   * @param speed speed limit for speed control mode (run mode 2). limit range is 0.0[rad/s] to 30.0[rad/s]
   */
  void set_limit_speed(float speed);

  /**
   * @brief Set position reference for position control mode
   *
   * @param position target encoder position for position control mode (run mode 1). no limit description on datasheet.
   */
  void set_position_ref(float position);

  /**
   * @brief Set speed reference for speeed control mode
   *
   * @param speed target speed for speed control mode (run mode 2). limit range is -30.0rad/s to 30.0rad/s
   */
  void set_speed_ref(float speed);

  /**
   * @brief Set current reference for torque control mode
   *
   * @param current target current for current control mode (run mode 3). limit range is -23A to 27A
   */
  void set_current_ref(float current);

  /**
   * @brief Read ram data from target motor
   *        If you need detailed information of RAM, please check the datasheet 4.1.11
   *
   * @param index target address of data (chose from ADDR_XXX definitions)
   */
  void read_ram_data(uint16_t index);

  /**
   * @brief Get current run mode
   *
   * @return uint8_t  run mode
   */
  uint8_t get_run_mode() const;

  /**
   * @brief Get the motor id
   *
   * @return uint8_t motor id
   */
  uint8_t get_motor_id() const;

  /**
   * @brief Update motor status from can buffer
   *
   * @return true   motor status updated
   * @return false  motor status not updated
   */
  bool update_motor_status();

  /**
   * @brief Get the motor status object
   *
   * @return MotorStatus  motor status
   */
  MotorStatus get_motor_status() const;

private:
  /**
   * @brief Write float data to target motor
   *
   * @param can_id target motor can id
   * @param addr target address of write data
   * @param value write value
   * @param min min value to clip data
   * @param max max value to clip data
   */
  void write_float_data(uint8_t can_id, uint16_t addr, float value, float min, float max);

  /**
   * @brief utility function for data convearsion (float to uint)
   *        This function is copied from datasheet
   *
   * @param x      target data to data conversion
   * @param x_min  min value
   * @param x_max  max value
   * @param bits   input data bit size (this parameter use for calc maximum value of input data type)
   * @return int   result
   */
  int float_to_uint(float x, float x_min, float x_max, int bits);

  float uint_to_float(uint16_t x, float x_min, float x_max);

  /**
   * @brief Send command to cybergear
   *
   * @param can_id  target motor can id
   * @param cmd_id  command id
   * @param option  option value this parameter use to implement additional data to can id
   * @param len     length of send data
   * @param data    send data
   */
  void send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data);

  uint8_t receive_motor_data(MotorStatus& mot);

  /**
   * @brief Print byte array as serial output
   *
   * @param data data
   * @param len length
   */
  void print_can_packet(uint32_t id, uint8_t *data, uint8_t len);


  MCP_CAN* can_;                //!< can connection instance
  uint8_t master_can_id_;       //!< master can id
  uint8_t target_can_id_;       //!< target can id
  uint8_t run_mode_;            //!< run mode
  uint8_t receive_buffer_[64];  //!< receive buffer
  MotorStatus motor_status_;    //!< current motor status
};

#endif // !CYBER_GEAR_DRIVER_H
