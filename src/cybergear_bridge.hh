#ifndef CYBER_GEAR_BRIDGE_HH
#define CYBER_GEAR_BRIDGE_HH

#include <RingBuf.h>
#include <Stream.h>
#include <sys/types.h>

#include <cstdint>
#include <vector>

#include "cybergear_bridge_packet.hh"
#include "cybergear_controller.hh"

typedef std::vector<uint8_t> ByteArray;

/**
 * @brief CybergearBridge for connnection with host-pc
 */
class CybergearBridge
{
public:
  CybergearBridge(CybergearController * controller, Stream * stream);
  virtual ~CybergearBridge();

  /**
   * @brief Process request command from host pc
   *        This function process request and send can command to cybergear.
   *        After call this function, call process_motor_response function to response pc requeset.
   */
  void process_request_command();

  /**
   * @brief Process motor response
   *        This function process can response and send process result to pc via Stream class.
   */
  void process_motor_response();

private:
  // process motor requests
  void process_enable_motor_request(const ByteArray & request_packet);
  void process_reset_motor_request(const ByteArray & request_packet);
  void process_position_control_request(const ByteArray & request_packet);
  void process_speed_control_request(const ByteArray & request_packet);
  void process_current_control_request(const ByteArray & request_packet);
  void process_motion_control_request(const ByteArray & request_packet);
  void process_set_mech_position_to_zero_request(const ByteArray & request_packet);
  void process_set_limit_speed(const ByteArray & request_packet);
  void process_set_limit_current(const ByteArray & request_packet);
  void process_set_limit_torque(const ByteArray & request_packet);
  void process_set_position_control_gain(const ByteArray & request_packet);
  void process_set_velocity_control_gain(const ByteArray & request_packet);

  // send motor status to pc
  void send_motor_status_response(uint8_t id, uint8_t seq);

  // get next packet from buffer for host-pc request
  bool get_next_packet(ByteArray & request_packet);

  CybergearController * p_controller_;    //!< Controller object pointer
  Stream * p_stream_;                     //!< Stream object pointer to communicate with ros
  RingBuf<uint8_t, 512> receive_buffer_;  //!< receive buffer
};

#endif  // CYBER_GEAR_BRIDGE_HH
