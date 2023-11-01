#ifndef CYBER_GEAR_BRIDGE_HH
#define CYBER_GEAR_BRIDGE_HH

#include <Stream.h>
#include <RingBuf.h>
#include <sys/types.h>
#include <vector>
#include <cstdint>
#include "cybergear_controller.hh"
#include "cybergear_bridge_packet.hh"

typedef std::vector<uint8_t> ByteArray;

class CybergearBridge
{
public:
  CybergearBridge(CybergearController *controller, Stream *stream);
  virtual ~CybergearBridge();

  void process_request_command();
  void process_motor_response();

private:
  // process motor requests
  void process_enable_motor_request(const ByteArray& request_packet);
  void process_reset_motor_request(const ByteArray& request_packet);
  void process_position_control_request(const ByteArray& request_packet);
  void process_speed_control_request(const ByteArray& request_packet);
  void process_current_control_request(const ByteArray& request_packet);
  void process_motion_control_request(const ByteArray& request_packet);
  void process_set_mech_position_to_zero_request(const ByteArray& request_packet);

  // send motor status to pc
  void send_motor_status_response(uint8_t id, uint8_t seq);
  bool get_next_packet(ByteArray & request_packet);

  CybergearController *p_controller_;     //!< Controller object pointer
  Stream *p_stream_;                      //!< Stream object pointer to communicate with ros
  RingBuf<uint8_t, 512> receive_buffer_;  //!< receive buffer
};

#endif // CYBER_GEAR_BRIDGE_HH