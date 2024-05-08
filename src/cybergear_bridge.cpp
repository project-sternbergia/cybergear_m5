#include "cybergear_bridge.hh"

#include <cstdint>

#include "cybergear_bridge.hh"
#include "cybergear_bridge_packet.hh"

CybergearBridge::CybergearBridge(CybergearController * controller, Stream * stream)
: p_controller_(controller), p_stream_(stream), receive_buffer_()
{
}

CybergearBridge::~CybergearBridge() {}

void CybergearBridge::process_request_command()
{
  // cretate bytearray from receive buffer
  ByteArray request_packet;
  while (get_next_packet(request_packet)) {
    // process motor command
    uint8_t packet_type =
      request_packet[static_cast<uint8_t>(Packet::CommandPacketIndex::PacketType)];
    if (packet_type == static_cast<uint8_t>(RequestPacket::Type::ControlCurrent)) {
      process_current_control_request(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::ControlPosition)) {
      process_position_control_request(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::ControlSpeed)) {
      process_speed_control_request(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::ControlMotion)) {
      process_motion_control_request(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::Enable)) {
      process_enable_motor_request(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::Reset)) {
      process_reset_motor_request(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::SetMechPosToZero)) {
      process_set_mech_position_to_zero_request(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::SetLimitSpeed)) {
      process_set_limit_speed(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::SetLimitCurrent)) {
      process_set_limit_current(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::SetLimitTorque)) {
      process_set_limit_torque(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::SetPositionControlGain)) {
      process_set_position_control_gain(request_packet);

    } else if (packet_type == static_cast<uint8_t>(RequestPacket::Type::SetVelocityControlGain)) {
      process_set_velocity_control_gain(request_packet);
    }
  }
}

void CybergearBridge::process_motor_response()
{
  p_controller_->process_packet();
  std::vector<uint8_t> ids = p_controller_->motor_ids();
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    if (p_controller_->check_update_flag(ids[idx])) {
      send_motor_status_response(ids[idx], 0);
      p_controller_->reset_update_flag(ids[idx]);
    }
  }
}

void CybergearBridge::process_enable_motor_request(const ByteArray & request_packet)
{
  EnableRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->enable_motor(request.id(), request.control_mode());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_reset_motor_request(const ByteArray & request_packet)
{
  ResetRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->reset_motor(request.id());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_position_control_request(const ByteArray & request_packet)
{
  ControlPositionRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->send_position_command(request.id(), request.ref_position());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_speed_control_request(const ByteArray & request_packet)
{
  ControlSpeedRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->send_speed_command(request.id(), request.ref_speed());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_current_control_request(const ByteArray & request_packet)
{
  ControlCurrentRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->send_current_command(request.id(), request.ref_current());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_motion_control_request(const ByteArray & request_packet)
{
  ControlMotionRequestPacket request(request_packet);
  if (request.unpack()) {
    CybergearMotionCommand cmd;
    cmd.position = request.ref_position();
    cmd.velocity = request.ref_velocity();
    cmd.effort = request.ref_current();
    cmd.kp = request.kp();
    cmd.kd = request.kd();
    p_controller_->send_motion_command(request.id(), cmd);
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_set_mech_position_to_zero_request(const ByteArray & request_packet)
{
  SetMechPosToZeroRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->set_mech_position_to_zero(request.id());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_set_limit_speed(const ByteArray & request_packet)
{
  SetLimitSpeedRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->set_speed_limit(request.id(), request.limit_speed());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_set_limit_current(const ByteArray & request_packet)
{
  SetLimitCurrentRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->set_current_limit(request.id(), request.limit_current());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_set_limit_torque(const ByteArray & request_packet)
{
  SetLimitTorqueRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->set_torque_limit(request.id(), request.limit_torque());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_set_position_control_gain(const ByteArray & request_packet)
{
  SetPositionControlGainRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->set_position_control_gain(request.id(), request.kp());
    p_controller_->process_packet();
  }
}

void CybergearBridge::process_set_velocity_control_gain(const ByteArray & request_packet)
{
  SetVelocityControlGainRequestPacket request(request_packet);
  if (request.unpack()) {
    p_controller_->set_velocity_control_gain(request.id(), request.kp(), request.ki());
    p_controller_->process_packet();
  }
}

void CybergearBridge::send_motor_status_response(uint8_t id, uint8_t seq)
{
  MotorStatus mot;
  p_controller_->get_motor_status(id, mot);
  MotorStatusResponsePacket response(
    mot.motor_id, mot.position, mot.velocity, mot.effort, mot.temperature, seq);
  if (response.pack()) {
    ByteArray response_packet = response.packet();
    p_stream_->write(response_packet.data(), response_packet.size());
    p_stream_->flush();
  }
}

bool CybergearBridge::get_next_packet(ByteArray & request_packet)
{
  // stock serial data to receive buffer
  while (p_stream_->available() > 0) {
    receive_buffer_.push(p_stream_->read());
  }

  // erase ring buffer to next header byte
  while (receive_buffer_[static_cast<uint8_t>(Packet::CommandPacketIndex::FrameHeader)] !=
         Packet::Header) {
    uint8_t pop_data;
    receive_buffer_.pop(pop_data);
    if (receive_buffer_.isEmpty()) return false;
  }

  // if receive buffer enough, return
  if (receive_buffer_.size() < static_cast<uint8_t>(Packet::CommandPacketIndex::PacketSize)) {
    return false;
  }

  // check command + data frame size
  uint8_t cmd_packet_size = static_cast<uint8_t>(Packet::CommandPacketIndex::PacketSize);
  uint8_t data_packet_size =
    receive_buffer_[static_cast<uint8_t>(Packet::CommandPacketIndex::DataFrameSize)];
  uint8_t total_packet_size = cmd_packet_size + data_packet_size;
  if (receive_buffer_.size() < total_packet_size) {
    return false;
  }

  // cretate bytearray from receive buffer
  request_packet.clear();
  request_packet.resize(total_packet_size);
  for (uint8_t idx = 0; idx < total_packet_size; ++idx) {
    receive_buffer_.pop(request_packet[idx]);
  }

  return true;
}
