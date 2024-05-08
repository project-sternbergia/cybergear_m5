#include "cybergear_bridge_packet.hh"

#include <memory.h>

#include "cybergear_driver_utils.hh"

RequestPacket::RequestPacket(uint8_t type, uint16_t size, const ByteArray & packet)
: Packet(type, packet[2], size)
{
  command_packet_ = ByteArray(packet.begin(), packet.begin() + CommandPacketSize);
  data_packet_ = ByteArray(packet.begin() + command_packet_.size(), packet.end());
}

bool RequestPacket::unpack()
{
  // check packet size
  if (command_packet_.size() != CommandPacketSize) {
    CG_DEBUG_PRINTLN("invalid command packet size");
    return false;
  }

  packet_header_ = command_packet_[0];
  packet_type_ = command_packet_[1];
  packet_id_ = command_packet_[2];
  packet_data_size_ = command_packet_[3];
  packet_sequence_ = command_packet_[4];
  packet_optional1_ = command_packet_[5];
  packet_optional2_ = command_packet_[6];
  packet_checksum_ = command_packet_[7];
  return validate();
}

bool RequestPacket::check_checksum() const
{
  // check command packet
  {
    uint8_t check_sum = 0x00;
    for (uint8_t idx = 0; idx < command_packet_.size() - 1; ++idx) {
      check_sum = (check_sum + command_packet_[idx]) & 0xFF;
    }
    if (check_sum != command_packet_.back()) {
      CG_DEBUG_PRINTLN("invalid command packet checksum");
      return false;
    }
  }

  // check data packet
  if (data_packet_.size() > 0) {
    uint8_t check_sum = 0x00;
    for (uint8_t idx = 0; idx < data_packet_.size() - 1; ++idx) {
      check_sum = (check_sum + data_packet_[idx]) & 0xFF;
    }
    if (check_sum != data_packet_.back()) {
      CG_DEBUG_PRINTLN("invalid data packet checksum");
      return false;
    }
  }
  return true;
}

bool RequestPacket::validate() const
{
  // check header
  if (packet_header_ != Packet::Header) {
    CG_DEBUG_PRINTLN("invalid header");
    return false;
  }

  if (packet_type_ >= static_cast<uint8_t>(Type::_INVALID_TYPE_RANGE)) {
    CG_DEBUG_PRINTLN("invalid packet type");
    return false;
  }

  if (packet_data_size_ != data_packet_.size()) {
    CG_DEBUG_PRINTLN("invalid data size");
    return false;
  }
  return check_checksum();
}

const ByteArray & RequestPacket::command_packet() const { return command_packet_; }

const ByteArray & RequestPacket::data_packet() const { return data_packet_; }

// EnableRequestPaket
EnableRequestPacket::EnableRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::Enable), Packet::CommandPacketSize, packet)
{
}

EnableRequestPacket::~EnableRequestPacket() {}

bool EnableRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  control_mode_ = command_packet_optional1();
  return true;
}

uint8_t EnableRequestPacket::control_mode() const { return control_mode_; }

// EnableRequestPaket
ResetRequestPacket::ResetRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::Reset), Packet::CommandPacketSize, packet)
{
}

ResetRequestPacket::~ResetRequestPacket() {}

bool ResetRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  return true;
}

// SetMechPosToZeroRequestPacket
SetMechPosToZeroRequestPacket::SetMechPosToZeroRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::SetMechPosToZero), Packet::CommandPacketSize, packet)
{
}

bool SetMechPosToZeroRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  return true;
}

// SetLimitSpeedRequestPacket
SetLimitSpeedRequestPacket::SetLimitSpeedRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::SetLimitSpeed), PacketSize, packet)
{
}

bool SetLimitSpeedRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&limit_speed_, data_packet().data(), sizeof(float));
  return true;
}

// SetLimitCurrentRequestPacket
SetLimitCurrentRequestPacket::SetLimitCurrentRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::SetLimitCurrent), PacketSize, packet)
{
}

bool SetLimitCurrentRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&limit_current_, data_packet().data(), sizeof(float));
  return true;
}

// SetLimitTorqueRequestPacket
SetLimitTorqueRequestPacket::SetLimitTorqueRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::SetLimitTorque), PacketSize, packet)
{
}

bool SetLimitTorqueRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&limit_torque_, data_packet().data(), sizeof(float));
  return true;
}

// SetPositionControlGainRequestPacket
SetPositionControlGainRequestPacket::SetPositionControlGainRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::SetPositionControlGain), PacketSize, packet)
{
}

bool SetPositionControlGainRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&kp_, data_packet().data(), sizeof(float));
  return true;
}

// SetVelocityControlGainRequestPacket
SetVelocityControlGainRequestPacket::SetVelocityControlGainRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::SetVelocityControlGain), PacketSize, packet)
{
}

bool SetVelocityControlGainRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&kp_, data_packet().data() + 0, sizeof(float));
  memcpy(&ki_, data_packet().data() + 4, sizeof(float));
  return true;
}

// ControlPositionRequestPacket
ControlPositionRequestPacket::ControlPositionRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::ControlPosition), PacketSize, packet)
{
}

ControlPositionRequestPacket::~ControlPositionRequestPacket() {}

bool ControlPositionRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&ref_position_, data_packet().data(), sizeof(float));
  return true;
}

// ControlSpeedRequestPacket
ControlSpeedRequestPacket::ControlSpeedRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::ControlSpeed), PacketSize, packet)
{
}

ControlSpeedRequestPacket::~ControlSpeedRequestPacket() {}

bool ControlSpeedRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&ref_speed_, data_packet().data(), sizeof(float));
  return true;
}

// ControlCurrentRequestPacket
ControlCurrentRequestPacket::ControlCurrentRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::ControlCurrent), PacketSize, packet)
{
}

ControlCurrentRequestPacket::~ControlCurrentRequestPacket() {}

bool ControlCurrentRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&ref_current_, data_packet().data(), sizeof(float));
  return true;
}

// ControlMotionRequestPacket
ControlMotionRequestPacket::ControlMotionRequestPacket(const ByteArray & packet)
: RequestPacket(static_cast<uint8_t>(Type::ControlMotion), PacketSize, packet)
{
}

ControlMotionRequestPacket::~ControlMotionRequestPacket() {}

bool ControlMotionRequestPacket::unpack()
{
  if (!RequestPacket::unpack()) return false;
  memcpy(&ref_position_, data_packet().data() + 0, sizeof(float));
  memcpy(&ref_velocity_, data_packet().data() + 4, sizeof(float));
  memcpy(&ref_current_, data_packet().data() + 8, sizeof(float));
  memcpy(&kp_, data_packet().data() + 12, sizeof(float));
  memcpy(&kd_, data_packet().data() + 16, sizeof(float));
  return true;
}

// Response Packet
ResponsePacket::ResponsePacket(uint8_t type, uint8_t id, uint16_t size, uint8_t seq)
: Packet(type, id, size),
  packet_(size),
  packet_sequence_(seq),
  command_packet_(CommandPacketSize),
  data_packet_(size - CommandPacketSize)
{
}

ResponsePacket::~ResponsePacket() {}

bool ResponsePacket::pack()
{
  if (packet_.size() < CommandPacketSize) {
    return false;
  }

  // pack command packet
  command_packet_[0] = Packet::Header;
  command_packet_[1] = static_cast<uint8_t>(type());
  command_packet_[2] = id();
  command_packet_[3] = packet_.size() - CommandPacketSize;
  command_packet_[4] = packet_sequence_;
  command_packet_[5] = 0x00;
  command_packet_[6] = 0x00;
  command_packet_[7] = calc_checksum(command_packet_);
  return true;
}

const ByteArray & ResponsePacket::packet()
{
  // copy command and data to packet
  std::copy(command_packet_.begin(), command_packet_.end(), packet_.begin());
  std::copy(data_packet_.begin(), data_packet_.end(), packet_.begin() + command_packet_.size());
  return packet_;
}

uint8_t ResponsePacket::calc_checksum(const ByteArray & packet) const
{
  uint8_t checksum = 0x00;
  for (uint8_t idx = 0; idx < packet.size() - 1; ++idx) {
    checksum = (checksum + packet[idx]) & 0xFF;
  }
  return checksum;
}

MotorStatusResponsePacket::MotorStatusResponsePacket(
  uint8_t id, float pos, float vel, float eff, float temp, uint8_t seq)
: ResponsePacket(static_cast<uint8_t>(ResponsePacket::Type::MotorStatus), id, PacketSize, seq),
  position_(pos),
  velocity_(vel),
  effort_(eff),
  tempareture_(temp)
{
}

MotorStatusResponsePacket::~MotorStatusResponsePacket() {}

bool MotorStatusResponsePacket::pack()
{
  if (!ResponsePacket::pack()) return false;
  memcpy(data_packet_.data() + 0, &position_, sizeof(position_));
  memcpy(data_packet_.data() + 4, &velocity_, sizeof(velocity_));
  memcpy(data_packet_.data() + 8, &effort_, sizeof(effort_));
  memcpy(data_packet_.data() + 12, &tempareture_, sizeof(tempareture_));
  data_packet_[16] = calc_checksum(data_packet_);
  return true;
}
