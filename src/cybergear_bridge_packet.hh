#ifndef CYBER_GEAR_BRIDGE_PAKCET_HH
#define CYBER_GEAR_BRIDGE_PAKCET_HH

#include <sys/types.h>
#include <vector>
#include <cstdint>

typedef std::vector<uint8_t> ByteArray;

/**
 * @brief Base packet class
 */
class Packet
{
public:
  static const uint8_t Header = 0x89;
  static const uint8_t CommandPacketSize = 0x08;

  /**
   * @brief CommandPacketIndex enum class
   */
  enum class CommandPacketIndex
  {
    FrameHeader = 0,    //!< frame header index
    PacketType,         //!< packet type index
    TargetMotorId,      //!< target motor id index
    DataFrameSize,      //!< data frame size index
    PacketSequence,     //!< packet sequence index
    RequsetType,        //!< request type index
    Optional,           //!< optional type index
    CheckSum,           //!< checksum index
    PacketSize          //!< packet size index
  };

  /**
   * @brief Construct a new Packet object
   *
   * @param type    packet type
   * @param id      motor id
   * @param size    packet size
   */
  explicit Packet(uint8_t type, uint8_t id, uint16_t size) : type_(type), id_(id), size_(size) {}

  /**
   * @brief Destroy the Packet object
   */
  virtual ~Packet() {}

  // accessors
  uint8_t type() const { return type_; }
  uint8_t id() const { return id_; }
  uint8_t size() const { return size_; }

private:
  uint8_t type_;
  uint8_t id_;
  uint8_t size_;
};

/**
 * @brief Request packet from PC
 */
class RequestPacket : public Packet
{
public:
  /**
   * @brief Request packet type
   */
  enum class Type
  {
    Enable = 0,           //!< enable request packet type
    Reset,                //!< reset request packet type (not implemented)
    GetMotorIdList,       //!< get motor id list packet type (not implemented)
    GetControlMode,       //!< get control mode packet type (not implemented)
    ControlMotion,        //!< control motion request packet type (not implemented)
    ControlSpeed,         //!< control speed request packet type (not implemented)
    ControlPosition,      //!< control position request packet type (not implemented)
    ControlCurrent,       //!< control current request packet type
    SetMechPosToZero,     //!< set current mechanical encoder position to zero
    _INVALID_TYPE_RANGE
  };

  /**
   * @brief Construct a new Request Packet object
   *
   * @param type    request packet type
   * @param size    packet size
   * @param packet  packet data
   */
  RequestPacket(uint8_t type, uint16_t size, const ByteArray &packet);

  /**
   * @brief Unpack packet
   *
   * @return true   success to unpack
   * @return false  failed to unpack
   */
  virtual bool unpack();

  /**
   * @brief Validate request packet
   *
   * @return true   ok
   * @return false  ng
   */
  bool validate() const;

  // accessors
  uint8_t command_packet_id() const {return packet_id_; };
  uint8_t command_packet_type() const {return packet_type_; };
  uint8_t command_packet_sequence() const {return packet_sequence_; };
  uint8_t command_packet_optional1() const {return packet_optional1_; };
  uint8_t command_packet_optional2() const {return packet_optional2_; };

protected:
  bool check_checksum() const;
  const ByteArray &command_packet() const;
  const ByteArray &data_packet() const;
  bool check_packet_type() const { return type() == command_packet_[static_cast<uint8_t>(Packet::CommandPacketIndex::PacketType)]; }

private:
  ByteArray command_packet_;
  ByteArray data_packet_;
  uint8_t packet_header_;
  uint8_t packet_id_;
  uint8_t packet_type_;
  uint8_t packet_data_size_;
  uint8_t packet_sequence_;
  uint8_t packet_optional1_;
  uint8_t packet_optional2_;
  uint8_t packet_checksum_;
};


/**
 * @brief EnableRequestPacket
 */
class EnableRequestPacket : public RequestPacket
{
public:
  /**
   * @brief Construct a new Enable Request Packet object
   *
   * @param packet data packet
   */
  explicit EnableRequestPacket(const ByteArray &packet);

  /**
   * @brief Destroy the Enable Request Packet object
   */
  virtual ~EnableRequestPacket();

  /**
   * @brief Unpack raw EnableRequestPacket
   *
   * @return true   OK
   * @return false  NG
   */
  virtual bool unpack();

  // accessor
  uint8_t control_mode() const;

private:
  uint8_t control_mode_;
};


/**
 * @brief ResetRequestPacket
 */
class ResetRequestPacket : public RequestPacket
{
public:
  /**
   * @brief Construct a new Reset Request Packet object
   *
   * @param packet data packet
   */
  explicit ResetRequestPacket(const ByteArray &packet);

  /**
   * @brief Destroy the Reset Request Packet object
   */
  virtual ~ResetRequestPacket();

  /**
   * @brief Unpack raw ResetRequestPacket
   *
   * @return true   OK
   * @return false  NG
   */
  virtual bool unpack();
};

/**
 * @brief ResetRequestPacket
 */
class SetMechPosToZeroRequestPacket : public RequestPacket
{
public:
  /**
   * @brief Construct a new Reset Request Packet object
   *
   * @param packet data packet
   */
  explicit SetMechPosToZeroRequestPacket(const ByteArray &packet);

  /**
   * @brief Destroy the Reset Request Packet object
   */
  virtual ~SetMechPosToZeroRequestPacket();

  /**
   * @brief Unpack raw ResetRequestPacket
   *
   * @return true   OK
   * @return false  NG
   */
  virtual bool unpack();
};

/**
 * @brief ControlPositionRequestPacket
 */
class ControlPositionRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 5;

  /**
   * @brief Construct a new Control Position Request Packet object
   *
   * @param packet data packet
   */
  explicit ControlPositionRequestPacket(const ByteArray &packet);

  /**
   * @brief Destroy the Control Position Request Packet object
   */
  virtual ~ControlPositionRequestPacket();

  /**
   * @brief Unpack raw ControlPositionRequestPacket
   *
   * @return true   OK
   * @return false  NG
   */
  virtual bool unpack();

  // accessor
  float ref_position() const { return ref_position_; }

private:
  float ref_position_;
};

class ControlSpeedRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 5;
  explicit ControlSpeedRequestPacket(const ByteArray &packet);
  virtual ~ControlSpeedRequestPacket();
  virtual bool unpack();
  float ref_speed() const { return ref_speed_; }

private:
  float ref_speed_;
};

class ControlCurrentRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 5;
  explicit ControlCurrentRequestPacket(const ByteArray &packet);
  virtual ~ControlCurrentRequestPacket();
  virtual bool unpack();
  float ref_current() const { return ref_current_; }

private:
  float ref_current_;
};

class ControlMotionRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 4 * 3 + 1;
  explicit ControlMotionRequestPacket(const ByteArray &packet);
  virtual ~ControlMotionRequestPacket();
  virtual bool unpack();
  float ref_position() const { return ref_position_; }
  float ref_velocity() const { return ref_velocity_; }
  float ref_current() const { return ref_current_; }

private:
  float ref_position_;
  float ref_velocity_;
  float ref_current_;
};

/**
 * @brief Response packet to PC
 */
class ResponsePacket : public Packet
{
public:
  enum class Type
  {
    MotorStatus
  };

  ResponsePacket(uint8_t type, uint8_t id, uint16_t size, uint8_t seq);
  virtual ~ResponsePacket();
  virtual bool pack();
  const ByteArray & packet();

protected:
  uint8_t calc_checksum(const ByteArray& packet) const;
  uint8_t packet_sequence_;
  ByteArray packet_;
  ByteArray command_packet_;
  ByteArray data_packet_;
};

class MotorStatusResponsePacket : public ResponsePacket
{
public:
  static const int PacketSize = CommandPacketSize + 16 + 1;
  MotorStatusResponsePacket(uint8_t id, float pos, float vel, float eff, float temp, uint8_t seq);
  virtual ~MotorStatusResponsePacket();
  virtual bool pack();

private:
  float position_;
  float velocity_;
  float effort_;
  float tempareture_;
  uint8_t sequence_;
};

#endif // CYBER_GEAR_BRIDGE_PACKET_HH