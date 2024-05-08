#ifndef CYBERGEAR_CAN_INTERFACE_MCP_HH
#define CYBERGEAR_CAN_INTERFACE_MCP_HH
#ifdef USE_MCP_CAN

#include "cybergear_can_interface.hh"
#define M5_COMMU_BORAD_CS_PIN 12
#define M5_COMMU_BORAD_INT_PIN 15

// foward definition
class MCP_CAN;

class CybergearCanInterfaceMcp : public CybergearCanInterface
{
public:
  CybergearCanInterfaceMcp();
  virtual ~CybergearCanInterfaceMcp();
  bool init(uint8_t cs_pin = M5_COMMU_BORAD_CS_PIN, uint8_t int_pin = M5_COMMU_BORAD_INT_PIN);
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext);
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len);
  virtual bool available();

private:
  MCP_CAN * pcan_ = nullptr;
  uint8_t receive_buffer_[64];  //!< receive buffer
};

#endif
#endif  // CYBERGEAR_CAN_INTERFACE_MCP_HH
