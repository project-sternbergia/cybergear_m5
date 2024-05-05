#ifndef CYBERGEAR_CAN_INTERFACE_HH
#define CYBERGEAR_CAN_INTERFACE_HH

#include <inttypes.h>

class CybergearCanInterface
{
public:
  CybergearCanInterface() {}
  virtual ~CybergearCanInterface() {}
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext) = 0;
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len) = 0;
  virtual bool available() = 0;
};

#endif  // CYBERGEAR_CAN_INTERFACE_HH
