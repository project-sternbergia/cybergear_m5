#ifndef CYBERGEAR_CAN_INTERFACE_TWAI_HH
#define CYBERGEAR_CAN_INTERFACE_TWAI_HH
#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "cybergear_can_interface.hh"

#define M5_ESP32_DEFAULT_RX_PIN 1
#define M5_ESP32_DEFAULT_TX_PIN 2

class CybergearCanInterfaceTwai : public CybergearCanInterface
{
public:
  CybergearCanInterfaceTwai();
  virtual ~CybergearCanInterfaceTwai();
  bool init(uint8_t rx_pin = M5_ESP32_DEFAULT_RX_PIN, uint8_t tx_pin = M5_ESP32_DEFAULT_TX_PIN);
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext);
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len);
  virtual bool available();
};

#endif  // CONFIG_IDF_TARGET_ESP32S3
#endif  // CYBERGEAR_CAN_INTERFACE_TWAI_HH
