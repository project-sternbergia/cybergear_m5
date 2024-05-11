#ifndef CYBERGEAR_CAN_INTERFACE_ESP32_HH
#define CYBERGEAR_CAN_INTERFACE_ESP32_HH
#ifdef CONFIG_IDF_TARGET_ESP32

#include "cybergear_can_interface.hh"

#define M5_CANBUS_TRANSIVER_UNIT
#ifndef M5_CANBUS_TRANSIVER_UNIT
#define M5_ESP32_DEFAULT_RX_PIN 22
#define M5_ESP32_DEFAULT_TX_PIN 21
#else
#define M5_ESP32_DEFAULT_RX_PIN 5
#define M5_ESP32_DEFAULT_TX_PIN 15
#endif

class CybergearCanInterfaceEsp32 : public CybergearCanInterface
{
public:
  CybergearCanInterfaceEsp32();
  virtual ~CybergearCanInterfaceEsp32();
  bool init(uint8_t rx_pin = M5_ESP32_DEFAULT_RX_PIN, uint8_t tx_pin = M5_ESP32_DEFAULT_TX_PIN);
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext);
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len);
  virtual bool available();

private:
  uint8_t receive_buffer_[64];  //!< receive buffer
};

#endif  //CONFIG_IDF_TARGET_ESP32
#endif  // ESP_CAN_INTERFACE_HH
