#include "cybergear_can_interface_twai.hh"

#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "ESP32-TWAI-CAN.hpp"
#include "cybergear_driver_utils.hh"

CybergearCanInterfaceTwai::CybergearCanInterfaceTwai() : CybergearCanInterface() {}

CybergearCanInterfaceTwai::~CybergearCanInterfaceTwai() {}

bool CybergearCanInterfaceTwai::init(uint8_t rx_pin, uint8_t tx_pin)
{
  return ESP32Can.begin(ESP32Can.convertSpeed(1000), tx_pin, rx_pin, 10, 10);
}

bool CybergearCanInterfaceTwai::send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
{
  CG_DEBUG_FUNC
  CanFrame frame = {0};
  frame.identifier = id;
  frame.extd = (ext) ? 1 : 0;
  frame.data_length_code = len;
  memcpy(frame.data, data, len);
  return ESP32Can.writeFrame(frame);
}

bool CybergearCanInterfaceTwai::read_message(unsigned long & id, uint8_t * data, uint8_t & len)
{
  CG_DEBUG_FUNC
  CanFrame frame = {0};
  if (ESP32Can.readFrame(frame, 1) == false) {
    return false;
  }

  if (frame.rtr) return false;

  id = frame.identifier;
  len = frame.data_length_code;
  memcpy(data, frame.data, frame.data_length_code);
  return true;
}

bool CybergearCanInterfaceTwai::available()
{
  CG_DEBUG_FUNC
  return (ESP32Can.inRxQueue() > 0);
}

#endif  // CONFIG_IDF_TARGET_ESP32S3
