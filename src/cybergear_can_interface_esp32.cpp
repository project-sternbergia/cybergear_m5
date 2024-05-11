#include "cybergear_can_interface_esp32.hh"

#ifdef CONFIG_IDF_TARGET_ESP32

// #include <M5Stack.h>
#include <RingBuf.h>

#include <cstdint>
#include <cstring>

#include "cybergear_can_interface.hh"
#include "cybergear_driver_utils.hh"

// use arduino-CAN
#include <CAN.h>

struct CanMessage
{
  uint64_t tx_id;
  uint64_t rx_id;
  bool is_extended;
  bool is_rtr;
  uint8_t dlc;
  uint8_t data[8];
};
static RingBuf<CanMessage, 100> buffer;

static void on_receive(int size)
{
  CanMessage msg;
  msg.is_extended = CAN.packetExtended();
  msg.is_rtr = CAN.packetRtr();
  msg.tx_id = CAN.packetTxId();
  msg.rx_id = CAN.packetRxId();
  msg.dlc = CAN.packetDlc();
  for (uint8_t idx = 0; idx < msg.dlc; ++idx) {
    if (CAN.available()) {
      msg.data[idx] = CAN.read();
    }
  }
  buffer.lockedPushOverwrite(msg);
}

CybergearCanInterfaceEsp32::CybergearCanInterfaceEsp32() : CybergearCanInterface() {}

CybergearCanInterfaceEsp32::~CybergearCanInterfaceEsp32() {}

bool CybergearCanInterfaceEsp32::init(uint8_t rx_pin, uint8_t tx_pin)
{
  CAN.setPins(rx_pin, tx_pin);
  CAN.onReceive(on_receive);
  if (!CAN.begin(1000E3)) {
    return false;
  }

  return true;
}

bool CybergearCanInterfaceEsp32::send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
{
  CG_DEBUG_FUNC
  // change packet type
  if (ext) {
    CAN.beginExtendedPacket(id);
  } else {
    CAN.beginPacket(id);
  }

  // send packet
  CAN.write(data, len);
  CAN.endPacket();
  return true;
}

bool CybergearCanInterfaceEsp32::read_message(unsigned long & id, uint8_t * data, uint8_t & len)
{
  CG_DEBUG_FUNC
  // check empty
  if (buffer.isEmpty()) return false;

  // get mseesage from buffer
  CanMessage msg;
  if (!buffer.lockedPop(msg)) return false;

  id = msg.rx_id;
  len = msg.dlc;
  memcpy(data, msg.data, len);
  return true;
}

bool CybergearCanInterfaceEsp32::available()
{
  CG_DEBUG_FUNC
  return (buffer.isEmpty() == false);
}

#endif  //  CONFIG_IDF_TARGET_ESP32
