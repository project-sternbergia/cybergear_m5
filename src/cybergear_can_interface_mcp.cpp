#include "cybergear_can_interface_mcp.hh"

#ifdef USE_MCP_CAN

#include "cybergear_driver_utils.hh"
#include "mcp_can.h"

CybergearCanInterfaceMcp::CybergearCanInterfaceMcp() : pcan_(nullptr) {}

CybergearCanInterfaceMcp::~CybergearCanInterfaceMcp() {}

bool CybergearCanInterfaceMcp::init(uint8_t cs_pin, uint8_t int_pin)
{
  if (pcan_ != nullptr) {
    CG_DEBUG_PRINTLN("Failed to open can intetrface. CAN interface has already opened.");
    return false;
  }

  pcan_ = new MCP_CAN(cs_pin);
  if (!pcan_->begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    CG_DEBUG_PRINTLN("Failed to open can intetrface.");
    return false;
  }

  pcan_->setMode(MCP_NORMAL);
  pinMode(int_pin, INPUT);
  return true;
}

bool CybergearCanInterfaceMcp::send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
{
  uint32_t msg_id = id;
  uint8_t ret = pcan_->sendMsgBuf(msg_id, ext, len, const_cast<uint8_t *>(data));
  return (ret == CAN_OK);
}

bool CybergearCanInterfaceMcp::read_message(unsigned long & id, uint8_t * data, uint8_t & len)
{
  if (pcan_->checkReceive() != CAN_MSGAVAIL) {
    return false;
  }

  uint8_t ret = pcan_->readMsgBuf(&id, &len, data);
  return (ret == CAN_OK);
}

bool CybergearCanInterfaceMcp::available()
{
  if (pcan_->checkReceive() != CAN_MSGAVAIL) {
    return false;
  }
  return true;
}

#endif  // USE_MCP_CAN
