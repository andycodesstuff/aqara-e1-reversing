#include "jn5189.h"

#define MEMORY_READ_FAULT_VALUE 0xEFBE3713

std::vector<uint8_t> JN5189::SWD::memory_read(uint32_t address, size_t n_bytes) {
  std::vector<uint8_t> bytes;
  uint32_t data;
  uint8_t ack;

  // Read memory starting from the given address. The address will be automatically incremented
  // by 4 (word size in bytes) at each successful memory access
  ack = write_AP(REG_AP_TAR, address);
  if (ack != ACK_OK) return bytes;

  for (size_t bytes_read = 0; bytes_read < n_bytes && address < 0xFFFFFFFF; bytes_read += 4) {
    ack = read_AP(REG_AP_DRW, &data);

    // Use an easily recognizable sentinel value for failed read operations
    if (ack != ACK_OK) data = MEMORY_READ_FAULT_VALUE;

    bytes.push_back(data & 0xFF);
    bytes.push_back((data >> 8) & 0xFF);
    bytes.push_back((data >> 16) & 0xFF);
    bytes.push_back((data >> 24) & 0xFF);

    address += bytes_read;
  }

  return bytes;
}

bool JN5189::SWD::connect() {
  uint32_t idcode;

  // Connect to the target and ensure the device is operating in SWD mode
  __switch_mode_jtag_to_swd();

  // Parse values
  auto ack      = read_DP(REG_DP_IDCODE, &idcode);
  auto version  = (idcode >> 28) & 0b1111;
  auto partno   = (idcode >> 12) & 0b1111111111111111;
  auto designer = (idcode >> 1)  & 0b11111111111;

  auto success = ack == ACK_OK && designer == DEFAULT_DESIGNER;
  if (!success) return false;

  if (success && log_level & LOG_LVL_INFO) {
    debug_serial.printf(" (parsed): ACK: %d\n", ack);
    debug_serial.printf(" (parsed): Version: 0x%01X\n", version);
    debug_serial.printf(" (parsed): Part number: 0x%04X\n", partno);
    debug_serial.printf(" (parsed): Designer: 0x%03X\n", designer);
  }

  // Prepare target for debugging
  write_DP(REG_DP_ABORT,     0x00000016); // Clear any sticky errors
  write_DP(REG_DP_SELECT,    0x00000000); // Get the SELECT register to a known state
  write_DP(REG_DP_CTRL_STAT, 0x50000F00); // Power-up the system and debug domain
  write_AP(REG_AP_CSW,       0x23000052); // Enable address auto-increment on memory access

  return success;
}

void JN5189::SWD::cpu_halt() {
  write_register(REG_DBG_CTRL_DHCSR, 0xA05F0003); // Enable debugging
  write_register(REG_DBG_CTRL_DEMCR, 0x00000001); // Enable CPU halt on reset
  write_register(REG_SYS_CTRL_AIRCR, 0x05FA0004); // Request a system reset
}

void JN5189::SWD::cpu_resume() {
  write_register(REG_DBG_CTRL_DHCSR, 0xA05F0000); // Disable debugging
  write_register(REG_DBG_CTRL_DEMCR, 0x00000000); // Disable CPU halt on reset
  write_register(REG_SYS_CTRL_AIRCR, 0x05FA0004); // Request a system reset
}
