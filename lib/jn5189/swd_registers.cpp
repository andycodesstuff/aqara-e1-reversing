#include "jn5189.h"

#include <cmath>

uint8_t JN5189::SWD::read_DP(uint8_t address, uint32_t * const &value) {
  return __access_register(APnDP_DP, RnW_READ, address, value);
}

uint8_t JN5189::SWD::write_DP(uint8_t address, uint32_t value) {
  return __access_register(APnDP_DP, RnW_WRITE, address, &value);
}

uint8_t JN5189::SWD::read_AP(uint8_t address, uint32_t * const &value) {
  uint32_t select = 0;
  uint8_t ack = 0;

  // Build AP select register (SELECT) value
  // 
  // Reference: https://developer.arm.com/documentation/ihi0031/g
  //            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
  //            Chapter B2: DP Reference Information
  //            B2.2.9: SELECT, AP Select register
  select |= address & 0b11110000; // APBANKSEL

  // Request access to the AP register
  ack = __access_register(APnDP_DP, RnW_WRITE, REG_DP_SELECT, &select);

  // Read selected register value
  ack = __access_register(APnDP_AP, RnW_READ, address & 0b1100, value);
  ack = __access_register(APnDP_DP, RnW_READ, REG_DP_RDBUFF, value);

  return ack;
}

uint8_t JN5189::SWD::write_AP(uint8_t address, uint32_t value) {
  uint32_t select = 0;
  uint8_t ack = 0;

  // Build AP select register (SELECT) value
  // 
  // Reference: https://developer.arm.com/documentation/ihi0031/g
  //            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
  //            Chapter B2: DP Reference Information
  //            B2.2.9: SELECT, AP Select register
  select |= address & 0b11110000; // APBANKSEL

  // Request access to the AP register
  ack = __access_register(APnDP_DP, RnW_WRITE, REG_DP_SELECT, &select);

  // Write to the selected register
  ack = __access_register(APnDP_AP, RnW_WRITE, address & 0b1100, &value);

  return ack;
}

uint8_t JN5189::SWD::read_register(uint32_t address, uint32_t * const &value) {
  uint8_t ack = 0;

  ack = write_AP(REG_AP_TAR, address);
  ack = read_AP(REG_AP_DRW, value);

  return ack;
}

uint8_t JN5189::SWD::write_register(uint32_t address, uint32_t value) {
  uint8_t ack = 0;

  ack = write_AP(REG_AP_TAR, address);
  ack = write_AP(REG_AP_DRW, value);

  return ack;
}

uint8_t JN5189::SWD::__access_register(RequestAPnDP APnDP,
                                       RequestRnW RnW,
                                       uint8_t address,
                                       uint32_t * const &value,
                                       int max_retries) {
  // Early exit on exceeding number of maximum register accesses
  if (max_retries == 0) {
    if (log_level & LOG_LVL_DEBUG) {
      debug_serial.printf("--- (SWD): request<APnDP=%d, RnW=%d, address=0x%02X> exceeded max retries\n", APnDP, RnW, address);
    }

    return ACK_FAULT;
  }

  // Access the requested register
  auto ack = __access_register_backend(APnDP, RnW, address, value);

  // Clear sticky error flags so that future requests can be (hopefully) fulfilled.
  // Note that reading/writing to CTRL/STAT and writing to ABORT can never result in a FAULT
  // and thus checking acks for the register accesses below is superfluous
  // 
  // Reference: https://developer.arm.com/documentation/ihi0031/g
  //            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
  //            Chapter B4: The Serial Wire Debug Port
  //            B4.2.4: FAULT response to read or write operation request
  if (ack & ACK_FAULT) {
    if (log_level & LOG_LVL_DEBUG) {
      uint32_t ctrl_stat = 0;
      __access_register_backend(APnDP_DP, RnW_READ, REG_DP_CTRL_STAT, &ctrl_stat);

      debug_serial.printf("--- (SWD): request<APnDP=%d, RnW=%d, address=0x%02X> failed\n", APnDP, RnW, address);
      debug_serial.printf("--- (SWD): CTRL/STAT: 0x%08X\n", ctrl_stat);
    }

    uint32_t abort = 0x00000016;
    __access_register_backend(APnDP_DP, RnW_WRITE, REG_DP_ABORT, &abort);

    return ACK_FAULT;
  }

  // Retry the same access on WAIT
  // 
  // Reference: https://developer.arm.com/documentation/ihi0031/g
  //            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
  //            Chapter B4: The Serial Wire Debug Port
  //            B4.2.3: WAIT response to read or write operation request
  if (ack & ACK_WAIT) {
    // Exponential backoff delay so that, hopefully, the target can fulfill the next request
    auto retry = (REG_ACCESS_MAX_RETRIES - max_retries) + 1;
    auto delay = pow(2, retry);
    delayMicroseconds(delay);

    if (log_level & LOG_LVL_DEBUG) {
      debug_serial.printf("--- (SWD): retry #%d, delay %dus\n", retry, delay);
    }

    return __access_register(APnDP, RnW, address, value, max_retries - 1);
  }

  return ACK_OK;
}

uint8_t JN5189::SWD::__access_register_backend(RequestAPnDP APnDP,
                                               RequestRnW RnW,
                                               uint8_t address,
                                               uint32_t * const &value) {
  uint8_t req = 0, par = 0, ack = 0;

  // Compute parity
  par |= APnDP << 3;
  par |= RnW << 2;
  par |= (address & 0b1100) >> 2;
  par  = __parity_bit(par);

  // Build and send request
  bitWrite(req, 0, 1);                    // Start
  bitWrite(req, 1, APnDP);                // APnDP
  bitWrite(req, 2, RnW);                  // RnW
  bitWrite(req, 3, (address >> 2) & 0b1); // Register address. This field provides bits [3:2]
  bitWrite(req, 4, (address >> 3) & 0b1); // of the address, as bits [1:0] are always 0b00
  bitWrite(req, 5, par);                  // Parity
  bitWrite(req, 6, 0);                    // Stop
  bitWrite(req, 7, 1);                    // Park
  __write_bits(req, 8);

  __turnaround_period();

  // Read ACK
  ack = __read_bits(3);

  if (ack == ACK_OK) {
    if (RnW & RnW_READ) {
      // Read RDATA and parity bit
      *value = __read_bits(32);
      __read_bits(1);

      __turnaround_period();
    } else {
      __turnaround_period();

      // Write WDATA and parity bit
      __write_bits(*value, 32);
      __write_bits(__parity_bit(*value), 1);
    }
  }

  __sync();

  return ack;
}

/**
 * Count the number of set bits. Return 0 if it's even or 1 if it's odd
 */
bool JN5189::SWD::__parity_bit(uint32_t value) {
  size_t set_bits = 0;

  while (value) {
    set_bits += value & 1;
    value >>= 1;
  }

  return set_bits % 2;
}
