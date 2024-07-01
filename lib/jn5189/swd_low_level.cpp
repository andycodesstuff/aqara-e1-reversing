#include "jn5189.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(value)   \
  ((value) & 0x80 ? '1' : '0'), \
  ((value) & 0x40 ? '1' : '0'), \
  ((value) & 0x20 ? '1' : '0'), \
  ((value) & 0x10 ? '1' : '0'), \
  ((value) & 0x08 ? '1' : '0'), \
  ((value) & 0x04 ? '1' : '0'), \
  ((value) & 0x02 ? '1' : '0'), \
  ((value) & 0x01 ? '1' : '0')

/**
 * Switch from JTAG to SWD operation
 * 
 * Reference: https://developer.arm.com/documentation/ihi0031/g
 *            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
 *            Chapter B5: The Serial Wire/JTAG Debug Port
 *            B5.2.2 Switching from JTAG to SWD operation
 */
void JN5189::SWD::__switch_mode_jtag_to_swd() {
  __line_reset();
  __write_bits(0xE79E, 16);
  __line_reset();
  __sync();
}

/**
 * Read bits from the SWD I/O line in LSB-first order
 */
uint32_t JN5189::SWD::__read_bits(size_t n_bits) {
  uint32_t buffer = 0;

  __setup_master_read();

  for (auto i = 0; i < n_bits; i++) {
    auto bit_value = digitalRead(swd_io_pin);

    bitWrite(buffer, i, bit_value);
    __pulse_clock();
  }

  if (log_level & LOG_LVL_DEBUG) {
    debug_serial.printf("<-- (SWD): %02X %02X %02X %02X | "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN "\n",
      (buffer >> 24) & 0xFF,
      (buffer >> 16) & 0xFF,
      (buffer >> 8) & 0xFF,
      (buffer) & 0xFF,
      BYTE_TO_BINARY(buffer >> 24),
      BYTE_TO_BINARY(buffer >> 16),
      BYTE_TO_BINARY(buffer >> 8),
      BYTE_TO_BINARY(buffer)
    );
  }

  return buffer;
}

/**
 * Write bits to the SWD I/O line in LSB-first order
 */
void JN5189::SWD::__write_bits(uint32_t value, size_t n_bits) {
  __setup_master_write();

  for (auto i = 0; i < n_bits; i++) {
    auto bit_value = bitRead(value, i);

    digitalWrite(swd_io_pin, bit_value);
    __pulse_clock();
  }

  if (log_level & LOG_LVL_DEBUG) {
    debug_serial.printf("--> (SWD): %02X %02X %02X %02X | "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN "\n",
      (value >> 24) & 0xFF,
      (value >> 16) & 0xFF,
      (value >> 8) & 0xFF,
      (value) & 0xFF,
      BYTE_TO_BINARY(value >> 24),
      BYTE_TO_BINARY(value >> 16),
      BYTE_TO_BINARY(value >> 8),
      BYTE_TO_BINARY(value)
    );
  }
}

/**
 * The line reset (or connection) sequence ensures that the SW-DP is synchronized correctly to
 * the header that is used to signal a connection. A line reset is achieved by holding the data
 * signal HIGH for at least 50 clock cycles, followed by at least two idle cycles
 * 
 * Reference: https://developer.arm.com/documentation/ihi0031/g
 *            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
 *            Chapter B4: The Serial Wire Debug Port
 *            B4.3.3 Connection and line reset sequence
 */
void JN5189::SWD::__line_reset() {
  digitalWrite(swd_io_pin, HIGH);

  for (auto i = 0; i < RESET_SEQUENCE_LENGTH + 2; i++) {
    __pulse_clock();
  }

  digitalWrite(swd_io_pin, LOW);
}

void JN5189::SWD::__sync() {
  __write_bits(0, SYNC_SEQUENCE_LENGTH);
}

/**
 * Every time the I/O line changes data direction, a turnaround period is inserted which both
 * sides should ignore. Defaults to one clock cycle
 */
void JN5189::SWD::__turnaround_period() {
  __pulse_clock();
}

void JN5189::SWD::__pulse_clock() {
  digitalWrite(swd_clk_pin, LOW);
  delayMicroseconds(CLOCK_HALF_CYCLE_MICROSEC);
  digitalWrite(swd_clk_pin, HIGH);
  delayMicroseconds(CLOCK_HALF_CYCLE_MICROSEC);
}

void JN5189::SWD::__setup_master_read() {
  pinMode(swd_io_pin, INPUT_PULLUP);
}

void JN5189::SWD::__setup_master_write() {
  pinMode(swd_io_pin, OUTPUT);
}
