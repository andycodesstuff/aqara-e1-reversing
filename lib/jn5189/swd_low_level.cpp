#include "jn5189.h"

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
  __send_bits_spi(0xE79E, 16);
  __line_reset();
  __sync();
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
  __send_bits_spi(0xFFFFFFFF, RESET_SEQUENCE_LENGTH / 2);
  __send_bits_spi(0xFFFFFFFF, RESET_SEQUENCE_LENGTH / 2);
}

void JN5189::SWD::__sync() {
  __send_bits_spi(0, SYNC_SEQUENCE_LENGTH);
}

/**
 * Every time the I/O line changes data direction, a turnaround period is inserted which both
 * sides should ignore. Defaults to one clock cycle
 */
void JN5189::SWD::__turnaround_period() {
  __send_bits_spi(0, TURNAROUND_PERIOD_LENGTH);
}
