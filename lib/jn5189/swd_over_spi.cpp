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

void JN5189::SWD::__init_spi() {
  spi_bus_config_t bus = {
    .mosi_io_num     = swd_io_pin,
    .miso_io_num     = -1,
    .sclk_io_num     = swd_clk_pin,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = 0
  };

  spi_device_interface_config_t config = {
    .command_bits     = 0,
    .address_bits     = 0,
    .dummy_bits       = 0,
    .mode             = 0,
    .duty_cycle_pos   = 0,
    .cs_ena_pretrans  = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz   = 10 * 1000 * 1000, // 10 MHz
    .spics_io_num     = -1,
    .flags            = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST,
    .queue_size       = 24,
    .pre_cb           = nullptr,
    .post_cb          = nullptr
  };

  esp_err_t res;

  res = spi_bus_initialize(HSPI_HOST, &bus, SPI_DMA_DISABLED);
  assert(res == ESP_OK);

  res = spi_bus_add_device(HSPI_HOST, &config, &spi_device);
  assert(res == ESP_OK);
}

void JN5189::SWD::__deinit_spi() {
  esp_err_t res;

  res = spi_bus_remove_device(spi_device);
  assert(res == ESP_OK);

  res = spi_bus_free(HSPI_HOST);
  assert(res == ESP_OK);
}

/**
 * Receive bits from the SWD I/O line in LSB-first order
 */
void JN5189::SWD::__recv_bits_spi(uint32_t * const &value, size_t n_bits) {
  spi_transaction_t trans;

  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.flags    = SPI_TRANS_USE_RXDATA;
  trans.cmd      = 0;
  trans.addr     = 0;
  trans.length   = 0;
  trans.rxlength = n_bits;

  auto res = spi_device_transmit(spi_device, &trans);
  assert(res == ESP_OK);

  *value  = 0;
  *value |= trans.rx_data[0];
  *value |= trans.rx_data[1] << 8;
  *value |= trans.rx_data[2] << 16;
  *value |= trans.rx_data[3] << 24;

  if (log_level & LOG_LVL_DEBUG) {
    debug_serial.printf("<-- (SWD): %02X %02X %02X %02X | "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN "\n",
      trans.rx_data[3],
      trans.rx_data[2],
      trans.rx_data[1],
      trans.rx_data[0],
      BYTE_TO_BINARY(trans.rx_data[3]),
      BYTE_TO_BINARY(trans.rx_data[2]),
      BYTE_TO_BINARY(trans.rx_data[1]),
      BYTE_TO_BINARY(trans.rx_data[0])
    );
  }
}

/**
 * Send bits to the SWD I/O line in LSB-first order
 */
void JN5189::SWD::__send_bits_spi(uint32_t value, size_t n_bits) {
  spi_transaction_t trans;

  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.flags      = SPI_TRANS_USE_TXDATA;
  trans.cmd        = 0;
  trans.addr       = 0;
  trans.length     = n_bits;
  trans.rxlength   = 0;
  trans.tx_data[0] = value & 0xFF;
  trans.tx_data[1] = (value >> 8) & 0xFF;
  trans.tx_data[2] = (value >> 16) & 0xFF;
  trans.tx_data[3] = (value >> 24) & 0xFF;

  auto res = spi_device_transmit(spi_device, &trans);
  assert(res == ESP_OK);

  if (log_level & LOG_LVL_DEBUG) {
    debug_serial.printf("--> (SWD): %02X %02X %02X %02X | "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN " "
      BYTE_TO_BINARY_PATTERN "\n",
      trans.tx_data[3],
      trans.tx_data[2],
      trans.tx_data[1],
      trans.tx_data[0],
      BYTE_TO_BINARY(trans.tx_data[3]),
      BYTE_TO_BINARY(trans.tx_data[2]),
      BYTE_TO_BINARY(trans.tx_data[1]),
      BYTE_TO_BINARY(trans.tx_data[0])
    );
  }
}
