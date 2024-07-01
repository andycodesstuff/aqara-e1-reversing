#include "jn5189.h"

std::vector<uint8_t> JN5189::ISP::unlock(ISPUnlockMode unlock_mode) {
  switch (unlock_mode) {
    case ISPUnlockMode::MODE_DEFAULT_STATE: {
      uint8_t payload[] = {0x00};
      size_t payload_length = 1;

      auto bytes_sent = __send_req(0x4E, payload, payload_length);
      break;
    }
    case ISPUnlockMode::MODE_START_ISP: {
      uint8_t payload[] = {0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
                                 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
      size_t payload_length = 17;

      auto bytes_sent = __send_req(0x4E, payload, payload_length);
      break;
    }
  }

  auto res = __recv_res();

  if (res[4] != 0x00) {
    debug_serial.printf("<-- (ISP): Error 0x%02X\n", res[4]);
    return std::vector<uint8_t>();
  }

  debug_serial.println(" (parsed): OK");
  return std::vector<uint8_t>();
}

std::vector<uint8_t> JN5189::ISP::device_info() {
  uint8_t payload[] = {};
  size_t payload_length = 0;

  auto bytes_sent = __send_req(0x32, payload, payload_length);
  auto res = __recv_res();

  if (res[4] != 0x00) {
    debug_serial.printf("<-- (ISP): Error 0x%02X\n", res[4]);
    return std::vector<uint8_t>();
  }

  debug_serial.println(" (parsed): OK");
  return std::vector<uint8_t>();
}

std::vector<uint8_t> JN5189::ISP::memory_info(uint8_t memory_id) {
  uint8_t payload[] = {memory_id};
  size_t payload_length = 1;

  auto bytes_sent = __send_req(0x4C, payload, payload_length);
  auto res = __recv_res();

  if (res[4] != 0x00) {
    debug_serial.printf("<-- (ISP): Error 0x%02X\n", res[4]);
    return std::vector<uint8_t>();
  }

  // auto memory_id  = res[5];
  uint32_t base_addr = res[6] | (res[7] << 8) | (res[8] << 16) | (res[9] << 24);
  size_t length      = res[10] | (res[11] << 8) | (res[12] << 16) | (res[13] << 24);
  size_t sector_size = res[14] | (res[15] << 8) | (res[16] << 16) | (res[17] << 24);
  auto type          = res[18];
  auto access        = res[19];

  debug_serial.printf(" (parsed): Memory ID: %u\n", memory_id);
  debug_serial.printf(" (parsed): Base address: 0x%08X\n", base_addr);
  debug_serial.printf(" (parsed): Length: 0x%08X\n", length);
  debug_serial.printf(" (parsed): Sector size: 0x%08X\n", sector_size);
  debug_serial.printf(" (parsed): Type: %u\n", type);
  debug_serial.printf(" (parsed): Access: 0x%02X\n", access);
  debug_serial.print(" (parsed): Name: ");
  for (size_t i = 20; i < res.size() - 4; i++) {
    debug_serial.printf("%c", res[i]);
  }
  debug_serial.println();

  return res;
}

std::vector<uint8_t> JN5189::ISP::memory_open(uint8_t memory_id, uint8_t access_level) {
  uint8_t payload[] = {memory_id, access_level};
  size_t payload_length = 2;

  auto bytes_sent = __send_req(0x40, payload, payload_length);
  auto res = __recv_res();

  if (res[4] != 0x00) {
    debug_serial.printf("<-- (ISP): Error 0x%02X\n", res[4]);
    return std::vector<uint8_t>();
  }

  debug_serial.println(" (parsed): OK");
  return std::vector<uint8_t>();
}

std::vector<uint8_t> JN5189::ISP::memory_close() {
  uint8_t payload[] = {0x00};
  size_t payload_length = 1;

  auto bytes_sent = __send_req(0x4A, payload, payload_length);
  auto res = __recv_res();

  if (res[4] != 0x00) {
    debug_serial.printf("<-- (ISP): Error 0x%02X\n", res[4]);
    return std::vector<uint8_t>();
  }

  debug_serial.println(" (parsed): OK");
  return std::vector<uint8_t>();
}

std::vector<uint8_t> JN5189::ISP::memory_read(uint32_t address, size_t length) {
  uint8_t payload[10];
  size_t payload_length = 10;

  payload[0] = 0x00;                    // Handle returned by open memory command: always use 0
  payload[1] = 0x00;                    // Read mode: always use 0
  payload[2] = address & 0xFF;          // Address within memory to start reading from
  payload[3] = (address >> 8) & 0xFF;
  payload[4] = (address >> 16) & 0xFF;
  payload[5] = (address >> 24) & 0xFF;
  payload[6] = length & 0xFF;           // Number of bytes to read
  payload[7] = (length >> 8) & 0xFF;
  payload[8] = (length >> 16) & 0xFF;
  payload[9] = (length >> 24) & 0xFF;

  auto bytes_sent = __send_req(0x46, payload, payload_length);
  auto res = __recv_res();

  if (res[4] != 0x00) {
    debug_serial.printf("<-- (ISP): Error 0x%02X\n", res[4]);
    return std::vector<uint8_t>();
  }

  debug_serial.println(" (parsed): OK");
  return std::vector<uint8_t>();
}

size_t JN5189::ISP::__send_req(uint8_t type, const uint8_t payload[], size_t payload_length) {
  // Standard packets format:
  //  - 1 byte for flags
  //  - 2 bytes for the request's length (in bytes)
  //  - 1 byte for the request's type
  //  - n bytes of payload
  //  - 4 bytes of CRC-32 checksum
  uint16_t request_length = 1 + 2 + 1 + payload_length + 4;
  uint8_t *request = new uint8_t[request_length];

  // Copy bytes
  request[0] = 0x00;
  request[1] = (request_length >> 8) & 0xFF;
  request[2] = request_length & 0xFF;
  request[3] = type;

  for (size_t i = 0; i < payload_length; i++) {
    request[i + 4] = payload[i];
  }

  // Compute CRC-32
  uint32_t checksum = crc32(request, request_length - 4);
  request[request_length - 4] = (checksum >> 24) & 0xFF;
  request[request_length - 3] = (checksum >> 16) & 0xFF;
  request[request_length - 2] = (checksum >> 8) & 0xFF;
  request[request_length - 1] = checksum & 0xFF;

  // Debug output
  debug_serial.print("--> (ISP):");
  for (size_t i = 0; i < request_length; i++) {
    debug_serial.printf(" %02X", request[i]);
  }
  debug_serial.println();

  // Send request
  size_t bytes_sent = isp_serial.write(request, request_length);
  delete request;

  return bytes_sent;
}

std::vector<uint8_t> JN5189::ISP::__recv_res() {
  std::vector<uint8_t> response;

  // Busy wait response
  while (!isp_serial.available());

  // Copy response to buffer
  while (isp_serial.available()) {
    uint8_t new_byte = isp_serial.read();
    response.push_back(new_byte);
  }

  // Debug output
  debug_serial.print("<-- (ISP):");
  for (size_t i = 0; i < response.size(); i++) {
    debug_serial.printf(" %02X", response[i]);
  }
  debug_serial.println();

  return response;
}
