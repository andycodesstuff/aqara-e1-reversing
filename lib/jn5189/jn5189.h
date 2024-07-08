#ifndef MY_JN5189_H_
#define MY_JN5189_H_

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <vector>

#include "crc32.h"

namespace JN5189 {
  class ISP {
    private:
      HardwareSerial &isp_serial;
      HardwareSerial &debug_serial;

      size_t __send_req(uint8_t type, const uint8_t payload[], size_t payload_length);
      std::vector<uint8_t> __recv_res();

    public:
      ISP(HardwareSerial &isp_serial, HardwareSerial &debug_serial):
        isp_serial(isp_serial),
        debug_serial(debug_serial) {}

      // 
      // Reference: https://www.nxp.com/webapp/Download?colCode=UM11138
      //            JN5189 User Manual, Rev. 1.5 - May 2021
      //            Chapter 38: In-System Programming (ISP)
      //            38.6.1.12 Unlock ISP
      //            Table 94: Request payload mode descriptions
      // 
      // Available ISP unlock modes:
      //  - 0x00 -> Default state: only Get Device Info command works in this mode
      //  - 0x01 -> Start ISP functionality: all commands work in this mode if the device is not
      //            locked. Key (n = 16): 0x11223344556677881122334455667788
      // 
      enum ISPUnlockMode {
        MODE_DEFAULT_STATE = 0x00,
        MODE_START_ISP = 0x01
      };

      std::vector<uint8_t> unlock(ISPUnlockMode unlock_mode);
      std::vector<uint8_t> device_info();

      // 
      // Reference: https://www.nxp.com/webapp/Download?colCode=UM11138
      //            JN5189 User Manual, Rev. 1.5 - May 2021
      //            Chapter 38: In-System Programming (ISP)
      //            38.6.1.11 Get Memory Info
      //            Table 90: JN5189(T)/JN5188(T) supported memory ID
      // 
      // Available memory regions with their IDs are available below:
      //  - 0 -> FLASH
      //  - 1 -> PSECT
      //  - 2 -> pFlash
      //  - 3 -> Config
      //  - 4 -> EFUSE
      //  - 5 -> ROM
      //  - 6 -> RAM0
      //  - 7 -> RAM1
      // 
      #define MEMORY_FLASH 0
      #define MEMORY_PSECT 1
      #define MEMORY_PFLASH 2
      #define MEMORY_CONFIG 3
      #define MEMORY_EFUSE 4
      #define MEMORY_ROM 5
      #define MEMORY_RAM0 6
      #define MEMORY_RAM1 7

      // 
      // Reference: https://www.nxp.com/webapp/Download?colCode=UM11138
      //            JN5189 User Manual, Rev. 1.5 - May 2021
      //            Chapter 38: In-System Programming (ISP)
      //            38.6.1.11 Get Memory Info
      //            Table 92: JN5189(T)/JN5188(T) available access bit values
      // 
      // Available memory access levels with their IDs are available below:
      //  - 0 -> Read enabled
      //  - 1 -> Write enabled
      //  - 2 -> Erase enabled
      //  - 3 -> Erase all enabled
      //  - 4 -> Blank check enabled
      // 
      #define ACCESS_READ_ENABLED 0
      #define ACCESS_WRITE_ENABLED 1
      #define ACCESS_ERASE_ENABLED 2
      #define ACCESS_ERASE_ALL_ENABLED 3
      #define ACCESS_BLANK_CHECK_ENABLED 4

      std::vector<uint8_t> memory_info(uint8_t memory_id);
      std::vector<uint8_t> memory_open(uint8_t memory_id, uint8_t access_level);
      std::vector<uint8_t> memory_close();
      std::vector<uint8_t> memory_read(uint32_t address, size_t length);
  };

  class SWD {
    private:
      uint8_t swd_clk_pin;
      uint8_t swd_io_pin;
      HardwareSerial &debug_serial;
      uint8_t log_level;

      // Drive the clock line at a frequency of 500 KHz
      // This is the maximum speed achieveable with microseconds delays
      // 
      //      T = 0.000002 sec
      //      f = 1 / T = 1 / 0.000002 = 500,000 Hz or 500 KHz
      // 
      #define CLOCK_HALF_CYCLE_MICROSEC 1
      #define RESET_SEQUENCE_LENGTH 50
      #define SYNC_SEQUENCE_LENGTH 8
      #define REG_ACCESS_MAX_RETRIES 16

      enum RequestAPnDP {
        APnDP_DP = 0b0, // Request access to a debug port register
        APnDP_AP = 0b1  // Request access to an access port register
      };

      enum RequestRnW {
        RnW_WRITE = 0b0,  // Request write access to the selected register
        RnW_READ  = 0b1   // Request read access to the selected register
      };

      enum ResponseACK {
        ACK_OK    = 0b001,
        ACK_WAIT  = 0b010,
        ACK_FAULT = 0b100
      };

      uint8_t __access_register(RequestAPnDP APnDP, RequestRnW RnW, uint8_t address, uint32_t * const &value, int max_retries = REG_ACCESS_MAX_RETRIES);
      uint8_t __access_register_backend(RequestAPnDP APnDP, RequestRnW RnW, uint8_t address, uint32_t * const &value);
      uint32_t __read_bits(size_t n_bits);
      void __write_bits(uint32_t value, size_t n_bits);
      bool __parity_bit(uint32_t value);

      void __switch_mode_jtag_to_swd();
      void __line_reset();
      void __sync();
      void __turnaround_period();
      void __pulse_clock();
      void __setup_master_read();
      void __setup_master_write();

    public:
      SWD(uint8_t swd_clk_pin, uint8_t swd_io_pin, HardwareSerial &debug_serial, uint8_t log_level):
        swd_clk_pin(swd_clk_pin),
        swd_io_pin(swd_io_pin),
        debug_serial(debug_serial),
        log_level(log_level) {}

      #define LOG_LVL_NONE  0b00
      #define LOG_LVL_INFO  0b01
      #define LOG_LVL_DEBUG 0b11

      #define DEFAULT_DESIGNER 0x23B
      #define DEFAULT_PARTNO 0xBA01

      std::vector<uint8_t> memory_read(uint32_t address, size_t length);

      bool connect();
      void cpu_halt();
      void cpu_resume();

      // 
      // Reference: https://developer.arm.com/documentation/ihi0031/g
      //            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
      //            Chapter B2: DP Reference Information
      //            Table B2-4, B2-5: DPv1 register map; SW-DP data link defined registers, DPv1
      // 
      #define REG_DP_IDCODE    0x00 // R
      #define REG_DP_ABORT     0x00 // W
      #define REG_DP_CTRL_STAT 0x04 // R/W if CTRLSEL bit in the SELECT register is 0
      #define REG_DP_WCR       0x04 // R/W if CTRLSEL bit in the SELECT register is 1
      #define REG_DP_RESEND    0x08 // R
      #define REG_DP_SELECT    0x08 // W
      #define REG_DP_RDBUFF    0x0C // R

      // 
      // Reference: https://developer.arm.com/documentation/ddi0439/b
      //            Cortex-M4 Technical Reference Manual - Revision r0p0
      //            Chapter 8: Debug
      //            Table 8-5: AHB-AP register summary
      // 
      // Reference: https://developer.arm.com/documentation/ihi0031/g
      //            ARM® Debug Interface Architecture Specification - ADIv5.0 to ADIv5.2
      //            Chapter C2: The Memory Access Port
      //            Table C2-6: MEM-AP programmers’ model
      // 
      #define REG_AP_CSW  0x00 // R/W
      #define REG_AP_TAR  0x04 // R/W
      #define REG_AP_DRW  0x0C // R/W
      #define REG_AP_BD0  0x10 // R/W
      #define REG_AP_BD1  0x14 // R/W
      #define REG_AP_BD2  0x18 // R/W
      #define REG_AP_BD3  0x1C // R/W
      #define REG_AP_BASE 0xF8 // R
      #define REG_AP_IDR  0xFC // R

      uint8_t read_DP(uint8_t address, uint32_t * const &value);
      uint8_t read_AP(uint8_t address, uint32_t * const &value);
      uint8_t write_DP(uint8_t address, uint32_t value);
      uint8_t write_AP(uint8_t address, uint32_t value); 
  };
}

#endif
