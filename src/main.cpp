#include <Arduino.h>
#include <iostream>
#include <sstream>
#include <vector>

#include "jn5189.h"

#define PWR_PIN 25
#define RSTN_PIN 26
#define DIO4_PIN 32
#define DIO5_PIN 33
#define UART_RX_PIN 16
#define UART_TX_PIN 17
#define UART_BAUD_RATE 115200
#define SWCLK_PIN GPIO_NUM_18
#define SWDIO_PIN GPIO_NUM_23
#define SERIAL_MONITOR_BAUD_RATE 115200

enum Command {
  ON,
  OFF,
  RESET,
  ISP_ENTER,
  ISP_UNLOCK_DEFAULT,
  ISP_UNLOCK_FULL,
  ISP_DEVICE_INFO,
  ISP_MEM_INFO,
  ISP_MEM_OPEN,
  ISP_MEM_CLOSE,
  ISP_MEM_READ,
  SWD_ENTER,
  SWD_LEAVE,
  SWD_MEM_READ,
  NONE
};

HardwareSerial UARTConnection(2);
JN5189::ISP *ISP;
JN5189::SWD *SWD;
bool show_prompt;
std::string line;

std::string read_line(char new_char);
std::vector<std::string> split(const std::string &s, char delim);
Command command_hash(const std::string &s);

void JN5189_TurnOn();
void JN5189_TurnOff();
void JN5189_Reset();
void JN5189_ISPModeEnter();
void JN5189_SWDModeEnter();
void JN5189_SWDModeLeave();

void setup() {
  // Initialise serial monitor connection
  Serial.begin(SERIAL_MONITOR_BAUD_RATE);
  Serial.println("Serial monitor initialised");

  // Initialise UART connection
  UARTConnection.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART2 initialised");

  // Initialise ISP and SWD helpers
  ISP = new JN5189::ISP(UARTConnection, Serial);
  SWD = new JN5189::SWD(SWCLK_PIN, SWDIO_PIN, Serial, LOG_LVL_NONE);

  // Initialise GPIO pins
  pinMode(PWR_PIN, OUTPUT);
  pinMode(RSTN_PIN, OUTPUT);
  pinMode(DIO4_PIN, OUTPUT);
  pinMode(DIO5_PIN, OUTPUT);

  show_prompt = true;
  line = "";
}

void loop() {
  if (show_prompt) {
    Serial.print("> ");

    show_prompt = false;
  }

  // Parse command on enter
  std::string user_input = read_line(Serial.read());
  std::vector<std::string> args = split(user_input, ' ');
  if (args.size() == 0) return;

  switch (command_hash(args[0])) {
    case Command::ON: {
      JN5189_TurnOn();
      break;
    }
    case Command::OFF: {
      JN5189_TurnOff();
      break;
    }
    case Command::RESET: {
      JN5189_Reset();
      break;
    }
    case Command::ISP_ENTER: {
      JN5189_ISPModeEnter();
      break;
    }
    case Command::ISP_UNLOCK_DEFAULT: {
      ISP->unlock(JN5189::ISP::ISPUnlockMode::MODE_DEFAULT_STATE);
      break;
    }
    case Command::ISP_UNLOCK_FULL: {
      ISP->unlock(JN5189::ISP::ISPUnlockMode::MODE_START_ISP);
      break;
    }
    case Command::ISP_DEVICE_INFO: {
      ISP->device_info();
      break;
    }
    case Command::ISP_MEM_INFO: {
      if (args.size() < 2) {
        Serial.println("isp-mem-info requires 1 positional argument: MEMORY_ID");
        break;
      }

      auto memory_id = std::stoi(args[1], 0, 16);

      ISP->memory_info(memory_id);
      break;
    }
    case Command::ISP_MEM_OPEN: {
      if (args.size() < 3) {
        Serial.println("isp-mem-open requires 2 positional arguments: MEMORY_ID, ACCESS_LEVEL");
        break;
      }

      auto memory_id = std::stoi(args[1], 0, 16);
      auto access_level = std::stoi(args[2], 0, 16);

      ISP->memory_open(memory_id, access_level);
      break;
    }
    case Command::ISP_MEM_CLOSE: {
      ISP->memory_close();
      break;
    }
    case Command::ISP_MEM_READ: {
      if (args.size() < 3) {
        Serial.println("isp-mem-read requires 2 positional arguments: ADDRESS, LENGTH");
        break;
      }

      auto address = std::stoi(args[1], 0, 16);
      auto length = std::stoi(args[2], 0, 16);

      ISP->memory_read(address, length);
      break;
    }
    case Command::SWD_ENTER: {
      JN5189_SWDModeEnter();
      break;
    }
    case Command::SWD_LEAVE: {
      JN5189_SWDModeLeave();
      break;
    }
    case Command::SWD_MEM_READ: {
      if (args.size() < 3) {
        Serial.println("swd-mem-read requires 2 positional arguments: ADDRESS, LENGTH");
        break;
      }

      auto address = std::stoi(args[1], 0, 16);
      auto length  = std::stoi(args[2], 0, 16);
      auto data    = SWD->memory_read(address, length);

      Serial.print("Data:");
      for (auto byte : data) Serial.printf(" %02X", byte);
      Serial.println();

      break;
    }
    default: {
      Serial.printf("Unrecognised command \"%s\"\n", user_input.c_str());
    }
  }

  show_prompt = true;
}

/**
 * Read a line from the serial monitor ended by (CR)LF. The input is echoed back to the user so
 * they can see the characters being typed
 */
std::string read_line(char new_char) {
  // Character 0xFF (255) is used by Serial.read() to signify no data from the serial connection
  if (new_char > 0 && new_char < 255) {
    switch (new_char) {
      case '\r':
        break;
      case '\n': {
        std::string ret_val(line);
        line = "";

        Serial.println();

        return ret_val;
      }
      default: {
        Serial.print(new_char);
        line += new_char;
      }
    }
  }

  return "";
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;

  while (getline(ss, item, delim)) {
    result.push_back(item);
  }

  return result;
}

Command command_hash(const std::string &s) {
  if (s == "on")                 return Command::ON;
  if (s == "off")                return Command::OFF;
  if (s == "reset")              return Command::RESET;
  if (s == "isp")                return Command::ISP_ENTER;
  if (s == "isp-unlock-default") return Command::ISP_UNLOCK_DEFAULT;
  if (s == "isp-unlock-full")    return Command::ISP_UNLOCK_FULL;
  if (s == "isp-device-info")    return Command::ISP_DEVICE_INFO;
  if (s == "isp-mem-info")       return Command::ISP_MEM_INFO;
  if (s == "isp-mem-open")       return Command::ISP_MEM_OPEN;
  if (s == "isp-mem-close")      return Command::ISP_MEM_CLOSE;
  if (s == "isp-mem-read")       return Command::ISP_MEM_READ;
  if (s == "swd")                return Command::SWD_ENTER;
  if (s == "swd-leave")          return Command::SWD_LEAVE;
  if (s == "swd-mem-read")       return Command::SWD_MEM_READ;

  return Command::NONE;
}

/**
 * Turn on the CPU by connecting ground
 */
void JN5189_TurnOn() {
  digitalWrite(PWR_PIN, HIGH);
}

/**
 * Turn off the CPU by disconnecting ground
 */
void JN5189_TurnOff() {
  digitalWrite(PWR_PIN, LOW);
}

/**
 * Reset the CPU by pulsing the RSTN pin low
 * 
 * The low pulse has to be longer than the minimum pulse width to successfully generate a reset.
 * We can take a safe guess and pulse for 1 millisecond.
 */
void JN5189_Reset() {
  digitalWrite(RSTN_PIN, HIGH);
  delay(1);
  digitalWrite(RSTN_PIN, LOW);
}

/**
 * Enter ISP mode
 * 
 * Reference: https://www.nxp.com/webapp/Download?colCode=UM11138
 *            JN5189 User Manual, Rev. 1.5 - May 2021
 *            Chapter 38: In-System Programming (ISP)
 *            38.4 Physical interface
 * 
 * The ISP protocol is implemented on USART 0, using DIO9 for RX and DIO8 for TX. The baud rate
 * is 115200 with formatting of 8 bits per character, no parity bit and 1 stop bit. In the
 * default configuration, ISP mode is entered during a cold start if DIO5 is pulled low and DIO4
 * is pulled high.
 */
void JN5189_ISPModeEnter() {
  // Turn the device off
  JN5189_TurnOff();
  delay(5);

  // Pull DIO4 high and DIO5 low, then turn the device on
  // NOTE: If there is no external driver into DIO4, then the internal pull-up will keep the pin
  //       high
  digitalWrite(DIO4_PIN, LOW);
  digitalWrite(DIO5_PIN, HIGH);
  JN5189_TurnOn();
  delay(5);

  // Restore pins to their default state
  digitalWrite(DIO4_PIN, LOW);
  digitalWrite(DIO5_PIN, LOW);
  delay(5);

  // Consume jibberish that sometimes appears at the beginning of a transmission
  while (UARTConnection.available()) UARTConnection.read();

  // Unlock full ISP functionalities
  // Step 1: Unlock ISP to default state
  ISP->unlock(JN5189::ISP::ISPUnlockMode::MODE_DEFAULT_STATE);

  // Step 2: Get device info (optional, but it provides extra assurance that things are working)
  ISP->device_info();

  // Step 3: Unlock full ISP functionalities using the default key
  ISP->unlock(JN5189::ISP::ISPUnlockMode::MODE_START_ISP);
}

/**
 * Enter hardware test mode. Then, check if SWD is enabled
 * 
 * Reference: https://www.nxp.com/webapp/Download?colCode=UM11138
 *            JN5189 User Manual, Rev. 1.5 - May 2021
 *            Chapter 7: Reset, Boot and Wakeup
 *            7.3.1 Boot modes
 * 
 * Hardware test mode is entered during a cold start if both DIO4 and DIO5 are pulled low.
 * Furthermore, hardware test mode needs to be left enabled by the device manufacturer and the
 * SWD_DIS bit stored in the flash memory has to be set to 0.
 */
void JN5189_SWDModeEnter() {
  // Turn the device off
  JN5189_TurnOff();
  delay(5);

  // Pull DIO4 and DIO5 low, then turn the device on
  digitalWrite(DIO4_PIN, HIGH);
  digitalWrite(DIO5_PIN, HIGH);
  JN5189_TurnOn();
  delay(5);

  // Restore pins to their default state
  digitalWrite(DIO4_PIN, LOW);
  digitalWrite(DIO5_PIN, LOW);
  delay(5);

  pinMode(SWCLK_PIN, OUTPUT);
  pinMode(SWDIO_PIN, OUTPUT);

  Serial.println("Connecting via SWD...");
  auto swd_enabled = SWD->connect();

  // Ensure SWD has been enabled
  if (!swd_enabled) {
    Serial.println("SWD is disabled for this device");
    JN5189_TurnOff();
    return;
  }

  Serial.println("Found SWD device, halting CPU...");
  SWD->cpu_halt();
  Serial.println("Halted, initialization complete");
}

void JN5189_SWDModeLeave() {
  Serial.println("Resetting CPU...");
  SWD->cpu_resume();
  Serial.println("Reset");

  Serial.println("Disconnecting...");
  SWD->disconnect();
  Serial.println("Disconnected");

  pinMode(SWCLK_PIN, INPUT);
  pinMode(SWDIO_PIN, INPUT);
}
