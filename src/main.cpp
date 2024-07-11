#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <iostream>
#include <sstream>
#include <vector>

#include "env.h"
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
#define SERVER_PORT 1337

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

std::string & trim(std::string &s);
std::vector<std::string> split(const std::string &s, char delim);
Command command_hash(const std::string &s);

void JN5189_TurnOn();
void JN5189_TurnOff();
void JN5189_Reset();
void JN5189_ISPModeEnter();
void JN5189_SWDModeEnter();
void JN5189_SWDModeLeave();

void server_handle_new_client(void *args, AsyncClient *client);
void server_handle_data(void *args, AsyncClient *client, void *data, size_t data_len);
void server_handle_error(void *args, AsyncClient *client, int8_t err);
void server_handle_timeout(void *args, AsyncClient *client, uint32_t timestamp);
void server_handle_disconnect(void *args, AsyncClient *client);
void server_handle_command(AsyncClient *client, const std::string &command);

void setup() {
  // Initialise serial monitor connection
  Serial.begin(SERIAL_MONITOR_BAUD_RATE);
  Serial.println("Serial monitor initialised");

  // Initialise UART connection
  UARTConnection.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART2 initialised");

  // Initialise WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Connecting to \"%s\"", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();
  Serial.print("Connected with IPv4 address ");
  Serial.println(WiFi.localIP());

  // Initialise TCP server
  auto server = new AsyncServer(SERVER_PORT);
  server->onClient(&server_handle_new_client, nullptr);
  server->begin();

  // Initialise ISP and SWD helpers
  ISP = new JN5189::ISP(UARTConnection, Serial);
  SWD = new JN5189::SWD(SWCLK_PIN, SWDIO_PIN, Serial, LOG_LVL_NONE);

  // Initialise GPIO pins
  pinMode(PWR_PIN, OUTPUT);
  pinMode(RSTN_PIN, OUTPUT);
  pinMode(DIO4_PIN, OUTPUT);
  pinMode(DIO5_PIN, OUTPUT);
}

void loop() {}

std::string & trim(std::string &s) {
  while (!s.empty() && isspace(s.front())) s.erase(s.begin());
  while (!s.empty() && isspace(s.back())) s.pop_back();

  return s;
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

void server_handle_new_client(void *args, AsyncClient *client) {
  auto remote_ip = client->remoteIP();

  Serial.print("[server] client ");
  Serial.print(remote_ip);
  Serial.print(" connected");
  Serial.println();

  // Register event handlers
  client->onData(server_handle_data, nullptr);
  client->onError(server_handle_error, nullptr);
  client->onTimeout(server_handle_timeout, nullptr);
  client->onDisconnect(server_handle_disconnect, nullptr);

  // Show command prompt
  client->write("> ");
}

void server_handle_data(void *args, AsyncClient *client, void *data, size_t data_len) {
  auto remote_ip = client->remoteIP();

  // Copy raw data into an std::string
  char *c_str = new char[data_len + 1];
  memcpy(c_str, data, data_len);
  c_str[data_len] = '\0';

  auto command = std::string(c_str);
  trim(command);
  delete c_str;

  Serial.print("[server] received data from ");
  Serial.print(remote_ip);
  Serial.print(" \"");
  Serial.print(command.c_str());
  Serial.print("\"");
  Serial.println();

  server_handle_command(client, command);
}

void server_handle_error(void *args, AsyncClient *client, int8_t err) {
  auto remote_ip = client->remoteIP();

  Serial.print("[server] connection error \"");
  Serial.print(client->errorToString(err));
  Serial.print("\" from ");
  Serial.print(remote_ip);
  Serial.println();
}

void server_handle_timeout(void *args, AsyncClient *client, uint32_t timestamp) {
  auto remote_ip = client->remoteIP();

  Serial.print("[server] received timeout from ");
  Serial.print(remote_ip);
  Serial.println();
}

void server_handle_disconnect(void *args, AsyncClient *client) {
  auto remote_ip = client->remoteIP();

  Serial.print("[server] client ");
  Serial.print(remote_ip);
  Serial.print(" disconnected");
  Serial.println();
}

void server_handle_command(AsyncClient *client, const std::string &command) {
  auto args = split(command, ' ');

  if (args.empty()) {
    client->write("> ");
    return;
  }

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
        client->write("isp-mem-info requires 1 positional argument: MEMORY_ID\n");
        break;
      }

      auto memory_id = std::stoi(args[1], 0, 16);

      ISP->memory_info(memory_id);
      break;
    }
    case Command::ISP_MEM_OPEN: {
      if (args.size() < 3) {
        client->write("isp-mem-open requires 2 positional arguments: MEMORY_ID, ACCESS_LEVEL\n");
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
        client->write("isp-mem-read requires 2 positional arguments: ADDRESS, LENGTH\n");
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
        client->write("swd-mem-read requires 2 positional arguments: ADDRESS, LENGTH\n");
        break;
      }

      char formatted_byte[4];
      auto address = std::stoi(args[1], 0, 16);
      auto length  = std::stoi(args[2], 0, 16);
      auto data    = SWD->memory_read(address, length);

      // Intentionally skip the string terminator as we're really sending raw bytes here, yet
      // we're leveraging the convenience of strings and string formatting
      client->add("Data:", 5);
      for (auto byte : data) {
        sprintf(formatted_byte, " %02X", byte);
        client->add(formatted_byte, 3);
      }
      client->add("\n", 1);

      client->send();
      break;
    }
    default: {
      auto msg = "Unrecognised command \"" + command + "\"\n";

      client->write(msg.c_str());
    }
  }

  // Show command prompt
  client->write("> ");
}
