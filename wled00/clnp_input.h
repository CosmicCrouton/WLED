#pragma once
#include <cstdint>
#include <atomic>
#include <mutex>

#define CLNP_BAUD_RATE_MIN 600
#define CLNP_BAUD_RATE_MAX 153000
#define CLNP_PACKET_SIZE 60
#define CLNP_PACKET_SIZE_MAX 270

/** @brief CLNP port constants.*/
enum clnp_num_t {
  CLNP_NUM_0, /** @brief CLNP port 0.*/
  CLNP_NUM_1, /** @brief CLNP port 1.*/
#if SOC_UART_NUM > 2
  CLNP_NUM_2, /** @brief CLNP port 2.*/
#endif
  CLNP_NUM_MAX
};

/** @brief CLNP port type.*/
typedef unsigned int clnp_port_t;

/*
 * Support for CLNP input via serial (e.g. max485) on ESP32
 * ESP32 Library from:
 * https://github.com/cosmiccrouton/esp_clnp
 */
class CLNPInput
{
public:
  void init(uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t inputPortNum, uint32_t baud_rate);

  /// True if clnp is currently connected
  bool isConnected() const { return connected; }

private:
  /**
   * Checks if the global clnp config has changed and updates the changes in rdm
   */
  void checkAndUpdateConfig();

  /// overrides everything and turns on all leds
  void turnOnAllLeds();

  /// installs the clnp driver
  /// @return false on fail
  bool installDriver();

  /// sets up the uart driver
  /// @return false on fail
  esp_err_t setup_uart(clnp_port_t clnp_num, uint32_t baud_rate, QueueHandle_t *uart_queue);

  /// The internal clnp task.
  /// This is the main loop of the clnp receiver. It never returns.
  friend void clnpReceiverTask(void * context);

  uint8_t inputPortNum = 255;
  uint8_t rxPin = 255;
  uint8_t txPin = 255;
  uint8_t enPin = 255;
  uint32_t baudRate = 19200;

  /// is written to by the clnp receive task.
  byte clnpdata[CLNP_PACKET_SIZE];
  /// True once the clnp input has been initialized successfully
  bool initialized = false; // true once init finished successfully
  /// True if clnp is currently connected
  std::atomic<bool> connected{false};
  std::atomic<bool> identify{false};
  /// Timestamp of the last time a clnp frame was received
  unsigned long lastUpdate = 0;

  /// Taskhandle of the clnp task that is running in the background
  TaskHandle_t task;
  QueueHandle_t uart_queue;

  /// Guards access to clnpData
  std::mutex clnpDataLock;
};
