#pragma once
#include <cstdint>
#include <atomic>
#include <mutex>

#include "freertos/message_buffer.h"

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


class CLNPInput
{
public:
  esp_err_t init(clnp_num_t clnp_num, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint32_t baud_rate);

  static MessageBufferHandle_t clnp_tx_message_buffer;

private:
  /**
   * Checks if the global clnp config has changed and updates the changes in rdm
   */
  void checkAndUpdateConfig();

  /// installs the clnp driver
  /// @return false on fail
  esp_err_t installDriver(clnp_num_t clnp_num, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint32_t baud_rate);

  /// sets up the uart driver
  /// @return false on fail
  esp_err_t setup_uart(clnp_port_t clnp_num, int baud_rate, QueueHandle_t uart_queue);

  /// sets up the internal uart driver used for resetting outputs
  esp_err_t setup_internal_uart();

  /// The internal clnp task.
  /// This is the main loop of the clnp receiver. It never returns.
  friend void clnpReceiverTask(void* context);

  /// The internal clnp task.
  /// This is the main loop of the clnp transmitter. It never returns.
  friend void clnpTransmitterTask(void* context);

  clnp_num_t clnp_num = CLNP_NUM_2;
  uint16_t tx_delay_ms = 0;

  /// Taskhandle of the clnp task that is running in the background
  TaskHandle_t task;
  static QueueHandle_t uart_queue;
};
