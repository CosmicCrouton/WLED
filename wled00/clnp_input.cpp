#include "wled.h"

#ifdef WLED_ENABLE_CLNP

#ifdef ESP8266
#error CLNP input is only supported on ESP32
#endif

#include "driver/uart.h"
#include "esp_check.h"

#include "clnp_input.h"
#include "clnp_protocol.h"

void clnpReceiverTask(void *context)
{
  uart_event_t event;  // Variable to hold UART event
  CLNPInput *instance = static_cast<CLNPInput *>(context);
  if (instance == nullptr) {
    return;
  }

  clnp_protocol::setup();

  if (instance->installDriver()) {
    while (true) {
      // Wait for UART events
      if(xQueueReceive(instance->uart_queue, (void * )&event, portMAX_DELAY)) {
          bzero(instance->clnpdata, CLNP_PACKET_SIZE);  // Clear the buffer

          DEBUG_PRINTF("uart[%d] event:", instance->inputPortNum);  // Log the event
          size_t read_len = 0;
          size_t bytes_processed = 0;

          unsigned long start_time;
          unsigned long end_time;
          unsigned long execution_time;


          switch(event.type) {
              case UART_DATA:
                  // Handle UART data received event
                  uart_get_buffered_data_len(instance->inputPortNum, &event.size);  // Get the buffered data length
                  read_len = uart_read_bytes(instance->inputPortNum, instance->clnpdata, event.size, 0);  // Read received data

                  start_time = micros();

                  while (bytes_processed < read_len) {
                      bytes_processed += clnp_protocol::process_transport(&instance->clnpdata[bytes_processed],
                                                                          read_len - bytes_processed);
                  }
                  end_time = micros();

                  execution_time = end_time - start_time; // Time in microseconds

                  DEBUG_PRINTF("Function execution time: %lu microseconds, size: %i\n", execution_time, event.size);
                  break;

              case UART_FIFO_OVF:
                  // Handle UART FIFO overflow event
                  DEBUG_PRINTF("FIFO overflow detected\n");
                  uart_flush_input(instance->inputPortNum);  // Clear the input buffer
                  break;

              case UART_BUFFER_FULL:
                  // Handle UART buffer full event
                  DEBUG_PRINTF("Ring buffer is full\n");
                  uart_flush_input(instance->inputPortNum);  // Clear the input buffer
                  break;

              case UART_PATTERN_DET:  //Unused
                  break;

              case UART_BREAK:
                  // Handle UART break event
                  DEBUG_PRINTF("UART break detected\n");
                  break;

              default:
                  // Handle other unknown UART events
                  DEBUG_PRINTF("Unknown uart event type: %d\n", event.type);
                  break;
          }
      }
    }
  }
}

bool CLNPInput::installDriver()
{
  DEBUG_PRINTF("CLNP port: %u\n", inputPortNum);
  if (setup_uart(inputPortNum, baudRate, &uart_queue) != ESP_OK) {
    DEBUG_PRINTF("Error: Failed to install clnp driver\n");
    return false;
  }

  DEBUG_PRINTF("CLNP RX pin: %u\n", rxPin);
  DEBUG_PRINTF("CLNP TX pin: %u\n", txPin);
  DEBUG_PRINTF("CLNP EN pin: %u\n", enPin);
  uart_set_pin(inputPortNum, txPin, rxPin, enPin, UART_PIN_NO_CHANGE);

  initialized = true;
  DEBUG_PRINTLN("CLNP initialized!");
  return true;
}

void CLNPInput::init(uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t inputPortNum, uint32_t baud_rate)
{
  if (inputPortNum <= (SOC_UART_NUM - 1) && inputPortNum > 0) {
    this->inputPortNum = inputPortNum;
  } else {
    DEBUG_PRINTF("CLNPInput: Error: invalid inputPortNum: %d\n", inputPortNum);
    return;
  }

  baudRate = baud_rate;

  if (rxPin > 0 && enPin > 0 && txPin > 0) {

    const managed_pin_type pins[] = {
        {(int8_t)txPin, false}, // these are not used as gpio pins, thus isOutput is always false.
        {(int8_t)rxPin, false},
        {(int8_t)enPin, false}};
    const bool pinsAllocated = PinManager::allocateMultiplePins(pins, 3, PinOwner::CLNP);
    if (!pinsAllocated) {
      DEBUG_PRINTF("CLNPInput: Error: Failed to allocate pins for CLNP. Pins already in use:\n");
      //DEBUG_PRINTF("rx in use by: %i\n", PinManager::getPinOwner(rxPin));
      //DEBUG_PRINTF("tx in use by: %i\n", PinManager::getPinOwner(txPin));
      //DEBUG_PRINTF("en in use by: %i\n", PinManager::getPinOwner(enPin));
      return;
    }

    this->rxPin = rxPin;
    this->txPin = txPin;
    this->enPin = enPin;

    // put clnp receiver into seperate task because it should not be blocked
    // pin to core 0 because wled is running on core 1
    DEBUG_PRINTLN("creating clnp task...");
    xTaskCreatePinnedToCore(clnpReceiverTask, "CLNP_RCV_TASK", 10240, this, 2, &task, 0);
    if (!task) {
      DEBUG_PRINTF("Error: Failed to create clnp rcv task");
    }
  }
  else {
    DEBUG_PRINTLN("CLNP input disabled due to rxPin, enPin or txPin not set");
    return;
  }
}

void CLNPInput::checkAndUpdateConfig()
{

  /**
   * The global configuration variables are modified by the web interface.
   * If they differ from the driver configuration, we have to update the driver
   * configuration.
   */

  //const uint16_t currentAddr = clnp_get_start_address(inputPortNum);
  //if (currentAddr != CLNPAddress) {
    //DEBUG_PRINTF("CLNP address has changed from %d to %d\n", currentAddr, CLNPAddress);
    //clnp_set_start_address(inputPortNum, CLNPAddress);
  //}
}

esp_err_t CLNPInput::setup_uart(clnp_port_t clnp_num, uint32_t baud_rate, QueueHandle_t *uart_queue) {

  ESP_RETURN_ON_FALSE(clnp_num < CLNP_NUM_MAX, false, "CLNP", "clnp_num error");

  // Configure UART parameters
  uart_config_t uart_config = {
      .baud_rate = baud_rate,            // Set baud rate
      .data_bits = UART_DATA_8_BITS,     // Set data bits to 8
      .parity = UART_PARITY_DISABLE,     // Disable parity check
      .stop_bits = UART_STOP_BITS_1,     // Set stop bits to 1
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Disable hardware flow control
      .source_clk = UART_SCLK_APB,       // Use APB clock
  };

  ESP_RETURN_ON_ERROR(uart_driver_install(clnp_num, CLNP_PACKET_SIZE_MAX * 2, CLNP_PACKET_SIZE_MAX * 2, 20, uart_queue, 0),
                  "CLNP",
                  "UART driver install failed");

  // Apply UART configuration
  ESP_RETURN_ON_ERROR(uart_param_config(clnp_num, &uart_config), "CLNP", "UART param config failed");

  // Setup UART in rs485 half duplex mode
  ESP_RETURN_ON_ERROR(uart_set_mode(clnp_num, UART_MODE_RS485_HALF_DUPLEX), "CLNP", "UART set mode failed");

  //uart_set_rts(clnp_num, 1);

  ESP_RETURN_ON_ERROR(uart_enable_rx_intr(clnp_num), "CLNP", "UART enable rx intr failed");
  ESP_RETURN_ON_ERROR(uart_set_rx_timeout(clnp_num, 3), "CLNP", "UART set rx timeout failed");

  ESP_RETURN_ON_ERROR(uart_flush(clnp_num), "CLNP", "UART flush failed");

  return ESP_OK;
}

uint32_t clnp_get_baud_rate(clnp_port_t clnp_num) {
  ESP_RETURN_ON_FALSE(clnp_num < CLNP_NUM_MAX, 0, "CLNP", "clnp_num error");

  uint32_t baud_rate;
  //taskENTER_CRITICAL(CLNP_SPINLOCK(clnp_num));
  uart_get_baudrate(clnp_num, &baud_rate);
  //taskEXIT_CRITICAL(CLNP_SPINLOCK(clnp_num));

  return baud_rate;
}

uint32_t clnp_set_baud_rate(clnp_port_t clnp_num, uint32_t baud_rate) {
  ESP_RETURN_ON_FALSE(clnp_num < CLNP_NUM_MAX, 0, "CLNP", "clnp_num error");

  // Clamp the baud rate to within CLNP specification
  if (baud_rate < CLNP_BAUD_RATE_MIN) {
    baud_rate = CLNP_BAUD_RATE_MIN;
  } else if (baud_rate > CLNP_BAUD_RATE_MAX) {
    baud_rate = CLNP_BAUD_RATE_MAX;
  }

  //taskENTER_CRITICAL(CLNP_SPINLOCK(clnp_num));
  uart_set_baudrate(clnp_num, baud_rate);
  //taskEXIT_CRITICAL(CLNP_SPINLOCK(clnp_num));

  return baud_rate;
}

#endif