#include "wled.h"

#ifdef WLED_ENABLE_CLNP

#ifdef ESP8266
#error CLNP input is only supported on ESP32
#endif

#include "driver/uart.h"
#include "esp_check.h"

#include "clnp_input.h"
#include "clnp_protocol.h"

#include "freertos/message_buffer.h"

QueueHandle_t CLNPInput::uart_queue = NULL;
MessageBufferHandle_t CLNPInput::clnp_tx_message_buffer = NULL;

void clnpTransmitterTask(void *context)
{
  CLNPInput *instance = static_cast<CLNPInput *>(context);
  if (instance == nullptr) {
    return;
  }

  size_t xReceivedBytes;
  uint8_t* tx_buff = (uint8_t*) malloc(CLNP_PACKET_SIZE_MAX);

  while(true)
  {
    if (xReceivedBytes = xMessageBufferReceive(CLNPInput::clnp_tx_message_buffer, tx_buff, CLNP_PACKET_SIZE_MAX, portMAX_DELAY ))
    {
      //Delay response by a device variable amount to help avoid bus collisions
      vTaskDelay(pdMS_TO_TICKS(instance->tx_delay_ms));

      if (uart_write_bytes(instance->clnp_num, tx_buff, xReceivedBytes) != xReceivedBytes) {
          DEBUG_PRINTF("Send data critical failure.");
      }
    }
  }

  free(tx_buff);
  tx_buff = NULL;
  vTaskDelete(NULL);
}

void clnpReceiverTask(void *context)
{
  CLNPInput *instance = static_cast<CLNPInput *>(context);
  if (instance == nullptr) {
    return;
  }

  uart_event_t event;  // Variable to hold UART event
  uint8_t* rx_buff = (uint8_t*) malloc(CLNP_PACKET_SIZE_MAX);

  unsigned long start_time;
  unsigned long end_time;
  unsigned long execution_time;

  while (true) {

    size_t read_len = 0;
    size_t bytes_processed = 0;
    size_t transport_byte_size = 0;

    // Wait for UART events
    if(xQueueReceive(CLNPInput::uart_queue, (void * )&event, portMAX_DELAY)) {
      bzero(rx_buff, CLNP_PACKET_SIZE_MAX);  // Clear the buffer

      DEBUG_PRINTF("uart event type: %d, size: %u, HWM: %u\n",
                   event.type, event.size, uxTaskGetStackHighWaterMark( NULL ));  // Log the event

      switch(event.type) {
        // Handle UART data received event
        case UART_DATA:
          read_len = uart_read_bytes(instance->clnp_num, rx_buff, event.size, 0);  // Read received data

          start_time = micros();

          //clnp_protocol::print_array("Received data:", instance->clnpdata, read_len);
          while (bytes_processed < read_len) {

              transport_byte_size = clnp_protocol::process_transport(&rx_buff[bytes_processed],
                                                                     read_len - bytes_processed);

              //If not a valid CLNP message, advance one byte at a time trying to resync with CLNP framing bytes.
              if (transport_byte_size == 0)
                bytes_processed += 1;

              bytes_processed += transport_byte_size;
          }



          end_time = micros();

          execution_time = end_time - start_time; // Time in microseconds

          DEBUG_PRINTF("Function execution time: %lu microseconds\n", execution_time);
          break;

        // Handle UART FIFO overflow event
        case UART_FIFO_OVF:
          DEBUG_PRINTF("FIFO overflow detected\n");
          uart_flush_input(instance->clnp_num);  // Clear the input buffer
          break;

        // Handle UART buffer full event
        case UART_BUFFER_FULL:
          DEBUG_PRINTF("Ring buffer is full\n");
          uart_flush_input(instance->clnp_num);  // Clear the input buffer
          break;

        // Unused
        case UART_PATTERN_DET:
          break;

        // Handle UART break event
        case UART_BREAK:
          DEBUG_PRINTF("UART break detected\n");
          break;

        // Handle other unknown UART events
        default:
          DEBUG_PRINTF("Unknown uart event type: %d\n", event.type);
          break;
      }
    }
  }

  free(rx_buff);
  rx_buff = NULL;
  vTaskDelete(NULL);
}

esp_err_t CLNPInput::init(clnp_num_t clnp_num, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint32_t baud_rate)
{
  DEBUG_PRINTF("CLNP port: %u, RX pin: %u, TX pin: %u, EN pin: %u\n", clnp_num, rxPin, txPin, enPin);
  DEBUG_PRINTF("RESET TX pin: %u, RESET RX pin: %u\n", GPIO_NUM_34, GPIO_NUM_15); //GPIO34, GPIO35

  ESP_RETURN_ON_FALSE(clnp_num < SOC_UART_NUM, ESP_FAIL, "CLNP", "Invalid CLNP port number");
  ESP_RETURN_ON_FALSE(rxPin > 0, ESP_FAIL, "CLNP", "RX pin not configured");
  ESP_RETURN_ON_FALSE(txPin > 0, ESP_FAIL, "CLNP", "TX pin not configured");
  ESP_RETURN_ON_FALSE(enPin > 0, ESP_FAIL, "CLNP", "EN pin not configured");

  const managed_pin_type pins[] = {
      {(int8_t)txPin, false}, // these are not used as gpio pins, thus isOutput is always false.
      {(int8_t)rxPin, false},
      {(int8_t)enPin, false},
      {(int8_t)GPIO_NUM_34, false},
      {(int8_t)GPIO_NUM_15, false}};

  const bool pinsAllocated = PinManager::allocateMultiplePins(pins, 5, PinOwner::CLNP);

  if (!pinsAllocated) {
    DEBUG_PRINTF("CLNPInput: Error: Failed to allocate pins for CLNP. Pins already in use!\n");
    return ESP_FAIL;
  }

  this->clnp_num = clnp_num;

  ESP_RETURN_ON_ERROR(clnp_protocol::setup(this->tx_delay_ms),
                      "CLNP", "Failed to setup CLNP protocol");

  ESP_RETURN_ON_ERROR(installDriver(clnp_num, rxPin, txPin, enPin, baud_rate),
                      "CLNP", "Failed to install driver");

  CLNPInput::clnp_tx_message_buffer = xMessageBufferCreate( CLNP_PACKET_SIZE_MAX * 8 );

  // put clnp receiver into seperate task because it should not be blocked
  // pin to core 0 because wled is running on core 1
  xTaskCreatePinnedToCore(clnpReceiverTask, "CLNP_RX_TASK", 4086, this, tskIDLE_PRIORITY+1, &task, 0);
  if (!task) {
    DEBUG_PRINTF("Error: Failed to create clnp rcv task");
    return ESP_FAIL;
  }

  xTaskCreatePinnedToCore(clnpTransmitterTask, "CLNP_TX_TASK", 4086, this, tskIDLE_PRIORITY, &task, 0);
  if (!task) {
    DEBUG_PRINTF("Error: Failed to create clnp rcv task");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t CLNPInput::installDriver(clnp_num_t clnp_num, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint32_t baud_rate)
{
  ESP_RETURN_ON_ERROR(uart_set_pin(clnp_num, txPin, rxPin, enPin, UART_PIN_NO_CHANGE),
                      "CLNP", "UART set pin failed");

  //ESP_RETURN_ON_ERROR(uart_set_pin(UART_NUM_1, GPIO_NUM_15, GPIO_NUM_34, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
  //                    "CLNP", "UART set pin failed");

  ESP_RETURN_ON_ERROR(setup_uart(clnp_num, baud_rate, CLNPInput::uart_queue), "CLNP", "Failed to setup clnp uart");

  //ESP_RETURN_ON_ERROR(setup_internal_uart(), "CLNP", "Failed to setup internal uart");

  DEBUG_PRINTLN("CLNP initialized!");

  return ESP_OK;
}


esp_err_t CLNPInput::setup_uart(clnp_port_t clnp_num, int baud_rate, QueueHandle_t queue) {

  ESP_RETURN_ON_FALSE(clnp_num < CLNP_NUM_MAX, false, "CLNP", "clnp_num error");

  // Configure UART parameters
  uart_config_t uart_config = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  ESP_RETURN_ON_ERROR(uart_driver_install(clnp_num, CLNP_PACKET_SIZE_MAX * 2, CLNP_PACKET_SIZE_MAX * 2, 20, &CLNPInput::uart_queue, 0),
                  "CLNP",
                  "UART driver install failed");

  // Apply UART configuration
  ESP_RETURN_ON_ERROR(uart_param_config(clnp_num, &uart_config), "CLNP", "UART param config failed");

  // Setup UART in rs485 half duplex mode
  ESP_RETURN_ON_ERROR(uart_set_mode(clnp_num, UART_MODE_RS485_HALF_DUPLEX), "CLNP", "UART set mode failed");

  ESP_RETURN_ON_ERROR(uart_enable_rx_intr(clnp_num), "CLNP", "UART enable rx intr failed");
  ESP_RETURN_ON_ERROR(uart_set_rx_timeout(clnp_num, 3), "CLNP", "UART set rx timeout failed");

  ESP_RETURN_ON_ERROR(uart_flush(clnp_num), "CLNP", "UART flush failed");

  return ESP_OK;
}

//Used internally to reset output channels, not related to CLNP protocol
esp_err_t CLNPInput::setup_internal_uart() {

  // Configure UART parameters
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB
  };

  // Apply UART configuration
  ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_1, &uart_config), "CLNP", "Internal UART param config failed");

  ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM_1, 100 * 2, 100 * 2, 0, NULL, 0), "CLNP", "Internal UART driver install failed");

  return ESP_OK;
}

#endif