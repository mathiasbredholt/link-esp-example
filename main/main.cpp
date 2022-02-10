#include <driver/gpio.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>

#include <ableton/Link.hpp>

#include "esp_wifi.h"

#define LED GPIO_NUM_2
#define PRINT_LINK_STATE false
#define USB_MIDI true

#define UART_PORT UART_NUM_1
#define TX_PIN 15
#define RX_PIN 12

#define LINK_TICK_PERIOD 100

void IRAM_ATTR timer_group0_isr(void *userParam) {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

  vTaskNotifyGiveFromISR(userParam, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void printTask(void *userParam) {
  while (true) {
    auto link = static_cast<ableton::Link *>(userParam);
    const auto quantum = 4.0;
    const auto sessionState = link->captureAppSessionState();
    const auto numPeers = link->numPeers();
    const auto time = link->clock().micros();
    const auto beats = sessionState.beatAtTime(time, quantum);
    std::cout << std::defaultfloat << "| peers: " << numPeers << " | "
              << "tempo: " << sessionState.tempo() << " | " << std::fixed
              << "beats: " << beats << " |" << std::endl;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void timerGroup0Init(int timerPeriodUS, void *userParam) {
  timer_config_t config = {.alarm_en = TIMER_ALARM_EN,
                           .counter_en = TIMER_PAUSE,
                           .intr_type = TIMER_INTR_LEVEL,
                           .counter_dir = TIMER_COUNT_UP,
                           .auto_reload = TIMER_AUTORELOAD_EN,
                           .divider = 80};

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timerPeriodUS);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_group0_isr, userParam, 0,
                     nullptr);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

void initUartPort(uart_port_t port, int txPin, int rxPin) {
  uart_config_t uart_config = {
      .baud_rate = 31250,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };

  if (USB_MIDI) {
    uart_config.baud_rate = 115200;
  }

  uart_param_config(port, &uart_config);
  uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(port, 512, 0, 0, NULL, 0);
}

void tickTask(void *userParam) {
  ableton::Link link(120.0f);
  link.enable(true);

  initUartPort(UART_PORT, TX_PIN, RX_PIN);

  timerGroup0Init(LINK_TICK_PERIOD, xTaskGetCurrentTaskHandle());

  if (PRINT_LINK_STATE) {
    xTaskCreate(printTask, "print", 8192, &link, 1, nullptr);
  }

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const auto state = link.captureAudioSessionState();
    const int ticks =
        std::floor(state.beatAtTime(link.clock().micros(), 1.) * 24);
    static int lastTicks;

    if (ticks > lastTicks) {
#ifdef USB_MIDI
      uint8_t data[4] = {0x0f, 0xf8, 0x00, 0x00};
      uart_write_bytes(UART_PORT, (char *)data, 4);
#else
      uint8_t data[1] = {0xf8};
      uart_write_bytes(UART_PORT, (char *)data, 1);
#endif
    }

    lastTicks = ticks;
  }
}

extern "C" void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  xTaskCreate(tickTask, "ticks", 8192, nullptr, 10, nullptr);
}
