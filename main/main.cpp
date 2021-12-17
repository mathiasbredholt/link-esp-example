#include <ableton/Link.hpp>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <protocol_examples_common.h>

#include "UartISR.h"
#include "esp_wifi.h"

#define LED GPIO_NUM_2
#define PRINT_LINK_STATE false
#define USB_MIDI true

#define UART_PORT UART_NUM_1
#define TX_PIN 15
#define RX_PIN 12

#define BUF_SIZE 20
#define FRAME_DUR_US 500 // 20 * 500 us = 10 ms total buffer duration

static bool gBuf[BUF_SIZE] = {0};
static int gBufIdx = 0;

void IRAM_ATTR timer_group0_isr(void *userParam)
{
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

  if (gBuf[gBufIdx]) {
#ifdef USB_MIDI
      uint8_t data[4] = {0x0f, 0xf8, 0x00, 0x00};
      uartWriteBytesFromISR(UART_PORT, data, 4);
#else
      uint8_t data[1] = {0xf8};
      uartWriteBytesFromISR(UART_PORT, data, 1);
#endif
  }

  ++gBufIdx;
  if (gBufIdx >= BUF_SIZE) {
    gBufIdx = 0;

    // Notifiy tick task to fill buffer
    vTaskNotifyGiveFromISR(userParam, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

void timerGroup0Init(int timerPeriodUS, void* userParam)
{
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
  // Allocate interrupt with high priority ESP_INTR_FLAG_LEVEL3
  timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_group0_isr, userParam,
    ESP_INTR_FLAG_LEVEL3, nullptr);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

void printTask(void *userParam)
{
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

void initUartPort(uart_port_t port, int txPin, int rxPin)
{
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

void tickTask(void *userParam)
{
  ableton::Link link(120.0f);
  link.enable(true);

  initUartPort(UART_PORT, TX_PIN, RX_PIN);

  if (PRINT_LINK_STATE) {
    xTaskCreate(printTask, "print", 8192, &link, 1, nullptr);
  }

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const auto state = link.captureAudioSessionState();
    static int lastTick;
    for (int i = 0; i < BUF_SIZE; ++i)
    {
      const int tick = state.beatAtTime(
        link.clock().micros() +
        std::chrono::microseconds(i * FRAME_DUR_US), 1.) * 24.;
      gBuf[i] = tick != lastTick;
      lastTick = tick;
    }
  }
}

extern "C" void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  esp_wifi_set_ps(WIFI_PS_NONE);

  TaskHandle_t tickTaskHandle;
  xTaskCreate(tickTask, "tick", 8192, nullptr, 30, &tickTaskHandle);

  timerGroup0Init(FRAME_DUR_US, tickTaskHandle);
  vTaskDelay(portMAX_DELAY);
}
