#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <typeinfo>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "pn5180.hpp"

#define ESP32_HOST HSPI_HOST
#define PIN_NUM_MISO 33
#define PIN_NUM_MOSI 16
#define PIN_NUM_CLK  21
#define PIN_NUM_NSS  1
#define PIN_NUM_BUSY 5
#define PIN_NUM_RST  6

extern "C" void app_main(void);

static const char* TAG = "main.cpp";
void showIRQStatus(uint32_t irqStatus);
void printUID(const char* tag, uint8_t* uid, uint8_t len);

extern const char afi_string[14][18];
extern const char manufacturerCode[110][100];

//#define WRITE_ENABLED
pn5180 nfc(PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_NSS, PIN_NUM_BUSY, PIN_NUM_RST, ESP32_HOST);
bool flag_written = 0;

void app_main(void)
{
  for (int i = 3; i >= 0; i--) {
      printf("Starting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  ESP_LOGI(TAG, "pn5180 example polling all specs");

  nfc.begin(); // Initialize SPI communication

  ESP_LOGI(TAG, "Starting read cycle...");
  vTaskDelay(pdMS_TO_TICKS(1000));

  while(1){
        const char *data = "a slice of meatloaf";
        PN5180Error_t success;
        /*success = nfc.writeData(data);
        if(PN5180_OK == success){
            ESP_LOGI(TAG, "Wrote tag!");
        }
        else{
            ESP_LOGI(TAG, "No tags found.");
        }*/
        success = nfc.readData();
        if(PN5180_OK == success){
            ESP_LOGI(TAG, "Found tag!");
            uint8_t cardNum = nfc.getNumCard();
            nfc.printInfo(cardNum-1);
            //nfc.printSingleBlock(0, 0);
            //nfc.printMultipleBlock(0, 0, 2);
            nfc.printData(cardNum-1);
        }
        else{
            ESP_LOGI(TAG, "No tags found.");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
  }
}