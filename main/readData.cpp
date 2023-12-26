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
  // NFC task, queue, sem
  /*nfc_task_queue = xQueueCreate(1, sizeof(nfc_task_action_t));
  nfc_task_sem = xSemaphoreCreateCounting( 10, 0 );
  nfc_cont_sem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(nfc_task, "NFC", 4096, NULL, GENERIC_TASK_PRIO, NULL, tskNO_AFFINITY);*/

  for (int i = 3; i >= 0; i--) {
      printf("Starting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  ESP_LOGI(TAG, "pn5180 example using ISO15493 NFC tags");
  //ESP_LOGI(TAG, "%s",typeid(nfc).name());
  nfc.begin(); // Initialize SPI communication, set up ISO15693, activate RF

  ESP_LOGI(TAG, "Starting read cycle...");
  vTaskDelay(pdMS_TO_TICKS(1000));

  while(1){
    // Inventory from NFC tag
    ISO15693ErrorCode_t rc = nfc.getInventory();
    if (ISO15693_EC_OK != rc) {
      nfc.printError(rc);
    }
    else{
      nfc.printUID();
      ESP_LOGI(TAG, "Manufacturer=%s", manufacturerCode[nfc.getManufacturer()]);
      vTaskDelay(pdMS_TO_TICKS(1000));

      // System information from NFC tag
      rc = nfc.getSystemInfo();
      if (ISO15693_EC_OK != rc) {
        nfc.printError(rc);
      }
      else{
        ESP_LOGI(TAG, "System info retrieved: DSFID=%d, AFI=%s, blockSize=%d, numBlocks=%d, IC Ref=%d", 
        nfc.getDsfid(), afi_string[nfc.getAfi()], nfc.getBlockSize(), nfc.getNumBlocks(), nfc.getICRef());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    if(rc == ISO15693_EC_OK){
      // Read all blocks
      ESP_LOGI(TAG, "Reading multiple blocks #0-%d", nfc.getNumBlocks()-1);
      rc = nfc.readMultipleBlock(0, nfc.getNumBlocks());
      if (ISO15693_EC_OK != rc) {
        ESP_LOGE(TAG, "Error in readMultipleBlock #0-%d:", nfc.getNumBlocks()-1);
        nfc.printError(rc);
      }
      else{
        nfc.printAllBlockData();
      }
      /*rc = nfc.readSingleBlock(0);
      if (ISO15693_EC_OK != rc) {
        ESP_LOGE(TAG, "Error in readSingleBlock #%d:", 0);
        nfc.printError(rc);
      }
      else{
        nfc.printGeneric(nfc.blockData, 0);
      }*/
#ifdef WRITE_ENABLED
      // This loop writes to the NFC tag one time per reset
      if(!flag_written){
        uint8_t dataFiller = 0;
        for (int i=0; i<nfc.numBlocks; i++) {
          for (int j=0; j<nfc.blockSize; j++) {
            nfc.blockData[(nfc.blockSize*i) + j] = dataFiller++;
          }
          ESP_LOGI(TAG,"Free malloc before: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
          rc = pn5180_writeSingleBlock(&nfc, i);
          if (ISO15693_EC_OK == rc) {
            ESP_LOGI(TAG, "Wrote block #%d", i);
          }
          else {
            ESP_LOGE(TAG, "Error in writeSingleBlock #%d: ", i);
            iso15693_printError(rc);
            break;
          }
        }
        flag_written = 1;
      }
#endif // WRITE_ENABLED
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if(ESP_OK == nfc.iso14443poll()){
      //if(nfc.mifareBlockRead(4)){
      //  ESP_LOGI(TAG, "Got mifare block");
      //  nfc.printSingleBlock(4);
      //}
      //else{
      //  ESP_LOGE(TAG, "Failed to get mifare block");
      //}
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}