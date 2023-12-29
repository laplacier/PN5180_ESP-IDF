// NAME: PN5180.cpp
//
// DESC: Implementation of PN5180 class.
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
//#define DEBUG 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <map>
#include <string>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "pn5180.hpp"

SemaphoreHandle_t nfc_task_sem;
SemaphoreHandle_t nfc_cont_sem;
QueueHandle_t nfc_task_queue;
static const char* TAG = "pn5180.cpp";

// Publicly available from https://www.kartenbezogene-identifier.de/de/chiphersteller-kennungen.html
const char manufacturerCode[110][100] = {
  "Unknown", "Motorola (UK)", "STMicroelectronics SA (FR)", "Hitachi Ltd (JP)", "NXP Semiconductors (DE)",
  "Infineon Technologies AG (DE)", "Cylink (US)", "Texas Instruments (FR)", "Fujitsu Limited (JP)", "Matsushita Electronics Corporation, Semiconductor Company (JP)",
  "NEC (JP)", "Oki Electric Industry Co Ltd (JP)", "Toshiba Corp (JP)", "Mitsubishi Electric Corp (JP)", "Samsung Electronics Co Ltd (KR)", "Hynix (KR)",
  "LG-Semiconductors Co Ltd (KR)", "Emosyn-EM Microelectronics (US)", "INSIDE Technology (FR)", "ORGA Kartensysteme GmbH (DE)", "Sharp Corporation (JP)",
  "ATMEL (FR)", "EM Microelectronic-Marin (CH)", "SMARTRAC TECHNOLOGY GmbH (DE)", "ZMD AG (DE)","XICOR Inc (US)",
  "Sony Corporation (JP)","Malaysia Microelectronic Solutions Sdn Bhd (MY)","Emosyn (US)","Shanghai Fudan Microelectronics Co Ltd (CN)","Magellan Technology Pty Limited (AU)",
  "Melexis NV BO (CH)","Renesas Technology Corp (JP)","TAGSYS (FR)","Transcore (US)","Shanghai Belling Corp Ltd (CN)",
  "Masktech Germany GmbH (DE)","Innovision Research and Technology Plc (UK)","Hitachi ULSI Systems Co Ltd (JP)","Yubico AB (SE)","Ricoh (JP)",
  "ASK (FR)","Unicore Microsystems LLC (RU)","Dallas semiconductor/Maxim (US)","Impinj Inc (US)","RightPlug Alliance (US)",
  "Broadcom Corporation (US)","MStar Semiconductor Inc (TW)","BeeDar Technology Inc (US)","RFIDsec (DK)","Schweizer Electronic AG (DE)",
  "AMIC Technology Corp (TW)","Mikron JSC (RU)","Fraunhofer Institute for Photonic Microsystems (DE)","IDS Microship AG (CH)","Kovio (US)",
  "HMT Microelectronic Ltd (CH)","Silicon Craft Technology (TH)","Advanced Film Device Inc. (JP)","Nitecrest Ltd (UK)","Verayo Inc. (US)",
  "HID Global (US)","Productivity Engineering Gmbh (DE)","Austriamicrosystems AG (reserved) (AT)","Gemalto SA (FR)","Renesas Electronics Corporation (JP)",
  "3Alogics Inc (KR)","Top TroniQ Asia Limited (Hong Kong)","GenTag Inc (USA)","Invengo Information Technology Co. Ltd (CN)","Guangzhou Sysur Microelectronics, Inc (CN)",
  "CEITEC S.A. (BR)","Shanghai Quanray Electronics Co. Ltd. (CN)","MediaTek Inc (TW)","Angstrem PJSC (RU)","Celisic Semiconductor (Hong Kong) Limited (CN)",
  "LEGIC Identsystems AG (CH)","Balluff GmbH (DE)","Oberthur Technologies (FR)","Silterra Malaysia Sdn. Bhd. (MY)","DELTA Danish Electronics, Light & Acoustics (DK)",
  "Giesecke & Devrient GmbH (DE)","Shenzhen China Vision Microelectronics Co., Ltd. (CN)","Shanghai Feiju Microelectronics Co. Ltd. (CN)","Intel Corporation (US)","Microsensys GmbH (DE)",
  "Sonix Technology Co., Ltd. (TW)","Qualcomm Technologies Inc (US)","Realtek Semiconductor Corp (TW)","Freevision Technologies Co. Ltd (CN)","Giantec Semiconductor Inc. (CN)",
  "JSC Angstrem-T (RU)","STARCHIP France","SPIRTECH (FR)","GANTNER Electronic GmbH (AT)","Nordic Semiconductor (NO)",
  "Verisiti Inc (US)","Wearlinks Technology Inc. (CN)","Userstar Information Systems Co., Ltd (TW)","Pragmatic Printing Ltd. (UK)","Associacao do Laboratorio de Sistemas Integraveis Tecnologico - LSI-TEC (BR)",
  "Tendyron Corporation (CN)","MUTO Smart Co., Ltd.(KR)","ON Semiconductor (US)","TÜBİTAK BİLGEM (TR)","Huada Semiconductor Co., Ltd (CN)",
  "SEVENEY (FR)","ISSM (FR)","Wisesec Ltd (IL)","Holtek (TW)"
};

const char afi_string[14][18] = {
  "All families","Transport","Financial",
  "Identification","Telecommunication","Medical",
  "Multimedia","Gaming","Data storage",
  "Item management","Express parcels","Postal services",
  "Airline bags","Unknown"
};

// https://www.nxp.com/docs/en/application-note/AN10833.pdf page 11/18
typedef struct {
  uint16_t numBlocks;
  uint16_t blockSize;
  uint16_t startBlock;
  uint16_t endBlock;
} MifareType_t;

/*
 * SAK = 0x08
 * https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf
 */
const MifareType_t MifareClassic1K = {
  .numBlocks = 64,
  .blockSize = 16,
  .startBlock = 4,
  .endBlock = 63
};

/*
 * SAK = 0x18
 * https://www.nxp.com/docs/en/data-sheet/MF1S70YYX_V1.pdf
 */
const MifareType_t MifareClassic4K = {
  .numBlocks = 256,
  .blockSize = 16,
  .startBlock = 4,
  .endBlock = 255
};

/*
 * SAK = 0x00
 * https://www.nxp.com/docs/en/data-sheet/MF0ULX1.pdf
 * 80 + 164 byte variants, just default to smaller to be safe
 */
const MifareType_t MifareUltralight = {
  .numBlocks = 20,
  .blockSize = 4,
  .startBlock = 4,
  .endBlock = 15
};

std::map<uint8_t, MifareType_t> mifare = {
  { 0x00, MifareUltralight },
  { 0x08, MifareClassic1K },
  { 0x18, MifareClassic4K },
};

static const uint8_t mifare_info[] = {

};
pn5180::pn5180(uint8_t MISOpin, uint8_t MOSIpin, uint8_t CLKpin, uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, spi_host_device_t host) :
  PN5180_MISO((gpio_num_t)MISOpin),
  PN5180_MOSI((gpio_num_t)MOSIpin),
  PN5180_CLK((gpio_num_t)CLKpin),
  PN5180_NSS((gpio_num_t)SSpin),
  PN5180_BUSY((gpio_num_t)BUSYpin),
  PN5180_RST((gpio_num_t)RSTpin),
  spi_host(host)
{
  /*
   * 11.4.1 Physical Host Interface
   * The interface of the PN5180 to a host microcontroller is based on a SPI interface,
   * extended by signal line BUSY. The maximum SPI speed is 7 Mbps and fixed to CPOL
   * = 0 and CPHA = 0.
   */
  // Settings for PN5180: 7Mbps, MSB first, SPI_MODE0 (CPOL=0, CPHA=0)
  // Configure physical bus settings for pn5180
  pn5180_buscfg={
      .mosi_io_num = (gpio_num_t)MOSIpin,
      .miso_io_num = (gpio_num_t)MISOpin,
      .sclk_io_num = (gpio_num_t)CLKpin,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .data4_io_num = -1,
      .data5_io_num = -1,
      .data6_io_num = -1,
      .data7_io_num = -1,
      /*
       * The number of bytes within the ‘Tx Data’ field must be in the range from 1 to 260
       * The number of bytes within the ‘Rx Data’ field range from 0 to 508
       * The host controls the number of bytes to be read via the SPI interface
       */
      .max_transfer_sz = 508,
  };

  // Configure software settings for pn5180
  pn5180_devcfg={
      .mode = 0,
      .clock_speed_hz = 1000000,
      .spics_io_num = (gpio_num_t)SSpin,
      .queue_size = 7,
      .pre_cb = NULL,
      .post_cb = NULL,
  };
}

uint8_t pn5180::getNumCard(void) const { return numCard; }
uint8_t pn5180::getManufacturer(uint8_t cardNum){ 
  if(cardNum > numCard) return 0;
  return card.manufacturer[cardNum]; 
}
uint8_t pn5180::getType(uint8_t cardNum) { 
  if(cardNum > numCard) return 0;
  return card.type[cardNum]; 
}
uint8_t pn5180::getDsfid(uint8_t cardNum) { 
  if(cardNum > numCard) return 0;
  return card.dsfid[cardNum];
}
uint8_t pn5180::getAfi(uint8_t cardNum) { 
  if(cardNum > numCard) return 0;
  return card.afi[cardNum];
}
uint8_t pn5180::getICRef(uint8_t cardNum) { 
  if(cardNum > numCard) return 0;
  return card.ic_ref[cardNum]; 
}
uint8_t pn5180::getBlockSize(uint8_t cardNum) { 
  if(cardNum > numCard) return 0;
  return card.blockSize[cardNum]; 
}
uint8_t pn5180::getNumBlocks(uint8_t cardNum) { 
  if(cardNum > numCard) return 0;
  return card.numBlocks[cardNum]; 
}
//uint8_t pn5180::getUID(void) const { return uid; }

///////////////
// Functions //
///////////////
void pn5180::begin(void){
  gpio_config_t io_conf = {};

  // NSS and Reset are outputs
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= (1ULL << PN5180_NSS);
  io_conf.pin_bit_mask |= (1ULL << PN5180_RST);
  gpio_config(&io_conf);

  // BUSY is input
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= (1ULL << PN5180_BUSY);
  gpio_config(&io_conf);

  gpio_set_level(PN5180_NSS, 1); // Deselect device
  gpio_set_level(PN5180_RST, 1); // Prevent reset

  // Apply settings
  ESP_LOGI(TAG, "init: Initializing bus SPI...");
  if(ESP_ERR_INVALID_STATE == spi_bus_initialize(spi_host, &pn5180_buscfg, SPI3_HOST)){
    ESP_LOGD(TAG, "SPI host %d already configured and in use, continuing...", spi_host);
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_add_device(spi_host, &pn5180_devcfg, &dev));
  ESP_LOGD(TAG, "init: Bus SPI Initialized");

  // Reset device
  reset();

  // PN5180 Product version
  readEEprom(PN5180_PRODUCT_VERSION, product, 2);
  if(0xff == product[1]){
    ESP_LOGE(TAG, "Initialization failed. Reset to restart.");
    while(1) vTaskDelay(portMAX_DELAY);
  }
  ESP_LOGD(TAG,"Product version: %d.%d",product[1],product[0]);

  
  // PN5180 Firmware version
  readEEprom(PN5180_FIRMWARE_VERSION, firmware, 2);
  ESP_LOGD(TAG,"Firmware version: %d.%d",firmware[1],firmware[0]);

  // PN5180 EEPROM version
  readEEprom(PN5180_EEPROM_VERSION, eeprom, 2);
  ESP_LOGD(TAG,"EEPROM version: %d.%d",eeprom[1],eeprom[0]);
}

void pn5180::end(void){
}

esp_err_t pn5180::reset(void) {
  uint32_t retries = 10;
  gpio_set_level(PN5180_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(PN5180_RST, 1);
  while((PN5180_IDLE_IRQ_STAT & getIRQStatus()) == 0 && retries > 0){
    vTaskDelay(pdMS_TO_TICKS(10));
    retries--;
  }
  if((PN5180_IDLE_IRQ_STAT & getIRQStatus()) == 0){
    ESP_LOGE(TAG, "reset: Timeout waiting for IRQ state IDLE");
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

/*
 * WRITE_REGISTER - 0x00
 */
esp_err_t pn5180::writeRegister(uint8_t reg, uint32_t value) {
  uint8_t *p = (uint8_t*)&value;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER_OR_MASK - 0x01
 */
esp_err_t pn5180::writeRegisterOrMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER_AND_MASK - 0x02
 */
esp_err_t pn5180::writeRegisterAndMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER MULTIPLE - 0x03
 */
esp_err_t pn5180::writeRegisterMultiple(uint8_t *reg, uint8_t *action, uint8_t len, uint32_t *value) {
  if(len > 42){
    ESP_LOGE(TAG, "writeRegisterMultiple: Max register writes limited to 42");
    return ESP_ERR_INVALID_SIZE;
  }
  if(!len){
    ESP_LOGE(TAG, "writeRegisterMultiple: Invalid length");
    return ESP_ERR_INVALID_SIZE;
  }
  
  uint8_t* cmd = (uint8_t*)heap_caps_malloc((len*6)+1, MALLOC_CAP_8BIT);
  if(cmd == NULL){
    ESP_LOGE(TAG, "writeRegisterMultiple: Failed malloc for cmd");
    return ESP_ERR_NO_MEM;
  }
  cmd[0] = PN5180_WRITE_REGISTER_MULTIPLE;
  /*
   * Command Structure:
   * 0x03, Write1, Write2, Write3, ....
   * Write struct: {Register, AND/OR/WRITE, Value}
   * Value written LSBFIRST - value[0], value[1], value[2], value[3] 
   */
  for(int i=0; i<len; i++){
    uint8_t *p = (uint8_t*)&value[i];
    uint8_t pos = i*6;
    cmd[pos+1] = reg[i];
    cmd[pos+2] = action[i];
    cmd[pos+3] = p[0];
    cmd[pos+4] = p[1];
    cmd[pos+5] = p[2];
    cmd[pos+6] = p[3];
  }
  esp_err_t ret = transceiveCommand(cmd, (len*6)+1, 0, 0);
  heap_caps_free(cmd);

  return ret;
}

/*
 * READ_REGISTER - 0x04
 */
esp_err_t pn5180::readRegister(uint8_t reg, uint32_t *value) {
  uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };
  return transceiveCommand(cmd, 2, (uint8_t*)value, 4);
}

/*
 * READ_REGISTER_MULTIPLE - 0x05
 */
esp_err_t pn5180::readRegisterMultiple(uint8_t *reg, uint8_t len, uint32_t *value) {
  if(len > 18){
    ESP_LOGE(TAG, "readRegisterMultiple: Max register reads limited to 18");
    return ESP_ERR_INVALID_SIZE;
  }
  if(!len){
    ESP_LOGE(TAG, "readRegisterMultiple: Invalid length");
    return ESP_ERR_INVALID_SIZE;
  }
  
  uint8_t* cmd = (uint8_t*)heap_caps_malloc(len+1, MALLOC_CAP_8BIT);
  if(cmd == NULL){
    ESP_LOGE(TAG, "readRegisterMultiple: Failed malloc for cmd");
    return ESP_ERR_NO_MEM;
  }
  cmd[0] = PN5180_READ_REGISTER_MULTIPLE;
  memcpy(cmd+1, reg, len);
  esp_err_t ret = transceiveCommand(cmd, len+1, (uint8_t*)value, len*4);
  heap_caps_free(cmd);

  return ret;
}

/*
 * WRITE_EEPROM - 0x06
 */
esp_err_t pn5180::writeEEprom(uint8_t addr, uint8_t *buffer, uint16_t len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    ESP_LOGE(TAG, "writeEEprom: Size of tx buffer cannot exceed 253 bytes");
    return ESP_ERR_INVALID_SIZE;
  }
	uint8_t cmd[len + 2];
	cmd[0] = PN5180_WRITE_EEPROM;
	cmd[1] = addr;
	for (int i = 0; i < len; i++) cmd[2 + i] = buffer[i];
	return transceiveCommand(cmd, len + 2, 0, 0);
}

/*
 * READ_EEPROM - 0x07
 */
esp_err_t pn5180::readEEprom(uint8_t addr, uint8_t *buffer, uint16_t len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    ESP_LOGE(TAG, "readEEprom: Size of rx buffer exceeds 253 bytes");
    return ESP_ERR_INVALID_SIZE;
  }
  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, (uint8_t)(len) };
  return transceiveCommand(cmd, 3, buffer, len);
}

uint32_t pn5180::getIRQStatus() {
  uint32_t irqStatus;
  readRegister(PN5180_IRQ_STATUS, &irqStatus);
  return irqStatus;
}

esp_err_t pn5180::clearIRQStatus(uint32_t irqMask) {
  return writeRegister(PN5180_IRQ_CLEAR, irqMask);
}

/*
 * LOAD_RF_CONFIG - 0x11
 */
esp_err_t pn5180::loadRFConfig(uint8_t txConf, uint8_t rxConf) {
  uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };
  return transceiveCommand(cmd, 3, 0, 0);
}

/*
 * RF_ON - 0x16
 */
esp_err_t pn5180::setRF_on() {
  uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };
  transceiveCommand(cmd, 2, 0, 0);

  uint8_t retries = 50;
  while (0 == (PN5180_TX_RFON_IRQ_STAT & getIRQStatus()) && retries > 0) {   // wait for RF field to set up (max 500ms)
    vTaskDelay(pdMS_TO_TICKS(10));
	  retries--;
  }
  ESP_LOGD(TAG, "setRF_on: IRQ State after set - %ld", getIRQStatus());
  if(0 == (PN5180_TX_RFON_IRQ_STAT & getIRQStatus())){
    ESP_LOGE(TAG, "setRF_on: Failed to detect IRQ state TX_RFON");
    return ESP_FAIL;
  }

  clearIRQStatus(PN5180_TX_RFON_IRQ_STAT);
  return ESP_OK;
}

/*
 * RF_OFF - 0x17
 */
esp_err_t pn5180::setRF_off() {

  uint8_t cmd[2] = { PN5180_RF_OFF, 0x00 };
  printIRQStatus(TAG, getIRQStatus());
  transceiveCommand(cmd, 2, 0, 0);

  uint8_t retries = 50;
  while (0 == (PN5180_TX_RFOFF_IRQ_STAT & getIRQStatus()) && retries > 0) {   // wait for RF field to set up (max 500ms)
    vTaskDelay(pdMS_TO_TICKS(10));
	  retries--;
  }
  if(0 == (PN5180_TX_RFOFF_IRQ_STAT & getIRQStatus())) {
    ESP_LOGE(TAG, "setRF_off: Failed to turn off RF, was it already off?");
    //printIRQStatus(TAG, getIRQStatus());
    return ESP_FAIL;
  }

  clearIRQStatus(PN5180_TX_RFOFF_IRQ_STAT);
  return ESP_OK;
};

esp_err_t pn5180::setupRF(uint8_t protocol) {
  esp_err_t ret;
  switch(protocol){
    case 1:
      ESP_LOGD(TAG, "setupRF: Loading RF-Configuration for ISO14443...");
      ret = loadRFConfig(0x00, 0x80); // PI-106, PI-106
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setupRF: Error loading ISO14443 RF Configuration");
        return ret;
      }
      break;
    case 2:
      ESP_LOGD(TAG, "setupRF: Loading RF-Configuration for ISO15693...");
      ret = loadRFConfig(0x0D, 0x8D); // ASK100
      if(ret != ESP_OK){
        ESP_LOGE(TAG, "setupRF: Error loading ISO15693 RF-Configuration");
        return ret;
      }
      break;
    case 3:
      ESP_LOGD(TAG, "setupRF: Loading RF-Configuration for ISO18003...");
      ret = loadRFConfig(0x0F, 0x8F); // Tari 18.88, Manchester 424_4
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setupRF: Error loading ISO18003 RF Configuration");
        return ret;
      }
      break;
    default:
      ESP_LOGE(TAG, "setupRF: Unknown protocol passed to function");
      break;
  }
  ESP_LOGD(TAG, "setupRF: Turning ON RF field...");
  ret = setRF_on();
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "setupRF: Error turning on RF");
    return ret;
  }
  return ESP_OK;
}

/*
 * SEND_DATA - 0x09
 */
esp_err_t pn5180::sendData(uint8_t *data, uint16_t len, uint8_t validBits) {
  if (len > 260){
    ESP_LOGW(TAG, "sendData: Length of data exceeds 260 bytes");
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t buffer[len+2];
  buffer[0] = PN5180_SEND_DATA;
  buffer[1] = validBits; // number of valid bits of last byte are transmitted (0 = all bits are transmitted)
  for (int i=0; i<len; i++) {
    buffer[2+i] = data[i];
  }

  uint8_t regs[2] = {PN5180_SYSTEM_CONFIG, PN5180_SYSTEM_CONFIG};
  uint8_t cmd[2] = {0x03, 0x02};
  uint32_t value[2] = {0xFFFFFFF8, 0x00000003};
  esp_err_t ret = writeRegisterMultiple(regs,cmd,2,value);
  if(ESP_OK != ret){
    ESP_LOGW(TAG,"sendData: Error writing data to multiple registers");
    return ESP_ERR_INVALID_STATE;
  }

  PN5180TransceiveState_t state = getTransceiveState();
  ESP_LOGD(TAG,"sendData: state=%d",(uint8_t)(state));
  if (PN5180_TS_WaitTransmit != state){
    ESP_LOGW(TAG, "sendData: TransceiveState not WaitTransmit");
    return ESP_ERR_INVALID_STATE;
  }

  return transceiveCommand(buffer, len+2, 0, 0);
}

PN5180Error_t pn5180::sendData(const char *data) {
  uint8_t len = strlen(data);
  if(!len) return PN5180_ERR_INVALID_PARAM;
  if(len > card.blockSize[numCard]) len = card.blockSize[numCard];

  uint8_t buffer[card.blockSize[numCard]+2] = {0};
  buffer[0] = PN5180_SEND_DATA;
  buffer[1] = 0; // number of valid bits of last byte are transmitted (0 = all bits are transmitted)
  memcpy(buffer+2, data, len);

  uint8_t regs[2] = {PN5180_SYSTEM_CONFIG, PN5180_SYSTEM_CONFIG};
  uint8_t cmd[2] = {0x03, 0x02};
  uint32_t value[2] = {0xFFFFFFF8, 0x00000003};
  esp_err_t ret =  writeRegisterMultiple(regs,cmd,2,value);
  if(ESP_OK != ret){
    ESP_LOGW(TAG, "sendData: Unable to write multiple registers");
    return PN5180_ERR_REGISTER;
  }

  PN5180TransceiveState_t state = getTransceiveState();
  ESP_LOGD(TAG,"sendData: state=%d",(uint8_t)(state));
  if (PN5180_TS_WaitTransmit != state){
    ESP_LOGW(TAG, "sendData: TransceiveState not WaitTransmit");
    return PN5180_ERR_TIMEOUT;
  }
  ret = transceiveCommand(buffer, card.blockSize[numCard]+2, 0, 0);
  if(ESP_OK != ret){
    ESP_LOGW(TAG, "sendData: SPI transaction failed");
    return PN5180_ERR_UNKNOWN;
  }

  return PN5180_OK;
}

/*
 * READ_DATA - 0x0A
 */
uint8_t* pn5180::readData(int len) {
  if (len > 508) return 0L;

  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };
  transceiveCommand(cmd, 2, readBuffer, len);

  return readBuffer;
}

/*
 * Get TRANSCEIVE_STATE from RF_STATUS register
 */

PN5180TransceiveState_t pn5180::getTransceiveState() {
  uint32_t rfStatus;
  esp_err_t ret;
  ret = readRegister(PN5180_RF_STATUS, &rfStatus);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Error reading RF_STATUS register");
    return PN5180_TS_Idle;
  }

  /*
   * TRANSCEIVE_STATEs:
   *  0 - idle
   *  1 - wait transmit
   *  2 - transmitting
   *  3 - wait receive
   *  4 - wait for data
   *  5 - receiving
   *  6 - loopback
   *  7 - reserved
   */
  ESP_LOGD(TAG, "getTransceiveState: rfStatus=%ld", rfStatus);
  PN5180TransceiveState_t state = (PN5180TransceiveState_t)((rfStatus >> 24) & 0x07);
  return state;
}

/* prepare LPCD registers */
esp_err_t pn5180::prepareLPCD() {
  //=======================================LPCD CONFIG================================================================================

  uint8_t data[255];
  uint8_t response[256];
    //1. Set Fieldon time                                           LPCD_FIELD_ON_TIME (0x36)
  uint8_t fieldOn = 0xF0;//0x## -> ##(base 10) x 8μs + 62 μs
  data[0] = fieldOn;
  writeEEprom(0x36, data, 1);
  readEEprom(0x36, response, 1);
  fieldOn = response[0];

    //2. Set threshold level                                         AGC_LPCD_THRESHOLD @ EEPROM 0x37
  uint8_t threshold = 0x03;
  data[0] = threshold;
  writeEEprom(0x37, data, 1);
  readEEprom(0x37, response, 1);
  threshold = response[0];

  //3. Select LPCD mode                                               LPCD_REFVAL_GPO_CONTROL (0x38)
  uint8_t lpcdMode = 0x01; // 1 = LPCD SELF CALIBRATION 
                           // 0 = LPCD AUTO CALIBRATION (this mode does not work, should look more into it, no reason why it shouldn't work)
  data[0] = lpcdMode;
  writeEEprom(0x38, data, 1);
  readEEprom(0x38, response, 1);
  lpcdMode = response[0];
  
  // LPCD_GPO_TOGGLE_BEFORE_FIELD_ON (0x39)
  uint8_t beforeFieldOn = 0xF0; 
  data[0] = beforeFieldOn;
  writeEEprom(0x39, data, 1);
  readEEprom(0x39, response, 1);
  beforeFieldOn = response[0];
  
  // LPCD_GPO_TOGGLE_AFTER_FIELD_ON (0x3A)
  uint8_t afterFieldOn = 0xF0; 
  data[0] = afterFieldOn;
  writeEEprom(0x3A, data, 1);
  readEEprom(0x3A, response, 1);
  afterFieldOn = response[0];
  vTaskDelay(pdMS_TO_TICKS(100));
  return ESP_OK;
}

/* switch the mode to LPCD (low power card detection)
 * Parameter 'wakeupCounterInMs' must be in the range from 0x0 - 0xA82
 * max. wake-up time is 2960 ms.
 */
esp_err_t pn5180::switchToLPCD(uint16_t wakeupCounterInMs) {
  // clear all IRQ flags
  clearIRQStatus(0xffffffff); 
  // enable only LPCD and general error IRQ
  writeRegister(PN5180_IRQ_ENABLE, PN5180_LPCD_IRQ_STAT | PN5180_GENERAL_ERROR_IRQ_STAT);  
  // switch mode to LPCD 
  uint8_t cmd[4] = { PN5180_SWITCH_MODE, 0x01, (uint8_t)(wakeupCounterInMs & 0xFF), (uint8_t)((wakeupCounterInMs >> 8U) & 0xFF) };
  return transceiveCommand(cmd, sizeof(cmd), 0, 0);
}

void pn5180::printIRQStatus(const char* tag, uint32_t irqStatus) {
  char states[255] = "";
  if (irqStatus & (1<< 0)) strcat(states,"RX ");
  if (irqStatus & (1<< 1)) strcat(states,"TX ");
  if (irqStatus & (1<< 2)) strcat(states,"IDLE ");
  if (irqStatus & (1<< 3)) strcat(states,"MODE_DETECTED ");
  if (irqStatus & (1<< 4)) strcat(states,"CARD_ACTIVATED ");
  if (irqStatus & (1<< 5)) strcat(states,"STATE_CHANGE ");
  if (irqStatus & (1<< 6)) strcat(states,"RFOFF_DET ");
  if (irqStatus & (1<< 7)) strcat(states,"RFON_DET ");
  if (irqStatus & (1<< 8)) strcat(states,"TX_RFOFF ");
  if (irqStatus & (1<< 9)) strcat(states,"TX_RFON ");
  if (irqStatus & (1<<10)) strcat(states,"RF_ACTIVE_ERROR ");
  if (irqStatus & (1<<11)) strcat(states,"TIMER0 ");
  if (irqStatus & (1<<12)) strcat(states,"TIMER1 ");
  if (irqStatus & (1<<13)) strcat(states,"TIMER2 ");
  if (irqStatus & (1<<14)) strcat(states,"RX_SOF_DET ");
  if (irqStatus & (1<<15)) strcat(states,"RX_SC_DET ");
  if (irqStatus & (1<<16)) strcat(states,"TEMPSENS_ERROR ");
  if (irqStatus & (1<<17)) strcat(states,"GENERAL_ERROR ");
  if (irqStatus & (1<<18)) strcat(states,"HV_ERROR ");
  if (irqStatus & (1<<19)) strcat(states,"LPCD ");
  ESP_LOGI(tag,"IRQ_Status: %s",states);
}

void pn5180::printRfStatus(const char* tag, uint32_t rfStatus) {
  char states[255] = "";
  if (rfStatus & (1<<10)) strcat(states,"RX_ACTIVE ");
  if (rfStatus & (1<<11)) strcat(states,"TX_ACTIVE ");
  if (rfStatus & (1<<12)) strcat(states,"RX_ENABLE ");
  if (rfStatus & (1<<16)) strcat(states,"RF_DET ");
  if (rfStatus & (1<<17)) strcat(states,"TX_RF_STATUS ");
  if (rfStatus & (1<<18)) strcat(states,"CRC_OK ");
  if (rfStatus & (1<<19)) strcat(states,"DPLL_ENABLE ");
  char buf[255];
  sprintf(buf, "TRANSCEIVE_STATE=%d", (uint8_t)((rfStatus >> 24) & 0x07));
  strcat(states, buf);
  ESP_LOGI(tag,"RF_Status: %s",states);
}

ISO15693ErrorCode_t pn5180::iso15693Poll(){
  numCard = 0;
  uint8_t numCol = 0;
  uint32_t collision[16] = {0};
  uint8_t maskLen = 0;

  do{
    if(numCol > 0){
      uint32_t mask = collision[0];
      do{
        mask >>= 4L;
        maskLen++;
      }while(mask > 0);
    } 
    uint8_t *readBuffer;
    uint8_t *p = (uint8_t*)&(collision[0]);
    //                      Flags,  CMD,
    uint8_t inventory[7] = { 0x06, 0x01, (uint8_t)(maskLen*4), p[0], p[1], p[2], p[3] };
    //                         |\- inventory flag + high data rate
    //                         \-- 16 slots: upto 16 cards, no AFI field present
    uint8_t cmdLen = 3 + (maskLen / 2) + (maskLen % 2);
    ESP_LOGD(TAG, "mask=%ld, maskLen=%d, cmdLen=%d", collision[0], maskLen, cmdLen);
    setupRF(ISO15693);               // 1. 2. Load ISO15693 config, RF on
    clearIRQStatus(0x000FFFFF);      // 3. Clear all IRQ_STATUS flags
    sendData(inventory, cmdLen, 0);  // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command

    for(int slot=0; slot<16; slot++){                                   // 7. Loop to check 16 time slots for data
      vTaskDelay(pdMS_TO_TICKS(15));
      uint32_t regStatus[2];
      uint8_t readRegs[2] = {PN5180_RX_STATUS, PN5180_IRQ_STATUS};
      readRegisterMultiple(readRegs, 2, regStatus);
      uint16_t len = (uint16_t)(regStatus[0] & 0x000001ff);
      if((regStatus[0] >> 18) & 0x01 && numCol < 16){                      // 7+ Determine if a collision occurred
        if(maskLen > 0) collision[numCol] = collision[0] | (slot << (maskLen * 2));
        else collision[numCol] = slot << (maskLen * 2);                // Yes, store position of collision
        ESP_LOGI(TAG, "Collision detected for UIDs matching %lX starting at LSB", collision[numCol]);
        numCol = numCol + 1;
      }
      else if(!(regStatus[1] & PN5180_RX_IRQ_STAT) || !len){               // 8. Check if a card has responded
        ESP_LOGI(TAG, "getInventoryMultiple: No card in this time slot. State=%ld", regStatus[1]);
      }
      else{
        ESP_LOGI(TAG, "slot=%d, irqStatus: %ld, RX_STATUS: %ld, Response length=%d", slot, regStatus[1], regStatus[0], len);
        readBuffer = readData(len);                                     // 9. Read reception buffer
        if(0L == readBuffer){
          ESP_LOGE(TAG,"getInventoryMultiple: ERROR in readData!");
          return ISO15693_EC_UNKNOWN_ERROR;
        }

        // Record raw UID data                                          // 10. Record all data to Inventory struct
        for (int i=0; i<8; i++) {
          card.uid[numCard][i] = readBuffer[2+i];
        }

        ESP_LOGI(TAG,"getInventory: Response flags: 0x%X, Data Storage Format ID: 0x%X", readBuffer[0], readBuffer[1]);
        numCard++;
      }
      
      if(slot+1 < 16){ // If we have more cards to poll for...
        writeRegisterAndMask(PN5180_TX_CONFIG, 0xFFFFFB3F);  // 11. Next SEND_DATA will only include EOF
        printIRQStatus(TAG, getIRQStatus());
        clearIRQStatus(0x000FFFFF);                              // 14. Clear all IRQ_STATUS flags
        sendData(inventory, 0, 0);                               // 12. 13. 15. Idle/StopCom Command, Transceive Command, Send EOF
      }
    }
    setRF_off();                                                 // 16. Switch off RF field
    if(numCol) {
      numCol--;
      for(int i=0; i<numCol; i++){
        collision[i] = collision[i+1];
      }
    }
  }while(numCol);
  return ISO15693_EC_OK;
}

PN5180Error_t pn5180::getInventory() {
  numCard = 0;
  setupRF(ISO14443);
  PN5180Error_t state = iso14443poll(ISO14443_REQA);
  if(state){
    clearInfo(numCard);
    setRF_off();
    if(PN5180_ERR_NO_CARD != state) reset();
    setupRF(ISO15693);
    state = iso15693PollSingle(numCard);
    if(state){
      clearInfo(numCard);
      setRF_off();
      if(PN5180_ERR_NO_CARD != state) reset();
      //state = iso18003PollSingle();
      if(state){
        clearInfo(numCard);
        setRF_off();
        if(PN5180_ERR_NO_CARD != state) reset();
      }
      else{
        //numCard++;
        //state = iso18003GetSystemInfo();
      }
    }
    else{
      numCard++;
      state = iso15693GetSystemInfo(numCard);
    }
  }
  else{
    numCard++;
  }
  setRF_off();
  return state;
}

PN5180Error_t pn5180::getData(uint8_t blockNo) {
  numCard = 0;
  if(ESP_OK != setupRF(ISO14443)){
    ESP_LOGW(TAG,"getData: Failed to setupRF for ISO14443");
  }
  PN5180Error_t state = readSingleBlock(ISO14443, blockNo);
  if(PN5180_ERR_NO_CARD == state){
    if(ESP_OK != setRF_off()){
      ESP_LOGW(TAG,"getData: Failed to setRF_off for ISO14443");
    }
    if(ESP_OK != setupRF(ISO15693)){
      ESP_LOGW(TAG,"getData: Failed to setupRF for ISO15693");
    }
    state = readSingleBlock(ISO15693, blockNo);
    if(PN5180_ERR_NO_CARD == state){
      setRF_off();
      //setupRF(ISO18003);
      //state = readSingleBlock(ISO18003, 0);
    }
    else if(PN5180_OK == state){
      numCard++;
    }
    else{
      ESP_LOGW(TAG,"getData: Failed to readSingleBlock for ISO15693, %d", state);
    }
  }
  else if(PN5180_OK == state){
    numCard++;
  }
  else{
    ESP_LOGW(TAG,"getData: Failed to readSingleBlock for ISO14443");
  }
  reset();
  return state;
}

PN5180Error_t pn5180::getData(uint8_t blockNo, uint8_t numBlock) {
  //vTaskDelay(pdMS_TO_TICKS(1000));
  numCard = 0;
  setupRF(ISO14443);
  PN5180Error_t state = readSingleBlock(ISO14443, blockNo);
  if(PN5180_ERR_NO_CARD == state){
    setRF_off();
    setupRF(ISO15693);
    if(!numBlock){
      state = readSingleBlock(ISO15693, blockNo);
    }
    else{
      state = readMultipleBlock(ISO15693, blockNo, numBlock);
    }
    if(PN5180_ERR_NO_CARD == state){
      //setupRF(ISO18003);
      //state = readSingleBlock(ISO18003, 0);
    }
    else if(PN5180_OK == state){
      numCard++;
    }
  }
  else if(PN5180_OK == state){
    numCard++;
  }
  reset();
  return state;
}
PN5180Error_t pn5180::readData(void){
  numCard = 0;
  card.blocksRead[numCard] = 0;
  PN5180Error_t state = PN5180_OK;
  setupRF(ISO14443);
  state = readMultipleBlock(ISO14443, 0, 2);
  ESP_LOGI(TAG, "readData: Read %d blocks", card.blocksRead[numCard]);
  if(PN5180_ERR_NO_CARD == state){
    card.blocksRead[numCard] = 0;
    setRF_off();
    setupRF(ISO15693);
    state = readMultipleBlock(ISO15693, 0, 6);
    if(PN5180_ERR_NO_CARD == state){
      //card.blocksRead[numCard] = 0;
      //setupRF(ISO18003);
      //state = readSingleBlock(ISO18003, 0);
    }
    else if(PN5180_OK == state){
      numCard++;
    }
  }
  else if(PN5180_OK == state){
    numCard++;
  }
  reset();
  return state;
}

PN5180Error_t pn5180::writeData(const char* data) {
  numCard = 0;
  setupRF(ISO14443);
  PN5180Error_t state = iso14443poll(ISO14443_REQA);
  if(PN5180_ERR_NO_CARD == state){
    setRF_off();
    setupRF(ISO15693);
    state = iso15693PollSingle(numCard);
    if(PN5180_ERR_NO_CARD == state){
      //setupRF(ISO18003);
      //state = readSingleBlock(ISO18003, 0);
      //if(state){
      //  reset();
      //  return state;
      //}
      //else if (PN5180_OK == state){
        // do write
      //}
    }
    else if(PN5180_OK == state){
      state = iso15693GetSystemInfo(numCard);
      if(PN5180_OK == state){
        uint8_t blocksToWrite = (strlen(data) / card.blockSize[numCard]) + 1; // Blocks to write, rounded up
        for(int i=0; i<blocksToWrite; i++){
          const char *slice = data+(i*card.blockSize[numCard]);
          bool success = iso15693WriteBlock(slice, card.startBlock[numCard]+i);
          if(!success){
            ESP_LOGW(TAG, "writeData: Failed to write ISO15693 tag!");
            reset();
            return PN5180_ERR_UNKNOWN;
          }
        }
        numCard++;
      }
      else{
        ESP_LOGW(TAG,"writeData: Failed to perform iso15693 sysInfo");
      }
    }
    else{
      ESP_LOGW(TAG,"writeData: Failed to perform iso15693 poll");
    }
  }
  else if(PN5180_OK == state){
    bool success = mifareAuthenticate(card.startBlock[numCard]);
    if(!success){
      ESP_LOGW(TAG, "Failed to authenticate Mifare tag!");
      mifareHalt();
      reset();
      return PN5180_ERR_NO_RESP;
    }
    uint8_t blocksToWrite = (strlen(data) / card.blockSize[numCard]) + 1; // Blocks to write, rounded up
    for(int i=0; i<blocksToWrite; i++){
      const char *slice = data+(i*card.blockSize[numCard]);
      success = mifareWriteBlock(slice, card.startBlock[numCard]+i);
      if(!success){
        ESP_LOGW(TAG, "Failed to write Mifare tag!");
        mifareHalt();
        reset();
        return PN5180_ERR_UNKNOWN;
      }
    }
    numCard++;
  }
  else{
    ESP_LOGW(TAG,"writeData: Failed to perform iso14443 poll");
  }
  reset();
  return state;
}

PN5180Error_t pn5180::readSingleBlock(uint8_t protocol, uint8_t blockNo) {
  bool success;
  PN5180Error_t state;
  switch(protocol){
    case ISO14443:
      state = iso14443poll(ISO14443_REQA);
      if(state){
        if(PN5180_ERR_NO_CARD != state) ESP_LOGW(TAG,"readSingleBlock: Failed to perform iso14443 poll");
        clearInfo(numCard);
        return state;
      }
      if(blockNo + card.startBlock[numCard] > card.endBlock[numCard]){
        ESP_LOGW(TAG, "Chosen block not a valid block");
        return PN5180_ERR_INVALID_PARAM;
      }
      success = mifareAuthenticate(blockNo+card.startBlock[numCard]);
      if(!success){
        ESP_LOGW(TAG, "Failed to authenticate Mifare tag!");
        return PN5180_ERR_NO_RESP;
      }
      success = mifareReadBlock(numCard, blockNo+card.startBlock[numCard]);
      if(!success){
        ESP_LOGW(TAG, "Failed to read Mifare tag!");
        return PN5180_ERR_NO_RESP;
      }
      break;
    case ISO15693:
      // Find UID and info
      state = iso15693PollSingle(numCard);
      if(state){
        if(PN5180_ERR_NO_CARD != state) ESP_LOGW(TAG,"readSingleBlock: Failed to perform iso15693 poll");
        clearInfo(numCard);
        return state;
      }
      state = iso15693GetSystemInfo(numCard);
      if(state){
        if(PN5180_ERR_NO_CARD != state) ESP_LOGW(TAG,"readSingleBlock: Failed to get Iso15693 sysInfo");
        clearInfo(numCard);
        return state;
      }
      //printInfo(numCard);
      success = iso15693ReadBlock(numCard, blockNo+card.startBlock[numCard]);
      if(!success){
        ESP_LOGW(TAG, "readSingleBlock: Failed to read ISO15693 tag!");
        return PN5180_ERR_NO_RESP;
      }
      break;
    case ISO18003:
      break;
    default:
      ESP_LOGW(TAG, "readSingleBlock: Unknown protocol selected");
      return PN5180_ERR_UNKNOWN;
      break;
  }
  return PN5180_OK;
}

PN5180Error_t pn5180::readMultipleBlock(uint8_t protocol, uint8_t blockNo, uint8_t numBlock) {
  bool success;
  PN5180Error_t state;
  switch(protocol){
    case ISO14443:
      state = iso14443poll(ISO14443_REQA);
      if(state){
        clearInfo(numCard);
        return state;
      }
      if(!numBlock || (card.startBlock[numCard] + blockNo + numBlock > card.endBlock[numCard])){
        ESP_LOGW(TAG, "readMultipleBlock: Chosen blocks exceed memory of ISO14443 card");
        return PN5180_ERR_INVALID_PARAM;
      }
      success = mifareAuthenticate(blockNo+card.startBlock[numCard]);
      if(!success){
        ESP_LOGE(TAG, "readMultipleBlock: Failed to authenticate Mifare tag!");
        mifareHalt();
        return PN5180_ERR_NO_RESP;
      }
      success = mifareReadMultipleBlock(numCard, card.startBlock[numCard], 2);
      if(!success){
        ESP_LOGE(TAG, "readMultipleBlock: Failed to read Mifare tag!");
        mifareHalt();
        return PN5180_ERR_NO_RESP;
      }
      mifareHalt();
      break;
    case ISO15693:
      // Find UID and info
      state = iso15693PollSingle(numCard);
      if(state){
        clearInfo(numCard);
        return state;
      }
      state = iso15693GetSystemInfo(numCard);
      if(state){
        clearInfo(numCard);
        return state;
      }
      printInfo(numCard);
      if(!numBlock || (card.startBlock[numCard] + blockNo + numBlock > card.endBlock[numCard])){
        ESP_LOGW(TAG, "readMultipleBlock: Chosen blocks exceed memory of ISO15693 card");
        return PN5180_ERR_INVALID_PARAM;
      }
      success = iso15693ReadMultipleBlock(numCard, blockNo+card.startBlock[numCard], numBlock);
      if(!success){
        ESP_LOGE(TAG, "readMultipleBlock: Failed to read tag!");
        return PN5180_ERR_NO_RESP;
      }
      break;
    case ISO18003:
      break;
    default:
      ESP_LOGE(TAG, "readSingleBlock: Unknown protocol selected");
      return PN5180_ERR_UNKNOWN;
      break;
  }
  return PN5180_OK;
}

PN5180Error_t pn5180::getInventoryMultiple() {
  setupRF(ISO14443);
  PN5180Error_t state = iso14443poll(ISO14443_REQA);
  if(state){
    clearInfo(numCard);
    setRF_off();
    if(PN5180_ERR_NO_CARD != state) reset();
    setupRF(ISO15693);
    state = iso15693PollSingle(numCard);
    if(state){
      clearInfo(numCard);
      setRF_off();
      if(PN5180_ERR_NO_CARD != state) reset();
      //state = iso18003PollSingle();
      if(state){
        clearInfo(numCard);
        setRF_off();
        if(PN5180_ERR_NO_CARD != state) reset();
      }
      else{
        //state = iso18003GetSystemInfo();
      }
    }
    else{
      state = iso15693GetSystemInfo(numCard);
    }
  }
  reset();
  return state;
}

void pn5180::printInfo(uint8_t cardNum) {
  printUID(cardNum);
  ESP_LOGI(TAG, "MemSize=%d BlockSize=%d NumBlocks=%d", card.blockSize[cardNum] * card.numBlocks[cardNum], card.blockSize[cardNum], card.numBlocks[cardNum]);
  ESP_LOGI(TAG, "FirstBlock=%d LastBlock=%d", card.startBlock[cardNum], card.endBlock[cardNum]);
}

void pn5180::clearInfo(uint8_t cardNum) {
  memset(card.uid[cardNum], 0, sizeof(card.uid[cardNum]));
  memset(card.data[cardNum], 0, sizeof(card.data[cardNum]));
  card.type[cardNum] = card.blockSize[cardNum] = card.numBlocks[cardNum] = card.startBlock[cardNum] = card.endBlock[cardNum] = 0;
  card.manufacturer[cardNum] = card.uidLength[cardNum] = card.afi[cardNum] = card.ic_ref[cardNum] = 0;
  card.dsfid[cardNum] = card.blocksRead[cardNum] = 0;
}

/*
 * Inventory, code=01
 *
 * Request format: SOF, Req.Flags, Inventory, AFI (opt.), Mask len, Mask value, CRC16, EOF
 * Response format: SOF, Resp.Flags, DSFID, UID, CRC16, EOF
 *
 */
PN5180Error_t pn5180::iso15693PollSingle(uint8_t cardNum) {
  //                      Flags,  CMD, maskLen
  uint8_t inventory[3] = { 0x26, 0x01, 0x00 };
  //                         |\- inventory flag + high data rate
  //                         \-- 1 slot: only one card, no AFI field present
  ESP_LOGD(TAG,"getInventory: Get Inventory...");
  clearIRQStatus(0x000FFFFF);      // 3. Clear all IRQ_STATUS flags
  sendData(inventory, 3, 0);  // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(20));

  uint32_t regStatus[2];
  uint8_t readRegs[2] = {PN5180_RX_STATUS, PN5180_IRQ_STATUS};
  readRegisterMultiple(readRegs, 2, regStatus);
  uint16_t len = (uint16_t)(regStatus[0] & 0x000001ff);
  ESP_LOGD(TAG, "irqStatus: %ld, RX_STATUS: %ld, Response length=%d", regStatus[1], regStatus[0], len);

  if(!len){
    ESP_LOGD(TAG, "iso15693PollSingle: No card");
    return PN5180_ERR_NO_CARD;
  }
  uint8_t *readBuffer;
  readBuffer = readData(len);

  if(readBuffer[9] != 224){
    ESP_LOGW(TAG, "iso15693PollSingle: UID in unrecognized format! %X should be E0", readBuffer[9]);
    return PN5180_ERR_UNKNOWN; // UIDs always start with E0h (224d)
  }

  // Record raw UID data
  for (int i=0; i<8; i++) {
    card.uid[cardNum][i] = readBuffer[2+i];
  }

  /*
   * https://www.nxp.com/docs/en/data-sheet/SL2S2002_SL2S2102.pdf
   *  
   * The 64-bit unique identifier (UID) is programmed during the production process according
   * to ISO/IEC 15693-3 and cannot be changed afterwards.
   * 
   * UID: AA:BB:CCDDDDDDDDDD
   * 
   * AA - Always E0
   * BB - Manufacturer Code (0x04 = NXP Semiconductors)
   * CC - Random ID, sometimes used by manufacturer for Tag Type (0x01 = ICODE SLIX)
   * DDDDDDDDDD - Random ID
   */

  // Record Manufacturer code
  card.manufacturer[cardNum] = card.uid[cardNum][6];

  // Record IC type
  card.type[cardNum] = ISO15693;

  ESP_LOGD(TAG,"iso15693PollSingle: Response flags: 0x%X, Data Storage Format ID: 0x%X", readBuffer[0], readBuffer[1]);
  card.uidLength[cardNum] = 8;
  return PN5180_OK;
}

/*
 * Get System Information, code=2B
 *
 * Request format: SOF, Req.Flags, GetSysInfo, UID (opt.), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
  *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, InfoFlags, UID, DSFID (opt.), AFI (opt.), Other fields (opt.), CRC16, EOF
 *
 *    InfoFlags:
 *    xxxx.3210
 *         |||\_ DSFID: 0=DSFID not supported, DSFID field NOT present; 1=DSFID supported, DSFID field present
 *         ||\__ AFI: 0=AFI not supported, AFI field not present; 1=AFI supported, AFI field present
 *         |\___ VICC memory size:
 *         |        0=Information on VICC memory size is not supported. Memory size field is present. ???
 *         |        1=Information on VICC memory size is supported. Memory size field is present.
 *         \____ IC reference:
 *                  0=Information on IC reference is not supported. IC reference field is not present.
 *                  1=Information on IC reference is supported. IC reference field is not present.
 *
 *    VICC memory size:
 *      xxxb.bbbb nnnn.nnnn
 *        bbbbb - Block size is expressed in number of bytes, on 5 bits, allowing to specify up to 32 bytes i.e. 256 bits.
 *        nnnn.nnnn - Number of blocks is on 8 bits, allowing to specify up to 256 blocks.
 *
 *    IC reference: The IC reference is on 8 bits and its meaning is defined by the IC manufacturer.
 */
PN5180Error_t pn5180::iso15693GetSystemInfo(uint8_t cardNum) {
  uint8_t cmd[10] = { 0x22, 0x2B, 1,2,3,4,5,6,7,8 };  // UID has LSB first!
  for (int i=0; i<8; i++) {
    cmd[2+i] = card.uid[cardNum][i];
  }

  ESP_LOGD(TAG,"getSystemInfo: Get System Information");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd, sizeof(cmd), ESP_LOG_DEBUG);

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(cmd, 10, 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  uint32_t irqStatus = getIRQStatus();
  if (!(irqStatus & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    //printIRQStatus(TAG, irqR);
    ESP_LOGW(TAG, "getSystemInfo: No data response.");
    return PN5180_ERR_NO_CARD;
  }
  uint16_t retries = 5;
  while (!(irqStatus & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqStatus = getIRQStatus();
	  retries--;
  }
  ESP_LOGD(TAG, "getSystemInfo: RX_IRQ_STAT Retries - %d", retries);
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!(irqStatus & PN5180_RX_IRQ_STAT)){
    ESP_LOGW(TAG, "getSystemInfo: Timeout waiting for response to end. Datalen=%d", len);
    return PN5180_ERR_TIMEOUT;
  }
  if(!len){
    ESP_LOGW(TAG, "getSystemInfo: Length is 0!!");
    return PN5180_ERR_NO_RESP;
  }
  uint8_t *p = readBuffer;
  p = readData(len);

  for (int i=0; i<8; i++) {
    card.uid[cardNum][i] = p[2+i];
  }

  uint8_t *pFlags = &p[10];
  uint8_t infoFlags = p[1];
  if(infoFlags & 0x01) { // DSFID flag
    card.dsfid[cardNum] = (uint8_t)(*pFlags++);
    ESP_LOGD(TAG, "getSystemInfo: DSFID=%X", card.dsfid[cardNum]); // Data storage format identifier
  }
  else{
    card.dsfid[cardNum] = 0;
    ESP_LOGD(TAG,"getSystemInfo: No DSFID");
  }
  
  if (infoFlags & 0x02) { // AFI flag
    card.afi[cardNum] = *pFlags++;
    card.afi[cardNum] >>= 4;
  }
  else{
    card.afi[cardNum] = 0;
    ESP_LOGD(TAG,"getSystemInfo: No AFI");
  }

  if (infoFlags & 0x04) { // VICC Memory size
    card.numBlocks[cardNum] = *pFlags++;
    card.blockSize[cardNum] = *pFlags++;
    card.blockSize[cardNum] &= 0x1F;
    card.startBlock[cardNum] = 0;
    card.endBlock[cardNum] = card.numBlocks[cardNum];
    card.numBlocks[cardNum]++;
    card.blockSize[cardNum]++;
  
    ESP_LOGD(TAG, "getSystemInfo: VICC MemSize=%d BlockSize=%d NumBlocks=%d", card.blockSize[cardNum] * card.numBlocks[cardNum], card.blockSize[cardNum], card.numBlocks[cardNum]);
  }
  else{
    card.blockSize[cardNum] = 0;
    card.numBlocks[cardNum] = 0;
    ESP_LOGW(TAG, "getSystemInfo: No VICC memory size");
  }
   
  if (infoFlags & 0x08) { // IC reference
    card.ic_ref[cardNum] = (uint8_t)(*pFlags++);
    ESP_LOGD(TAG, "getSystemInfo: IC Ref=%X", card.ic_ref[cardNum]);
  }
  else{
    card.ic_ref[cardNum] = 0; 
    ESP_LOGD(TAG,"getSystemInfo: No IC ref");
  }

  return PN5180_OK;
}

/*
 * Read single block, code=20
 *
 * Request format: SOF, Req.Flags, ReadSingleBlock, UID (opt.), BlockNumber, CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, BlockData (len=blockLength), CRC16, EOF
 */
bool pn5180::iso15693ReadBlock(uint8_t cardNum, uint8_t blockNo) {
  if(ISO15693 != card.type[cardNum]){
    ESP_LOGW(TAG, "Card is not ISO15693 compatible");
    return false;
  }
  //                 flags,  cmd, uid,             blockNo
  uint8_t cmd[11] = { 0x22, 0x20, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                    |\- high data rate
  //                    \-- no options, addressed by UID
  for (int i=0; i<8; i++) {
    cmd[2+i] = card.uid[cardNum][i];
  }

  ESP_LOGD(TAG,"readSingleBlock: Read Single Block #%d, size=%d: ", blockNo, card.blockSize[cardNum]);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd, 11, ESP_LOG_DEBUG);

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(cmd, 11, 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  uint32_t irqStatus = getIRQStatus();
  if (!(irqStatus & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    //printIRQStatus(TAG, irqR);
    return false;
  }
  uint16_t retries = 10;
  while (!(irqStatus & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(5 / portTICK_PERIOD_MS);
    irqStatus = getIRQStatus();
	  retries--;
  }
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!(irqStatus & PN5180_RX_IRQ_STAT)){
    ESP_LOGW(TAG, "readSingleBlock: Timeout waiting for response to end. Datalen=%d", len);
    return false;
  }
  if(!len){
    ESP_LOGW(TAG, "readSingleBlock: Length is 0!!");
    return false;
  }

  uint8_t *p = readBuffer;
  p = readData(len);

  uint8_t startAddr = card.blocksRead[cardNum] * card.blockSize[cardNum];
  for (int i=0; i<card.blockSize[cardNum]; i++) {
    card.data[cardNum][startAddr + i] = p[1+i];
  }
  card.blocksRead[cardNum]++;

  ESP_LOGD(TAG,"readSingleBlock: Value=");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, card.data[cardNum], card.blockSize[cardNum], ESP_LOG_DEBUG);

  return true;
}

/*
 * 
 */
PN5180Error_t pn5180::iso14443poll(uint8_t atqaCmd){
	if (writeRegisterAndMask(PN5180_SYSTEM_CONFIG, 0xFFFFFFBF) != ESP_OK) {     // Turn off crypto1
		ESP_LOGW(TAG, "iso14443Poll: Failed to disable crypto");
		return PN5180_ERR_REGISTER;
	}
  if (writeRegisterAndMask(PN5180_CRC_RX_CONFIG, 0xFFFFFFFE) != ESP_OK) {     // 3. Switch the CRC extension off in Tx direction
		ESP_LOGW(TAG, "iso14443Poll: Failed to disable CRC TX");
		return PN5180_ERR_REGISTER;
	}
	if (writeRegisterAndMask(PN5180_CRC_TX_CONFIG, 0xFFFFFFFE) != ESP_OK) {     // 4. Switch the CRC extension off in Rx direction
		ESP_LOGW(TAG, "iso14443Poll: Failed to disable CRC RX");
		return PN5180_ERR_REGISTER;
	}
  clearIRQStatus(0x000FFFFF);                                                 // 5. Clear the interrupt register IRQ_STATUS

  uint8_t cmd[7] = {atqaCmd,0,0,0,0,0,0};
  sendData(cmd, 1, 7);                                                        // 6. 7. 8. Idle/StopCom Command, Transceive Command, REQA command
  
  vTaskDelay(pdMS_TO_TICKS(15));
  uint32_t irqStatus = getIRQStatus();
  if(!(irqStatus & PN5180_RX_IRQ_STAT)){                                      // 9. Wait until a Card has responded via checking the IRQ_STATUS register
    ESP_LOGD(TAG, "iso14443Poll: No card detected. State=%ld", irqStatus);
    //printIRQStatus(TAG, irqStatus);
    return PN5180_ERR_NO_CARD;
  }
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001FF);

  if(!len){ // If len=0, ATQA unsuccessful, no card
    ESP_LOGD(TAG, "iso14443Poll: No card detected. State=%ld", irqStatus);
    return PN5180_ERR_NO_CARD;
  }
  
  uint8_t* p = readBuffer;
  p = readData(2);                                                    // 10. Read the reception buffer (ATQA)

  /* Table specifies the coding of 2 byte ATQA, All RFU bits shall be set to (0)b
   * | 15 14 13 12 | 11 10 09 08 |  07 06  |  05 |     04 03 02 01 00      |
   * |     RFU     |   Private   | UID len | RFU | Bit frame anticollision |
   * 
   * UID len = 0(single UID), 1(double UID), 2(triple UID), 3(RFU)
   * 
   * However, the recommendation for MIFARE is to ignore ATQA, and only use this as
   * a means to recognize the presence of a card.
   */ 
  ESP_LOGD(TAG, "iso14443Poll: ATQA: %d:%d, len: %d", p[0], p[1], len);

  /* ANTICOLLISION CASCADE LEVEL 1 */
  uint8_t casc1[2] = {0x93, 0x20};
  memcpy(cmd, casc1, 2);
  clearIRQStatus(0xFFFFFFFF);
	sendData(cmd, 2, 0x00); // Cascade level 1 (0x93), NVB (0x20)
  vTaskDelay(pdMS_TO_TICKS(15));

  /* Ensure we detect the expected length of the response (5 bytes) */
  readRegister(PN5180_RX_STATUS, &rxStatus);
  len = (uint16_t)(rxStatus & 0x000001FF);
  if (len != 5 && len != 0) {
		ESP_LOGW(TAG, "iso14443Poll: Card present but collision detected, try again...");
    return PN5180_ERR_COLLISION;
	}
  if (!len){
    ESP_LOGW(TAG, "iso14443Poll: No length detected! Did the card move?");
    return PN5180_ERR_NO_RESP;
  }

  /* Obtain and process SAK response */
  uint8_t* pSak = sak;
	pSak = readData(len); // Get SAK
  ESP_LOGD(TAG, "len=%d, SAK=%x:%x:%x:%x:%x", len, pSak[0], pSak[1], pSak[2], pSak[3], pSak[4]);


  if (writeRegisterOrMask(PN5180_CRC_RX_CONFIG, 0x01) != ESP_OK) { // 3. Switch the CRC extension on in Tx direction
		ESP_LOGE(TAG, "iso14443Poll: Failed to enable CRC TX");
    return PN5180_ERR_REGISTER;
	}
	if (writeRegisterOrMask(PN5180_CRC_TX_CONFIG, 0x01) != ESP_OK) { // 4. Switch the CRC extension on in Rx direction
		ESP_LOGE(TAG, "iso14443Poll: Failed to enable CRC RX");
    return PN5180_ERR_REGISTER;
	}

  cmd[1] = 0x70;
  memcpy(cmd+2, pSak, 5);
  clearIRQStatus(0x000FFFFF);

  ESP_LOGD(TAG, "sendCmd: %x:%x:%x:%x:%x:%x:%x", cmd[0],cmd[1],cmd[2],cmd[3],cmd[4], cmd[5], cmd[6]);
	sendData(cmd, 7, 0x00); // Cascade level (cascLevel), NVB (0x70)
  vTaskDelay(pdMS_TO_TICKS(15));
  
  readRegister(PN5180_RX_STATUS, &rxStatus);
  len = (uint16_t)(rxStatus & 0x000001ff);

  uint8_t* pUID = card.uid[numCard];
  if(!len){
    ESP_LOGI(TAG, "iso14443Poll: No length detected. We are done.");
    card.uidLength[numCard] = 4;
    card.type[numCard] = ISO14443;
    memcpy(pUID, cmd+2, 4);
    return PN5180_OK;
  }

  uint8_t* sak2Buf;
  sak2Buf = readData(len); // Read SAK (should always be len 1)
  ESP_LOGD(TAG, "readBuffer: %x:%x:%x:%x:%x", readBuffer[0],readBuffer[1],readBuffer[2],readBuffer[3],readBuffer[4]);
  ESP_LOGD(TAG, "sendCmd: %x:%x:%x:%x:%x:%x:%x", cmd[0],cmd[1],cmd[2],cmd[3],cmd[4], cmd[5], cmd[6]);

/////////
  ESP_LOGD(TAG, "ATS: %d", sak2Buf[0]);
  if (!(sak2Buf[0] & 0x04)) { // Check bit 3 of 2nd SAK. If 0, the uidLength = 4
    ESP_LOGD(TAG, "iso14443Poll: sak_sel2 confirmed. We should be activated.");
		card.uidLength[numCard] = 4;
    card.type[numCard] = ISO14443;
    iso14443GetSystemInfo(sak2Buf[0]);
    memcpy(pUID, cmd+2, 4);
    return PN5180_OK;
	}
  /*ESP_LOGI(TAG, "UID ongoing: %d:%d:%d", uid[0],uid[1],uid[2]);
  // Cascade level 2
  p = iso14443Collision(0x95);
  if(p[0] == 255){
    return ESP_ERR_INVALID_STATE;
  }
  else if (!(sak2 & 0x04)) { // Check bit 3 of 2nd SAK. If 0, the uidLength = 7
		for(int i=0; i<4; i++){
      uid[3+i] = sak[i];
    }
    ESP_LOGI(TAG, "UID len7: %d:%d:%d:%d:%d:%d:%d", uid[0],uid[1],uid[2],uid[3], uid[4],uid[5],uid[6]);
    uidLength = 7;
    setRF_off();
    printUID();
    return ESP_OK;
	}
  for(int i=0; i<3; i++){
    uid[3+i] = sak[i+1];
  }
  ESP_LOGI(TAG, "UID ongoing: %d:%d:%d:%d:%d:%d", uid[0],uid[1],uid[2],uid[3], uid[4],uid[5]);
  // Cascade level 3
  p = iso14443Collision(0x97);
  if(p[0] == 255){
    return ESP_ERR_INVALID_STATE;
  }
  else if (!(sak2 & 0x04)) { // Check bit 3 of 2nd SAK. If 0, the uidLength = 10
		for(int i=0; i<4; i++){
      uid[6+i] = sak[i];
    }
    uidLength = 10;
    setRF_off();
    printUID();
    return ESP_OK;
	}*/
  return PN5180_ERR_UNKNOWN; // If we got this far something went wrong.
}

void pn5180::iso14443GetSystemInfo(uint8_t ats){
  ESP_LOGD(TAG, "Looking for mifate tag type 0x%x", ats);
  const MifareType_t mType = mifare.find(ats)->second;
  card.numBlocks[numCard] = mType.numBlocks;
  card.blockSize[numCard] = mType.blockSize;
  card.startBlock[numCard] = mType.startBlock;
  card.endBlock[numCard] = mType.endBlock;
}

bool pn5180::mifareAuthenticate(uint8_t blockNo){
  uint8_t auth[13] = {0x0C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x60, 
                      blockNo, 
                      card.uid[numCard][0], 
                      card.uid[numCard][1], 
                      card.uid[numCard][2], 
                      card.uid[numCard][3]};
  clearIRQStatus(0x000FFFFF);

  uint8_t *p = readBuffer;
  esp_err_t ret = transceiveCommand(auth, 13, p, 1);
  if(ESP_OK != ret){
    ESP_LOGW(TAG, "mifareAuthenticate: Failed during transceive command");
  }
  ESP_LOGI(TAG, "mifareAuthenticate: Response: %x", p[0]);

  return (p[0]) ? false : true;
}

bool pn5180::mifareReadBlock(uint8_t cardNum, uint8_t blockNo) {
  uint8_t readB[2] = {0x30, blockNo};

  clearIRQStatus(0x000FFFFF);
  sendData(readB, 2, 0);
  vTaskDelay(pdMS_TO_TICKS(15));
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001FF);
  
  ESP_LOGI(TAG, "mifareReadBlock: blockNo=%d len=%d", blockNo, len);
  if(!len){
    ESP_LOGW(TAG, "Unable to read Mifare block");
    return false;
  }

  uint8_t *p = readBuffer;
  p = readData(len);
  uint8_t startAddr = card.blocksRead[cardNum] * card.blockSize[cardNum];
  for (int i=0; i<card.blockSize[cardNum]; i++) {
    card.data[cardNum][startAddr + i] = p[i];
  }
  uint8_t *q = card.blocksRead;
  q[cardNum] = card.blocksRead[cardNum] + 1;
  ESP_LOGI(TAG, "mifareReadBlock: cardNum=%d, blocksRead=%d, data=", cardNum, q[cardNum]);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, p, len, ESP_LOG_INFO);
  
	return true;
}

bool pn5180::mifareReadMultipleBlock(uint8_t cardNum, uint8_t blockNo, uint8_t numBlock) {
  uint8_t readB[2] = {0x30, blockNo};

  for(int block=0; block<numBlock; block++) {
    readB[1] = blockNo + block;
    clearIRQStatus(0x000FFFFF);
    sendData(readB, 2, 0);
    vTaskDelay(pdMS_TO_TICKS(15));
    uint32_t rxStatus;
    readRegister(PN5180_RX_STATUS, &rxStatus);
    uint16_t len = (uint16_t)(rxStatus & 0x000001FF);
    
    ESP_LOGI(TAG, "mifareReadBlock: blockNo=%d len=%d", blockNo, len);
    if(!len){
      ESP_LOGW(TAG, "Unable to read Mifare block");
      return false;
    }

    uint8_t *p = readBuffer;
    p = readData(len);
    uint8_t startAddr = block * card.blockSize[cardNum];
    for (int i=0; i<card.blockSize[cardNum]; i++) {
      card.data[cardNum][startAddr + i] = p[i];
    }
    uint8_t *q = card.blocksRead;
    q[cardNum] = card.blocksRead[cardNum] + 1;
    ESP_LOGI(TAG, "mifareReadBlock: cardNum=%d, blocksRead=%d, data=", cardNum, q[cardNum]);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, p, len, ESP_LOG_INFO);
  }
	return true;
}

bool pn5180::mifareWriteBlock(const char *data, uint8_t blockNo) {
  uint8_t writeCmd[2] = {0xA0, blockNo};
  //cmd[0] = 0xA0;
  //cmd[1] = blockNo;
  ESP_LOGI(TAG, "mifareWrite: cmd=%x %x blockNo=%d", writeCmd[0], writeCmd[1], blockNo);
  clearIRQStatus(0x000FFFFF);
  writeRegisterAndMask(PN5180_CRC_RX_CONFIG, 0xFFFFFFFE);
  
  sendData(writeCmd, 2, 0);
  vTaskDelay(pdMS_TO_TICKS(15));
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!len){
    ESP_LOGW(TAG,"mifareWrite: No response in pt1");
    return false;
  }
  ESP_LOGI(TAG, "mifareWrite: ACK len=%d", len);

  uint8_t *p = readBuffer;
  p = readData(len);
  uint8_t startAddr = blockNo * card.blockSize[numCard];
  for (int i=0; i<card.blockSize[numCard]; i++) {
    card.data[numCard][startAddr + i] = p[1+i];
  }
  if(p[0] != 0x0A){
    ESP_LOGW(TAG,"mifareWrite: Write pt1 failed! ACK=%d", p[0]);
    while(1) vTaskDelay(portMAX_DELAY);
    //return false;
  }
  ESP_LOGD(TAG,"mifareWrite: Write pt1 acknowledged");

  uint8_t dataToWrite[card.blockSize[numCard]] = {0};
  len = strlen(data);
  memcpy(dataToWrite, data, len);

  clearIRQStatus(0x000FFFFF);
  ESP_LOGI(TAG,"mifareWrite: len=%d dataToWrite=", card.blockSize[numCard]);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, dataToWrite, card.blockSize[numCard], ESP_LOG_INFO);
  sendData(dataToWrite, card.blockSize[numCard], 0);
  vTaskDelay(pdMS_TO_TICKS(15));
  readRegister(PN5180_RX_STATUS, &rxStatus);
  len = (uint16_t)(rxStatus & 0x000001ff);
  if(!len){
    ESP_LOGW(TAG,"mifareWrite: No response in pt2");
    return false;
  }

  p = readData(len);
  if(p[0] != 0x0A){
    ESP_LOGW(TAG,"mifareWrite: Write pt2 failed! ACK=%d", p[0]);
    return false;
  }
  ESP_LOGD(TAG,"mifareWrite: Write pt2 acknowledged");

  writeRegisterOrMask(PN5180_CRC_RX_CONFIG, 0x01);
  return true;
}

void pn5180::mifareHalt(void) {
  uint8_t halt[2] = {0x50, 0x00};

  writeRegisterOrMask(PN5180_CRC_RX_CONFIG, 0x1);
  clearIRQStatus(0x000FFFFF);
	sendData(halt, 2, 0x00);
  vTaskDelay(pdMS_TO_TICKS(15));
}

/*
 * Write single block, code=21
 *
 * Request format: SOF, Requ.Flags, WriteSingleBlock, UID (opt.), BlockNumber, BlockData (len=blcokLength), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Resp.Flags, CRC16, EOF
 */
bool pn5180::iso15693WriteBlock(uint8_t cardNum, uint8_t blockNo, uint8_t* data, uint8_t len) {
  if(len > card.blockSize[cardNum]) len = card.blockSize[cardNum];

  uint8_t dataToWrite[8] = {0};
  memcpy(dataToWrite, data, len);
  
  //                               flags, cmd, uid,             blockNo
  uint8_t writeCmd[] = { 0x22, 0x21, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                                  |\- high data rate
  //                                  \-- no options, addressed by UID

  //uint8_t writeCmdSize = sizeof(writeSingleBlockCmd) + blockSize;
  uint8_t cmd[sizeof(writeCmd) + len];
  memcpy(writeCmd, card.uid[cardNum], 8);
  writeCmd[10] = blockNo;
  memcpy(cmd, writeCmd, 2);
  memcpy(cmd+sizeof(writeCmd), dataToWrite, len);

  ESP_LOGI(TAG,"iso15693WriteBlock: Write Single Block #%d, size=%d: ", blockNo, card.blockSize[cardNum]);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, writeCmd, 21, ESP_LOG_INFO);

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(writeCmd, 13+card.blockSize[cardNum], 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  uint32_t irqStatus = getIRQStatus();
  if (!(irqStatus & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    ESP_LOGW(TAG, "iso15693WriteBlock: No RX_SOF from IRQ_STATUS");
    //printIRQStatus(TAG, irqR);
    return false;
  }
  uint16_t retries = 10;
  while (!(irqStatus & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(5 / portTICK_PERIOD_MS);
    irqStatus = getIRQStatus();
	  retries--;
  }
  if(!(irqStatus & PN5180_RX_IRQ_STAT)) ESP_LOGW(TAG, "iso15693WriteBlock: No RX from IRQ_STATUS");
  return true;
}

bool pn5180::iso15693WriteBlock(const char *blockData, uint8_t blockNo) {
  uint8_t dataToWrite[card.blockSize[numCard]] = {0};
  uint8_t len = strlen(blockData);
  if(len > card.blockSize[numCard]) len = card.blockSize[numCard];
  memcpy(dataToWrite, blockData, len);
  
  //                               flags, cmd, uid,             blockNo
  uint8_t writeCmd[] = { 0x22, 0x21, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                                  |\- high data rate
  //                                  \-- no options, addressed by UID

  //uint8_t writeCmdSize = sizeof(writeSingleBlockCmd) + blockSize;
  uint8_t cmd[sizeof(writeCmd) + card.blockSize[numCard]];
  //ESP_LOGD(TAG,"Free malloc after: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
  //uint8_t pos = 0;
  memcpy(writeCmd+2, card.uid[numCard], 8);
  writeCmd[10] = blockNo;
  memcpy(cmd, writeCmd, sizeof(writeCmd));
  memcpy(cmd+sizeof(writeCmd), dataToWrite, card.blockSize[numCard]);
  //writeCmd[pos++] = writeSingleBlockCmd[0];
  //writeCmd[pos++] = writeSingleBlockCmd[1];
  //for (int i=0; i<8; i++) {
  //  writeCmd[pos++] = uid[i];
  //}
  //writeCmd[pos++] = blockNo;
  //uint8_t startAddr = blockNo * card.blockSize[numCard];
  // Start of actual data creation loop
  //for (int i=0; i<len; i++) {
  //  writeCmd[pos++] = blockData[startAddr + i];
  //}
  // End of data loop

  ESP_LOGI(TAG,"iso15693WriteBlock: Write Single Block #%d, size=%d: ", blockNo, sizeof(writeCmd)+card.blockSize[numCard]);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd, sizeof(cmd), ESP_LOG_INFO);

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(cmd, sizeof(writeCmd)+card.blockSize[numCard], 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  /*uint32_t irqStatus = getIRQStatus();
  if (!(irqStatus & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    //printIRQStatus(TAG, irqR);
    ESP_LOGW(TAG, "iso15693WriteBlock: Didn't detect RX_SOF");
    return false;
  }
  uint16_t retries = 10;
  while (!(irqStatus & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(5 / portTICK_PERIOD_MS);
    irqStatus = getIRQStatus();
	  retries--;
  }*/
  return true;
}

/*
 * Read multiple block, code=23
 *
 * Request format: SOF, Req.Flags, ReadMultipleBlock, UID (opt.), FirstBlockNumber, numBlocks, CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, BlockData (len=nfc->blockSize * numBlock), CRC16, EOF
 */
bool pn5180::iso15693ReadMultipleBlock(uint8_t cardNum, uint8_t blockNo, uint8_t numBlock) {
  if(blockNo > card.endBlock[cardNum]){
    ESP_LOGW(TAG, "iso15693ReadMultipleBlock: Starting block exceeds memory");
    return false;
  }
  if( (blockNo + numBlock - 1) > card.endBlock[cardNum] ){
    ESP_LOGW(TAG, "iso15693ReadMultipleBlock: End of block exceeds memory");
    return false;
  }

  //uint8_t readMultipleCmd[4] = {0x02, 0x23, blockNo, numBlock-1};
  //                              flags, cmd, uid,             blockNo
  uint8_t cmd[12] = { 0x22, 0x23, 1,2,3,4,5,6,7,8, blockNo, (uint8_t)(numBlock-1) }; // UID has LSB first!
  //                                |\- high data rate
  //                                \-- no options, addressed by UID
  for (int i=0; i<8; i++) {
    cmd[2+i] = card.uid[cardNum][i];
  }

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(cmd, 12, 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  uint32_t irqStatus = getIRQStatus();
  uint16_t retries = 5;
  while (!(irqStatus & PN5180_RX_SOF_DET_IRQ_STAT) && retries > 0) {   // 8. wait for RX start of frame (max 50ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqStatus = getIRQStatus();
	  retries--;
  }
  if (!(irqStatus & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    //printIRQStatus(TAG, irqStatus);
    ESP_LOGW(TAG, "iso15693ReadMultipleBlock: No data response.");
    return false;
  }
  retries = 5;
  while (!(irqStatus & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqStatus = getIRQStatus();
	  retries--;
  }
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!(irqStatus & PN5180_RX_IRQ_STAT)){
    ESP_LOGW(TAG, "iso15693ReadMultipleBlock: Timeout waiting for response to end. Datalen=%d", len);
    return false;
  }
  if(!len){
    ESP_LOGW(TAG, "iso15693ReadMultipleBlock: Length is 0!!");
    return false;
  }

  uint8_t *p = readBuffer;
  uint8_t startAddr = card.blocksRead[cardNum] * card.blockSize[cardNum];
  p = readData(len);
  for (int i=0; i<numBlock * card.blockSize[cardNum]; i++) {
    ESP_LOGD(TAG,"iso15693ReadMultipleBlock: resultPtr=%d", p[1+i]);
    card.data[cardNum][startAddr + i] = p[1+i];
  }
  card.blocksRead[cardNum]++;

  ESP_LOGD(TAG,"iso15693ReadMultipleBlock: Value=");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, card.data[cardNum], numBlock * card.blockSize[cardNum], ESP_LOG_DEBUG);

  setRF_off();
  return true;
}

void pn5180::printError(uint8_t err) {
  char strErr[50] = "";
  switch (err) {
    case EC_NO_CARD: strcat(strErr,"No card detected!"); break;
    case ISO15693_EC_OK: strcat(strErr,"OK!"); break;
    case ISO15693_EC_NOT_SUPPORTED: strcat(strErr,"Command is not supported!"); break;
    case ISO15693_EC_NOT_RECOGNIZED: strcat(strErr,"Command is not recognized!"); break;
    case ISO15693_EC_OPTION_NOT_SUPPORTED: strcat(strErr,"Option is not supported!"); break;
    case ISO15693_EC_UNKNOWN_ERROR: strcat(strErr,"Unknown error!"); break;
    case ISO15693_EC_BLOCK_NOT_AVAILABLE: strcat(strErr,"Specified block is not available!"); break;
    case ISO15693_EC_BLOCK_ALREADY_LOCKED: strcat(strErr,"Specified block is already locked!"); break;
    case ISO15693_EC_BLOCK_IS_LOCKED: strcat(strErr,"Specified block is locked and cannot be changed!"); break;
    case ISO15693_EC_BLOCK_NOT_PROGRAMMED: strcat(strErr,"Specified block was not successfully programmed!"); break;
    case ISO15693_EC_BLOCK_NOT_LOCKED: strcat(strErr,"Specified block was not successfully locked!"); break;
    default:
      if ((err >= 0xA0) && (err <= 0xDF)) {
        strcat(strErr,"Custom command error code!");
      }
      else strcat(strErr,"Undefined error code in ISO15693!");
  }
  ESP_LOGE(TAG, "ISO15693 Error: %s", strErr);
}

void pn5180::printUID(uint8_t cardNum){
  printf("\033[32mI (%ld) %s: UID=", esp_log_timestamp(), TAG);
  for(int i=card.uidLength[cardNum]-1; i>=0; i--){
    if(card.uid[cardNum][i] < 16) printf("0");
    printf("%X", card.uid[cardNum][i]);
    if(i > 0) printf(":");
  }
  printf("\n\033[0m");
}

void pn5180::printSingleBlock(uint8_t cardNum, uint8_t blockNum){
  if(blockNum + card.startBlock[cardNum] > card.endBlock[cardNum]){
    ESP_LOGW(TAG, "Chosen block (%d) is not a valid block", blockNum + card.startBlock[cardNum]);
  }
  else{
    if(ESP_LOG_INFO <= esp_log_level_get(TAG)){
      uint16_t startAddr = card.blockSize[cardNum] * (blockNum /*+ card.startBlock[cardNum]*/);
      // Hex print
      ESP_LOGD(TAG, "startAddr=%d", startAddr);
      printf("\033[32mI (%ld) %s: ", esp_log_timestamp(), TAG);
      for (int i=0; i<card.blockSize[cardNum]; i++) {
        if(card.data[cardNum][startAddr + i] < 16) printf("0");
        printf("%X", card.data[cardNum][startAddr + i]);
        if(i < card.blockSize[cardNum] - 1) printf(":");
      }
      
      printf(" ");
      
      // String print
      for (int i=0; i<card.blockSize[cardNum]; i++) {
        char c = card.data[cardNum][startAddr + i];
        if (isprint(c)) {
          printf("%c",c);
        }
        else printf(".");
      }
      printf("\n\033[0m");
    }
  }
}

void pn5180::printMultipleBlock(uint8_t cardNum, uint8_t blockNum, uint8_t numBlock){
  if(blockNum + card.startBlock[cardNum] + numBlock > card.endBlock[cardNum]){
    ESP_LOGW(TAG, "Chosen blocks exceed card memory");
  }
  else if(ESP_LOG_INFO <= esp_log_level_get(TAG)){
    // Hex print
    for(int block=blockNum; block<numBlock; block++){
      uint16_t startAddr = ((block + card.startBlock[cardNum]) * card.blockSize[cardNum]);
      printf("\033[32mI (%ld) %s: Block %d", esp_log_timestamp(), TAG, blockNum);
      for (int i=0; i<card.blockSize[cardNum]; i++) {
        if(card.data[cardNum][startAddr + i] < 16) printf("0");
        printf("%X", card.data[cardNum][startAddr + i]);
        if(i < card.blockSize[cardNum] - 1) printf(":");
      }
      
      printf(" ");
      
      // String print
      for (int i=0; i<card.blockSize[cardNum]; i++) {
        char c = card.data[cardNum][startAddr + i];
        if (isprint(c)) {
          printf("%c",c);
        }
        else printf(".");
      }
      printf("\n\033[0m");
    }
  }
}

void pn5180::printData(uint8_t cardNum){
  if(ESP_LOG_INFO <= esp_log_level_get(TAG)){
    bool flag_stopChar = false;
    uint16_t pos = 0;
    // Hex print
    printf("\033[32mI (%ld) %s: printData: ", esp_log_timestamp(), TAG);
    while(!flag_stopChar){
      char c = card.data[cardNum][pos++];
      if (isprint(c)) {
        printf("%c",c);
      }
      else flag_stopChar = true;
    }
    printf("\n\033[0m");
  }
}

esp_err_t pn5180::transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
  esp_err_t ret;
  ////////////////////
  // Initialization //
  ////////////////////
  ESP_LOGD(TAG, "transceiveCommand: Write, wait for busy...");
  ret = busyWait(1000); // 1.
  if(ret == ESP_ERR_TIMEOUT){
    ESP_LOGE(TAG, "transceiveCommand: BUSY signal line timeout");
    return ret;
  }
  ESP_LOGD(TAG, "transceiveCommand: SPI transaction: write %d read %d", sendBufferLen, recvBufferLen);
  
  //////////////////
  // Send command //
  //////////////////
  ESP_LOGD(TAG, "transceiveCommand: Write data:");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, sendBuffer, sendBufferLen, ESP_LOG_DEBUG);
  ret = spi_txn(dev, sendBuffer, sendBufferLen, NULL, 0); // 2. 3. 4.
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "pn5180_command: SPI transaction write failed");
    return ret;
  }

  // Finish if write-only command
  if ((0 == recvBuffer) || (0 == recvBufferLen)) return ret;

  ESP_LOGD(TAG, "command: Read, wait for busy...");
  ret = busyWait(1000); // 5.
  if(ret == ESP_ERR_TIMEOUT){
    ESP_LOGE(TAG, "command: BUSY signal line timeout");
    return ret;
  }
  memset(recvBuffer, 0xFF, recvBufferLen);

  //////////////////////
  // Receive Response //
  //////////////////////
  ret = spi_txn(dev, recvBuffer, recvBufferLen, recvBuffer, recvBufferLen); // 6. 7. 8.
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "pn5180_command: SPI transaction read failed");
    reset();
    return ret;
  }

  ESP_LOGD(TAG, "command: Read data:");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, recvBuffer, recvBufferLen, ESP_LOG_DEBUG);
  
  return ret;
}

esp_err_t pn5180::spi_txn(spi_device_handle_t dev, const void *tx, int txLen, void *rx, int rxLen) {
    spi_transaction_t txn = {
        .length = (size_t)(txLen * 8),
        .rxlength = (size_t)(rxLen * 8),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(dev, &txn);
}

/*
 * The BUSY signal is used to indicate that the PN5180 is not able to send or receive data
 * over the SPI interface
 */
esp_err_t pn5180::busyWait(uint32_t timeout){
  while (gpio_get_level(PN5180_BUSY) && timeout > 0){
    vTaskDelay(pdMS_TO_TICKS(1));
    timeout--;
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  if(gpio_get_level(PN5180_BUSY)){
    ESP_LOGE(TAG, "busy_wait: Timeout waiting for BUSY pin LOW");
    reset();
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}