/* NAME: iso14443.h
 *
 * DESC: ISO14443 protocol on NXP Semiconductors PN5180 module for ESP-IDF.
 * FILE TRANSLATED, BUT UNTESTED.
 * 
 * Forked from https://github.com/ATrappmann/PN5180-Library
 * Copyright (c) 2019 by Dirk Carstensen. All rights reserved.
 *
 * This file is part of the PN5180 component for ESP-IDF.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 */
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "iso14443.h"
#include "pn5180.h"

#ifdef FALSE
static const char* TAG = "iso14443.c";

esp_err_t pn5180_setupRF() {
	esp_err_t ret;
	ESP_LOGD(TAG, "Loading RF-Configuration...");
	ret = pn5180_loadRFConfig(0x00, 0x80);
	if (ret != ESP_OK) {  // ISO14443 parameters
		ESP_LOGE(TAG, "Error loading RF Config");
		return ret;
	}

	ESP_LOGD(TAG, "Turning ON RF field...");
	ret = pn5180_setRF_on();
	if(ret != ESP_OK){
		ESP_LOGE(TAG, "Error turning on RF");
		return ret;
	}

	return ESP_OK;
}

uint16_t pn5180_rxBytesReceived() {
	uint32_t rxStatus;
	uint16_t len = 0;
	pn5180_readRegister(PN5180_RX_STATUS, &rxStatus);
	// Lower 9 bits has length
	len = (uint16_t)(rxStatus & 0x000001ff);
	return len;
}

/*
* buffer : must be 10 byte array
* buffer[0-1] is ATQA
* buffer[2] is sak
* buffer[3..6] is 4 byte UID
* buffer[7..9] is remaining 3 bytes of UID for 7 Byte UID tags
* kind : 0  we send REQA, 1 we send WUPA
*
* return value: the uid length:
* -	zero if no tag was recognized
* - -1 general error
* - -2 card in field but with error
* -	single Size UID (4 byte)
* -	double Size UID (7 byte)
* -	triple Size UID (10 byte) - not yet supported
*/
int8_t pn5180_activateTypeA(uint8_t *buffer, uint8_t kind) {
	uint8_t cmd[7];
	uint8_t* ptr;
	uint8_t uidLength = 0;
	
	// Load standard TypeA protocol already done in reset()
	if (pn5180_loadRFConfig(0x0, 0x80) != ESP_OK) {
		ESP_LOGE(TAG, "*** ERROR: Load standard TypeA protocol failed!");
		return -1;
	}
	
	// OFF Crypto
	if (pn5180_writeRegisterWithAndMask(PN5180_SYSTEM_CONFIG, 0xFFFFFFBF) != ESP_OK) {
		ESP_LOGE(TAG, "*** ERROR: OFF Crypto failed!");
		return -1;
	}
	// clear RX CRC
	if (pn5180_writeRegisterWithAndMask(PN5180_CRC_RX_CONFIG, 0xFFFFFFFE) != ESP_OK) {
		ESP_LOGE(TAG, "*** ERROR: Clear RX CRC failed!");
		return -1;
	}
	// clear TX CRC
	if (pn5180_writeRegisterWithAndMask(PN5180_CRC_TX_CONFIG, 0xFFFFFFFE) != ESP_OK) {
		ESP_LOGE(TAG, "*** ERROR: Clear TX CRC failed!");
		return -1;
	}

	//Send REQA/WUPA, 7 bits in last byte
	cmd[0] = (kind == 0) ? 0x26 : 0x52;
	if (pn5180_sendData(cmd, 1, 0x07) != ESP_OK) {
		ESP_LOGE(TAG, "*** ERROR: Send REQA/WUPA failed!");
		return -1;
	}

	// READ 2 bytes ATQA into buffer
	buffer = pn5180_readData(2);
	
	// send Anti collision 1, 8 bits in last byte
	cmd[0] = 0x93;
	cmd[1] = 0x20;
	if (pn5180_sendData(cmd, 2, 0x00) != ESP_OK) {
		ESP_LOGE(TAG, "*** ERROR: Send Anti collision 1 failed!");
		return -2;
	}

	// read 5 bytes sak, we will store at offset 2 for later usage
	ptr = &cmd[2]; 
	ptr = pn5180_readData(5);
	
	//Enable RX CRC calculation
	if (pn5180_writeRegisterWithOrMask(PN5180_CRC_RX_CONFIG, 0x01) != ESP_OK){
		ESP_LOGE(TAG, "Unable to write RX OR Mask");
		return -2;
	}
	//Enable TX CRC calculation
	if (pn5180_writeRegisterWithOrMask(PN5180_CRC_TX_CONFIG, 0x01) != ESP_OK){
		ESP_LOGE(TAG, "Unable to write TX OR Mask");
		return -2;
	}

	//Send Select anti collision 1, the remaining bytes are already in offset 2 onwards
	cmd[0] = 0x93;
	cmd[1] = 0x70;
	if (pn5180_sendData(cmd, 7, 0x00) != ESP_OK) {
		// no remaining bytes, we have a 4 byte UID
		return 4;
	}
	//Read 1 byte SAK into buffer[2]
	ptr = &buffer[2]; 
	ptr = pn5180_readData(1);
	// Check if the tag is 4 Byte UID or 7 byte UID and requires anti collision 2
	// If Bit 3 is 0 it is 4 Byte UID
	if ((buffer[2] & 0x04) == 0) {
		// Take first 4 bytes of anti collision as UID store at offset 3 onwards. job done
		for (int i = 0; i < 4; i++) buffer[3+i] = cmd[2 + i];
		uidLength = 4;
	}
	else {
		// Take First 3 bytes of UID, Ignore first byte 88(CT)
		if (cmd[2] != 0x88)
		  return 0;
		for (int i = 0; i < 3; i++) buffer[3+i] = cmd[3 + i];
		// Clear RX CRC
		if (pn5180_writeRegisterWithAndMask(PN5180_CRC_RX_CONFIG, 0xFFFFFFFE) != ESP_OK){
			ESP_LOGE(TAG, "Unable to write RX AND Mask");
			return -2;
		}
		// Clear TX CRC
		if (pn5180_writeRegisterWithAndMask(PN5180_CRC_TX_CONFIG, 0xFFFFFFFE) != ESP_OK){
			ESP_LOGE(TAG, "Unable to write TX AND Mask");
			return -2;
		}
		// Do anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x20;
		if (pn5180_sendData(cmd, 2, 0x00) != ESP_OK){
			ESP_LOGE(TAG, "Error sending anti collision 2");
			return -2;
		}
		//Read 5 bytes. we will store at offset 2 for later use
		ptr = &cmd[2];
		ptr = pn5180_readData(5);
		// first 4 bytes belongs to last 4 UID bytes, we keep it.
		for (int i = 0; i < 4; i++) {
		  buffer[6 + i] = cmd[2+i];
		}
		//Enable RX CRC calculation
		if (pn5180_writeRegisterWithOrMask(PN5180_CRC_RX_CONFIG, 0x01) != ESP_OK){
			ESP_LOGE(TAG, "Unable to write RX CRC");
			return -2;
		}
		//Enable TX CRC calculation
		if (pn5180_writeRegisterWithOrMask(PN5180_CRC_TX_CONFIG, 0x01) != ESP_OK){
			ESP_LOGE(TAG, "Unable to write TX CRC");
			return -2;
		}
		//Send Select anti collision 2 
		cmd[0] = 0x95;
		cmd[1] = 0x70;
		if (pn5180_sendData(cmd, 7, 0x00) != ESP_OK){
			ESP_LOGE(TAG, "Unable to send anti collision 2");
			return -2;
		}
		//Read 1 byte SAK into buffer[2]
		ptr = &buffer[2];
		ptr = pn5180_readData(1);
		uidLength = 7;
	}
    return uidLength;
}

esp_err_t pn5180_mifareBlockRead(uint8_t blockno, uint8_t *buffer) {
	esp_err_t = ret;
	uint16_t len;
	uint8_t cmd[2];
	// Send mifare command 30,blockno
	cmd[0] = 0x30;
	cmd[1] = blockno;
	ret = sendData(cmd, 2, 0x00); 
	if (ret != ESP_OK)
	  return ret;
	//Check if we have received any data from the tag
	vTaskDelay(pdMS_TO_TICKS(5));
	len = pn5180_rxBytesReceived();
	if (len == 16) {
		// READ 16 bytes into  buffer
		return pn5180_readData(16, buffer);
	}
	return ESP_ERR_INVALID_SIZE;
}


uint8_t pn5180_mifareBlockWrite16(uint8_t blockno, uint8_t *buffer) {
	uint8_t cmd[2];
	// Clear RX CRC
	pn5180_writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);

	// Mifare write part 1
	cmd[0] = 0xA0;
	cmd[1] = blockno;
	pn5180_sendData(cmd, 2, 0x00);
	pn5180_readData(1, cmd);

	// Mifare write part 2
	pn5180_sendData(buffer,16, 0x00);
	vTaskDelay(pdMS_TO_TICKS(5));

	// Read ACK/NAK
	pn5180_readData(1, cmd);

	//Enable RX CRC calculation
	pn5180_writeRegisterWithOrMask(CRC_RX_CONFIG, 0x1);
	return cmd[0];
}

esp_err_t pn5180_mifareHalt() {
	uint8_t cmd[2];
	//mifare Halt
	cmd[0] = 0x50;
	cmd[1] = 0x00;
	return pn5180_sendData(cmd, 2, 0x00);
}
#endif

int8_t pn5180_readCardSerial(uint8_t *buffer) {
  
    uint8_t response[10];
	int8_t uidLength;
	// An ISO14443 device will respond to a request with
    // ATQA
    // Offset 2 is SAK.
    // UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
    // UID 7 bytes : offset 3 to 9 is UID
	// | ATQA  |SAK|           UID             |
	// | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
    for (int i = 0; i < 10; i++) response[i] = 0;
	// try to activate Type A until response or timeout
	uidLength = pn5180_activateTypeA(response, 0);

	
	if (uidLength <= 0)
	  return uidLength;
	// UID length must be at least 4 bytes
	if (uidLength < 4)
	  return 0;
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
	  uidLength = 0;
		
	// first UID byte should not be 0x00 or 0xFF
	if ((response[3] == 0x00) || (response[3] == 0xFF)) 
		uidLength = 0;
		
	// check for valid uid, skip first byte (0x04)
	// 0x04 0x00 0xFF 0x00 => invalid uid
	bool validUID = false;
	for (int i = 1; i < uidLength; i++) {
		if ((response[i+3] != 0x00) && (response[i+3] != 0xFF)) {
			validUID = true;
			break;
		}
	}
	if (uidLength == 4) {
		if ((response[3] == 0x88)) {
			// must not be the CT-flag (0x88)!
			validUID = false;
		};
	}
	if (uidLength == 7) {
		if ((response[6] == 0x88)) {
			// must not be the CT-flag (0x88)!
			validUID = false;
		};
		if ((response[6] == 0x00) && (response[7] == 0x00) && (response[8] == 0x00) && (response[9] == 0x00)) {
			validUID = false;
		};
		if ((response[6] == 0xFF) && (response[7] == 0xFF) && (response[8] == 0xFF) && (response[9] == 0xFF)) {
			validUID = false;
		};
	};
//	mifareHalt();
	if (validUID) {
		for (int i = 0; i < uidLength; i++) buffer[i] = response[i+3];
		return uidLength;
	} else {
		return 0;
	}
}

esp_err_t pn5180_isCardPresent() {

    uint8_t buffer[10];
	if(pn5180_readCardSerial(buffer) >=4) return ESP_OK;
	return ESP_FAIL;
}