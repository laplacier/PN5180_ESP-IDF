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
#ifndef iso14443_h
#define iso14443_h

#include "pn5180.h"

typedef struct {
  uint8_t uid[6];
  uint8_t uid_raw[8];
  // The physical memory of an ISO15693 VICC is organized in the form of blocks or pages of fixed size. Up to 256 blocks can be addressed and a block size can be up to 32 bytes.
  uint8_t numBlocks;
  uint16_t blockSize;
  uint8_t* blockData;
} ISO14443NFC_t;

uint16_t pn5180_rxBytesReceived();
uint32_t pn5180_GetNumberOfBytesReceivedAndValidBits();

// Mifare TypeA
int8_t pn5180_activateTypeA(uint8_t *buffer, uint8_t kind);
esp_err_t pn5180_mifareBlockRead(uint8_t blockno,uint8_t *buffer);
uint8_t pn5180_mifareBlockWrite16(uint8_t blockno, uint8_t *buffer);
esp_err_t pn5180_mifareHalt();

esp_err_t pn5180_setupRF();
int8_t pn5180_readCardSerial(uint8_t *buffer);    
esp_err_t pn5180_isCardPresent();    

#endif