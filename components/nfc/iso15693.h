/* NAME: iso15693.h
 *
 * DESC: ISO15693 protocol on NXP Semiconductors PN5180 module for ESP-IDF.
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
#ifndef iso15693_h
#define iso15693_h

#include "pn5180.h"

typedef enum {
  EC_NO_CARD = -1,
  ISO15693_EC_OK = 0,
  ISO15693_EC_NOT_SUPPORTED = 0x01,
  ISO15693_EC_NOT_RECOGNIZED = 0x02,
  ISO15693_EC_OPTION_NOT_SUPPORTED = 0x03,
  ISO15693_EC_UNKNOWN_ERROR = 0x0f,
  ISO15693_EC_BLOCK_NOT_AVAILABLE = 0x10,
  ISO15693_EC_BLOCK_ALREADY_LOCKED = 0x11,
  ISO15693_EC_BLOCK_IS_LOCKED = 0x12,
  ISO15693_EC_BLOCK_NOT_PROGRAMMED = 0x13,
  ISO15693_EC_BLOCK_NOT_LOCKED = 0x14,
  ISO15693_EC_CUSTOM_CMD_ERROR = 0xA0
} ISO15693ErrorCode_t;

typedef struct {
  uint8_t manufacturer;
  uint8_t type;
  uint8_t uid[8];
  uint8_t dsfid;
  uint8_t afi;
  uint8_t ic_ref;
  // The physical memory of an ISO15693 VICC is organized in the form of blocks or pages of fixed size. Up to 256 blocks can be addressed and a block size can be up to 32 bytes.
  uint8_t numBlocks;
  uint16_t blockSize;
  uint8_t *blockData;
} ISO15693NFC_t;

typedef struct {
  uint8_t numCard;
  uint8_t numCol;
  uint8_t uid[16][8];
} ISO15693Inventory_t;

extern const char afi_string[14][30];

ISO15693ErrorCode_t pn5180_ISO15693Command(uint8_t *cmd, uint16_t cmdLen, uint8_t **resultPtr);
ISO15693ErrorCode_t pn5180_getInventory(ISO15693NFC_t *nfc);
ISO15693ErrorCode_t pn5180_getInventoryMultiple(ISO15693Inventory_t *nfc);
ISO15693ErrorCode_t pn5180_inventoryPoll(ISO15693Inventory_t *nfc, uint32_t *collision, uint8_t *numCol);
ISO15693ErrorCode_t pn5180_readSingleBlock(ISO15693NFC_t *nfc, uint8_t blockNo);
ISO15693ErrorCode_t pn5180_writeSingleBlock(ISO15693NFC_t *nfc, uint8_t blockNo);
ISO15693ErrorCode_t pn5180_readMultipleBlock(ISO15693NFC_t *nfc, uint8_t blockNo, uint8_t numBlock);
ISO15693ErrorCode_t pn5180_getSystemInfo(ISO15693NFC_t *nfc);
// ICODE SLIX2 specific commands, see https://www.nxp.com/docs/en/data-sheet/SL2S2602.pdf
ISO15693ErrorCode_t pn5180_getRandomNumber(uint8_t *randomData);
ISO15693ErrorCode_t pn5180_setPassword(uint8_t identifier, uint8_t *password, uint8_t *random);
ISO15693ErrorCode_t pn5180_enablePrivacy(uint8_t *password, uint8_t *random);
// helpers
ISO15693ErrorCode_t pn5180_enablePrivacyMode(uint8_t *password);
ISO15693ErrorCode_t pn5180_disablePrivacyMode(uint8_t *password); 
esp_err_t pn5180_setupRF(void);
void iso15693_printError(ISO15693ErrorCode_t errno);
void iso15693_printGeneric(const char *tag, uint8_t *dataBuf, uint16_t blockSize, uint8_t blockNum);
#endif /* PN5180ISO15693_H */
