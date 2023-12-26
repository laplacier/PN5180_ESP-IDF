#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>

/*
 * numBlocks = Number of addressable blocks
 * blockSize = Number of bytes in a block
 * startBlock = 0-index of first safe R/W
 * endBlock = 0-index of last safe R/W
 */
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