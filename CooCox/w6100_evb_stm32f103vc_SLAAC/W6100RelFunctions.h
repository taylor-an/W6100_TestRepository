#ifndef __W6100RELFUNCTIONS_H_
#define __W6100RELFUNCTIONS_H_

#include "wizchip_conf.h"

#define W6100Address  ((uint32_t)0x60000000)

uint8_t spiReadByte(void);
void spiWriteByte(uint8_t byte);

iodata_t busReadByte(uint32_t addr);
void busWriteByte(uint32_t addr, iodata_t data);

void csEnable(void);
void csDisable(void);
void resetAssert(void);
void resetDeassert(void);
void EnterCris(void);
void ExitCris(void);
void W6100Reset(void);
#endif
