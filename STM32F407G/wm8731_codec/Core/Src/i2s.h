#ifndef I2S_H
#define I2S_H

#include "codec.h"
#include "audio.h"

void I2S_Block_Init(void);
void I2S_Block_PlayRec(uint32_t txAddr, uint32_t rxAddr, uint32_t Size);

#endif /* I2S_H */
