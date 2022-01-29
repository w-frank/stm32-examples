#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "stm32l4xx_hal.h"


void EEPROM_Write(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_Read(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_PageErase(uint16_t page);

#endif /* INC_EEPROM_H_ */
