#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "main.h"

#define WM8731_I2C_ADDR 0x34

#define WM8731_REG_LLINEIN   0
#define WM8731_REG_RLINEIN   1
#define WM8731_REG_LHEADOUT  2
#define WM8731_REG_RHEADOUT  3
#define WM8731_REG_ANALOG    4
#define WM8731_REG_DIGITAL   5
#define WM8731_REG_POWERDOWN 6
#define WM8731_REG_INTERFACE 7
#define WM8731_REG_SAMPLING  8
#define WM8731_REG_ACTIVE    9
#define WM8731_REG_RESET     15

/* WM8731 Settings */
#define AUDIO_INPUT_LINEIN 0
#define AUDIO_INPUT_MIC    1

#define I2S_USE_DMA 1

#define I2S_BUFFER_SIZE 8

bool WM8731_init(I2C_HandleTypeDef I2C_Handle);
bool WM8731_write(I2C_HandleTypeDef I2C_Handle, uint8_t reg, uint16_t val);
bool WM8731_volumeInteger(I2C_HandleTypeDef I2C_Handle, unsigned int n);
bool WM8731_inputLevel(I2C_HandleTypeDef I2C_Handle, float n);
bool WM8731_inputSelect(I2C_HandleTypeDef I2C_Handle, int n);
bool WM8731_volume(I2C_HandleTypeDef I2C_Handle, float n);
