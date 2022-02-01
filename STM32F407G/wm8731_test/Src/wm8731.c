#include "wm8731.h"

bool WM8731_init(I2C_HandleTypeDef I2C_Handle)
{
	/* I2S configuration */
	HAL_Delay(5);
	WM8731_write(I2C_Handle, WM8731_REG_RESET, 0);

	WM8731_write(I2C_Handle, WM8731_REG_INTERFACE, 0b00000010); // 0x02=0b00000010 // I2S, 16 bit, MCLK slave
	WM8731_write(I2C_Handle, WM8731_REG_SAMPLING,  0b00100000); // 0x20=0b00100000  // 256*Fs, 44.1 kHz, MCLK/1

	WM8731_write(I2C_Handle, WM8731_REG_DIGITAL, 0x08); // DAC soft mute
	WM8731_write(I2C_Handle, WM8731_REG_ANALOG, 0x00);  // disable all

	WM8731_write(I2C_Handle, WM8731_REG_POWERDOWN, 0x00);

	WM8731_write(I2C_Handle, WM8731_REG_LHEADOUT, 0x80);
	WM8731_write(I2C_Handle, WM8731_REG_RHEADOUT, 0x80);

	HAL_Delay(5);
	WM8731_write(I2C_Handle, WM8731_REG_ACTIVE, 1);
	HAL_Delay(5);

	WM8731_write(I2C_Handle, WM8731_REG_DIGITAL, 0b00100);   // DAC unmuted
	WM8731_write(I2C_Handle, WM8731_REG_ANALOG, 0b00010000); // DAC selected

	WM8731_volume(I2C_Handle, 0.5);
	return true;
}

bool WM8731_write(I2C_HandleTypeDef I2C_Handle, uint8_t reg, uint16_t val)
{

	const static uint8_t TRANSMIT_LENGTH = 2;
	uint8_t data[TRANSMIT_LENGTH];

	data[0] = (reg << 1) | ((val >> 8) & 1);
	data[1] = val & 0xFF;

	//bool result = I2C_TX_write(WM8731_I2C_ADDR, data, TRANSMIT_LENGTH);
	bool result = HAL_I2C_Master_Transmit(&I2C_Handle, WM8731_I2C_ADDR, data, TRANSMIT_LENGTH, HAL_MAX_DELAY);
	return result;
}

bool WM8731_volumeInteger(I2C_HandleTypeDef I2C_Handle, unsigned int n)
{
	// n = 127 for max volume (+6 dB)
	// n = 48 for min volume (-73 dB)
	// n = 0 to 47 for mute
	if (n > 127)
		n = 127;

	WM8731_write(I2C_Handle, WM8731_REG_LHEADOUT, n | 0x180);
	WM8731_write(I2C_Handle, WM8731_REG_RHEADOUT, n | 0x80);
	return true;
}

bool WM8731_inputLevel(I2C_HandleTypeDef I2C_Handle, float n)
{
	// range is 0x00 (min) - 0x1F (max)

	int _level = (int) (n * 31.f);

	_level = _level > 0x1F ? 0x1F : _level;
	WM8731_write(I2C_Handle, WM8731_REG_LLINEIN, _level);
	WM8731_write(I2C_Handle, WM8731_REG_RLINEIN, _level);
	return true;
}

bool WM8731_inputSelect(I2C_HandleTypeDef I2C_Handle, int n)
{
	if (n == AUDIO_INPUT_LINEIN)
	{
		WM8731_write(I2C_Handle, WM8731_REG_ANALOG, 0x12);
	}
	else if (n == AUDIO_INPUT_MIC)
	{
		WM8731_write(I2C_Handle, WM8731_REG_ANALOG, 0x15);
	}
	else
	{
		return false;
	}
	return true;
}

bool WM8731_volume(I2C_HandleTypeDef I2C_Handle, float n)
{
	return WM8731_volumeInteger(I2C_Handle, n * 80.0 + 47.499);
}


