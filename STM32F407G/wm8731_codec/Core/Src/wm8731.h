
#ifndef WM8731_H
#define WM8731_H

#include <stdint.h>

/* I2C clock speed configuration (Hz) */
#define I2C_SPEED 100000

/* Uncomment lines below to select stanard for audio communication
   between codec and I2S peripheral */
#define I2S_STANDARD_PHILLIPS
// #define I2S_STANDARD_MSB
// #define I2S_STANDARD_LSB

#ifdef I2S_STANDARD_PHILLIPS
 #define  CODEC_STANDARD                0x04
 #define I2S_STANDARD                   I2S_Standard_Phillips         
#elif defined(I2S_STANDARD_MSB)
 #define  CODEC_STANDARD                0x00
 #define I2S_STANDARD                   I2S_Standard_MSB    
#elif defined(I2S_STANDARD_LSB)
 #define  CODEC_STANDARD                0x08
 #define I2S_STANDARD                   I2S_Standard_LSB    
#else 
 #error "Error: No audio communication standard selected !"
#endif /* I2S_STANDARD */

/* I2S peripheral configuration defines */
#define CODEC_I2S                      SPI2
#define CODEC_I2S_EXT                  I2S2ext
#define CODEC_I2S_CLK                  RCC_APB1Periph_SPI2
#define CODEC_I2S_ADDRESS              0x4000380C
#define CODEC_I2S_EXT_ADDRESS          0x4000340C
#define CODEC_I2S_GPIO_AF              GPIO_AF_SPI2
#define CODEC_I2S_IRQ                  SPI2_IRQn
#define CODEC_I2S_EXT_IRQ              SPI2_IRQn
#define CODEC_I2S_GPIO_CLOCK           (RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB)
#define CODEC_I2S_WS_PIN               GPIO_Pin_12
#define CODEC_I2S_SCK_PIN              GPIO_Pin_13
#define CODEC_I2S_SDI_PIN              GPIO_Pin_14
#define CODEC_I2S_SDO_PIN              GPIO_Pin_15
#define CODEC_I2S_MCK_PIN              GPIO_Pin_6
#define CODEC_I2S_WS_PINSRC            GPIO_PinSource12
#define CODEC_I2S_SCK_PINSRC           GPIO_PinSource13
#define CODEC_I2S_SDI_PINSRC           GPIO_PinSource14
#define CODEC_I2S_SDO_PINSRC           GPIO_PinSource15
#define CODEC_I2S_MCK_PINSRC           GPIO_PinSource6
#define CODEC_I2S_GPIO                 GPIOB
#define CODEC_I2S_MCK_GPIO             GPIOC
#define AUDIO_I2S_IRQHandler           SPI2_IRQHandler
#define AUDIO_I2S_EXT_IRQHandler       SPI2_IRQHandler


#define AUDIO_MAL_DMA_PERIPH_DATA_SIZE DMA_PeripheralDataSize_HalfWord
#define AUDIO_MAL_DMA_MEM_DATA_SIZE    DMA_MemoryDataSize_HalfWord
#define DMA_MAX_SZE                    0xFFFF

/* I2S DMA Stream definitions */
#define AUDIO_I2S_DMA_CLOCK            RCC_AHB1Periph_DMA1
#define AUDIO_I2S_DMA_STREAM           DMA1_Stream4
#define AUDIO_I2S_DMA_DREG             CODEC_I2S_ADDRESS
#define AUDIO_I2S_DMA_CHANNEL          DMA_Channel_0
#define AUDIO_I2S_DMA_IRQ              DMA1_Stream4_IRQn
#define AUDIO_I2S_DMA_FLAG_TC          DMA_FLAG_TCIF4
#define AUDIO_I2S_DMA_FLAG_HT          DMA_FLAG_HTIF4
#define AUDIO_I2S_DMA_FLAG_FE          DMA_FLAG_FEIF4
#define AUDIO_I2S_DMA_FLAG_TE          DMA_FLAG_TEIF4
#define AUDIO_I2S_DMA_FLAG_DME         DMA_FLAG_DMEIF4
#define AUDIO_I2S_EXT_DMA_STREAM       DMA1_Stream3
#define AUDIO_I2S_EXT_DMA_DREG         CODEC_I2S_EXT_ADDRESS
#define AUDIO_I2S_EXT_DMA_CHANNEL      DMA_Channel_3
#define AUDIO_I2S_EXT_DMA_IRQ          DMA1_Stream3_IRQn
#define AUDIO_I2S_EXT_DMA_FLAG_TC      DMA_FLAG_TCIF3
#define AUDIO_I2S_EXT_DMA_FLAG_HT      DMA_FLAG_HTIF3
#define AUDIO_I2S_EXT_DMA_FLAG_FE      DMA_FLAG_FEIF3
#define AUDIO_I2S_EXT_DMA_FLAG_TE      DMA_FLAG_TEIF3
#define AUDIO_I2S_EXT_DMA_FLAG_DME     DMA_FLAG_DMEIF3

/* I2C peripheral configuration defines (control interface of the audio codec) */
#define CODEC_I2C                      I2C2
#define CODEC_I2C_CLK                  RCC_APB1Periph_I2C2
#define CODEC_I2C_GPIO_CLOCK           RCC_AHB1Periph_GPIOB
#define CODEC_I2C_GPIO_AF              GPIO_AF_I2C2
#define CODEC_I2C_GPIO                 GPIOB
#define CODEC_I2C_SCL_PIN              GPIO_Pin_10
#define CODEC_I2C_SDA_PIN              GPIO_Pin_11
#define CODEC_I2S_SCL_PINSRC           GPIO_PinSource10
#define CODEC_I2S_SDA_PINSRC           GPIO_PinSource11

#define CODEC_SAMPLERATE 48000
#define NYQUIST_FREQ (CODEC_SAMPLERATE / 2)

/* Mask for the bit EN of the I2S CFGR register */
#define I2S_ENABLE_MASK 0x0400

#define WM8731_ADDR_0   0x1A
#define WM8731_ADDR_0   0x1B
#define WM8731_NUM_REGS 10

/* The 7-bit Codec address (sent through I2C interface) */
#define CODEC_ADDRESS (WM8731_ADDR_0 << 1)

/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define CODEC_FLAG_TIMEOUT             ((uint32_t)0x1000)
#define CODEC_LONG_TIMEOUT             ((uint32_t)(300 * CODEC_FLAG_TIMEOUT))

/* Codec output device */
#define OUTPUT_DEVICE_SPEAKER         1
#define OUTPUT_DEVICE_HEADPHONE       2
#define OUTPUT_DEVICE_BOTH            3
#define OUTPUT_DEVICE_AUTO            4

const uint16_t wm8731_init_data[] = 
{
    0x017,  // Reg 00: Left Line In (0dB, mute off)
    0x017,  // Reg 01: Right Line In (0dB, mute off)
    0x079,  // Reg 02: Left Headphone out (0dB)
    0x079,  // Reg 03: Right Headphone out (0dB)
    0x012,  // Reg 04: Analog Audio Path Control (DAC sel, Mute Mic)
    0x000,  // Reg 05: Digital Audio Path Control
    0x062,  // Reg 06: Power Down Control (Clkout, Osc, Mic Off)
//  0x00E,  // Reg 07: Digital Audio Interface Format (i2s, 32-bit, slave)
    0x002,  // Reg 07: Digital Audio Interface Format (i2s, 16-bit, slave)
    0x000,  // Reg 08: Sampling Control (Normal, 256x, 48k ADC/DAC)
    0x001   // Reg 09: Active Control
};

uint32_t Codec_Init(uint32_t sample_rate);

void     Codec_CtrlInterface_Init(void);
void     Codec_AudioInterface_Init(uint32_t audio_freq);
void     Codec_Reset(void);
uint32_t Codec_WriteRegister(uint8_t register_addr, uint16_t register_value);
void     Codec_GPIO_Init(void);

#endif /* WM8731_H */