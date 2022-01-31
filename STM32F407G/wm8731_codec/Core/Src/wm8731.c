#include "wm8731.h"

__IO uint32_t  codec_timeout = CODEC_LONG_TIMEOUT;   
__IO uint8_t output_dev = 0;

I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s2;

uint32_t Codec_Timeout_UserCallback(void)
{
    /* do nothing */
    return 0;
}

uint32_t Codec_Init(uint32_t audio_freq)
{
    uint32_t counter = 0;

    /* Configure the Codec IOs */
    Codec_GPIO_Init();

    /* Initialise the Codec control interface */
    Codec_CtrlInterface_Init();

    /* Configure the I2S peripheral */
    Codec_AudioInterface_Init(audio_freq);

    /* Reset the Codec registers */
    Codec_Reset();

    /* Return comminication control value */
    return counter;
}

/**
  * @brief  Resets the audio codec. It restores the default configuration of the 
  *         codec (this function should be called before initializing the codec)
  * @note   This function calls an external driver function: The IO Expander driver.
  * @param  None
  * @retval None
  */
void Codec_Reset()
{

    Codec_WriteRegister(0x0f, 0);

    /* Load default values */
    for (uint8_t i = 0; i < WM8731_NUM_REGS; i++)
    {
        Codec_WriteRegister(i, w8731_init_data[i]);
    }
}

/**
  * @brief  Writes a Byte to a given register into the audio codec through the 
            control interface (I2C)
  * @param  register_addr: The address (location) of the register to be written.
  * @param  register_value: the Byte value to be written into destination register.
  * @retval 0 if write successful, else failed
  */
uint32_t Codec_WriteRegister(uint8_t register_addr, uint16_t register_value)
{
    uint32_t result = 0;

    /* Assemble 2-byte data in WM8731 format */
    uint8_t byte_1 = ((register_addr << 1) & 0xFE) | ((register_value >> 8) & 0x01);
    uint8_t byte_2 = register_value & 0xFF;

    /* Wait while I2C bus is busy */
    codec_timeout = CODEC_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BUSY))
    {
        if((codec_timeout--) == 0)
            return Codec_Timeout_UserCallback();
    }

    /* Start the config sequence */
    I2C_GenerateStart(CODEC_I2C, ENABLE);

    /* Test on EV5 and clear it */
    codec_timeout = CODEC_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((codec_timeout--) == 0)
            return Codec_Timeout_UserCallback();
    }

    /* Transmit the slave address and enable writing operation */
    I2C_Send7bitAddress(CODEC_I2C, CODEC_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    codec_timeout = CODEC_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if((codec_timeout--) == 0)
            return Codec_Timeout_UserCallback();
    }

    /* Transmit the first addess for write operation */
    I2C_SendData(CODEC_I2C, byte1);

    /* Test on EV8 and clear it */
    codec_timeout = CODEC_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(CODEC_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
    {
        if((codec_timeout--) == 0)
            return Codec_Timeout_UserCallback();
    }

    /* Prepare the register value to be sent */
    I2C_SendData(CODEC_I2C, byte2);

    /*!< Wait till all data has been physically transferred on the bus */
    codec_timeout = CODEC_LONG_TIMEOUT;
    while(!I2C_GetFlagStatus(CODEC_I2C, I2C_FLAG_BTF))
    {
        if((codec_timeout--) == 0)
            return Codec_TIMEOUT_UserCallback();
    }

    /* End the configuration sequence */
    I2C_GenerateStop(CODEC_I2C, ENABLE);

    return result;

}

/**
  * @brief  Initializes the Audio Codec control interface (I2C).
  *         Using STM32 HAL.
  * @param  None
  * @retval None
  */
void Codec_CtrlInterface_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = I2C_SPEED;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0x33;
    // ACK enable?
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Initializes the Audio Codec audio interface (I2S)
  * @note   This function assumes that the I2S input clock (through PLL_R in 
  *         Devices RevA/Z and through dedicated PLLI2S_R in Devices RevB/Y)
  *         is already configured and ready to be used.    
  * @param  AudioFreq: Audio frequency to be configured for the I2S peripheral. 
  * @retval None
  */
void Codec_AudioInterface_Init(uint32_t audio_freq)
{
    hi2s3.Instance = SPI3;
    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;

    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK)
    {
        Error_Handler();
    }

    /* Initialize the I2S extended channel for RX */
    //I2S_FullDuplexConfig(CODEC_I2S_EXT, &I2S_InitStructure);??

    /* The I2S peripheral will be enabled only in the EVAL_AUDIO_Play() function 
    or by user functions if DMA mode not enabled */  
}

/**
  * @brief Initialize IOs used by the Audio Codec (on the control and audio 
  *        interfaces).
  * @param  None
  * @retval None
  */
void Codec_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable I2S and I2C GPIO clocks */
    RCC_AHB1PeriphClockCmd(CODEC_I2C_GPIO_CLOCK | CODEC_I2S_GPIO_CLOCK, ENABLE);

    /* CODEC_I2C SCL and SDA pins configuration ------------------------------*/
    GPIO_InitStructure.GPIO_Pin = CODEC_I2C_SCL_PIN | CODEC_I2C_SDA_PIN; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(CODEC_I2C_GPIO, &GPIO_InitStructure);

    /* Connect pins to I2C peripheral */
    GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2S_SCL_PINSRC, CODEC_I2C_GPIO_AF);  
    GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2S_SDA_PINSRC, CODEC_I2C_GPIO_AF);  

    /* CODEC_I2S output pins configuration: WS, SCK SD0 and SDI pins ---------*/
    GPIO_InitStructure.GPIO_Pin = CODEC_I2S_SCK_PIN | CODEC_I2S_SDO_PIN | CODEC_I2S_SDI_PIN | CODEC_I2S_WS_PIN; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(CODEC_I2S_GPIO, &GPIO_InitStructure);

    /* CODEC_I2S pins configuration: MCK pin */
    GPIO_InitStructure.GPIO_Pin = CODEC_I2S_MCK_PIN; 
    GPIO_Init(CODEC_I2S_MCK_GPIO, &GPIO_InitStructure);

    /* Connect pins to I2S peripheral  */
    GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_WS_PINSRC, CODEC_I2S_GPIO_AF);  
    GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_SCK_PINSRC, CODEC_I2S_GPIO_AF);
    GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_SDO_PINSRC, CODEC_I2S_GPIO_AF);
    GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_SDI_PINSRC, CODEC_I2S_GPIO_AF);
    GPIO_PinAFConfig(CODEC_I2S_MCK_GPIO, CODEC_I2S_MCK_PINSRC, CODEC_I2S_GPIO_AF); 
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                            |Audio_RST_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : CS_I2C_SPI_Pin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PDM_OUT_Pin */
    GPIO_InitStruct.Pin = PDM_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BOOT1_Pin */
    GPIO_InitStruct.Pin = BOOT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : CLK_IN_Pin */
    GPIO_InitStruct.Pin = CLK_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                            Audio_RST_Pin */
    GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                            |Audio_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
    GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}