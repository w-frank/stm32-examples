#include "eeprom.h"

// Define the I2C port
extern I2C_HandleTypeDef hi2cl;
#define EEPROM_I2C &hi2cl

// EEPROM I2C address (8-bits)
#define EEPROM_I2C_ADDR 0xA0

#define EEPROM_PAGE_SIZE 64 // page size in bytes
#define EEPROM_PAGE_NUM 512 // number of pages

uint16_t bytes_to_write(uint16_t size, uint16_t offset)
{
    if (size + offset < EEPROM_PAGE_SIZE) return size;
    else return EEPROM_PAGE_SIZE - offset;
}

void EEPROM_Write(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
    // Calculate page start address bit location
    int page_addr = log(EEPROM_PAGE_SIZE)/log(2);

    // Calculate the start and end pages
    uint16_t start_page = page;
    uint16_t end_page = page + ((size + offset)/EEPROM_PAGE_SIZE);

    // Calculate the number of pages to be written
    uint16_t num_pages = (end_page - start_page) + 1;
    uint16_t position = 0;

    // Write the data
    for(int i = 0; i < num_pages; i++)
    {
        uint16_t mem_addr = start_page << page_addr | offset;
        uint16_t bytes_remaining = bytes_to_write(size, offset);

        HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_I2C_ADDR, mem_addr, 2, 
                            &data[position], bytes_remaining, 1000);

        start_page += 1;
        offset = 0;
        size = size - bytes_remaining;
        position += bytes_remaining;

        HAL_Delay(5); // Write cycle delay (5 ms)

    }
}

void EEPROM_Read(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
    // Calculate page start address bit location
    int page_addr = log(EEPROM_PAGE_SIZE)/log(2);

    // Calculate the start and end pages
    uint16_t start_page = page;
    uint16_t end_page = page + ((size + offset)/EEPROM_PAGE_SIZE);

    // Calculate the number of pages to be written
    uint16_t num_pages = (end_page - start_page) + 1;
    uint16_t position = 0;

    // Read the data
    for(int i = 0; i < num_pages; i++)
    {
        uint16_t mem_addr = start_page << page_addr | offset;
        uint16_t bytes_remaining = bytes_to_write(size, offset);

        HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_I2C_ADDR, mem_addr, 2, 
                            &data[position], bytes_remaining, 1000);

        start_page += 1;
        offset = 0;
        size = size - bytes_remaining;
        position += bytes_remaining;

    }

}

EEPROM_PageErase(uint16_t page)
{
    // Calculate the memory address based on the page number
    int page_addr = log(EEPROM_PAGE_SIZE)/log(2);
    uint16_t mem_addr = start_page << page_addr;

    // Create a buffer to store the reset values
    uint8_t data[EEPROM_PAGE_SIZE];
    memset(data, 0xFF, EEPROM_PAGE_SIZE);

    // Write the reset data to the EEPROM
    HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_I2C_ADDR, mem_addr, 2, data, 
                        EEPROM_PAGE_SIZE, 1000);

    HAL_Delay(5); // Write cycle delay (5 ms)
}