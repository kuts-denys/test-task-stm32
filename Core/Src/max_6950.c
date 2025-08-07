#include "max_6950.h"

extern SPI_HandleTypeDef hspi1;

#define MAX6950_REG_DECODE_MODE 0x01
#define MAX6950_REG_INTENSITY 0x02
#define MAX6950_REG_SCAN_LIMIT 0x03
#define MAX6950_REG_CONFIG 0x04

#define MAX6950_REG_DIGIT0 0x60
#define MAX6950_REG_DIGIT1 0x61
#define MAX6950_REG_DIGIT2 0x62
#define MAX6950_REG_DIGIT3 0x63
#define MAX6950_REG_DIGIT4 0x64

static void MAX6950_WriteRegister(uint8_t reg_addr, uint8_t data) {
  uint8_t tx_buffer[2] = {reg_addr, data};

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

void MAX6950_Init(void) {
  // display all 5 digits
  MAX6950_WriteRegister(MAX6950_REG_SCAN_LIMIT, 0x04);
  // enable decoding for 0-1 and 3-4 digits, the second one will be set manually
  // to hyphen
  MAX6950_WriteRegister(MAX6950_REG_DECODE_MODE, 0x1B); // 0b00011011
  // set 8/16 duty cycle (medium) brightness
  MAX6950_WriteRegister(MAX6950_REG_INTENSITY, 0x07);
  // enable normal mode
  MAX6950_WriteRegister(MAX6950_REG_CONFIG, 0x01);
}

void MAX6950_UpdateDisplay(uint8_t count) {
  uint8_t minutes = count / 60;
  uint8_t seconds = count % 60;

  // always 0 in our case
  uint8_t digit4 = 0;
  uint8_t digit3 = minutes; // either 0 or 1

  uint8_t digit1 = seconds / 10;
  uint8_t digit0 = seconds % 10;

  // write digits to the display
  MAX6950_WriteRegister(MAX6950_REG_DIGIT4, digit4);
  MAX6950_WriteRegister(MAX6950_REG_DIGIT3, digit3);
  MAX6950_WriteRegister(MAX6950_REG_DIGIT2,
                        0x01); // activate only g-segment (central hyphen)
  MAX6950_WriteRegister(MAX6950_REG_DIGIT1, digit1);
  MAX6950_WriteRegister(MAX6950_REG_DIGIT0, digit0);
}
