#include "SPO2.h"
extern AFE44XX afe44xx;
#define AFE44XX_SPI_SPEED_AFE49XX 2000000
SPISettings SPI_SETTINGS_SPO2(AFE44XX_SPI_SPEED_AFE49XX, MSBFIRST, SPI_MODE0); 

void afe44xx_reset()
{
    SPI.beginTransaction(SPI_SETTINGS_SPO2);
    digitalWrite(AFE44XX_CS, LOW);        // enable device
    SPI.transfer(CONTROL0);             // send address to device
    SPI.transfer((0x000004 >> 16) & 0xFF); // write top 8 bits
    SPI.transfer((0x000004 >> 8) & 0xFF);  // write middle 8 bits
    SPI.transfer(0x000004 & 0xFF);         // write bottom 8 bits
    digitalWrite(AFE44XX_CS, HIGH);       // disable device
    SPI.endTransaction();
}