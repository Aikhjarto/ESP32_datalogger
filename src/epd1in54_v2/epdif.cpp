/**
 *  @filename   :   epdif.cpp
 *  @brief      :   Implements EPD interface functions
 *                  Users have to implement all the functions in epdif.cpp
 *  @author     :   Yehui from Waveshare
 *
 *  Copyright (C) Waveshare     August 10 2017
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "epdif.h"
#include <SPI.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>

EpdIf::EpdIf() {
    spi_clk_pin=-1;
    spi_MISO_pin=-1;
    spi_MOSI_pin=-1;
    spi_cs_pin=-1;
}

EpdIf::EpdIf(int8_t spi_clk, int8_t spi_miso, int8_t spi_mosi, int8_t spi_cs) {
    AssignSPIPins(spi_clk, spi_miso, spi_mosi, spi_cs);

};

void EpdIf::AssignSPIPins(int8_t spi_clk, int8_t spi_miso, int8_t spi_mosi, int8_t spi_cs) {
    spi_clk_pin=spi_clk;
    spi_MISO_pin=spi_miso;
    spi_MOSI_pin=spi_mosi;
    spi_cs_pin=spi_cs;
}


EpdIf::~EpdIf() {
};

void EpdIf::DigitalWrite(int pin, int value) {
    digitalWrite(pin, value);
}

int EpdIf::DigitalRead(int pin) {
    pinMode(pin, INPUT);
    return digitalRead(pin);
}

void EpdIf::DelayMs(unsigned int delaytime) {
    delay(delaytime);
}

void EpdIf::SpiTransfer(unsigned char data) {
    digitalWrite(spi_cs_pin, LOW);
    SPI.transfer(data);
    digitalWrite(spi_cs_pin, HIGH);
}

void EpdIf::SpiTransfer(unsigned char * data, uint32_t size) {
    digitalWrite(spi_cs_pin, LOW);
    SPI.transfer(data, size);
    digitalWrite(spi_cs_pin, HIGH);
}

int EpdIf::IfInit(void) {
    pinMode(spi_cs_pin, OUTPUT);

    SPI.end(); // release standard SPI pins
    // Serial.printf("SPI.begin(%d, %d, %d, %d); // (SCK, MISO, MOSI, SS)\n", 
    //               spi_clk_pin, spi_MISO_pin, spi_MOSI_pin, spi_cs_pin);
    SPI.begin(spi_clk_pin, spi_MISO_pin, spi_MOSI_pin, spi_cs_pin); // map and init SPI pins SCK(13), MISO(12), MOSI(14), SS(15)
    SPI.beginTransaction(SPISettings(20000000, SPI_MSBFIRST, SPI_MODE0));
    return 0;
}
