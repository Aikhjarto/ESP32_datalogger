/*****************************************************************************
* | File      	:   epd1in54_V2.cpp
* | Author      :   Waveshare team
* | Function    :   1.54inch e-paper V2
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2019-06-24
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include <stdlib.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <pgmspace.h>
#include "epd1in54_V2.h"

// // EPD1IN54 commands
#define DRIVER_OUTPUT_CONTROL                       0x01
#define BOOSTER_SOFT_START_CONTROL                  0x0C
#define GATE_SCAN_START_POSITION                    0x0F
#define DEEP_SLEEP_MODE                             0x10
#define DATA_ENTRY_MODE_SETTING                     0x11
#define SW_RESET                                    0x12
#define TEMPERATURE_SENSOR_CONTROL                  0x1A
#define MASTER_ACTIVATION                           0x20
#define DISPLAY_UPDATE_CONTROL_1                    0x21
#define DISPLAY_UPDATE_CONTROL_2                    0x22
#define WRITE_RAM                                   0x24
#define WRITE_RAM_RED                               0x26
#define WRITE_VCOM_REGISTER                         0x2C
#define WRITE_LUT_REGISTER                          0x32
#define SET_DUMMY_LINE_PERIOD                       0x3A
#define SET_GATE_TIME                               0x3B
#define BORDER_WAVEFORM_CONTROL                     0x3C
#define SET_RAM_X_ADDRESS_START_END_POSITION        0x44
#define SET_RAM_Y_ADDRESS_START_END_POSITION        0x45
#define SET_RAM_X_ADDRESS_COUNTER                   0x4E
#define SET_RAM_Y_ADDRESS_COUNTER                   0x4F
#define TERMINATE_FRAME_READ_WRITE                  0xFF


// waveform full refresh
unsigned char WF_Full_1IN54[159] =
{											
0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x80,	0x48,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x40,	0x48,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x8,	0x1,	0x0,	0x8,	0x1,	0x0,	0x2,					
0xA,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
0x22,	0x22,	0x22,	0x22,	0x22,	0x22,	0x0,	0x0,	0x0,			
0x22,	0x17,	0x41,	0x0,	0x32,	0x20
};

// waveform partial refresh(fast)
unsigned char WF_PARTIAL_1IN54_0[159] =
{
0x0,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x80,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x40,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0xF,0x0,0x0,0x0,0x0,0x0,0x0,
0x1,0x1,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x22,0x22,0x22,0x22,0x22,0x22,0x0,0x0,0x0,
0x02,0x17,0x41,0xB0,0x32,0x28,
};

Epd::~Epd()
{
};

Epd::Epd() {
	width = EPD_WIDTH;
	height = EPD_HEIGHT;
}

Epd::Epd(unsigned int reset, unsigned int dc, unsigned int busy,
		uint8_t spi_clk, uint8_t spi_miso, uint8_t spi_mosi, uint8_t spi_cs)
  : EpdIf(spi_clk, spi_miso, spi_mosi, spi_cs) {

	AssignControlPins(reset, dc, busy);

	width = EPD_WIDTH;
	height = EPD_HEIGHT;
};

void Epd::AssignControlPins(unsigned int reset, unsigned int dc, unsigned int busy){
	reset_pin = reset;
	dc_pin = dc;
	busy_pin = busy;

    pinMode(reset, OUTPUT);
    pinMode(dc_pin, OUTPUT);
    pinMode(busy_pin, INPUT); 
	
}

/**
 *  @brief: basic function for sending commands
 */
void Epd::SendCommand(unsigned char command)
{
	DigitalWrite(dc_pin, LOW);
	SpiTransfer(command);
}

/**
 *  @brief: basic function for sending data
 */
void Epd::SendData(unsigned char data)
{
	DigitalWrite(dc_pin, HIGH);
	SpiTransfer(data);
}

void Epd::SendData(unsigned char * data, int32_t size)
{
	DigitalWrite(dc_pin, HIGH);
	SpiTransfer(data, size);
}


/**
 *  @brief: Wait until the busy_pin goes HIGH
 */
void Epd::WaitUntilIdle(void)
{
	while(DigitalRead(busy_pin) == 1) {      //LOW: idle, HIGH: busy
		// DelayMs(10);
		vTaskDelay(10*portTICK_PERIOD_MS);
	}
}

void Epd::Lut(unsigned char* lut)
{
	SendCommand(WRITE_LUT_REGISTER);
	for(unsigned char i=0; i<153; i++)
		SendData(lut[i]);
	// SendData(lut, 153);
	WaitUntilIdle();
}

void Epd::SetLut(unsigned char* lut)
{
	Lut(lut);
	
    SendCommand(0x3f);
    SendData(lut[153]);
	
    SendCommand(0x03);
    SendData(lut[154]);
	
    SendCommand(0x04);
    SendData(lut[155]);
	SendData(lut[156]);
	SendData(lut[157]);
	
	SendCommand(WRITE_VCOM_REGISTER);
    SendData(lut[158]);
}

// High Direction
int Epd::HDirInit(void)
{
	/* this calls the peripheral hardware interface, see epdif */
	if (IfInit() != 0) {
		return -1;
	}
	/* EPD hardware init start */
	Reset();

	WaitUntilIdle();
	SendCommand(SW_RESET);  //SWRESET
	WaitUntilIdle();

	SendCommand(DRIVER_OUTPUT_CONTROL); //Driver output control
	SendData(0xC7);
	SendData(0x00);
	SendData(0x01);

	SendCommand(DATA_ENTRY_MODE_SETTING); //data entry mode
	SendData(0x01);

	SendCommand(SET_RAM_X_ADDRESS_START_END_POSITION); //set Ram-X address start/end position
	SendData(0x00);
	SendData(0x18);    //0x0C-->(18+1)*8=200

	SendCommand(SET_RAM_Y_ADDRESS_START_END_POSITION); //set Ram-Y address start/end position
	SendData(0xC7);   //0xC7-->(199+1)=200
	SendData(0x00);
	SendData(0x00);
	SendData(0x00);

	SendCommand(BORDER_WAVEFORM_CONTROL); //BorderWavefrom
	SendData(0x01);

	SendCommand(0x18);
	SendData(0x80);

	SendCommand(DISPLAY_UPDATE_CONTROL_2); // //Load Temperature and waveform setting.
	SendData(0xB1);
	SendCommand(MASTER_ACTIVATION);

	SendCommand(SET_RAM_X_ADDRESS_COUNTER);   // set RAM x address count to 0;
	SendData(0x00);
	SendCommand(SET_RAM_Y_ADDRESS_COUNTER);   // set RAM y address count to 0X199;
	SendData(0xC7);
	SendData(0x00);
	WaitUntilIdle();

	SetLut(WF_Full_1IN54);
	/* EPD hardware init end */

	return 0;
}

// Low Direction
int Epd::LDirInit(void)
{
	/* this calls the peripheral hardware interface, see epdif */
	if (IfInit() != 0) {
		return -1;
	}
	/* EPD hardware init start */
	Reset();

	WaitUntilIdle();
	SendCommand(SW_RESET);  //SWRESET
	WaitUntilIdle();

	SendCommand(DRIVER_OUTPUT_CONTROL); //Driver output control
	SendData(0xC7);
	SendData(0x00);
	SendData(0x00);

	SendCommand(DATA_ENTRY_MODE_SETTING); //data entry mode
	SendData(0x03);

	SendCommand(SET_RAM_X_ADDRESS_START_END_POSITION);
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	SendData((0 >> 3) & 0xFF);
	SendData((199 >> 3) & 0xFF);
	SendCommand(SET_RAM_Y_ADDRESS_START_END_POSITION);
	SendData(0 & 0xFF);
	SendData((0 >> 8) & 0xFF);
	SendData(199 & 0xFF);
	SendData((199 >> 8) & 0xFF);

	SendCommand(BORDER_WAVEFORM_CONTROL); //BorderWavefrom
	SendData(0x01);

	SendCommand(0x18);
	SendData(0x80);

	SendCommand(DISPLAY_UPDATE_CONTROL_2); // //Load Temperature and waveform setting.
	SendData(0XB1);
	SendCommand(MASTER_ACTIVATION);

	SendCommand(SET_RAM_X_ADDRESS_COUNTER);   // set RAM x address count to 0;
	SendData(0x00);
	SendCommand(SET_RAM_Y_ADDRESS_COUNTER);   // set RAM y address count to 0X199;
	SendData(0xC7);
	SendData(0x00);
	WaitUntilIdle();

	SetLut(WF_Full_1IN54);
	/* EPD hardware init end */

	return 0;
}


/**
 *  @brief: module reset.
 *          often used to awaken the module in deep sleep,
 *          see Epd::Sleep();
 */
void Epd::Reset(void)
{
	// Serial.printf("ePaper control pins: BUSY %d, RST %d, DC %d\n", busy_pin, reset_pin, dc_pin);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(20);
	DigitalWrite(reset_pin, LOW);                //module reset
	DelayMs(5);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(20);
}

void Epd::Clear(void)
{
	int w, h;
	w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	h = EPD_HEIGHT;
 
	SendCommand(WRITE_RAM);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}
	SendCommand(WRITE_RAM_RED);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}
	//DISPLAY REFRESH
	DisplayFrame();
}

void Epd::Display(const unsigned char* frame_buffer)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	if (frame_buffer != NULL) {
		SendCommand(WRITE_RAM);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}
	}

	//DISPLAY REFRESH
	DisplayFrame();
}

void Epd::DisplayPartBaseImage(const unsigned char* frame_buffer)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	if (frame_buffer != NULL) {
		SendCommand(WRITE_RAM);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}

		SendCommand(WRITE_RAM_RED);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}
	}

	//DISPLAY REFRESH
	DisplayFrame();
}

void Epd::DisplayPartBaseImageNoRefresh(const unsigned char* frame_buffer)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	if (frame_buffer != NULL) {
		SendCommand(WRITE_RAM);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}

		SendCommand(WRITE_RAM_RED);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}
	}
}

void Epd::DisplayPartBaseWhiteImage(void)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	SendCommand(WRITE_RAM);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}

	SendCommand(WRITE_RAM_RED);
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			SendData(0xff);
		}
	}


	//DISPLAY REFRESH
	DisplayFrame();
}


void Epd::DisplayPart(const unsigned char* frame_buffer)
{
	int w = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	int h = EPD_HEIGHT;

	if (frame_buffer != NULL) {
		SendCommand(WRITE_RAM);
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				SendData(pgm_read_byte(&frame_buffer[i + j * w]));
			}
		}
	}

	//DISPLAY REFRESH
	DisplayPartFrame();
}


/**
 *  @brief: private function to specify the memory area for data R/W
 */
void Epd::SetMemoryArea(int x_start, int y_start, int x_end, int y_end)
{
	SendCommand(SET_RAM_X_ADDRESS_START_END_POSITION);
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	SendData((x_start >> 3) & 0xFF);
	SendData((x_end >> 3) & 0xFF);
	SendCommand(SET_RAM_Y_ADDRESS_START_END_POSITION);
	SendData(y_start & 0xFF);
	SendData((y_start >> 8) & 0xFF);
	SendData(y_end & 0xFF);
	SendData((y_end >> 8) & 0xFF);
}

/**
 *  @brief: private function to specify the start point for data R/W
 */
void Epd::SetMemoryPointer(int x, int y)
{
	SendCommand(SET_RAM_X_ADDRESS_COUNTER);
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	SendData((x >> 3) & 0xFF);
	SendCommand(SET_RAM_Y_ADDRESS_COUNTER);
	SendData(y & 0xFF);
	SendData((y >> 8) & 0xFF);
	WaitUntilIdle();
}


/**
 *  @brief: update the display
 *          there are 2 memory areas embedded in the e-paper display
 *          but once this function is called,
 *          the the next action of SetFrameMemory or ClearFrame will
 *          set the other memory area.
 */
void Epd::DisplayFrame(void)
{
	//DISPLAY REFRESH
	SendCommand(DISPLAY_UPDATE_CONTROL_2);
	SendData(0xc7);
	SendCommand(MASTER_ACTIVATION);
	WaitUntilIdle();
}

void Epd::DisplayPartFrame(void)
{
	SendCommand(DISPLAY_UPDATE_CONTROL_2);
	SendData(0xcF);
	SendCommand(MASTER_ACTIVATION);
	WaitUntilIdle();
}

void Epd::SetFrameMemory(
        const unsigned char* image_buffer,
        int x,
        int y,
        int image_width,
        int image_height
)
{
	int x_end;
	int y_end;
	
	DigitalWrite(reset_pin, LOW);                //module reset
	DelayMs(2);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(2);
	SendCommand(BORDER_WAVEFORM_CONTROL);
	SendData(0x80);

	if (
	        image_buffer == NULL ||
	        x < 0 || image_width < 0 ||
	        y < 0 || image_height < 0
	) {
		return;
	}
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	x &= 0xF8;
	image_width &= 0xF8;
	if (x + image_width >= this->width) {
		x_end = this->width - 1;
	} else {
		x_end = x + image_width - 1;
	}
	if (y + image_height >= this->height) {
		y_end = this->height - 1;
	} else {
		y_end = y + image_height - 1;
	}
	SetMemoryArea(x, y, x_end, y_end);
	SetMemoryPointer(x, y);
	SendCommand(WRITE_RAM);
	/* send the image data */
	for (int j = 0; j < y_end - y + 1; j++) {
		for (int i = 0; i < (x_end - x + 1) / 8; i++) {
			SendData(image_buffer[i + j * (image_width / 8)]);
		}
	}
}

void Epd::SetFrameMemoryPartial(
        const unsigned char* image_buffer,
        int x,
        int y,
        int image_width,
        int image_height
)
{
	int x_end;
	int y_end;
	
	DigitalWrite(reset_pin, LOW);                //module reset
	DelayMs(2);
	DigitalWrite(reset_pin, HIGH);
	DelayMs(2);

	SetLut(WF_PARTIAL_1IN54_0);
    SendCommand(0x37); 
    SendData(0x00);  
    SendData(0x00);  
    SendData(0x00);  
    SendData(0x00); 
    SendData(0x00);  	
    SendData(0x40);  
    SendData(0x00);  
    SendData(0x00);   
    SendData(0x00);  
    SendData(0x00);

	SendCommand(BORDER_WAVEFORM_CONTROL);
	SendData(0x80);

	SendCommand(DISPLAY_UPDATE_CONTROL_2); 
	SendData(0xc0); 
	SendCommand(MASTER_ACTIVATION); 
	WaitUntilIdle();
	
	if (
	        image_buffer == NULL ||
	        x < 0 || image_width < 0 ||
	        y < 0 || image_height < 0
	) {
		return;
	}
	/* x point must be the multiple of 8 or the last 3 bits will be ignored */
	x &= 0xF8;
	image_width &= 0xF8;
	if (x + image_width >= this->width) {
		x_end = this->width - 1;
	} else {
		x_end = x + image_width - 1;
	}
	if (y + image_height >= this->height) {
		y_end = this->height - 1;
	} else {
		y_end = y + image_height - 1;
	}
	SetMemoryArea(x, y, x_end, y_end);
	SetMemoryPointer(x, y);
	SendCommand(WRITE_RAM);
	/* send the image data */
	for (int j = 0; j < y_end - y + 1; j++) {
		for (int i = 0; i < (x_end - x + 1) / 8; i++) {
			SendData(image_buffer[i + j * (image_width / 8)]);
		}
	}
}


void Epd::SetFrameMemory(
        const unsigned char* image_buffer,
        int x,
        int y,
        int image_width,
        int image_height,
		char use_partial_update
){
	if (use_partial_update) {
		SetFrameMemoryPartial(image_buffer, x, y, image_width, image_height);
	} else {
		SetFrameMemory(image_buffer, x, y, image_width, image_height);
	}
}

/**
 *  @brief: After this command is transmitted, the chip would enter the
 *          deep-sleep mode to save power.
 *          The deep sleep mode would return to standby by hardware reset.
 *          The only one parameter is a check code, the command would be
 *          executed if check code = 0xA5.
 *          You can use Epd::Init() to awaken
 */
void Epd::Sleep()
{
	SendCommand(DEEP_SLEEP_MODE); //enter deep sleep
	SendData(0x01);
	DelayMs(200);

	DigitalWrite(reset_pin, LOW);
}

/* END OF FILE */
