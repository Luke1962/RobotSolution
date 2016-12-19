#pragma once
#ifndef __HWINCLUDES__
#define __HWINCLUDES__

#include <TinyGPSplus\TinyGPS++.h>	//se manca non compila a causa del robotModel.cpp nella stessa cartella di robot\Commands_Enum.h

// TFT TYPE                Uncomment one of the following----------
#define TFT_ILI9327_8	//x:320 X y:480 con driver 	
//#define TFT_ili9341
//#define TFT_ili9488	

#ifdef TFT_ILI9327_8 // TFT 320x480
	#pragma region TFT
	#define TFT_X_WIDTH 320
	#define TFT_Y_HEIGHT 480
	////UTFT     tft(ILI9327_8,30,31,32,33 );		//was 38, 39, 40, 41
	//UTFT     tft(ILI9327_8, Pin_TFT_RS, Pin_TFT_WR, Pin_TFT_CS, Pin_TFT_RST);		//was 38, 39, 40, 41

																					//extern uint8_t BigFont[];
	extern uint8_t SmallFont[];


	#pragma endregion

	//#pragma region touch screen
	//	#include <XPT2046/XPT2046.h>

	//	#define CS_PIN  53	///was 9
	//	#define TIRQ_PIN  2
	//	XPT2046 ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
	//#pragma endregion

	// Initialize touchscreen
	#pragma region touch screen
	//#include <SPI.h>


	//#define  TS1	//Touch screen  driver XPT2046_Touchscreen
	#define  TS2	//Touch screen  driver XPT2046-2

	#ifdef  TS1
	#include <XPT2046_Touchscreen\XPT2046_Touchscreen.h>
	#define isTouching() touched()
	#else
	#include <XPT2046-2/XPT2046-2.h>

	#endif // TS1



	//class TS_Point {
	//public:
	//	TS_Point(void) : x(0), y(0), z(0) {}
	//	TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
	//	bool operator==(TS_Point p) { return ((p.x == x) && (p.y == y) && (p.z == z)); }
	//	bool operator!=(TS_Point p) { return ((p.x != x) || (p.y != y) || (p.z != z)); }
	//	uint16_t x, y, z;
	//};
	//#include <XPT2046\XPT2046.h>


	//XPT2046 ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
	#ifdef TS1
		XPT2046_Touchscreen ts((uint8_t)Pin_TS_CS, (uint8_t)Pin_TS_TIRQ);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
	#else

		XPT2046 ts((uint8_t)Pin_TS_CS, (uint8_t)Pin_TS_TIRQ);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

	#endif // TS1


	#pragma endregion


#endif
#ifdef TFT_ili9488

#pragma region TFT
#define TFT_X_WIDTH 240
#define TFT_Y_HEIGHT 320
#include <UTFT\UTFT.h>
UTFT    tft(ILI9327_8, 38, 39, 40, 41);

#define TFTprintAtStr(x,y,s) tft.print(s,x,y)
#define TFTprintAtNumF(x,y,d,decimals) tft.printNumF( d,decimals,x,y)
#define TFTprintAtNumI(x,y,i) tft.printNumI(i,x,y)

#pragma endregion

#pragma region touch screen
#include <UTouch.h>
UTouch  myTouch(6, 5, 4, 3, 2);
#pragma endregion
#endif
#ifdef  SPFD5408

#include <SPFD5408/SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408/SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
//#include <SPFD5408_TouchScreen.h>
// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:


#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4


Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
#define TFTprintAtStr(x,y,s) tft.print(s,x,y)
#define TFTprintAtNumF(x,y,d,decimals) tft.printNumF( d,decimals,x,y)
#define TFTprintAtNumI(x,y,i) tft.printNumI(i,x,y)

#endif // 1 


#endif // !__HWINCLUDES__
