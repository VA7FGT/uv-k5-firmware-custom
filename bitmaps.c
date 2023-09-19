
#include "bitmaps.h"

// all these images are on their right sides
// turn your monitor 90-deg anti-clockwise to see the images

const uint8_t BITMAP_PowerSave[8] =
{
	#if 0
		// "S"
		0b00000000,
		0b00100110,
		0b01001001,
		0b01001001,
		0b01001001,
		0b01001001,
		0b01001001,
		0b00110010
	#else
		// "PS"
		0b00000000,
		0b01111111,
		0b00010001,
		0b00001110,
		0b00000000,
		0b01000110,
		0b01001001,
		0b00110001
	#endif
};

#ifndef ENABLE_REVERSE_BAT_SYMBOL
	// Quansheng way (+ pole to the left)
	const uint8_t BITMAP_BatteryLevel1[17] =
	{
		0b00000000,
		0b00111110,
		0b00100010,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01111111
	};
	
	const uint8_t BITMAP_BatteryLevel2[17] =
	{
		0b00000000,
		0b00111110,
		0b00100010,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01111111
	};
	
	const uint8_t BITMAP_BatteryLevel3[17] =
	{
		0b00000000,
		0b00111110,
		0b00100010,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01111111
	};
	
	const uint8_t BITMAP_BatteryLevel4[17] =
	{
		0b00000000,
		0b00111110,
		0b00100010,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01111111
	};
	
	const uint8_t BITMAP_BatteryLevel5[17] =
	{
		0b00000000,
		0b00111110,
		0b00100010,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01111111
	};
#else
	// reversed (+ pole to the right)
	const uint8_t BITMAP_BatteryLevel1[17] =
	{
		0b00000000,
		0b01111111,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b00100010,
		0b00111110
	};
	
	const uint8_t BITMAP_BatteryLevel2[17] =
	{
		0b00000000,
		0b01111111,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b00100010,
		0b00111110
	};
	
	const uint8_t BITMAP_BatteryLevel3[17] =
	{
		0b00000000,
		0b01111111,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b00100010,
		0b00111110
	};
	
	const uint8_t BITMAP_BatteryLevel4[17] =
	{
		0b00000000,
		0b01111111,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01000001,
		0b01000001,
		0b01000001,
		0b00100010,
		0b00111110
	};
	
	const uint8_t BITMAP_BatteryLevel5[17] =
	{
		0b00000000,
		0b01111111,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b01011101,
		0b01011101,
		0b01000001,
		0b00100010,
		0b00111110
	};
#endif

const uint8_t BITMAP_USB_C[9] =
{	// USB symbol
	0b00000000,
	0b00011100,
	0b00100111,
	0b01000100,
	0b01000100,
	0b01000100,
	0b01000100,
	0b00100111,
	0b00011100
};

const uint8_t BITMAP_KeyLock[9] =
{	// padlock symbol
	0b00000000,
	0b01111100,
	0b01000110,
	0b01000101,
	0b01000101,
	0b01000101,
	0b01000101,
	0b01000110,
	0b01111100
};

const uint8_t BITMAP_F_Key[9] =
{	// F-Key symbol
	0b11111111,
	0b10000001,
	0b10111101,
	0b10010101,
	0b10010101,
	0b10010101,
	0b10000101,
	0b10000001,
	0b11111111
};

const uint8_t BITMAP_VOX[18] =
{	// "VOX"
	0b00000000,
	0b00011111,
	0b00100000,
	0b01000000,
	0b00100000,
	0b00011111,

	0b00000000,
	0b00111110,
	0b01000001,
	0b01000001,
	0b01000001,
	0b00111110,

	0b00000000,
	0b01100011,
	0b00010100,
	0b00001000,
	0b00010100,
	0b01100011
};

#if 0
	const uint8_t BITMAP_WX[12] =
	{	// "WX"
		0b00000000,
		0b01111111,
		0b00100000,
		0b00011000,
		0b00100000,
		0b01111111,

		0b00000000,
		0b01100011,
		0b00010100,
		0b00001000,
		0b00010100,
		0b01100011
	};
#else
	// 'XB' (cross-band)
	const uint8_t BITMAP_XB[12] =
	{	// "XB"
		0b00000000,
		0b01100011,
		0b00010100,
		0b00001000,
		0b00010100,
		0b01100011,

		0b00000000,
		0b01111111,
		0b01001001,
		0b01001001,
		0b01001001,
		0b00110110
	};
#endif

const uint8_t BITMAP_TDR1[12] =
{	// "DW"
	0b00000000,
	0b01111111,
	0b01000001,
	0b01000001,
	0b01000001,
	0b00111110,

	0b00000000,
	0b01111111,
	0b00100000,
	0b00011000,
	0b00100000,
	0b01111111
};

const uint8_t BITMAP_TDR2[12] =
{	// "--"
	0b00000000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,

	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
};

#ifdef ENABLE_VOICE
	const uint8_t BITMAP_VoicePrompt[9] =
	{
		0b00000000,
		0b00011000,
		0b00011000,
		0b00100100,
		0b00100100,
		0b01000010,
		0b01000010,
		0b11111111,
		0b00011000
	};
#endif

#ifdef ENABLE_FMRADIO
	const uint8_t BITMAP_FM[12] =
	{	// "FM"
		0b00000000,
		0b01111111,
		0b00001001,
		0b00001001,
		0b00001001,
		0b00000001,

		0b00000000,
		0b01111111,
		0b00000010,
		0b00001100,
		0b00000010,
		0b01111111
	};
#endif

#ifdef ENABLE_NOAA
	const uint8_t BITMAP_NOAA[12] =
	{	// "NS"
		0b00000000,
		0b01111111,
		0b00000100,
		0b00001000,
		0b00010000,
		0b01111111,

		0b00000000,
		0b01000110,
		0b01001001,
		0b01001001,
		0b01001001,
		0b00110001
	};
#endif

const uint8_t BITMAP_SC[12] =
{	// "SC"
	0b00000000,
	0b01000110,
	0b01001001,
	0b01001001,
	0b01001001,
	0b00110001,

	0b00000000,
	0b00111110,
	0b01000001,
	0b01000001,
	0b01000001,
	0b00100010
};

const uint8_t BITMAP_Antenna[5] =
{
	0b00000011,
	0b00000101,
	0b01111111,
	0b00000101,
	0b00000011
};

const uint8_t BITMAP_AntennaLevel1[3] =
{
	0b01100000,
	0b01100000,
	0b00000000
};

const uint8_t BITMAP_AntennaLevel2[3] =
{
	0b01110000,
	0b01110000,
	0b00000000
};

const uint8_t BITMAP_AntennaLevel3[3] =
{
	0b01111000,
	0b01111000,
	0b00000000
};

const uint8_t BITMAP_AntennaLevel4[3] =
{
	0b01111100,
	0b01111100,
	0b00000000
};

const uint8_t BITMAP_AntennaLevel5[3] =
{
	0b01111110,
	0b01111110,
	0b00000000
};

const uint8_t BITMAP_AntennaLevel6[3] =
{
	0b01111111,
	0b01111111,
	0b00000000
};

const uint8_t BITMAP_CurrentIndicator[8] =
{
	0b11111111,
	0b11111111,
	0b01111110,
	0b01111110,
	0b00111100,
	0b00111100,
	0b00011000,
	0b00011000
};

const uint8_t BITMAP_VFO_Default[8] =
{
	0b00000000,
	0b01111111,
	0b01111111,
	0b00111110,
	0b00111110,
	0b00011100,
	0b00011100,
	0b00001000
};

const uint8_t BITMAP_VFO_NotDefault[8] =
{
	0b00000000,
	0b01000001,
	0b01000001,
	0b00100010,
	0b00100010,
	0b00010100,
	0b00010100,
	0b00001000
};
/*
const uint8_t BITMAP_ScanList[6] =
{	// diamond symbol
	0b00001000,
	0b00011100,
	0b00111110,
	0b00111110,
	0b00011100,
	0b00001000
};
*/
const uint8_t BITMAP_ScanList1[6] =
{	// 1 symbol
	0b00000000,
	0b01000010,
	0b01111110,
	0b01000010,
	0b00000000,
	0b00000000
};

const uint8_t BITMAP_ScanList2[6] =
{	// 2 symbol
	0b00000000,
	0b01000010,
	0b01111110,
	0b01000010,
	0b01111110,
	0b01000010
};
