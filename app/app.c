/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <string.h>

#include "app/action.h"
#ifdef ENABLE_AIRCOPY
	#include "app/aircopy.h"
#endif
#include "app/app.h"
#include "app/dtmf.h"
#ifdef ENABLE_FMRADIO
	#include "app/fm.h"
#endif
#include "app/generic.h"
#include "app/main.h"
#include "app/menu.h"
#include "app/scanner.h"
#include "app/uart.h"
#include "ARMCM0.h"
#include "audio.h"
#include "board.h"
#include "bsp/dp32g030/gpio.h"
#include "driver/backlight.h"
#ifdef ENABLE_FMRADIO
	#include "driver/bk1080.h"
#endif
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "dtmf.h"
#include "external/printf/printf.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#if defined(ENABLE_OVERLAY)
	#include "sram-overlay.h"
#endif
#include "ui/battery.h"
#include "ui/inputbox.h"
#include "ui/menu.h"
#include "ui/rssi.h"
#include "ui/status.h"
#include "ui/ui.h"

static void APP_ProcessKey(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld);

static void APP_CheckForIncoming(void)
{
	if (!g_SquelchLost)
		return;

	if (gScanState == SCAN_OFF)
	{
		if (gCssScanMode != CSS_SCAN_MODE_OFF && gRxReceptionMode == RX_MODE_NONE)
		{
			ScanPauseDelayIn_10ms = 100;      // 1 second
			gScheduleScanListen   = false;
			gRxReceptionMode      = RX_MODE_DETECTED;
		}

		if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF)
		{
			#ifdef ENABLE_NOAA
				if (gIsNoaaMode)
				{
					gNOAA_Countdown_10ms = 20;    // 200ms
					gScheduleNOAA        = false;
				}
			#endif

			FUNCTION_Select(FUNCTION_INCOMING);
			return;
		}

		if (gRxReceptionMode != RX_MODE_NONE)
		{
			FUNCTION_Select(FUNCTION_INCOMING);
			return;
		}

		gDualWatchCountdown_10ms   = dual_watch_count_after_rx_10ms;
		gScheduleDualWatch = false;

		// let the user see DW is not active
		gDualWatchActive = false;
		gUpdateStatus    = true;
	}
	else
	{
		if (gRxReceptionMode != RX_MODE_NONE)
		{
			FUNCTION_Select(FUNCTION_INCOMING);
			return;
		}

		ScanPauseDelayIn_10ms = 20;     // 200ms
		gScheduleScanListen   = false;
	}

	gRxReceptionMode = RX_MODE_DETECTED;

	FUNCTION_Select(FUNCTION_INCOMING);
}

static void APP_HandleIncoming(void)
{
	bool bFlag;

	if (!g_SquelchLost)
	{
		FUNCTION_Select(FUNCTION_FOREGROUND);
		gUpdateDisplay = true;
		return;
	}

	bFlag = (gScanState == SCAN_OFF && gCurrentCodeType == CODE_TYPE_OFF);

	#ifdef ENABLE_NOAA
		if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE) && gNOAACountdown_10ms > 0)
		{
			gNOAACountdown_10ms = 0;
			bFlag               = true;
		}
	#endif

	if (g_CTCSS_Lost && gCurrentCodeType == CODE_TYPE_CONTINUOUS_TONE)
	{
		bFlag       = true;
		gFoundCTCSS = false;
	}

	if (g_CDCSS_Lost && gCDCSSCodeType == CDCSS_POSITIVE_CODE && (gCurrentCodeType == CODE_TYPE_DIGITAL || gCurrentCodeType == CODE_TYPE_REVERSE_DIGITAL))
	{
		gFoundCDCSS = false;
	}
	else
	if (!bFlag)
		return;

	DTMF_HandleRequest();

	if (gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF)
	{
		if (gRxVfo->DTMF_DECODING_ENABLE || gSetting_KILLED)
		{
			if (gDTMF_CallState == DTMF_CALL_STATE_NONE)
			{
				if (gRxReceptionMode == RX_MODE_DETECTED)
				{
					gDualWatchCountdown_10ms = dual_watch_count_after_1_10ms;
					gScheduleDualWatch = false;

					gRxReceptionMode = RX_MODE_LISTENING;

					// let the user see DW is not active
					gDualWatchActive = false;
					gUpdateStatus    = true;

					return;
				}
			}
		}
	}

	APP_StartListening(FUNCTION_RECEIVE);
}

static void APP_HandleReceive(void)
{
	#define END_OF_RX_MODE_SKIP 0
	#define END_OF_RX_MODE_END  1
	#define END_OF_RX_MODE_TTE  2

	uint8_t Mode = END_OF_RX_MODE_SKIP;

	if (gFlagTailNoteEliminationComplete)
	{
		Mode = END_OF_RX_MODE_END;
		goto Skip;
	}

	if (gScanState != SCAN_OFF && IS_FREQ_CHANNEL(gNextMrChannel))
	{
		if (g_SquelchLost)
			return;

		Mode = END_OF_RX_MODE_END;
		goto Skip;
	}

	switch (gCurrentCodeType)
	{
		default:
		case CODE_TYPE_OFF:
			break;

		case CODE_TYPE_CONTINUOUS_TONE:
			if (gFoundCTCSS && gFoundCTCSSCountdown_10ms == 0)
			{
				gFoundCTCSS = false;
				gFoundCDCSS = false;
				Mode        = END_OF_RX_MODE_END;
				goto Skip;
			}
			break;

		case CODE_TYPE_DIGITAL:
		case CODE_TYPE_REVERSE_DIGITAL:
			if (gFoundCDCSS && gFoundCDCSSCountdown_10ms == 0)
			{
				gFoundCTCSS = false;
				gFoundCDCSS = false;
				Mode        = END_OF_RX_MODE_END;
				goto Skip;
			}
			break;
	}

	if (g_SquelchLost)
	{
		#ifdef ENABLE_NOAA
			if (!gEndOfRxDetectedMaybe && IS_NOT_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE))
		#else
			if (!gEndOfRxDetectedMaybe)
		#endif
		{
			switch (gCurrentCodeType)
			{
				case CODE_TYPE_OFF:
					if (gEeprom.SQUELCH_LEVEL)
					{
						if (g_CxCSS_TAIL_Found)
						{
							Mode               = END_OF_RX_MODE_TTE;
							g_CxCSS_TAIL_Found = false;
						}
					}
					break;

				case CODE_TYPE_CONTINUOUS_TONE:
					if (g_CTCSS_Lost)
					{
						gFoundCTCSS = false;
					}
					else
					if (!gFoundCTCSS)
					{
						gFoundCTCSS               = true;
						gFoundCTCSSCountdown_10ms = 100;   // 1 sec
					}

					if (g_CxCSS_TAIL_Found)
					{
						Mode               = END_OF_RX_MODE_TTE;
						g_CxCSS_TAIL_Found = false;
					}
					break;

				case CODE_TYPE_DIGITAL:
				case CODE_TYPE_REVERSE_DIGITAL:
					if (g_CDCSS_Lost && gCDCSSCodeType == CDCSS_POSITIVE_CODE)
					{
						gFoundCDCSS = false;
					}
					else
					if (!gFoundCDCSS)
					{
						gFoundCDCSS               = true;
						gFoundCDCSSCountdown_10ms = 100;   // 1 sec
					}

					if (g_CxCSS_TAIL_Found)
					{
						if (BK4819_GetCTCType() == 1)
							Mode = END_OF_RX_MODE_TTE;

						g_CxCSS_TAIL_Found = false;
					}

					break;
			}
		}
	}
	else
		Mode = END_OF_RX_MODE_END;

	if (!gEndOfRxDetectedMaybe         &&
	     Mode == END_OF_RX_MODE_SKIP   &&
	     gNextTimeslice40ms            &&
	     gEeprom.TAIL_NOTE_ELIMINATION &&
	    (gCurrentCodeType == CODE_TYPE_DIGITAL || gCurrentCodeType == CODE_TYPE_REVERSE_DIGITAL) &&
	     BK4819_GetCTCType() == 1)
		Mode = END_OF_RX_MODE_TTE;
	else
		gNextTimeslice40ms = false;

Skip:
	switch (Mode)
	{
		case END_OF_RX_MODE_SKIP:
			break;

		case END_OF_RX_MODE_END:
			RADIO_SetupRegisters(true);

			#ifdef ENABLE_NOAA
				if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE))
					gNOAACountdown_10ms = 300;         // 3 sec
			#endif

			gUpdateDisplay = true;

			if (gScanState != SCAN_OFF)
			{
				switch (gEeprom.SCAN_RESUME_MODE)
				{
					case SCAN_RESUME_TO:
						break;

					case SCAN_RESUME_CO:
						ScanPauseDelayIn_10ms = 360;
						gScheduleScanListen   = false;
						break;

					case SCAN_RESUME_SE:
						SCANNER_Stop();
						break;
				}
			}

			break;

		case END_OF_RX_MODE_TTE:
			if (gEeprom.TAIL_NOTE_ELIMINATION)
			{
				GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);

				gTailNoteEliminationCountdown_10ms = 20;
				gFlagTailNoteEliminationComplete   = false;
				gEndOfRxDetectedMaybe = true;
				gEnableSpeaker        = false;
			}
			break;
	}
}

static void APP_HandleFunction(void)
{
	switch (gCurrentFunction)
	{
		case FUNCTION_FOREGROUND:
			APP_CheckForIncoming();
			break;

		case FUNCTION_TRANSMIT:
			break;

		case FUNCTION_MONITOR:
			break;

		case FUNCTION_INCOMING:
			APP_HandleIncoming();
			break;

		case FUNCTION_RECEIVE:
			APP_HandleReceive();
			break;

		case FUNCTION_POWER_SAVE:
			if (!gRxIdleMode)
				APP_CheckForIncoming();
			break;
	}
}

void APP_StartListening(FUNCTION_Type_t Function)
{
	if (!gSetting_KILLED)
	{
		#ifdef ENABLE_FMRADIO
			if (gFmRadioMode)
				BK1080_Init(0, false);
		#endif

		gVFO_RSSI_Level[gEeprom.RX_CHANNEL == 0] = 0;

		GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);

		gEnableSpeaker = true;

		BACKLIGHT_TurnOn();

		if (gScanState != SCAN_OFF)
		{
			switch (gEeprom.SCAN_RESUME_MODE)
			{
				case SCAN_RESUME_TO:
					if (!gScanPauseMode)
					{
						ScanPauseDelayIn_10ms = 500;
						gScheduleScanListen   = false;
						gScanPauseMode        = true;
					}
					break;

				case SCAN_RESUME_CO:
				case SCAN_RESUME_SE:
					ScanPauseDelayIn_10ms = 0;
					gScheduleScanListen   = false;
					break;
			}

			bScanKeepFrequency = true;
		}

		#ifdef ENABLE_NOAA
			if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE) && gIsNoaaMode)
			{
				gRxVfo->CHANNEL_SAVE                      = gNoaaChannel + NOAA_CHANNEL_FIRST;
				gRxVfo->pRX->Frequency                    = NoaaFrequencyTable[gNoaaChannel];
				gRxVfo->pTX->Frequency                    = NoaaFrequencyTable[gNoaaChannel];
				gEeprom.ScreenChannel[gEeprom.RX_CHANNEL] = gRxVfo->CHANNEL_SAVE;
				gNOAA_Countdown_10ms                      = 500;   // 5 sec
				gScheduleNOAA                             = false;
			}
		#endif

		if (gCssScanMode != CSS_SCAN_MODE_OFF)
			gCssScanMode = CSS_SCAN_MODE_FOUND;

		if (gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF && gEeprom.DUAL_WATCH != DUAL_WATCH_OFF)
		{
			gDualWatchCountdown_10ms = dual_watch_count_after_2_10ms;
			gScheduleDualWatch = false;

			gRxVfoIsActive = true;

			// let the user see DW is not active
			gDualWatchActive = false;
			gUpdateStatus    = true;
		}

		const uint32_t rx_frequency = gRxVfo->pRX->Frequency;
/*
		if (gRxVfo->IsAM)
		{	// AM

			// RX AF level
			//
			// REG_48 <15:12> 11  ???
			//
			// REG_48 <11:10> 0 AF Rx Gain-1
			//                0 =   0dB
			//                1 =  -6dB
			//                2 = -12dB
			//                3 = -18dB
			//
			// REG_48 <9:4>   60 AF Rx Gain-2  -26dB ~ 5.5dB   0.5dB/step
			//                63 = max
			//                 0 = min = mute
			//
			// REG_48 <3:0>   15 AF DAC Gain (after Gain-1 and Gain-2) approx 2dB/step
			//                15 = max
			//                 0 = min
			//
			BK4819_WriteRegister(BK4819_REG_48,
			#if 0
				// QS calibrated RX AF gain
				(11u << 12)                 |     // ???
				( 0u << 10)                 |     // AF Rx Gain-1
				(gEeprom.VOLUME_GAIN <<  4) |     // AF Rx Gain-2
				(gEeprom.DAC_GAIN    <<  0));     // AF DAC Gain (after Gain-1 and Gain-2)
			#else
				// max RX AF gain
				(11u << 12) |     // ???
				( 0u << 10) |     // AF Rx Gain-1
				(63u <<  4) |     // AF Rx Gain-2
				(15u <<  0));     // AF DAC Gain (after Gain-1 and Gain-2)
			#endif

//			BK4819_WriteRegister(0x4B, BK4819_ReadRegister(0x4B) & ~(1u << 5));	// enable RX ALC

			// help improve AM RX distorted audio by reducing the PGA gain (still very bad with stronge signals)
			//
			// I think a solution is to dynamically change these values as the RSSI moves up/down ?
			// without a detailed datasheet on the chip it's difficult/impossible to fix things
			//
			// REG_10 <15:0> 0x0038 Rx AGC Gain Table[0]. (Index Max->Min is 3,2,1,0,-1)
			//
			//         <9:8> = LNA Gain Short
			//                 3 =   0dB
			//                 2 = -11dB
			//                 1 = -16dB
			//                 0 = -19dB
			//
			//         <7:5> = LNA Gain
			//                 7 =   0dB
			//                 6 =  -2dB
			//                 5 =  -4dB
			//                 4 =  -6dB
			//                 3 =  -9dB
			//                 2 = -14dB
			//                 1 = -19dB
			//                 0 = -24dB
			//
			//         <4:3> = MIXER Gain
			//                 3 =  0dB
			//                 2 = -3dB
			//                 1 = -6dB
			//                 0 = -8dB
			//
			//         <2:0> = PGA Gain
			//                 7 =   0dB
			//                 6 =  -3dB
			//                 5 =  -6dB
			//                 4 =  -9dB
			//                 3 = -15dB
			//                 2 = -21dB
			//                 1 = -27dB
			//                 0 = -33dB
			//
			// LNA_SHORT ..   0dB
			// LNA ........  14dB
			// MIXER ......   0dB
			// PGA ........ -15dB
			//
			{
				uint16_t lna_short;  // whats "LNA SHORT" mean ?
				uint16_t lna;
				uint16_t mixer;
				uint16_t pga;
				// seems the RX gain abrutly reduces above this frequency, why ?
				if (rx_frequency <= 22640000)
				{
					lna_short = 3;   // original
					lna       = 2;   // original
					mixer     = 3;   // original
					pga       = 3;   // reduced - seems to help reduce the AM demodulation distortion
				}
				else
				{
					lna_short = 3;   // original
					lna       = 4;   // increased
					mixer     = 3;   // original
				//	pga       = 6;   // original
					pga       = 7;   // increased
				}

				BK4819_WriteRegister(BK4819_REG_13, (lna_short << 8) | (lna << 5) | (mixer << 3) | (pga << 0));

				// what do these 4 other gain tables do ???
				//BK4819_WriteRegister(BK4819_REG_12, 0x037B);  // 000000 11 011 11 011
				//BK4819_WriteRegister(BK4819_REG_11, 0x027B);  // 000000 10 011 11 011
				//BK4819_WriteRegister(BK4819_REG_10, 0x007A);  // 000000 00 011 11 010
				//BK4819_WriteRegister(BK4819_REG_14, 0x0019);  // 000000 00 000 11 001
			}

			gNeverUsed = 0;
		}
		else
		{	// FM

			// RX AF level
			//
			// REG_48 <15:12> 11  ???
			//
			// REG_48 <11:10> 0 AF Rx Gain-1
			//                0 =   0dB
			//                1 =  -6dB
			//                2 = -12dB
			//                3 = -18dB
			//
			// REG_48 <9:4>   60 AF Rx Gain-2  -26dB ~ 5.5dB   0.5dB/step
			//                63 = max
			//                 0 = min = mute
			//
			// REG_48 <3:0>   15 AF DAC Gain (after Gain-1 and Gain-2) approx 2dB/step
			//                15 = max
			//                 0 = min
			//
			BK4819_WriteRegister(BK4819_REG_48,
				(11u << 12)                 |     // ???
				( 0u << 10)                 |     // AF Rx Gain-1
				(gEeprom.VOLUME_GAIN <<  4) |     // AF Rx Gain-2
				(gEeprom.DAC_GAIN    <<  0));     // AF DAC Gain (after Gain-1 and Gain-2)

//			BK4819_WriteRegister(0x4B, BK4819_ReadRegister(0x4B) | (1u << 5));	// disable RX ALC

			// REG_10 <15:0> 0x0038 Rx AGC Gain Table[0]. (Index Max->Min is 3,2,1,0,-1)
			//
			//         <9:8> = LNA Gain Short
			//                 3 =   0dB
			//                 2 = -11dB
			//                 1 = -16dB
			//                 0 = -19dB
			//
			//         <7:5> = LNA Gain
			//                 7 =   0dB
			//                 6 =  -2dB
			//                 5 =  -4dB
			//                 4 =  -6dB
			//                 3 =  -9dB
			//                 2 = -14dB
			//                 1 = -19dB
			//                 0 = -24dB
			//
			//         <4:3> = MIXER Gain
			//                 3 =  0dB
			//                 2 = -3dB
			//                 1 = -6dB
			//                 0 = -8dB
			//
			//         <2:0> = PGA Gain
			//                 7 =   0dB
			//                 6 =  -3dB
			//                 5 =  -6dB
			//                 4 =  -9dB
			//                 3 = -15dB
			//                 2 = -21dB
			//                 1 = -27dB
			//                 0 = -33dB
			//
			// LNA_SHORT ..   0dB
			// LNA ........  14dB
			// MIXER ......   0dB
			// PGA ........  -3dB
			//                                  LNA SHORT     LNA         MIXER        PGA
			BK4819_WriteRegister(BK4819_REG_13, (3u << 8) | (2u << 5) | (3u << 3) | (6u << 0));
		}
*/
		if (gRxVfo->IsAM)
		{	// AM
			{
				uint16_t lna_short;  // whats "LNA SHORT" mean ?
				uint16_t lna;
				uint16_t mixer;
				uint16_t pga;
				// seems the RX gain abrutly reduces above this frequency, why ?
				if (rx_frequency <= 22640000)
				{
					lna_short = 3;   // original
					lna       = 2;   // original
					mixer     = 3;   // original
					pga       = 3;   // reduced - seems to help reduce the AM demodulation distortion
				}
				else
				{
					lna_short = 3;   // original
					lna       = 4;   // increased
					mixer     = 3;   // original
				//	pga       = 6;   // original
					pga       = 7;   // increased
				}

				BK4819_WriteRegister(BK4819_REG_13, (lna_short << 8) | (lna << 5) | (mixer << 3) | (pga << 0));

				// what do these 4 other gain tables do ???
				//BK4819_WriteRegister(BK4819_REG_12, 0x037B);  // 000000 11 011 11 011
				//BK4819_WriteRegister(BK4819_REG_11, 0x027B);  // 000000 10 011 11 011
				//BK4819_WriteRegister(BK4819_REG_10, 0x007A);  // 000000 00 011 11 010
				//BK4819_WriteRegister(BK4819_REG_14, 0x0019);  // 000000 00 000 11 001
			}
			BK4819_WriteRegister(BK4819_REG_48,
				// max RX AF gain
				(11u << 12) |     // ???
				( 0u << 10) |     // AF Rx Gain-1
				(63u <<  4) |     // AF Rx Gain-2
				(15u <<  0));     // AF DAC Gain (after Gain-1 and Gain-2)
			gNeverUsed = 0;
		}
		else
		{	// FM
			BK4819_WriteRegister(BK4819_REG_48,
				(11u << 12)                 |     // ???
				( 0u << 10)                 |     // AF Rx Gain-1
				(gEeprom.VOLUME_GAIN <<  4) |     // AF Rx Gain-2
				(gEeprom.DAC_GAIN    <<  0));     // AF DAC Gain (after Gain-1 and Gain-2)
		}
		
		#ifdef ENABLE_VOICE
			if (gVoiceWriteIndex == 0)
		#endif
				BK4819_SetAF(gRxVfo->IsAM ? BK4819_AF_AM : BK4819_AF_OPEN);

		FUNCTION_Select(Function);

		#ifdef ENABLE_FMRADIO
			if (Function == FUNCTION_MONITOR || gFmRadioMode)
		#else
			if (Function == FUNCTION_MONITOR)
		#endif
		{
			GUI_SelectNextDisplay(DISPLAY_MAIN);
			return;
		}

		gUpdateDisplay = true;
	}
}

void APP_SetFrequencyByStep(VFO_Info_t *pInfo, int8_t Step)
{
	uint32_t Frequency = pInfo->ConfigRX.Frequency + (Step * pInfo->StepFrequency);

	if (pInfo->StepFrequency == 833)
	{
		const uint32_t Lower = LowerLimitFrequencyBandTable[pInfo->Band];
		const uint32_t Delta = Frequency - Lower;
		uint32_t       Base  = (Delta / 2500) * 2500;
		const uint32_t Index = ((Delta - Base) % 2500) / 833;

		if (Index == 2)
			Base++;

		Frequency = Lower + Base + (Index * 833);
	}

	if (Frequency > UpperLimitFrequencyBandTable[pInfo->Band])
		Frequency = LowerLimitFrequencyBandTable[pInfo->Band];
	else
	if (Frequency < LowerLimitFrequencyBandTable[pInfo->Band])
		Frequency = FREQUENCY_FloorToStep(UpperLimitFrequencyBandTable[pInfo->Band], pInfo->StepFrequency, LowerLimitFrequencyBandTable[pInfo->Band]);

	pInfo->ConfigRX.Frequency = Frequency;
}

static void FREQ_NextChannel(void)
{
	APP_SetFrequencyByStep(gRxVfo, gScanState);

	RADIO_ApplyOffset(gRxVfo);
	RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
	RADIO_SetupRegisters(true);

	gUpdateDisplay        = true;
	ScanPauseDelayIn_10ms = 10;
	bScanKeepFrequency    = false;
}

static void MR_NextChannel(void)
{
	uint8_t Ch;
	uint8_t Ch1        = gEeprom.SCANLIST_PRIORITY_CH1[gEeprom.SCAN_LIST_DEFAULT];
	uint8_t Ch2        = gEeprom.SCANLIST_PRIORITY_CH2[gEeprom.SCAN_LIST_DEFAULT];
	uint8_t PreviousCh = gNextMrChannel;
	bool    bEnabled   = gEeprom.SCAN_LIST_ENABLED[gEeprom.SCAN_LIST_DEFAULT];

	if (bEnabled)
	{
		if (gCurrentScanList == 0)
		{
			gPreviousMrChannel = gNextMrChannel;
			if (RADIO_CheckValidChannel(Ch1, false, 0))
				gNextMrChannel   = Ch1;
			else
				gCurrentScanList = 1;
		}

		if (gCurrentScanList == 1)
		{
			if (RADIO_CheckValidChannel(Ch2, false, 0))
				gNextMrChannel   = Ch2;
			else
				gCurrentScanList = 2;
		}

		if (gCurrentScanList == 2)
			gNextMrChannel = gPreviousMrChannel;
		else
			goto Skip;
	}

	Ch = RADIO_FindNextChannel(gNextMrChannel + gScanState, gScanState, true, gEeprom.SCAN_LIST_DEFAULT);
	if (Ch == 0xFF)
		return;

	gNextMrChannel = Ch;

Skip:
	if (PreviousCh != gNextMrChannel)
	{
		gEeprom.MrChannel[gEeprom.RX_CHANNEL]     = gNextMrChannel;
		gEeprom.ScreenChannel[gEeprom.RX_CHANNEL] = gNextMrChannel;

		RADIO_ConfigureChannel(gEeprom.RX_CHANNEL, 2);
		RADIO_SetupRegisters(true);

		gUpdateDisplay = true;
	}

	ScanPauseDelayIn_10ms = 20;

	bScanKeepFrequency    = false;

	if (bEnabled)
		if (++gCurrentScanList >= 2)
			gCurrentScanList = 0;
}

#ifdef ENABLE_NOAA
	static void NOAA_IncreaseChannel(void)
	{
		if (++gNoaaChannel > 9)
			gNoaaChannel = 0;
	}
#endif

static void DUALWATCH_Alternate(void)
{
	#ifdef ENABLE_NOAA
		if (gIsNoaaMode)
		{
			if (IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[0]) || IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[1]))
				gEeprom.RX_CHANNEL = 1 - gEeprom.RX_CHANNEL;
			else
				gEeprom.RX_CHANNEL = 0;

			gRxVfo = &gEeprom.VfoInfo[gEeprom.RX_CHANNEL];

			if (gEeprom.VfoInfo[0].CHANNEL_SAVE >= NOAA_CHANNEL_FIRST)
				NOAA_IncreaseChannel();
		}
		else
	#endif
	{	// toggle between VFO's
		gEeprom.RX_CHANNEL = (1 - gEeprom.RX_CHANNEL) & 1;
		gRxVfo             = &gEeprom.VfoInfo[gEeprom.RX_CHANNEL];

		if (!gDualWatchActive)
		{	// let the user see DW is active
			gDualWatchActive = true;
			gUpdateStatus    = true;
		}
	}

	RADIO_SetupRegisters(false);

	#ifdef ENABLE_NOAA
		gDualWatchCountdown_10ms = gIsNoaaMode ? dual_watch_count_noaa_10ms : dual_watch_count_toggle_10ms;
	#else
		gDualWatchCountdown_10ms = dual_watch_count_toggle_10ms;
	#endif
}

void APP_CheckRadioInterrupts(void)
{
	if (gScreenToDisplay == DISPLAY_SCANNER)
		return;

	while (BK4819_ReadRegister(BK4819_REG_0C) & 1u)
	{	// BK chip interrupt request

		uint16_t interrupt_status_bits;

		// reset the interrupt ?
		BK4819_WriteRegister(BK4819_REG_02, 0);

		// fetch the interrupt status bits
		interrupt_status_bits = BK4819_ReadRegister(BK4819_REG_02);

		// 0 = no phase shift
		// 1 = 120deg phase shift
		// 2 = 180deg phase shift
		// 3 = 240deg phase shift
//		const uint8_t ctcss_shift = BK4819_GetCTCShift();
//		if (ctcss_shift > 0)
//			g_CTCSS_Lost = true;

		if (interrupt_status_bits & BK4819_REG_02_DTMF_5TONE_FOUND)
		{	// save the new DTMF RX'ed character

			// fetch the RX'ed char
			const char c = DTMF_GetCharacter(BK4819_GetDTMF_5TONE_Code());
			if (c != 0xff)
			{
				gDTMF_RequestPending = true;
				gDTMF_RecvTimeout    = DTMF_RX_timeout_500ms;
				// shift the RX buffer down one - if need be
				if (gDTMF_WriteIndex >= sizeof(gDTMF_Received))
					memmove(gDTMF_Received, &gDTMF_Received[1], gDTMF_WriteIndex-- - 1);
				gDTMF_Received[gDTMF_WriteIndex++] = c;

				if (gCurrentFunction == FUNCTION_RECEIVE)
				{
					#ifdef ENABLE_DTMF_DECODER
						gDTMF_RecvTimeoutSaved = DTMF_RX_timeout_saved_500ms;
						size_t len = strlen(gDTMF_ReceivedSaved);
						// shift the RX buffer down one - if need be
						if (len >= (sizeof(gDTMF_ReceivedSaved) - 1))
							memmove(gDTMF_ReceivedSaved, &gDTMF_ReceivedSaved[1], len--);
						gDTMF_ReceivedSaved[len++] = c;
						gDTMF_ReceivedSaved[len]   = '\0';
						gUpdateDisplay = true;
					#endif

					DTMF_HandleRequest();
				}
			}
		}

		if (interrupt_status_bits & BK4819_REG_02_CxCSS_TAIL)
			g_CxCSS_TAIL_Found = true;

		if (interrupt_status_bits & BK4819_REG_02_CDCSS_LOST)
		{
			g_CDCSS_Lost = true;
			gCDCSSCodeType = BK4819_GetCDCSSCodeType();
		}

		if (interrupt_status_bits & BK4819_REG_02_CDCSS_FOUND)
			g_CDCSS_Lost = false;

		if (interrupt_status_bits & BK4819_REG_02_CTCSS_LOST)
			g_CTCSS_Lost = true;

		if (interrupt_status_bits & BK4819_REG_02_CTCSS_FOUND)
			g_CTCSS_Lost = false;

		if (interrupt_status_bits & BK4819_REG_02_VOX_LOST)
		{
			g_VOX_Lost         = true;
			gVoxPauseCountdown = 10;

			if (gEeprom.VOX_SWITCH)
			{
				if (gCurrentFunction == FUNCTION_POWER_SAVE && !gRxIdleMode)
				{
					gBatterySave_10ms = 20;     // 200ms
					gBatterySaveCountdownExpired = 0;
				}

				if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF && (gScheduleDualWatch || gDualWatchCountdown_10ms < dual_watch_count_after_vox_10ms))
				{
					gDualWatchCountdown_10ms = dual_watch_count_after_vox_10ms;
					gScheduleDualWatch = false;

					// let the user see DW is not active
					gDualWatchActive = false;
					gUpdateStatus    = true;
				}
			}
		}

		if (interrupt_status_bits & BK4819_REG_02_VOX_FOUND)
		{
			g_VOX_Lost         = false;
			gVoxPauseCountdown = 0;
		}

		if (interrupt_status_bits & BK4819_REG_02_SQUELCH_LOST)
		{
			g_SquelchLost = true;
			BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_GREEN, true);
		}

		if (interrupt_status_bits & BK4819_REG_02_SQUELCH_FOUND)
		{
			g_SquelchLost = false;
			BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_GREEN, false);
		}

		#ifdef ENABLE_AIRCOPY
			if (interrupt_status_bits & BK4819_REG_02_FSK_FIFO_ALMOST_FULL &&
			    gScreenToDisplay == DISPLAY_AIRCOPY &&
			    gAircopyState == AIRCOPY_TRANSFER &&
			    gAirCopyIsSendMode == 0)
			{
				unsigned int i;
				for (i = 0; i < 4; i++)
					g_FSK_Buffer[gFSKWriteIndex++] = BK4819_ReadRegister(BK4819_REG_5F);
				AIRCOPY_StorePacket();
			}
		#endif
	}
}

void APP_EndTransmission(void)
{	// back to RX mode

	RADIO_SendEndOfTransmission();

	if (gCurrentVfo->pTX->CodeType != CODE_TYPE_OFF)
	{	// CTCSS/DCS is enabled

		//if (gEeprom.TAIL_NOTE_ELIMINATION && gEeprom.REPEATER_TAIL_TONE_ELIMINATION > 0)
		if (gEeprom.TAIL_NOTE_ELIMINATION)
		{	// send the CTCSS/DCS tail tone - allows the receivers to mute the usual FM squelch tail/crash
			RADIO_EnableCxCSS();
		}
		#if 0
			else
			{	// TX a short blank carrier
				// this gives the receivers time to mute RX audio before we drop carrier
				BK4819_ExitSubAu();
				SYSTEM_DelayMs(200);
			}
		#endif
	}

	RADIO_SetupRegisters(false);
}

static void APP_HandleVox(void)
{
	if (!gSetting_KILLED)
	{
		if (gVoxResumeCountdown == 0)
		{
			if (gVoxPauseCountdown)
				return;
		}
		else
		{
			g_VOX_Lost         = false;
			gVoxPauseCountdown = 0;
		}

		if (gCurrentFunction != FUNCTION_RECEIVE  &&
		    gCurrentFunction != FUNCTION_MONITOR  &&
		    gScanState       == SCAN_OFF          &&
		    gCssScanMode     == CSS_SCAN_MODE_OFF
			#ifdef ENABLE_FMRADIO
				&& !gFmRadioMode
			#endif
		)
		{
			if (gVOX_NoiseDetected)
			{
				if (g_VOX_Lost)
					gVoxStopCountdown_10ms = 100;    // 1 sec
				else
				if (gVoxStopCountdown_10ms == 0)
					gVOX_NoiseDetected = false;

				if (gCurrentFunction == FUNCTION_TRANSMIT && !gPttIsPressed && !gVOX_NoiseDetected)
				{
					if (gFlagEndTransmission)
					{
						FUNCTION_Select(FUNCTION_FOREGROUND);
					}
					else
					{
						APP_EndTransmission();

						if (gEeprom.REPEATER_TAIL_TONE_ELIMINATION == 0)
							FUNCTION_Select(FUNCTION_FOREGROUND);
						else
							gRTTECountdown = gEeprom.REPEATER_TAIL_TONE_ELIMINATION * 10;
					}

					gUpdateDisplay       = true;
					gFlagEndTransmission = false;

					return;
				}
			}
			else
			if (g_VOX_Lost)
			{
				gVOX_NoiseDetected = true;

				if (gCurrentFunction == FUNCTION_POWER_SAVE)
					FUNCTION_Select(FUNCTION_FOREGROUND);

				if (gCurrentFunction != FUNCTION_TRANSMIT)
				{
					gDTMF_ReplyState = DTMF_REPLY_NONE;
					RADIO_PrepareTX();
					gUpdateDisplay = true;
				}
			}
		}
	}
}

void APP_Update(void)
{
	#ifdef ENABLE_VOICE
		if (gFlagPlayQueuedVoice)
		{
			AUDIO_PlayQueuedVoice();
			gFlagPlayQueuedVoice = false;
		}
	#endif

	if (gCurrentFunction == FUNCTION_TRANSMIT && gTxTimeoutReached)
	{
		gTxTimeoutReached    = false;
		gFlagEndTransmission = true;

		APP_EndTransmission();

		AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP);

		RADIO_SetVfoState(VFO_STATE_TIMEOUT);

		GUI_DisplayScreen();
	}

	if (gReducedService)
		return;

	if (gCurrentFunction != FUNCTION_TRANSMIT)
		APP_HandleFunction();

	#ifdef ENABLE_FMRADIO
		if (gFmRadioCountdown_500ms > 0)
			return;
	#endif

	#ifdef ENABLE_VOICE
		if (gScreenToDisplay != DISPLAY_SCANNER && gScanState != SCAN_OFF && gScheduleScanListen && !gPttIsPressed && gVoiceWriteIndex == 0)
	#else
		if (gScreenToDisplay != DISPLAY_SCANNER && gScanState != SCAN_OFF && gScheduleScanListen && !gPttIsPressed)
	#endif
	{
		if (IS_FREQ_CHANNEL(gNextMrChannel))
		{
			if (gCurrentFunction == FUNCTION_INCOMING)
				APP_StartListening(FUNCTION_RECEIVE);
			else
				FREQ_NextChannel();
		}
		else
		{
			if (gCurrentCodeType == CODE_TYPE_OFF && gCurrentFunction == FUNCTION_INCOMING)
				APP_StartListening(FUNCTION_RECEIVE);
			else
				MR_NextChannel();
		}

		gScanPauseMode      = false;
		gRxReceptionMode    = RX_MODE_NONE;
		gScheduleScanListen = false;
	}

	#ifdef ENABLE_VOICE
		if (gCssScanMode == CSS_SCAN_MODE_SCANNING && gScheduleScanListen && gVoiceWriteIndex == 0)
	#else
		if (gCssScanMode == CSS_SCAN_MODE_SCANNING && gScheduleScanListen)
	#endif
	{
		MENU_SelectNextCode();
		gScheduleScanListen = false;
	}

	#ifdef ENABLE_NOAA
		#ifdef ENABLE_VOICE
			if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF && gIsNoaaMode && gScheduleNOAA && gVoiceWriteIndex == 0)
		#else
			if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF && gIsNoaaMode && gScheduleNOAA)
		#endif
		{
			NOAA_IncreaseChannel();
			RADIO_SetupRegisters(false);

			gNOAA_Countdown_10ms = 7;      // 70ms
			gScheduleNOAA        = false;
		}
	#endif

	// toggle between the VFO's if dual watch is enabled
	if (gScreenToDisplay != DISPLAY_SCANNER && gEeprom.DUAL_WATCH != DUAL_WATCH_OFF)
	{
		#ifdef ENABLE_VOICE
			if (gScheduleDualWatch && gVoiceWriteIndex == 0)
		#else
			if (gScheduleDualWatch)
		#endif
		{
			if (gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF)
			{
				if (!gPttIsPressed &&
					#ifdef ENABLE_FMRADIO
						!gFmRadioMode &&
					#endif
				    gDTMF_CallState == DTMF_CALL_STATE_NONE &&
				    gCurrentFunction != FUNCTION_POWER_SAVE)
				{
					DUALWATCH_Alternate();    // toggle between the two VFO's

					if (gRxVfoIsActive && gScreenToDisplay == DISPLAY_MAIN)
						GUI_SelectNextDisplay(DISPLAY_MAIN);

					gRxVfoIsActive     = false;
					gScanPauseMode     = false;
					gRxReceptionMode   = RX_MODE_NONE;
					gScheduleDualWatch = false;
				}
			}
		}
	}

	#ifdef ENABLE_FMRADIO
		if (gScheduleFM                          &&
			gFM_ScanState    != FM_SCAN_OFF      &&
			gCurrentFunction != FUNCTION_MONITOR &&
			gCurrentFunction != FUNCTION_RECEIVE &&
			gCurrentFunction != FUNCTION_TRANSMIT)
		{
			FM_Play();
			gScheduleFM = false;
		}
	#endif

	if (gEeprom.VOX_SWITCH)
		APP_HandleVox();

	if (gSchedulePowerSave)
	{
		#ifdef ENABLE_NOAA
			if (
				#ifdef ENABLE_FMRADIO
					gFmRadioMode                  ||
			    #endif
				gPttIsPressed                     ||
			    gKeyBeingHeld                     ||
				gEeprom.BATTERY_SAVE == 0         ||
			    gScanState != SCAN_OFF            ||
			    gCssScanMode != CSS_SCAN_MODE_OFF ||
			    gScreenToDisplay != DISPLAY_MAIN  ||
			    gDTMF_CallState != DTMF_CALL_STATE_NONE)
			{
				gBatterySaveCountdown_10ms   = battery_save_count_10ms;
			}
			else
			if ((IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[0]) && IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[1])) || !gIsNoaaMode)
			{
				FUNCTION_Select(FUNCTION_POWER_SAVE);
			}
			else
			{
				gBatterySaveCountdown_10ms   = battery_save_count_10ms;
			}
		#else
			if (
				#ifdef ENABLE_FMRADIO
					gFmRadioMode                  ||
			    #endif
				gPttIsPressed                     ||
			    gKeyBeingHeld                     ||
				gEeprom.BATTERY_SAVE == 0         ||
			    gScanState != SCAN_OFF            ||
			    gCssScanMode != CSS_SCAN_MODE_OFF ||
			    gScreenToDisplay != DISPLAY_MAIN  ||
			    gDTMF_CallState != DTMF_CALL_STATE_NONE)
			{
				gBatterySaveCountdown_10ms   = battery_save_count_10ms;
			}
			else
			{
				FUNCTION_Select(FUNCTION_POWER_SAVE);
			}
			
			gSchedulePowerSave = false;
		#endif
	}

	#ifdef ENABLE_VOICE
		if (gBatterySaveCountdownExpired && gCurrentFunction == FUNCTION_POWER_SAVE && gVoiceWriteIndex == 0)
	#else
		if (gBatterySaveCountdownExpired && gCurrentFunction == FUNCTION_POWER_SAVE)
	#endif
	{
		if (gRxIdleMode)
		{
			BK4819_Conditional_RX_TurnOn_and_GPIO6_Enable();

			if (gEeprom.VOX_SWITCH)
				BK4819_EnableVox(gEeprom.VOX1_THRESHOLD, gEeprom.VOX0_THRESHOLD);

			if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF && gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF)
			{
				DUALWATCH_Alternate();    // toggle between the two VFO's

				gUpdateRSSI = false;
			}

			FUNCTION_Init();

			gBatterySave_10ms   = 10;    // 100ms
			gRxIdleMode         = false;
		}
		else
		if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF || gScanState != SCAN_OFF || gCssScanMode != CSS_SCAN_MODE_OFF || gUpdateRSSI)
		{
			gCurrentRSSI = BK4819_GetRSSI();
			UI_UpdateRSSI(gCurrentRSSI);

			gBatterySave_10ms   = gEeprom.BATTERY_SAVE * 10;
			gRxIdleMode         = true;

			BK4819_DisableVox();
			BK4819_Sleep();
			BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2, false);

			// Authentic device checked removed

		}
		else
		{
			DUALWATCH_Alternate();    // toggle between the two VFO's

			gUpdateRSSI         = true;
			gBatterySave_10ms   = 10;   // 100ms
		}

		gBatterySaveCountdownExpired = false;
	}
}

// called every 10ms
void APP_CheckKeys(void)
{
	KEY_Code_t Key;

	#ifdef ENABLE_AIRCOPY
		if (gSetting_KILLED || (gScreenToDisplay == DISPLAY_AIRCOPY && gAircopyState != AIRCOPY_READY))
			return;
	#else
		if (gSetting_KILLED)
			return;
	#endif

	if (gPttIsPressed)
	{
		if (GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT))
		{	// PTT released
			#if 0
				// denoise the PTT
				unsigned int i     = 6;   // test the PTT button for 6ms
				unsigned int count = 0;
				while (i-- > 0)
				{
					SYSTEM_DelayMs(1);
					if (!GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT))
					{	// PTT pressed
						if (count > 0)
							count--;
						continue;
					}
					if (++count < 3)
						continue;

					// stop transmitting
					APP_ProcessKey(KEY_PTT, false, false);
					gPttIsPressed = false;
					if (gKeyReading1 != KEY_INVALID)
						gPttWasReleased = true;
					break;
				}
			#else
				if (++gPttDebounceCounter >= 3)	    // 30ms
				{	// stop transmitting
					APP_ProcessKey(KEY_PTT, false, false);
					gPttIsPressed = false;
					if (gKeyReading1 != KEY_INVALID)
						gPttWasReleased = true;
				}
			#endif
		}
		else
			gPttDebounceCounter = 0;
	}
	else
	if (!GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT))
	{	// PTT pressed
		if (++gPttDebounceCounter >= 3)	    // 30ms
		{	// start transmitting
			boot_counter_10ms   = 0;
			gPttDebounceCounter = 0;
			gPttIsPressed       = true;
			APP_ProcessKey(KEY_PTT, true, false);
		}
	}
	else
		gPttDebounceCounter = 0;

	// *****************

	// scan the hardware keys
	Key = KEYBOARD_Poll();

	if (Key != KEY_INVALID)
		boot_counter_10ms = 0;   // cancel boot screen/beeps if any key pressed

	if (gKeyReading0 != Key)
	{	// new key pressed

		if (gKeyReading0 != KEY_INVALID && Key != KEY_INVALID)
			APP_ProcessKey(gKeyReading1, false, gKeyBeingHeld);  // key pressed without releasing previous key

		gKeyReading0     = Key;
		gDebounceCounter = 0;
		return;
	}

	if (++gDebounceCounter == key_debounce_10ms)
	{	// debounced new key pressed

		if (Key == KEY_INVALID)
		{
			if (gKeyReading1 != KEY_INVALID)
			{
				APP_ProcessKey(gKeyReading1, false, gKeyBeingHeld);
				gKeyReading1 = KEY_INVALID;
			}
		}
		else
		{
			gKeyReading1 = Key;
			APP_ProcessKey(Key, true, false);
		}

		gKeyBeingHeld = false;
		return;
	}

	// key is being held pressed

	if (gDebounceCounter == key_repeat_delay_10ms)
	{	// initial delay after pressed
		if (Key == KEY_STAR  ||
		    Key == KEY_F     ||
		    Key == KEY_SIDE2 ||
		    Key == KEY_SIDE1 ||
		    Key == KEY_UP    ||
		    Key == KEY_DOWN  ||
		    Key == KEY_EXIT
		    #ifdef ENABLE_MAIN_KEY_HOLD
		        || Key <= KEY_9       // keys 0-9 can be held down to bypass pressing the F-Key
		    #endif
			)
		{
			gKeyBeingHeld = true;
			APP_ProcessKey(Key, true, true);
		}
		return;
	}

	if (gDebounceCounter > key_repeat_delay_10ms)
	{	// key repeat
		if (Key == KEY_UP || Key == KEY_DOWN)
		{
			gKeyBeingHeld = true;
			if ((gDebounceCounter % key_repeat_10ms) == 0)
				APP_ProcessKey(Key, true, true);
		}

		if (gDebounceCounter < 0xFFFF)
			return;

		gDebounceCounter = key_repeat_delay_10ms;
		return;
	}
}

void APP_TimeSlice10ms(void)
{
	gFlashLightBlinkCounter++;

	#ifdef ENABLE_BOOT_BEEPS
		if (boot_counter_10ms > 0)
			if ((boot_counter_10ms % 25) == 0)
				AUDIO_PlayBeep(BEEP_880HZ_40MS_OPTIONAL);
	#endif

	if (UART_IsCommandAvailable())
	{
		__disable_irq();
		UART_HandleCommand();
		__enable_irq();
	}

	if (gReducedService)
		return;

	if (gCurrentFunction != FUNCTION_POWER_SAVE || !gRxIdleMode)
		APP_CheckRadioInterrupts();

	if (gCurrentFunction != FUNCTION_TRANSMIT)
	{
		if (gUpdateStatus)
			UI_DisplayStatus(false);

		if (gUpdateDisplay)
		{
			GUI_DisplayScreen();
			gUpdateDisplay = false;
		}
	}

	// Skipping authentic device checks

	#ifdef ENABLE_FMRADIO
		if (gFmRadioCountdown_500ms > 0)
			return;
	#endif

	if (gFlashLightState == FLASHLIGHT_BLINK && (gFlashLightBlinkCounter & 15u) == 0)
		GPIO_FlipBit(&GPIOC->DATA, GPIOC_PIN_FLASHLIGHT);

	if (gVoxResumeCountdown > 0)
		gVoxResumeCountdown--;

	if (gVoxPauseCountdown > 0)
		gVoxPauseCountdown--;

	if (gCurrentFunction == FUNCTION_TRANSMIT)
	{
		#ifdef ENABLE_ALARM
			if (gAlarmState == ALARM_STATE_TXALARM || gAlarmState == ALARM_STATE_ALARM)
			{
				uint16_t Tone;

				gAlarmRunningCounter++;
				gAlarmToneCounter++;

				Tone = 500 + (gAlarmToneCounter * 25);
				if (Tone > 1500)
				{
					Tone              = 500;
					gAlarmToneCounter = 0;
				}

				BK4819_SetScrambleFrequencyControlWord(Tone);

				if (gEeprom.ALARM_MODE == ALARM_MODE_TONE && gAlarmRunningCounter == 512)
				{
					gAlarmRunningCounter = 0;

					if (gAlarmState == ALARM_STATE_TXALARM)
					{
						gAlarmState = ALARM_STATE_ALARM;
						RADIO_EnableCxCSS();
						BK4819_SetupPowerAmplifier(0, 0);
						BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1, false);
						BK4819_Enable_AfDac_DiscMode_TxDsp();
						BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_RED, false);
						GUI_DisplayScreen();
					}
					else
					{
						gAlarmState = ALARM_STATE_TXALARM;
						GUI_DisplayScreen();
						BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_RED, true);
						RADIO_SetTxParameters();
						BK4819_TransmitTone(true, 500);
						SYSTEM_DelayMs(2);
						GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
						gEnableSpeaker    = true;
						gAlarmToneCounter = 0;
					}
				}
			}
		#endif

		// repeater tail tone elimination
		if (gRTTECountdown > 0)
		{
			if (--gRTTECountdown == 0)
			{
				FUNCTION_Select(FUNCTION_FOREGROUND);
				gUpdateDisplay = true;
			}
		}
	}

	#ifdef ENABLE_FMRADIO
		if (gFmRadioMode && gFM_RestoreCountdown > 0)
		{
			if (--gFM_RestoreCountdown == 0)
			{
				FM_Start();
				GUI_SelectNextDisplay(DISPLAY_FM);
			}
		}
	#endif

	if (gScreenToDisplay == DISPLAY_SCANNER)
	{
		uint32_t               Result;
		int32_t                Delta;
		BK4819_CssScanResult_t ScanResult;
		uint16_t               CtcssFreq;

		if (gScanDelay_10ms > 0)
		{
			gScanDelay_10ms--;
			APP_CheckKeys();
			return;
		}

		if (gScannerEditState != 0)
		{
			APP_CheckKeys();
			return;
		}

		switch (gScanCssState)
		{
			case SCAN_CSS_STATE_OFF:
				if (!BK4819_GetFrequencyScanResult(&Result))
					break;

				Delta          = Result - gScanFrequency;
				gScanFrequency = Result;

				if (Delta < 0)
					Delta = -Delta;
				if (Delta < 100)
					gScanHitCount++;
				else
					gScanHitCount = 0;

				BK4819_DisableFrequencyScan();

				if (gScanHitCount < 3)
				{
					BK4819_EnableFrequencyScan();
				}
				else
				{
					BK4819_SetScanFrequency(gScanFrequency);
					gScanCssResultCode     = 0xFF;
					gScanCssResultType     = 0xFF;
					gScanHitCount          = 0;
					gScanUseCssResult      = false;
					gScanProgressIndicator = 0;
					gScanCssState          = SCAN_CSS_STATE_SCANNING;
					GUI_SelectNextDisplay(DISPLAY_SCANNER);
					gUpdateStatus          = true;
				}

				//gScanDelay_10ms = scan_delay_10ms;
				gScanDelay_10ms = 20 / 10;   // 20ms
				break;

			case SCAN_CSS_STATE_SCANNING:
				ScanResult = BK4819_GetCxCSSScanResult(&Result, &CtcssFreq);
				if (ScanResult == BK4819_CSS_RESULT_NOT_FOUND)
					break;

				BK4819_Disable();

				if (ScanResult == BK4819_CSS_RESULT_CDCSS)
				{
					const uint8_t Code = DCS_GetCdcssCode(Result);
					if (Code != 0xFF)
					{
						gScanCssResultCode = Code;
						gScanCssResultType = CODE_TYPE_DIGITAL;
						gScanCssState      = SCAN_CSS_STATE_FOUND;
						gScanUseCssResult  = true;
						gUpdateStatus      = true;
					}
				}
				else
				if (ScanResult == BK4819_CSS_RESULT_CTCSS)
				{
					const uint8_t Code = DCS_GetCtcssCode(CtcssFreq);
					if (Code != 0xFF)
					{
						if (Code == gScanCssResultCode && gScanCssResultType == CODE_TYPE_CONTINUOUS_TONE)
						{
							if (++gScanHitCount >= 2)
							{
								gScanCssState     = SCAN_CSS_STATE_FOUND;
								gScanUseCssResult = true;
								gUpdateStatus     = true;
							}
						}
						else
							gScanHitCount = 0;

						gScanCssResultType = CODE_TYPE_CONTINUOUS_TONE;
						gScanCssResultCode = Code;
					}
				}

				if (gScanCssState < SCAN_CSS_STATE_FOUND)
				{
					BK4819_SetScanFrequency(gScanFrequency);
					gScanDelay_10ms = scan_delay_10ms;
					break;
				}

				GUI_SelectNextDisplay(DISPLAY_SCANNER);
				break;

			default:
				break;
		}
	}

	#ifdef ENABLE_AIRCOPY
		if (gScreenToDisplay == DISPLAY_AIRCOPY && gAircopyState == AIRCOPY_TRANSFER && gAirCopyIsSendMode == 1)
		{
			if (gAircopySendCountdown > 0)
			{
				if (--gAircopySendCountdown == 0)
				{
					AIRCOPY_SendMessage();
					GUI_DisplayScreen();
				}
			}
		}
	#endif

	APP_CheckKeys();
}

void cancelUserInputModes(void)
{
	gKeyInputCountdown = 0;
	if (gDTMF_InputMode || gInputBoxIndex > 0)
	{
		gDTMF_InputMode       = false;
		gDTMF_InputIndex      = 0;
		memset(gDTMF_String, 0, sizeof(gDTMF_String));
		gInputBoxIndex        = 0;
		gRequestDisplayScreen = DISPLAY_MAIN;
		gBeepToPlay           = BEEP_1KHZ_60MS_OPTIONAL;
	}
}

// this is called once every 500ms
void APP_TimeSlice500ms(void)
{
	// Skipped authentic device check

	if (gKeypadLocked > 0)
		if (--gKeypadLocked == 0)
			gUpdateDisplay = true;

	if (gKeyInputCountdown > 0)
		if (--gKeyInputCountdown == 0)
			cancelUserInputModes();

	// Skipped authentic device check

	#ifdef ENABLE_FMRADIO
		if (gFmRadioCountdown_500ms > 0)
		{
			gFmRadioCountdown_500ms--;
			return;
		}
	#endif

	if (gReducedService)
	{
		BOARD_ADC_GetBatteryInfo(&gBatteryCurrentVoltage, &gBatteryCurrent);
		if (gBatteryCurrent > 500 || gBatteryCalibration[3] < gBatteryCurrentVoltage)
			overlay_FLASH_RebootToBootloader();
		return;
	}

	gBatteryCheckCounter++;

	// Skipped authentic device check

	if (gCurrentFunction != FUNCTION_TRANSMIT)
	{
		if ((gBatteryCheckCounter & 1) == 0)
		{
			BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryVoltageIndex++], &gBatteryCurrent);

			if (gBatteryVoltageIndex > 3)
				gBatteryVoltageIndex = 0;

			BATTERY_GetReadings(true);
		}

		if (gCurrentFunction != FUNCTION_POWER_SAVE)
		{
			gCurrentRSSI = BK4819_GetRSSI();
			UI_UpdateRSSI(gCurrentRSSI);
		}

		#ifdef ENABLE_FMRADIO
			if ((gFM_ScanState == FM_SCAN_OFF || gAskToSave) && gCssScanMode == CSS_SCAN_MODE_OFF)
		#else
			if (gCssScanMode == CSS_SCAN_MODE_OFF)
		#endif
		{
			if (gBacklightCountdown > 0)
				if (--gBacklightCountdown == 0)
					if (gEeprom.BACKLIGHT < 5)
						GPIO_ClearBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);   // turn backlight off

			#ifdef ENABLE_AIRCOPY
				if (gScanState == SCAN_OFF && gScreenToDisplay != DISPLAY_AIRCOPY && (gScreenToDisplay != DISPLAY_SCANNER || gScanCssState >= SCAN_CSS_STATE_FOUND))
			#else
				if (gScanState == SCAN_OFF && (gScreenToDisplay != DISPLAY_SCANNER || gScanCssState >= SCAN_CSS_STATE_FOUND))
			#endif
			{
				if (gEeprom.AUTO_KEYPAD_LOCK && gKeyLockCountdown > 0 && !gDTMF_InputMode)
				{
					if (--gKeyLockCountdown == 0)
						gEeprom.KEY_LOCK = true;     // lock the keyboard

					gUpdateStatus = true;            // lock symbol needs showing
				}

				if (gVoltageMenuCountdown > 0)
				{
					if (--gVoltageMenuCountdown == 0)
					{
						if (gInputBoxIndex > 0 || gDTMF_InputMode || gScreenToDisplay == DISPLAY_MENU)
							AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);

						if (gScreenToDisplay == DISPLAY_SCANNER)
						{
							BK4819_StopScan();

							RADIO_ConfigureChannel(0, 2);
							RADIO_ConfigureChannel(1, 2);

							RADIO_SetupRegisters(true);
						}

						gWasFKeyPressed  = false;
						gUpdateStatus    = true;
						gInputBoxIndex   = 0;
						gDTMF_InputMode  = false;
						gDTMF_InputIndex = 0;
						gAskToSave       = false;
						gAskToDelete     = false;

						#ifdef ENABLE_FMRADIO
							if (gFmRadioMode && gCurrentFunction != FUNCTION_RECEIVE && gCurrentFunction != FUNCTION_MONITOR && gCurrentFunction != FUNCTION_TRANSMIT)
								GUI_SelectNextDisplay(DISPLAY_FM);
						else
						#endif
							GUI_SelectNextDisplay(DISPLAY_MAIN);
					}
				}
			}
		}

	}

	#ifdef ENABLE_FMRADIO
		if (!gPttIsPressed && gFM_ResumeCountdown_500ms > 0)
		{
			if (--gFM_ResumeCountdown_500ms == 0)
			{
				RADIO_SetVfoState(VFO_STATE_NORMAL);
				if (gCurrentFunction != FUNCTION_RECEIVE && gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_MONITOR && gFmRadioMode)
				{
					FM_Start();
					GUI_SelectNextDisplay(DISPLAY_FM);
				}
			}
		}
	#endif

	if (gLowBattery)
	{
		gLowBatteryBlink = ++gLowBatteryCountdown & 1;

		UI_DisplayBattery(gLowBatteryCountdown);

		if (gCurrentFunction != FUNCTION_TRANSMIT)
		{
			if (gLowBatteryCountdown < 30)
			{
				if (gLowBatteryCountdown == 29 && !gChargingWithTypeC)
					AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP);
			}
			else
			{
				gLowBatteryCountdown = 0;

				if (!gChargingWithTypeC)
				{
					AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP);

					#ifdef ENABLE_VOICE
						AUDIO_SetVoiceID(0, VOICE_ID_LOW_VOLTAGE);
					#endif

					if (gBatteryDisplayLevel == 0)
					{
						#ifdef ENABLE_VOICE
							AUDIO_PlaySingleVoice(true);
						#endif

						gReducedService = true;

						FUNCTION_Select(FUNCTION_POWER_SAVE);

						ST7565_Configure_GPIO_B11();

						//if (gEeprom.BACKLIGHT < 5)
							GPIO_ClearBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);
					}
					#ifdef ENABLE_VOICE
						else
							AUDIO_PlaySingleVoice(false);
					#endif
				}
			}
		}
	}

	if (gScreenToDisplay == DISPLAY_SCANNER && gScannerEditState == 0 && gScanCssState < SCAN_CSS_STATE_FOUND)
	{
		if (++gScanProgressIndicator > 32)
		{
			if (gScanCssState == SCAN_CSS_STATE_SCANNING && !gScanSingleFrequency)
				gScanCssState = SCAN_CSS_STATE_FOUND;
			else
				gScanCssState = SCAN_CSS_STATE_FAILED;

			gUpdateStatus = true;
		}

		gUpdateDisplay = true;
	}

	if (gDTMF_CallState != DTMF_CALL_STATE_NONE && gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_RECEIVE)
	{
		if (gDTMF_AUTO_RESET_TIME > 0)
		{
			if (--gDTMF_AUTO_RESET_TIME == 0)
			{
				gDTMF_CallState = DTMF_CALL_STATE_NONE;
				gUpdateDisplay  = true;
			}
		}

		if (gDTMF_DecodeRing && gDTMF_DecodeRingCountdown_500ms > 0)
		{
			if ((--gDTMF_DecodeRingCountdown_500ms % 3) == 0)
				AUDIO_PlayBeep(BEEP_440HZ_500MS);

			if (gDTMF_DecodeRingCountdown_500ms == 0)
				gDTMF_DecodeRing = false;
		}
	}

	if (gDTMF_IsTx && gDTMF_TxStopCountdown_500ms > 0)
	{
		if (--gDTMF_TxStopCountdown_500ms == 0)
		{
			gDTMF_IsTx     = false;
			gUpdateDisplay = true;
		}
	}

	if (gDTMF_RecvTimeout > 0)
	{
		if (--gDTMF_RecvTimeout == 0)
		{
			gDTMF_WriteIndex = 0;
			memset(gDTMF_Received, 0, sizeof(gDTMF_Received));
		}
	}

	#ifdef ENABLE_DTMF_DECODER
		if (gDTMF_RecvTimeoutSaved > 0)
		{
			if (--gDTMF_RecvTimeoutSaved == 0)
			{
				gDTMF_ReceivedSaved[0] = '\0';
				gUpdateDisplay = true;
			}
		}
	#endif
}

#ifdef ENABLE_ALARM
	static void ALARM_Off(void)
	{
		gAlarmState = ALARM_STATE_OFF;
		GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
		gEnableSpeaker = false;

		if (gEeprom.ALARM_MODE == ALARM_MODE_TONE)
		{
			RADIO_SendEndOfTransmission();
			RADIO_EnableCxCSS();
		}

		gVoxResumeCountdown = 80;

		SYSTEM_DelayMs(5);

		RADIO_SetupRegisters(true);

		gRequestDisplayScreen = DISPLAY_MAIN;
	}
#endif

void CHANNEL_Next(bool bFlag, int8_t Direction)
{
	RADIO_SelectVfos();

	gNextMrChannel   = gRxVfo->CHANNEL_SAVE;
	gCurrentScanList = 0;
	gScanState       = Direction;

	if (IS_MR_CHANNEL(gNextMrChannel))
	{
		if (bFlag)
			gRestoreMrChannel = gNextMrChannel;
		MR_NextChannel();
	}
	else
	{
		if (bFlag)
			gRestoreFrequency = gRxVfo->ConfigRX.Frequency;
		FREQ_NextChannel();
	}

	ScanPauseDelayIn_10ms = 50;    // 500ms
	gScheduleScanListen   = false;
	gRxReceptionMode      = RX_MODE_NONE;
	gScanPauseMode        = false;
	bScanKeepFrequency    = false;
}

static void APP_ProcessKey(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	bool bFlag = false;

//	const bool backlight_was_on = (gBacklightCountdown > 0 || gEeprom.BACKLIGHT >= 5);
	const bool backlight_was_on = GPIO_CheckBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);

	if (Key == KEY_EXIT && !backlight_was_on)
	{	// just turn the light on for now
		BACKLIGHT_TurnOn();
		gBeepToPlay = BEEP_NONE;
		return;
	}

	if (gCurrentFunction == FUNCTION_POWER_SAVE)
		FUNCTION_Select(FUNCTION_FOREGROUND);

	gBatterySaveCountdown_10ms = battery_save_count_10ms;

	if (gEeprom.AUTO_KEYPAD_LOCK)
		gKeyLockCountdown = 30;     // 15 seconds

	#ifdef ENABLE_DTMF_DECODER
		if (Key == KEY_EXIT && bKeyPressed && bKeyHeld && gDTMF_ReceivedSaved[0] > 0)
		{	// clear the live DTMF decoder if the EXIT key is held
			gDTMF_RecvTimeoutSaved = 0;
			gDTMF_ReceivedSaved[0] = '\0';
			gUpdateDisplay         = true;
		}
	#endif

	if (!bKeyPressed)
	{
		if (gFlagSaveVfo)
		{
			SETTINGS_SaveVfoIndices();
			gFlagSaveVfo = false;
		}

		if (gFlagSaveSettings)
		{
			SETTINGS_SaveSettings();
			gFlagSaveSettings = false;
		}

		#ifdef ENABLE_FMRADIO
			if (gFlagSaveFM)
			{
				SETTINGS_SaveFM();
				gFlagSaveFM = false;
			}
		#endif

		if (gFlagSaveChannel)
		{
			SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, gEeprom.TX_CHANNEL, gTxVfo, gFlagSaveChannel);
			gFlagSaveChannel = false;
			RADIO_ConfigureChannel(gEeprom.TX_CHANNEL, 1);
			RADIO_SetupRegisters(true);
			GUI_SelectNextDisplay(DISPLAY_MAIN);
		}
	}
	else
	{
		if (Key != KEY_PTT)
			gVoltageMenuCountdown = menu_timeout_500ms;

		BACKLIGHT_TurnOn();

		if (gDTMF_DecodeRing)
		{
			gDTMF_DecodeRing = false;
			AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL);
			if (Key != KEY_PTT)
			{
				gPttWasReleased = true;
				return;
			}
		}
	}

	if (gEeprom.KEY_LOCK && gCurrentFunction != FUNCTION_TRANSMIT && Key != KEY_PTT)
	{	// keyboard is locked

		if (Key == KEY_F)
		{	// function/key-lock key

			if (!bKeyPressed)
				return;

			if (!bKeyHeld)
			{
				// keypad is locked, tell the user
				AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
				gKeypadLocked  = 4;      // 2 seconds
				gUpdateDisplay = true;
				return;
			}
		}
		else
		if (Key != KEY_SIDE1 && Key != KEY_SIDE2)
		{
			if (!bKeyPressed || bKeyHeld)
				return;

			// keypad is locked, tell the user
			AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
			gKeypadLocked  = 4;          // 2 seconds
			gUpdateDisplay = true;
			return;
		}
	}

	if (gScanState != SCAN_OFF &&
	    Key != KEY_PTT  &&
	    Key != KEY_UP   &&
		Key != KEY_DOWN &&
		Key != KEY_EXIT &&
		Key != KEY_STAR)
	{	// scanning
		if (bKeyPressed && !bKeyHeld)
			AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
		return;
	}

	if (gCssScanMode != CSS_SCAN_MODE_OFF &&
		Key != KEY_PTT  &&
		Key != KEY_UP   &&
		Key != KEY_DOWN &&
		Key != KEY_EXIT &&
		Key != KEY_STAR &&
		Key != KEY_MENU)
	{	// code scanning
		if (bKeyPressed && !bKeyHeld)
			AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
		return;
	}

	if (gPttWasPressed && Key == KEY_PTT)
	{
		bFlag = bKeyHeld;
		if (!bKeyPressed)
		{
			bFlag          = true;
			gPttWasPressed = false;
		}
	}

	if (gPttWasReleased && Key != KEY_PTT)
	{
		if (bKeyHeld)
			bFlag = true;

		if (!bKeyPressed)
		{
			bFlag           = true;
			gPttWasReleased = false;
		}
	}

	if (gWasFKeyPressed && Key > KEY_9 && Key != KEY_F && Key != KEY_STAR)
	{
		gWasFKeyPressed = false;
		gUpdateStatus   = true;
	}

	if (gF_LOCK && (Key == KEY_PTT || Key == KEY_SIDE2 || Key == KEY_SIDE1))
		return;

	if (!bFlag)
	{
		if (gCurrentFunction == FUNCTION_TRANSMIT)
		{	// transmitting

			#ifdef ENABLE_ALARM
				if (gAlarmState == ALARM_STATE_OFF)
			#endif
			{
				if (Key == KEY_PTT)
				{
					GENERIC_Key_PTT(bKeyPressed);
				}
				else
				{
					char Code;

					if (Key == KEY_SIDE2)
					{
						Code = 0xFE;
					}
					else
					{
						Code = DTMF_GetCharacter(Key);
						if (Code == 0xFF)
							goto Skip;
					}

					if (!bKeyPressed || bKeyHeld)
					{
						if (!bKeyPressed)
						{
							GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);

							gEnableSpeaker = false;

							BK4819_ExitDTMF_TX(false);

							if (gCurrentVfo->SCRAMBLING_TYPE == 0 || !gSetting_ScrambleEnable)
								BK4819_DisableScramble();
							else
								BK4819_EnableScramble(gCurrentVfo->SCRAMBLING_TYPE - 1);
						}
					}
					else
					{
						if (gEeprom.DTMF_SIDE_TONE)
						{
							GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
							gEnableSpeaker = true;
						}

						BK4819_DisableScramble();

						if (Code == 0xFE)
							BK4819_TransmitTone(gEeprom.DTMF_SIDE_TONE, 1750);
						else
							BK4819_PlayDTMFEx(gEeprom.DTMF_SIDE_TONE, Code);
					}
				}
			}
			#ifdef ENABLE_ALARM
				else
				if (!bKeyHeld && bKeyPressed)
				{
					ALARM_Off();

					if (gEeprom.REPEATER_TAIL_TONE_ELIMINATION == 0)
						FUNCTION_Select(FUNCTION_FOREGROUND);
					else
						gRTTECountdown = gEeprom.REPEATER_TAIL_TONE_ELIMINATION * 10;

					if (Key == KEY_PTT)
						gPttWasPressed  = true;
					else
						gPttWasReleased = true;
				}
			#endif
		}
		else
		if (Key != KEY_SIDE1 && Key != KEY_SIDE2)
		{
			switch (gScreenToDisplay)
			{
				case DISPLAY_MAIN:
					MAIN_ProcessKeys(Key, bKeyPressed, bKeyHeld);
					#ifdef ENABLE_MAIN_KEY_HOLD
						bKeyHeld = false;	// allow the channel setting to be saved
					#endif
					break;

				#ifdef ENABLE_FMRADIO
					case DISPLAY_FM:
						FM_ProcessKeys(Key, bKeyPressed, bKeyHeld);
						break;
				#endif

				case DISPLAY_MENU:
					MENU_ProcessKeys(Key, bKeyPressed, bKeyHeld);
					break;

				case DISPLAY_SCANNER:
					SCANNER_ProcessKeys(Key, bKeyPressed, bKeyHeld);
					break;

				#ifdef ENABLE_AIRCOPY
					case DISPLAY_AIRCOPY:
						AIRCOPY_ProcessKeys(Key, bKeyPressed, bKeyHeld);
						break;
				#endif

				case DISPLAY_INVALID:
				default:
					break;
			}
		}
		else
		#ifdef ENABLE_AIRCOPY
			if (gScreenToDisplay != DISPLAY_SCANNER && gScreenToDisplay != DISPLAY_AIRCOPY)
		#else
			if (gScreenToDisplay != DISPLAY_SCANNER)
		#endif
		{
			ACTION_Handle(Key, bKeyPressed, bKeyHeld);
		}
		else
		if (!bKeyHeld && bKeyPressed)
			gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
	}
	else
	{
		if (Key == KEY_EXIT && bKeyHeld)
			cancelUserInputModes();
	}

Skip:
	if (gBeepToPlay != BEEP_NONE)
	{
		AUDIO_PlayBeep(gBeepToPlay);
		gBeepToPlay = BEEP_NONE;
	}

	if (gFlagAcceptSetting)
	{
		MENU_AcceptSetting();

		gFlagRefreshSetting = true;
		gFlagAcceptSetting  = false;
	}

	if (gFlagStopScan)
	{
		BK4819_StopScan();

		gFlagStopScan = false;
	}

	if (gRequestSaveSettings)
	{
		if (!bKeyHeld)
			SETTINGS_SaveSettings();
		else
			gFlagSaveSettings = 1;
		gRequestSaveSettings = false;
		gUpdateStatus        = true;
	}

	#ifdef ENABLE_FMRADIO
		if (gRequestSaveFM)
		{
			if (!bKeyHeld)
				SETTINGS_SaveFM();
			else
				gFlagSaveFM = true;
			gRequestSaveFM = false;
		}
	#endif

	if (gRequestSaveVFO)
	{
		if (!bKeyHeld)
			SETTINGS_SaveVfoIndices();
		else
			gFlagSaveVfo = true;
		gRequestSaveVFO = false;
	}

	if (gRequestSaveChannel > 0)
	{
		if (!bKeyHeld)
		{
			SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, gEeprom.TX_CHANNEL, gTxVfo, gRequestSaveChannel);
			if (gScreenToDisplay != DISPLAY_SCANNER)
				gVfoConfigureMode = VFO_CONFIGURE_1;
		}
		else
		{
			gFlagSaveChannel = gRequestSaveChannel;
			if (gRequestDisplayScreen == DISPLAY_INVALID)
				gRequestDisplayScreen = DISPLAY_MAIN;
		}

		gRequestSaveChannel = 0;
	}


	if (gVfoConfigureMode != VFO_CONFIGURE_0)
	{
		if (gFlagResetVfos)
		{
			RADIO_ConfigureChannel(0, gVfoConfigureMode);
			RADIO_ConfigureChannel(1, gVfoConfigureMode);
		}
		else
			RADIO_ConfigureChannel(gEeprom.TX_CHANNEL, gVfoConfigureMode);

		if (gRequestDisplayScreen == DISPLAY_INVALID)
			gRequestDisplayScreen = DISPLAY_MAIN;

		gFlagReconfigureVfos = true;
		gVfoConfigureMode    = VFO_CONFIGURE_0;
		gFlagResetVfos       = false;
	}

	if (gFlagReconfigureVfos)
	{
		RADIO_SelectVfos();

		#ifdef ENABLE_NOAA
			RADIO_ConfigureNOAA();
		#endif

		RADIO_SetupRegisters(true);

		gDTMF_AUTO_RESET_TIME       = 0;
		gDTMF_CallState             = DTMF_CALL_STATE_NONE;
		gDTMF_TxStopCountdown_500ms = 0;
		gDTMF_IsTx                  = false;

		gVFO_RSSI_Level[0]    = 0;
		gVFO_RSSI_Level[1]    = 0;

		gFlagReconfigureVfos  = false;
	}

	if (gFlagRefreshSetting)
	{
		MENU_ShowCurrentSetting();
		gFlagRefreshSetting = false;
	}

	if (gFlagStartScan)
	{
		#ifdef ENABLE_VOICE
			AUDIO_SetVoiceID(0, VOICE_ID_SCANNING_BEGIN);
			AUDIO_PlaySingleVoice(true);
		#endif

		SCANNER_Start();

		gRequestDisplayScreen = DISPLAY_SCANNER;
		gFlagStartScan        = false;
	}

	if (gFlagPrepareTX)
	{
		RADIO_PrepareTX();
		gFlagPrepareTX = false;
	}

	#ifdef ENABLE_VOICE
		if (gAnotherVoiceID != VOICE_ID_INVALID)
		{
			if (gAnotherVoiceID < 76)
				AUDIO_SetVoiceID(0, gAnotherVoiceID);
			AUDIO_PlaySingleVoice(false);
			gAnotherVoiceID = VOICE_ID_INVALID;
		}
	#endif

	GUI_SelectNextDisplay(gRequestDisplayScreen);

	gRequestDisplayScreen = DISPLAY_INVALID;
}
