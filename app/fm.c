/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Copyright 2024 kamilsss655
 * https://github.com/kamilsss655
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

#ifdef ENABLE_FMRADIO

// #include <string.h>

#include "app/action.h"
#include "app/fm.h"
#include "app/generic.h"
#include "audio.h"
#include "driver/bk1080.h"
#include "misc.h"
#include "settings.h"
#include "ui/ui.h"

bool              gFmRadioMode;
uint8_t           gFmRadioCountdown_500ms;
volatile uint16_t gFmPlayCountdown_10ms;
volatile int8_t   gFM_ScanState;
uint8_t           gFM_ChannelPosition;
bool              gFM_FoundFrequency;
uint16_t          gFM_RestoreCountdown_10ms;



const uint8_t BUTTON_STATE_PRESSED = 1 << 0;
const uint8_t BUTTON_STATE_HELD = 1 << 1;

const uint8_t BUTTON_EVENT_PRESSED = BUTTON_STATE_PRESSED;
const uint8_t BUTTON_EVENT_HELD = BUTTON_STATE_PRESSED | BUTTON_STATE_HELD;
const uint8_t BUTTON_EVENT_SHORT =  0;
const uint8_t BUTTON_EVENT_LONG =  BUTTON_STATE_HELD;

void FM_TurnOff(void)
{
	gFmRadioMode              = false;
	gFM_ScanState             = FM_SCAN_OFF;
	gFM_RestoreCountdown_10ms = 0;

	AUDIO_AudioPathOff();

	gEnableSpeaker = false;

	BK1080_Init(0, false);

	gUpdateStatus  = true;
}

void FM_Tune(uint16_t Frequency, int8_t Step, bool bFlag)
{
	AUDIO_AudioPathOff();

	gEnableSpeaker = false;

	gFmPlayCountdown_10ms = (gFM_ScanState == FM_SCAN_OFF) ? fm_play_countdown_noscan_10ms : fm_play_countdown_scan_10ms;

	gScheduleFM                 = false;
	gFM_FoundFrequency          = false;
	gAskToSave                  = false;
	gAskToDelete                = false;
	gEeprom.FM_FrequencyPlaying = Frequency;

	if (!bFlag)
	{
		Frequency += Step;
		if (Frequency < gEeprom.FM_LowerLimit)
			Frequency = gEeprom.FM_UpperLimit;
		else
		if (Frequency > gEeprom.FM_UpperLimit)
			Frequency = gEeprom.FM_LowerLimit;

		gEeprom.FM_FrequencyPlaying = Frequency;
	}

	gFM_ScanState = Step;

	BK1080_SetFrequency(gEeprom.FM_FrequencyPlaying);
}

void FM_PlayAndUpdate(void)
{
	gFM_ScanState = FM_SCAN_OFF;

	BK1080_SetFrequency(gEeprom.FM_FrequencyPlaying);

	gFmPlayCountdown_10ms = 0;
	gScheduleFM           = false;
	gAskToSave            = false;

	AUDIO_AudioPathOn();

	gEnableSpeaker   = true;
}

static void Key_EXIT()
{
	ACTION_FM();
	return;
}

static void Key_UP_DOWN(bool direction)
{
	BK1080_TuneNext(direction);
	gEeprom.FM_FrequencyPlaying = BK1080_GetFrequency();
	// save
	gRequestSaveSettings = true;
}

void FM_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	switch (Key)
	{	
		case KEY_UP:
			Key_UP_DOWN(true);
			break;
		case KEY_DOWN:
			Key_UP_DOWN(false);
			break;;
		case KEY_EXIT:
			Key_EXIT();
			break;
		case KEY_F:
			GENERIC_Key_F(bKeyPressed, bKeyHeld);
			break;
		case KEY_PTT:
			GENERIC_Key_PTT(bKeyPressed);
			break;
		default:
			if (!bKeyHeld && bKeyPressed)
				gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			break;
	}
}

void FM_Play(void)
{
	FM_PlayAndUpdate();
	GUI_SelectNextDisplay(DISPLAY_FM);
}

void FM_Start(void)
{
	gFmRadioMode              = true;
	gFM_ScanState             = FM_SCAN_OFF;
	gFM_RestoreCountdown_10ms = 0;

	BK1080_Init(gEeprom.FM_FrequencyPlaying, true);

	AUDIO_AudioPathOn();

	gEnableSpeaker       = true;
	gUpdateStatus        = true;
}

#endif
