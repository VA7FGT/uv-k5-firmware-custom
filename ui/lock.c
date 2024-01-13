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

#ifdef ENABLE_PWRON_PASSWORD

#include <string.h>

#include "ARMCM0.h"
#include "app/uart.h"
#include "audio.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "misc.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/inputbox.h"
#include "ui/lock.h"
#include "board.h"

static void Render(void)
{
	unsigned int i;
	char         String[5];

	memset(gStatusLine,  0, sizeof(gStatusLine));
	memset(gFrameBuffer, 0, sizeof(gFrameBuffer));

	strcpy(String, "LOCK");
	UI_PrintString(String, 0, 127, 1, 10);
	for (i = 0; i < 4; i++)
		String[i] = (gInputBox[i] == 10) ? '-' : '*';
	String[6] = 0;
	UI_PrintString(String, 0, 127, 3, 12);

	ST7565_BlitStatusLine();
	ST7565_BlitFullScreen();
}

void UI_DisplayLock(void)
{
	KEY_Code_t  Key;
	BEEP_Type_t Beep;

	gUpdateDisplay = true;

	memset(gInputBox, 10, sizeof(gInputBox));

	while (1)
	{
		while (!gNextTimeslice) {}

		// TODO: Original code doesn't do the below, but is needed for proper key debounce

		gNextTimeslice = false;

		Key = KEYBOARD_Poll();
		if (gEeprom.PASSWORD_WRONG_ATTEMPTS >= PASSWORD_MAX_RETRIES)
		{
			BOARD_FactoryReset(true);
			return;
		}
		if (gKeyReading0 == Key)
		{
			if (++gDebounceCounter == key_debounce_10ms)
			{
				if (Key == KEY_INVALID)
				{
					gKeyReading1 = KEY_INVALID;
				}
				else
				{
					gKeyReading1 = Key;

					switch (Key)
					{
						case KEY_0:
						case KEY_1:
						case KEY_2:
						case KEY_3:
						case KEY_4:
						case KEY_5:
						case KEY_6:
						case KEY_7:
						case KEY_8:
						case KEY_9:
							INPUTBOX_Append(Key - KEY_0);

							if (gInputBoxIndex < 4)   // 4 frequency digits
							{
								Beep = BEEP_1KHZ_60MS_OPTIONAL;
							}
							else
							{
								uint32_t Password;

								gInputBoxIndex = 0;
								Password = StrToUL(INPUTBOX_GetAscii());

								if ((gEeprom.POWER_ON_PASSWORD) == Password)
								{
									AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL);
									gEeprom.PASSWORD_WRONG_ATTEMPTS = 0;
									return;
								}
								else
								{
									gEeprom.PASSWORD_WRONG_ATTEMPTS++;
								}

								SETTINGS_SaveSettings();

								memset(gInputBox, 10, sizeof(gInputBox));

								Beep = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
							}

							AUDIO_PlayBeep(Beep);

							gUpdateDisplay = true;
							break;

						case KEY_EXIT:
							if (gInputBoxIndex > 0)
							{
								gInputBox[--gInputBoxIndex] = 10;
								gUpdateDisplay = true;
							}

							AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL);

						default:
							break;
					}
				}

				gKeyBeingHeld = false;
			}
		}
		else
		{
			gDebounceCounter = 0;
			gKeyReading0     = Key;
		}

		if (gUpdateDisplay)
		{
			Render();
			gUpdateDisplay = false;
		}
	}
}

#endif
