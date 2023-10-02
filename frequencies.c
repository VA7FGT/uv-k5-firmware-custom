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

#include "frequencies.h"
#include "misc.h"
#include "settings.h"

// the BK4819 has 2 bands it covers, 18MHz ~ 630MHz and 760MHz ~ 1300MHz
const freq_band_table_t BX4819_band1 = { 1800000,  63000000};
const freq_band_table_t BX4819_band2 = {84000000, 130000000};

const freq_band_table_t frequencyBandTable[7] =
{
	#ifndef ENABLE_WIDE_RX
		// QS original
		{ 5000000,   7600000},
		{10800000,  13600000},
		{13600000,  17400000},
		{17400000,  35000000},
		{35000000,  40000000},
		{40000000,  47000000},
		{47000000,  60000000}
	#else
		// extended range
		{ 1800000,  10800000},
		{10800000,  13600000},
		{13600000,  17400000},
		{17400000,  35000000},
		{35000000,  40000000},
		{40000000,  47000000},
		{47000000, 130000000}
	#endif
};

#ifdef ENABLE_NOAA
	const uint32_t NoaaFrequencyTable[10] =
	{
		16255000,
		16240000,
		16247500,
		16242500,
		16245000,
		16250000,
		16252500,
		16152500,
		16177500,
		16327500
	};
#endif

#ifndef ENABLE_12_5KHZ_STEP
	// QS steps (*10 Hz)
	const uint16_t StepFrequencyTable[7] = {250, 500, 625, 1000, 1250, 2500, 833};
#else
	// includes 1.25kHz step
	const uint16_t StepFrequencyTable[7] = {125, 250, 625, 1000, 1250, 2500, 833};
#endif

FREQUENCY_Band_t FREQUENCY_GetBand(uint32_t Frequency)
{
	int band;
	for (band = ARRAY_SIZE(frequencyBandTable) - 1; band >= 0; band--)
		if (Frequency >= frequencyBandTable[band].lower)
//		if (Frequency <  frequencyBandTable[band].upper)
			return (FREQUENCY_Band_t)band;

	return BAND1_50MHz;
//	return BAND_NONE;
}

uint8_t FREQUENCY_CalculateOutputPower(uint8_t TxpLow, uint8_t TxpMid, uint8_t TxpHigh, int32_t LowerLimit, int32_t Middle, int32_t UpperLimit, int32_t Frequency)
{
	if (Frequency <= LowerLimit)
		return TxpLow;

	if (UpperLimit <= Frequency)
		return TxpHigh;

	if (Frequency <= Middle)
	{
		TxpMid += ((TxpMid - TxpLow) * (Frequency - LowerLimit)) / (Middle - LowerLimit);
		return TxpMid;
	}

	TxpMid += ((TxpHigh - TxpMid) * (Frequency - Middle)) / (UpperLimit - Middle);

	return TxpMid;
}

uint32_t FREQUENCY_FloorToStep(uint32_t Upper, uint32_t Step, uint32_t Lower)
{
	uint32_t Index;

	if (Step == 833)
	{
		const uint32_t Delta = Upper - Lower;
		uint32_t       Base  = (Delta / 2500) * 2500;
		const uint32_t Index = ((Delta - Base) % 2500) / 833;

		if (Index == 2)
			Base++;

		return Lower + Base + (Index * 833);
	}

	Index = (Upper - Lower) / Step;

	return Lower + (Step * Index);
}

int TX_freq_check(const uint32_t Frequency)
{	// return '0' if TX frequency is allowed
	// otherwise return '-1'

	if (Frequency < frequencyBandTable[0].lower || Frequency > frequencyBandTable[ARRAY_SIZE(frequencyBandTable) - 1].upper)
		return -1;  // not allowed outside this range

	if (Frequency >= BX4819_band1.upper && Frequency < BX4819_band2.lower)
		return -1;  // BX chip does not work in this range

	switch (gSetting_F_LOCK)
	{
		case F_LOCK_OFF:
			if (Frequency >= 13600000 && Frequency < 17400000)
				return 0;
			if (Frequency >= 17400000 && Frequency < 35000000)
				if (gSetting_200TX)
					return 0;
			if (Frequency >= 35000000 && Frequency < 40000000)
				if (gSetting_350TX && gSetting_350EN)
					return 0;
			if (Frequency >= 40000000 && Frequency < 47000000)
				return 0;
			if (Frequency >= 47000000 && Frequency <= 60000000)
				if (gSetting_500TX)
					return 0;
			break;

		case F_LOCK_FCC:
			if (Frequency >= 14400000 && Frequency < 14800000)
				return 0;
			if (Frequency >= 42000000 && Frequency < 45000000)
				return 0;
			break;

		case F_LOCK_CE:
			if (Frequency >= 14400000 && Frequency < 14600000)
				return 0;
			if (Frequency >= 43000000 && Frequency < 44000000)
				return 0;
			break;

		case F_LOCK_GB:
			if (Frequency >= 14400000 && Frequency < 14800000)
				return 0;
			if (Frequency >= 43000000 && Frequency < 44000000)
				return 0;
			break;

		case F_LOCK_430:
			if (Frequency >= 13600000 && Frequency < 17400000)
				return 0;
			if (Frequency >= 40000000 && Frequency < 43000000)
				return 0;
			break;

		case F_LOCK_438:
			if (Frequency >= 13600000 && Frequency < 17400000)
				return 0;
			if (Frequency >= 40000000 && Frequency < 43800000)
				return 0;
			break;
	}

	// dis-allowed TX frequency
	return -1;
}

int RX_freq_check(const uint32_t Frequency)
{	// return '0' if RX frequency is allowed
	// otherwise return '-1'

	if (Frequency < frequencyBandTable[0].lower || Frequency > frequencyBandTable[ARRAY_SIZE(frequencyBandTable) - 1].upper)
		return -1;

	if (Frequency >= BX4819_band1.upper && Frequency < BX4819_band2.lower)
		return -1;

	return 0;   // OK frequency
}
