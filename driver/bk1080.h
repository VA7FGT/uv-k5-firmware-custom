/* Original work Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Modified work Copyright 2024 kamilsss655
 * https://github.com/kamilsss655
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

#ifndef DRIVER_BK1080_H
#define DRIVER_BK1080_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/bk1080-regs.h"

extern uint16_t BK1080_BaseFrequency;
extern uint16_t BK1080_FrequencyDeviation;

void BK1080_Init(uint16_t Frequency, bool bDoScan);
uint16_t BK1080_ReadRegister(BK1080_Register_t Register);
void BK1080_WriteRegister(BK1080_Register_t Register, uint16_t Value);
void BK1080_Mute(bool Mute);
void BK1080_SetFrequency(uint16_t Frequency);
void BK1080_GetFrequencyDeviation(uint16_t Frequency);
void BK1080_TuneNext(bool direction);
uint16_t BK1080_GetFrequency();

#endif

