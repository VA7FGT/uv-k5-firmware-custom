/* Copyright 2023 fagci
 * https://github.com/fagci
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


/*

   )                     )      )  
( /(   (        (     ( /(   ( /(  
)\())  )\       )\    )\())  )\()) 
((_)\((((_)(   (((_) |((_)\  ((_)\  
_((_))\ _ )\  )\___ |_ ((_)__ ((_) 
| || |(_)_\(_)((/ __|| |/ / \ \ / / 
| __ | / _ \   | (__   ' <   \ V /  
|_||_|/_/ (_\   \___| _|\_\   |_|   
   (      )\ )                      
   )\    (()/(                      
((((_)(   /(_))                     
 )\ _ )\ (_))                       
 (_)_\(_)/ __|                      
  / _ \  \__ \                      
 (_/ \_\ |___/          )           
 )\ )          (     ( /(           
(()/(    (     )\    )\())          
 /(_))   )\  (((_) |((_)\           
(_))_|_ ((_) )\___ |_ ((_)          
| |_ | | | |((/ __|| |/ /           
| __|| |_| | | (__   ' <            
|_|   \___/   \___| _|\_\           

// Yeah most of this is duplicated from spectrum
// Yeah the conflicting vars are prefixed with W
// Yeah this whole thing was a half baked idea 

// lol
// Jane, VA7FGT

*/

#ifndef WATERFALL_H
#define WATERFALL_H 

#include "../bitmaps.h"
#include "../board.h"
#include "../bsp/dp32g030/gpio.h"
#include "../driver/bk4819-regs.h"
#include "../driver/bk4819.h"
#include "../driver/gpio.h"
#include "../driver/keyboard.h"
#include "../driver/st7565.h"
#include "../driver/system.h"
#include "../driver/systick.h"
#include "../external/printf/printf.h"
#include "../font.h"
#include "../helper/battery.h"
#include "../misc.h"
#include "../radio.h"
#include "../settings.h"
#include "../ui/helper.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const uint8_t WDrawingEndY = 40;

static const uint8_t WU8RssiMap[] = {
    121, 115, 109, 103, 97, 91, 85, 79, 73, 63,
};

static const uint16_t WscanStepValues[] = {
    1, 10, 50, 100, 250, 500, 625, 833, 
    1000, 1250, 1500, 2000, 2500, 5000, 10000,
};

static const uint16_t WscanStepBWRegValues[] = {
    //     RX  RXw TX  BW
    // 0b0 000 000 001 01 1000
    // 1
    0b0000000001011000, // 6.25
    // 10
    0b0000000001011000, // 6.25
    // 50
    0b0000000001011000, // 6.25
    // 100
    0b0000000001011000, // 6.25
    // 250
    0b0000000001011000, // 6.25
    // 500
    0b0010010001011000, // 6.25
    // 625
    0b0100100001011000, // 6.25
    // 833
    0b0110110001001000, // 6.25
    // 1000
    0b0110110001001000, // 6.25
    // 1250
    0b0111111100001000, // 6.25
    // 2500
    0b0011011000101000, // 25
    // 10000
    0b0011011000101000, // 25
};

static const uint16_t WlistenBWRegValues[] = {
    0b0011011000101000, // 25
    0b0111111100001000, // 12.5
    0b0100100001011000, // 6.25
};

typedef enum WState {
  WSPECTRUM,
  WFREQ_INPUT,
  WSTILL,
} WState;

typedef enum WStepsCount {
  WSTEPS_128,
  WSTEPS_64,
  WSTEPS_32,
  WSTEPS_16,
} WStepsCount;

typedef enum WScanStep {
  WS_STEP_0_01kHz,
  WS_STEP_0_1kHz,
  WS_STEP_0_5kHz,
  WS_STEP_1_0kHz,

  WS_STEP_2_5kHz,
  WS_STEP_5_0kHz,
  WS_STEP_6_25kHz,
  WS_STEP_8_33kHz,
  WS_STEP_10_0kHz,
  WS_STEP_12_5kHz,
  WS_STEP_15_0kHz,
  WS_STEP_20_0kHz,
  WS_STEP_25_0kHz,
  WS_STEP_50_0kHz,
  WS_STEP_100_0kHz,
} WScanStep;

typedef struct WSpectrumSettings {
  uint32_t frequencyChangeStep;  
  WStepsCount stepsCount;
  WScanStep scanStepIndex;
  uint16_t scanDelay;
  uint16_t rssiTriggerLevel;
  BK4819_FilterBandwidth_t bw;
  BK4819_FilterBandwidth_t listenBw;
  int dbMin;
  int dbMax;  
  ModulationMode_t modulationType;
  bool backlightState;
} WSpectrumSettings;

typedef struct WKeyboardState {
  KEY_Code_t current;
  KEY_Code_t prev;
  uint8_t counter;
} WKeyboardState;

typedef struct WScanInfo {
  uint16_t rssi, rssiMin, rssiMax;
  uint16_t i, iPeak;
  uint32_t f, fPeak;
  uint16_t scanStep;
  uint16_t measurementsCount;
} WScanInfo;

typedef struct WPeakInfo {
  uint16_t t;
  uint16_t rssi;
  uint32_t f;
  uint16_t i;
} WPeakInfo;

void APP_RunWaterfall(void);

#endif /* ifndef WATERFALL_H */

// vim: ft=c