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
#include "app/waterfall.h"
//#include "am_fix.h"     // Not needed with AM being fixed
#include "audio.h"
#include "misc.h"

#ifdef ENABLE_SCAN_RANGES
#include "chFrScanner.h"
#endif

#include "driver/backlight.h"
#include "frequencies.h"
#include "ui/helper.h"
#include "ui/main.h"
#include "radio.h"

struct WFrequencyBandInfo {
  uint32_t lower;
  uint32_t upper;
  uint32_t middle;
};

#define F_MIN frequencyBandTable[0].lower
#define F_MAX frequencyBandTable[BAND7_470MHz].upper  // Hardcode band 7

const uint16_t WRSSI_MAX_VALUE = 65535;

static uint32_t WinitialFreq;
static char String[32];
bool WisInitialized = false;
bool WisListening = true;
bool WmonitorMode = false;
bool WredrawStatus = true;
bool WredrawScreen = false;
bool WnewScanStart = true;
bool WpreventKeypress = true;
bool WaudioState = true;
bool WlockAGC = false;

WState WcurrentState = WSPECTRUM, previousState = WSPECTRUM;

WPeakInfo Wpeak;
WScanInfo WscanInfo;
WKeyboardState Wkbd = {KEY_INVALID, KEY_INVALID, 0};

#ifdef ENABLE_SCAN_RANGES
static uint16_t WblacklistFreqs[15];
static uint8_t WblacklistFreqsIdx;
#endif

const char *WbwOptions[] = {"  25k", "12.5k", "6.25k"};
const uint8_t WmodulationTypeTuneSteps[] = {100, 50, 10};
const uint8_t WmodTypeReg47Values[] = {1, 7, 5};
const int8_t WdBmCorrTable[7] = {
			-15, // band 1
			-25, // band 2
			-20, // band 3
			-4, // band 4
			-7, // band 5
			-6, // band 6
			 -1  // band 7
		};

WSpectrumSettings Wsettings = {.stepsCount = WSTEPS_64,
                             .scanStepIndex = WS_STEP_25_0kHz,
                             .frequencyChangeStep = 80000,
                             .scanDelay = 3200,
                             .rssiTriggerLevel = 150,
                             .backlightState = true,
                             .bw = BK4819_FILTER_BW_WIDE,
                             .listenBw = BK4819_FILTER_BW_WIDE,
                             .modulationType = false,
                             .dbMin = -130,
                             .dbMax = -50};

uint32_t WfMeasure = 0;
uint32_t WcurrentFreq, WtempFreq;
uint16_t WrssiHistory[128];
int Wvfo;
uint8_t WfreqInputIndex = 0;
uint8_t WfreqInputDotIndex = 0;
KEY_Code_t WfreqInputArr[10];
char WfreqInputString[11];

uint8_t WmenuState = 0;
uint16_t WlistenT = 0;

RegisterSpec WregisterSpecs[] = {
    {},
    {"LNAs", BK4819_REG_13, 8, 0b11, 1},
    {"LNA", BK4819_REG_13, 5, 0b111, 1},
    {"PGA", BK4819_REG_13, 0, 0b111, 1},
    {"IF", BK4819_REG_3D, 0, 0xFFFF, 0x2aaa},
    // {"MIX", 0x13, 3, 0b11, 1}, // TODO: hidden
};

uint16_t WstatuslineUpdateTimer = 0;

static uint8_t WDBm2S(int dbm) {
  uint8_t i = 0;
  dbm *= -1;
  for (i = 0; i < ARRAY_SIZE(WU8RssiMap); i++) {
    if (dbm >= WU8RssiMap[i]) {
      return i;
    }
  }
  return i;
}

static int WRssi2DBm(uint16_t rssi) {
  return (rssi / 2) - 160 + WdBmCorrTable[gRxVfo->Band];
}

static uint16_t WGetRegMenuValue(uint8_t st) {
  RegisterSpec s = WregisterSpecs[st];
  return (BK4819_ReadRegister(s.num) >> s.offset) & s.mask;
}

void WLockAGC()
{
  RADIO_SetupAGC(Wsettings.modulationType==MODULATION_AM, WlockAGC);
  WlockAGC = true;
}

static void WSetRegMenuValue(uint8_t st, bool add) {
  uint16_t v = WGetRegMenuValue(st);
  RegisterSpec s = WregisterSpecs[st];

  if(s.num == BK4819_REG_13)
    WLockAGC();

  uint16_t reg = BK4819_ReadRegister(s.num);
  if (add && v <= s.mask - s.inc) {
    v += s.inc;
  } else if (!add && v >= 0 + s.inc) {
    v -= s.inc;
  }
  // TODO: use max value for bits count in max value, or reset by additional
  // mask in spec
  reg &= ~(s.mask << s.offset);
  BK4819_WriteRegister(s.num, reg | (v << s.offset));
  WredrawScreen = true;
}

// GUI functions

static void WPutPixel(uint8_t x, uint8_t y, bool fill) {
  UI_DrawPixelBuffer(gFrameBuffer, x, y, fill);
}
/*
static void WPutPixelStatus(uint8_t x, uint8_t y, bool fill) {
  UI_DrawPixelBuffer(&gStatusLine, x, y, fill);
}*/

static void WDrawVLine(int sy, int ey, int nx, bool fill) {
  for (int i = sy; i <= ey; i++) {
    if (i < 56 && nx < 128) {
      WPutPixel(nx, i, fill);
    }
  }
}

/*
static void GUI_DisplaySmallest(const char *pString, uint8_t x, uint8_t y,
                                bool statusbar, bool fill) {
  uint8_t c;
  uint8_t pixels;
  const uint8_t *p = (const uint8_t *)pString;

  while ((c = *p++) && c != '\0') {
    c -= 0x20;
    for (int i = 0; i < 3; ++i) {
      pixels = gFont3x5[c][i];
      for (int j = 0; j < 6; ++j) {
        if (pixels & 1) {
          if (statusbar)
            WPutPixelStatus(x + i, y + j, fill);
          else
            WPutPixel(x + i, y + j, fill);
        }
        pixels >>= 1;
      }
    }
    x += 4;
  }
}*/

// Utility functions

KEY_Code_t WGetKey() {
  KEY_Code_t btn = KEYBOARD_Poll();
  if (btn == KEY_INVALID && !GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT)) {
    btn = KEY_PTT;
  }
  return btn;
}

static int Wclamp(int v, int min, int max) {
  return v <= min ? min : (v >= max ? max : v);
}

static uint8_t Wmy_abs(signed v) { return v > 0 ? v : -v; }

void WSetState(WState state) {
  previousState = WcurrentState;
  WcurrentState = state;
  WredrawScreen = true;
  WredrawStatus = true;
}

// Radio functions

static void WToggleAFBit(bool on) {
  uint16_t reg = BK4819_ReadRegister(BK4819_REG_47);
  reg &= ~(1 << 8);
  if (on)
    reg |= on << 8;
  BK4819_WriteRegister(BK4819_REG_47, reg);
}

static const BK4819_REGISTER_t Wregisters_to_save[] ={
  BK4819_REG_30,
  BK4819_REG_37,
  BK4819_REG_3D,
  BK4819_REG_43,
  BK4819_REG_47,
  BK4819_REG_48,
  BK4819_REG_7E,
};

static uint16_t Wregisters_stack [sizeof(Wregisters_to_save)];

static void WBackupRegisters() {
  for (uint32_t i = 0; i < ARRAY_SIZE(Wregisters_to_save); i++){
    Wregisters_stack[i] = BK4819_ReadRegister(Wregisters_to_save[i]);
  }
}

static void WRestoreRegisters() {

  for (uint32_t i = 0; i < ARRAY_SIZE(Wregisters_to_save); i++){
    BK4819_WriteRegister(Wregisters_to_save[i], Wregisters_stack[i]);
  }
}

static void WToggleAFDAC(bool on) {
  uint32_t Reg = BK4819_ReadRegister(BK4819_REG_30);
  Reg &= ~(1 << 9);
  if (on)
    Reg |= (1 << 9);
  BK4819_WriteRegister(BK4819_REG_30, Reg);
}

static void WSetF(uint32_t f) {
  WfMeasure = f;

  BK4819_SetFrequency(WfMeasure);
  BK4819_PickRXFilterPathBasedOnFrequency(WfMeasure);
  uint16_t reg = BK4819_ReadRegister(BK4819_REG_30);
  BK4819_WriteRegister(BK4819_REG_30, 0);
  BK4819_WriteRegister(BK4819_REG_30, reg);
}

// Spectrum related

bool WIsPeakOverLevel() { return Wpeak.rssi >= Wsettings.rssiTriggerLevel; }

static void WResetPeak() {
  Wpeak.t = 0;
  Wpeak.rssi = 0;
}

bool WIsCenterMode() { return Wsettings.scanStepIndex < WS_STEP_2_5kHz; }
// scan step in 0.01khz
uint16_t WGetScanStep() { return WscanStepValues[Wsettings.scanStepIndex]; }

uint16_t WGetStepsCount()
{
#ifdef ENABLE_SCAN_RANGES
  if(gScanRangeStart) {
    return (gScanRangeStop - gScanRangeStart) / WGetScanStep();
  }
#endif
  return 128 >> Wsettings.stepsCount;
}

uint32_t WGetBW() { return WGetStepsCount() * WGetScanStep(); }
uint32_t WGetFStart() {
  return WIsCenterMode() ? WcurrentFreq - (WGetBW() >> 1) : WcurrentFreq;
}

uint32_t WGetFEnd() { return WcurrentFreq + WGetBW(); }

static void WTuneToPeak() {
  WscanInfo.f = Wpeak.f;
  WscanInfo.rssi = Wpeak.rssi;
  WscanInfo.i = Wpeak.i;
  WSetF(WscanInfo.f);
}

static void WDeInitSpectrum() {
  WSetF(WinitialFreq);
  WRestoreRegisters();
  WisInitialized = false;
}

uint8_t WGetBWRegValueForScan() {
  return WscanStepBWRegValues[Wsettings.scanStepIndex];
}

uint16_t WGetRssi() {
  // SYSTICK_DelayUs(800);
  // testing autodelay based on Glitch value
  while ((BK4819_ReadRegister(0x63) & 0b11111111) >= 255) {
    SYSTICK_DelayUs(100);
  }
  uint16_t rssi = BK4819_GetRSSI();
#ifdef ENABLE_AM_FIX
  if(settings.modulationType==MODULATION_AM && gSetting_AM_fix)
    rssi += AM_fix_get_gain_diff()*2;
#endif
  return rssi;
}

static void WToggleAudio(bool on) {
  if (on == WaudioState) {
    return;
  }
  WaudioState = on;
  if (on) {
    AUDIO_AudioPathOn();
  } else {
    AUDIO_AudioPathOff();
  }
}

static void WToggleRX(bool on) {
  WisListening = on;

  RADIO_SetupAGC(on, WlockAGC);
  BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, on);

  WToggleAudio(on);
  WToggleAFDAC(on);
  WToggleAFBit(on);

  if (on) {
    WlistenT = 1000;
    BK4819_WriteRegister(0x43, WlistenBWRegValues[Wsettings.listenBw]);
  } else {
    BK4819_WriteRegister(0x43, WGetBWRegValueForScan());
  }
}

// Scan info

static void WResetScanStats() {
  WscanInfo.rssi = 0;
  WscanInfo.rssiMax = 0;
  WscanInfo.iPeak = 0;
  WscanInfo.fPeak = 0;
}

static void WInitScan() {
  WResetScanStats();
  WscanInfo.i = 0;
  WscanInfo.f = WGetFStart();

  WscanInfo.scanStep = WGetScanStep();
  WscanInfo.measurementsCount = WGetStepsCount();
}

static void WResetBlacklist() {
  for (int i = 0; i < 128; ++i) {
    if (WrssiHistory[i] == WRSSI_MAX_VALUE)
      WrssiHistory[i] = 0;
  }
#ifdef ENABLE_SCAN_RANGES
  memset(WblacklistFreqs, 0, sizeof(WblacklistFreqs));
  WblacklistFreqsIdx = 0;
#endif
}

static void WRelaunchScan() {
  WInitScan();
  WResetPeak();
  WToggleRX(false);
#ifdef SPECTRUM_AUTOMATIC_SQUELCH
  settings.rssiTriggerLevel = RSSI_MAX_VALUE;
#endif
  WpreventKeypress = true;
  WscanInfo.rssiMin = WRSSI_MAX_VALUE;
}

static void WUpdateScanInfo() {
  if (WscanInfo.rssi > WscanInfo.rssiMax) {
    WscanInfo.rssiMax = WscanInfo.rssi;
    WscanInfo.fPeak = WscanInfo.f;
    WscanInfo.iPeak = WscanInfo.i;
  }

  if (WscanInfo.rssi < WscanInfo.rssiMin) {
    WscanInfo.rssiMin = WscanInfo.rssi;
    Wsettings.dbMin = WRssi2DBm(WscanInfo.rssiMin);
    WredrawStatus = true;
  }
}

static void WAutoTriggerLevel() {
  if (Wsettings.rssiTriggerLevel == WRSSI_MAX_VALUE) {
    Wsettings.rssiTriggerLevel = Wclamp(WscanInfo.rssiMax + 8, 0, WRSSI_MAX_VALUE);
  }
}

static void WUpdatePeakInfoForce() {
  Wpeak.t = 0;
  Wpeak.rssi = WscanInfo.rssiMax;
  Wpeak.f = WscanInfo.fPeak;
  Wpeak.i = WscanInfo.iPeak;
  WAutoTriggerLevel();
}

static void WUpdatePeakInfo() {
  if (Wpeak.f == 0 || Wpeak.t >= 1024 || Wpeak.rssi < WscanInfo.rssiMax)
    WUpdatePeakInfoForce();
}

static void WSetRssiHistory(uint16_t idx, uint16_t rssi)
{
#ifdef ENABLE_SCAN_RANGES
  if(WscanInfo.measurementsCount > 128) {
    uint8_t i = (uint32_t)ARRAY_SIZE(WrssiHistory) * 1000 / WscanInfo.measurementsCount * idx / 1000;
    if(WrssiHistory[i] < rssi || WisListening)
      WrssiHistory[i] = rssi;
    WrssiHistory[(i+1)%128] = 0;
    return;
  }
#endif
  WrssiHistory[idx] = rssi;
}

static void WMeasure()
{
  uint16_t rssi = WscanInfo.rssi = WGetRssi();
  WSetRssiHistory(WscanInfo.i, rssi);
}

// Update things by keypress

static uint16_t Wdbm2rssi(int dBm) {
  return (dBm + 160 - WdBmCorrTable[gRxVfo->Band]) * 2;
}

static void WClampRssiTriggerLevel() {
  Wsettings.rssiTriggerLevel =
      Wclamp(Wsettings.rssiTriggerLevel, Wdbm2rssi(Wsettings.dbMin),
            Wdbm2rssi(Wsettings.dbMax));
}

static void WUpdateRssiTriggerLevel(bool inc) {
  if (inc)
    Wsettings.rssiTriggerLevel += 2;
  else
    Wsettings.rssiTriggerLevel -= 2;

  WClampRssiTriggerLevel();

  WredrawScreen = true;
  WredrawStatus = true;
}

static void WUpdateDBMax(bool inc) {
  if (inc && Wsettings.dbMax < 10) {
    Wsettings.dbMax += 1;
  } else if (!inc && Wsettings.dbMax > Wsettings.dbMin) {
    Wsettings.dbMax -= 1;
  } else {
    return;
  }

  WClampRssiTriggerLevel();
  WredrawStatus = true;
  WredrawScreen = true;
  SYSTEM_DelayMs(20);
}

static void WUpdateScanStep(bool inc) {
  if (inc) {
    Wsettings.scanStepIndex = Wsettings.scanStepIndex != WS_STEP_100_0kHz ? Wsettings.scanStepIndex + 1 : 0;
  } else {
    Wsettings.scanStepIndex = Wsettings.scanStepIndex != 0 ? Wsettings.scanStepIndex - 1 : WS_STEP_100_0kHz;
  }

  Wsettings.frequencyChangeStep = WGetBW() >> 1;
  WRelaunchScan();
  WResetBlacklist();
  WredrawScreen = true;
}

static void WUpdateCurrentFreq(bool inc) {
  if (inc && WcurrentFreq < F_MAX) {
    WcurrentFreq += Wsettings.frequencyChangeStep;
  } else if (!inc && WcurrentFreq > F_MIN) {
    WcurrentFreq -= Wsettings.frequencyChangeStep;
  } else {
    return;
  }
  WRelaunchScan();
  WResetBlacklist();
  WredrawScreen = true;
}

static void WUpdateCurrentFreqStill(bool inc) {
  uint8_t offset = WmodulationTypeTuneSteps[Wsettings.modulationType];
  uint32_t f = WfMeasure;
  if (inc && f < F_MAX) {
    f += offset;
  } else if (!inc && f > F_MIN) {
    f -= offset;
  }
  WSetF(f);
  WredrawScreen = true;
}

static void WUpdateFreqChangeStep(bool inc) {
  uint16_t diff = WGetScanStep() * 4;
  if (inc && Wsettings.frequencyChangeStep < 200000) {
    Wsettings.frequencyChangeStep += diff;
  } else if (!inc && Wsettings.frequencyChangeStep > 10000) {
    Wsettings.frequencyChangeStep -= diff;
  }
  SYSTEM_DelayMs(100);
  WredrawScreen = true;
}

static void WToggleModulation() {
  if (Wsettings.modulationType < MODULATION_UKNOWN - 1) {
    Wsettings.modulationType++;
  } else {
    Wsettings.modulationType = MODULATION_FM;
  }
  RADIO_SetModulation(Wsettings.modulationType);

  WRelaunchScan();
  WredrawScreen = true;
}

static void WToggleListeningBW() {
  if (Wsettings.listenBw == BK4819_FILTER_BW_NARROWER) {
    Wsettings.listenBw = BK4819_FILTER_BW_WIDE;
  } else {
    Wsettings.listenBw++;
  }
  WredrawScreen = true;
}

static void WToggleBacklight() {
  Wsettings.backlightState = !Wsettings.backlightState;
  if (Wsettings.backlightState) {
    BACKLIGHT_TurnOn();
  } else {
    BACKLIGHT_TurnOff();
  }
}

static void WToggleStepsCount() {
  if (Wsettings.stepsCount == WSTEPS_128) {
    Wsettings.stepsCount = WSTEPS_16;
  } else {
    Wsettings.stepsCount--;
  }
  Wsettings.frequencyChangeStep = WGetBW() >> 1;
  WRelaunchScan();
  WResetBlacklist();
  WredrawScreen = true;
}

static void WResetFreqInput() {
  WtempFreq = 0;
  for (int i = 0; i < 10; ++i) {
    WfreqInputString[i] = '-';
  }
}

static void WFreqInput() {
  WfreqInputIndex = 0;
  WfreqInputDotIndex = 0;
  WResetFreqInput();
  WSetState(WFREQ_INPUT);
}

static void WUpdateFreqInput(KEY_Code_t key) {
  if (key != KEY_EXIT && WfreqInputIndex >= 10) {
    return;
  }
  if (key == KEY_STAR) {
    if (WfreqInputIndex == 0 || WfreqInputDotIndex) {
      return;
    }
    WfreqInputDotIndex = WfreqInputIndex;
  }
  if (key == KEY_EXIT) {
    WfreqInputIndex--;
    if (WfreqInputDotIndex == WfreqInputIndex)
      WfreqInputDotIndex = 0;
  } else {
    WfreqInputArr[WfreqInputIndex++] = key;
  }

  WResetFreqInput();

  uint8_t WdotIndex =
      WfreqInputDotIndex == 0 ? WfreqInputIndex : WfreqInputDotIndex;

  KEY_Code_t digitKey;
  for (int i = 0; i < 10; ++i) {
    if (i < WfreqInputIndex) {
      digitKey = WfreqInputArr[i];
      WfreqInputString[i] = digitKey <= KEY_9 ? '0' + digitKey - KEY_0 : '.';
    } else {
      WfreqInputString[i] = '-';
    }
  }

  uint32_t base = 100000; // 1MHz in BK units
  for (int i = WdotIndex - 1; i >= 0; --i) {
    WtempFreq += (WfreqInputArr[i] - KEY_0) * base;
    base *= 10;
  }

  base = 10000; // 0.1MHz in BK units
  if (WdotIndex < WfreqInputIndex) {
    for (int i = WdotIndex + 1; i < WfreqInputIndex; ++i) {
      WtempFreq += (WfreqInputArr[i] - KEY_0) * base;
      base /= 10;
    }
  }
  WredrawScreen = true;
}

static void WBlacklist() {
#ifdef ENABLE_SCAN_RANGES
  WblacklistFreqs[WblacklistFreqsIdx++ % ARRAY_SIZE(WblacklistFreqs)] = Wpeak.i;
#endif

  WSetRssiHistory(Wpeak.i, WRSSI_MAX_VALUE);
  WResetPeak();
  WToggleRX(false);
  WResetScanStats();
}

#ifdef ENABLE_SCAN_RANGES
static bool WIsBlacklisted(uint16_t idx)
{
  for(uint8_t i = 0; i < ARRAY_SIZE(WblacklistFreqs); i++)
    if(WblacklistFreqs[i] == idx)
      return true;
  return false;
}
#endif

// Draw things

// applied x2 to prevent initial rounding
uint8_t WRssi2PX(uint16_t rssi, uint8_t pxMin, uint8_t pxMax) {
  const int DB_MIN = Wsettings.dbMin << 1;
  const int DB_MAX = Wsettings.dbMax << 1;
  const int DB_RANGE = DB_MAX - DB_MIN;

  const uint8_t PX_RANGE = pxMax - pxMin;

  int dbm = Wclamp(WRssi2DBm(rssi) << 1, DB_MIN, DB_MAX);

  return ((dbm - DB_MIN) * PX_RANGE + DB_RANGE / 2) / DB_RANGE + pxMin;
}

uint8_t WRssi2Y(uint16_t rssi) {
  return WDrawingEndY - WRssi2PX(rssi, 0, WDrawingEndY);
}

static void WDrawSpectrum() {
  for (uint8_t x = 0; x < 128; ++x) {
    uint16_t rssi = WrssiHistory[x >> Wsettings.stepsCount];
    if (rssi != WRSSI_MAX_VALUE) {
      WDrawVLine(WRssi2Y(rssi), WDrawingEndY, x, true);
    }
  }
}

static void WDrawStatus() {
#ifdef SPECTRUM_EXTRA_VALUES
  sprintf(String, "%d/%d P:%d T:%d", Wsettings.dbMin, Wsettings.dbMax,
          WRssi2DBm(Wpeak.rssi), WRssi2DBm(Wsettings.rssiTriggerLevel));
#else
  sprintf(String, "%d/%d", Wsettings.dbMin, Wsettings.dbMax);
#endif
  GUI_DisplaySmallest(String, 0, 1, true, true);

  BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryCheckCounter++ % 4]);

  uint16_t voltage = (gBatteryVoltages[0] + gBatteryVoltages[1] +
                      gBatteryVoltages[2] + gBatteryVoltages[3]) /
                     4 * 760 / gBatteryCalibration[3];

  unsigned perc = BATTERY_VoltsToPercent(voltage);

  // sprintf(String, "%d %d", voltage, perc);
  // GUI_DisplaySmallest(String, 48, 1, true, true);

  gStatusLine[116] = 0b00011100;
  gStatusLine[117] = 0b00111110;
  for (int i = 118; i <= 126; i++) {
    gStatusLine[i] = 0b00100010;
  }

  for (unsigned i = 127; i >= 118; i--) {
    if (127 - i <= (perc + 5) * 9 / 100) {
      gStatusLine[i] = 0b00111110;
    }
  }
}

static void WDrawF(uint32_t f) {
  sprintf(String, "%u.%05u", f / 100000, f % 100000);
  UI_PrintStringSmall(String, 8, 127, 0);

  sprintf(String, "%3s", gModulationStr[Wsettings.modulationType]);
  GUI_DisplaySmallest(String, 116, 1, false, true);
  sprintf(String, "%s", WbwOptions[Wsettings.listenBw]);
  GUI_DisplaySmallest(String, 108, 7, false, true);
}

static void WDrawNums() {

  if (WcurrentState == WSPECTRUM) {
    sprintf(String, "%ux", WGetStepsCount());
    GUI_DisplaySmallest(String, 0, 1, false, true);
    sprintf(String, "%u.%02uk", WGetScanStep() / 100, WGetScanStep() % 100);
    GUI_DisplaySmallest(String, 0, 7, false, true);
  }

  if (WIsCenterMode()) {
    sprintf(String, "%u.%05u \x7F%u.%02uk", WcurrentFreq / 100000,
            WcurrentFreq % 100000, Wsettings.frequencyChangeStep / 100,
            Wsettings.frequencyChangeStep % 100);
    GUI_DisplaySmallest(String, 36, 49, false, true);
  } else {
    sprintf(String, "%u.%05u", WGetFStart() / 100000, WGetFStart() % 100000);
    GUI_DisplaySmallest(String, 0, 49, false, true);

    sprintf(String, "\x7F%u.%02uk", Wsettings.frequencyChangeStep / 100,
            Wsettings.frequencyChangeStep % 100);
    GUI_DisplaySmallest(String, 48, 49, false, true);

    sprintf(String, "%u.%05u", WGetFEnd() / 100000, WGetFEnd() % 100000);
    GUI_DisplaySmallest(String, 93, 49, false, true);
  }
}

static void WDrawRssiTriggerLevel() {
  if (Wsettings.rssiTriggerLevel == WRSSI_MAX_VALUE || WmonitorMode)
    return;
  uint8_t y = WRssi2Y(Wsettings.rssiTriggerLevel);
  for (uint8_t x = 0; x < 128; x += 2) {
    WPutPixel(x, y, true);
  }
}

static void WDrawTicks() {
  uint32_t f = WGetFStart();
  uint32_t span = WGetFEnd() - WGetFStart();
  uint32_t step = span / 128;
  for (uint8_t i = 0; i < 128; i += (1 << Wsettings.stepsCount)) {
    f = WGetFStart() + span * i / 128;
    uint8_t barValue = 0b00000001;
    (f % 10000) < step && (barValue |= 0b00000010);
    (f % 50000) < step && (barValue |= 0b00000100);
    (f % 100000) < step && (barValue |= 0b00011000);

    gFrameBuffer[5][i] |= barValue;
  }

  // center
  if (WIsCenterMode()) {
    memset(gFrameBuffer[5] + 62, 0x80, 5);
    gFrameBuffer[5][64] = 0xff;
  } else {
    memset(gFrameBuffer[5] + 1, 0x80, 3);
    memset(gFrameBuffer[5] + 124, 0x80, 3);

    gFrameBuffer[5][0] = 0xff;
    gFrameBuffer[5][127] = 0xff;
  }
}

static void WDrawArrow(uint8_t x) {
  for (signed i = -2; i <= 2; ++i) {
    signed v = x + i;
    if (!(v & 128)) {
      gFrameBuffer[5][v] |= (0b01111000 << Wmy_abs(i)) & 0b01111000;
    }
  }
}

static void WOnKeyDown(uint8_t key) {
  switch (key) {
  case KEY_3:
    WUpdateDBMax(true);
    break;
  case KEY_9:
    WUpdateDBMax(false);
    break;
  case KEY_1:
    WUpdateScanStep(true);
    break;
  case KEY_7:
    WUpdateScanStep(false);
    break;
  case KEY_2:
    WUpdateFreqChangeStep(true);
    break;
  case KEY_8:
    WUpdateFreqChangeStep(false);
    break;
  case KEY_UP:
#ifdef ENABLE_SCAN_RANGES
    if(!gScanRangeStart)
#endif
      WUpdateCurrentFreq(true);
    break;
  case KEY_DOWN:
#ifdef ENABLE_SCAN_RANGES
    if(!gScanRangeStart)
#endif
      WUpdateCurrentFreq(false);
    break;
  case KEY_SIDE1:
    WBlacklist();
    break;
  case KEY_STAR:
    WUpdateRssiTriggerLevel(true);
    break;
  case KEY_F:
    WUpdateRssiTriggerLevel(false);
    break;
  case KEY_5:
#ifdef ENABLE_SCAN_RANGES
    if(!gScanRangeStart)
#endif
      WFreqInput();
    break;
  case KEY_0:
    WToggleModulation();
    break;
  case KEY_6:
    WToggleListeningBW();
    break;
  case KEY_4:
#ifdef ENABLE_SCAN_RANGES
    if(!gScanRangeStart)
#endif
      WToggleStepsCount();
    break;
  case KEY_SIDE2:
    WToggleBacklight();
    break;
  case KEY_PTT:
    WSetState(WSTILL);
    WTuneToPeak();
    break;
  case KEY_MENU:
    break;
  case KEY_EXIT:
    if (WmenuState) {
      WmenuState = 0;
      break;
    }
    WDeInitSpectrum();
    break;
  default:
    break;
  }
}

static void WOnKeyDownFreqInput(uint8_t key) {
  switch (key) {
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
  case KEY_STAR:
    WUpdateFreqInput(key);
    break;
  case KEY_EXIT:
    if (WfreqInputIndex == 0) {
      WSetState(previousState);
      break;
    }
    WUpdateFreqInput(key);
    break;
  case KEY_MENU:
    if (WtempFreq < F_MIN || WtempFreq > F_MAX) {
      break;
    }
    WSetState(previousState);
    WcurrentFreq = WtempFreq;
    if (WcurrentState == WSPECTRUM) {
      WResetBlacklist();
      WRelaunchScan();
    } else {
      WSetF(WcurrentFreq);
    }
    break;
  default:
    break;
  }
}

void WOnKeyDownStill(KEY_Code_t key) {
  switch (key) {
  case KEY_3:
    WUpdateDBMax(true);
    break;
  case KEY_9:
    WUpdateDBMax(false);
    break;
  case KEY_UP:
    if (WmenuState) {
      WSetRegMenuValue(WmenuState, true);
      break;
    }
    WUpdateCurrentFreqStill(true);
    break;
  case KEY_DOWN:
    if (WmenuState) {
      WSetRegMenuValue(WmenuState, false);
      break;
    }
    WUpdateCurrentFreqStill(false);
    break;
  case KEY_STAR:
    WUpdateRssiTriggerLevel(true);
    break;
  case KEY_F:
    WUpdateRssiTriggerLevel(false);
    break;
  case KEY_5:
    WFreqInput();
    break;
  case KEY_0:
    WToggleModulation();
    break;
  case KEY_6:
    WToggleListeningBW();
    break;
  case KEY_SIDE1:
    WmonitorMode = !WmonitorMode;
    break;
  case KEY_SIDE2:
    WToggleBacklight();
    break;
  case KEY_PTT:
    // TODO: start transmit
    /* BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true); */
    break;
  case KEY_MENU:
    if (WmenuState == ARRAY_SIZE(WregisterSpecs) - 1) {
      WmenuState = 1;
    } else {
      WmenuState++;
    }
    WredrawScreen = true;
    break;
  case KEY_EXIT:
    if (!WmenuState) {
      WSetState(WSPECTRUM);
      WlockAGC = false;
      WmonitorMode = false;
      WRelaunchScan();
      break;
    }
    WmenuState = 0;
    break;
  default:
    break;
  }
}

static void WRenderFreqInput() { UI_PrintString(WfreqInputString, 2, 127, 0, 8); }

static void WRenderStatus() {
  memset(gStatusLine, 0, sizeof(gStatusLine));
  WDrawStatus();
  ST7565_BlitStatusLine();
}

static void WRenderSpectrum() {
  WDrawTicks();
  WDrawArrow(128u * Wpeak.i / WGetStepsCount());
  WDrawSpectrum();
  WDrawRssiTriggerLevel();
  WDrawF(Wpeak.f);
  WDrawNums();
}

static void WRenderStill() {
  WDrawF(WfMeasure);

  const uint8_t METER_PAD_LEFT = 3;

  memset(&gFrameBuffer[2][METER_PAD_LEFT], 0b00010000, 121);

  for (int i = 0; i < 121; i+=5) {
    gFrameBuffer[2][i + METER_PAD_LEFT] = 0b00110000;
  }

  for (int i = 0; i < 121; i+=10) {
    gFrameBuffer[2][i + METER_PAD_LEFT] = 0b01110000;
  }

  uint8_t x = WRssi2PX(WscanInfo.rssi, 0, 121);
  for (int i = 0; i < x; ++i) {
    if (i % 5) {
      gFrameBuffer[2][i + METER_PAD_LEFT] |= 0b00000111;
    }
  }

  int dbm = Rssi2DBm(WscanInfo.rssi);
  uint8_t s = WDBm2S(dbm);
  sprintf(String, "S: %u", s);
  GUI_DisplaySmallest(String, 4, 25, false, true);
  sprintf(String, "%d dBm", dbm);
  GUI_DisplaySmallest(String, 28, 25, false, true);

  if (!WmonitorMode) {
    uint8_t x = WRssi2PX(Wsettings.rssiTriggerLevel, 0, 121);
    gFrameBuffer[2][METER_PAD_LEFT + x] = 0b11111111;
  }

  const uint8_t PAD_LEFT = 4;
  const uint8_t CELL_WIDTH = 30;
  uint8_t offset = PAD_LEFT;
  uint8_t row = 4;

  for (int i = 0, idx = 1; idx <= 4; ++i, ++idx) {
    if (idx == 5) {
      row += 2;
      i = 0;
    }
    offset = PAD_LEFT + i * CELL_WIDTH;
    if (WmenuState == idx) {
      for (int j = 0; j < CELL_WIDTH; ++j) {
        gFrameBuffer[row][j + offset] = 0xFF;
        gFrameBuffer[row + 1][j + offset] = 0xFF;
      }
    }
    sprintf(String, "%s", WregisterSpecs[idx].name);
    GUI_DisplaySmallest(String, offset + 2, row * 8 + 2, false,
                        WmenuState != idx);
    sprintf(String, "%u", WGetRegMenuValue(idx));
    GUI_DisplaySmallest(String, offset + 2, (row + 1) * 8 + 1, false,
                        WmenuState != idx);
  }
}

static void WRender() {
  // Clear Display
  memset(gFrameBuffer, 0, sizeof(gFrameBuffer));

  switch (WcurrentState) {
  case WSPECTRUM:
    WRenderSpectrum();
    break;
  case WFREQ_INPUT:
    WRenderFreqInput();
    break;
  case WSTILL:
    WRenderStill();
    break;
  }

  ST7565_BlitFullScreen();
}

bool WHandleUserInput() {
  Wkbd.prev = Wkbd.current;
  Wkbd.current = WGetKey();

  if (Wkbd.current != KEY_INVALID && Wkbd.current == Wkbd.prev) {
    if (Wkbd.counter < 16)
      Wkbd.counter++;
    else
      Wkbd.counter -= 3;
    SYSTEM_DelayMs(20);
  } else {
    Wkbd.counter = 0;
  }

  if (Wkbd.counter == 3 || Wkbd.counter == 16) {
    switch (WcurrentState) {
    case WSPECTRUM:
      WOnKeyDown(Wkbd.current);
      break;
    case WFREQ_INPUT:
      WOnKeyDownFreqInput(Wkbd.current);
      break;
    case WSTILL:
      WOnKeyDownStill(Wkbd.current);
      break;
    }
  }

  return true;
}

static void WScan() {
  if (WrssiHistory[WscanInfo.i] != WRSSI_MAX_VALUE
#ifdef ENABLE_SCAN_RANGES
  && !WIsBlacklisted(WscanInfo.i)
#endif
  ) {
    WSetF(WscanInfo.f);
    WMeasure();
    WUpdateScanInfo();
  }
}

static void WNextScanStep() {
  ++Wpeak.t;
  ++WscanInfo.i;
  WscanInfo.f += WscanInfo.scanStep;
}

static void WUpdateScan() {
  WScan();

  if (WscanInfo.i < WscanInfo.measurementsCount) {
    WNextScanStep();
    return;
  }

  if(WscanInfo.measurementsCount < 128)
    memset(&WrssiHistory[WscanInfo.measurementsCount], 0,
      sizeof(WrssiHistory) - WscanInfo.measurementsCount*sizeof(WrssiHistory[0]));

  WredrawScreen = true;
  WpreventKeypress = false;

  WUpdatePeakInfo();
  if (WIsPeakOverLevel()) {
    WToggleRX(true);
    WTuneToPeak();
    return;
  }

  WnewScanStart = true;
}

static void WUpdateStill() {
  WMeasure();
  WredrawScreen = true;
  WpreventKeypress = false;

  Wpeak.rssi = WscanInfo.rssi;
  WAutoTriggerLevel();

  WToggleRX(WIsPeakOverLevel() || WmonitorMode);
}

static void WUpdateListening() {
  WpreventKeypress = false;
  if (WcurrentState == WSTILL) {
    WlistenT = 0;
  }
  if (WlistenT) {
    WlistenT--;
    SYSTEM_DelayMs(1);
    return;
  }

  if (WcurrentState == WSPECTRUM) {
    BK4819_WriteRegister(0x43, WGetBWRegValueForScan());
    WMeasure();
    BK4819_WriteRegister(0x43, WlistenBWRegValues[Wsettings.listenBw]);
  } else {
    WMeasure();
  }

  Wpeak.rssi = WscanInfo.rssi;
  WredrawScreen = true;

  if (WIsPeakOverLevel() || WmonitorMode) {
    WlistenT = 1000;
    return;
  }

  WToggleRX(false);
  WResetScanStats();
}

static void WTick() {
#ifdef ENABLE_AM_FIX
  if (gNextTimeslice) {
    gNextTimeslice = false;
    if(Wsettings.modulationType == MODULATION_AM && !WlockAGC) {
      WAM_fix_10ms(vfo); //allow AM_Fix to apply its AGC action
    }
  }
#endif

#ifdef ENABLE_SCAN_RANGES
  if (gNextTimeslice_500ms) {
    gNextTimeslice_500ms = false;

    // if a lot of steps then it takes long time
    // we don't want to wait for whole scan
    // listening has it's own timer
    if(WGetStepsCount()>128 && !WisListening) {
      WUpdatePeakInfo();
      if (WIsPeakOverLevel()) {
        WToggleRX(true);
        WTuneToPeak();
        return;
      }
      WredrawScreen = true;
      WpreventKeypress = false;
    }
  }
#endif

  if (!WpreventKeypress) {
    WHandleUserInput();
  }
  if (WnewScanStart) {
    WInitScan();
    WnewScanStart = false;
  }
  if (WisListening && WcurrentState != WFREQ_INPUT) {
    WUpdateListening();
  } else {
    if (WcurrentState == WSPECTRUM) {
      WUpdateScan();
    } else if (WcurrentState == WSTILL) {
      WUpdateStill();
    }
  }
  if (WredrawStatus || ++WstatuslineUpdateTimer > 4096) {
    WRenderStatus();
    WredrawStatus = false;
    WstatuslineUpdateTimer = 0;
  }
  if (WredrawScreen) {
    WRender();
    WredrawScreen = false;
  }
}

void APP_RunWaterfall() {
  // TX here coz it always? set to active VFO
  Wvfo = gEeprom.TX_VFO;
  // set the current frequency in the middle of the display
#ifdef ENABLE_SCAN_RANGES
  if(gScanRangeStart) {
    WcurrentFreq = WinitialFreq = gScanRangeStart;
    for(uint8_t i = 0; i < ARRAY_SIZE(WscanStepValues); i++) {
      if(WscanStepValues[i] >= gTxVfo->StepFrequency) {
        Wsettings.scanStepIndex = i;
        break;
      }
    }
    Wsettings.stepsCount = WSTEPS_128;
  }
  else
#endif
    WcurrentFreq = WinitialFreq = gTxVfo->pRX->Frequency -
                                ((WGetStepsCount() / 2) * WGetScanStep());

  WBackupRegisters();

  WisListening = true; // to turn off RX later
  WredrawStatus = true;
  WredrawScreen = true;
  WnewScanStart = true;


  WToggleRX(true), WToggleRX(false); // hack to prevent noise when squelch off
  RADIO_SetModulation(Wsettings.modulationType = gTxVfo->Modulation);

  BK4819_SetFilterBandwidth(Wsettings.listenBw = BK4819_FILTER_BW_WIDE, false);

  WRelaunchScan();

  memset(WrssiHistory, 0, sizeof(WrssiHistory));

  WisInitialized = true;

  while (WisInitialized) {
    WTick();
  }
}
