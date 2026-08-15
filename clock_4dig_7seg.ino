// Connect SN74HC595:
// pin 8 - GND, pin 16 - VCC
// pin 10 - VCC
// pin 11 (SRCLK47) to pin 2 arduino
// pin 12 (RCLK47) to pin 3 arduino
// pin 13 (OE47) to pin 4 arduino
// pin 14 (SER47) to pin 5 arduino

// In 4-digits 7-segment display
// Connect a,b,c,d,e,f,g,dp pins through 220 ohm to
// 15,1,2,3,4,5,6,7 pins of SN74HC595 respectively.
// Connect d1,d2,d3,d4 pins to 6, 7, 8, 9 arduino pins respectively.
//
//                d1 a  f  d2 d3 b
//                |  |  |  |  |  |
//  -------------------------------------------
//  |                                         |
//  |      -----   -----   -----   -----      |
//  |      |   |   |   |   |   |   |   |      |
//  |      -----   -----   -----   -----      |
//  |      |   |   |   |   |   |   |   |      |
//  |      ----- . ----- . ----- . ----- .    |
//  |                                         |
//  -------------------------------------------
//                |  |  |  |  |  |
//                e  d  dp c  g  d4

#define USE_EEPROM
#ifdef USE_EEPROM
#include <EEPROM.h>

// memory
const int ADDR_TIME_ON = 0; // unsigned long
const int ADDR_TIME_OFF = 4; // unsigned long
const int ADDR_PPM_DELAY = 8; // unsigned long
#endif

const byte SRCLK47			 = 2; // pin 11 SN74HC595 for 4digital segment
const byte RCLK47			 = 3; // pin 12 SN74HC595 for 4digital segment
const byte OE47				 = 4; // pin 13 SN74HC595 (PWM) 4digital segment
const byte SER47			 = 5; // pin 14 SN74HC595 for 4digital segment
const byte DIGITS[]			 = {6, 7, 8, 9};
const byte BUTTON_SEL		 = 10; // make INPUT_PULLUP
const byte BUTTON_INCR		 = 12; // make INPUT_PULLUP
const byte RELAY			 = 11;

unsigned long currentMillis = 0;
unsigned long previousMillisTIME = 0; // will store last TIME was updated
const unsigned long intervalTIME = 1000; // milisec
unsigned long actualIntervalTIME = 0; // milisec

// 100 -> 200 slow timer. Need make faster
// 200 -> 100 fast timer. Need make slower
#define USE_QUARTZ_CALIB
#ifdef USE_QUARTZ_CALIB
  signed long loadedDelay = 0; // positive value - fast, negative - slow
  byte isMakeSlow = 0;
  // if isMakeSlow = 0 ppmDelay (microsec) the more, the faster
  // if isMakeSlow = 1 ppmDelay (microsec) the more, the slower
  unsigned long ppmDelay = 0; // microsec
  unsigned long delayMillis = ppmDelay/1000 + 1; // milisec // ceil(ppmDelay)
  unsigned long accTimeCalib = delayMillis*1000; // microsec
  unsigned long calibrationCounter = 0; // microsec
#endif

//    #define MAKE_SLOW 0
//    // 999000: 200 -> 100 стал быстрее надо замедлить, уменьшить число
//    // 1000: 000 -> 100 стал медленнее надо ускорить, увеличить число
//    // Nano
//    // 10000: 300 -> 400 увеличить
//    // 100000: 080 -> 000
//    // 100000: 140 -> 050
//    // 90000 : 120 -> 100
//    // 80000 : 170 -> 200
//    // 89000 : 980 -> 1000
//    // 90000 : 030 -> 080
//    // 95000 : 1020 -> 860
//    // 100000: 970 -> 880
//    // 95000 : 1000 -> 900
//    // 105000: 970 -> 800
//    // 93000 : 1040 -> 980
//    // 89000 : 980
//  #else
//    // MEGA
//    // 569000: 150 -> 100 увеличить почти
//    // 570000: 320 -> увеличить
//    // 571000: 440 -> 100 увеличить
//  #endif
// #endif

unsigned long currentTime = 0;

byte relayState = 0;
byte lastRelayState = 0;

unsigned long relayOnTime = 0;
unsigned long relayOffTime = 0;

unsigned long previousMillisDigits = 0; // will store last time 4-Digits was updated
const unsigned long intervalDigits = 1; // milisec
const byte hexArray[] = {B11111100, B01100000, B11011010, B11110010,
						 B01100110, B10110110, B10111110, B11100000,
						 B11111110, B11110110, B11101110, B00111110,
						 B10011100, B01111010, B10011110, B10001110}; // abcdefg.
const byte prog1Array[] = {B00000000, B00000000, B11001110, B01100000};
const byte prog2Array[] = {B00000000, B00000000, B11001110, B11011010};
const byte prog3Array[] = {B00000000, B00000000, B11001110, B11110010};
const byte prog4Array[] = {B00000000, B00000000, B11001110, B01100110};
byte currentDigit = 0;
byte symbolInDigits[] = {0, 0, 0, 0}; // Fill in by symbols
byte fillInFlag = 1;

byte currentButtonSel = 0;
byte currentButtonIncr = 0;
unsigned long previousMillisButton = 0; // will store last time Button was updated
const unsigned long intervalButton = 50; // interval for contact bounce (milliseconds)
unsigned long pressButtonSelEvent = 0;
unsigned long pressButtonIncrEvent = 0;
const unsigned long intervalHoldButton = 2000; // milisec for holding button
unsigned long previousMillisPushIncr = 0;
const unsigned long intervalAutoIncrFirst = 500;
unsigned long previousMillisHoldIncr = 0;
const unsigned long intervalAutoIncr = 100; // Скорость намотки (мс)
const unsigned long intervalAutoIncrFast = 10; // Быстрая скорость намотки (мс)

enum buttonState : byte{
  RELEASED,
  PUSHED,
  HOLD,
  HOLD_2SEC,
};
buttonState buttonSel = RELEASED;
buttonState buttonIncr = RELEASED;

enum workState : byte{
  VIEW_TIME, // current time
  PROG_1, // P1 logo
  SET_HOUR_TIME, //
  SET_MINUTE_TIME,//
  PROG_2,
  SET_HOUR_ON,
  SET_MINUTE_ON,
  PROG_3,
  SET_HOUR_OFF,
  SET_MINUTE_OFF,
  PROG_4,
  SET_PPM_DELAY,
  SET_PPT_DELAY,
};
workState workMode = SET_HOUR_TIME;
// workMode 0 -
// workMode 1 - blink display 2 seconds
byte displayVisible = 1;
byte settingDigitFirst = 0; // diap to blink
byte settingDigitLast = 1; // diap to blink
unsigned long previousMillisBlink = 0;
const unsigned long intervalBlink4Hz = 125;
const unsigned long intervalBlink2Hz = 250;

void setup() {
  Serial.begin(9600);
  pinMode(OE47, OUTPUT);
  analogWrite(OE47, 220); // brightness of 4digital segment
  pinMode(SER47, OUTPUT);
  pinMode(RCLK47, OUTPUT);
  pinMode(SRCLK47, OUTPUT);
  clearShift(SRCLK47, SER47, RCLK47);
  pinMode (DIGITS[0], OUTPUT);
  pinMode (DIGITS[1], OUTPUT);
  pinMode (DIGITS[2], OUTPUT);
  pinMode (DIGITS[3], OUTPUT);
  digitalWrite(DIGITS[0], HIGH);
  digitalWrite(DIGITS[1], HIGH);
  digitalWrite(DIGITS[2], HIGH);
  digitalWrite(DIGITS[3], HIGH);
  pinMode(BUTTON_SEL, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  pinMode(BUTTON_INCR, INPUT_PULLUP);
#ifdef USE_EEPROM
  loadSettings();
#endif
}

void loop() {

  currentMillis = millis();
  actualIntervalTIME = currentMillis - previousMillisTIME;
  if (actualIntervalTIME >= intervalTIME) {
    previousMillisTIME = previousMillisTIME + (actualIntervalTIME/intervalTIME)*intervalTIME; // save the last time you changed clock time

#ifdef USE_QUARTZ_CALIB
    calibrationCounter = calibrationCounter + ppmDelay; // add calibration increment
#endif
    // Serial.println(previousMillisTIME);
    // setTime(currentTime);
    if (workMode != workState::SET_HOUR_TIME && workMode != workState::SET_MINUTE_TIME) {
      currentTime = ++currentTime%86400;
      if (workMode == workState::VIEW_TIME) {
        fillInFlag = 1;
      }
    }
  }

#ifdef USE_QUARTZ_CALIB
  if (isMakeSlow) {
    currentMillis = millis();
    if (currentMillis - previousMillisTIME >= delayMillis) {
      if (calibrationCounter >= accTimeCalib) {
          // Serial.print("SLOW");
          // Serial.println(calibrationCounter);
        calibrationCounter = calibrationCounter - accTimeCalib;
        previousMillisTIME = previousMillisTIME + delayMillis;
      }
    }
  } else {
    if (calibrationCounter >= accTimeCalib) {
        // Serial.print("FAST");
        // Serial.println(calibrationCounter);
      calibrationCounter = calibrationCounter - accTimeCalib;
      previousMillisTIME = previousMillisTIME - delayMillis;
    }
  }
#endif

  if (workMode == workState::VIEW_TIME) {
    if (relayOnTime == relayOffTime) {
      relayState = 0;
    } else if (relayOnTime < relayOffTime) {
      // Обычный режим: включено, если внутри интервала
      relayState = (currentTime >= relayOnTime && currentTime < relayOffTime);
    } else {
      // Режим через полночь: включено, если СЕЙЧАС позже начала ИЛИ раньше конца
      relayState = (currentTime >= relayOnTime || currentTime < relayOffTime);
    }
    if (lastRelayState != relayState) {
      lastRelayState = relayState;
      digitalWrite(RELAY, !relayState);
      // Serial.println(relayState ? "RELAY ON" : "RELAY OFF");
    }
  }

  currentMillis = millis();
  if (currentMillis - previousMillisButton >= intervalButton) {
    previousMillisButton = currentMillis; // save the last time you pushed button
    currentButtonSel = !digitalRead(BUTTON_SEL); // 1 - on, 0 - off
    currentButtonIncr = !digitalRead(BUTTON_INCR);
  }

  switch (buttonSel) {
    case buttonState::RELEASED:
      if (currentButtonSel) { // if pressed
        pressButtonSelEvent = currentMillis;
        buttonSel = buttonState::PUSHED;
        // Serial.println("Select Button state: PUSHED");
      }
      break;
    case buttonState::PUSHED:
      buttonSel = buttonState::HOLD;
      // Serial.println("Select Button state: HOLD");
      break;
    case buttonState::HOLD:
      if (!currentButtonSel) {
        buttonSel = buttonState::RELEASED;
        // Serial.println("Select Button state: RELEASED");
      } else if (currentMillis - pressButtonSelEvent >= intervalHoldButton) {
        buttonSel = buttonState::HOLD_2SEC;
        // Serial.println("Select Button state: HOLD_2SEC");
      }
      break;
    case buttonState::HOLD_2SEC:
      if (!currentButtonSel) {
        buttonSel = buttonState::RELEASED;
        // Serial.println("Select Button state: RELEASED");
      }
      break;
  }

  switch (buttonIncr) {
    case buttonState::RELEASED:
      if (currentButtonIncr) { // if pressed
        pressButtonIncrEvent = currentMillis;
        buttonIncr = buttonState::PUSHED;
        // Serial.println("Increment Button state: PUSHED");
      }
      break;
    case buttonState::PUSHED:
      buttonIncr = buttonState::HOLD;
      // Serial.println("Increment Button state: HOLD");
      break;
    case buttonState::HOLD:
      if (!currentButtonIncr) {
        buttonIncr = buttonState::RELEASED;
        // Serial.println("Increment Button state: RELEASED");
      } else if (currentMillis - pressButtonIncrEvent >= intervalHoldButton) {
        buttonIncr = buttonState::HOLD_2SEC;
        // Serial.println("Increment Button state: HOLD_2SEC");
      }
      break;
    case buttonState::HOLD_2SEC:
      if (!currentButtonIncr) {
        buttonIncr = buttonState::RELEASED;
        // Serial.println("Increment Button state: RELEASED");
      }
      break;
  }

  currentMillis = millis();
  switch (workMode) {
    case workState::VIEW_TIME:
      if (buttonSel == buttonState::HOLD) {
        if (currentMillis - previousMillisBlink >= intervalBlink4Hz) {
          previousMillisBlink = currentMillis;
          displayVisible = !displayVisible;
        }
      } else {
        displayVisible = 1;
      }
      if (buttonSel == buttonState::HOLD_2SEC) {
        fillInFlag = 1;
        workMode = workState::PROG_1;
      }
      break;
    case workState::PROG_1:
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        buttonIncr = buttonState::HOLD;
        workMode = workState::PROG_2;
      }
      if (buttonSel == buttonState::RELEASED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 1; // diap to blink
        currentTime = currentTime - (currentTime%60); // set seconds = 0
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        workMode = workState::SET_HOUR_TIME;
      }
      break;
    case workState::SET_HOUR_TIME:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        handleTimeSetting(currentTime, true); // true = меняем часы
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(currentTime, true);
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 2; // diap to blink
        settingDigitLast = 3; // diap to blink
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
        workMode = workState::SET_MINUTE_TIME;
      }
      break;
    case workState::SET_MINUTE_TIME:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        handleTimeSetting(currentTime, false);
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(currentTime, false);
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
        previousMillisTIME = millis();
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
        workMode = workState::VIEW_TIME;
      }
      break;
    case workState::PROG_2:
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        buttonIncr = buttonState::HOLD;
        workMode = workState::PROG_3;
      }
      if (buttonSel == buttonState::RELEASED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 1; // diap to blink
        relayOnTime = relayOnTime - (relayOnTime%60); // set seconds = 0
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        workMode = workState::SET_HOUR_ON;
      }
      break;
    case workState::SET_HOUR_ON:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        handleTimeSetting(relayOnTime, true);
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOnTime, true);
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 2; // diap to blink
        settingDigitLast = 3; // diap to blink
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
        workMode = workState::SET_MINUTE_ON;
      }
      break;
    case workState::SET_MINUTE_ON:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        handleTimeSetting(relayOnTime, false);
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOnTime, false);
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
#ifdef USE_EEPROM
        saveSettings(1);
#endif
        workMode = workState::VIEW_TIME;
      }
      break;
    case workState::PROG_3:
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        buttonIncr = buttonState::HOLD;
        workMode = workState::PROG_1;
      }
#ifdef USE_QUARTZ_CALIB
      if (buttonIncr == buttonState::HOLD_2SEC) {
        fillInFlag = 1;
        workMode = workState::PROG_4;
      }
#endif
      if (buttonSel == buttonState::RELEASED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 1; // diap to blink
        relayOffTime = relayOffTime - (relayOffTime%60); // set seconds = 0
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        workMode = workState::SET_HOUR_OFF;
      }
      break;
    case workState::SET_HOUR_OFF:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        handleTimeSetting(relayOffTime, true);
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOffTime, true);
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 2; // diap to blink
        settingDigitLast = 3; // diap to blink
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
        workMode = workState::SET_MINUTE_OFF;
      }
      break;
    case workState::SET_MINUTE_OFF:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        handleTimeSetting(relayOffTime, false);
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOffTime, false);
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
#ifdef USE_EEPROM
        saveSettings(2);
#endif
        workMode = workState::VIEW_TIME;
      }
      break;
#ifdef USE_QUARTZ_CALIB
    case workState::PROG_4:
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        buttonIncr = buttonState::HOLD;
        workMode = workState::PROG_1;
      }
      if (buttonSel == buttonState::RELEASED && buttonIncr == buttonState::RELEASED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        workMode = workState::SET_PPM_DELAY;
      }
      break;
    case workState::SET_PPM_DELAY:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        signed long currentPPM = loadedDelay % 1000L;
        currentPPM++;
        if (currentPPM > 999) currentPPM = -999;
        signed long head = (abs(loadedDelay) / 1000L) * 1000L;
        if (currentPPM < 0) loadedDelay = -head + currentPPM;
        else loadedDelay = head + currentPPM;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncrFast) {
            previousMillisHoldIncr = currentMillis;
            fillInFlag = 1;
            signed long currentPPM = loadedDelay % 1000L;
            currentPPM++;
            if (currentPPM > 999) currentPPM = -999;
            signed long head = (abs(loadedDelay) / 1000L) * 1000L;
            if (currentPPM < 0) loadedDelay = -head + currentPPM;
            else loadedDelay = head + currentPPM;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
        workMode = workState::SET_PPT_DELAY;
      }
      break;
    case workState::SET_PPT_DELAY:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        if (loadedDelay >= 0) loadedDelay += 1000L;
        else loadedDelay -= 1000L;
        if (abs(loadedDelay) >= 1000000L) {
          loadedDelay = loadedDelay % 1000L; // Сброс тысяч, оставляем только PPM
        }
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncrFast) {
            previousMillisHoldIncr = currentMillis;
            fillInFlag = 1;
            if (loadedDelay >= 0) loadedDelay += 1000L;
            else loadedDelay -= 1000L;
            if (abs(loadedDelay) >= 1000000L) {
            loadedDelay = loadedDelay % 1000L; // Сброс тысяч, оставляем только PPM
          }
        }
      }
    }
    if (buttonSel == buttonState::PUSHED) {
      fillInFlag = 1;
      settingDigitFirst = 0; // diap to blink
      settingDigitLast = 3; // diap to blink
      displayVisible = 1;
      buttonSel = buttonState::HOLD;
#ifdef USE_EEPROM
      saveSettings(3);
#endif
      setPPMDelay();
      workMode = workState::VIEW_TIME;
    }
    break;
#endif // USE_QUARTZ_CALIB
  }

  // Update buffer
    if (fillInFlag) {
    switch (workMode) {
      case workState::VIEW_TIME: setTime(currentTime); break;
      case workState::PROG_1: setProg(1); break;
      case workState::SET_HOUR_TIME: setTime(currentTime); break;
      case workState::SET_MINUTE_TIME: setTime(currentTime); break;
      case workState::PROG_2: setProg(2); break;
      case workState::SET_HOUR_ON: setTime(relayOnTime); break;
      case workState::SET_MINUTE_ON: setTime(relayOnTime); break;
      case workState::PROG_3: setProg(3); break;
      case workState::SET_HOUR_OFF: setTime(relayOffTime); break;
      case workState::SET_MINUTE_OFF: setTime(relayOffTime); break;
#ifdef USE_QUARTZ_CALIB
      case workState::PROG_4: setProg(4); break;
      case workState::SET_PPM_DELAY: setValue(loadedDelay % 1000L); break;
      case workState::SET_PPT_DELAY: setLargeValue(abs(loadedDelay)); break;
#endif
    }
    fillInFlag = 0;
  }

  currentMillis = millis();
  if (currentMillis - previousMillisDigits >= intervalDigits) {
    previousMillisDigits = currentMillis;
    digitalWrite(DIGITS[currentDigit], HIGH); // Turn off previuos digit
    currentDigit = ++currentDigit%4;
    printSymbol(symbolInDigits[currentDigit], SRCLK47, SER47, RCLK47);
    if (currentDigit >= settingDigitFirst && currentDigit <= settingDigitLast) {
      if (displayVisible) {
        digitalWrite(DIGITS[currentDigit], LOW); // Turn on next digit
      }
    } else {
      digitalWrite(DIGITS[currentDigit], LOW); // Turn on next digit
    }
  }

}

void clearShift(const int SRCLK, const int SER, const int RCLK) {
  // for 7-segment display
  digitalWrite(SRCLK, LOW);
  digitalWrite(SER, LOW);
  digitalWrite(RCLK, LOW);
  for (byte i = 0; i < 8; i++) {
    digitalWrite(SRCLK, HIGH);
    digitalWrite(SRCLK, LOW);
  }
  digitalWrite(RCLK, HIGH);
  digitalWrite(RCLK, LOW);
}

void printSymbol(byte num, const byte SRCLK, const byte SER, const byte RCLK) {
  for (byte i = 0; i < 8; i++) {
    digitalWrite(SER, ((1 << i) & num) ? HIGH : LOW);
    digitalWrite(SRCLK, HIGH);
    digitalWrite(SRCLK, LOW);
  }
  digitalWrite(SER, LOW);
  digitalWrite(RCLK, HIGH);
  digitalWrite(RCLK, LOW);
}

void handleTimeSetting(unsigned long &time, bool isHours) {
  fillInFlag = 1;
  unsigned long hours = time / 3600L;
  unsigned long minutes = (time % 3600L) / 60L;
  
  if (isHours) {
    hours = ++hours % 24;
  } else {
    minutes = ++minutes % 60;
  }
  time = (hours * 3600L) + (minutes * 60L);
}

void setTime(unsigned long time) {
  byte hour = time/3600;
  byte minute = (time/60)%60;
  byte second = time%60;

  // char buf[10]; // Буфер для строки "00:00:00" + символ завершения строки
  // sprintf(buf, "%02d:%02d:%02d", hour, minute, second);
  // Serial.println(buf);

  symbolInDigits[0] = hexArray[hour / 10];
  symbolInDigits[1] = hexArray[hour % 10];
  if (second % 2) symbolInDigits[1] |= B00000001; // мигающая точка
  symbolInDigits[2] = hexArray[minute / 10];
  symbolInDigits[3] = hexArray[minute % 10];
}

void setValue(signed long num) {
  // Serial.println(num);
  long absVal = abs(num);
  byte hundreds = (absVal/100)%10;
  byte tens = (absVal/10)%10;
  byte ones = absVal%10;
  // Serial.println(hundreds);
  // Serial.println(tens);
  // Serial.println(ones);

  // Очищаем буфер (заполняем пустотой)
  symbolInDigits[0] = 0;
  symbolInDigits[1] = 0;
  symbolInDigits[2] = 0;
  symbolInDigits[3] = hexArray[ones]; // Последняя цифра всегда есть
  
  // Логика для десятков и сотен (гашение нулей и позиция минуса)
  if (absVal >= 100) {
    symbolInDigits[1] = hexArray[hundreds];
    symbolInDigits[2] = hexArray[tens];
    if (num < 0) symbolInDigits[0] = B00000010; // Минус в первом разряде: [-123]
  }
  else if (absVal >= 10) {
    symbolInDigits[2] = hexArray[tens];
    if (num < 0) symbolInDigits[1] = B00000010; // Минус во втором разряде: [ -12]
  }
  else {
    if (num < 0) symbolInDigits[2] = B00000010; // Минус в третьем разряде: [ -1]
  }
}

void setLargeValue(signed long num) {
  // Serial.println(loadedDelay);
  long thousands = num / 1000;
  setValue(thousands);
  symbolInDigits[3] |= B00000001;
}

void setProg(byte num) {

  switch (num) {
    case 1:
      for (byte i = 0; i < 4; i++) {
        symbolInDigits[i] = prog1Array[i];
      }
      break;
    case 2:
      for (byte i = 0; i < 4; i++) {
        symbolInDigits[i] = prog2Array[i];
      }
      break;
    case 3:
      for (byte i = 0; i < 4; i++) {
        symbolInDigits[i] = prog3Array[i];
      }
      break;
    case 4:
      for (byte i = 0; i < 4; i++) {
        symbolInDigits[i] = prog4Array[i];
      }
      break;
    default:
      break;
  }
}

#ifdef USE_EEPROM
void saveSettings(byte num) {
  switch (num) {
    case 1:
      EEPROM.put(ADDR_TIME_ON, relayOnTime);
      break;
    case 2:
      EEPROM.put(ADDR_TIME_OFF, relayOffTime);
      break;
#ifdef USE_QUARTZ_CALIB
    case 3:
      EEPROM.put(ADDR_PPM_DELAY, loadedDelay);
      break;
#endif // USE_QUARTZ_CALIB
    default:
      break;
  }
  // Serial.println("Settings saved to EEPROM");
}

void loadSettings() {
  EEPROM.get(ADDR_TIME_ON, relayOnTime);
  EEPROM.get(ADDR_TIME_OFF, relayOffTime);

  // Проверка на "мусор" (если EEPROM пустая, там будут огромные числа)
  if (relayOnTime >= 86400) relayOnTime = 0;
  if (relayOffTime >= 86400) relayOffTime = 0;
  // Serial.println("Settings loaded from EEPROM");
  
#ifdef USE_QUARTZ_CALIB
  EEPROM.get(ADDR_PPM_DELAY, loadedDelay);
  // Serial.println(loadedDelay);
  setPPMDelay();
#endif

}
#endif

#ifdef USE_QUARTZ_CALIB
void setPPMDelay() {
  // Serial.println(loadedDelay);
  ppmDelay = abs(loadedDelay); // microsec
  isMakeSlow = (loadedDelay >= 0) ? 0 : 1; // positive value - fast, negative - slow
  delayMillis = ppmDelay/1000 + 1; // milisec // ceil(ppmDelay)
  accTimeCalib = delayMillis*1000; // microsec
  calibrationCounter = 0; // microsec
}
#endif