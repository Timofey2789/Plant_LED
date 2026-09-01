// Connect SN74HC595:
// pin 8 - GND, pin 16 - VCC
// pin 10 - VCC
// pin 11 (SRCLK47) to pin 2 arduino
// pin 12 (RCLK47) to pin 3 arduino
// pin 13 (OE47) to pin 4 arduino
// pin 14 (SER47) to pin 5 arduino

// In 4-digits 7-segment display
// Connect f,b,d,dp,c,e,a,g pins through 220 ohm to
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

#define USE_OPTRONE
#ifndef USE_OPTRONE
  #define USE_QUARTZ
  #ifdef USE_QUARTZ
    #define USE_QUARTZ_CALIB
  #endif
#endif

#define USE_EEPROM
#ifdef USE_EEPROM
#include <EEPROM.h>
// memory
const int ADDR_TIME_ON = 0;   // uint16_t (2 byte)
const int ADDR_TIME_OFF = 2;  // uint16_t (2 byte)
const int ADDR_K_COEFF = 4;   // float (4 byte)
const int ADDR_C_COEFF = 8;   // float (4 byte)
#endif

const byte SRCLK47     = 2; // pin 11 SN74HC595 for 4digital segment
const byte RCLK47      = 4; // pin 12 SN74HC595 for 4digital segment
const byte OE47        = 5; // pin 13 SN74HC595 (PWM) 4digital segment
const byte SER47       = 14; // A0 pin 14 SN74HC595 for 4digital segment
const byte DIGITS[]    = {15, 19, 3, 18};
const byte THERMO      = 20; // A6
const byte BUTTON_SEL  = 17; // А3 make INPUT_PULLUP
const byte BUTTON_INCR = 16; // А2 make INPUT_PULLUP
const byte RELAY       = 7;
const byte OPTRONE     = 8;

unsigned long currentMillis = 0;
unsigned long previousMillisTIME = 0; // will store last TIME was updated
const unsigned long intervalTIME = 1000; // milisec
unsigned long actualIntervalTIME = 0; // milisec

#ifdef USE_OPTRONE
  const unsigned long previousInterruptTIME = 2500; // microsec
  volatile byte optroneHalfCycles = 0;
  volatile bool updateTIME = false;

  volatile byte currentTimeHours = 12;
  volatile byte currentTimeMinutes = 0;
  volatile byte currentTimeSeconds = 0;
#else
  byte currentTimeHours = 12;
  byte currentTimeMinutes = 0;
  byte currentTimeSeconds = 0;
#endif

byte relayState = 0;
byte lastRelayState = 0;

byte relayOnTimeHours = 0;
byte relayOnTimeMinutes = 0;

byte relayOffTimeHours = 0;
byte relayOffTimeMinutes = 0;

unsigned long previousMillisDigits = 0; // will store last time 4-Digits was updated
const unsigned long intervalDigits = 1; // milisec
const byte hexArray[] = {B11101110, B01001000, B01100111, B01101011,
						 B11001001, B10101011, B10101111, B01001010,
						 B11101111, B11101011, B11001111, B10101101,
						 B10100110, B01101101, B10100111, B10000111}; // fbd.ceag
const byte dotSign = B00010000;
const byte minusSign = B00000001;
const byte prog1Array[] = {B00000000, B00000000, B11000111, B01001000};
const byte prog2Array[] = {B00000000, B00000000, B11000111, B01100111};
const byte prog3Array[] = {B00000000, B00000000, B11000111, B01101011};
const byte prog4Array[] = {B00000000, B00000000, B11000111, B11001001};

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
  VIEW_TEMP, // average temperature
  PROG_1, // P1 logo
  SET_HOUR_TIME,
  SET_MINUTE_TIME,
  PROG_2,
  SET_HOUR_ON,
  SET_MINUTE_ON,
  PROG_3,
  SET_HOUR_OFF,
  SET_MINUTE_OFF,
  PROG_4,
  SET_K_HIGH,
  SET_K_LOW,
  SET_C_HIGH,
  SET_C_LOW,
};
workState workMode = SET_HOUR_TIME; // VIEW_TIME;
byte displayVisible = 1;
byte settingDigitFirst = 0; // diap to blink
byte settingDigitLast = 1; // diap to blink
unsigned long previousMillisBlink = 0;
const unsigned long intervalBlink4Hz = 125;
const unsigned long intervalBlink2Hz = 250;

// thermoresistor
const float B_COEFFICIENT = 3988.0; // B-коэффициент термистора (обычно 3950)
const float BETA_TEMP = B_COEFFICIENT / 298.15;
const float R_RESISTOR = 10.8;
const float NOMINAL_RESISTANCE = 10.0;
const float NOMINAL_TEMPERATURE = 25.0;
unsigned long previousTempSensor = 0;
const unsigned long intervalTempSensor = 10000; // Проверяем температуру раз в 10 секунд
unsigned long actualIntervalTempSensor = 0; // milisec

float tempC_aver = 25.0; // Стартовое значение (примерная комнатная температура)
const float k_filter = 1.0; // Коэффициент фильтрации (от 0.01 до 1.0) 1.0 - полностью новое значеие
byte print_tempC_aver = 0;

// Формула: Ошибка в секундах за сутки * 11.57.
// 100 -> 200 slow timer. Need make faster
// 200 -> 100 fast timer. Need make slower
#ifdef USE_QUARTZ_CALIB
  float coeff_K = 2.971f;
  float coeff_C = 2921.1f;
  int16_t k_raw_high = 29;    // (coeff_K * 10.0f) % 100
  int16_t k_raw_low = 71;     // (coeff_K * 1000.0f) % 100
  int16_t c_raw_high  = 292;  // coeff_C / 10.0f
  int16_t c_raw_low = 11;     // (coeff_C * 10.0f) % 100

  float ppmCalculated = 0.0f;
  int32_t loadedDelay = 0; // positive value - fast, negative - slow
  byte isMakeSlow = (loadedDelay >= 0) ? 0 : 1;
  // if isMakeSlow = 0 ppmDelay (microsec) the more, the faster
  // if isMakeSlow = 1 ppmDelay (microsec) the more, the slower
  uint32_t ppmDelay = abs(loadedDelay); // microsec
  uint32_t delayMillis = ppmDelay/1000 + 1; // milisec // ceil(ppmDelay)
  uint32_t accTimeCalib = delayMillis*1000; // microsec
  uint32_t calibrationCounter = 0; // microsec
#endif

#ifdef USE_OPTRONE
ISR(PCINT0_vect) {
  static unsigned long previousInterrupt = 0; // will store last time Interrupt was updated
  unsigned long currentMicros = micros();

  if (PINB & 1) { // posedge on pin 8
    if (currentMicros - previousInterrupt >= previousInterruptTIME) { // debounce
      optroneHalfCycles++;
      if (optroneHalfCycles >= 100) { // 100 half cycles = 1 second
        optroneHalfCycles = 0;
        updateTIME = true;
        if (workMode != workState::SET_HOUR_TIME && workMode != workState::SET_MINUTE_TIME) {
          currentTimeSeconds++;
          if (currentTimeSeconds >= 60) {
            currentTimeSeconds = 0;
            currentTimeMinutes++;
            if (currentTimeMinutes >= 60) {
              currentTimeMinutes = 0;
              currentTimeHours++;
              if (currentTimeHours >= 24) {
                currentTimeHours = 0;
              }
            }
          }
        }
      }
    }
  }
}
#endif

void setup() {
  Serial.begin(115200);
  pinMode(OE47, OUTPUT);
  analogWrite(OE47, 220); // brightness of 4digital segment
  pinMode(SER47, OUTPUT);
  pinMode(RCLK47, OUTPUT);
  pinMode(SRCLK47, OUTPUT);
  clearShift();
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
  pinMode(THERMO, INPUT);

#ifdef USE_OPTRONE
  pinMode(OPTRONE, INPUT_PULLUP);
  noInterrupts();
  PCICR |= (1 << PCIE0); // Interrupts on PortB
  PCMSK0 |= (1 << PCINT0); // Interrupt on pin 8
  interrupts();
#endif

#ifdef USE_EEPROM
  loadSettings();
#endif

  readThermo();
}

void loop() { 

#ifdef USE_OPTRONE
  if (updateTIME) { // 1 sec interval
    updateTIME = false;
    // Update time on indicator
    if (workMode == workState::VIEW_TIME) {
      fillInFlag = 1;
    }
    // update temperature
    if (++print_tempC_aver >= 10) {
      readThermo();
      if (workMode == workState::VIEW_TEMP) {
        fillInFlag = 1;
      }
      // debug print
      Serial.print(tempC_aver);
      Serial.print(",");
      Serial.println(currentMillis);
      print_tempC_aver = 0;
    }
  }
#else
// READ TEMPERATURE ???
#ifdef USE_QUARTZ_CALIB
  currentMillis = millis();
  actualIntervalTempSensor = currentMillis - previousTempSensor;
  if (actualIntervalTempSensor >= intervalTempSensor) {
    previousTempSensor = previousTempSensor + (actualIntervalTempSensor/intervalTempSensor)*intervalTempSensor;
    readThermo(); // temporary
    if (workMode == workState::VIEW_TEMP) {
      fillInFlag = 1;
    }
  }
#endif

  currentMillis = millis();
  actualIntervalTIME = currentMillis - previousMillisTIME;
  if (actualIntervalTIME >= intervalTIME) {
    previousMillisTIME = previousMillisTIME + (actualIntervalTIME/intervalTIME)*intervalTIME; // save the last time you changed clock time

#ifdef USE_QUARTZ_CALIB
    calibrationCounter = calibrationCounter + ppmDelay; // add calibration increment (microsec)
#endif
    // if (++print_tempC_aver >= 10) {
    //   Serial.print(tempC_aver);
    //   Serial.print(",");
    //   Serial.println(currentMillis);
    //   print_tempC_aver = 0;
    // }
    // Serial.println(previousMillisTIME);
    if (workMode != workState::SET_HOUR_TIME && workMode != workState::SET_MINUTE_TIME) {
      // КАСКАДНЫЙ СЧЕТЧИК НА БЫСТРЫХ БАЙТАХ:
      currentTimeSeconds++;
      if (currentTimeSeconds >= 60) {
        currentTimeSeconds = 0;
        currentTimeMinutes++;
        if (currentTimeMinutes >= 60) {
          currentTimeMinutes = 0;
          currentTimeHours++;
          if (currentTimeHours >= 24) {
            currentTimeHours = 0;
          }
        }
      }
      if (workMode == workState::VIEW_TIME) {
        fillInFlag = 1;
      }
    }
  }

#ifdef USE_QUARTZ_CALIB
  if (isMakeSlow) {
    currentMillis = millis();
    if (currentMillis - previousMillisTIME >= delayMillis) { // checking that previousMillisTIME does not overtake currentMillis
      if (calibrationCounter >= accTimeCalib) {
        // Serial.println();
        // Serial.println("SLOW ");
        // Serial.println(calibrationCounter);
        calibrationCounter = calibrationCounter - accTimeCalib; // microsec
        previousMillisTIME = previousMillisTIME + delayMillis; // milisec
      }
    }
  } else {
    if (calibrationCounter >= accTimeCalib) {
      // Serial.println();
      // Serial.println("FAST ");
      // Serial.println(calibrationCounter);
      calibrationCounter = calibrationCounter - accTimeCalib; // microsec
      previousMillisTIME = previousMillisTIME - delayMillis; // milisec
    }
  }
#endif
#endif // USE_OPTRONE

  if (workMode == workState::VIEW_TIME) {
    uint16_t currentMinutesOfDecay = (static_cast<uint16_t>(currentTimeHours) * 60) + currentTimeMinutes;
    uint16_t relayOnMinutesOfDecay = (static_cast<uint16_t>(relayOnTimeHours) * 60) + relayOnTimeMinutes;
    uint16_t relayOffMinutesOfDecay = (static_cast<uint16_t>(relayOffTimeHours) * 60) + relayOffTimeMinutes;
    if (relayOnMinutesOfDecay == relayOffMinutesOfDecay) {
      relayState = 0;
    } else if (relayOnMinutesOfDecay < relayOffMinutesOfDecay) {
      // Обычный режим: включено, если внутри интервала
      relayState = (currentMinutesOfDecay >= relayOnMinutesOfDecay && currentMinutesOfDecay < relayOffMinutesOfDecay);
    } else {
      // Режим через полночь: включено, если СЕЙЧАС позже начала ИЛИ раньше конца
      relayState = (currentMinutesOfDecay >= relayOnMinutesOfDecay || currentMinutesOfDecay < relayOffMinutesOfDecay);
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
        if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
          fillInFlag = 1;
          workMode = workState::VIEW_TEMP;
          break;
        }
      }
      if (buttonSel == buttonState::HOLD_2SEC) {
        fillInFlag = 1;
        workMode = workState::PROG_1;
      }
      break;
    case workState::VIEW_TEMP:
      if (buttonIncr == buttonState::RELEASED) {
        fillInFlag = 1;
        workMode = workState::VIEW_TIME;
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
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        workMode = workState::SET_HOUR_TIME;
        currentTimeSeconds = 0;
      }
      break;
    case workState::SET_HOUR_TIME:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        handleTimeSetting(currentTimeHours, true); // true = меняем часы
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(currentTimeHours, true); // true = меняем часы
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
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
        handleTimeSetting(currentTimeMinutes, false); // false = меняем минуты
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(currentTimeMinutes, false); // false = меняем минуты
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
#ifdef USE_OPTRONE
        noInterrupts();
        optroneHalfCycles = 0;
        interrupts();
#else // USE_QUARTZ
        previousMillisTIME = millis();
#endif
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
        handleTimeSetting(relayOnTimeHours, true); // true = меняем часы
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOnTimeHours, true); // true = меняем часы
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
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
        handleTimeSetting(relayOnTimeMinutes, false); // false = меняем минуты
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOnTimeMinutes, false); // false = меняем минуты
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
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
        handleTimeSetting(relayOffTimeHours, true); // true = меняем часы
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOffTimeHours, true); // true = меняем часы
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
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
        handleTimeSetting(relayOffTimeMinutes, false); // false = меняем минуты
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncr) {
            handleTimeSetting(relayOffTimeMinutes, false); // false = меняем минуты
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
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
        workMode = workState::SET_K_HIGH;
      }
      break;
    case workState::SET_K_HIGH:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        k_raw_high++;
        if (k_raw_high > 99) k_raw_high = -99; // Диапазон для наклона
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncrFast) {
            fillInFlag = 1;
            k_raw_high++;
            if (k_raw_high > 99) k_raw_high = -99; // Диапазон для наклона
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
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
        workMode = workState::SET_K_LOW;
      }
      break;
    case workState::SET_K_LOW:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        k_raw_low = (k_raw_low + 1) % 100; // от 00 до 99 (всегда плюс)
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncrFast) {
            fillInFlag = 1;
            k_raw_low = (k_raw_low + 1) % 100; // от 00 до 99 (всегда плюс)
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        coeff_K = static_cast<float>(k_raw_high) / 10.0f + ((k_raw_high >= 0 ? 1 : -1) * static_cast<float>(k_raw_low) / 1000.0f); // Переводим обратно во float
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
#ifdef USE_EEPROM
        saveSettings(3);
#endif
        workMode = workState::SET_C_HIGH;
      }
      break;
    case workState::SET_C_HIGH:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        c_raw_high++;
        if (c_raw_high > 999) c_raw_high = -999; // Полный диапазон со знаком минус!
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncrFast) {
            fillInFlag = 1;
            c_raw_high++;
            if (c_raw_high > 999) c_raw_high = -999; // Полный диапазон со знаком минус!
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
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
        workMode = workState::SET_C_LOW;
      }
      break;
    case workState::SET_C_LOW:
      if (currentMillis - previousMillisBlink >= intervalBlink2Hz) {
        previousMillisBlink = currentMillis;
        displayVisible = !displayVisible;
      }
      if (buttonIncr == buttonState::PUSHED) {
        fillInFlag = 1;
        c_raw_low = (c_raw_low + 1) % 100; // от 00 до 99 (всегда плюс)
        previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
        displayVisible = 1;
        previousMillisPushIncr = currentMillis;
        previousMillisHoldIncr = currentMillis;
        buttonIncr = buttonState::HOLD;
      }
      if (buttonIncr == buttonState::HOLD || buttonIncr == buttonState::HOLD_2SEC) {
        if (currentMillis - previousMillisPushIncr >= intervalAutoIncrFirst) {
          if (currentMillis - previousMillisHoldIncr >= intervalAutoIncrFast) {
            fillInFlag = 1;
            c_raw_low = (c_raw_low + 1) % 100; // от 00 до 99 (всегда плюс)
            previousMillisBlink = currentMillis; // Сбрасываем таймер мигания на текущий момент
            displayVisible = 1;
            previousMillisHoldIncr = currentMillis;
          }
        }
      }
      if (buttonSel == buttonState::PUSHED) {
        fillInFlag = 1;
        coeff_C = static_cast<float>(c_raw_high) * 10.0f + ((c_raw_high >= 0 ? 1 : -1) * static_cast<float>(c_raw_low) / 10.0f);
        settingDigitFirst = 0; // diap to blink
        settingDigitLast = 3; // diap to blink
        displayVisible = 1;
        buttonSel = buttonState::HOLD;
#ifdef USE_EEPROM
        saveSettings(4);
#endif
        updateCalibrationByTemperature(); // Сразу пересчитываем таймеры с новыми коэффициентами
        workMode = workState::VIEW_TIME;
      }
      break;
#endif // USE_QUARTZ_CALIB
  }

  // Update buffer
  if (fillInFlag) {
    switch (workMode) {
      case workState::VIEW_TIME:       setTime(currentTimeHours, currentTimeMinutes, currentTimeSeconds); break;
      case workState::VIEW_TEMP:       setTemp(tempC_aver); break;
      case workState::PROG_1:          setProg(1); break;
      case workState::SET_HOUR_TIME:   setTime(currentTimeHours, currentTimeMinutes, currentTimeSeconds); break;
      case workState::SET_MINUTE_TIME: setTime(currentTimeHours, currentTimeMinutes, currentTimeSeconds); break;
      case workState::PROG_2:          setProg(2); break;
      case workState::SET_HOUR_ON:     setTime(relayOnTimeHours, relayOnTimeMinutes, 0); break;
      case workState::SET_MINUTE_ON:   setTime(relayOnTimeHours, relayOnTimeMinutes, 0); break;
      case workState::PROG_3:          setProg(3); break;
      case workState::SET_HOUR_OFF:    setTime(relayOffTimeHours, relayOffTimeMinutes, 0); break;
      case workState::SET_MINUTE_OFF:  setTime(relayOffTimeHours, relayOffTimeMinutes, 0); break;
#ifdef USE_QUARTZ_CALIB
      case workState::PROG_4:          setProg(4); break;
      case workState::SET_K_HIGH:      setDotValue(k_raw_high, 2);  break;
      case workState::SET_K_LOW:       setDotValue(k_raw_low, 0);  break;
      case workState::SET_C_HIGH:      setValue(c_raw_high);  break;
      case workState::SET_C_LOW:       setDotValue(c_raw_low, 2); break;
#endif
    }
    fillInFlag = 0;
  }

  currentMillis = millis();
  if (currentMillis - previousMillisDigits >= intervalDigits) {
    previousMillisDigits = currentMillis;
    digitalWrite(DIGITS[currentDigit], HIGH); // Turn off previuos digit
    currentDigit = ++currentDigit%4;
    printSymbol(symbolInDigits[currentDigit]);
    if (currentDigit >= settingDigitFirst && currentDigit <= settingDigitLast) {
      if (displayVisible) {
        digitalWrite(DIGITS[currentDigit], LOW); // Turn on next digit
      }
    } else {
      digitalWrite(DIGITS[currentDigit], LOW); // Turn on next digit
    }
  }
  
}

void clearShift() {
  // for 7-segment display
  shiftOut(SER47, SRCLK47, LSBFIRST, 0);
  digitalWrite(RCLK47, HIGH);
  digitalWrite(RCLK47, LOW);
}

void printSymbol(byte num) {
  shiftOut(SER47, SRCLK47, LSBFIRST, num);
  digitalWrite(RCLK47, HIGH);
  digitalWrite(RCLK47, LOW);
}

void handleTimeSetting(volatile byte &time, bool isHours) {
  fillInFlag = 1;
  if (isHours) {
    time = (time + 1) % 24;
  } else { // minutes
    time = (time + 1) % 60;
  }
}

void setTime(byte hour, byte minute, byte second) {
  symbolInDigits[0] = hexArray[hour / 10];
  symbolInDigits[1] = hexArray[hour % 10];
  if (second % 2) symbolInDigits[1] |= dotSign; // мигающая точка
  symbolInDigits[2] = hexArray[minute / 10];
  symbolInDigits[3] = hexArray[minute % 10];

  // if (!second) {
  // char buf[10]; // Буфер для строки "00:00:00" + символ завершения строки
  // sprintf(buf, "%02d:%02d:%02d", hour, minute, second);
  // Serial.println(buf);
  // }
}

void setValue(int16_t num) {
  // Serial.println(num);
  uint16_t absVal = abs(num);
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
    if (num < 0) symbolInDigits[0] = minusSign; // Минус в первом разряде: [-123]
  }
  else if (absVal >= 10) {
    symbolInDigits[2] = hexArray[tens];
    if (num < 0) symbolInDigits[1] = minusSign; // Минус во втором разряде: [ -12]
  }
  else {
    if (num < 0) symbolInDigits[2] = minusSign; // Минус в третьем разряде: [  -1]
  }
}

void setDotValue(int16_t num, byte dotPosition) {
  uint16_t absVal = abs(num);

  // Очищаем буфер (заполняем пустотой)
  symbolInDigits[0] = 0;
  symbolInDigits[1] = 0;
  symbolInDigits[2] = 0;
  symbolInDigits[3] = 0;

  byte digitIdx = 3;
    while (digitIdx >= 0) {
      symbolInDigits[digitIdx] = hexArray[absVal % 10];
      absVal /= 10;
      if (absVal == 0 && digitIdx <= dotPosition) {
        digitIdx--;
        break;
      }
      digitIdx--;
    }
    if (num < 0) {
      if (digitIdx >= 0) {
        symbolInDigits[digitIdx] = minusSign;
      } else {
        symbolInDigits[0] = minusSign;
      }
    }
    if (dotPosition <= 3) {
      symbolInDigits[dotPosition] |= dotSign;
    }
}

void setLargeValue(signed long num) {
  // Serial.println(loadedDelay);
  long thousands = num / 1000;
  setValue(thousands);
  symbolInDigits[3] |= dotSign;
}

void setTemp(float temp) {
  
  int16_t tempInt = static_cast<int16_t>(temp * 10.0f + (temp >= 0 ? 0.5f : -0.5f));
  uint16_t absTemp = abs(tempInt);

  byte tenths = absTemp % 10;   // Десятые доли (последняя цифра)
  byte whole = absTemp / 10;    // Целая часть температуры
  byte ones = whole % 10;       // Единицы
  byte tens = whole / 10;       // Десятки

  if (tempInt < 0) {symbolInDigits[0] = minusSign;}
  else {symbolInDigits[0] = 0;}
  symbolInDigits[1] = hexArray[tens];
  symbolInDigits[2] = hexArray[ones] | dotSign;
  symbolInDigits[3] = hexArray[tenths];
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
      // EEPROM.put(ADDR_TIME_ON, relayOnTime);
      EEPROM.put(ADDR_TIME_ON, static_cast<uint16_t>((relayOnTimeHours << 8) | relayOnTimeMinutes));
      break;
    case 2:
      // EEPROM.put(ADDR_TIME_OFF, relayOffTime);
      EEPROM.put(ADDR_TIME_OFF, static_cast<uint16_t>((relayOffTimeHours << 8) | relayOffTimeMinutes));
      break;
#ifdef USE_QUARTZ_CALIB
    case 3:
      EEPROM.put(ADDR_K_COEFF, coeff_K);
      break;
    case 4:
      EEPROM.put(ADDR_C_COEFF, coeff_C);
      break;
#endif // USE_QUARTZ_CALIB
    default:
      break;
  }
  // Serial.println("Settings saved to EEPROM");
}

void loadSettings() {
  uint16_t value;
  EEPROM.get(ADDR_TIME_ON, value);
  relayOnTimeHours = value >> 8;
  relayOnTimeMinutes = value;
  EEPROM.get(ADDR_TIME_OFF, value);
  relayOffTimeHours = value >> 8;
  relayOffTimeMinutes = value;

  // Проверка на "мусор" (если EEPROM пустая, там могут быть огромные числа)
  if (relayOnTimeHours >= 24) relayOnTimeHours = 0;
  if (relayOffTimeHours >= 24) relayOffTimeHours = 0;
  if (relayOnTimeMinutes >= 60) relayOnTimeMinutes = 0;
  if (relayOffTimeMinutes >= 60) relayOffTimeMinutes = 0;
  // Serial.println("Settings loaded from EEPROM");

#ifdef USE_QUARTZ_CALIB
  float coeff = 0;
  EEPROM.get(ADDR_K_COEFF, coeff);
  if (!isnan(coeff) && coeff < 10.0f && coeff > -10.0f) coeff_K = coeff;
  EEPROM.get(ADDR_C_COEFF, coeff);
  if (!isnan(coeff) && coeff < 10000.0f && coeff > -10000.0f) coeff_C = coeff;

  int32_t total_k_units = static_cast<int32_t>(abs(coeff_K) * 1000.0f + 0.5f);
  k_raw_high = (total_k_units / 100) * (coeff_K >= 0 ? 1 : -1);
  k_raw_low = total_k_units % 100;

  int32_t total_c_units = static_cast<int32_t>(abs(coeff_C) * 10.0f + 0.5f);
  c_raw_high = (total_c_units / 100) * (coeff_C >= 0 ? 1 : -1);
  c_raw_low = total_c_units % 100;

  // Serial.println(coeff_K);
  // Serial.println(coeff_C);
  updateCalibrationByTemperature();
#endif

}
#endif

void readThermo() {
  long a = analogRead(THERMO);
  if (a > 0 && a < 1023) {
    //1
    // float tempC = B_COEFFICIENT / (log((1025.0 * 10 / a - 10) / 10) + B_COEFFICIENT / 298.0) - 273.15;
    //2
    float Rt = R_RESISTOR * ((1023.0 / static_cast<float>(a)) - 1.0);
    float tempC = B_COEFFICIENT / (log(Rt / NOMINAL_RESISTANCE) + BETA_TEMP) - 273.15;
    //3
    // float tempC = B_COEFFICIENT / (log((1023.0 * 10 / a - 10) / 10) + BETA_TEMP) - 273.15;
    //4
    // float Rt = R_RESISTOR * (1023.0 / (float)a - 1.0);
    // Serial.println(a);
    // Serial.println(Rt);
    // float steinhart;
    // steinhart = Rt / NOMINAL_RESISTANCE;     // (R/Ro)
    // steinhart = log(steinhart);                      // ln(R/Ro)
    // steinhart /= B_COEFFICIENT;                      // 1/B * ln(R/Ro)
    // steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
    // steinhart = 1.0 / steinhart;                     // Инвертируем, получаем Кельвины
    // float tempC = steinhart - 273.15;  // Переводим в Цельсии
    // Serial.print("Temp: ");
    // Serial.println(tempC);
    // Serial.println("degree Celsius");


    // average temperature TEMPORARY (make overflow)
    // tempC_aver = tempC;
    // tempC_aver *= measurementNum;
    // tempC_aver += tempC;
    // measurementNum++;
    // if (measurementNum) tempC_aver /= measurementNum;

    // Serial.print(" ");
    // Serial.println(tempC_aver);
    
    tempC_aver = (tempC * k_filter) + (tempC_aver * (1.0 - k_filter));
    // Пересчитываем кривую PPM от температуры
#ifdef USE_QUARTZ_CALIB
    updateCalibrationByTemperature();
#endif

    // Serial.print(tempC_aver);
    // Serial.print(",");
    // Serial.println(millis());
  }
}

#ifdef USE_QUARTZ_CALIB
void updateCalibrationByTemperature() {
  // Линейная аппроксимация:
  ppmCalculated = coeff_C + (coeff_K * tempC_aver);
  // Округляем до целого числа для переменной loadedDelay
  loadedDelay = static_cast<int32_t>(ppmCalculated >= 0 ? ppmCalculated + 0.5f : ppmCalculated - 0.5f);
  // Обновляем настройки таймера
  setPPMDelay();
}

void setPPMDelay() {
  ppmDelay = abs(loadedDelay); // microsec
  isMakeSlow = (loadedDelay >= 0) ? 0 : 1; // positive value - fast, negative - slow
  delayMillis = ppmDelay/1000 + 1; // milisec // ceil(ppmDelay)
  accTimeCalib = delayMillis*1000; // microsec
  // calibrationCounter = 0; // если вызывать setPPM во время работы платы, то счётчик не должен сбрасываться
  // Serial.print("loadedDelay: ");
  // Serial.println(loadedDelay);
  // Serial.print("ppmDelay: ");
  // Serial.println(ppmDelay);
  // Serial.print("isMakeSlow: ");
  // Serial.println(isMakeSlow);
  // Serial.print("delayMillis: ");
  // Serial.println(delayMillis);
  // Serial.print("accTimeCalib: ");
  // Serial.println(accTimeCalib);
  // Serial.print("calibrationCounter: ");
  // Serial.println(calibrationCounter);
}
#endif