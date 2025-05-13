#define DBG 0
#include <Arduino.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include "u8g2_font_5x8_custom_mr.h"
#include "Menu.h"
#include "DisplayMenu.h"
#include <ClickButton.h>
#include <AD9959.h>
#include <Wire.h>
#include <ESP32Encoder.h>

// #include <Encoder.h>
// #ifndef GRA_AND_AFCH_ENCODER_MOD3
//   #error The "Encoder" library modified by GRA and AFCH must be used!
// #endif

#define FIRMWAREVERSION 2.04

//2.04 13.12.2024 закончено внедрения третьего параметра для Clock Src - ext TCXO/OCXO
//    Строковые константы завернуты в макрос F
//2.03 27.11.2024 начало внедрения третьего параметра для Clock Src - ext TCXO/OCXO
//2.02 23.09.2024 Переход на дисплеи с контроллером SSD1306 и SSD1309
//2.01 19.09.2024 Переход на плату версии 2.х
//    управление коммутатором тактирования
//    управелние светодиодом индикации источника тактирования
//    исправлена ошибка с поляронстью фазы и поэтому теперь не используется функция CorrectPhase
//    изменены значения по-умолчанию: TCXO - 50 МГц, N - 10, Amplitude - -3dBm
//    Увеличина мощность выходного сигнала с -7 до -3 dBm (чтобы увеличить мощность необходимо заменить резистор R11 c 1.91 kOhm на 1.2 kOhm, это обязательно нужно сделать иначе мощность на экране не будет совпадать с фактической)

//1.23 Добавлены комманды включения и выключения выходов (всех одновременно)
//1.22 ускорена обработка команд через последовтельный порт
//1.21 12.06.2023 добавлена поддержка управления через последовтельный порт
//1.2 15.07.2021 исправлен баг с уходом фазы при перестройке частоты
//1.1 06.11.2020 исправлена фаза на выходах F2 и F3
//0.17 Контроль граничных значений частоты и вывод сообщений об ошибках на экран
//0.16 изменение настроек в меню тактирования применятся только после принудительного сохранения
// библиотека AD9959 изменена таким образом чтобы можнобыло менять частоту через функцию SetClock reference_freq
//0.15 кнопка back выходит из меню настроек тактирования
//0.14 19.10.2020 исправление ошибок
//0.13 Добавлено сохранение настроек тактирования в EEPROM
//0.12 Добавляем сохранение основных настроек в EEPROM
//0.11 включаем в DisplayMenu отображение реальных значений, устраняем ошибку когда кнопка back не отключает режим редактирования

//AAAAAAA TCXO 50MHZ сделать по умолчанию!!!!
// RF Switcher
// REF_LED ON/OFF TCXO - pin 8
// V1 - pin 49
// V2 - pin 48

//#include <AsyncStream.h>
//AsyncStream<110> serialbuffer(&Serial, '\n');
#include <GParser.h>

#include  "AD9959.h"
#ifndef GRA_AND_AFCH_AD9959_MOD
  #error The "AD9959" library modified by GRA and AFCH must be used!
#endif

#define POWER_DOWN_CONTROL_PIN 3
#define SDIO_3_PIN 20
#define SDIO_1_PIN 8

#define P0_PIN 4
#define P1_PIN 5
#define P2_PIN 6
#define P3_PIN 7

#define CLOCK_SOURCE_TCXO_INDEX 0
#define CLOCK_SOURCE_EXT_TCXO_INDEX 1
#define CLOCK_SOURCE_EXT_OSC_INDEX 2
#define TCXO_POWER_PIN 8 //REF_LED
#define TCXO_PATH_PIN 48 //V2
#define EXTERANL_SRC_PATH_PIN 49 //V1

#define OLED_SDA 35
#define OLED_SCL 36

bool isPWR_DWN = false;

class MyAD9959 : public AD9959<
    46,              // Reset pin (active = high)
    16,              // Chip Enable (active = low)
    15,              // I/O_UPDATE: Apply config changes (pulse high)
    25000000        // 40MHz crystal (optional)
> 
{
  public:
    void AllChanAutoClearPhase()
    {
     //setChannels(MyAD9959::Channel0);
     write(MyAD9959::FR2, FR2_Bits::AllChanAutoClearPhase); 
    }
};

MyAD9959  dds;

//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

uint32_t EXT_OSC_Freq=BASE_DDS_CORE_CLOCK;
uint32_t DDS_Core_Clock=BASE_DDS_CORE_CLOCK;
#define LOW_FREQ_LIMIT  100000 //Hz (100 kHz)
#define HIGH_FREQ_LIMIT  225000000 //Hz (225 MHZ)
uint32_t ui32HIGH_FREQ_LIMIT=0;
uint32_t F0OutputFreq=0, F1OutputFreq=0, F2OutputFreq=0, F3OutputFreq=0;

#define MODE_PIN 39   
#define BACK_PIN 41

ClickButton modeButton (MODE_PIN, LOW, CLICKBTN_PULLUP);
ClickButton backButton (BACK_PIN, LOW, CLICKBTN_PULLUP);

// Encoder myEnc(37, 38); // Voron EC12PLGSDVF D25K24 (blue)
// Encoder myEnc(19, 18); // Bourns PEC11R-4220K-S0024 (green)

/*adapter code for rotory encoder*/
// 创建ESP32Encoder对象
ESP32Encoder encoder;
int lastEncoderCount = 0;

// 在setup()函数中初始化编码器
void setupEncoder() {
  // 直接初始化编码器，不设置内部上拉电阻
  encoder.attachFullQuad(37, 38);
  
  // 清零编码器计数
  encoder.clearCount();
  lastEncoderCount = 0;
}

// 在loop()函数中获取编码器值的函数
int getEncoderPosition() {
  int newCount = encoder.getCount();
  int diff = newCount - lastEncoderCount;
  lastEncoderCount = newCount;
  
  // 将差值除以2，因为每个格子产生2个计数
  return diff / 2;
}
bool MenuEditMode=false;

void ModeButtonDown()
{
  volatile static uint32_t lastTimeButtonPressed=millis();
  volatile static uint8_t lastState=1;
  volatile static uint8_t currState=1;
  if (millis()-lastTimeButtonPressed < modeButton.debounceTime) return;
  currState=digitalRead(MODE_PIN);
  if ((lastState == HIGH) && (currState == LOW)) modeButton.Update();
  lastState=currState;
}

void DownButtonDown()
{
  volatile static uint32_t lastTimeButtonPressed=millis();
  volatile static uint8_t lastState=1;
  volatile static uint8_t currState=1;
  if (millis()-lastTimeButtonPressed < backButton.debounceTime) return;
  currState=digitalRead(BACK_PIN);
  if ((lastState == HIGH) && (currState == LOW)) backButton.Update();
  lastState=currState;
}

uint32_t DegToPOW(uint16_t deg) //функция принимает значение в градусах умноженное на 10
{
  #define TWO_POW_14 16384
  uint32_t POW=(deg/3600.0)*TWO_POW_14; //360.0 changed to 3600.0
  return POW;
}

uint16_t dBmToASF(uint8_t dBm)
{
  //return (uint16_t)powf(10,(-1*dBm+7+60.206)/20.0); //10(maxValue)+log(1024(2^10))*20=60,205999132796239042747778944899
  return (uint16_t)powf(10,(-1*dBm+3+60.206)/20.0);
}

uint16_t CorrectPhase(uint16_t phase) // компенсация ошибки разводки на плате весрии 1.1
{
  if (phase < 1800) phase = phase + 1800;
    else phase = phase - 1800;
    return phase;
}

int8_t FreqInRange() //1,2,3,4 - Higher, -1,-2,-3,-4 - lower, 0 - in range
{
  //ui32HIGH_FREQ_LIMIT=DDS_Core_Clock*0.45;
  F0OutputFreq=F0_MHz.value * 1000000UL + F0_kHz.value * 1000UL + F0_Hz.value;
  F1OutputFreq=F1_MHz.value * 1000000UL + F1_kHz.value * 1000UL + F1_Hz.value;
  F2OutputFreq=F2_MHz.value * 1000000UL + F2_kHz.value * 1000UL + F2_Hz.value;
  F3OutputFreq=F3_MHz.value * 1000000UL + F3_kHz.value * 1000UL + F3_Hz.value;

  #if DBG==1
  Serial.print("ui32HIGH_FREQ_LIMIT=");
  Serial.println(ui32HIGH_FREQ_LIMIT);
  #endif
  
  if ((F0OutputFreq > ui32HIGH_FREQ_LIMIT) || (F0OutputFreq > HIGH_FREQ_LIMIT)) return 1;
  if (F0OutputFreq < LOW_FREQ_LIMIT) return -1;

  if ((F1OutputFreq > ui32HIGH_FREQ_LIMIT) || (F1OutputFreq > HIGH_FREQ_LIMIT)) return 2;
  if (F1OutputFreq < LOW_FREQ_LIMIT) return -2;

  if ((F2OutputFreq > ui32HIGH_FREQ_LIMIT) || (F2OutputFreq > HIGH_FREQ_LIMIT)) return 3;
  if (F2OutputFreq < LOW_FREQ_LIMIT) return -3;

  if ((F3OutputFreq > ui32HIGH_FREQ_LIMIT) || (F3OutputFreq > HIGH_FREQ_LIMIT)) return 4;
  if (F3OutputFreq < LOW_FREQ_LIMIT) return -4;
  return 0;
}

void ApplyChangesToDDS()
{
  //dds.reference_freq=ClockFreq.Ref_Clk[ClockFreq.value];
  //dds.setClock(DDSCoreClock.value,0); //<-- set PLL multiplier

  if (FreqInRange() !=0) return;

  //dds.write(MyAD9959::FR1, FR1_Bits::SyncClkDisable); // Don't output SYNC_CLK
  //dds.AllChanAutoClearPhase();

  dds.setFrequency(MyAD9959::Channel0, F0OutputFreq); 
  dds.setAmplitude(MyAD9959::Channel0, dBmToASF(F0_Amplitude.value));   
  dds.setPhase(MyAD9959::Channel0, DegToPOW(F0_Phase.value * 10 + F0_PhaseFraction.value));    

  dds.setFrequency(MyAD9959::Channel1, F1OutputFreq);  
  dds.setAmplitude(MyAD9959::Channel1, dBmToASF(F1_Amplitude.value));    
  dds.setPhase(MyAD9959::Channel1, DegToPOW(F1_Phase.value * 10 + F1_PhaseFraction.value));    

  dds.setFrequency(MyAD9959::Channel2, F2OutputFreq);  
  dds.setAmplitude(MyAD9959::Channel2, dBmToASF(F2_Amplitude.value));   
  //dds.setPhase(MyAD9959::Channel2, DegToPOW(CorrectPhase(F2_Phase.value * 10 + F2_PhaseFraction.value))); // компенсация ошибки разводки на плате весрии 1.1
  dds.setPhase(MyAD9959::Channel2, DegToPOW(F2_Phase.value * 10 + F2_PhaseFraction.value));

  dds.setFrequency(MyAD9959::Channel3, F3OutputFreq);  
  dds.setAmplitude(MyAD9959::Channel3, dBmToASF(F3_Amplitude.value));    
  //dds.setPhase(MyAD9959::Channel3, DegToPOW(CorrectPhase(F3_Phase.value * 10 + F3_PhaseFraction.value))); // компенсация ошибки разводки на плате весрии 1.1  
  dds.setPhase(MyAD9959::Channel3, DegToPOW(F3_Phase.value * 10 + F3_PhaseFraction.value)); 
  
  dds.update();
}




void selectClockSrcPath(uint8_t path)
{
  switch (path)
  {
    case CLOCK_SOURCE_TCXO_INDEX:
      digitalWrite(TCXO_POWER_PIN, HIGH);
      digitalWrite(TCXO_PATH_PIN, HIGH);
      digitalWrite(EXTERANL_SRC_PATH_PIN, LOW);
    break;
    case CLOCK_SOURCE_EXT_TCXO_INDEX:  //////////// ЭТО ЭКСТЕРНАЛ TCXO!!!!!!!!!!  //
      digitalWrite(TCXO_POWER_PIN, LOW);
      digitalWrite(TCXO_PATH_PIN, LOW);
      digitalWrite(EXTERANL_SRC_PATH_PIN, HIGH);
    break;
    case CLOCK_SOURCE_EXT_OSC_INDEX:
      digitalWrite(TCXO_POWER_PIN, LOW);
      digitalWrite(TCXO_PATH_PIN, LOW);
      digitalWrite(EXTERANL_SRC_PATH_PIN, HIGH);
    break;    
  }
}

/*the following code come from read serial commands file*/
int C=-1; //Номер канала(выхода) для управления, по умолчанию не задан (-1), допустимые значения: 0 - 3

#define SERIAL_PACKAGE_MAX_LENGTH 110
char Buff[SERIAL_PACKAGE_MAX_LENGTH];

const char HELP_STRING [] PROGMEM = "C — Set the current output Channel: (0 — 3)\n"
          "F — Sets Frequency in Hz (100000 — 225000000)\n"
          "A — Sets the power (Amplitude) level of the selected channel in dBm (-60 — -7)\n"
          "P — Sets the Phase of the selected channel in dBm (0 — 360)\n"
          "M — Gets Model\n"
          "E - Enable Outputs (ALL)\n"
          "D - Disable Outputs (ALL)\n"  
          "V — Gets Firmware Version\n"
          "h — This Help\n"
          "; — Commands Separator"
          "\n"
          "Example:\n"
          "C0;F100000;A-10\n"
          "Sets the Frequency to 100 kHz, and Output Power (Amplitude) to -10 dBm on Channel 0 (RF OUT0).\n"
          "Any number of commands in any order is allowed, but the very first command must be \"C\".\n"
          "Note: by default, the maximum length of one message is 64 bytes";


bool inRange(int32_t val, int32_t minimum, int32_t maximum)
{
  return ((minimum <= val) && (val <= maximum));
}

void ReadSerialCommands()
{
  if (!Serial.available()) return;
  int RcvCounter=0;
  RcvCounter = Serial.readBytesUntil('\n', Buff, 110);
  if (RcvCounter == 0) return;
  Buff[RcvCounter]='\0';
  
  int32_t value=0;
  char command;

    GParser data(Buff, ';');
    int commandsCounter = data.split();

    for (int i=0; i < commandsCounter; i++)
    {
      sscanf(data[i], "%c%ld", &command, &value);
      switch (command)
      {

        case 'C': //Current Channel (0 - 3)
          if (inRange(value, 0, 3))
          {
            Serial.print(F("The Channel number is set to: "));
            Serial.println(value);
            C = value;
          } else Serial.println(F("The Channel number is OUT OF RANGE (0 — 3)"));
        break;

        case 'F': //RF Frequency
          if (C==-1) {Serial.println(F("The output Channel is not selected! Use \"C\" command to select the Channel.")); return;}
          if (inRange(value, LOW_FREQ_LIMIT, ui32HIGH_FREQ_LIMIT))
          {
            Serial.print(F("The Frequency of Channel "));
            Serial.print(C);
            Serial.print(F(" is set to: "));
            Serial.println(value);
            uint16_t H, K, M;
            H = value % 1000;
            K = (value / 1000) % 1000;
            M = value / 1000000;
            switch (C)
            {
              case 0:
                F0_Hz.value = H;
                F0_kHz.value = K;
                F0_MHz.value = M;
                F0OutputFreq = value;
              break;
              case 1:
                F1_Hz.value = H;
                F1_kHz.value = K;
                F1_MHz.value = M;
                F0OutputFreq = value;
              break;
              case 2:
                F2_Hz.value = H;
                F2_kHz.value = K;
                F2_MHz.value = M;
                F0OutputFreq = value;
              break;
              case 3:
                F3_Hz.value = H;
                F3_kHz.value = K;
                F3_MHz.value = M;
                F0OutputFreq = value;
              break;
            }
          } else 
          {
            Serial.print(F("Frequency is OUT OF RANGE ("));
            Serial.println(String(LOW_FREQ_LIMIT) + " - " + String(ui32HIGH_FREQ_LIMIT) + ")");
          }
        break;

        case 'A': //Power(Amplitude), dBm -60 - -7
          if (C==-1) {Serial.println(F("The output Channel is not selected! Use \"C\" command to select the Channel.")); return;}
          if (inRange(value, -60, -7))
          {
            Serial.print(F("The Power (Amplitude) of Channel "));
            Serial.print(C);
            Serial.print(F(" is set to: "));
            Serial.println(value);
            switch (C)
            {
              case 0:
                F0_Amplitude.value = -1 * value;
              break;
              case 1:
                F1_Amplitude.value = -1 * value;
              break;
              case 2:
                F2_Amplitude.value = -1 * value;
              break;
              case 3:
                F3_Amplitude.value = -1 * value;
              break;
            }
          } else Serial.println(F("Power is OUT OF RANGE (-60 — -7)"));
        break;

        case 'P': //Phase, 0.0 - 360.0
          if (C==-1) {Serial.println(F("The output Channel is not selected! Use \"C\" command to select the Channel.")); return;}
          if (inRange(value, 0, 360))
          {
            Serial.print(F("The Phase of Channel "));
            Serial.print(C);
            Serial.print(F(" is set to: "));
            Serial.println(value);
            switch (C)
            {
              case 0:
                F0_Phase.value = value;
              break;
              case 1:
                F1_Phase.value = value;
              break;
              case 2:
                F2_Phase.value = value;
              break;
              case 3:
                F3_Phase.value = value;
              break;
            }
          } else Serial.println(F("Phase is OUT OF RANGE (0 — 360)"));
        break;

        case 'D': //Firmware Version request
          Serial.println(F("Outputs Disabled"));
          digitalWrite(POWER_DOWN_CONTROL_PIN, HIGH);
          isPWR_DWN = true;
        break;

        case 'E': //Firmware Version request
          Serial.println(F("Outputs Enabled"));
          digitalWrite(POWER_DOWN_CONTROL_PIN, LOW);
          isPWR_DWN = false;
        break;

        case 'V': //Firmware Version request
          Serial.println(FIRMWAREVERSION);
          //Serial.println(value);
        break;

        case 'M': //Model request
          Serial.println(F("DDS9959 v1.1"));
          //Serial.println(value);
        break;

        case 'h': //Model request
          Serial.println((const __FlashStringHelper *) HELP_STRING);
        break;

        default:
        Serial.print(F("Unknown command:"));
        Serial.println(command);
        Serial.println((const __FlashStringHelper *) HELP_STRING);
      } //switch
    } //for

    DisplayMenu(menuType);
    ApplyChangesToDDS();
}

/*display menu file*/
void DisplayMenu(uint8_t menuType) {
  //do   {
  switch (menuType) {
    case MAIN_MENU:
      u8g2.setCursor(58, 9);
      if (isPWR_DWN) {
        u8g2.print(F("OFF"));
      } else {
        u8g2.print(F("   "));
      }

      // *********** First ROW Start ***********
      u8g2.setCursor(2, 20);
      if (curItem == &F0) u8g2.setDrawColor(0);
      else u8g2.setDrawColor(1);
      u8g2.print(F("F0"));
      u8g2.setDrawColor(1);
      u8g2.setCursor(17, 20);
      if (curItem == &F0_MHz) {
        if (F0_MHz.bBlink == true) u8g2.setDrawColor(F0_MHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      //u8g2.print("299");
      u8g2.print(PreZero(F0_MHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F0_kHz) {
        if (F0_kHz.bBlink == true) u8g2.setDrawColor(F0_kHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F0_kHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F0_Hz) {
        if (F0_Hz.bBlink == true) u8g2.setDrawColor(F0_Hz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F0_Hz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.setCursor(77, 20);
      if (curItem == &F0_Amplitude) {
        if (F0_Amplitude.bBlink == true) u8g2.setDrawColor(F0_Amplitude.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F("-"));
      u8g2.print(PreZero(F0_Amplitude.value, 2));
      u8g2.setDrawColor(1);
      u8g2.setCursor(97, 20);
      if (curItem == &F0_Phase) {
        if (F0_Phase.bBlink == true) u8g2.setDrawColor(F0_Phase.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F0_Phase.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F0_PhaseFraction) {
        if (F0_PhaseFraction.bBlink == true) u8g2.setDrawColor(F0_PhaseFraction.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F0_PhaseFraction.value);
      u8g2.setDrawColor(1);
      u8g2.print(F("{"));
      // *********** First ROW End ***********

      // *********** 2nd ROW Start ***********
      u8g2.setCursor(2, 30);
      if (curItem == &F1) u8g2.setDrawColor(0);
      else u8g2.setDrawColor(1);
      u8g2.print(F("F1"));
      u8g2.setDrawColor(1);
      u8g2.setCursor(17, 30);
      if (curItem == &F1_MHz) {
        if (F1_MHz.bBlink == true) u8g2.setDrawColor(F1_MHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F1_MHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F1_kHz) {
        if (F1_kHz.bBlink == true) u8g2.setDrawColor(F1_kHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F1_kHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F1_Hz) {
        if (F1_Hz.bBlink == true) u8g2.setDrawColor(F1_Hz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F1_Hz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.setCursor(77, 30);
      if (curItem == &F1_Amplitude) {
        if (F1_Amplitude.bBlink == true) u8g2.setDrawColor(F1_Amplitude.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F("-"));
      u8g2.print(PreZero(F1_Amplitude.value, 2));
      u8g2.setDrawColor(1);
      u8g2.setCursor(97, 30);
      if (curItem == &F1_Phase) {
        if (F1_Phase.bBlink == true) u8g2.setDrawColor(F1_Phase.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F1_Phase.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F1_PhaseFraction) {
        if (F1_PhaseFraction.bBlink == true) u8g2.setDrawColor(F1_PhaseFraction.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F1_PhaseFraction.value);
      u8g2.setDrawColor(1);
      u8g2.print(F("{"));
      // *********** 2nd ROW End ***********

      // *********** 3nd ROW Start ***********
      u8g2.setCursor(2, 40);
      if (curItem == &F2) u8g2.setDrawColor(0);
      else u8g2.setDrawColor(1);
      u8g2.print(F("F2"));
      u8g2.setDrawColor(1);
      u8g2.setCursor(17, 40);
      if (curItem == &F2_MHz) {
        if (F2_MHz.bBlink == true) u8g2.setDrawColor(F2_MHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F2_MHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F2_kHz) {
        if (F2_kHz.bBlink == true) u8g2.setDrawColor(F2_kHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F2_kHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F2_Hz) {
        if (F2_Hz.bBlink == true) u8g2.setDrawColor(F2_Hz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F2_Hz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.setCursor(77, 40);
      if (curItem == &F2_Amplitude) {
        if (F2_Amplitude.bBlink == true) u8g2.setDrawColor(F2_Amplitude.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F("-"));
      u8g2.print(PreZero(F2_Amplitude.value, 2));
      u8g2.setDrawColor(1);
      u8g2.setCursor(97, 40);
      if (curItem == &F2_Phase) {
        if (F2_Phase.bBlink == true) u8g2.setDrawColor(F2_Phase.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F2_Phase.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F2_PhaseFraction) {
        if (F2_PhaseFraction.bBlink == true) u8g2.setDrawColor(F2_PhaseFraction.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F2_PhaseFraction.value);
      u8g2.setDrawColor(1);
      u8g2.print(F("{"));
      // *********** 3nd ROW End ***********

      // *********** 4th ROW Start ***********
      u8g2.setCursor(2, 50);
      if (curItem == &F3) u8g2.setDrawColor(0);
      else u8g2.setDrawColor(1);
      u8g2.print(F("F3"));
      u8g2.setDrawColor(1);
      u8g2.setCursor(17, 50);
      if (curItem == &F3_MHz) {
        if (F3_MHz.bBlink == true) u8g2.setDrawColor(F3_MHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F3_MHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F3_kHz) {
        if (F3_kHz.bBlink == true) u8g2.setDrawColor(F3_kHz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F3_kHz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F3_Hz) {
        if (F3_Hz.bBlink == true) u8g2.setDrawColor(F3_Hz.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F3_Hz.value, 3));
      u8g2.setDrawColor(1);
      u8g2.setCursor(77, 50);
      if (curItem == &F3_Amplitude) {
        if (F3_Amplitude.bBlink == true) u8g2.setDrawColor(F3_Amplitude.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F("-"));
      u8g2.print(PreZero(F3_Amplitude.value, 2));
      u8g2.setDrawColor(1);
      u8g2.setCursor(97, 50);
      if (curItem == &F3_Phase) {
        if (F3_Phase.bBlink == true) u8g2.setDrawColor(F3_Phase.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(PreZero(F3_Phase.value, 3));
      u8g2.setDrawColor(1);
      u8g2.print(F("."));
      if (curItem == &F3_PhaseFraction) {
        if (F3_PhaseFraction.bBlink == true) u8g2.setDrawColor(F3_PhaseFraction.GetColor());
        else u8g2.setDrawColor(0);
      } else u8g2.setDrawColor(1);
      u8g2.print(F3_PhaseFraction.value);
      u8g2.setDrawColor(1);
      u8g2.print(F("{"));
      // *********** 4th ROW End ***********

      /*int y=0;
  u8g2.setDrawColor(0);
  for (int i=0; i<22; i++)
  {
    u8g2.drawHLine(0, y, 128);
    y=y+2;
  }*/
      // ***** ERROR MESSAGES **********
      u8g2.setCursor(0, 61);
      switch (FreqInRange()) {
        case 0:
          u8g2.print(F("                "));
          break;
        case 1:
          u8g2.print(F("F0 Too HIGH Freq"));  // F0 Too HIGH FREQ
          break;
        case 2:
          u8g2.print(F("F1 Too HIGH Freq"));
          break;
        case 3:
          u8g2.print(F("F2 Too HIGH Freq"));
          break;
        case 4:
          u8g2.print(F("F3 Too HIGH Freq"));
          break;
        case -1:
          u8g2.print(F("F0 Too LOW Freq "));  // F0 Too LOW FREQ
          break;
        case -2:
          u8g2.print(F("F1 Too LOW Freq"));
          break;
        case -3:
          u8g2.print(F("F2 Too LOW Freq"));
          break;
        case -4:
          u8g2.print(F("F3 Too LOW Freq"));
          break;
      }
      //********ERROR MESSAGES END******

      u8g2.sendBuffer();
      //u8g2.setClipWindow(2, 12, 17, 52); //?
      break;
    case CORE_CLOCK_MENU:
      u8g2.setCursor(51, 8);
      u8g2.print(F("SETUP"));
      u8g2.setCursor(0, 26);
      u8g2.print(F("Clock Src:               "));  //Ext. TCXO/OCXO
 
      if (curItem == &ClockSrc) {
        if (ClockSrc.bBlink == true) u8g2.setDrawColor(ClockSrc.GetColor());
        else u8g2.setDrawColor(0);
      }
      u8g2.setCursor(55, 26);
      u8g2.print(ClockSrc.ClockSourceNames[ClockSrc.value]);

      u8g2.setDrawColor(1);
      //u8g2.print(" "); //////////
      u8g2.setCursor(0, 34);
      u8g2.print(F("Clock Freq: "));
      if ((ClockSrc.value == TCXO_OCXO) || (ClockSrc.value == EXT_TCXO_OCXO)) {
        if (curItem == &ClockFreq) {
          if (ClockFreq.bBlink == true) u8g2.setDrawColor(ClockFreq.GetColor());
          else u8g2.setDrawColor(0);
        }
        u8g2.print(ClockFreq.Ref_Clk[ClockFreq.value] / 1000000);
      } else {
        /*if (curItem == &ExtClockFreqGHz) {if (ExtClockFreqGHz.bBlink==true) u8g2.setDrawColor(ExtClockFreqGHz.GetColor()); else u8g2.setDrawColor(0);}
        u8g2.print(ExtClockFreqGHz.value);
        u8g2.setDrawColor(1);*/
        if (curItem == &ExtClockFreqMHz) {
          if (ExtClockFreqMHz.bBlink == true) u8g2.setDrawColor(ExtClockFreqMHz.GetColor());
          else u8g2.setDrawColor(0);
        }
        u8g2.print(PreZero(ExtClockFreqMHz.value));
      }
      u8g2.setDrawColor(1);
      u8g2.print(F(" MHz "));

      //*********
      u8g2.setCursor(0, 42);
      u8g2.print(F("Core Clock: "));
      u8g2.setDrawColor(1);
      //u8g2.print(" ");
      if (curItem == &DDSCoreClock) {
        if (DDSCoreClock.bBlink == true) u8g2.setDrawColor(DDSCoreClock.GetColor());
        else u8g2.setDrawColor(0);
      }
      //display.print(DDSCoreClock.DDSCoreClock / 1000000);
      u8g2.print((uint32_t)(DDSCoreClock.GetDDSCoreClock() / 1000000));
      /*Serial.print("DDSCoreClock.GetDDSCoreClock()"); //4del
      Serial.println((uint32_t)DDSCoreClock.GetDDSCoreClock()); //4del*/
      u8g2.setDrawColor(1);
      u8g2.print(F(" MHz "));
      //***********

      u8g2.setCursor(0, 63);
      if (curItem == &CoreClockSave) u8g2.setDrawColor(0);
      u8g2.println(F("SAVE"));

      u8g2.setDrawColor(1);
      u8g2.setCursor(103, 63);
      if (curItem == &CoreClockExit) u8g2.setDrawColor(0);
      u8g2.println(F("EXIT"));
      u8g2.setDrawColor(1);
      u8g2.sendBuffer();
      break;
  }
  //} while ( u8g2.nextPage() );
}

void DrawBackground() {
  u8g2.clearBuffer();
  u8g2.setCursor(2, 9);
  u8g2.print(F("Ch Freq,Hz     dBm Phase}"));

  u8g2.drawHLine(0, 0, 128);
  u8g2.drawHLine(0, 11, 128);
  u8g2.drawVLine(14, 0, 52);
  u8g2.drawVLine(74, 0, 52);
  u8g2.drawVLine(94, 0, 52);
  u8g2.drawHLine(0, 52, 128);
  u8g2.drawVLine(0, 0, 52);
  u8g2.drawVLine(127, 0, 52);
  u8g2.setCursor(0, 61);
  u8g2.setCursor(88, 61);
  u8g2.print(F("TT 2025"));
  u8g2.sendBuffer();
}

String PreZero(int Digit, uint8_t Qty) {
  Qty = 3;
  switch (Qty) {
    case 3:
      if ((Digit < 100) && (Digit >= 10)) return "0" + String(Digit);
      if (Digit < 10) return "00" + String(Digit);
      return String(Digit);
    case 2:
      if (Digit < 10) return "0" + String(Digit);
      return String(Digit);
  }
}

void DisplayHello() {
  u8g2.setDrawColor(1);
  u8g2.setCursor(51, 8);
  u8g2.print(F("Hint:"));

  u8g2.setCursor(15, 25);
  u8g2.println(F("Push and hold Encoder"));
  u8g2.setCursor(15, 33);
  u8g2.println(F("   to enter Setup.   "));

  u8g2.setCursor(0, 63);
  u8g2.print(F("Firmware ver.: "));
  u8g2.print(FIRMWAREVERSION);

  u8g2.sendBuffer();
}

/*menu.cpp*/
void ClockFreqClass::incValue(int addendum)
{
    incrementValue(addendum);
    DDSCoreClock.SetNForDDSCoreFreq(BASE_DDS_CORE_CLOCK);
    DDSCoreClock.SetDDSCoreClock();
}

void ClockFreqClass::decValue(int addendum)
{
    decrementValue(addendum);
    DDSCoreClock.SetNForDDSCoreFreq(BASE_DDS_CORE_CLOCK);
    DDSCoreClock.SetDDSCoreClock();
}

void ClockSrcClass::incValue(int addendum)
{
  incrementValue(1);
  if ((value == TCXO_OCXO) || (value == EXT_TCXO_OCXO))
  {
    nextItem=&ClockFreq;
    CoreClockSave.prevItem=&DDSCoreClock;
  } else
    {
      nextItem=&ExtClockFreqMHz;
      CoreClockSave.prevItem=&ExtClockFreqMHz;
    }
}

void ClockSrcClass::decValue(int addendum)
{
  decrementValue(1);
  if ((value == TCXO_OCXO) || (value == EXT_TCXO_OCXO))
  {
    nextItem=&ClockFreq;
    CoreClockSave.prevItem=&DDSCoreClock;
  } else
    {
      nextItem=&ExtClockFreqMHz;
      CoreClockSave.prevItem=&ExtClockFreqMHz;
    }
}

void ExtClockFreqMHzClass::incValue(int addendum)
{
  incrementValue(addendum);
  DDSCoreClock.DDSCoreClock=value*1000000;
  /*if ((value>ExtMHzMin) && (ExtClockFreqGHz.value==ExtGHzMax)) ExtClockFreqGHz.value=ExtGHzMax-1;
  if ((value<ExtMHzMin) && (ExtClockFreqGHz.value==ExtGHzMin)) ExtClockFreqGHz.value=ExtGHzMin+1;*/
}

void ExtClockFreqMHzClass::decValue(int addendum)
{
  decrementValue(addendum);
  DDSCoreClock.DDSCoreClock=value*1000000;
  /*if ((value>ExtMHzMin) && (ExtClockFreqGHz.value==ExtGHzMax)) ExtClockFreqGHz.value=ExtGHzMax-1;
  if ((value<ExtMHzMin) && (ExtClockFreqGHz.value==ExtGHzMin)) ExtClockFreqGHz.value=ExtGHzMin+1;*/
}

bool DDSCoreClockClass::ExtClockFreqInRange()
{
  if (((ExtClockFreqMHz.value*1000000) < MIN_DDS_CORE_CLOCK) || ((ExtClockFreqMHz.value*1000000) > MAX_DDS_CORE_CLOCK)) return false;
    else return true;
}

uint64_t DDSCoreClockClass::GetDDSCoreClock()
{
  if ((ClockSrc.value == TCXO_OCXO) || (ClockSrc.value == EXT_TCXO_OCXO))
  {
    if (!IsFreqInRange()) SetDDSCoreClock();
    return DDSCoreClock;
  } else 
    {
      if (ExtClockFreqInRange())
      {
        return ExtClockFreqMHz.value*1000000;
      }
      else return BASE_EXT_DDS_CORE_CLOCK;
    }
}

bool CoreClockSaveClass::goToEditMode (bool editMode)
    {
      if ((ClockSrc.value == TCXO_OCXO) || (ClockSrc.value == EXT_TCXO_OCXO)) dds.setClock(DDSCoreClock.value, ClockFreq.Ref_Clk[ClockFreq.value], 0);
        else dds.setClock(0, DDSCoreClock.GetDDSCoreClock(), 0);

      DDS_Core_Clock = DDSCoreClock.GetDDSCoreClock();
      ui32HIGH_FREQ_LIMIT = DDS_Core_Clock * 0.45;

      #if DBG==1
      Serial.print("ui32HIGH_FREQ_LIMIT=");
      Serial.print(ui32HIGH_FREQ_LIMIT);
      #endif
      
      ApplyChangesToDDS();
      SaveClockSettings();
      curItem=&CoreClockExit;
      return false;
    }

void MenuLinking()
{
  // ** F Next Items**
  F0.nextItem = &F1;
  F1.nextItem = &F2;
  F2.nextItem = &F3;
  F3.nextItem = &F0; //loop back to F0 item
  // **F Prev Items**
  F3.prevItem = &F2;
  F2.prevItem = &F1;
  F1.prevItem = &F0;
  F0.prevItem = &F3; //loop back to F3 item

  // ** Freq0 Next Items**
  F0_MHz.nextItem = &F0_kHz;
  F0_kHz.nextItem = &F0_Hz;
  F0_Hz.nextItem = &F0_Amplitude;
  F0_Amplitude.nextItem = &F0_Phase; 
  F0_Phase.nextItem = &F0_PhaseFraction;
  F0_PhaseFraction.nextItem = &F0_MHz; //loop back to MHz item
  // **Freq0 Prev Items**
  F0_PhaseFraction.prevItem = &F0_Phase;
  F0_Phase.prevItem = &F0_Amplitude;
  F0_Amplitude.prevItem = &F0_Hz;
  F0_Hz.prevItem = &F0_kHz;
  F0_kHz.prevItem = &F0_MHz;
  F0_MHz.prevItem = &F0_PhaseFraction; //loop back to PhaseFractio item

    // ** Freq1 Next Items**
  F1_MHz.nextItem = &F1_kHz;
  F1_kHz.nextItem = &F1_Hz;
  F1_Hz.nextItem = &F1_Amplitude;
  F1_Amplitude.nextItem = &F1_Phase; 
  F1_Phase.nextItem = &F1_PhaseFraction;
  F1_PhaseFraction.nextItem = &F1_MHz; //loop back to MHz item
  // **Freq1 Prev Items**
  F1_PhaseFraction.prevItem = &F1_Phase;
  F1_Phase.prevItem = &F1_Amplitude;
  F1_Amplitude.prevItem = &F1_Hz;
  F1_Hz.prevItem = &F1_kHz;
  F1_kHz.prevItem = &F1_MHz;
  F1_MHz.prevItem = &F1_PhaseFraction; //loop back to PhaseFractio item

    // ** Freq2 Next Items**
  F2_MHz.nextItem = &F2_kHz;
  F2_kHz.nextItem = &F2_Hz;
  F2_Hz.nextItem = &F2_Amplitude;
  F2_Amplitude.nextItem = &F2_Phase; 
  F2_Phase.nextItem = &F2_PhaseFraction;
  F2_PhaseFraction.nextItem = &F2_MHz; //loop back to MHz item
  // **Freq2 Prev Items**
  F2_PhaseFraction.prevItem = &F2_Phase;
  F2_Phase.prevItem = &F2_Amplitude;
  F2_Amplitude.prevItem = &F2_Hz;
  F2_Hz.prevItem = &F2_kHz;
  F2_kHz.prevItem = &F2_MHz;
  F2_MHz.prevItem = &F2_PhaseFraction; //loop back to PhaseFractio item

    // ** Freq3 Next Items**
  F3_MHz.nextItem = &F3_kHz;
  F3_kHz.nextItem = &F3_Hz;
  F3_Hz.nextItem = &F3_Amplitude;
  F3_Amplitude.nextItem = &F3_Phase; 
  F3_Phase.nextItem = &F3_PhaseFraction;
  F3_PhaseFraction.nextItem = &F3_MHz; //loop back to MHz item
  // **Freq3 Prev Items**
  F3_PhaseFraction.prevItem = &F3_Phase;
  F3_Phase.prevItem = &F3_Amplitude;
  F3_Amplitude.prevItem = &F3_Hz;
  F3_Hz.prevItem = &F3_kHz;
  F3_kHz.prevItem = &F3_MHz;
  F3_MHz.prevItem = &F3_PhaseFraction; //loop back to PhaseFractio item

  // **** Parent/Child ****
  F0.childItem=&F0_MHz;
  F1.childItem=&F1_MHz;
  F2.childItem=&F2_MHz;
  F3.childItem=&F3_MHz;
  
  F0_MHz.parentItem=&F0;
  F0_kHz.parentItem=&F0;
  F0_Hz.parentItem=&F0;
  F0_Amplitude.parentItem=&F0;
  F0_Phase.parentItem=&F0;
  F0_PhaseFraction.parentItem=&F0;

  F1_MHz.parentItem=&F1;
  F1_kHz.parentItem=&F1;
  F1_Hz.parentItem=&F1;
  F1_Amplitude.parentItem=&F1;
  F1_Phase.parentItem=&F1;
  F1_PhaseFraction.parentItem=&F1;

  F2_MHz.parentItem=&F2;
  F2_kHz.parentItem=&F2;
  F2_Hz.parentItem=&F2;
  F2_Amplitude.parentItem=&F2;
  F2_Phase.parentItem=&F2;
  F2_PhaseFraction.parentItem=&F2;

  F3_MHz.parentItem=&F3;
  F3_kHz.parentItem=&F3;
  F3_Hz.parentItem=&F3;
  F3_Amplitude.parentItem=&F3;
  F3_Phase.parentItem=&F3;
  F3_PhaseFraction.parentItem=&F3;

  // ** Core clock menu items linkning **
  // ** Next Items**
  ClockSrc.nextItem=&ClockFreq;
  ClockFreq.nextItem=&DDSCoreClock;
  DDSCoreClock.nextItem=&CoreClockSave;
  CoreClockSave.nextItem=&CoreClockExit;
  CoreClockExit.nextItem=&ClockSrc; //loop back to ClockSrc item

  ExtClockFreqMHz.nextItem=&CoreClockSave;
  
  // ** Prev Items**
  CoreClockExit.prevItem=&CoreClockSave;
  CoreClockSave.prevItem=&DDSCoreClock;
  DDSCoreClock.prevItem=&ClockFreq;
  ClockFreq.prevItem=&ClockSrc;
  ClockSrc.prevItem=&CoreClockExit; //loop back to CoreClockExit item

  ExtClockFreqMHz.prevItem=&ClockSrc;

  //********parents**************
  ClockSrc.parentItem=&F0;
  ClockFreq.parentItem=&F0;
  DDSCoreClock.parentItem=&F0;
  CoreClockSave.parentItem=&F0;
  CoreClockExit.parentItem=&F0;
}

void MenuInitValues()
{
  F0_MHz.maxValue=MAX_OUT_FREQ;
  F0_kHz.maxValue=999;
  F0_Hz.maxValue=999;
  //F0_Amplitude.minValue=7;
  F0_Amplitude.minValue=3;
  F0_Amplitude.maxValue=60;
  F0_Phase.maxValue=360;
  F0_PhaseFraction.maxValue=9;

  F1_MHz.maxValue=MAX_OUT_FREQ;
  F1_kHz.maxValue=999;
  F1_Hz.maxValue=999;
  F1_Amplitude.minValue=3;
  F1_Amplitude.maxValue=60;
  F1_Phase.maxValue=360;
  F1_PhaseFraction.maxValue=9;

  F2_MHz.maxValue=MAX_OUT_FREQ;
  F2_kHz.maxValue=999;
  F2_Hz.maxValue=999;
  F2_Amplitude.minValue=3;
  F2_Amplitude.maxValue=60;
  F2_Phase.maxValue=360;
  F2_PhaseFraction.maxValue=9;

  F3_MHz.maxValue=MAX_OUT_FREQ;
  F3_kHz.maxValue=999;
  F3_Hz.maxValue=999;
  F3_Amplitude.minValue=3;
  F3_Amplitude.maxValue=60;
  F3_Phase.maxValue=360;
  F3_PhaseFraction.maxValue=9;

  ClockSrc.maxValue=EXT_OSC; //2
  ClockSrc.minValue=TCXO_OCXO; //0
  
  ClockFreq.maxValue=4;
  ClockFreq.minValue=0;

  //ui32CurrentOutputFreq=GetFreq();
  
  DDSCoreClock.DDSCoreClock=DDS_Core_Clock;
  ExtClockFreqMHz.minValue=ExtMHzMin;
  ExtClockFreqMHz.maxValue=ExtMHzMax;
  DDSCoreClock.minValue=N_MIN_VALUE;
  DDSCoreClock.maxValue=N_MAX_VALUE;

  //***********
  if (DDSCoreClock.value > N_MAX_VALUE) DDSCoreClock.value=DDSCoreClock.FindMinimalN();
  if (ClockFreq.Ref_Clk[ClockFreq.value]*(DDSCoreClock.value)*2 > MAX_DDS_CORE_CLOCK_PLL) DDSCoreClock.value=DDSCoreClock.FindMinimalN();

  if (DDSCoreClock.value < N_MIN_VALUE) DDSCoreClock.value=DDSCoreClock.FindMaximalN();
  if (ClockFreq.Ref_Clk[ClockFreq.value]*(DDSCoreClock.value)*2 < MIN_DDS_CORE_CLOCK_PLL) DDSCoreClock.value=DDSCoreClock.FindMaximalN();
  //**********
  DDSCoreClock.SetDDSCoreClock();

  //BASE_EXT_DDS_CORE_CLOCK
  ExtClockFreqMHz.value=BASE_EXT_DDS_CORE_CLOCK/1000000; //4del
}

void LoadMainSettings()
{
  // if (EEPROM.read(MAIN_SETTINGS_FLAG_ADR)!=55)
  if (1)
  {  
    F0_MHz.value=INIT_MHZ;
    F0_kHz.value=INIT_KHZ;
    F0_Hz.value=INIT_HZ;
    F0_Amplitude.value=INIT_AMPLITUDE;
    F0_Phase.value=INIT_PHASE;
    F0_PhaseFraction.value=INIT_PHASE_FRACTION;

    F1_MHz.value=INIT_MHZ;
    F1_kHz.value=INIT_KHZ;
    F1_Hz.value=INIT_HZ;
    F1_Amplitude.value=INIT_AMPLITUDE;
    F1_Phase.value=INIT_PHASE;
    F1_PhaseFraction.value=INIT_PHASE_FRACTION;

    F2_MHz.value=INIT_MHZ;
    F2_kHz.value=INIT_KHZ;
    F2_Hz.value=INIT_HZ;
    F2_Amplitude.value=INIT_AMPLITUDE;
    F2_Phase.value=INIT_PHASE;
    F2_PhaseFraction.value=INIT_PHASE_FRACTION;

    F3_MHz.value=INIT_MHZ;
    F3_kHz.value=INIT_KHZ;
    F3_Hz.value=INIT_HZ;
    F3_Amplitude.value=INIT_AMPLITUDE;
    F3_Phase.value=INIT_PHASE;
    F3_PhaseFraction.value=INIT_PHASE_FRACTION;
    // SaveMainSettings();
    
    #if DBG==1
    PrintValues();
    #endif
  }
  else
  {
    EEPROM.get(F0_MHZ_ADR, F0_MHz.value);
    EEPROM.get(F0_KHZ_ADR, F0_kHz.value);
    EEPROM.get(F0_HZ_ADR, F0_Hz.value);
    EEPROM.get(F0_AMPLITUDE_ADR, F0_Amplitude.value);
    EEPROM.get(F0_PHASE_ADR, F0_Phase.value);
    EEPROM.get(F0_PHASE_FRACTION_ADR, F0_PhaseFraction.value);

    EEPROM.get(F1_MHZ_ADR, F1_MHz.value);
    EEPROM.get(F1_KHZ_ADR, F1_kHz.value);
    EEPROM.get(F1_HZ_ADR, F1_Hz.value);
    EEPROM.get(F1_AMPLITUDE_ADR, F1_Amplitude.value);
    EEPROM.get(F1_PHASE_ADR, F1_Phase.value);
    EEPROM.get(F1_PHASE_FRACTION_ADR, F1_PhaseFraction.value);

    EEPROM.get(F2_MHZ_ADR, F2_MHz.value);
    EEPROM.get(F2_KHZ_ADR, F2_kHz.value);
    EEPROM.get(F2_HZ_ADR, F2_Hz.value);
    EEPROM.get(F2_AMPLITUDE_ADR, F2_Amplitude.value);
    EEPROM.get(F2_PHASE_ADR, F2_Phase.value);
    EEPROM.get(F2_PHASE_FRACTION_ADR, F2_PhaseFraction.value);
    
    EEPROM.get(F3_MHZ_ADR, F3_MHz.value);
    EEPROM.get(F3_KHZ_ADR, F3_kHz.value);
    EEPROM.get(F3_HZ_ADR, F3_Hz.value);
    EEPROM.get(F3_AMPLITUDE_ADR, F3_Amplitude.value);
    EEPROM.get(F3_PHASE_ADR, F3_Phase.value);
    EEPROM.get(F3_PHASE_FRACTION_ADR, F3_PhaseFraction.value);
    
    #if DBG==1
    Serial.println(F("Value from EEPROM"));
    PrintValues();
    #endif
  }
}

void SaveMainSettings()
{
  // EEPROM.put(F0_MHZ_ADR, F0_MHz.value);
  // EEPROM.put(F0_KHZ_ADR, F0_kHz.value);
  // EEPROM.put(F0_HZ_ADR, F0_Hz.value);
  // EEPROM.put(F0_AMPLITUDE_ADR, F0_Amplitude.value);
  // EEPROM.put(F0_PHASE_ADR, F0_Phase.value);
  // EEPROM.put(F0_PHASE_FRACTION_ADR, F0_PhaseFraction.value);

  // EEPROM.put(F1_MHZ_ADR, F1_MHz.value);
  // EEPROM.put(F1_KHZ_ADR, F1_kHz.value);
  // EEPROM.put(F1_HZ_ADR, F1_Hz.value);
  // EEPROM.put(F1_AMPLITUDE_ADR, F1_Amplitude.value);
  // EEPROM.put(F1_PHASE_ADR, F1_Phase.value);
  // EEPROM.put(F1_PHASE_FRACTION_ADR, F1_PhaseFraction.value);

  // EEPROM.put(F2_MHZ_ADR, F2_MHz.value);
  // EEPROM.put(F2_KHZ_ADR, F2_kHz.value);
  // EEPROM.put(F2_HZ_ADR, F2_Hz.value);
  // EEPROM.put(F2_AMPLITUDE_ADR, F2_Amplitude.value);
  // EEPROM.put(F2_PHASE_ADR, F2_Phase.value);
  // EEPROM.put(F2_PHASE_FRACTION_ADR, F2_PhaseFraction.value);
  
  // EEPROM.put(F3_MHZ_ADR, F3_MHz.value);
  // EEPROM.put(F3_KHZ_ADR, F3_kHz.value);
  // EEPROM.put(F3_HZ_ADR, F3_Hz.value);
  // EEPROM.put(F3_AMPLITUDE_ADR, F3_Amplitude.value);
  // EEPROM.put(F3_PHASE_ADR, F3_Phase.value);
  // EEPROM.put(F3_PHASE_FRACTION_ADR, F3_PhaseFraction.value);
  
  // EEPROM.write(MAIN_SETTINGS_FLAG_ADR,55);
}

#if DBG==1
void PrintValues()
{
    Serial.print(F("F0_MHz="));
    Serial.println(F0_MHz.value);
    Serial.print(F("F0_kHz="));
    Serial.println(F0_kHz.value);
    Serial.print(F("F0_Hz="));
    Serial.println(F0_Hz.value);
    Serial.print(F("F0_Amplitude="));
    Serial.println(F0_Amplitude.value);
    Serial.print(F("F0_Phase="));
    Serial.println(F0_Phase.value);
    Serial.print(F("F0_PhaseFraction="));
    Serial.println(F0_PhaseFraction.value);
    
    Serial.print(F("F1_MHz="));
    Serial.println(F1_MHz.value);
    Serial.print(F("F1_kHz="));
    Serial.println(F1_kHz.value);
    Serial.print(F("F1_Hz="));
    Serial.println(F1_Hz.value);
    Serial.print(F("F1_Amplitude="));
    Serial.println(F1_Amplitude.value);
    Serial.print(F("F1_Phase="));
    Serial.println(F1_Phase.value);
    Serial.print(F("F1_PhaseFraction="));
    Serial.println(F1_PhaseFraction.value);

    Serial.print(F("F2_MHz="));
    Serial.println(F2_MHz.value);
    Serial.print(F("F2_kHz="));
    Serial.println(F2_kHz.value);
    Serial.print(F("F2_Hz="));
    Serial.println(F2_Hz.value);
    Serial.print(F("F2_Amplitude="));
    Serial.println(F2_Amplitude.value);
    Serial.print(F("F2_Phase="));
    Serial.println(F2_Phase.value);
    Serial.print(F("F2_PhaseFraction="));
    Serial.println(F2_PhaseFraction.value);
    
    Serial.print(F("F3_MHz="));
    Serial.println(F3_MHz.value);
    Serial.print(F("F3_kHz="));
    Serial.println(F3_kHz.value);
    Serial.print(F("F3_Hz="));
    Serial.println(F3_Hz.value);
    Serial.print(F("F3_Amplitude="));
    Serial.println(F3_Amplitude.value);
    Serial.print(F("F3_Phase="));
    Serial.println(F3_Phase.value);
    Serial.print(F("F3_PhaseFraction="));
    Serial.println(F3_PhaseFraction.value);
}
#endif

void LoadClockSettings()
{
  // if (EEPROM.read(CLOCK_SETTINGS_FLAG_ADR)!=55)
  if (1)
  {
    ClockSrc.value=INIT_CLOCK_SOURCE_INDEX;
    ClockFreq.value=INIT_CLOCK_FREQ_INDEX;
    ExtClockFreqMHz.value=INIT_EXT_CLOCK_FREQ_MHZ;
    DDSCoreClock.value=INIT_DDS_CORE_CLOCK_N;
    #if DBG==1
    Serial.println(F("Set default clock's settings"));
    Serial.print(F("ClockSrc.value="));
    Serial.println(ClockSrc.value);
    Serial.print(F("ClockFreq.value="));
    Serial.println(ClockFreq.value);
    Serial.print(F("ExtClockFreqMHz.value="));
    Serial.println(ExtClockFreqMHz.value);
    Serial.print(F("DDSCoreClock.value="));
    Serial.println(DDSCoreClock.value);
    #endif

    // SaveClockSettings();
  } else
  {
    EEPROM.get(CLOCK_SOURCE_ADR, ClockSrc.value);
    EEPROM.get(CLOCK_TCXO_FREQ_INDEX_ADR, ClockFreq.value);
    EEPROM.get(EXT_CLOCK_FREQ_MHZ_ADR, ExtClockFreqMHz.value);
    EEPROM.get(DDS_CORE_CLOCK_N_ADR, DDSCoreClock.value);

    #if DBG==1
    Serial.println(F("Reading clock's settings from EEPROM"));
    Serial.print(F("ClockSrc.value="));
    Serial.println(ClockSrc.value);
    Serial.print(F("ClockFreq.value="));
    Serial.println(ClockFreq.value);
    Serial.print(F("ExtClockFreqMHz.value="));
    Serial.println(ExtClockFreqMHz.value);
    Serial.print(F("DDSCoreClock.value="));
    Serial.println(DDSCoreClock.value);
    #endif
  }

  selectClockSrcPath(ClockSrc.value);

  //***********
  if (DDSCoreClock.value > N_MAX_VALUE) DDSCoreClock.value=DDSCoreClock.FindMinimalN();
  if (ClockFreq.Ref_Clk[ClockFreq.value]*(DDSCoreClock.value) > MAX_DDS_CORE_CLOCK_PLL) DDSCoreClock.value=DDSCoreClock.FindMinimalN();

  if (DDSCoreClock.value < N_MIN_VALUE) DDSCoreClock.value=DDSCoreClock.FindMaximalN();
  if (ClockFreq.Ref_Clk[ClockFreq.value]*(DDSCoreClock.value) < MIN_DDS_CORE_CLOCK_PLL) DDSCoreClock.value=DDSCoreClock.FindMaximalN();
  //**********
  DDSCoreClock.SetDDSCoreClock();

  if ((ClockSrc.value == TCXO_OCXO) || (ClockSrc.value == EXT_TCXO_OCXO)) dds.setClock(DDSCoreClock.value, ClockFreq.Ref_Clk[ClockFreq.value], 0);
        else dds.setClock(0, DDSCoreClock.GetDDSCoreClock(), 0);

  Serial.println(F("CoreClock"));

  #if DBG==1
  Serial.print("DDSCoreClock.value=");
  Serial.println(DDSCoreClock.value);

  Serial.print("ClockFreq.Ref_Clk[ClockFreq.value]=");
  Serial.println(ClockFreq.Ref_Clk[ClockFreq.value]);
  #endif

  if ((ClockSrc.value == TCXO_OCXO) || (ClockSrc.value == EXT_TCXO_OCXO))
  {
    ClockSrc.nextItem=&ClockFreq;
    CoreClockSave.prevItem=&DDSCoreClock;
  } else
    {
      ClockSrc.nextItem=&ExtClockFreqMHz;
      CoreClockSave.prevItem=&ExtClockFreqMHz;
    }

  DDS_Core_Clock = DDSCoreClock.GetDDSCoreClock();
  ui32HIGH_FREQ_LIMIT = DDS_Core_Clock * 0.45;
}

void SaveClockSettings()
{
    EEPROM.put(CLOCK_SOURCE_ADR, ClockSrc.value);
    EEPROM.put(CLOCK_TCXO_FREQ_INDEX_ADR, ClockFreq.value);
    EEPROM.put(EXT_CLOCK_FREQ_MHZ_ADR, ExtClockFreqMHz.value);
    EEPROM.put(DDS_CORE_CLOCK_N_ADR, DDSCoreClock.value);
    
    EEPROM.write(CLOCK_SETTINGS_FLAG_ADR, 55);

    //selectClockSrcPath(ClockSrc.value);

    LoadClockSettings();

    //DisplayMessage("SETUP", "SAVED");
    DisplaySaved();
    delay(1000);
}

void DisplaySaved(void)
{
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.setCursor(51, 8);
  u8g2.println(F("SETUP"));
  u8g2.setCursor(51, 40);
  u8g2.print(F("SAVED"));
  
  u8g2.sendBuffer();
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println(F("DDS AD9959 by GRA & AFCH"));
  Serial.println(F("HW v2.x"));
  Serial.print(F("SW v"));
  Serial.println(FIRMWAREVERSION);
  
  u8g2.setBusClock(800000);
  //u8g2.setBusClock(1000000UL);
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.setFontMode(0);
  u8g2.setFont(u8g2_font_5x8_custom_mr);
  u8g2.clearBuffer();

  DisplayHello();
  delay(3000);
  u8g2.clearBuffer();
  
  DrawBackground();
  //u8g2.writeBufferPBM(Serial);
  MenuLinking();
  MenuInitValues();

  modeButton.Update();
  if (modeButton.depressed == true) //если при включении была зажата кнопка MODE, то затираем управляющие флаги в EEPROM, которые восстановят заводские значения всех параметров
  {
    // EEPROM.write(CLOCK_SETTINGS_FLAG_ADR, 255); //flag that force save default clock settings to EEPROM 
    // EEPROM.write(MAIN_SETTINGS_FLAG_ADR, 255); //flag that force save default main settings to EEPROM 
  }
  
  pinMode(POWER_DOWN_CONTROL_PIN, OUTPUT);
  digitalWrite(POWER_DOWN_CONTROL_PIN, LOW);

  pinMode(SDIO_1_PIN, INPUT);

  pinMode(SDIO_3_PIN, OUTPUT);
  digitalWrite(SDIO_3_PIN, LOW);
  
  pinMode(P0_PIN, OUTPUT);
  pinMode(P1_PIN, OUTPUT);
  pinMode(P2_PIN, OUTPUT);
  pinMode(P3_PIN, OUTPUT);
  digitalWrite(P0_PIN, LOW);
  digitalWrite(P1_PIN, LOW);
  digitalWrite(P2_PIN, LOW);
  digitalWrite(P3_PIN, LOW);

  setupEncoder();

  // pinMode(TCXO_POWER_PIN, OUTPUT);
  // pinMode(TCXO_PATH_PIN, OUTPUT);
  // pinMode(EXTERANL_SRC_PATH_PIN, OUTPUT);
  // digitalWrite(TCXO_POWER_PIN, LOW);
  // digitalWrite(TCXO_PATH_PIN, LOW);
  // digitalWrite(EXTERANL_SRC_PATH_PIN, LOW);

  LoadClockSettings();
  LoadMainSettings();

  //debug
   /*SPI.beginTransaction(SPISettings(2000000, 1, SPI_MODE3));
   digitalWrite(6, LOW);

    SPI.transfer(0x1);
    SPI.transfer(0x80 | 10*0x04 | 0x03);
    SPI.transfer(0x00 | 0x00 | 0x00);
    SPI.transfer(0x20); // Don't output SYNC_CLK
    //SPI.transfer(0x00);

   digitalWrite(6, HIGH);
   SPI.endTransaction();

   SPI.beginTransaction(SPISettings(2000000, 1, SPI_MODE3));
   digitalWrite(6, LOW);

    SPI.transfer(0x2);
    SPI.transfer(B11101111);
    SPI.transfer(B11111111);

   digitalWrite(6, HIGH);
   SPI.endTransaction();

  dds.setFrequency(MyAD9959::Channel0, 100000000); 
  dds.setAmplitude(MyAD9959::Channel0, dBmToASF(10)); 

  dds.setFrequency(MyAD9959::Channel1, 100000000); 
  dds.setAmplitude(MyAD9959::Channel1, dBmToASF(10)); 

  dds.setFrequency(MyAD9959::Channel2, 100000000); 
  dds.setAmplitude(MyAD9959::Channel2, dBmToASF(10)); 

  dds.setFrequency(MyAD9959::Channel3, 100000000); 
  dds.setAmplitude(MyAD9959::Channel3, dBmToASF(10)); 

  dds.update();
   while(1) */
  //debug end
  
  dds.AllChanAutoClearPhase();
  ApplyChangesToDDS();

  curItem = &F0;

  modeButton.debounceTime   = 25;   // Debounce timer in ms
  modeButton.multiclickTime = 10;  // Time limit for multi clicks
  modeButton.longClickTime  = 1000; // time until "held-down clicks" register

  backButton.debounceTime   = 25;   // Debounce timer in ms
  backButton.multiclickTime = 10;  // Time limit for multi clicks
  backButton.longClickTime  = 1000; // time until "held-down clicks" register

  /*curItem = &ClockSrc; //4del
  menuType = CORE_CLOCK_MENU; //4del 
  u8g2.clearBuffer(); //4del*/
  
  DisplayMenu(menuType); 
  attachInterrupt(digitalPinToInterrupt(MODE_PIN), ModeButtonDown, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_PIN), DownButtonDown, CHANGE);
  
}

void loop() 
{ 
  while (1)
  {
    //u8g2.firstPage();
    ReadSerialCommands();
    // int curPos=0;
    // curPos=myEnc.read();
    int curPos = getEncoderPosition();
    
    modeButton.Update();
    backButton.Update();

    if (modeButton.clicks == 1) 
    {
      //MenuEditMode=!MenuEditMode;
      MenuEditMode=curItem->goToEditMode(MenuEditMode);
      DisplayMenu(menuType);
    }

    if (backButton.clicks > 0) 
    {
      curItem->bBlink=false;
      curItem=curItem->moveToParentItem();
      MenuEditMode=false;
      //Serial.println("Back btn pressed");
      LoadClockSettings();
      u8g2.clearBuffer();
      DrawBackground();
      DisplayMenu(menuType);
    }
    
    if ((modeButton.clicks == -1) && (modeButton.depressed == true)) 
    {
      curItem = &ClockSrc; 
      menuType = CORE_CLOCK_MENU; 
      u8g2.clearBuffer();
      DisplayMenu(menuType);
    } 

    if (curPos>0)
    {
      switch(MenuEditMode)
      {
        case true: 
          curItem->incValue(curPos); 
          ApplyChangesToDDS(); //4del
          /*ui32CurrentOutputFreq=GetFreq(); 
          DDS.setFreq(ui32CurrentOutputFreq,0); 
          DDS.setAmpdB(Amplitude.value * -1, 0); */ //uncomment
          if (menuType == MAIN_MENU) SaveMainSettings();
        break;
        case false: curItem=curItem->moveToNextItem(); break;
      }
      DisplayMenu(menuType);
    }
    
    if (curPos<0)
    {
      switch(MenuEditMode)
      {
        case true: 
          curItem->decValue(curPos);
          /*
          ui32CurrentOutputFreq=GetFreq();
          DDS.setFreq(ui32CurrentOutputFreq,0);
          DDS.setAmpdB(Amplitude.value * -1, 0);*/ //uncomment
          ApplyChangesToDDS(); //4del
          if (menuType == MAIN_MENU) SaveMainSettings(); 
        break;
        case false: curItem=curItem->moveToPrevItem(); break;
      }
      DisplayMenu(menuType);
    }
    /*uint32_t prev;
    prev=millis();*/
    DisplayMenu(menuType);  
    //Serial.println(millis()-prev);
  }
}