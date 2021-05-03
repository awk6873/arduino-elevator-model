/*******************************************************************************

 Bare Conductive MPR121 library
 ------------------------------
 
 SimpleTouch.ino - simple MPR121 touch detection demo with serial output
 
 Based on code by Jim Lindblom and plenty of inspiration from the Freescale 
 Semiconductor datasheets and application notes.
 
 Bare Conductive code written by Stefan Dzisiewski-Smith and Peter Krige.
 
 This work is licensed under a MIT license https://opensource.org/licenses/MIT
 
 Copyright (c) 2016, Bare Conductive
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

*******************************************************************************/

#include <MPR121.h>
#include <Wire.h>


#define FLOORS 5                         // кол-во этажей

// адрес i2c MPR121
#define MPR121_ADDRESS 0x5A
// порт для IRQ MPR121
#define MPR121_IRQ_PIN A3

// группа кнопок на этажах - MPR121 (Port C:A3, PCINT1)  
// соответствие пинов MPR121 кнопкам
const int floorButtonPins[FLOORS + 1] = 
  {0, 0, 1, 2, 3, 4};
// соответствие пинов MPR121 индикации  
const int floorLEDPins[FLOORS + 1] = 
  {0, 5, 6, 7, 8, 9};
// маска разрядов пинов индикации  
const int floorLEDBitsMask = 0b111110;  
  
volatile int buttons = 0;
byte PCINT1BitsMask = 0b1000;

// обработчик прерываний от контроллера кнопок на этажах (A3)
ISR(PCINT1_vect) {

  // разрешаем прерывания для нормальной работы i2c
  interrupts();
  // получаем состояние кнопок
  buttons = (unsigned int)MPR121.getRegister(MPR121_TS1) + ((unsigned int)MPR121.getRegister(MPR121_TS2)<<8);

  // определяем только что "нажатые" кнопки
  for (int i = 1; i <= FLOORS; i++) {
    // !!!
    if (buttons & _BV(floorButtonPins[i]))
      // запомнить нажатие!!!
      // включаем светодиоды индикации
      MPR121.digitalWrite(floorLEDPins[i], 1);
    else 
      MPR121.digitalWrite(floorLEDPins[i], 0);
  }
}

void setup()
{
  Serial.begin(9600);
  while(!Serial);  // only needed if you want serial feedback with the
  		   // Arduino Leonardo or Bare Touch Board

  // инициализация MPR121
  if(!MPR121.begin(MPR121_ADDRESS)){ 
    Serial.print("Error setting up MPR121: ");  
    switch(MPR121.getError()){
      case NO_ERROR:
        Serial.println("no error");
        break;  
      case ADDRESS_UNKNOWN:
        Serial.println("incorrect address");
        break;
      case READBACK_FAIL:
        Serial.println("readback failure");
        break;
      case OVERCURRENT_FLAG:
        Serial.println("overcurrent on REXT pin");
        break;      
      case OUT_OF_RANGE:
        Serial.println("electrode out of range");
        break;
      case NOT_INITED:
        Serial.println("not initialised");
        break;
      default:
        Serial.println("unknown error");
        break;      
    }
    while(1);
  }

  // переводим в режим Стоп
  MPR121.stop();
  // задаем IRQ
  MPR121.setInterruptPin(MPR121_IRQ_PIN);
  // порог значения нажатия кнопки
  MPR121.setTouchThreshold(40);
  // порог значения отпускания кнопки
  MPR121.setReleaseThreshold(20);  
  // частота i2c 400 кГц
  MPR121.goFast();

  // включаем автоконфигурацию
  MPR121.setRegister(MPR121_ACCR0, 0b11001011);
  MPR121.setRegister(MPR121_USL, 202);  // (3.3 - 0.7) / 3.3 * 256
  MPR121.setRegister(MPR121_LSL, 131);  // (3.3 - 0.7) / 3.3 * 256 * 0.65
  MPR121.setRegister(MPR121_TL,  182);  // (3.3 - 0.7) / 3.3 * 256 * 0.9

  // указываем порты кнопок
  MPR121.setNumEnabledElectrodes(FLOORS);
  // запускаем
  MPR121.run();
  
  // порты индикации
  // каждый инициализируем на вывод, открытый, HIGH SIDE
  MPR121.setRegister(MPR121_EN, floorLEDBitsMask);
  MPR121.setRegister(MPR121_DIR, floorLEDBitsMask);
  MPR121.setRegister(MPR121_CTL0, floorLEDBitsMask);
  MPR121.setRegister(MPR121_CTL1, floorLEDBitsMask);
  MPR121.setRegister(MPR121_DAT, 0);
  
  // мигаем индикаторами
  for (int i = 1; i <= FLOORS; i++) {
    MPR121.digitalWrite(floorLEDPins[i], 1);
    delay(200);
    MPR121.digitalWrite(floorLEDPins[i], 0);
  }
  
  // получаем начальное состояние кнопок
  MPR121.updateTouchData();

  // порт прерываний от MPR121
  pinMode(MPR121_IRQ_PIN, INPUT_PULLUP);
  noInterrupts(); 
  PCICR = 0b00000011;       // разрешаем прерывания PCINT0 и PCINT1
  // PCMSK0 = buttonBitsMask;  // от пинов кнопок в кабине
  PCMSK1 = PCINT1BitsMask;  // от пина контроллера кнопок на этажах
  interrupts();
}

void loop()
{

  Serial.print(buttons);
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDC0));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDC1));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDC2));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDC3));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDC4));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDT01));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDT23));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_CDT45));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_E0BV));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_E1BV));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_E2BV));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_E3BV));
  Serial.print("   ");
  Serial.print(MPR121.getRegister(MPR121_E4BV));
  Serial.println();
  delay(1000);
  
  /*
  if(MPR121.touchStatusChanged()){
    MPR121.updateTouchData();
    for(int i=0; i<numElectrodes; i++){
      if(MPR121.isNewTouch(i)){
        Serial.print("electrode ");
        Serial.print(i, DEC);
        Serial.println(" was just touched");  
      } else if(MPR121.isNewRelease(i)){
        Serial.print("electrode ");
        Serial.print(i, DEC);
        Serial.println(" was just released");  
      }
    } 
  }
  */
}
