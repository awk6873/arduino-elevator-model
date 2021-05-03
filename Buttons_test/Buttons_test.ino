// группы кнопок: для кабины - D8-D12 (PCINT0), на этажах - A0-A4 (PCINT1)

// обработчик прерываний от кнопок кабины (D8-D12)
ISR(PCINT0_vect) {
    

}

/*
// обработчик прерываний от кнопок на этажах (A0-A4)
ISR(PCINT1_vect) {
  volatile static byte PINC_prev;
  byte diff;
  
  // определяем маску изменившихся пинов
  diff = PINC ^ PINC_prev;
  // сохраняем состояние 
  PINC_prev = PINC;
  
  Serial.println(PINC, BIN);
  Serial.println(PINC_prev, BIN);
  Serial.println(diff, BIN);
  Serial.println(PINC & diff, BIN);
  
  for (int i = 1; i <= 5; i++) {
    //if (PINC & diff & _BV(i-1) == 0) {
      // на пине активный сигнал
      
      // фиксируем индикацию кнопки вызова
      DDRC |= _BV(i-1);        // переключаем пин на выход
      PORTC &= ~(_BV(i-1));    // выводим 0
      

    //}
  }  
  
  digitalWrite(13, 1);
  delay(10);
  digitalWrite(13, 0);
}
*/


void setup() {
  
  Serial.begin(9600);
  
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  
  noInterrupts();
  //PCICR = 0b00000011;   // Разрешаем прерывания PCINT0 и PCINT1
  //PCMSK0 = 0b00011111;  // от 5 младших пинов
  //PCMSK1 = 0b00011111;
  interrupts();
}
    
void loop() {
  volatile static byte PINC_prev;
  byte diff;
  
  // определяем маску изменившихся пинов
  diff = 0b11;
  // сохраняем состояние 
  PINC_prev = PINC;
  
  //Serial.println(diff, BIN);
  //Serial.println(PINC & diff, BIN);
  
  for (int i = 1; i <= 5; i++) {
    Serial.println(PINC & diff & _BV(i-1) & 0b0, BIN);
    if (PINC & diff & _BV(i-1) & 0b0 == 0b0) {
      // на пине активный сигнал
      
      Serial.println("yes");
      // фиксируем индикацию кнопки вызова
      DDRC |= _BV(i-1);        // переключаем пин на выход
      PORTC &= ~(_BV(i-1));    // выводим 0
      

    }
  } 
  delay(2000); 
  /*
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  delay(5000);
  */
}
