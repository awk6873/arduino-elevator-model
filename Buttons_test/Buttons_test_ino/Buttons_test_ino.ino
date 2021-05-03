  
volatile int currentFloor = 0;

// обработчик прерываний от кнопок на этажах (A0-A4)
ISR(PCINT1_vect) {
  static uint8_t reg_prev = 0b111;
  uint8_t reg, diff;
  
  if (currentFloor != 0)
    return;
  
  reg = PINC & 0b111;
  diff = reg ^ reg_prev;
  reg_prev = reg;
  
  for (int i = 1; i <= 3; i++) {
    if (diff & _BV(i-1) && !(reg & _BV(i-1))) {
      DDRC |= _BV(i-1);
      PORTC &= ~(_BV(i-1));
      currentFloor = i;
    }
  }  
}

void setup() {
  
  Serial.begin(9600);
  
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);

  noInterrupts();
  PCICR = 0b00000010;   // Разрешаем прерывания PCINT0 и PCINT1
  PCMSK0 = 0b00011111;  // от 5 младших пинов
  PCMSK1 = 0b00011111;
  interrupts();  
} 
 
void loop() {

  Serial.println(currentFloor);
  delay(2000);
  currentFloor = 0;
  
}  
