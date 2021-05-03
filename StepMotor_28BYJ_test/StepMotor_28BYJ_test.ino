const int motorEnablePin = 2;
const int motorDirPin = 3;
const int motorStepPin = 4;


unsigned int Vmin = 100; // угловая скорость начальная
// угл. скорость макс. для 130 мм/сек, шкив Ф 42 мм, пер.число редуктора 64, полушаг
unsigned int Vmax = (130 / (PI * 42)) * 64 * 32 *2;  // для 130 мм/сек
unsigned int T = 3;              // время разгона (сек.)
unsigned int M = 100;            // дискретных интервалов (/сек.) 
unsigned int N = T * M;          // дискретных интервалов всего   250
unsigned int K = (Vmax - Vmin) / N;      // изменение скорости за интервал 520

int dir = 0;

void setup() {
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorStepPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  digitalWrite(motorEnablePin, 1);
  digitalWrite(motorDirPin, dir);
  Serial.begin(115200);
}

void loop() {

  digitalWrite(motorEnablePin, 0);
  
  // цикл по интервалам
  
  // разгон
  for (int i = 1; i <= N; i++) {
    // вычисляем период шага для интервала в мкс
    unsigned int P = 1000000 / (Vmin + K * i);
    // выдаем импульсы для интервала
    for (unsigned int j = 1; j <= K * i / M; j++) {
      digitalWrite(motorStepPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(motorStepPin, LOW);
      delayMicroseconds(P - 5);
    }
    //Serial.println(P);
  }
  
  // постоянное движение
  for (int j = 1; j <= 20000; j++) {
      digitalWrite(motorStepPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(motorStepPin, LOW);
      delayMicroseconds((unsigned int) (1000000 / Vmax) - 5);
  }
  
  // торможение
  for (int i = N; i >= 1; i--) {
    // вычисляем период шага для интервала в мкс
    unsigned int P = 1000000 / (Vmin + K * i);
    // выдаем импульсы для интервала
    for (unsigned int j = 1; j <= K * i / M; j++) {
      digitalWrite(motorStepPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(motorStepPin, LOW);
      delayMicroseconds(P - 5);
    }
  }  
  
  digitalWrite(motorEnablePin, 1);
  delay(2000);
  
  dir = (dir == 0)?1:0;
  digitalWrite(motorDirPin, dir);
}  

