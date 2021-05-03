int stp = 13;
int dir = 12;

unsigned int Vmin = 100;
unsigned int Vmax = 130 / (PI * 42) * 64 * 32 *2;  // 1550
unsigned int T = 3;              // время разгона (сек.)
unsigned int M = 100;            // дискретных интервалов (/сек.) 
unsigned int N = T * M;          // дискретных интервалов всего   250
unsigned int K = (Vmax - Vmin) / N;      // изменение скорости за интервал 520

void setup() {
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  digitalWrite(dir, 1);
  Serial.begin(115200);
}

void loop() {

  // цикл по интервалам
  
  for (int i = 1; i <= N; i++) {
    // вычисляем период шага для интервала в мкс
    unsigned int P = 1000000 / (Vmin + K * i);
    // выдаем импульсы для интервала
    for (unsigned int j = 1; j <= K * i / M; j++) {
      digitalWrite(stp, HIGH);
      delayMicroseconds(5);
      digitalWrite(stp, LOW);
      delayMicroseconds(P - 5);
    }
    //Serial.println(P);
  }
  for (int j = 1; j <= 20000; j++) {
      digitalWrite(stp, HIGH);
      delayMicroseconds(5);
      digitalWrite(stp, LOW);
      delayMicroseconds((unsigned int) (1000000 / Vmax) - 5);
  }
  for (int i = N; i >= 1; i--) {
    // вычисляем период шага для интервала в мкс
    unsigned int P = 1000000 / (Vmin + K * i);
    // выдаем импульсы для интервала
    for (unsigned int j = 1; j <= K * i / M; j++) {
      digitalWrite(stp, HIGH);
      delayMicroseconds(5);
      digitalWrite(stp, LOW);
      delayMicroseconds(P - 5);
    }
  }  
  delay(2000);
}  

