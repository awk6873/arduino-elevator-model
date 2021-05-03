int stp = 13;

int dir = 12;


int Vmax = 50 / (PI * 48) * 64 * 2048;

int T = 3;     // время разгона (сек.)

int M = 100;   // дискретных интервалов (/сек.)

int N = T * M; // дискретных интервалов всего

int K = Vmax / N;  // приращение скорости за интервал 




void setup() {
  

  pinMode(stp, OUTPUT);
  
  pinMode(dir, OUTPUT);

  Serial.begin(115200);

}



void loop() {

  // цикл по интервалам

  for (int i = 1; i <= N; i++) {

    // вычисляем период шага для интервала в мкс

    int P = 1000000 / (K * i);

    // выдаем импульсы для интервала

    for (int j = 1; j <= i; j++) {

      digitalWrite(stp, HIGH);

      delayMicroseconds(1);

      digitalWrite(stp, LOW);

      delayMicroseconds(P);

    }
  
  Serial.println(K * i);

  } 
  
  delay(2000);

}  
