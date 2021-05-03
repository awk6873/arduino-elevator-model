#define DEBUG

const int motorEnablePin = 2;
const int motorDirPin = 3;
const int motorStepPin = 4;


const int motorDirUp = 0;                // сигнал DIR драйвера ШД для подъема/спуска кабины
const int motorDirDown = 1;
const int motorEnable = 0;               // сигнал Enable ШД
const int motorDisable = 1;

const long motorVmin = 100;         // скорость начальная, имп./сек. 
// скорость макс. для 130 мм/сек, шкив Ф 42 мм, пер.число редуктора 64, режим полушаг, имп./сек.
const long motorVmax = (130 / (PI * 42)) * 64 * 32 *2;  // ~ 4033
const int motorAccSteps = 2000;  // максимальное число шагов для разгона/торможения
const long motorAcc = (motorVmax * motorVmax - motorVmin * motorVmin) / (2 * motorAccSteps); // ускорение


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
  
int steps;
float V;
long accFactor;
  
  // число шагов
  steps = 25000;
  
  // определяем границы областей разгона и торможения - не больше 1/3 от всего перемещения
  int accLastStep = steps / 3;
  if (accLastStep > motorAccSteps) accLastStep = motorAccSteps;
  int decFirstStep = steps - accLastStep;
  
  // разгон от начальной скорости
  V = motorVmin;
  
  #ifdef DEBUG
  Serial.print("Move motor with steps: ");
  Serial.print(steps);
  Serial.print(", direction: ");
  Serial.print(dir);
  Serial.print(", acceleration: ");
  Serial.print(motorAcc);
  Serial.print(", accel.steps: ");
  Serial.println(accLastStep);
  #endif
  
  // включаем драйвер
  digitalWrite(motorEnablePin, motorEnable);
  
  // цикл по шагам перемещения
  for (int i = 1; i <= steps; i++) {
    // засекам время
    unsigned long t = micros();
 
    // формируем импульс  
    digitalWrite(motorStepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(motorStepPin, LOW);
    
    // вычисляем период импульса
    unsigned long T = 1000000 / V;
    
    // вычисляем скорость для следующего шага
    if (i <= accLastStep) {
      // находимся в зоне разгона
      V = sqrt(V * V + 2 * motorAcc);
      if (V > motorVmax) V = motorVmax;
    }
    else if (i >= decFirstStep) {
      // находимся в зоне торможения  
      V = sqrt(V * V - 2 * motorAcc);
      if (V < motorVmin) V = motorVmin;
    }  

    // задержка для формирования периода импульса
    delayMicroseconds(T - (micros() - t));

    //Serial.print(i);
    //Serial.print(';');
    //Serial.println(V);
  }

  // выключаем драйвер
  digitalWrite(motorEnablePin, motorDisable);
  delay(2000);

  //while(1);
  
  dir = (dir == motorDirUp)?motorDirDown:motorDirUp;
  digitalWrite(motorDirPin, dir);
}  

