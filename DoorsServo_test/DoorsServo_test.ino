#define DEBUG


// пины для сервы дверей
const int doorsServoPin = A5;
const int doorsSensorPin = A6;

// перемещение дверей в положение
const int Open = 1;
const int Close = 0;
const int SemiOpen = 2;

const int doorsOpenPWM = 2400;         // ШИМ для серво дверей, мкс
const int doorsClosedPWM = 600;
const int doorsSemiOpenPWM = 1000;      // ШИМ приоткрытия дверей при инициализации
const int doorsShiftTime = 1800;       // время на перемещение дверей, мс
const int doorsStepTime = 20;          // период импульсов серво дверей, мс
const int doorsFinishPulses = 3;       // число импульсов для "довода" в конечное положение
int doorsStepPWM = (doorsOpenPWM - doorsClosedPWM) / (doorsShiftTime / doorsStepTime); // шаг ШИМ
const int doorsSensorThreshold = 980;  // порог значения АЦП датчика тока серво
const int doorsTryLimit = 10;          // количество попыток перемещения дверей
const int doorsTryDelay = 1000;        // задержка между попытками, мс
int doorsPWM;

void setup() {
  
  pinMode(doorsServoPin, OUTPUT);
  digitalWrite(doorsServoPin, 0);
  pinMode(doorsSensorPin, INPUT);

  Serial.begin(115200);
  
  // двери в приоткрытое положение
  doorsPWM = doorsClosedPWM;
  doorsShift(SemiOpen, 0); 
  delay(1000); 
}

void loop() {

  doorsShift(Close, 1);
  delay(3000);
  doorsShift(Open, 1);
  delay(3000);
  
}


// открывание/закрывание/приоткрывание дверей
void doorsShift(int dir, int sensorOnOff) {
  int doorsSrcPWM;
  int doorsTgtPWM;
  int doorsStepPWMCurr;
  int checkSensor = 0;
  int blockage = 0;
  int i;
 
  // определяем начальное и конечное положение и знак шага
  if (dir == Close) {
    // закрывание
    doorsSrcPWM = doorsOpenPWM;
    doorsTgtPWM = doorsClosedPWM;
    doorsStepPWMCurr = -1 * doorsStepPWM;
  }
  else {
    if (dir == Open) {
      // открывание
      doorsSrcPWM = doorsClosedPWM;
      doorsTgtPWM = doorsOpenPWM;
      doorsStepPWMCurr = doorsStepPWM;  
      }
    else {
      // приоткрывание
      doorsSrcPWM = doorsClosedPWM;
      doorsTgtPWM = doorsSemiOpenPWM;
      doorsStepPWMCurr = doorsStepPWM;
    }
  }

  // цикл попыток перемещения дверей
  for (i = 1; i < doorsTryLimit; i++) {
    #ifdef DEBUG
    Serial.println();
    Serial.print("Doors shift trying: ");   
    Serial.print(i);
    Serial.print(", from PWM: ");   
    Serial.print(doorsPWM);
    Serial.print(" to: ");   
    Serial.print(doorsTgtPWM);
    Serial.print(", step: ");   
    Serial.println(doorsStepPWMCurr);
    #endif  
    
    // перемещаемся к конечному положению, пока не перескочили через него
    while (doorsStepPWMCurr > 0 && doorsPWM <= doorsTgtPWM || doorsStepPWMCurr < 0 && doorsPWM >= doorsTgtPWM) {
      #ifdef DEBUG
      //Serial.print(doorsPWM);
      //Serial.print(' ');
      #endif
      // импульс на серву 
      digitalWrite(doorsServoPin, HIGH);
      delayMicroseconds(doorsPWM);
      digitalWrite(doorsServoPin, LOW);
      // задержка для периода управления сервой
      delay(doorsStepTime);

      // замеряем ток
      int sensor = analogRead(doorsSensorPin);
      
      // признак необходимости проверки препятствия
      if (checkSensor > 0)
        checkSensor--;
        
      // проверяем препятствие
      if (sensorOnOff > 0 && checkSensor == 0 && sensor < doorsSensorThreshold) {
        // есть препятствие
        // для движения в обратную сторону меняем местами начальное и конечное положение и знак шага
        int tmp = doorsTgtPWM; doorsTgtPWM = doorsSrcPWM; doorsSrcPWM = tmp;
        doorsStepPWMCurr *= -1;
        // откатываем текущее положение на 5 шагов 
        doorsPWM += 5 * doorsStepPWMCurr;
        // устанавливаем признак препятствия
        blockage = 1;
        // блокируем проверку препятствия на несколько последующих шагов
        checkSensor = 10;
        #ifdef DEBUG
        Serial.println();
        Serial.print("Doors sensor level: ");
        Serial.print(sensor);
        Serial.print(", shift back from PWM: ");   
        Serial.print(doorsPWM);
        Serial.print(" to: ");   
        Serial.print(doorsTgtPWM);
        Serial.print(", step: ");   
        Serial.println(doorsStepPWMCurr);
        #endif
      }  

      // двигаем значение ШИМ
      doorsPWM += doorsStepPWMCurr;
    }
        
    if (blockage != 0) {
      // было препятствие, 
      // восстанавливаем начальное и конечное положения и знак шага
      int tmp = doorsTgtPWM; doorsTgtPWM = doorsSrcPWM; doorsSrcPWM = tmp;
      doorsStepPWMCurr *= -1;
      
      // сбрасываем признак препятствия
      blockage = 0;
      
      // и делаем следующую попытку с задержкой
      delay(doorsTryDelay);
      continue;
    }

    // доводим двери в конечное положение
    doorsPWM = doorsTgtPWM;
    for (int j = 0; j < doorsFinishPulses; j++) {
      digitalWrite(doorsServoPin, 1);
      delayMicroseconds(doorsPWM);
      digitalWrite(doorsServoPin, 0);
      delay(doorsStepTime);
    }
    
    // нормальное завершение попытки
    break;
  }  
  if (i == doorsTryLimit) {
    // превышено количество попыток, аварийный останов
    #ifdef DEBIG
    Serial.println();
    Serial.println("Doors shift limit exceeded! Emergency stop");
    #endif
    while(1);
  }
  
}  
 
