// Програма управления макетом лифта
// Версия Arduino >= 1.6.7

#include <MPR121.h>
#include <Wire.h>

// вывод сообщений в Serial
#define DEBUG
// отладка без оборудования, не проверять состояние концевика шахты и датчика дверей
// #define SENSORS_OFF

#define FLOORS 5                         // кол-во этажей
#define STEPS_PER_FLOOR 5300             // кол-во шагов ШД на этаж (5250)
#define STEPS_OFFSET 500                 // кол-во шагов для 1-го этажа от приямка (500)
const long motorFloorSteps[FLOORS + 1] =  // номера шагов ШД, соответствующие этажам
  {0, 
   STEPS_OFFSET,
   STEPS_OFFSET + STEPS_PER_FLOOR * 1,
   STEPS_OFFSET + STEPS_PER_FLOOR * 2,
   STEPS_OFFSET + STEPS_PER_FLOOR * 3,
   STEPS_OFFSET + STEPS_PER_FLOOR * 4};
const long motorStepsLimit = STEPS_PER_FLOOR * FLOORS + STEPS_OFFSET * 2; // ограничитель числа шагов при калибровке положения кабины
const int motorDirUp = 1;                // сигнал DIR драйвера ШД для подъема/спуска кабины
const int motorDirDown = 0;
const int motorEnable = 0;               // сигнал Enable ШД
const int motorDisable = 1;

const long motorVmin = 20;              // начальная скорость, имп./сек. 
// макс. скорость для N мм/сек, шкив Ф 42 мм, пер.число редуктора 43, режим 1/2 шага (имп./сек.)
const long motorVmax = (35 / (PI * 42)) * 96 * 2 * 43;  // макс 50 мм / сек
// максимальный размер области разгона/торможения, шагов
const int motorAccSteps = 2000;          // максимальный размер области разгона/торможения, шагов
const int motorSlowFactor = 2;           // коэффициент уменьшения скорости в режиме калибровки
const long motorAcc = (motorVmax * motorVmax - motorVmin * motorVmin) / (2 * motorAccSteps); // ускорение

const int motorEnablePin = 2;            // порты управления драйвера ШД
const int motorDirPin = 3;
const int motorStepPin = 4;
const int limitSensorPin = A7;           // порт концевого выключателя "подвала"
 
// порты для сервы дверей (управление и датчик)
const int doorsServoPin = A1;
const int doorsSensorPin = A2;

// перемещение дверей в положение
const int doorsOpen = 1;
const int doorsClose = 0;
const int doorsSemiOpen = 2;

const int doorsOpenPWM = 2400;         // ШИМ для серво дверей, мкс
const int doorsClosedPWM = 590;
const int doorsSemiOpenPWM = 1000;      // ШИМ приоткрытия дверей при инициализации
const int doorsShiftTime = 1800;       // время на перемещение дверей, мс
const int doorsStepTime = 20;          // период импульсов серво дверей, мс
const int doorsFinishPulses = 5;       // число импульсов для "довода" в конечное положение
int doorsStepPWM = (doorsOpenPWM - doorsClosedPWM) / (doorsShiftTime / doorsStepTime); // шаг ШИМ
const int doorsSensorThreshold = 980;  // порог значения АЦП датчика тока серво
const int doorsTryLimit = 10;          // количество попыток перемещения дверей
const int doorsTryDelay = 1000;        // задержка между попытками, мс
int doorsPWM;

// порты индикатора номера этажа
const int displayPin0 = 5;
const int displayPin1 = 6;
const int displayPin2 = 7;

// светодиод индикации состояния
const int statusLED = 13;

// уровень громкости звука
const int soundLevel = 17;
// порядковые номера файлов на SD-карте и длительность звуковых фрагментов
// (фрагменты для номеров этажей - от 1 до 5, длительность ~2 сек.)
// зв.фрагмент прибытия на этаж
const int soundArrive = 6;
const int soundArriveLen = 3;
// зв.фрагмент включения системы
const int soundOn = 7;
const int soundOnLen = 2;

// группа кнопок в кабине D8-D12 (Port B, PCINT0)
// соответствие пинов кнопкам
const int cageButtonPins[FLOORS + 1] = 
  {0, 12, 11, 10, 9, 8};
// соответствие номеров разрядов порта B кнопкам
const int cageButtonBits[FLOORS + 1] = 
  {0, 4, 3, 2, 1, 0};  
// маска разрядов порта B   
const byte buttonBitsMask = 0b11111;  

// группа кнопок на этажах - MPR121 (Port C:A3, PCINT1)  
// соответствие пинов MPR121 кнопкам
const int floorButtonPins[FLOORS + 1] = 
  {0, 0, 1, 2, 3, 4};
// соответствие пинов MPR121 индикации  
const int floorLEDPins[FLOORS + 1] = 
  {0, 5, 6, 7, 8, 9};
// маска разрядов пинов индикации  
const int floorLEDBitsMask = 0b111110;  

// адрес i2c MPR121
#define MPR121_ADDRESS 0x5A
// порт для IRQ MPR121
#define MPR121_IRQ_PIN A3
// пороги срабатывания MPR121
#define MPR121_TOUCH_THRESHOLD 40
#define MPR121_RELEASE_THRESHOLD 20  
// маска прерываний от MPR121
byte PCINT1BitsMask = 0b1000;

// состояния вызовов из кабины 
volatile int cageCallStatus[FLOORS + 1] = 
{0, 0, 0, 0, 0, 0};
// состояния вызовов на этажах
volatile int floorCallStatus[FLOORS + 1] = 
{0, 0, 0, 0, 0, 0};

// текущий этаж
int currentFloor;
// текущее положение ШД (в шагах)
long motorCurrentStep;
// этаж обрабатываемого вызова
int targetFloor = 0;
// направление движение кабины
int moveDirection = 0;
const int moveUp = 2;
const int moveDown = 1;


// обработчик прерываний от кнопок в кабине (D8-D12)
ISR(PCINT0_vect) {
  static uint8_t reg_prev = buttonBitsMask;
  uint8_t reg, diff;
  
  // выделяем изменившиеся с прошлого вызова пины
  reg = PINB & buttonBitsMask;
  diff = reg ^ reg_prev;
  
  // запоминаем состояние пинов для след.вызова
  reg_prev = reg;
  
  // определяем пины, изменившиеся в 0
  for (int i = 1; i <= FLOORS; i++) {
    
    if (diff & _BV(cageButtonBits[i]) && !(reg & _BV(cageButtonBits[i]))) {
      // пин изменился в 0    
      // переключаем пин на выход
      DDRB |= _BV(cageButtonBits[i]);
      // выводим в него 0 для фиксации индикации нажатой кнопки
      PORTB &= ~(_BV(cageButtonBits[i]));
      
      // запоминаем вызов
      cageCallStatus[i] = 1;
    }
  }  
}

// обработчик прерываний от контроллера кнопок на этажах (A3)
ISR(PCINT1_vect) {

  // разрешаем прерывания для нормальной работы i2c
  interrupts();
  
  // получаем состояние кнопок
  unsigned int buttons = (unsigned int)MPR121.getRegister(MPR121_TS1); // + ((unsigned int)MPR121.getRegister(MPR121_TS2)<<8);

  // запрещаем прерывания
  noInterrupts();
  
  // определяем "нажатые" кнопки
  for (int i = 1; i <= FLOORS; i++) {
    if (buttons & _BV(floorButtonPins[i])) {
      // запоминаем вызов
      floorCallStatus[i] = 1;
      // включаем светодиод индикации
      MPR121.digitalWrite(floorLEDPins[i], 1);
    }
  }
}

// инициализация
void setup() {

  Serial.begin(9600);
  #ifdef DEBUG
  Serial.println("System initialization");
  #endif
  
  // инициализация MPR121
  if(!MPR121.begin(MPR121_ADDRESS, MPR121_TOUCH_THRESHOLD, MPR121_RELEASE_THRESHOLD, MPR121_IRQ_PIN)){ 
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
  
  // частота i2c 400 кГц
  MPR121.goFast();

  // порты кнопок на этажах
  MPR121.setNumEnabledElectrodes(FLOORS);
  // порты индикации кнопок
  // инициализируем на вывод, открытый, HIGH SIDE
  MPR121.setRegister(MPR121_EN, floorLEDBitsMask);
  MPR121.setRegister(MPR121_DIR, floorLEDBitsMask);
  MPR121.setRegister(MPR121_CTL0, floorLEDBitsMask);
  MPR121.setRegister(MPR121_CTL1, floorLEDBitsMask);
  MPR121.setRegister(MPR121_DAT, 0);
  // мигаем подсветкой
  for (int i = 1; i <= FLOORS; i++) {
    MPR121.digitalWrite(floorLEDPins[i], 1);
    delay(200);
    MPR121.digitalWrite(floorLEDPins[i], 0);
  }
  
  // получаем начальное состояние кнопок
  MPR121.updateTouchData();
  
  // порты кнопок в кабине
  // каждый инициализируем на вывод, включаем подсветку, потом переключаем на ввод
  for (int i = 1; i <= FLOORS; i++) {
    pinMode(cageButtonPins[i], OUTPUT);
    digitalWrite(cageButtonPins[i], LOW);
    delay(200);
    pinMode(cageButtonPins[i], INPUT_PULLUP);
  }

  // порты ШД
  pinMode(motorEnablePin, OUTPUT);
  digitalWrite(motorEnablePin, motorEnable);
  pinMode(motorStepPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  
  // порт концевика
  pinMode(limitSensorPin, INPUT);

  // порты сервы дверей
  pinMode(doorsServoPin, OUTPUT);
  pinMode(doorsSensorPin, INPUT);
  
  // порты индикатора номера этажа
  pinMode(displayPin0, OUTPUT);
  pinMode(displayPin1, OUTPUT);
  pinMode(displayPin2, OUTPUT);
  
  // индикатор состояния
  pinMode(statusLED, OUTPUT);
  
  // устанавливаем уровень звука
  DFPlayer_send_cmd(0x06, 0, soundLevel);
  delay(100);
  // звуковой сигнал включения
  DFPlayer_send_cmd(0x03, 0, soundOn);
  delay(soundOnLen * 1000);

  noInterrupts();
  // настройка прерываний от кнопок 
  PCICR = 0b00000011;       // разрешаем прерывания PCINT0 и PCINT1
  PCMSK0 = buttonBitsMask;  // от пинов кнопок в кабине
  PCMSK1 = PCINT1BitsMask;  // от контроллера кнопок на этажах
  interrupts();

  // двери в приоткрытое положение
  doorsPWM = doorsClosedPWM;
  doorsShift(doorsSemiOpen, 0); 
  delay(1000);
  // двери в закрытое положение
  doorsShift(doorsClose, 1);
  delay(1000);
  
  // переделать алгоритм - сначала приподнимать из приямка до срабатывания концевика ???
  // калибровка положения кабины
  // опускаем до срабатывания концевика
  moveCage(0);
  delay(1000);

  // поднимаем на 1 этаж
  moveCage(1);

}
    
// основной цикл    
void loop() {
  /*
  delay(1000);
  Serial.print(currentFloor);
  Serial.print(" ");
  Serial.print(targetFloor);
  Serial.print(" ");
  Serial.println(moveDirection);
  for (int i = 0; i <= FLOORS; i++) {
    Serial.print(cageCallStatus[i]); 
    Serial.print(" ");
  }
  Serial.println();
  for (int i = 0; i <= FLOORS; i++) {
    Serial.print(floorCallStatus[i]); 
    Serial.print(" ");
  }
  Serial.println();  
  */
  
  // определение нового или следующего вызова
  // в порядке убывания приоритета
  if (cageCallStatus[currentFloor] || floorCallStatus[currentFloor])
    // вызов с текущего этажа
	  targetFloor = currentFloor;
  else 
    // с других этажей в зависимости от направления движения
    switch (moveDirection) {
	    case 0:	
	      // кабина неподвижна
        // вызовы из кабины и с этажей, начиная с верхнего этажа
	      for (int i = FLOORS; i >= 1; i--) {
	        if (cageCallStatus[i] || floorCallStatus[i]) {
		        // есть вызов	
		        targetFloor = i;  
		        break;
	        }
	      }
	    break;
	
      case moveUp: 
	      // кабина двигалась вверх
	      // только вызовы из кабины, начиная от текущего этажа вверх
	      for (int i = currentFloor; i <= FLOORS; i++) {
	        if (cageCallStatus[i]) {
		        // есть вызов		  	
		        targetFloor = i;  
		        break;
	        }
	      }
      break;
	  
	    case moveDown: 
	      // кабина двигалась вниз
	      // вызовы из кабины и с этажей, начиная от текущего этажа вниз
	      for (int i = currentFloor; i >= 1; i--) {
	        if (cageCallStatus[i] || floorCallStatus[i]) {
		        // есть вызов	
		        targetFloor = i;  
		        break;
	        }
	      }
	      break;
      }	  
	
  if (targetFloor == 0) {
    // вызовов нет
	  // сбрасываем направление движения
    moveDirection = 0;
	  // идем на следующую итерацию ожидания вызова
	  return;
  }

  #ifdef DEBUG
  Serial.print("Call from floor: ");
  Serial.print(targetFloor);
  Serial.print(", current floor: ");
  Serial.print(currentFloor);
  Serial.print(", direction was: ");
  if (moveDirection == 0) Serial.println("-");
  if (moveDirection == moveUp) Serial.println("Up");
  if (moveDirection == moveDown) Serial.println("Down"); 
  #endif
  
  // вызов есть
  if (targetFloor != currentFloor) {
    // вызов с другого этажа
    
	  // определяем направление движения
	  if (targetFloor > currentFloor)
	    moveDirection = moveUp;
    else
	    moveDirection = moveDown;
	
    // перемещаем кабину на этаж вызова
    moveCage(targetFloor);
    
    // звуковой сигнал прибытия и сообщение
    DFPlayer_send_cmd(0x03, 0, soundArrive);
    delay(soundArriveLen * 1000);
    DFPlayer_send_cmd(0x03, 0, targetFloor);
    delay(2000);
  }

  delay(2000);

  // выключаем индикацию кнопок этажа вызова
  buttonLightOff(targetFloor);
  
  // открываем двери
  doorsShift(doorsOpen, 1);
  
  // пауза 5 с
  delay(5000);
  
  // закрываем двери
  doorsShift(doorsClose, 1);
  
  delay(1000);
  
  // вызов обработан
  targetFloor = 0;  
}


// процедура перемещения кабины на указанный этаж
// этаж 0 - калибровка положения (останов кабины по концевику)
void moveCage(int tgtFloor) {
float V; 
long steps, accLastStep, decFirstStep;
int dir, stepIncrement;

  if (tgtFloor == currentFloor && currentFloor != 0)
    return;
 
  if (tgtFloor > 0) {
    // обычный режим
    
    // определяем направление и число шагов
    if (tgtFloor > currentFloor) {
      // движение вверх
      steps = motorFloorSteps[tgtFloor] - motorFloorSteps[currentFloor];
      dir = motorDirUp;
      stepIncrement = 1;
    }
    else {
      // движение вниз
      steps = motorFloorSteps[currentFloor] - motorFloorSteps[tgtFloor];
      dir = motorDirDown;
      stepIncrement = -1;
    }
    
    // определяем границы областей разгона и торможения - не больше 1/3 от всего перемещения
    accLastStep = steps / 3;
    if (accLastStep > motorAccSteps) accLastStep = motorAccSteps;
    decFirstStep = steps - accLastStep;
  
    // разгон от начальной скорости
    V = motorVmin;
  
    #ifdef DEBUG
    Serial.print("Moving cage to floor: ");
    Serial.print(tgtFloor); 
    Serial.print(", direction: ");
    Serial.print(dir);
    Serial.print(", steps: ");
    Serial.print(steps);
    Serial.print(", acceleration: ");
    Serial.print(motorAcc);
    Serial.print(", accel.steps: ");
    Serial.println(accLastStep);
    #endif
  
    // включаем мотор
    digitalWrite(motorDirPin, dir);
    setStatusLED(1);
  
    // цикл по шагам перемещения
    for (long i = 1; i <= steps; i++) {
      // засекаем время
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

      // двигаем счетчик шагов
      motorCurrentStep += stepIncrement;
      // проверяем прохождения очередного этажа
      for (long k = 1; k <= FLOORS; k++) {
        if (motorFloorSteps[k] == motorCurrentStep) {
          currentFloor = k; 
          displayFloorNumber(currentFloor);
          break;
        } 
      }  

      // задержка для формирования периода импульса
      delayMicroseconds(T - (micros() - t));

    }
    // выключаем индикатор
    setStatusLED(0);
  } 
  else {
    // режим калибровки - приподнимаем или опускаем кабину медленно до выключения/включения концевика
    
    // уменьшаем область разгона для ограничения скорости
    accLastStep = motorAccSteps / motorSlowFactor;
  
    // разгон от начальной скорости
    V = motorVmin;
  
    #ifdef DEBUG
    Serial.print("Moving cage to lowest position");
    Serial.print(", max.steps: ");
    Serial.print(motorStepsLimit);
    Serial.print(", acceleration: ");
    Serial.print(motorAcc);
    Serial.print(", accel.steps: ");
    Serial.println(accLastStep);
    #endif
    
    // текущее положение концевика
    int limitSensor = analogRead(limitSensorPin) == 0?0:1;
    long i = 0;

    // для отладки без оборудования считаем, что концевик сработал
    #ifdef SENSORS_OFF
    limitSensor = 0;
    #endif
    
    #ifdef DEBUG
    Serial.print("Tetminal switch state: ");
    Serial.println(limitSensor);
    #endif
    
    if (limitSensor == 1) {
      // кабина в не нижнем положении
      
      // направление - вниз
      digitalWrite(motorDirPin, motorDirDown);
      setStatusLED(1);
    
      // перемещаем кабину до срабатывания концевика или до ограничителя числа шагов
      for (i = 1; i <= motorStepsLimit; i++) {
        // засекам время
        unsigned long t = micros();

        if ((analogRead(limitSensorPin) == 0?0:1) != limitSensor)
          // переключился концевик 
          break;
        
        // формируем импульс  
        digitalWrite(motorStepPin, HIGH);
        delayMicroseconds(100);
        digitalWrite(motorStepPin, LOW);
    
        if (i <= accLastStep) {
          // находимся в зоне разгона
          V = sqrt(V * V + 2 * motorAcc);
        }
    
        // вычисляем период импульса
        unsigned long T = 1000000 / V;

        // задержка для формирования периода импульса
        delayMicroseconds(T - (micros() - t));
      }
      // выключаем индикатор
      setStatusLED(0);
        
      if (i >= motorStepsLimit) {
        // ошибка - вышли из цикла по ограничителю шагов
        #ifdef DEBUG
        Serial.println("Terminal switch malfunction! Emergency stop");
        #endif
        while(1);
      }  
    }
    
    #ifdef DEBUG
    Serial.print("Position reached on ");
    Serial.print(i);
    Serial.println(" step");
    #endif
    
    // выводим 0 этаж на индикатор
    displayFloorNumber(0); 
    
    // обнуляем номера текущего этажа и шага
    currentFloor = 0;
    motorCurrentStep = 0;
  }  
}  

// открывание/закрывание/приоткрывание дверей
void doorsShift(int dir, int sensorOnOff) {
  int doorsSrcPWM;
  int doorsTgtPWM;
  int doorsStepPWMCurr;
  int checkSensorCnt = 0;
  int blockage = 0;
  int i;

  // для отладки без оборудования датчик дверей не проверяем
  #ifdef SENSORS_OFF
  sensorOnOff = 0;
  #endif

  // определяем начальное и конечное положение и знак шага
  if (dir == doorsClose) {
    // закрывание
    doorsSrcPWM = doorsOpenPWM;
    doorsTgtPWM = doorsClosedPWM;
    doorsStepPWMCurr = -1 * doorsStepPWM;
  }
  else {
    if (dir == doorsOpen) {
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

  setStatusLED(1);

  // цикл попыток перемещения дверей
  for (i = 1; i < doorsTryLimit; i++) {
    #ifdef DEBUG
    Serial.print("Doors ");
    Serial.print((dir == doorsClose)?"close":"open");
    Serial.print(" trying: ");   
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
      if (checkSensorCnt > 0)
        checkSensorCnt--;
        
      // проверяем препятствие
      if (sensorOnOff > 0 && checkSensorCnt == 0 && sensor < doorsSensorThreshold) {
        // есть препятствие
        // для движения в обратную сторону меняем местами начальное и конечное положение и знак шага
        int tmp = doorsTgtPWM; doorsTgtPWM = doorsSrcPWM; doorsSrcPWM = tmp;
        doorsStepPWMCurr *= -1;
        // откатываем текущее положение на 5 шагов 
        doorsPWM += 5 * doorsStepPWMCurr;
        // устанавливаем признак препятствия
        blockage = 1;
        // блокируем проверку препятствия на несколько последующих шагов
        checkSensorCnt = 10;
        #ifdef DEBUG
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
      
  setStatusLED(0);
}

// процедура отправки команды в DFPLayer MP3
void DFPlayer_send_cmd(byte cmd, byte prm1, byte prm2) 
{
  // вычисляем контрольную сумму (2 байта)
  int16_t checksum = -(0xFF + 0x06 + cmd + 0x00 + prm1 + prm2);

  // собираем строку команды
  byte cmd_line[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, prm1, prm2, checksum >> 8, (byte) checksum & 0xFF, 0xEF};

  #ifdef DEBUG
  Serial.print("Sending command to sound module: ");
  Serial.print(cmd);
  Serial.print(", ");
  Serial.print(prm1);
  Serial.print(", ");
  Serial.println(prm2);
  #endif

  // отправляем в модуль
  for (byte i=0; i<10; i++)
    Serial.write(cmd_line[i]);
    
  #ifdef DEBUG
  Serial.println();
  #endif  
}

// процедура индикации номера этажа
void displayFloorNumber(int num) {
  digitalWrite(displayPin0, num & 1);
  digitalWrite(displayPin1, (num >> 1) & 1);
  digitalWrite(displayPin2, (num >> 2) & 1);
}

// процедура сброса статуса и выключения индикации кнопок вызова в кабине и на этаже 
void buttonLightOff(int flr) {
    cageCallStatus[flr] = 0;
    floorCallStatus[flr] = 0;
    pinMode(cageButtonPins[flr], INPUT_PULLUP);
    MPR121.digitalWrite(floorLEDPins[flr], 0);
}

// процедура управления индикатором состояния (LED13)
void setStatusLED(int s) {
  digitalWrite(statusLED, s);
}  
