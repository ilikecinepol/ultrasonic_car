#include <math.h>
#include <Servo.h>

const int numSensors = 5;                                   // Количество датчиков
const int sensorPins[numSensors] = { A0, A1, A2, A3, A4 };  // Пины датчиков
const int ledPinActive = 11;                                // Светодиод при активном подавлении
const int ledPinInactive = 10;                              // Светодиод, когда подавление не активно
const int emitterPin = 9;                                   // Пин для излучателя
const int servoPin = 8;                                     // Пин для подключения серводвигателя
const int smoothingFactor = 10;                             // Фактор сглаживания
const int numSamples = 50;                                  // Количество выборок для фильтрации
const unsigned long measurementDuration = 500;              // Время для накопления данных в миллисекундах

// Углы поворота для каждого датчика (в отраженном порядке)
const int servoPositions[numSensors] = { 180, 150, 90, 30, 0 };

// Для таймера подавления
unsigned long suppressionStartTime = 0;
const unsigned long suppressionDuration = 3000;  // 3 секунды
bool suppressionActive = false;
bool suppressionTriggered = false;  // Флаг, чтобы не перезапускать подавление

// Массивы для хранения данных по каждому датчику
int signalSamples[numSensors][numSamples];        // Сигнальные выборки для каждого датчика
int filteredValue[numSensors] = { 0 };            // Фильтрованные значения для каждого датчика
int dynamicThreshold[numSensors] = { 0 };         // Динамический порог для каждого датчика
int previousValue[numSensors] = { 0 };            // Предыдущее значение для каждого датчика
unsigned long crossingCount[numSensors] = { 0 };  // Количество пересечений динамического нуля
float receivedFrequency[numSensors] = { 0 };      // Частоты сигналов на каждом из датчиков
int sampleIndex[numSensors] = { 0 };              // Индекс выборки для каждого датчика
unsigned long startTime = 0;                      // Время начала измерений
int activeSensor = -1;                            // Переменная для хранения датчика с частотой > 20 кГц

Servo myServo;  // Создаем объект для управления серводвигателем

// Функция подавления
void suppression(bool state) {
  if (state) {
    digitalWrite(ledPinActive, HIGH);   // Включаем светодиод на 11 пине (активное подавление)
    digitalWrite(ledPinInactive, LOW);  // Выключаем светодиод на 10 пине (когда активен режим подавления)
    tone(emitterPin, 40000);            // Включаем подавление
    suppressionStartTime = millis();    // Запоминаем время начала подавления
    suppressionActive = true;
    suppressionTriggered = true;  // Подавление было запущено
  } else {
    digitalWrite(ledPinActive, LOW);     // Выключаем светодиод на 11 пине
    digitalWrite(ledPinInactive, HIGH);  // Включаем светодиод на 10 пине (когда подавление отключено)
    noTone(emitterPin);                  // Отключаем подавление
    suppressionActive = false;
  }
}

// Функция для поворота серводвигателя в зависимости от датчика
void moveServoToSensor(int sensor) {
  if (sensor >= 0 && sensor < numSensors) {
    int targetPosition = servoPositions[sensor];
    myServo.write(targetPosition);  // Поворачиваем серводвигатель на нужный угол
    Serial.print("Сервомотор повернут на ");
    Serial.print(targetPosition);
    Serial.println(" градусов.");
  }
}

// Функция для сбора данных с каждого датчика
void detectSignals() {
  for (int i = 0; i < numSensors; i++) {
    int signalValue = analogRead(sensorPins[i]);

    // Программный фильтр для сглаживания сигнала (скользящее среднее)
    filteredValue[i] = (filteredValue[i] * (smoothingFactor - 1) + signalValue) / smoothingFactor;

    // Сохраняем значения в массив сигналов для каждого датчика
    signalSamples[i][sampleIndex[i]] = filteredValue[i];
  }
}

// Функция для расчета частоты по количеству пересечений динамического нуля
void calculateFrequencies() {
  for (int i = 0; i < numSensors; i++) {
    // Рассчитываем динамический порог (среднее значение выборок)
    long sumSamples = 0;
    for (int j = 0; j < numSamples; j++) {
      sumSamples += signalSamples[i][j];
    }
    dynamicThreshold[i] = sumSamples / numSamples;

    // Проверка пересечения с динамическим нулём
    if ((previousValue[i] < dynamicThreshold[i] && filteredValue[i] >= dynamicThreshold[i]) || (previousValue[i] > dynamicThreshold[i] && filteredValue[i] <= dynamicThreshold[i])) {
      // Увеличиваем счётчик пересечений
      crossingCount[i]++;
    }

    // Обновляем предыдущее значение для каждого датчика
    previousValue[i] = filteredValue[i];
  }
}

// Функция для вычисления частот на основе количества пересечений
void calculateFinalFrequencies() {
  for (int i = 0; i < numSensors; i++) {
    receivedFrequency[i] = (crossingCount[i] / (measurementDuration / 1000.0)) / 2.0;  // Частота в Гц (делим на 2, т.к. каждое пересечение представляет собой полупериод)

    // Если частота на датчике превышает 20 кГц, запоминаем номер датчика
    if (receivedFrequency[i] > 20.0 && !suppressionTriggered) {  // Проверяем, чтобы подавление не было активным
      activeSensor = i;                                          // Запоминаем номер датчика
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ledPinActive, OUTPUT);    // Активный светодиод
  pinMode(ledPinInactive, OUTPUT);  // Неактивный светодиод
  pinMode(emitterPin, OUTPUT);

  myServo.attach(servoPin);  // Подключаем сервомотор к нужному пину
  myServo.write(90);         // По умолчанию серводвигатель в позиции 90 градусов (датчик 2)

  // Инициализация массивов
  for (int i = 0; i < numSensors; i++) {
    for (int j = 0; j < numSamples; j++) {
      signalSamples[i][j] = 0;
    }
  }

  startTime = millis();                // Запоминаем время старта
  digitalWrite(ledPinInactive, HIGH);  // Включаем светодиод 10, т.к. подавление не активно
}

void loop() {
  // Опрос всех датчиков
  detectSignals();

  // Расчет частоты для всех датчиков
  calculateFrequencies();

  // Проверяем, прошло ли необходимое время для накопления данных
  if (millis() - startTime >= measurementDuration) {
    // Вычисляем частоты по количеству пересечений
    calculateFinalFrequencies();

    // Вывод частот для всех датчиков
    for (int i = 0; i < numSensors; i++) {
      Serial.print("Частота на датчике ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(receivedFrequency[i]);
      Serial.println(" Гц");
    }

    // Если частота на каком-либо датчике больше 20 кГц и подавление не было запущено
    if (activeSensor != -1 && !suppressionTriggered && !suppressionActive) {
      Serial.print("Частота выше 20 кГц на датчике: ");
      Serial.println(activeSensor);
      suppression(true);                    // Включаем подавление
      moveServoToSensor(activeSensor);      // Поворачиваем серводвигатель к датчику
      receivedFrequency[activeSensor] = 0;  // Сбрасываем частоту для активного датчика
    }

    // Сбрасываем счётчики для следующего периода измерения
    for (int i = 0; i < numSensors; i++) {
      crossingCount[i] = 0;
    }

    // Обновляем время начала нового периода измерений
    startTime = millis();
  }

  // Если подавление активно, проверяем, прошло ли 3 секунды
  if (suppressionActive && (millis() - suppressionStartTime >= suppressionDuration)) {
    suppression(false);            // Выключаем подавление через 3 секунды
    suppressionTriggered = false;  // Сбрасы
    suppressionTriggered = false;  // Сбрасываем флаг, чтобы подавление могло быть запущено снова
    activeSensor = -1;             // Сбрасываем активный датчик
  }

  // Обновляем индекс выборки для всех датчиков
  for (int i = 0; i < numSensors; i++) {
    sampleIndex[i] = (sampleIndex[i] + 1) % numSamples;
  }

  delay(1);  // Минимальная задержка для плавной работы
}
