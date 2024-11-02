const int emitterPin = 9;  // Пин для излучателя
const int ledPin = 13;

void setup() {
  pinMode(emitterPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  tone(emitterPin, 40000);  // Генерация сигнала на 40 кГц
  digitalWrite(ledPin, HIGH);
//   delay(100);  // Подача сигнала в течение 100 мс
//   noTone(emitterPin);  // Отключение сигнала
//   delay(500);  // Ждём перед следующим сигналом
// 
}
