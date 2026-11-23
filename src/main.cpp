#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// ВЕДОМЫЙ: принимает данные по I2C и управляет двумя сервоприводами и светодиодом
// Сервопривод №1 (ось X) -> D9, сервопривод №2 (ось Y) -> D10, светодиод -> D4
// Шина: SDA = A4, SCL = A5
//
// Пакет от мастера: 5 байт -> xH, xL, yH, yL, btn (x и y: 0..1023, big-endian,
// btn: 0/1). Ведомый никогда не «тянет» шину: I2C-транзакцию завершает мастер,
// сервоприводы обновляются из loop() по последним принятым значениям.
//
// Сборка раздвоена по env: этот файл собирается в [env:servo] (build_src_filter
// включает только main.cpp), а src/joystick.cpp — в [env:master]. Оба файла
// определяют setup()/loop() (Arduino.h вызывает их), поэтому в одну прошивку
// они попадать не должны — это разведено фильтром источников в platformio.ini.

const uint8_t MY_ADDR     = 8;        // тот же адрес, что у мастера
const uint8_t PIN_SERVO_X = 9;        // SERVO_1, ось X
const uint8_t PIN_SERVO_Y = 10;       // SERVO_2, ось Y
const uint8_t PIN_LED     = 4;        // светодиод
const uint8_t MSG_SIZE    = 5;        // длина пакета в байтах

Servo servoX;                         // крутится от оси X
Servo servoY;                         // крутится от оси Y

// ПРАВИЛО: переменные, которые пишет ISR (onReceive) и читает loop(),
// должны быть volatile — иначе компилятор закеширует их в регистрах и сервы
// «залипнут» на стартовых 90 градусах.
volatile int     g_x   = 512;         // 0..1023, старт — середина
volatile int     g_y   = 512;
volatile uint8_t g_btn = 0;           // 0/1

// Колбэк I2C-приёмника (вызывается из ISR-контекста).
void onReceive(int count) {
  // count == MSG_SIZE — ожидаемый пакет. Иначе сливаем буфер целиком,
  // чтобы следующий пакет начинался с чистого состояния.
  if (count != MSG_SIZE) {
    while (Wire.available() > 0) (void)Wire.read();
    return;
  }

  const uint8_t xh  = (uint8_t)Wire.read();
  const uint8_t xl  = (uint8_t)Wire.read();
  const uint8_t yh  = (uint8_t)Wire.read();
  const uint8_t yl  = (uint8_t)Wire.read();
  const uint8_t btn = (uint8_t)Wire.read();

  // AVR — 8-битная платформа: запись int занимает несколько инструкций,
  // а loop() может быть прервана между ними → «рваный» кадр.
  // Публикуем пару (x, y) атомарно: на время записи выключаем прерывания.
  const uint8_t oldSREG = SREG;
  cli();
  g_x   = (int)(((uint16_t)xh << 8) | xl);
  g_y   = (int)(((uint16_t)yh << 8) | yl);
  g_btn = btn ? 1 : 0;
  SREG = oldSREG;
}

void setup() {
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  pinMode(PIN_LED, OUTPUT);

  Wire.begin(MY_ADDR);
  Wire.onReceive(onReceive);          // колбэк при приходе пакета

  // Сразу встаём в центр, до первого пакета от мастера.
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  // Читаем «снимок» опубликованных значений один раз (атомарно).
  const uint8_t oldSREG = SREG;
  cli();
  const int     xCopy   = g_x;
  const int     yCopy   = g_y;
  const uint8_t btnCopy = g_btn;
  SREG = oldSREG;

  // x, y: 0..1023 → 0..180 градусов.
  const int angleX = map(constrain(xCopy, 0, 1023), 0, 1023, 0, 180);
  const int angleY = map(constrain(yCopy, 0, 1023), 0, 1023, 0, 180);

  servoX.write(angleX);
  servoY.write(angleY);

  // Кнопка зажигает светодиод.
  digitalWrite(PIN_LED, btnCopy ? HIGH : LOW);

  delay(15);                          // ~66 Гц
}

