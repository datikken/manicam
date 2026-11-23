#include <Arduino.h>
#include <Wire.h>

// МАСТЕР: читает джойстик и кнопку, шлёт пакет ведомому по I2C, который
// управляет двумя сервоприводами и светодиодом.
//
// Джойстик:  VRx -> A0, VRy -> A1, SW (кнопка) -> D2 (INPUT_PULLUP)
// Шина:      SDA = A4, SCL = A5
// Светодиод: D13 — индикатор состояния связи (на плате Nano он уже есть)
//
// Пакет ведомому: 5 байт -> xH, xL, yH, yL, btn
//   x, y  — 0..1023, big-endian (как в main.cpp ведомого)
//   btn   — 0/1, 1 = кнопка нажата
//
// ВНИМАНИЕ: контракт по байтам должен совпадать с main.cpp (MSG_SIZE = 5,
// порядок полей xH,xL,yH,yl,btn). Если меняете формат — меняйте в обоих файлах.

// Адрес ведомого на шине. Должен совпадать с MY_ADDR в main.cpp.
const uint8_t SLAVE_ADDR = 8;

const uint8_t PIN_JOY_X  = A0;        // ось X
const uint8_t PIN_JOY_Y  = A1;        // ось Y
const uint8_t PIN_BTN    = 2;         // кнопка джойстика, к земле -> PULLUP
const uint8_t PIN_LINK   = 13;        // индикатор связи на плате

const uint8_t  MSG_SIZE       = 5;    // длина пакета в байтах
const uint16_t SEND_PERIOD_MS = 15;   // ~66 Гц, как и у ведомого
const uint16_t DEADBAND       = 12;   // мёртвая зона вокруг центра (0..1023)

// Дребезг кнопки: простой антидребезг по времени, без библиотек.
const uint16_t DEBOUNCE_MS = 20;

static uint16_t g_last_x   = 0;       // последнее отправленное значение X
static uint16_t g_last_y   = 0;
static uint8_t  g_last_btn = 0;
static bool     g_has_sent = false;   // первый пакет шлём всегда, без мёртвой зоны

static bool     g_btn_stable  = false;
static bool     g_btn_raw     = false;
static uint32_t g_btn_changed = 0;

// Инициализация I2C-мастера и пинов. Вызывается из setup().
void joystickSetup() {
  pinMode(PIN_BTN, INPUT_PULLUP);     // кнопка замыкает на GND -> активный низкий
  pinMode(PIN_LINK, OUTPUT);
  digitalWrite(PIN_LINK, LOW);

  Wire.begin();                      // мастер: адрес не указываем
  Wire.setClock(100000);             // 100 кГц — стандартный режим, надёжен на макетке
}

// Пересчитывает «стабильное» состояние кнопки с учётом дребезга.
static bool readButtonDebounced(uint32_t now) {
  const bool raw = (digitalRead(PIN_BTN) == LOW);   // инвертировано: LOW = нажата

  if (raw != g_btn_raw) {
    g_btn_raw     = raw;
    g_btn_changed = now;
    return g_btn_stable;
  }

  if ((now - g_btn_changed) >= DEBOUNCE_MS && raw != g_btn_stable) {
    g_btn_stable = raw;
  }
  return g_btn_stable;
}

// Подавляет шум джойстика у центра.
static uint16_t applyDeadband(uint16_t value) {
  const int16_t center = 512;
  const int16_t delta  = (int16_t)value - center;
  // DEADBAND — uint16_t, -DEADBAND повышается до int (знаковый) -> -Wsign-compare.
  // Сравниваем в знаковом типе одинаковой ширины.
  const int16_t band = (int16_t)DEADBAND;
  if (delta > (int16_t)(-band) && delta < band) {
    return (uint16_t)center;
  }
  return value;
}

// Основная работа мастера: читает входы и шлёт пакет. Вызывается из loop().
void joystickLoop() {
  static uint32_t last_send = 0;
  const uint32_t  now       = millis();

  if (now - last_send < SEND_PERIOD_MS) {
    return;
  }
  last_send = now;

  // Аналоговые оси: 0..1023 на AVR.
  const uint16_t raw_x = (uint16_t)analogRead(PIN_JOY_X);
  const uint16_t raw_y = (uint16_t)analogRead(PIN_JOY_Y);
  const uint16_t x     = applyDeadband(raw_x);
  const uint16_t y     = applyDeadband(raw_y);
  const uint8_t  btn   = readButtonDebounced(now) ? 1 : 0;

  // Пропускаем пакет, если ничего не изменилось — впустую не дёргаем шину.
  // Первый пакет после старта отправляем всегда, чтобы ведомый «проснулся».
  if (g_has_sent && x == g_last_x && y == g_last_y && btn == g_last_btn) {
    return;
  }

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((uint8_t)((x >> 8) & 0xFF));   // xH
  Wire.write((uint8_t)(x & 0xFF));          // xL
  Wire.write((uint8_t)((y >> 8) & 0xFF));   // yH
  Wire.write((uint8_t)(y & 0xFF));          // yL
  Wire.write(btn);                          // btn
  const uint8_t result = Wire.endTransmission();

  if (result == 0) {
    // Байт передан и подтверждён ведомым — связь есть.
    g_last_x   = x;
    g_last_y   = y;
    g_last_btn = btn;
    g_has_sent = true;
    digitalWrite(PIN_LINK, HIGH);
  } else {
    // Ведомый не ответил: не обновляем «последние» значения, чтобы
    // при восстановлении связи переслать актуальный кадр.
    g_has_sent = false;
    digitalWrite(PIN_LINK, LOW);
  }
}

// Точки входа Arduino. Их вызывает main() из framework-arduino-avr,
// поэтому в файле обязательно должны быть именно setup() и loop().
void setup() {
  joystickSetup();
}

void loop() {
  joystickLoop();
}
