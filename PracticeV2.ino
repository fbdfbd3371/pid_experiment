// =======================================================
//  ESP8266 — Dual ESC Balance (Full PID, Potentiometer A0)
//  Relay: D0 (GPIO16), ACTIVE LOW (LOW=ON, HIGH=OFF)
//  ESC1 : D1 (GPIO5)
//  ESC2 : D2 (GPIO4)
//  Pot  : A0 (центр 660, диапазон 540..810)
//  Логика: Countdown -> Калибровка обоих (MAX->MIN) -> Баланс PID
// =======================================================

#include <Arduino.h>
#include <Servo.h>

// -------------------------------
// Пины
// -------------------------------
const uint8_t PinRelay = D0;
const uint8_t PinEsc1  = D2;
const uint8_t PinEsc2  = D1;
const uint8_t PinPot   = A0;

// -------------------------------
// Диапазоны импульсов ESC (по отлаженному кейсу)
// -------------------------------
const int CalMinUs = 800;     // калибровочный минимум
const int CalMaxUs = 2300;    // калибровочный максимум

// Рабочий диапазон и базовые значения (безопасные для стенда)
const int PulseMinUs   = 1100;   // рабочий MIN (холостой)
const int BaseUs       = 1250;   // базовая тяга вокруг которой рулить
const int PulseMaxUs   = 1500;   // верхняя граница (держим низко, чтобы спокойно отрабатывать)
const int DeltaMaxUs   = 120;    // |Δ| ограничитель дифференциала тяги

// Ограничение скорости изменения (анти-рывки)
const int SlewStepUs   = 4;      // макс изменение за один шаг цикла управления

// -------------------------------
// Потенциометр и цель
// -------------------------------
const int PotCenter    = 660;
const int PotMinValid  = 540;
const int PotMaxValid  = 810;

// Если коррекция «в перевёрнутую» сторону — поставь true
bool InvertDirection   = false;

// Фильтр потенциометра (экспоненциальное сглаживание)
float PotAlpha         = 0.20f;   // 0..1; больше — быстрее реагирует
int   PotFiltered      = PotCenter;

// -------------------------------
// ПИД (единицы — микросекунды ШИМ на "дельту")
// Kp:  us / count
// Ki:  us / (count * sec)    (накапливаем через dt)
// Kd:  us / (count / sec)    (D берём по измерению: -Kd * dY/dt)
// -------------------------------
float KpUsPerCount     = 0.25f;   // мягкий старт
float KiUsPerCountSec  = 0.1f;   // очень небольшой интеграл
float KdUsPerCountPerSec = 0.1f; // мягкая производная

// Анти-виндап интегратора (ограничиваем вклад I в микросекундах)
const int IntegralMaxUs = 80;     // вклад I-терма ограничен, чтобы не «зависал» двигатель

// Фильтр D-терма (на производной измерения)
float DAlpha           = 0.20f;   // 0..1; больше — меньше сглаживания (быстрее, но шумнее)

// Тайминги
const unsigned long LoopDtMs       = 10;     // период управления ~100 Гц
const unsigned long PrintEveryMs   = 100;    // период логирования
const unsigned long CalMaxHoldMs   = 5000;   // держим MAX при калибровке
const unsigned long CalMinHoldMs   = 6000;   // держим MIN при калибровке
const unsigned long CooldownMs     = 3000;

Servo Esc1, Esc2;
int   CmdUs1 = CalMinUs;
int   CmdUs2 = CalMinUs;
bool  RelayPowered = false;

// Состояние ПИД
float IntUs = 0.0f;          // вклад интегратора в микросекундах
float DerivFilt = 0.0f;      // отфильтрованная производная измерения (counts/sec)
int   LastPotFilt = PotCenter;
unsigned long LastTsMs = 0;

#define LOGF(...) do { Serial.printf("[%10lu ms] ", millis()); Serial.printf(__VA_ARGS__); } while(0)

// -------------------------------
// Утилиты
// -------------------------------
inline void RelayOn()  { digitalWrite(PinRelay, LOW);  RelayPowered = true;  LOGF("Relay: ON  (ESC power applied)\n"); }
inline void RelayOff() { digitalWrite(PinRelay, HIGH); RelayPowered = false; LOGF("Relay: OFF (ESC power removed)\n"); }

int ClampInt(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// Плавный выход обоих моторов к целевому значению (анти-рывки)
void SlewBothTo(int target1, int target2) {
  target1 = ClampInt(target1, PulseMinUs, PulseMaxUs);
  target2 = ClampInt(target2, PulseMinUs, PulseMaxUs);
  bool done = false;
  while (!done) {
    done = true;
    if (CmdUs1 != target1) {
      int step = ClampInt(target1 - CmdUs1, -SlewStepUs, SlewStepUs);
      CmdUs1 += step;
      done = false;
    }
    if (CmdUs2 != target2) {
      int step = ClampInt(target2 - CmdUs2, -SlewStepUs, SlewStepUs);
      CmdUs2 += step;
      done = false;
    }
    Esc1.writeMicroseconds(CmdUs1);
    Esc2.writeMicroseconds(CmdUs2);
    delay(LoopDtMs);
  }
}

// Ракетный обратный отсчёт
void RocketCountdown(int from = 10) {
  Serial.println("\n=== SYSTEM START COUNTDOWN ===");
  for (int i = from; i >= 1; --i) { Serial.printf("T-%d...\n", i); delay(1000); }
  Serial.println(">>> IGNITION SEQUENCE START <<<");
}

// Калибровка обоих ESC: attach -> MAX -> реле ON -> hold -> MIN -> hold
void CalibrateBothMaxMin() {
  Serial.println("\n=== CALIBRATION (BOTH): MAX -> MIN ===");

  // Привязываем ESC заранее и ставим MAX
  Esc1.attach(PinEsc1, CalMinUs, CalMaxUs);
  Esc2.attach(PinEsc2, CalMinUs, CalMaxUs);
  LOGF("ESC1/ESC2 attached [%d..%d] us\n", CalMinUs, CalMaxUs);

  Esc1.writeMicroseconds(CalMaxUs);
  Esc2.writeMicroseconds(CalMaxUs);
  CmdUs1 = CmdUs2 = CalMaxUs;

  // Включаем питание и держим MAX
  RelayOn();
  LOGF("CAL: set MAX=%d us (hold %lums)\n", CalMaxUs, CalMaxHoldMs);
  delay(CalMaxHoldMs);

  // MIN и держим
  Esc1.writeMicroseconds(CalMinUs);
  Esc2.writeMicroseconds(CalMinUs);
  CmdUs1 = CmdUs2 = CalMinUs;
  LOGF("CAL: set MIN=%d us (hold %lums)\n", CalMinUs, CalMinHoldMs);
  delay(CalMinHoldMs);

  LOGF("CAL: complete\n");
}

// После калибровки — мягко выходим на базовую тягу
void SlewToBase() {
  LOGF("Slew to BaseUs=%d\n", BaseUs);
  SlewBothTo(BaseUs, BaseUs);
  LOGF("At Base\n");
}

// Один шаг ПИД-регулятора
void ControlStepPID() {
  // Время
  unsigned long now = millis();
  float dt = (LastTsMs == 0) ? (LoopDtMs * 0.001f) : ((now - LastTsMs) * 0.001f);
  if (dt < 0.001f) dt = 0.001f;  // страховка от деления на 0
  LastTsMs = now;

  // Считываем и фильтруем потенциометр
  int raw = analogRead(PinPot);
  PotFiltered = (int)roundf(PotFiltered + PotAlpha * (raw - PotFiltered));

  // Ошибка (цель - измерение)
  int error = PotCenter - PotFiltered;  // e>0: нужно добавить тягу M1 и/или снять M2 (если ориентация правильная)
  if (InvertDirection) error = -error;

  // --- P-терм (us) ---
  float pUs = KpUsPerCount * (float)error;

  // --- D-терм (по измерению): dY/dt в counts/sec, затем uD = -Kd * dY/dt ---
  float dCounts = (float)(PotFiltered - LastPotFilt);
  float dMeasPerSec = dCounts / dt;                    // counts/sec
  DerivFilt = (1.0f - DAlpha) * DerivFilt + DAlpha * dMeasPerSec;
  float dUs = -KdUsPerCountPerSec * DerivFilt;         // us

  // --- I-терм (анти-виндап) ---
  // условная интеграция: интегрируем только если не в жёсткой сатурации по |Δ| (см. ниже)
  // сначала рассчитаем "предварительный" u без нового I, проверим сатурацию — и только потом добавим I.
  float uNoI = pUs + dUs;

  // Интегрируем, если "почти не насыщены" (зазор 5 us) или интеграл «помогает выйти из насыщения»
  bool allowIntegrate = (fabs(uNoI) < (DeltaMaxUs - 5));
  if (allowIntegrate) {
    IntUs += (KiUsPerCountSec * (float)error) * dt;    // us
    // ограничение интегратора
    if (IntUs >  IntegralMaxUs) IntUs =  IntegralMaxUs;
    if (IntUs < -IntegralMaxUs) IntUs = -IntegralMaxUs;
  }

  float u = uNoI + IntUs;

  // Ограничение выхода по |Δ|
  int deltaUs = (int)roundf(u);
  deltaUs = ClampInt(deltaUs, -DeltaMaxUs, +DeltaMaxUs);

  // Цели для моторов
  int target1 = ClampInt(BaseUs + deltaUs, PulseMinUs, PulseMaxUs);
  int target2 = ClampInt(BaseUs - deltaUs, PulseMinUs, PulseMaxUs);

  // Ограничение скорости изменения (slew)
  int step1 = ClampInt(target1 - CmdUs1, -SlewStepUs, SlewStepUs);
  int step2 = ClampInt(target2 - CmdUs2, -SlewStepUs, SlewStepUs);
  CmdUs1 += step1;
  CmdUs2 += step2;

  // Выводим команды
  Esc1.writeMicroseconds(CmdUs1);
  Esc2.writeMicroseconds(CmdUs2);

  // Обновляем "прошлое" для производной
  LastPotFilt = PotFiltered;
}

// Печать состояния не чаще PrintEveryMs
void PrintState() {
  static unsigned long last = 0;
  if (millis() - last < PrintEveryMs) return;
  last = millis();
  int raw = analogRead(PinPot);
  int err = (InvertDirection ? -(PotCenter - PotFiltered) : (PotCenter - PotFiltered));
  Serial.printf("ADC=%4d Filt=%4d Err=%4d  Kp=%.2f Ki=%.3f Kd=%.2f  IntUs=%.1f  dFilt=%.1f  Cmd(M1/M2)=%d/%d Base=%d\n",
                raw, PotFiltered, err, KpUsPerCount, KiUsPerCountSec, KdUsPerCountPerSec,
                IntUs, DerivFilt, CmdUs1, CmdUs2, BaseUs);
}

// -------------------------------
// SETUP / LOOP
// -------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== DUAL ESC BALANCE (Full PID) ===");

  pinMode(PinRelay, OUTPUT);
  RelayOff(); // старт без питания

  RocketCountdown(10);

  // Калибровка ESC (MAX -> MIN), как у тебя сработало
  CalibrateBothMaxMin();

  // Мягкий выход на базу
  SlewToBase();

  // Инициализация фильтров
  PotFiltered = PotCenter;
  LastPotFilt = PotCenter;
  DerivFilt = 0.0f;
  IntUs = 0.0f;
  LastTsMs = millis();

  LOGF("Balance start. Setpoint=%d, BaseUs=%d, Kp=%.2f Ki=%.3f Kd=%.2f, Invert=%d\n",
       PotCenter, BaseUs, KpUsPerCount, KiUsPerCountSec, KdUsPerCountPerSec, (int)InvertDirection);
}

void loop() {
  // Быстрая проверка датчика: вне допустимого — уйти на безопасный минимум
  int raw = analogRead(PinPot);
  if (raw < 300 || raw > 900) {
    Esc1.writeMicroseconds(PulseMinUs);
    Esc2.writeMicroseconds(PulseMinUs);
    LOGF("WARN: Pot out of range (raw=%d). Safe MIN applied.\n", raw);
    delay(200);
    return;
  }

  ControlStepPID();
  PrintState();
  delay(LoopDtMs);
}
