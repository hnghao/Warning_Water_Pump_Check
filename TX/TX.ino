/*
  TRANSMISSION.ino (TX)
  - Arduino Nano + HC-12
  - Sensor: ON = HIGH, OFF = LOW  (active-HIGH)
  - Gửi frame: AA 55 SEQ MASK CRC8
  - LED hiển thị tại TX: A1..A5
  - BUZZER ở D12:
      + Sensor5 ON -> sau 2s beep 2 lần -> tắt (1 lần)
      + Khi Sensor4&5 cùng ON -> mới kích lại y như trên (1 lần)
*/

#include <Arduino.h>
#include <SoftwareSerial.h>

// HC-12 TXD -> D11 (RX), HC-12 RXD <- D10 (TX)
SoftwareSerial hc12(11, 10); // RX, TX

// =======================
// Cấu hình
// =======================
#define HC12_BAUD      2400
#define TX_PERIOD_MS   20
#define DEBOUNCE_MS    15
#define REPEAT_SEND    4  // thời gian xác nhận báo còi sau khi nước đầy

// ====== Sensor ON là HIGH ======
#define SENSOR_ON_LEVEL_LOW  1   // 0 = ON khi HIGH, 1 = ON khi LOW

// LED wiring:
// - LED_ACTIVE_LOW = 0: Pin HIGH sáng (Pin -> R -> LED -> GND)
// - LED_ACTIVE_LOW = 1: Pin LOW sáng (+5V -> R -> LED -> Pin)
#define LED_ACTIVE_LOW 0

// =======================
// BUZZER (D12)
// =======================
#define BUZZER_PIN         12
#define BUZZER_DELAY_MS    2000
#define BUZZER_FREQ_HZ     2000
#define BUZZER_ON_MS       150
#define BUZZER_OFF_MS      150
#define BUZZER_BEEP_COUNT  10 // số lần còi kêu

// 5 chân cảm biến
const uint8_t S1 = 2;
const uint8_t S2 = 3;
const uint8_t S3 = 4;
const uint8_t S4 = 5;
const uint8_t S5 = 6;

// 5 LED: A1..A5
const uint8_t L1 = A1;
const uint8_t L2 = A2;
const uint8_t L3 = A3;
const uint8_t L4 = A4;
const uint8_t L5 = A5;

// =======================
// Struct debounce
// =======================
struct DebouncedInput {
  uint8_t  pin;
  bool     stableOn;
  bool     lastRawOn;
  uint32_t lastChangeMs;
};

// =======================
// CRC8 poly 0x07 cho 2 byte (SEQ, MASK)
// =======================
static uint8_t crc8_2bytes(uint8_t a, uint8_t b) {
  uint8_t crc = 0x00;
  uint8_t data[2] = { a, b };
  for (uint8_t k = 0; k < 2; k++) {
    crc ^= data[k];
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// =======================
// Đọc sensor ON theo lựa chọn HIGH/LOW
// =======================
static bool readSensorOn(uint8_t pin) {
#if SENSOR_ON_LEVEL_LOW
  return (digitalRead(pin) == LOW);   // ON = LOW
#else
  return (digitalRead(pin) == HIGH);  // ON = HIGH
#endif
}

static void updateDebounce(DebouncedInput &d) {
  bool rawOn = readSensorOn(d.pin);

  if (rawOn != d.lastRawOn) {
    d.lastRawOn = rawOn;
    d.lastChangeMs = millis();
  }

  if ((millis() - d.lastChangeMs) >= DEBOUNCE_MS) {
    d.stableOn = rawOn;
  }
}

static uint8_t buildMask(const DebouncedInput &a,
                         const DebouncedInput &b,
                         const DebouncedInput &c,
                         const DebouncedInput &d,
                         const DebouncedInput &e) {
  uint8_t m = 0;
  m |= (a.stableOn ? 1 : 0) << 0;
  m |= (b.stableOn ? 1 : 0) << 1;
  m |= (c.stableOn ? 1 : 0) << 2;
  m |= (d.stableOn ? 1 : 0) << 3;
  m |= (e.stableOn ? 1 : 0) << 4;
  return m;
}

// LED helpers
static void setLed(uint8_t pin, bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

static void applyMaskToLocalLEDs(uint8_t m) {
  setLed(L1, (m & (1 << 0)));
  setLed(L2, (m & (1 << 1)));
  setLed(L3, (m & (1 << 2)));
  setLed(L4, (m & (1 << 3)));
  setLed(L5, (m & (1 << 4)));
}

// Send frame
static void sendFrame(uint8_t seq, uint8_t mask) {
  uint8_t c = crc8_2bytes(seq, mask);
  hc12.write(0xAA);
  hc12.write(0x55);
  hc12.write(seq);
  hc12.write(mask);
  hc12.write(c);
}

// =======================
// BUZZER state machine
// =======================
enum BuzState : uint8_t { BUZ_IDLE = 0, BUZ_WAIT_DELAY, BUZ_ON, BUZ_OFF, BUZ_DONE };

static BuzState buzState = BUZ_IDLE;
static uint32_t buzT0 = 0;
static uint8_t buzBeepIndex = 0;

static void buzzerStopNow() { digitalWrite(BUZZER_PIN, HIGH); }

static void buzzerStartCycle() {
  buzState = BUZ_WAIT_DELAY;
  buzT0 = millis();
  buzBeepIndex = 0;
  buzzerStopNow();
}

static bool buzzerIsIdle() { return (buzState == BUZ_IDLE || buzState == BUZ_DONE); }

static void buzzerResetAll() {
  buzzerStopNow();
  buzState = BUZ_IDLE;
  buzT0 = 0;
  buzBeepIndex = 0;
}

static void buzzerUpdate(bool allowRun) {
  if (!allowRun) {
    if (buzState != BUZ_IDLE) buzzerResetAll();
    return;
  }

  switch (buzState) {
    case BUZ_IDLE: break;

    case BUZ_WAIT_DELAY:
      if (millis() - buzT0 >= BUZZER_DELAY_MS) {
        buzState = BUZ_ON;
        buzT0 = millis();
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;

    case BUZ_ON:
      if (millis() - buzT0 >= BUZZER_ON_MS) {
        buzzerStopNow();
        buzState = BUZ_OFF;
        buzT0 = millis();
      }
      break;

    case BUZ_OFF:
      if (millis() - buzT0 >= BUZZER_OFF_MS) {
        buzBeepIndex++;
        if (buzBeepIndex >= BUZZER_BEEP_COUNT) {
          buzzerStopNow();
          buzState = BUZ_DONE;
        } else {
          buzState = BUZ_ON;
          buzT0 = millis();
          digitalWrite(BUZZER_PIN, LOW);
        }
      }
      break;

    case BUZ_DONE:
      buzzerStopNow();
      break;
  }
}

// =======================
// Debounce objects
// =======================
DebouncedInput d1{S1, false, false, 0};
DebouncedInput d2{S2, false, false, 0};
DebouncedInput d3{S3, false, false, 0};
DebouncedInput d4{S4, false, false, 0};
DebouncedInput d5{S5, false, false, 0};

static uint8_t g_seq = 0;

// Trigger state
static bool stage1_done = false;
static bool stage2_done = false;
static bool prev5 = false;
static bool prev45 = false;

void setup() {
  // Sensor ON=HIGH => INPUT
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // LED A1..A5
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
  pinMode(L5, OUTPUT);
  applyMaskToLocalLEDs(0);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerStopNow();

  // init debounce
  d1.lastRawOn = readSensorOn(S1); d1.stableOn = d1.lastRawOn; d1.lastChangeMs = millis();
  d2.lastRawOn = readSensorOn(S2); d2.stableOn = d2.lastRawOn; d2.lastChangeMs = millis();
  d3.lastRawOn = readSensorOn(S3); d3.stableOn = d3.lastRawOn; d3.lastChangeMs = millis();
  d4.lastRawOn = readSensorOn(S4); d4.stableOn = d4.lastRawOn; d4.lastChangeMs = millis();
  d5.lastRawOn = readSensorOn(S5); d5.stableOn = d5.lastRawOn; d5.lastChangeMs = millis();

  hc12.begin(HC12_BAUD);
}

void loop() {
  updateDebounce(d1);
  updateDebounce(d2);
  updateDebounce(d3);
  updateDebounce(d4);
  updateDebounce(d5);

  uint8_t currentMask = buildMask(d1, d2, d3, d4, d5);

  // LED tại TX
  applyMaskToLocalLEDs(currentMask);

  // ===== BUZZER LOGIC =====
  bool s5  = (currentMask & (1 << 4)) != 0;
  bool s45 = (currentMask & (1 << 4)) && (currentMask & (1 << 3));

  if (!s5) {
    stage1_done = false;
    stage2_done = false;
    prev5 = false;
    prev45 = false;
    buzzerUpdate(false);
  } else {
    buzzerUpdate(true);

    // Trigger 1: Sensor5 vừa ON
    if (!stage1_done && (s5 && !prev5) && buzzerIsIdle()) {
      buzzerStartCycle();
      stage1_done = true;
    }

    // Trigger 2: Sensor4&5 vừa ON (sau khi đã có stage1)
    if (stage1_done && !stage2_done && (s45 && !prev45) && buzzerIsIdle()) {
      buzzerStartCycle();
      stage2_done = true;
    }

    prev5 = s5;
    prev45 = s45;
  }

  // ===== GỬI THEO CHU KỲ =====
  static uint32_t t = 0;
  if (millis() - t >= TX_PERIOD_MS) {
    t += TX_PERIOD_MS;

    uint8_t seq = g_seq++;

    for (uint8_t i = 0; i < REPEAT_SEND; i++) {
      sendFrame(seq, currentMask);
      delay(5);
    }
  }
}
