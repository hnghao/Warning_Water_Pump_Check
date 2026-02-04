/*
  TRANSMISSION.ino
  - Arduino Nano + HC-12 (TX)
  - 5 cảm biến kiểu "nút nhấn": ON -> OUT kéo xuống LOW (0), OFF -> HIGH nhờ INPUT_PULLUP
  - Gửi liên tục: frame AA 55 SEQ MASK CRC8
  - MASK: bit0..bit4 tương ứng sensor1..sensor5
*/

#include <Arduino.h>
#include <SoftwareSerial.h>

// =======================
// HC-12 Serial
// =======================
// HC-12 TXD -> D10 (RX), HC-12 RXD <- D11 (TX)
SoftwareSerial hc12(10, 11); // RX, TX

// =======================
// Cấu hình
// =======================
#define HC12_BAUD      2400   // phải trùng cấu hình AT+Bxxxx của HC-12 (2400 hoặc 1200)
#define TX_PERIOD_MS   20     // chu kỳ gửi (20ms = 50Hz). Nếu nhiễu nặng, tăng 30~50ms
#define DEBOUNCE_MS    15     // debounce cho cảm biến kiểu nút nhấn
#define REPEAT_SEND    2      // gửi lặp 2 lần để chống rớt gói (2 hoặc 3)

// 5 chân cảm biến
const uint8_t S1 = 2;
const uint8_t S2 = 3;
const uint8_t S3 = 4;
const uint8_t S4 = 5;
const uint8_t S5 = 6;

// =======================
// Struct debounce (KHAI BÁO TRƯỚC khi dùng!)
// =======================
struct DebouncedInput {
  uint8_t  pin;
  bool     stableOn;       // trạng thái đã debounce (ON/OFF)
  bool     lastRawOn;      // trạng thái thô lần trước
  uint32_t lastChangeMs;   // thời điểm raw đổi lần cuối
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
// Cảm biến active-LOW: LOW = ON
// =======================
static bool readOn_ActiveLow(uint8_t pin) {
  return (digitalRead(pin) == LOW);
}

// =======================
// Update debounce
// =======================
static void updateDebounce(DebouncedInput &d) {
  bool rawOn = readOn_ActiveLow(d.pin);

  if (rawOn != d.lastRawOn) {
    d.lastRawOn = rawOn;
    d.lastChangeMs = millis();
  }

  if ((millis() - d.lastChangeMs) >= DEBOUNCE_MS) {
    d.stableOn = rawOn;
  }
}

// =======================
// Tạo mask 5 bit
// =======================
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

// =======================
// Gửi 1 frame
// =======================
static void sendFrame(uint8_t seq, uint8_t mask) {
  uint8_t c = crc8_2bytes(seq, mask);

  hc12.write(0xAA);
  hc12.write(0x55);
  hc12.write(seq);
  hc12.write(mask);
  hc12.write(c);
}

// =======================
// Biến toàn cục debounce
// =======================
DebouncedInput d1{S1, false, false, 0};
DebouncedInput d2{S2, false, false, 0};
DebouncedInput d3{S3, false, false, 0};
DebouncedInput d4{S4, false, false, 0};
DebouncedInput d5{S5, false, false, 0};

static uint8_t g_seq = 0;

void setup() {
  // Active-LOW + OUT như nút nhấn => bắt buộc PULLUP để tránh floating khi OFF
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  pinMode(S5, INPUT_PULLUP);

  // init debounce theo trạng thái hiện tại
  d1.lastRawOn = readOn_ActiveLow(S1); d1.stableOn = d1.lastRawOn; d1.lastChangeMs = millis();
  d2.lastRawOn = readOn_ActiveLow(S2); d2.stableOn = d2.lastRawOn; d2.lastChangeMs = millis();
  d3.lastRawOn = readOn_ActiveLow(S3); d3.stableOn = d3.lastRawOn; d3.lastChangeMs = millis();
  d4.lastRawOn = readOn_ActiveLow(S4); d4.stableOn = d4.lastRawOn; d4.lastChangeMs = millis();
  d5.lastRawOn = readOn_ActiveLow(S5); d5.stableOn = d5.lastRawOn; d5.lastChangeMs = millis();

  hc12.begin(HC12_BAUD);
}

void loop() {
  // update debounce liên tục
  updateDebounce(d1);
  updateDebounce(d2);
  updateDebounce(d3);
  updateDebounce(d4);
  updateDebounce(d5);

  static uint32_t t = 0;
  if (millis() - t >= TX_PERIOD_MS) {
    t += TX_PERIOD_MS;

    uint8_t mask = buildMask(d1, d2, d3, d4, d5);
    uint8_t seq = g_seq++;

    // Gửi lặp để tăng độ chắc
    for (uint8_t i = 0; i < REPEAT_SEND; i++) {
      sendFrame(seq, mask);
      delay(5);
    }
  }
}
