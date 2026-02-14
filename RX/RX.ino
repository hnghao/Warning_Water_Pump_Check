/*
  RECEIVER.ino (RX)
  - Arduino Nano + HC-12
  - Nhận frame: AA 55 SEQ MASK CRC8
  - Lọc chống chớp: cần MASK lặp lại N lần liên tiếp (STABLE_FRAMES_REQUIRED)
  - Điều khiển 5 LED: A1..A5 (LED1..LED5)
*/

#include <Arduino.h>
#include <SoftwareSerial.h>

// HC-12 TXD -> D11 (RX), HC-12 RXD <- D10 (TX)
SoftwareSerial hc12(11, 10); // RX, TX

#define HC12_BAUD              2400
#define STABLE_FRAMES_REQUIRED 3
#define FAILSAFE_TIMEOUT_MS    1000

// LED wiring:
// - LED_ACTIVE_LOW = 0: Pin HIGH sáng (Pin -> R -> LED -> GND)
// - LED_ACTIVE_LOW = 1: Pin LOW sáng (+5V -> R -> LED -> Pin)
#define LED_ACTIVE_LOW         0

// 5 LED: A1..A5 (đúng yêu cầu)
const uint8_t L1 = A1;
const uint8_t L2 = A2;
const uint8_t L3 = A3;
const uint8_t L4 = A4;
const uint8_t L5 = A5;

static uint8_t crc8_2bytes(uint8_t a, uint8_t b) {
  uint8_t crc = 0x00;
  uint8_t data[2] = {a, b};
  for (uint8_t k = 0; k < 2; k++) {
    crc ^= data[k];
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void setLed(uint8_t pin, bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

static void applyMask(uint8_t m) {
  setLed(L1, (m & (1 << 0)));
  setLed(L2, (m & (1 << 1)));
  setLed(L3, (m & (1 << 2)));
  setLed(L4, (m & (1 << 3)));
  setLed(L5, (m & (1 << 4)));
}

void setup() {
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
  pinMode(L5, OUTPUT);
  applyMask(0);

  hc12.begin(HC12_BAUD);
}

void loop() {
  // Parser frame AA 55 SEQ MASK CRC
  static uint8_t st = 0;
  static uint8_t seq = 0, mask = 0, crc = 0;

  // anti-flicker
  static uint8_t candidateMask = 0;
  static uint8_t stableCount = 0;
  static uint8_t appliedMask = 0;
  static uint32_t lastGoodMs = 0;

  while (hc12.available()) {
    uint8_t b = hc12.read();

    switch (st) {
      case 0: st = (b == 0xAA) ? 1 : 0; break;
      case 1: st = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); break;
      case 2: seq = b; st = 3; break;
      case 3: mask = b; st = 4; break;
      case 4:
        crc = b;
        st = 0;

        if (crc == crc8_2bytes(seq, mask)) {
          lastGoodMs = millis();

          // cần MASK lặp lại N lần liên tiếp
          if (stableCount == 0) {
            candidateMask = mask;
            stableCount = 1;
          } else if (mask == candidateMask) {
            if (stableCount < 255) stableCount++;
          } else {
            candidateMask = mask;
            stableCount = 1;
          }

          if (stableCount >= STABLE_FRAMES_REQUIRED && candidateMask != appliedMask) {
            appliedMask = candidateMask;
            applyMask(appliedMask);
            // không reset stableCount để tránh “đuổi theo” nhiễu
          }
        }
        break;
    }
  }

  // failsafe: mất link thì giữ nguyên LED (không nhấp nháy)
  if (lastGoodMs != 0 && (millis() - lastGoodMs) > FAILSAFE_TIMEOUT_MS) {
    // giữ nguyên appliedMask
  }
}
