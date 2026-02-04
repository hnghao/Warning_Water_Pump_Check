#include <SoftwareSerial.h>

// HC-12 TXD -> D10 (RX), HC-12 RXD <- D11 (TX)
SoftwareSerial hc12(10, 11); // RX, TX

// ================== CẤU HÌNH ==================
#define HC12_BAUD              2400
#define STABLE_FRAMES_REQUIRED 3     // chống chớp: số frame giống nhau liên tiếp
#define FAILSAFE_TIMEOUT_MS    1000  // không có frame hợp lệ quá lâu: giữ trạng thái hiện tại

// LED wiring:
// - LED_ACTIVE_LOW = 0: Pin HIGH sáng (Pin -> R -> LED -> GND)
// - LED_ACTIVE_LOW = 1: Pin LOW sáng (+5V -> R -> LED -> Pin)
#define LED_ACTIVE_LOW         0

// ===== BUZZER =====
#define BUZZER_PIN             7     // <-- đổi sang chân bạn muốn
#define BUZZER_FREQ_HZ         2000  // tần số beep
#define BUZZER_BEEP_ON_MS      150   // thời gian kêu 1 tiếng
#define BUZZER_BEEP_OFF_MS     150   // thời gian nghỉ giữa các tiếng
#define BUZZER_DELAY_10S_MS    10000 // chờ 10 giây
#define BUZZER_BEEP_COUNT      4     // 4 tiếng mỗi lần
#define BUZZER_REPEAT_TIMES    2     // lặp 2 lần rồi tắt

// 5 LED
const uint8_t L1 = 2;
const uint8_t L2 = 3;
const uint8_t L3 = 4;
const uint8_t L4 = 5;
const uint8_t L5 = 6;

// ================== CRC8 ==================
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

// ================== LED helpers ==================
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

// ================== BUZZER state machine ==================
// chỉ kích khi sensor5 ON, sau 10s -> beep 4 tiếng -> lặp 2 lần -> done
enum BuzState : uint8_t {
  BUZ_IDLE = 0,
  BUZ_WAIT_10S,
  BUZ_BEEP_ON,
  BUZ_BEEP_OFF,
  BUZ_DONE
};

static BuzState buzState = BUZ_IDLE;
static uint32_t buzT0 = 0;
static uint8_t buzBeepIndex = 0;   // 0..(BUZZER_BEEP_COUNT-1)
static uint8_t buzRoundIndex = 0;  // 0..(BUZZER_REPEAT_TIMES-1)

static void buzzerStopNow() {
  digitalWrite(BUZZER_PIN, HIGH);
}

static void buzzerResetAll() {
  buzzerStopNow();
  buzState = BUZ_IDLE;
  buzT0 = 0;
  buzBeepIndex = 0;
  buzRoundIndex = 0;
}

// gọi trong loop() thường xuyên
static void buzzerUpdate(bool sensor5On) {
  // Nếu sensor5 OFF: hủy toàn bộ, đảm bảo không kêu
  if (!sensor5On) {
    if (buzState != BUZ_IDLE) buzzerResetAll();
    return;
  }

  // sensor5 ON:
  switch (buzState) {
    case BUZ_IDLE:
      // bắt đầu đếm 10 giây (chỉ 1 lần cho 1 vòng)
      buzState = BUZ_WAIT_10S;
      buzT0 = millis();
      buzBeepIndex = 0;
      // buzRoundIndex giữ nguyên (để lặp 2 lần)
      break;

    case BUZ_WAIT_10S:
      if (millis() - buzT0 >= BUZZER_DELAY_10S_MS) {
        // bắt đầu beep 4 tiếng
        buzState = BUZ_BEEP_ON;
        buzT0 = millis();
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;

    case BUZ_BEEP_ON:
      if (millis() - buzT0 >= BUZZER_BEEP_ON_MS) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzState = BUZ_BEEP_OFF;
        buzT0 = millis();
      }
      break;

    case BUZ_BEEP_OFF:
      if (millis() - buzT0 >= BUZZER_BEEP_OFF_MS) {
        buzBeepIndex++;

        if (buzBeepIndex >= BUZZER_BEEP_COUNT) {
          // kết thúc 1 lần 4 tiếng
          buzRoundIndex++;

          if (buzRoundIndex >= BUZZER_REPEAT_TIMES) {
            // xong 2 lần -> DONE
            buzState = BUZ_DONE;
            buzzerStopNow();
          } else {
            // còn lần nữa -> quay về chờ 10 giây rồi kêu tiếp
            buzState = BUZ_WAIT_10S;
            buzT0 = millis();
            buzBeepIndex = 0;
          }
        } else {
          // tiếp tục beep tiếng tiếp theo
          buzState = BUZ_BEEP_ON;
          buzT0 = millis();
          digitalWrite(BUZZER_PIN, LOW);
        }
      }
      break;

    case BUZ_DONE:
      // Đã hoàn tất 2 vòng. Dù sensor5 vẫn ON cũng không kêu nữa.
      buzzerStopNow();
      break;
  }
}

// ================== RX parser + anti-flicker ==================
void setup() {
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
  pinMode(L5, OUTPUT);
  applyMask(0);

  pinMode(BUZZER_PIN, OUTPUT);
  buzzerStopNow();

  hc12.begin(HC12_BAUD);
}

void loop() {
  static uint8_t st = 0;
  static uint8_t seq = 0, mask = 0, crc = 0;

  static uint8_t candidateMask = 0;
  static uint8_t stableCount = 0;
  static uint8_t appliedMask = 0;
  static uint32_t lastGoodMs = 0;

  // --- nhận & parse frame ---
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

          // anti-flicker: cần MASK lặp lại N lần liên tiếp
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
            // LƯU Ý: không reset stableCount để không “đuổi theo” nhiễu
          }
        }
        break;
    }
  }

  // failsafe: nếu mất link lâu thì giữ nguyên (không tắt LED để tránh nhấp nháy)
  if (lastGoodMs != 0 && (millis() - lastGoodMs) > FAILSAFE_TIMEOUT_MS) {
    // giữ nguyên appliedMask
  }

  // --- update buzzer theo trạng thái sensor5 (bit4) đã APPLY (ổn định) ---
  bool sensor5On = (appliedMask & (1 << 4)) != 0;
  buzzerUpdate(sensor5On);
}
