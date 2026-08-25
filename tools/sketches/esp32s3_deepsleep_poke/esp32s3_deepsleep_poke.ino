// Direct register-poke validation for the emulator's deep-sleep state machine.
// Mirrors the other P5 "poke" sketches (RMT/TWAI/I2C/...): we drive RTC_CNTL
// directly instead of going through the esp-idf deep-sleep driver (whose
// preparation path polls peripherals the emulator does not model and hangs).
//
// Behaviour: on first boot we program the sleep timer and write
// RTC_CNTL_SLEEP_EN (bit 31 of STATE0 @ +0x18).  The emulator detects that,
// fast-forwards the sleep, and reboots with the timer wakeup-cause bit set.
// On the reboot we read RTC_CNTL_SLP_WAKEUP_CAUSE_REG (@ +0x130) and, if the
// timer bit (BIT3) is set, print DEEPSLEEP WOKE / DEEPSLEEP PASS.

#define RTC_CNTL_BASE 0x60008000
#define SLP_TIMER0_REG (RTC_CNTL_BASE + 0x04)
#define SLP_TIMER1_REG (RTC_CNTL_BASE + 0x08)
#define STATE0_REG     (RTC_CNTL_BASE + 0x18)
#define WAKEUP_CAUSE_REG (RTC_CNTL_BASE + 0x130)
#define SLEEP_EN_BIT   (1u << 31)
#define TIMER_WAKEUP_BIT (1u << 3)

void setup() {
  Serial.begin(115200);
  delay(50);

  uint32_t cause = REG_READ(WAKEUP_CAUSE_REG);
  if (cause & TIMER_WAKEUP_BIT) {
    Serial.println("DEEPSLEEP WOKE");
    Serial.println("DEEPSLEEP PASS");
    while (1) { delay(10); }
  }

  Serial.println("DEEPSLEEP START");

  // Program a short sleep period (value is irrelevant for the emulator, which
  // fast-forwards the sleep as a fixed step budget; we just need SLP_TIMER0/1
  // to carry a non-zero duration so the model records something).
  REG_WRITE(SLP_TIMER0_REG, 0x00001234);
  REG_WRITE(SLP_TIMER1_REG, 0x00000000);

  // Trigger the power-down: set RTC_CNTL_SLEEP_EN in STATE0.
  REG_WRITE(STATE0_REG, REG_READ(STATE0_REG) | SLEEP_EN_BIT);

  // Should never return: the emulator halts the CPU and reboots.
  while (1) { delay(10); }
}

void loop() {}
