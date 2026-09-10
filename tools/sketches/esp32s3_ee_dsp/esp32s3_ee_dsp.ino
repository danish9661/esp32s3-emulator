// ESP32-S3 TIE/DSP execution validation for the emulator.
// Real firmware driving ee.* vector ops through inline asm (assembled by
// the stock S3 toolchain GAS — no driver stack involved, so this exercises
// the raw TIE datapath: Q regs, ACCX, SAR, vector ALU/MAC, 128-bit loads).
//
// Test 1 (dot): vld.128.ip q0/q1 <- A/B ([1..16]), vmulas.s8.accx,
//   rur.accx_0/1 -> accx must equal 1^2+...+16^2 = 1496.
// Test 2 (sat add): vld.128.ip q0/q1 <- C/D ([100]/[50]), vadds.s8 q2,
//   vst.128.ip q2 -> mem must read back [127]*16 (asymmetric saturation).
// Markers: "EE DSP DOT OK", "EE DSP VADDS OK", "EE DSP DONE"
// (any mismatch prints "EE DSP FAIL", which the battery treats as failure).

static int8_t dsp_a[16] __attribute__((aligned(16)));
static int8_t dsp_b[16] __attribute__((aligned(16)));
static int8_t dsp_c[16] __attribute__((aligned(16)));
static int8_t dsp_d[16] __attribute__((aligned(16)));
static int8_t dsp_o[16] __attribute__((aligned(16)));

void setup() {
  Serial.begin(115200);
  delay(200);
  for (int i = 0; i < 16; i++) {
    dsp_a[i] = (int8_t)(i + 1);
    dsp_b[i] = (int8_t)(i + 1);
    dsp_c[i] = 100;
    dsp_d[i] = 50;
    dsp_o[i] = 0;
  }

  // --- Test 1: dot product into ACCX.
  uint8_t *pa = (uint8_t *)dsp_a;
  uint8_t *pb = (uint8_t *)dsp_b;
  uint32_t lo = 0, hi = 0;
  __asm__ volatile(
      "ee.zero.accx\n"
      "ee.vld.128.ip q0, %0, 16\n"
      "ee.vld.128.ip q1, %1, 16\n"
      "ee.vmulas.s8.accx q0, q1\n"
      "rur.accx_0 %2\n"
      "rur.accx_1 %3\n"
      : "+a"(pa), "+a"(pb), "=a"(lo), "=a"(hi)
      :
      : "memory");
  Serial.print("EE DSP DOT accx=");
  Serial.println(lo);
  if (lo == 1496 && hi == 0) {
    Serial.println("EE DSP DOT OK");
  } else {
    Serial.println("EE DSP FAIL");
  }

  // --- Test 2: saturating vector add (100+50 clamps to 127, not -128).
  uint8_t *pc = (uint8_t *)dsp_c;
  uint8_t *pd = (uint8_t *)dsp_d;
  uint8_t *po = (uint8_t *)dsp_o;
  __asm__ volatile(
      "ee.vld.128.ip q0, %0, 16\n"
      "ee.vld.128.ip q1, %1, 16\n"
      "ee.vadds.s8 q2, q0, q1\n"
      "ee.vst.128.ip q2, %2, 16\n"
      : "+a"(pc), "+a"(pd), "+a"(po)
      :
      : "memory");
  bool ok = true;
  for (int i = 0; i < 16; i++) {
    if (dsp_o[i] != 127) ok = false;
  }
  Serial.print("EE DSP VADDS o0=");
  Serial.println((int)dsp_o[0]);
  if (ok) {
    Serial.println("EE DSP VADDS OK");
  } else {
    Serial.println("EE DSP FAIL");
  }
  Serial.println("EE DSP DONE");
}

void loop() {
  delay(1000);
}
