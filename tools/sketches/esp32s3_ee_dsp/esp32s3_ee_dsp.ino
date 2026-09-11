// ESP32-S3 TIE/DSP execution validation for the emulator.
// Real firmware driving ee.* vector ops through inline asm (assembled by
// the stock S3 toolchain GAS — no driver stack involved, so this exercises
// the raw TIE datapath: Q regs, ACCX, SAR, vector ALU/MAC, 128-bit loads).
//
// Test 1 (dot): vld.128.ip q0/q1 <- A/B ([1..16]), vmulas.s8.accx,
//   rur.accx_0/1 -> accx must equal 1^2+...+16^2 = 1496.
// Test 2 (sat add): vld.128.ip q0/q1 <- C/D ([100]/[50]), vadds.s8 q2,
//   vst.128.ip q2 -> mem must read back [127]*16 (asymmetric saturation).
// Test 3 (cmul fused): vld q2/q3 <- E/F ([3,0,4,0...]/[1,0,0...]),
//   SAR=1, cmul.s16.ld.incp q0 <- G + MAC q1 = (3+4i)(1+0i)>>1 = (1,2),
//   vst q1 -> H must read back [1,0,2,0...].
// Test 4 (ldf128 round trip): S = [1.0f, 2.0f, 3.0f, 4.0f] words,
//   ldf.128.ip f4-f7 <- S (raw word, a2 = S), stf.128.ip f4-f7 -> D
//   (raw word), D must equal S word-for-word (pairwise-swap round trip).
// Test 5 (srs): dot = 1496 again, srs shift 2 -> 374 (ACCX becomes 374),
//   then srs shift 0 -> 374 again (proves the ACCX write-back: the second
//   srs sees 374, not the original 1496).
// Markers: "EE DSP DOT OK", "EE DSP VADDS OK", "EE DSP CMUL OK",
// "EE DSP LDF128 OK", "EE DSP SRS OK", "EE DSP DONE"
// (any mismatch prints "EE DSP FAIL", which the battery treats as failure).

static int8_t dsp_a[16] __attribute__((aligned(16)));
static int8_t dsp_b[16] __attribute__((aligned(16)));
static int8_t dsp_c[16] __attribute__((aligned(16)));
static int8_t dsp_d[16] __attribute__((aligned(16)));
static int8_t dsp_o[16] __attribute__((aligned(16)));
static int8_t dsp_e[16] __attribute__((aligned(16)));
static int8_t dsp_f[16] __attribute__((aligned(16)));
static int8_t dsp_g[16] __attribute__((aligned(16)));
static int8_t dsp_h[16] __attribute__((aligned(16)));
static uint32_t dsp_s[4] __attribute__((aligned(16)));
static uint32_t dsp_t[4] __attribute__((aligned(16)));

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

  // --- Test 3: fused complex multiply + load.
  for (int i = 0; i < 16; i++) {
    dsp_e[i] = 0;
    dsp_f[i] = 0;
    dsp_g[i] = (int8_t)(i + 40);
    dsp_h[i] = 0;
  }
  dsp_e[0] = 3;
  dsp_e[2] = 4;  // q2 pair0 = (3+4i)
  dsp_f[0] = 1;  // q3 pair0 = (1+0i)
  uint8_t *pe = (uint8_t *)dsp_e;
  uint8_t *pf = (uint8_t *)dsp_f;
  uint8_t *pg = (uint8_t *)dsp_g;
  uint8_t *ph = (uint8_t *)dsp_h;
  int sar1 = 1;
  __asm__ volatile(
      "wsr.sar %4\n"
      "ee.vld.128.ip q2, %0, 16\n"
      "ee.vld.128.ip q3, %1, 16\n"
      "ee.cmul.s16.ld.incp q0, %2, q1, q2, q3, 0\n"
      "ee.vst.128.ip q1, %3, 16\n"
      : "+a"(pe), "+a"(pf), "+a"(pg), "+a"(ph)
      : "a"(sar1)
      : "memory");
  Serial.print("EE DSP CMUL h0=");
  Serial.println((int)dsp_h[0]);
  if (dsp_h[0] == 1 && dsp_h[1] == 0 && dsp_h[2] == 2 && dsp_h[3] == 0) {
    Serial.println("EE DSP CMUL OK");
  } else {
    Serial.println("EE DSP FAIL");
  }

  // --- Test 4: 128-bit FPR load/store round trip (raw words: GAS emits
  // identical bytes for the same mnemonics, cross-checked during bring-up).
  dsp_s[0] = 0x3F800000;
  dsp_s[1] = 0x40000000;
  dsp_s[2] = 0x40400000;
  dsp_s[3] = 0x40800000;
  for (int i = 0; i < 4; i++) dsp_t[i] = 0;
  uint32_t *ps = dsp_s;
  uint32_t *pt = dsp_t;
  __asm__ volatile(
      "mov.n a2, %0\n"
      ".word 0x8272602f\n"  // ee.ldf.128.ip f4, f5, f6, f7, a2, 0
      "mov.n a2, %1\n"
      ".word 0x9272602f\n"  // ee.stf.128.ip f4, f5, f6, f7, a2, 0
      :
      : "a"(ps), "a"(pt)
      : "a2", "memory");
  bool ok128 = true;
  for (int i = 0; i < 4; i++) {
    if (dsp_t[i] != dsp_s[i]) ok128 = false;
  }
  Serial.print("EE DSP LDF128 t0=");
  Serial.println(dsp_t[0], HEX);
  if (ok128) {
    Serial.println("EE DSP LDF128 OK");
  } else {
    Serial.println("EE DSP FAIL");
  }

  // --- Test 5: SRS shift+saturate with ACCX write-back.
  uint8_t *pa5 = (uint8_t *)dsp_a;
  uint8_t *pb5 = (uint8_t *)dsp_b;
  uint32_t srs_a = 0, srs_b = 0;
  int sh2 = 2, sh0 = 0;
  __asm__ volatile(
      "ee.zero.accx\n"
      "ee.vld.128.ip q0, %0, 16\n"
      "ee.vld.128.ip q1, %1, 16\n"
      "ee.vmulas.s8.accx q0, q1\n"
      "ee.srs.accx %2, %4, 0\n"
      "ee.srs.accx %3, %5, 0\n"
      : "+a"(pa5), "+a"(pb5), "=a"(srs_a), "=a"(srs_b)
      : "a"(sh2), "a"(sh0)
      : "memory");
  Serial.print("EE DSP SRS a=");
  Serial.print(srs_a);
  Serial.print(" b=");
  Serial.println(srs_b);
  if (srs_a == 374 && srs_b == 374) {
    Serial.println("EE DSP SRS OK");
  } else {
    Serial.println("EE DSP FAIL");
  }
  Serial.println("EE DSP DONE");
}

void loop() {
  delay(1000);
}
