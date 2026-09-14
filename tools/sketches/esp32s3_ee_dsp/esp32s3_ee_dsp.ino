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
static int8_t dsp_o2[16] __attribute__((aligned(16)));
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
  // NOTE: srs outputs use early-clobber '&' — the first srs writes its
  // rd before the second srs reads its shift input, so GCC must not
  // alias an output over a later-consumed input (without '&', GCC put
  // srs_a and sh0 in the same register and the second shift read 374).
  __asm__ volatile(
      "ee.zero.accx\n"
      "ee.vld.128.ip q0, %0, 16\n"
      "ee.vld.128.ip q1, %1, 16\n"
      "ee.vmulas.s8.accx q0, q1\n"
      "ee.srs.accx %2, %4, 0\n"
      "ee.srs.accx %3, %5, 0\n"
      : "+a"(pa5), "+a"(pb5), "=&a"(srs_a), "=&a"(srs_b)
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

  // --- Test 6: fused vector-ALU + load/store (previously-unmapped tail:
  // vmax.s16.ld.incp, vmin.s8.st.incp, vsubs.s32.ld.incp, vmul.u8.st).
  // Reload C/D ([100]/[50]) into q0/q1 with plain loads (deterministic
  // ALU sources regardless of earlier tests), then one fused op per
  // family, storing each ALU result for the verdict.
  uint8_t *pa6 = (uint8_t *)dsp_c;
  uint8_t *pb6 = (uint8_t *)dsp_d;
  uint8_t *po6 = (uint8_t *)dsp_o;
  for (int i = 0; i < 16; i++) dsp_o[i] = 0;
  // NOTE: SAR is set from a stack-slot reload inside the asm (wsr reads a
  // dedicated input re-loaded per block): an earlier draft passed SAR in a
  // register across Serial.print calls, and GCC kept it in a3 without
  // reloading — the calls clobbered it, silently changing the shift.
  __asm__ volatile(
      "movi a3, 1\n"
      "wsr.sar a3\n"
      "ee.vld.128.ip q0, %0, 16\n"
      "ee.vld.128.ip q1, %1, 16\n"
      // q5 = max(q0,q1) = [100] (s16 lanes); store q5 to dsp_o via vst.
      "ee.vmax.s16.ld.incp q2, %0, q5, q0, q1\n"
      "ee.vst.128.ip q5, %2, 16\n"
      : "+a"(pa6), "+a"(pb6), "+a"(po6)
      :
      : "a3", "memory");
  bool ok_max = true;
  for (int i = 0; i < 16; i++) {
    if (dsp_o[i] != 100) ok_max = false;
  }
  Serial.print("EE DSP FUSED max=");
  Serial.println((int)(uint8_t)dsp_o[0]);
  // q3 = (q0*q1)>>1 truncated to u8 = (100*50)>>1 & 0xFF = 196.
  // The fused store advances its AR by 16 (incp postupdate), so the
  // verdict reads dsp_o[16]; dsp_o[0..16) holds the store half (mem-qu
  // source q2, seeded to [7] below through a dedicated array so the
  // verdict is independent of stale Q state). This validates BOTH halves
  // of the fused store (store-half bytes + ALU result) with no ambiguity.
  static int8_t dsp_p[16] __attribute__((aligned(16)));
  for (int i = 0; i < 16; i++) dsp_p[i] = 7;
  uint8_t *pa8 = (uint8_t *)dsp_c;
  uint8_t *pb8 = (uint8_t *)dsp_d;
  uint8_t *po8 = (uint8_t *)dsp_o;
  uint8_t *pp8 = (uint8_t *)dsp_p;
  uint8_t *pq8 = (uint8_t *)dsp_o2;
  for (int i = 0; i < 16; i++) { dsp_o[i] = 0; dsp_o2[i] = 0; }
  __asm__ volatile(
      "movi a3, 1\n"
      "wsr.sar a3\n"
      "ee.vld.128.ip q0, %0, 16\n"
      "ee.vld.128.ip q1, %1, 16\n"
      "ee.vld.128.ip q2, %3, 16\n"
      "ee.vmul.u8.st.incp q2, %2, q3, q0, q1\n"
      "ee.vst.128.ip q3, %4, 16\n"
      : "+a"(pa8), "+a"(pb8), "+a"(po8), "+a"(pp8), "+a"(pq8)
      :
      : "a3", "memory");
  Serial.print("EE DSP FUSED store=");
  Serial.print((int)(uint8_t)dsp_o[0]);
  Serial.print(" mul=");
  Serial.println((int)(uint8_t)dsp_o2[0]);
  if (ok_max && (uint8_t)dsp_o[0] == 7 && (uint8_t)dsp_o2[0] == 196) {
    Serial.println("EE DSP FUSED OK");
  } else {
    Serial.println("EE DSP FAIL");
  }
  Serial.println("EE DSP DONE");
}

void loop() {
  delay(1000);
}
