/* ESP32-S3 ROM stub: soft-float doubles (IEEE-754 binary64, little-endian
 * double passed/returned in a2:a3 = lo:hi).  Written from the IEEE-754
 * spec; the 64-bit integer helper calls (__ashldi3/__ashrdi3/__lshrdi3/
 * __udivdi3/__umoddi3) resolve at link time to the ROM stub's integer
 * slots (the real ROM's layout is identical: the helpers sit right after
 * __adddf3 at 0x40002184).  Compiled with xtensa-esp-elf-gcc, linked at
 * the real ROM addresses by softfloat.ld, emitted as a raw blob into
 * rom_stub.rs (tools/gen_softfloat.sh regenerates it).
 *
 * Conventions: unpacked mantissas are normalized to bit 55
 * (M in [2^55, 2^56), value = M * 2^(e - 1023 - 55)); pack() renormalizes,
 * rounds to nearest even at bit 3, and strips the implicit bit.  Denormals
 * unpack with e = 1 and no implicit bit, like libgcc fp-bit.
 */

typedef unsigned long long u64;
typedef long long s64;
typedef unsigned int u32;
typedef int s32;

#define DF_SIGN 0x8000000000000000ULL
#define DF_EXP  0x7FF0000000000000ULL
#define DF_MANT 0x000FFFFFFFFFFFFFULL

/* variable 64-bit shifts on 32-bit halves: keeps the blob free of
 * __ashldi3/__lshrdi3 calls (the ROM stub provides those at fixed
 * addresses, but the linker misresolves direct l32rs against them) */
static u64 shl64(u64 x, unsigned n)
{
    u32 lo = (u32)x, hi = (u32)(x >> 32);
    if (n >= 64)
        return 0;
    if (n == 0)
        return x;
    if (n < 32) {
        hi = (hi << n) | (lo >> (32 - n));
        lo = (u32)(lo << n);
    } else {
        hi = lo << (n - 32);
        lo = 0;
    }
    return ((u64)hi << 32) | lo;
}

static u64 shr64(u64 x, unsigned n)
{
    u32 lo = (u32)x, hi = (u32)(x >> 32);
    if (n >= 64)
        return 0;
    if (n == 0)
        return x;
    if (n < 32) {
        lo = (lo >> n) | (hi << (32 - n));
        hi >>= n;
    } else {
        lo = hi >> (n - 32);
        hi = 0;
    }
    return ((u64)hi << 32) | lo;
}

static u64 mul64(u64 a, u64 b)
{
    u32 al = (u32)a, ah = (u32)(a >> 32);
    u32 bl = (u32)b, bh = (u32)(b >> 32);
    u64 r = (u64)al * bl;
    r += ((u64)al * bh) << 32;
    r += ((u64)ah * bl) << 32;
    return r;
}

/* unpack: returns the biased exponent (1 for denormals), sign, and a
 * normalized mantissa (implicit bit set, bit 55). */
static int unpack(u64 x, int *sign, u64 *mant)
{
    int e = (int)((x >> 52) & 0x7FF);
    u64 m = x & DF_MANT;
    *sign = (x >> 63) ? 1 : 0;
    if (e == 0) {
        e = 1;
    } else {
        m |= 0x10000000000000ULL;
    }
    *mant = m << 3;
    return e;
}

/* pack: renormalize, round to nearest even at bit 3, strip the implicit
 * bit.  Returns the raw double; underflow -> +0, overflow -> inf. */
static u64 pack(u64 m, int e, int sign)
{
    while (e > 1 && m < (1ULL << 55)) {
        m <<= 1;
        e--;
    }
    while (m >= (1ULL << 56)) {
        m >>= 1;
        e++;
    }
    if (e <= 0)
        return 0;
    if (e >= 0x7FF)
        return ((u64)sign << 63) | DF_EXP;
    if ((m & 7) > 4 || ((m & 7) == 4 && (m & 8)))
        m += 8;
    m >>= 3;
    if (m & (1ULL << 53)) {
        m >>= 1;
        e++;
    }
    if (e >= 0x7FF)
        return ((u64)sign << 63) | DF_EXP;
    return ((u64)sign << 63) | ((u64)e << 52) | (m & DF_MANT);
}

static u64 addsub(u64 a, u64 b, int sub)
{
    int sa, sb, ea, eb, e, s;
    u64 ma, mb, m;
    int ra, rb;

    ra = ((a >> 52) & 0x7FF) == 0x7FF || (a & DF_MANT) != 0;
    rb = ((b >> 52) & 0x7FF) == 0x7FF || (b & DF_MANT) != 0;
    if (ra || rb) {
        /* nan wins, then inf; inf - inf = nan */
        if ((a & DF_EXP) == DF_EXP && (a & DF_MANT))
            return 0x7FF8000000000000ULL;
        if ((b & DF_EXP) == DF_EXP && (b & DF_MANT))
            return 0x7FF8000000000000ULL;
        sa = (a >> 63) ? 1 : 0;
        sb = (b >> 63) ? 1 : 0;
        if (sub)
            sb ^= 1;
        if ((a & DF_EXP) == DF_EXP && (b & DF_EXP) == DF_EXP && sa != sb)
            return 0x7FF8000000000000ULL;
        if ((a & DF_EXP) == DF_EXP)
            return ((u64)sa << 63) | DF_EXP;
        return ((u64)sb << 63) | DF_EXP;
    }
    if ((a & (DF_EXP | DF_MANT)) == 0) {
        sb = (b >> 63) ? 1 : 0;
        if (sub)
            sb ^= 1;
        return ((u64)sb << 63) | (b & (DF_EXP | DF_MANT));
    }
    if ((b & (DF_EXP | DF_MANT)) == 0)
        return a;
    sa = (a >> 63) ? 1 : 0;
    sb = (b >> 63) ? 1 : 0;
    if (sub)
        sb ^= 1;
    ea = unpack(a, &sa, &ma);
    eb = unpack(b, &sb, &mb);
    if (sa == sb) {
        if (ea < eb) {
            ma = shr64(ma, eb - ea);
            e = eb;
        } else {
            mb = shr64(mb, ea - eb);
            e = ea;
        }
        m = ma + mb;
        s = sa;
        if (m & (1ULL << 56)) {
            m >>= 1;
            e++;
        }
    } else {
        if (ea < eb) {
            ma = shr64(ma, eb - ea);
            e = eb;
        } else {
            mb = shr64(mb, ea - eb);
            e = ea;
        }
        if (ma >= mb) {
            m = ma - mb;
            s = sa;
        } else {
            m = mb - ma;
            s = sb;
        }
        if (m == 0)
            return 0;
    }
    return pack(m, e, s);
}

u64 __adddf3(u64 a, u64 b) { return addsub(a, b, 0); }
u64 __subdf3(u64 a, u64 b) { return addsub(a, b, 1); }

u64 __muldf3(u64 a, u64 b)
{
    int sa, sb, ea, eb, e, s;
    u64 ma, mb, m, r;

    if ((a >> 52) == 0x7FF || (a & DF_MANT) != 0 ||
        (b >> 52) == 0x7FF || (b & DF_MANT) != 0) {
        if ((a & DF_EXP) == DF_EXP && (a & DF_MANT))
            return 0x7FF8000000000000ULL;
        if ((b & DF_EXP) == DF_EXP && (b & DF_MANT))
            return 0x7FF8000000000000ULL;
        if ((a & DF_EXP) == DF_EXP && (b & (DF_EXP | DF_MANT)) == 0)
            return 0x7FF8000000000000ULL; /* inf * 0 */
        if ((b & DF_EXP) == DF_EXP && (a & (DF_EXP | DF_MANT)) == 0)
            return 0x7FF8000000000000ULL;
        sa = (a >> 63) ? 1 : 0;
        sb = (b >> 63) ? 1 : 0;
        return ((u64)(sa ^ sb) << 63) | DF_EXP;
    }
    if ((a & (DF_EXP | DF_MANT)) == 0 || (b & (DF_EXP | DF_MANT)) == 0)
        return 0;
    sa = (a >> 63) ? 1 : 0;
    sb = (b >> 63) ? 1 : 0;
    ea = unpack(a, &sa, &ma);
    eb = unpack(b, &sb, &mb);
    r = mul64(ma, mb);
    m = r >> 56; /* top 56 bits of the 112-bit product */
    if (r & 0xFFFFFFFFFFFFFFULL)
        m |= 1; /* sticky */
    e = ea + eb - 1023;
    s = sa ^ sb;
    if (m & (1ULL << 56)) {
        m >>= 1;
        e++;
    }
    return pack(m, e, s);
}

u64 __divdf3(u64 a, u64 b)
{
    int sa, sb, ea, eb, e, s, i;
    u64 ma, mb, m, q;

    if ((a >> 52) == 0x7FF || (a & DF_MANT) != 0 ||
        (b >> 52) == 0x7FF || (b & DF_MANT) != 0) {
        if ((a & DF_EXP) == DF_EXP && (a & DF_MANT))
            return 0x7FF8000000000000ULL;
        if ((b & DF_EXP) == DF_EXP && (b & DF_MANT))
            return 0x7FF8000000000000ULL;
        if ((a & DF_EXP) == DF_EXP && (b & DF_EXP) == DF_EXP)
            return 0x7FF8000000000000ULL; /* inf / inf */
        sa = (a >> 63) ? 1 : 0;
        sb = (b >> 63) ? 1 : 0;
        if ((a & DF_EXP) == DF_EXP)
            return ((u64)(sa ^ sb) << 63) | DF_EXP;
        return 0; /* 0 / inf */
    }
    if ((b & (DF_EXP | DF_MANT)) == 0) {
        if ((a & (DF_EXP | DF_MANT)) == 0)
            return 0x7FF8000000000000ULL; /* 0 / 0 */
        sa = (a >> 63) ? 1 : 0;
        sb = (b >> 63) ? 1 : 0;
        return ((u64)(sa ^ sb) << 63) | DF_EXP; /* x / 0 = inf */
    }
    if ((a & (DF_EXP | DF_MANT)) == 0)
        return 0;
    sa = (a >> 63) ? 1 : 0;
    sb = (b >> 63) ? 1 : 0;
    ea = unpack(a, &sa, &ma);
    eb = unpack(b, &sb, &mb);
    m = 0;
    q = 0;
    for (i = 0; i < 56; i++) {
        m = (m << 1) | ((ma >> 55) & 1);
        ma <<= 1;
        q <<= 1;
        if (m >= mb) {
            m -= mb;
            q |= 1;
        }
    }
    if (q == 0)
        return 0;
    e = ea - eb + 1023;
    s = sa ^ sb;
    return pack(q, e, s);
}

u64 __extendsfdf2(u32 f)
{
    int s = (f >> 31) ? 1 : 0;
    int e = (int)((f >> 23) & 0xFF);
    u32 m = f & 0x7FFFFF;

    if (e == 0xFF) {
        if (m)
            return 0x7FF8000000000000ULL;
        return ((u64)s << 63) | DF_EXP;
    }
    if (e == 0 && m == 0)
        return ((u64)s << 63);
    if (e == 0) {
        int n = 0;
        while ((m & 0x800000) == 0) {
            m <<= 1;
            n++;
        }
        return ((u64)s << 63) | ((u64)(1 + 896 - n) << 52) | ((u64)m << 29);
    }
    return ((u64)s << 63) | ((u64)(e + 896) << 52) | ((u64)m << 29);
}

s32 __fixdfsi(u64 a)
{
    int e = (int)((a >> 52) & 0x7FF);
    u64 m;
    int s = (a >> 63) ? 1 : 0;
    s32 r;

    if (e == 0x7FF)
        return s ? 0x80000000 : 0x7FFFFFFF;
    if (e == 0 && (a & DF_MANT) == 0)
        return 0;
    m = (a & DF_MANT) | 0x10000000000000ULL;
    if (e < 1023)
        return 0;
    if (e - 1023 >= 32)
        return s ? 0x80000000 : 0x7FFFFFFF;
    m = shr64(m, 1078 - e);
    r = (s32)m;
    if (s)
        r = -r;
    return r;
}

u64 __floatsidf(s32 i)
{
    u32 u = (u32)i;
    int s = 0, k = 0;
    u64 m;

    if (i < 0) {
        s = 1;
        u = (u32)0 - u;
    }
    if (u == 0)
        return 0;
    m = (u64)u;
    while (m < (1ULL << 55)) {
        m <<= 1;
        k++;
    }
    return pack(m, 1023 + 55 - k, s);
}

u64 __floatunsidf(u32 u)
{
    u64 m = (u64)u;
    int k = 0;

    if (u == 0)
        return 0;
    while (m < (1ULL << 55)) {
        m <<= 1;
        k++;
    }
    return pack(m, 1023 + 55 - k, 0);
}

/* compare: -1 (a<b), 0 (a==b), 1 (a>b), 2 (unordered) */
static int cmp(u64 a, u64 b)
{
    int ea, eb, sa, sb;
    u64 ma, mb;

    if ((a & DF_EXP) == DF_EXP && (a & DF_MANT))
        return 2;
    if ((b & DF_EXP) == DF_EXP && (b & DF_MANT))
        return 2;
    ea = (int)((a >> 52) & 0x7FF);
    eb = (int)((b >> 52) & 0x7FF);
    ma = a & DF_MANT;
    mb = b & DF_MANT;
    if (ea == 0 && ma == 0 && eb == 0 && mb == 0)
        return 0; /* +0 == -0 */
    sa = (a >> 63) ? 1 : 0;
    sb = (b >> 63) ? 1 : 0;
    if (sa != sb)
        return sa ? -1 : 1;
    if (ea != eb)
        return ((ea < eb) != (sa != 0)) ? -1 : 1;
    if (ma != mb)
        return ((ma < mb) != (sa != 0)) ? -1 : 1;
    return 0;
}

s32 __eqdf2(u64 a, u64 b) { return cmp(a, b) == 0 ? 0 : 1; }
s32 __nedf2(u64 a, u64 b) { return cmp(a, b) != 0 ? 0 : 1; }
s32 __gtdf2(u64 a, u64 b) { return cmp(a, b) > 0 ? 1 : 0; }
s32 __ledf2(u64 a, u64 b) { return cmp(a, b) <= 0 ? 0 : 1; }
s32 __ltdf2(u64 a, u64 b) { return cmp(a, b) < 0 ? -1 : 0; }
s32 __unorddf2(u64 a, u64 b) { return cmp(a, b) == 2 ? 1 : 0; }

u32 __bswapsi2(u32 x)
{
    return ((x & 0xFF) << 24) | ((x & 0xFF00) << 8) |
           ((x >> 8) & 0xFF00) | ((x >> 24) & 0xFF);
}