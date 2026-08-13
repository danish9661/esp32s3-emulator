#ifndef _cpu_esp32s3_h_
#define _cpu_esp32s3_h_
#include <stdint.h>

typedef union {
    int8_t s8[16] QEMU_ALIGNED(16);
    uint8_t u8[16];
    int16_t s16[8];
    uint16_t u16[8];
    uint32_t u32[4];
    int32_t  s32[4];
    uint64_t u64[2];
} Q_reg;

typedef Q_reg esp_qreg_t;

typedef union {
    uint8_t u8[20];
} ACCQ_reg;

typedef struct CPUXtensaEsp32s3State_s
{
    Q_reg Q[8]; /* Active Q registers.*/
    ACCQ_reg ACCQ[2];
    Q_reg  UA_STATE;
    int64_t ACCX;
    uint8_t SAR_BYTE;
    uint8_t fft_width;
    uint8_t gpio_out;
    Q_reg  temp; // used as temp register for EE.FFT.AMS.S16.LD.INCP.UAUP operation
    int16_t temp_asm[2]; // Temp register value for ee.fft.ams.s16.st.incp operation
}CPUXtensaEsp32s3State;
#endif // _cpu_esp32s3_h_