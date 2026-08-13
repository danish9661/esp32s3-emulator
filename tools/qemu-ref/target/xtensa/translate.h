#ifndef _translate_h_
#define _translate_h_

#include "qemu/osdep.h"

#include "cpu.h"
#include "exec/exec-all.h"
#include "disas/disas.h"
#include "tcg/tcg-op.h"
#include "qemu/log.h"
#include "qemu/qemu-print.h"
#include "exec/cpu_ldst.h"
#include "semihosting/semihost.h"
#include "exec/translator.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#include "exec/log.h"


struct DisasContext {
    DisasContextBase base;
    const XtensaConfig *config;
    uint32_t pc;
    int cring;
    int ring;
    uint32_t lbeg_off;
    uint32_t lend;

    bool sar_5bit;
    bool sar_m32_5bit;
    bool sar_m32_allocated;
    TCGv_i32 sar_m32;

    unsigned window;
    unsigned callinc;
    bool cwoe;

    bool debug;
    bool icount;
    TCGv_i32 next_icount;

    unsigned cpenable;

    uint32_t op_flags;
    xtensa_insnbuf_word insnbuf[MAX_INSNBUF_LENGTH];
    xtensa_insnbuf_word slotbuf[MAX_INSNBUF_LENGTH];
};

// These functions common for Xtensa and will be used in other Xtensa CPUs extensions
void gen_exception_cause(DisasContext *dc, uint32_t cause);
MemOp gen_load_store_alignment(DisasContext *dc, MemOp mop, TCGv_i32 addr);
void get_f32_i1(const OpcodeArg *arg, OpcodeArg *arg32, int i0);
void put_f32_i1(const OpcodeArg *arg, const OpcodeArg *arg32, int i0);
void get_f32_o1(const OpcodeArg *arg, OpcodeArg *arg32, int o0);
void put_f32_o1(const OpcodeArg *arg, const OpcodeArg *arg32, int o0);

#endif // _translate_h_