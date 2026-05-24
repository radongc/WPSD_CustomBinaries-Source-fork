/*
 * AIBridge IMBE codec wrapper.
 *
 * Bridges 11-byte IMBE frames (as produced by MMDVMHost's P25 network
 * protocol) and 8 kHz 16-bit PCM:
 *   - Decode:  mbelib's mbe_processImbe4400Data
 *   - Encode:  OP25's imbe_vocoder::imbe_encode + bit packing that mirrors
 *              exactly what mbelib's decoder expects.
 *
 * The encode bit packing is the inverse of mbe_decodeImbe4400Parms in
 * mbelib's imbe7200x4400.c:
 *   - b0 (8 bits) is split: bits 7..2 -> imbe_d[0..5], bits 1..0 -> imbe_d[85..86]
 *   - For each i in 2..6, frame_vector[i+1]'s low ba[L9][i-2][0] bits are
 *     placed in bb[i+1][width-1 .. 0] (LSB-first within bb).
 *   - The bo[L9][k] lookup maps each bb[r][c] entry to imbe_d[6+k] for
 *     k = 0..78 (79 entries).
 *
 * We pull the bo[], ba[], hoba[], ImbeJi[], B2[], quantstep[], standdev[]
 * tables directly from mbelib's header via install.sh, which sed-injects
 * 'static' on the const declarations so they live in our TU only.
 *
 * Limitations of this first-pass implementation:
 *   - V/UV flags and HOC bits not driven from OP25's frame_vector (which
 *     can't carry that much data in 8 int16s); they're left zero. That
 *     means all bands report as "voiced" and higher-order DCT
 *     coefficients are zero, which slightly flattens the timbre but
 *     should remain intelligible.
 *   - L is read from imbe_vocoder::param()->L if available; otherwise
 *     defaults to L=20 (a midrange value).
 */

extern "C" {
#include <mbelib.h>
}
#include "imbe_vocoder.h"
#include "mbelib_imbe_const.h"
#include <cstdint>
#include <cstring>

namespace {

/* Helper: clamp L to mbelib's table range. */
inline int clamp_L9(int L) {
    if (L < 9) L = 9;
    if (L > 56) L = 56;
    return L - 9;
}

} /* anonymous namespace */

extern "C" {

struct aibridge_imbe_ctx {
    mbe_parms cur_mp;
    mbe_parms prev_mp;
    mbe_parms prev_mp_enhanced;
    imbe_vocoder vocoder;
};

aibridge_imbe_ctx *aibridge_imbe_create(void) {
    auto *ctx = new aibridge_imbe_ctx();
    mbe_initMbeParms(&ctx->cur_mp, &ctx->prev_mp, &ctx->prev_mp_enhanced);
    return ctx;
}

void aibridge_imbe_destroy(aibridge_imbe_ctx *ctx) {
    delete ctx;
}

/* Decode 11 bytes (88 post-FEC IMBE bits) -> 160 int16 PCM samples. */
void aibridge_imbe_decode(aibridge_imbe_ctx *ctx,
                          const uint8_t *imbe_11b,
                          int16_t *pcm_160) {
    char imbe_bits[88];
    for (int i = 0; i < 88; i++) {
        imbe_bits[i] = (imbe_11b[i / 8] >> (7 - (i % 8))) & 1;
    }
    int errs = 0, errs2 = 0;
    char err_str[64] = {0};
    mbe_processImbe4400Data(pcm_160, &errs, &errs2, err_str, imbe_bits,
                            &ctx->cur_mp, &ctx->prev_mp,
                            &ctx->prev_mp_enhanced, 3);
}

/* Encode 160 int16 PCM samples -> 11 bytes IMBE.
 *
 * OP25's imbe_vocoder::imbe_encode populates frame_vector[0..7] with the
 * 88-bit IMBE frame already packed in the standard on-the-wire layout:
 *   fv[0] = 12 bits, fv[1..3] = 12 bits each, fv[4..6] = 11 bits each,
 *   fv[7] = 7 bits.  Total = 88 bits.
 *
 * Verified against OP25's encode_frame_vector and mbelib's decoder:
 * b_vec[0]'s top 6 bits land in fv[0][11..6] = imbe_d[0..5], and
 * b_vec[0]'s bottom 2 bits land in fv[7][2..1] = imbe_d[85..86] —
 * which is exactly where mbelib reads b0 from. So OP25 and mbelib
 * agree on the on-the-wire layout.
 *
 * Encoding is just: concatenate fv[0..7] MSB-first at the right widths
 * and pack the resulting 88 bits into 11 bytes MSB-first. No bo[]
 * remapping needed — that table is mbelib's internal representation
 * for decoding, not the wire format.
 */
void aibridge_imbe_encode(aibridge_imbe_ctx *ctx,
                          const int16_t *pcm_160,
                          uint8_t *imbe_11b) {
    /* imbe_vocoder::imbe_encode may modify its PCM input (pre-emphasis). */
    int16_t pcm_local[160];
    std::memcpy(pcm_local, pcm_160, sizeof(pcm_local));

    int16_t fv[8] = {0};
    ctx->vocoder.imbe_encode(fv, pcm_local);

    /* Pack fv[0..7] into 88 bits MSB-first, at standard widths. */
    static const int widths[8] = {12, 12, 12, 12, 11, 11, 11, 7};
    char imbe_d[88];
    std::memset(imbe_d, 0, sizeof(imbe_d));
    int bit = 0;
    for (int i = 0; i < 8; i++) {
        int w = widths[i];
        uint32_t v = (uint32_t)(uint16_t)fv[i];
        for (int j = w - 1; j >= 0; j--) {
            imbe_d[bit++] = (char)((v >> j) & 1);
        }
    }

    /* Collapse 88 single-bit chars into 11 bytes MSB-first. */
    std::memset(imbe_11b, 0, 11);
    for (int i = 0; i < 88; i++) {
        if (imbe_d[i]) {
            imbe_11b[i / 8] |= (uint8_t)(1u << (7 - (i % 8)));
        }
    }
}

} /* extern "C" */
