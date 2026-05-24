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

/* Encode 160 int16 PCM samples -> 11 bytes IMBE. */
void aibridge_imbe_encode(aibridge_imbe_ctx *ctx,
                          const int16_t *pcm_160,
                          uint8_t *imbe_11b) {
    /* imbe_vocoder::imbe_encode may modify its PCM input (pre-emphasis).
     * Copy so we don't surprise the caller. */
    int16_t pcm_local[160];
    std::memcpy(pcm_local, pcm_160, sizeof(pcm_local));

    int16_t fv[8] = {0};
    ctx->vocoder.imbe_encode(fv, pcm_local);

    /* Determine L from the vocoder's internal state. param()->L is the
     * number of harmonics for this frame; mbelib's tables are indexed by
     * L9 = L - 9, valid range 9..56. */
    const IMBE_PARAM *p = ctx->vocoder.param();
    int L = (p ? p->num_harms : 20);
    int L9 = clamp_L9(L);

    /* The mbelib decoder's bb[][] bit array, which we fill from fv[] and
     * then scatter into imbe_d[] using the bo[] table. */
    char bb[58][12];
    std::memset(bb, 0, sizeof(bb));

    /* Pack quantizer indices for Gm[2..6] into bb[3..7].
     * mbelib reads bb[i+1][j] for j = width-1 down to 0 to form an MSB-first
     * binary string, so bb[i+1][j] = (bm >> j) & 1 with j as LSB-first index. */
    for (int i = 2; i <= 6; i++) {
        int width = (int)ba[L9][i - 2][0];
        if (width <= 0 || width > 12) continue;
        uint32_t bm = (uint32_t)(uint16_t)fv[i + 1];
        for (int j = 0; j < width; j++) {
            bb[i + 1][j] = (char)((bm >> j) & 1);
        }
    }

    /* Pack V/UV flags into bb[1][0..K-1]. K = number of V/UV bands.
     *   K = (L + 2) / 3  for L < 37
     *   K = 12           for L >= 37
     * (per mbe_decodeImbe4400Parms in mbelib's imbe7200x4400.c.)
     * fv[1] is the K-bit V/UV bitfield from imbe_vocoder. */
    int K_bands = (L < 37) ? ((L + 2) / 3) : 12;
    if (K_bands > 12) K_bands = 12;
    uint32_t vuv_bits = (uint32_t)(uint16_t)fv[1];
    for (int j = 0; j < K_bands; j++) {
        bb[1][j] = (char)((vuv_bits >> j) & 1);
    }

    /* Pack gain (b2) into bb[2][0..5]. mbelib's decoder builds b2 as a
     * 6-bit value from bb[2] entries (mbelib uses B2[64] which is a
     * 6-bit-indexed lookup → 64 entries). fv[2] is the gain index. */
    uint32_t gain_bits = (uint32_t)(uint16_t)fv[2];
    for (int j = 0; j < 6; j++) {
        bb[2][j] = (char)((gain_bits >> j) & 1);
    }

    /* Pack imbe_d as 88 single-bit chars first, then collapse to bytes. */
    char imbe_d[88];
    std::memset(imbe_d, 0, sizeof(imbe_d));

    /* b0: 8 bits split. Bits 7..2 -> imbe_d[0..5]; bits 1..0 -> imbe_d[85..86].
     * mbelib's decode reconstructs b0 via strtol with the same ordering. */
    uint32_t b0 = (uint32_t)(uint16_t)fv[0] & 0xFF;
    imbe_d[0] = (char)((b0 >> 7) & 1);
    imbe_d[1] = (char)((b0 >> 6) & 1);
    imbe_d[2] = (char)((b0 >> 5) & 1);
    imbe_d[3] = (char)((b0 >> 4) & 1);
    imbe_d[4] = (char)((b0 >> 3) & 1);
    imbe_d[5] = (char)((b0 >> 2) & 1);
    imbe_d[85] = (char)((b0 >> 1) & 1);
    imbe_d[86] = (char)((b0 >> 0) & 1);

    /* Use bo[L9][k] to scatter bb[r][c] into imbe_d[6..84] (79 positions). */
    for (int k = 0; k < 79; k++) {
        int r = bo[L9][k][0];
        int c = bo[L9][k][1];
        if (r >= 0 && r < 58 && c >= 0 && c < 12) {
            imbe_d[6 + k] = bb[r][c];
        }
    }
    /* imbe_d[87] (the last position) is a parity / reserved bit — leave 0. */

    /* Collapse 88 single-bit chars into 11 bytes MSB-first. */
    std::memset(imbe_11b, 0, 11);
    for (int i = 0; i < 88; i++) {
        if (imbe_d[i]) {
            imbe_11b[i / 8] |= (uint8_t)(1u << (7 - (i % 8)));
        }
    }
}

} /* extern "C" */
