/*
 * AIBridge IMBE codec wrapper.
 *
 * Bridges 11-byte IMBE frames (as produced by MMDVMHost's P25 network
 * protocol) and 8 kHz 16-bit PCM:
 *   - Decode:  mbelib's mbe_processImbe4400Data
 *   - Encode:  OP25's imbe_vocoder::imbe_encode + bit packing per TIA-102.BABA
 *
 * Built into libaibridge_imbe.so and loaded by imbe_codec.py via ctypes.
 *
 * NOTE on encode bit packing: the 88 information bits in an IMBE frame
 * carry the b0..b7 parameter vector. Some field widths are L-dependent
 * (L = number of harmonics, derived from b0), but using the maximum
 * widths in fixed positions gives a packing that round-trips through
 * most decoders. If audio quality is poor we may need to switch to
 * an L-aware packing.
 */

/* mbelib.h doesn't have extern "C" guards in older versions, so g++ would
 * otherwise mangle the names and the linker can't find them in libmbe.so
 * (which exports plain C symbols). Wrap the include. */
extern "C" {
#include <mbelib.h>
}
#include "imbe_vocoder.h"
#include <cstdint>
#include <cstring>

namespace {

/* IMBE 88-bit packing layout. Offsets/widths sum to 88. */
constexpr int B_OFFSETS[8] = {  0,  7, 13, 21, 29, 40, 51, 62 };
constexpr int B_WIDTHS[8]  = {  7,  6,  8,  8, 11, 11, 11, 26 };

/* Read `count` bits MSB-first starting at bit offset `start` in a packed
 * byte buffer. */
uint32_t get_bits(const uint8_t *buf, int start, int count) {
    uint32_t v = 0;
    for (int i = 0; i < count; i++) {
        int idx = start + i;
        v = (v << 1) | ((buf[idx / 8] >> (7 - (idx % 8))) & 1);
    }
    return v;
}

/* Write `count` bits of `value` MSB-first into the buffer at bit offset
 * `start`. Assumes buf is pre-zeroed; only sets ones. */
void set_bits(uint8_t *buf, int start, int count, uint32_t value) {
    for (int i = 0; i < count; i++) {
        int idx = start + (count - 1 - i);
        if (value & (1u << i)) {
            buf[idx / 8] |= (uint8_t)(1u << (7 - (idx % 8)));
        }
    }
}

} /* anonymous namespace */

extern "C" {

/* Per-stream codec state. mbelib needs prev-frame parameters across calls
 * because the synthesizer interpolates pitch and energy. The imbe_vocoder
 * instance is similarly stateful (the encoder's pitch tracker carries
 * across frames). */
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
    /* Unpack 11 bytes -> 88 single-bit chars (mbelib's input format). */
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
    /* OP25's imbe_encode takes a non-const PCM pointer (it may pre-emphasize
     * in place). Copy to a local buffer so we don't surprise the caller. */
    int16_t pcm_local[160];
    std::memcpy(pcm_local, pcm_160, sizeof(pcm_local));

    int16_t b_vec[8] = {0};
    ctx->vocoder.imbe_encode(b_vec, pcm_local);

    /* Pack b_vec into 88 bits MSB-first. */
    std::memset(imbe_11b, 0, 11);
    for (int i = 0; i < 8; i++) {
        set_bits(imbe_11b, B_OFFSETS[i], B_WIDTHS[i], (uint32_t)b_vec[i]);
    }
}

} /* extern "C" */
