/*
 * AIBridge IMBE codec wrapper.
 *
 * Bridges 11-byte IMBE frames (as produced by MMDVMHost's P25 network
 * protocol) and 8 kHz 16-bit PCM, using mbelib for the decode side.
 *
 * Encode is currently a no-op (returns silent zero frames). When the
 * encoder library is wired in, replace aibridge_imbe_encode below.
 *
 * Built into libaibridge_imbe.so and loaded by imbe_codec.py via ctypes.
 *
 * mbelib API reference: https://github.com/szechyjs/mbelib
 *   mbe_processImbe4400Data(aout, errs, errs2, err_str, imbe_d[88],
 *                           cur_mp, prev_mp, prev_mp_enhanced, uvquality)
 * The "4400" is the IMBE post-FEC information rate (4400 bps).
 */

#include <mbelib.h>
#include <cstdint>
#include <cstring>

extern "C" {

/* Per-stream codec state. mbelib needs prev-frame parameters across calls
 * because the synthesizer interpolates pitch and energy. We allocate a
 * fresh context per Python-side codec instance. */
struct aibridge_imbe_ctx {
    mbe_parms cur_mp;
    mbe_parms prev_mp;
    mbe_parms prev_mp_enhanced;
};

aibridge_imbe_ctx *aibridge_imbe_create(void) {
    auto *ctx = new aibridge_imbe_ctx();
    mbe_initMbeParms(&ctx->cur_mp, &ctx->prev_mp, &ctx->prev_mp_enhanced);
    return ctx;
}

void aibridge_imbe_destroy(aibridge_imbe_ctx *ctx) {
    delete ctx;
}

/* Decode 11 bytes (88 post-FEC IMBE bits) -> 160 int16 PCM samples.
 * Input bits are MSB-first in each byte, matching what MMDVMHost writes
 * into its network packets. */
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

    /* uvquality: 0-255, controls unvoiced-band whitening; 3 is the
     * common DSD/OP25 default for natural-sounding speech.
     * mbe_processImbe4400Data writes 160 int16 samples directly into
     * the output buffer — no float conversion needed. */
    mbe_processImbe4400Data(pcm_160, &errs, &errs2, err_str, imbe_bits,
                            &ctx->cur_mp, &ctx->prev_mp,
                            &ctx->prev_mp_enhanced, 3);
}

/* Encode 160 int16 PCM samples -> 11 bytes IMBE.
 * Not yet implemented. Returns silent IMBE (all zeros) so the TX path
 * keeps running; the radio will produce the same "shhhh" hum we saw with
 * MockCodec until a real encoder is wired in.
 *
 * To finish this: either pull op25's imbe_vocoder encoder and do the
 * b0-b7 -> 88-bit packing per TIA-102.BABA, or swap in a DVSI AMBE-3000
 * dongle. */
void aibridge_imbe_encode(aibridge_imbe_ctx *ctx,
                          const int16_t *pcm_160,
                          uint8_t *imbe_11b) {
    (void)ctx;
    (void)pcm_160;
    std::memset(imbe_11b, 0, 11);
}

} /* extern "C" */
