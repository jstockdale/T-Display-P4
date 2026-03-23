/**
 * meshtastic_crypto.h — AES-CTR encryption/decryption for Meshtastic channel PSK.
 *
 * Nonce layout from CryptoEngine.cpp:253-261:
 *   [0..7]   packetId    (uint64_t LE — upper 32 bits are always 0 for current firmware)
 *   [8..11]  fromNode    (uint32_t LE)
 *   [12..15] blockCounter (starts at 0, incremented per 16-byte AES block)
 *
 * AES-CTR works identically for encrypt and decrypt (XOR with keystream).
 *
 * Part of meshtastic-lite.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ─── Key Structures ────────────────────────────────────────────────────────────

struct MeshCryptoKey {
    uint8_t bytes[32];
    int8_t  length;     // 0 = no encryption, 16 = AES-128, 32 = AES-256, -1 = invalid
};

// ─── Nonce Construction ────────────────────────────────────────────────────────

/**
 * Build the 16-byte nonce/IV for AES-CTR.
 * Matches CryptoEngine::initNonce() exactly.
 */
static inline void meshBuildNonce(uint8_t nonce[16], uint32_t fromNode, uint32_t packetId) {
    memset(nonce, 0, 16);
    // packetId as uint64_t LE in bytes [0..7]
    uint64_t pid64 = (uint64_t)packetId;
    memcpy(nonce, &pid64, sizeof(uint64_t));
    // fromNode as uint32_t LE in bytes [8..11]
    memcpy(nonce + 8, &fromNode, sizeof(uint32_t));
    // bytes [12..15] = block counter, starts at 0 (handled by CTR mode)
}

// ─── Platform-specific AES-CTR ─────────────────────────────────────────────────
//
// We provide two implementations:
//   1. ESP-IDF mbedtls (hardware-accelerated on ESP32-P4)
//   2. Portable software fallback using a tiny AES implementation
//
// Define MESH_CRYPTO_USE_MBEDTLS=1 to use mbedtls (recommended on ESP-IDF).
// Otherwise a software AES-CTR is used.

#if defined(MESH_CRYPTO_USE_MBEDTLS) && MESH_CRYPTO_USE_MBEDTLS

#include "mbedtls/aes.h"

/**
 * AES-CTR encrypt/decrypt in-place.  Uses ESP-IDF mbedtls with HW acceleration.
 */
static inline bool meshCryptCtr(const MeshCryptoKey *key,
                                 uint8_t nonce[16],
                                 uint8_t *data, size_t len)
{
    if (key->length <= 0 || len == 0) return true; // no-op

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);

    int ret = mbedtls_aes_setkey_enc(&ctx, key->bytes, key->length * 8);
    if (ret != 0) {
        mbedtls_aes_free(&ctx);
        return false;
    }

    // mbedtls CTR mode needs a stream block and offset counter
    uint8_t stream_block[16] = {0};
    size_t nc_off = 0;
    uint8_t nonce_counter[16];
    memcpy(nonce_counter, nonce, 16);

    ret = mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce_counter, stream_block, data, data);

    mbedtls_aes_free(&ctx);
    return ret == 0;
}

#else // Software AES-CTR fallback

// Minimal AES implementation for non-ESP platforms.
// You can replace this with your own AES block cipher.
// For now, we declare the interface and expect the user to link an AES impl.

#ifdef __cplusplus
extern "C" {
#endif

/**
 * External AES block encrypt function.
 * Must encrypt a single 16-byte block: out = AES_encrypt(key, in).
 * key_bits is 128 or 256.
 */
extern void mesh_aes_block_encrypt(const uint8_t *key, int key_bits,
                                    const uint8_t in[16], uint8_t out[16]);

#ifdef __cplusplus
}
#endif

/**
 * Software AES-CTR encrypt/decrypt in-place.
 */
static inline bool meshCryptCtr(const MeshCryptoKey *key,
                                 uint8_t nonce[16],
                                 uint8_t *data, size_t len)
{
    if (key->length <= 0 || len == 0) return true;

    int key_bits = key->length * 8;
    uint8_t counter_block[16];
    uint8_t keystream[16];
    memcpy(counter_block, nonce, 16);

    for (size_t offset = 0; offset < len; offset += 16) {
        mesh_aes_block_encrypt(key->bytes, key_bits, counter_block, keystream);

        size_t block_len = (len - offset < 16) ? (len - offset) : 16;
        for (size_t i = 0; i < block_len; i++) {
            data[offset + i] ^= keystream[i];
        }

        // Increment the 32-bit counter in bytes [12..15] (little-endian)
        for (int i = 12; i < 16; i++) {
            if (++counter_block[i] != 0) break;
        }
    }
    return true;
}

#endif // MESH_CRYPTO_USE_MBEDTLS

// ─── Convenience Wrappers ──────────────────────────────────────────────────────

/**
 * Decrypt a Meshtastic payload in-place.
 * `data` points to the encrypted payload (after the 16-byte header).
 */
static inline bool meshDecrypt(const MeshCryptoKey *key,
                                uint32_t fromNode, uint32_t packetId,
                                uint8_t *data, size_t len)
{
    uint8_t nonce[16];
    meshBuildNonce(nonce, fromNode, packetId);
    return meshCryptCtr(key, nonce, data, len);
}

/**
 * Encrypt a Meshtastic payload in-place. (CTR is symmetric.)
 */
static inline bool meshEncrypt(const MeshCryptoKey *key,
                                uint32_t fromNode, uint32_t packetId,
                                uint8_t *data, size_t len)
{
    return meshDecrypt(key, fromNode, packetId, data, len);
}

// ═══════════════════════════════════════════════════════════════════════════════
// PKI Direct Message Crypto (X25519 + SHA256 + XChaCha20-Poly1305)
//
// Meshtastic 2.3+ uses PKI for encrypted DMs between specific nodes.
// Protocol:
//   1. shared = X25519(our_private, sender_public)     — ECDH key agreement
//   2. key    = SHA256(shared)                          — KDF
//   3. nonce  = [packetId_LE(8), fromNode_LE(4), zeros(12)]  — 24 bytes
//   4. plaintext = XChaCha20-Poly1305-Decrypt(key, nonce, ciphertext)
//      ciphertext has a 16-byte Poly1305 tag appended.
//
// XChaCha20-Poly1305 is NOT in mbedtls, so we build it from:
//   HChaCha20(key, nonce[0..15]) → subkey     [implemented below, ~40 lines]
//   ChaCha20-Poly1305(subkey, sub_nonce, ..)   [self-contained impl below]
// where sub_nonce = [0x00, 0x00, 0x00, 0x00, nonce[16..23]] (12 bytes)
// ═══════════════════════════════════════════════════════════════════════════════

#if defined(MESH_CRYPTO_USE_MBEDTLS) && MESH_CRYPTO_USE_MBEDTLS

#include "mbedtls/sha256.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"

// ─── ChaCha20 / Poly1305 / XChaCha20-Poly1305 (self-contained) ─────────────
//
// ESP-IDF 5.4.1 doesn't expose mbedtls ChaCha20-Poly1305 via Kconfig,
// so we implement it directly. The quarter-round is shared with HChaCha20.
//
// References:
//   RFC 8439 — ChaCha20 and Poly1305 for IETF Protocols
//   draft-irtf-cfrg-xchacha — XChaCha20-Poly1305

#define MESH_ROTL32(v, n) (((v) << (n)) | ((v) >> (32 - (n))))

#define MESH_QR(a, b, c, d) \
    a += b; d ^= a; d = MESH_ROTL32(d, 16); \
    c += d; b ^= c; b = MESH_ROTL32(b, 12); \
    a += b; d ^= a; d = MESH_ROTL32(d, 8);  \
    c += d; b ^= c; b = MESH_ROTL32(b, 7);

static inline uint32_t mesh_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void mesh_store_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v);      p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// ─── ChaCha20 Block Function (RFC 8439 §2.3) ────────────────────────────────

static inline void meshChaCha20Block(const uint32_t input[16], uint8_t out[64]) {
    uint32_t s[16];
    memcpy(s, input, sizeof(s));

    for (int i = 0; i < 10; i++) {
        MESH_QR(s[0], s[4], s[8],  s[12]);
        MESH_QR(s[1], s[5], s[9],  s[13]);
        MESH_QR(s[2], s[6], s[10], s[14]);
        MESH_QR(s[3], s[7], s[11], s[15]);
        MESH_QR(s[0], s[5], s[10], s[15]);
        MESH_QR(s[1], s[6], s[11], s[12]);
        MESH_QR(s[2], s[7], s[8],  s[13]);
        MESH_QR(s[3], s[4], s[9],  s[14]);
    }

    for (int i = 0; i < 16; i++)
        mesh_store_le32(out + i * 4, s[i] + input[i]);
}

// ─── ChaCha20 Stream Cipher (RFC 8439 §2.4) ────────────────────────────────

static inline void meshChaCha20Crypt(const uint8_t key[32], const uint8_t nonce[12],
                                      uint32_t counter,
                                      const uint8_t *in, size_t len, uint8_t *out)
{
    uint32_t state[16];
    state[0] = 0x61707865; state[1] = 0x3320646e;
    state[2] = 0x79622d32; state[3] = 0x6b206574;
    for (int i = 0; i < 8; i++) state[4 + i] = mesh_le32(key + i * 4);
    state[12] = counter;
    state[13] = mesh_le32(nonce + 0);
    state[14] = mesh_le32(nonce + 4);
    state[15] = mesh_le32(nonce + 8);

    uint8_t block[64];
    size_t off = 0;
    while (off < len) {
        meshChaCha20Block(state, block);
        size_t chunk = (len - off < 64) ? (len - off) : 64;
        for (size_t i = 0; i < chunk; i++)
            out[off + i] = in[off + i] ^ block[i];
        off += chunk;
        state[12]++;  // increment counter
    }
}

// ─── HChaCha20 (draft-irtf-cfrg-xchacha §2.2) ─────────────────────────────

static inline void meshHChaCha20(const uint8_t key[32], const uint8_t nonce[16],
                                  uint8_t out[32])
{
    uint32_t s[16];
    s[0]  = 0x61707865; s[1]  = 0x3320646e;
    s[2]  = 0x79622d32; s[3]  = 0x6b206574;
    for (int i = 0; i < 8; i++) s[4 + i] = mesh_le32(key + i * 4);
    s[12] = mesh_le32(nonce + 0);  s[13] = mesh_le32(nonce + 4);
    s[14] = mesh_le32(nonce + 8);  s[15] = mesh_le32(nonce + 12);

    for (int i = 0; i < 10; i++) {
        MESH_QR(s[0], s[4], s[8],  s[12]);
        MESH_QR(s[1], s[5], s[9],  s[13]);
        MESH_QR(s[2], s[6], s[10], s[14]);
        MESH_QR(s[3], s[7], s[11], s[15]);
        MESH_QR(s[0], s[5], s[10], s[15]);
        MESH_QR(s[1], s[6], s[11], s[12]);
        MESH_QR(s[2], s[7], s[8],  s[13]);
        MESH_QR(s[3], s[4], s[9],  s[14]);
    }

    mesh_store_le32(out + 0,  s[0]);  mesh_store_le32(out + 4,  s[1]);
    mesh_store_le32(out + 8,  s[2]);  mesh_store_le32(out + 12, s[3]);
    mesh_store_le32(out + 16, s[12]); mesh_store_le32(out + 20, s[13]);
    mesh_store_le32(out + 24, s[14]); mesh_store_le32(out + 28, s[15]);
}

// ─── Poly1305 MAC (RFC 8439 §2.5) ──────────────────────────────────────────
//
// 130-bit arithmetic using 5 × 26-bit limbs. Follows djb's reference closely.

struct MeshPoly1305 {
    uint32_t r[5];    // clamped key (r)
    uint32_t h[5];    // accumulator
    uint32_t pad[4];  // one-time pad (s)
    uint8_t  buf[16]; // partial block buffer
    size_t   buf_len;

    void init(const uint8_t key[32]) {
        // r = key[0..15], clamped per RFC 8439 §2.5.2
        uint32_t t0 = mesh_le32(key + 0);
        uint32_t t1 = mesh_le32(key + 4);
        uint32_t t2 = mesh_le32(key + 8);
        uint32_t t3 = mesh_le32(key + 12);

        r[0] = (t0) & 0x3ffffff;
        r[1] = ((t0 >> 26) | (t1 << 6)) & 0x3ffff03;
        r[2] = ((t1 >> 20) | (t2 << 12)) & 0x3ffc0ff;
        r[3] = ((t2 >> 14) | (t3 << 18)) & 0x3f03fff;
        r[4] = (t3 >> 8) & 0x00fffff;

        h[0] = h[1] = h[2] = h[3] = h[4] = 0;

        pad[0] = mesh_le32(key + 16);
        pad[1] = mesh_le32(key + 20);
        pad[2] = mesh_le32(key + 24);
        pad[3] = mesh_le32(key + 28);

        buf_len = 0;
    }

    void blocks(const uint8_t *data, size_t nblocks, uint32_t hibit) {
        uint32_t r0 = r[0], r1 = r[1], r2 = r[2], r3 = r[3], r4 = r[4];
        uint32_t s1 = r1 * 5, s2 = r2 * 5, s3 = r3 * 5, s4 = r4 * 5;
        uint32_t h0 = h[0], h1 = h[1], h2 = h[2], h3 = h[3], h4 = h[4];

        for (size_t i = 0; i < nblocks; i++, data += 16) {
            // h += m[i]
            uint32_t t0 = mesh_le32(data + 0);
            uint32_t t1 = mesh_le32(data + 4);
            uint32_t t2 = mesh_le32(data + 8);
            uint32_t t3 = mesh_le32(data + 12);

            h0 += (t0) & 0x3ffffff;
            h1 += ((t0 >> 26) | (t1 << 6)) & 0x3ffffff;
            h2 += ((t1 >> 20) | (t2 << 12)) & 0x3ffffff;
            h3 += ((t2 >> 14) | (t3 << 18)) & 0x3ffffff;
            h4 += (t3 >> 8) | (hibit << 24);

            // h *= r (mod 2^130 - 5)
            uint64_t d0 = (uint64_t)h0*r0 + (uint64_t)h1*s4 + (uint64_t)h2*s3 + (uint64_t)h3*s2 + (uint64_t)h4*s1;
            uint64_t d1 = (uint64_t)h0*r1 + (uint64_t)h1*r0 + (uint64_t)h2*s4 + (uint64_t)h3*s3 + (uint64_t)h4*s2;
            uint64_t d2 = (uint64_t)h0*r2 + (uint64_t)h1*r1 + (uint64_t)h2*r0 + (uint64_t)h3*s4 + (uint64_t)h4*s3;
            uint64_t d3 = (uint64_t)h0*r3 + (uint64_t)h1*r2 + (uint64_t)h2*r1 + (uint64_t)h3*r0 + (uint64_t)h4*s4;
            uint64_t d4 = (uint64_t)h0*r4 + (uint64_t)h1*r3 + (uint64_t)h2*r2 + (uint64_t)h3*r1 + (uint64_t)h4*r0;

            // Partial reduction
            uint32_t c;
            c = (uint32_t)(d0 >> 26); h0 = (uint32_t)d0 & 0x3ffffff; d1 += c;
            c = (uint32_t)(d1 >> 26); h1 = (uint32_t)d1 & 0x3ffffff; d2 += c;
            c = (uint32_t)(d2 >> 26); h2 = (uint32_t)d2 & 0x3ffffff; d3 += c;
            c = (uint32_t)(d3 >> 26); h3 = (uint32_t)d3 & 0x3ffffff; d4 += c;
            c = (uint32_t)(d4 >> 26); h4 = (uint32_t)d4 & 0x3ffffff; h0 += c * 5;
            c = h0 >> 26; h0 &= 0x3ffffff; h1 += c;
        }

        h[0] = h0; h[1] = h1; h[2] = h2; h[3] = h3; h[4] = h4;
    }

    void update(const uint8_t *data, size_t len) {
        // Handle partial buffer
        if (buf_len > 0) {
            size_t want = 16 - buf_len;
            if (len < want) { memcpy(buf + buf_len, data, len); buf_len += len; return; }
            memcpy(buf + buf_len, data, want);
            blocks(buf, 1, 1);
            data += want; len -= want; buf_len = 0;
        }
        // Full blocks
        if (len >= 16) {
            size_t nblocks = len / 16;
            blocks(data, nblocks, 1);
            data += nblocks * 16;
            len -= nblocks * 16;
        }
        // Remainder
        if (len > 0) { memcpy(buf, data, len); buf_len = len; }
    }

    void finish(uint8_t tag[16]) {
        // Process final partial block (if any) with hibit=0
        if (buf_len > 0) {
            buf[buf_len] = 1;
            memset(buf + buf_len + 1, 0, 16 - buf_len - 1);
            blocks(buf, 1, 0);
        }

        // Full carry chain
        uint32_t h0 = h[0], h1 = h[1], h2 = h[2], h3 = h[3], h4 = h[4];
        uint32_t c;
        c = h1 >> 26; h1 &= 0x3ffffff; h2 += c;
        c = h2 >> 26; h2 &= 0x3ffffff; h3 += c;
        c = h3 >> 26; h3 &= 0x3ffffff; h4 += c;
        c = h4 >> 26; h4 &= 0x3ffffff; h0 += c * 5;
        c = h0 >> 26; h0 &= 0x3ffffff; h1 += c;

        // Compute h - p = h - (2^130 - 5)
        uint32_t g0 = h0 + 5; c = g0 >> 26; g0 &= 0x3ffffff;
        uint32_t g1 = h1 + c; c = g1 >> 26; g1 &= 0x3ffffff;
        uint32_t g2 = h2 + c; c = g2 >> 26; g2 &= 0x3ffffff;
        uint32_t g3 = h3 + c; c = g3 >> 26; g3 &= 0x3ffffff;
        uint32_t g4 = h4 + c - (1UL << 26);

        // Select h or g based on carry
        uint32_t mask = (g4 >> 31) - 1;  // 0 if g4 < 0, 0xFFFFFFFF if g4 >= 0
        g0 &= mask; g1 &= mask; g2 &= mask; g3 &= mask; g4 &= mask;
        mask = ~mask;
        h0 = (h0 & mask) | g0; h1 = (h1 & mask) | g1;
        h2 = (h2 & mask) | g2; h3 = (h3 & mask) | g3;
        h4 = (h4 & mask) | g4;

        // h = h % (2^128) + pad
        uint64_t f;
        f = (uint64_t)(h0 | (h1 << 26)) + pad[0];          mesh_store_le32(tag + 0,  (uint32_t)f);
        f = (uint64_t)(((h1 >> 6) | (h2 << 20))) + pad[1] + (f >> 32); mesh_store_le32(tag + 4,  (uint32_t)f);
        f = (uint64_t)(((h2 >> 12) | (h3 << 14))) + pad[2] + (f >> 32); mesh_store_le32(tag + 8,  (uint32_t)f);
        f = (uint64_t)(((h3 >> 18) | (h4 << 8))) + pad[3] + (f >> 32); mesh_store_le32(tag + 12, (uint32_t)f);
    }
};

// ─── ChaCha20-Poly1305 AEAD (RFC 8439 §2.8) ────────────────────────────────

static inline void mesh_pad16(MeshPoly1305 *mac, size_t len) {
    if (len % 16 != 0) {
        uint8_t zeros[16] = {0};
        mac->update(zeros, 16 - (len % 16));
    }
}

static inline void mesh_store_le64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++) { p[i] = (uint8_t)(v >> (i * 8)); }
}

#define MESH_XCHACHA_TAG_LEN 16

/**
 * Decrypt + authenticate using XChaCha20-Poly1305-IETF.
 * Self-contained — no mbedtls chachapoly dependency.
 */
static inline bool meshXChaCha20Poly1305Decrypt(
    const uint8_t key[32], const uint8_t nonce[24],
    const uint8_t *ct, size_t ct_len,
    const uint8_t *aad, size_t aad_len,
    uint8_t *out)
{
    if (ct_len < MESH_XCHACHA_TAG_LEN) return false;

    // Step 1: HChaCha20(key, nonce[0..15]) → subkey
    uint8_t subkey[32];
    meshHChaCha20(key, nonce, subkey);

    // Step 2: Build 12-byte sub-nonce = [0x00000000, nonce[16..23]]
    uint8_t sub_nonce[12];
    memset(sub_nonce, 0, 4);
    memcpy(sub_nonce + 4, nonce + 16, 8);

    // Step 3: Generate Poly1305 one-time key (ChaCha20 block 0)
    uint8_t poly_key[64];
    uint8_t zeros[64] = {0};
    meshChaCha20Crypt(subkey, sub_nonce, 0, zeros, 64, poly_key);

    // Step 4: Separate ciphertext body and tag
    size_t body_len = ct_len - MESH_XCHACHA_TAG_LEN;
    const uint8_t *tag = ct + body_len;

    // Step 5: Verify Poly1305 tag over (AAD || pad || ciphertext || pad || lengths)
    MeshPoly1305 mac;
    mac.init(poly_key);
    if (aad && aad_len > 0) { mac.update(aad, aad_len); mesh_pad16(&mac, aad_len); }
    mac.update(ct, body_len);
    mesh_pad16(&mac, body_len);
    uint8_t lens[16];
    mesh_store_le64(lens, aad_len);
    mesh_store_le64(lens + 8, body_len);
    mac.update(lens, 16);

    uint8_t computed_tag[16];
    mac.finish(computed_tag);

    // Constant-time tag comparison
    uint8_t diff = 0;
    for (int i = 0; i < 16; i++) diff |= computed_tag[i] ^ tag[i];

    // Wipe poly key
    memset(poly_key, 0, sizeof(poly_key));

    if (diff != 0) {
        memset(subkey, 0, sizeof(subkey));
        return false;  // authentication failed
    }

    // Step 6: Decrypt (ChaCha20 starting at counter 1)
    meshChaCha20Crypt(subkey, sub_nonce, 1, ct, body_len, out);

    // Wipe key material
    memset(subkey, 0, sizeof(subkey));

    return true;
}

// ─── X25519 ECDH Shared Secret ──────────────────────────────────────────────

/**
 * Compute X25519 ECDH shared secret.
 * shared_out = X25519(our_private, their_public)
 *
 * All mbedtls allocations are temporary and freed before return.
 * Returns true on success.
 */
static inline bool meshX25519SharedSecret(
    const uint8_t our_private[32],
    const uint8_t their_public[32],
    uint8_t shared_out[32])
{
    mbedtls_ecp_group grp;
    mbedtls_mpi d;       // our private scalar
    mbedtls_ecp_point Q; // their public point
    mbedtls_mpi z;       // shared secret X coordinate

    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&z);

    int ret;

    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519);
    if (ret != 0) goto done;

    // Import our private key (32-byte LE)
    ret = mbedtls_mpi_read_binary_le(&d, our_private, 32);
    if (ret != 0) goto done;

    // Import their public key (32-byte LE X coordinate)
    // For Curve25519, ecp_point_read_binary reads raw 32-byte X coordinate
    ret = mbedtls_ecp_point_read_binary(&grp, &Q, their_public, 32);
    if (ret != 0) goto done;

    // Compute shared secret: z = ECDH(d, Q) → X coordinate of d * Q
    ret = mbedtls_ecdh_compute_shared(&grp, &z, &Q, &d, NULL, NULL);
    if (ret != 0) goto done;

    // Export as 32-byte LE
    ret = mbedtls_mpi_write_binary_le(&z, shared_out, 32);

done:
    mbedtls_mpi_free(&z);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
    mbedtls_ecp_group_free(&grp);
    return ret == 0;
}

// ─── SHA-256 ──────────────────────────────────────────────────────────────────

/**
 * SHA-256 hash. Uses mbedtls (HW-accelerated on ESP32-P4).
 */
static inline bool meshSha256(const uint8_t *data, size_t len, uint8_t out[32]) {
    return mbedtls_sha256(data, len, out, 0) == 0;  // 0 = SHA-256 (not SHA-224)
}

// ─── Meshtastic PKI Decrypt ─────────────────────────────────────────────────

/**
 * Build 24-byte PKI nonce from packetId and fromNode.
 * Layout: [packetId_LE(8), fromNode_LE(4), zeros(12)]
 */
static inline void meshBuildPkiNonce(uint8_t nonce[24], uint32_t fromNode, uint32_t packetId) {
    memset(nonce, 0, 24);
    uint64_t pid64 = (uint64_t)packetId;
    memcpy(nonce, &pid64, sizeof(uint64_t));
    memcpy(nonce + 8, &fromNode, sizeof(uint32_t));
}

/**
 * Decrypt a PKI-encrypted Meshtastic DM payload.
 *
 * @param our_private    our X25519 private key (32 bytes)
 * @param sender_public  sender's X25519 public key (32 bytes, from their NODEINFO)
 * @param fromNode       sender's NodeNum (for nonce)
 * @param packetId       packet ID (for nonce)
 * @param ct             ciphertext payload (from after 16-byte packet header)
 * @param ct_len         length of ciphertext (includes 16-byte Poly1305 tag)
 * @param out            plaintext output buffer (must be >= ct_len - 16 bytes)
 * @param out_len        receives plaintext length on success
 * @return true if decryption and authentication succeeded
 */
static inline bool meshDecryptPki(
    const uint8_t our_private[32],
    const uint8_t sender_public[32],
    uint32_t fromNode, uint32_t packetId,
    const uint8_t *ct, size_t ct_len,
    uint8_t *out, size_t *out_len)
{
    if (ct_len < MESH_XCHACHA_TAG_LEN) return false;

    // 1. X25519 ECDH key agreement
    uint8_t shared[32];
    if (!meshX25519SharedSecret(our_private, sender_public, shared)) {
        memset(shared, 0, sizeof(shared));
        return false;
    }

    // 2. KDF: SHA256(shared secret)
    uint8_t key[32];
    if (!meshSha256(shared, 32, key)) {
        memset(shared, 0, sizeof(shared));
        memset(key, 0, sizeof(key));
        return false;
    }
    memset(shared, 0, sizeof(shared));  // wipe raw shared secret

    // 3. Build 24-byte nonce
    uint8_t nonce[24];
    meshBuildPkiNonce(nonce, fromNode, packetId);

    // 4. XChaCha20-Poly1305 decrypt + authenticate
    bool ok = meshXChaCha20Poly1305Decrypt(key, nonce, ct, ct_len, NULL, 0, out);

    // Wipe key material from stack
    memset(key, 0, sizeof(key));
    memset(nonce, 0, sizeof(nonce));

    if (ok && out_len) {
        *out_len = ct_len - MESH_XCHACHA_TAG_LEN;
    }
    return ok;
}

#endif // MESH_CRYPTO_USE_MBEDTLS
