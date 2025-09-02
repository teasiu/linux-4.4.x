#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
#ifndef hydrogen_H
#define hydrogen_H

#ifndef __KERNEL__
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#else
#include <linux/kernel.h>
#include <linux/string.h>
typedef uint16_t uint_fast16_t;
#define abort() panic("error")
#endif

#ifdef __cplusplus
#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wlong-long"
#endif
extern "C" {
#endif

#ifndef __GNUC__
#ifdef __attribute__
#undef __attribute__
#endif
#define __attribute__(a)
#endif

#define HYDRO_VERSION_MAJOR 1
#define HYDRO_VERSION_MINOR 0

int hydro_init(void);

/* ---------------- */

#define randombytes_SEEDBYTES 32

uint32_t randombytes_random(void);

uint32_t randombytes_uniform(const uint32_t upper_bound);

void randombytes_buf(void *out, size_t out_len);

void randombytes_buf_deterministic(void *out, size_t out_len,
                                   const uint8_t seed[randombytes_SEEDBYTES]);

/* ---------------- */

#define hydro_hash_BYTES 32
#define hydro_hash_BYTES_MAX 65535
#define hydro_hash_BYTES_MIN 16
#define hydro_hash_CONTEXTBYTES 8
#define hydro_hash_KEYBYTES 32
#define hydro_hash_KEYBYTES_MAX 32
#define hydro_hash_KEYBYTES_MIN 16

typedef struct hydro_hash_state {
    uint32_t state[12];
    uint8_t  buf_off;
    uint8_t  align[3];
} hydro_hash_state;

void hydro_hash_keygen(uint8_t *key, size_t key_len);

int hydro_hash_init(hydro_hash_state *state,
                    const char ctx[hydro_hash_CONTEXTBYTES], const uint8_t *key,
                    size_t key_len);

int hydro_hash_update(hydro_hash_state *state, const void *in_, size_t in_len);

int hydro_hash_final(hydro_hash_state *state, uint8_t *out, size_t out_len);

int hydro_hash_hash(uint8_t *out, size_t out_len, const void *in_,
                    size_t in_len, const char ctx[hydro_hash_CONTEXTBYTES],
                    const uint8_t *key, size_t key_len);

/* ---------------- */

#define hydro_secretbox_CONTEXTBYTES 8
#define hydro_secretbox_HEADERBYTES (20 + 16)
#define hydro_secretbox_KEYBYTES 32
#define hydro_secretbox_PROBEBYTES 16

void hydro_secretbox_keygen(uint8_t key[hydro_secretbox_KEYBYTES]);

int hydro_secretbox_encrypt(uint8_t *c, const void *m_, size_t mlen,
                            uint64_t      msg_id,
                            const char    ctx[hydro_secretbox_CONTEXTBYTES],
                            const uint8_t key[hydro_secretbox_KEYBYTES]);

int hydro_secretbox_decrypt(void *m_, const uint8_t *c, size_t clen,
                            uint64_t      msg_id,
                            const char    ctx[hydro_secretbox_CONTEXTBYTES],
                            const uint8_t key[hydro_secretbox_KEYBYTES])
    __attribute__((warn_unused_result));

void
hydro_secretbox_probe_create(uint8_t probe[hydro_secretbox_PROBEBYTES],
                             const uint8_t *c, size_t c_len,
                             const char    ctx[hydro_secretbox_CONTEXTBYTES],
                             const uint8_t key[hydro_secretbox_KEYBYTES]);

int
hydro_secretbox_probe_verify(const uint8_t probe[hydro_secretbox_PROBEBYTES],
                             const uint8_t *c, size_t c_len,
                             const char    ctx[hydro_secretbox_CONTEXTBYTES],
                             const uint8_t key[hydro_secretbox_KEYBYTES])
            __attribute__((warn_unused_result));

/* ---------------- */

#define hydro_kdf_CONTEXTBYTES 8
#define hydro_kdf_KEYBYTES 32
#define hydro_kdf_BYTES_MAX 65535
#define hydro_kdf_BYTES_MIN 16

void hydro_kdf_keygen(uint8_t key[hydro_kdf_KEYBYTES]);

int hydro_kdf_derive_from_key(uint8_t *subkey, size_t subkey_len,
                              uint64_t      subkey_id,
                              const char    ctx[hydro_kdf_CONTEXTBYTES],
                              const uint8_t key[hydro_kdf_KEYBYTES]);

/* ---------------- */

#define hydro_sign_BYTES 64
#define hydro_sign_CONTEXTBYTES 8
#define hydro_sign_PUBLICKEYBYTES 32
#define hydro_sign_SECRETKEYBYTES 64
#define hydro_sign_SEEDBYTES 32

typedef struct hydro_sign_state {
    hydro_hash_state hash_st;
} hydro_sign_state;

typedef struct hydro_sign_keypair {
    uint8_t pk[hydro_sign_PUBLICKEYBYTES];
    uint8_t sk[hydro_sign_SECRETKEYBYTES];
} hydro_sign_keypair;

void hydro_sign_keygen(hydro_sign_keypair *kp);

void hydro_sign_keygen_deterministic(hydro_sign_keypair *kp,
                                     const uint8_t seed[hydro_sign_SEEDBYTES]);

int hydro_sign_init(hydro_sign_state *state,
                    const char        ctx[hydro_sign_CONTEXTBYTES]);

int hydro_sign_update(hydro_sign_state *state, const void *m_, size_t mlen);

int hydro_sign_final_create(hydro_sign_state *state,
                            uint8_t           csig[hydro_sign_BYTES],
                            const uint8_t     sk[hydro_sign_SECRETKEYBYTES]);

int hydro_sign_final_verify(hydro_sign_state *state,
                            const uint8_t     csig[hydro_sign_BYTES],
                            const uint8_t     pk[hydro_sign_PUBLICKEYBYTES])
    __attribute__((warn_unused_result));

int hydro_sign_create(uint8_t csig[hydro_sign_BYTES], const void *m_,
                      size_t mlen, const char ctx[hydro_sign_CONTEXTBYTES],
                      const uint8_t sk[hydro_sign_SECRETKEYBYTES]);

int hydro_sign_verify(const uint8_t csig[hydro_sign_BYTES], const void *m_,
                      size_t mlen, const char ctx[hydro_sign_CONTEXTBYTES],
                      const uint8_t pk[hydro_sign_PUBLICKEYBYTES])
    __attribute__((warn_unused_result));

/* ---------------- */

#define hydro_kx_SESSIONKEYBYTES 32
#define hydro_kx_PUBLICKEYBYTES 32
#define hydro_kx_SECRETKEYBYTES 32
#define hydro_kx_PSKBYTES 32
#define hydro_kx_SEEDBYTES 32

#define hydro_kx_RESPONSE1BYTES 32
#define hydro_kx_RESPONSE2BYTES 80
#define hydro_kx_RESPONSE3BYTES 48

typedef struct hydro_kx_keypair {
    uint8_t pk[hydro_kx_PUBLICKEYBYTES];
    uint8_t sk[hydro_kx_SECRETKEYBYTES];
} hydro_kx_keypair;

typedef struct hydro_kx_session_keypair {
    uint8_t rx[hydro_kx_SESSIONKEYBYTES];
    uint8_t tx[hydro_kx_SESSIONKEYBYTES];
} hydro_kx_session_keypair;

typedef struct hydro_kx_state {
    hydro_kx_keypair eph_kp;
    uint8_t          h[32];
    uint8_t          ck[32];
    uint8_t          k[32];
} hydro_kx_state;

void hydro_kx_keygen(hydro_kx_keypair *static_kp);

void hydro_kx_keygen_deterministic(hydro_kx_keypair *static_kp,
                                   const uint8_t     seed[hydro_kx_SEEDBYTES]);

int hydro_kx_xx_1(hydro_kx_state *state,
                  uint8_t         response1[hydro_kx_RESPONSE1BYTES],
                  const uint8_t   psk[hydro_kx_PSKBYTES]);

int hydro_kx_xx_2(hydro_kx_state *        state,
                  uint8_t                 response2[hydro_kx_RESPONSE2BYTES],
                  const uint8_t           response1[hydro_kx_RESPONSE1BYTES],
                  const uint8_t           psk[hydro_kx_PSKBYTES],
                  const hydro_kx_keypair *static_kp);

int hydro_kx_xx_3(hydro_kx_state *state, hydro_kx_session_keypair *kp,
                  uint8_t       response3[hydro_kx_RESPONSE3BYTES],
                  uint8_t       peer_static_pk[hydro_kx_PUBLICKEYBYTES],
                  const uint8_t response2[hydro_kx_RESPONSE2BYTES],
                  const uint8_t psk[hydro_kx_PSKBYTES],
                  const hydro_kx_keypair *static_kp);

int hydro_kx_xx_4(hydro_kx_state *state, hydro_kx_session_keypair *kp,
                  uint8_t       peer_static_pk[hydro_kx_PUBLICKEYBYTES],
                  const uint8_t response3[hydro_kx_RESPONSE3BYTES],
                  const uint8_t psk[hydro_kx_PSKBYTES]);

/* ---------------- */

void hydro_memzero(void *pnt, size_t len);

void hydro_increment(uint8_t *n, size_t len);

bool hydro_equal(const void *b1_, const void *b2_, size_t len);

int hydro_compare(const uint8_t *b1_, const uint8_t *b2_, size_t len);

char *hydro_bin2hex(char *hex, size_t hex_maxlen, const uint8_t *bin,
                    size_t bin_len);

int hydro_hex2bin(uint8_t *bin, size_t bin_maxlen, const char *hex,
                  size_t hex_len, const char *ignore, size_t *bin_len,
                  const char **hex_end);

int hydro_pad(size_t *padded_buflen_p, unsigned char *buf,
	      size_t unpadded_buflen, size_t blocksize, size_t max_buflen);

int hydro_unpad(size_t *unpadded_buflen_p, const unsigned char *buf,
		size_t padded_buflen, size_t blocksize);

/* ---------------- */

#define HYDRO_HWTYPE_ATMEGA328 1

#ifndef HYDRO_HWTYPE
#ifdef __AVR__
#define HYDRO_HWTYPE HYDRO_HWTYPE_ATMEGA328
#endif
#endif

#ifdef __cplusplus
}
#endif


#endif
