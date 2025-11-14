/* wolfssl_user_settings.h
 *
 * Copyright (C) 2014-2024 wolfSSL Inc.
 *
 * This file is part of wolfSSH.
 *
 * wolfSSH is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * wolfSSH is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with wolfSSH.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef WOLFSSL_USER_SETTINGS_H
#define WOLFSSL_USER_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#undef  LS_HASH
#define LS_HASH

#ifdef CONFIG_SOC_LSQSH
#undef  LS_HASH_SHA512
#define LS_HASH_SHA512
#endif

#undef LS_CRYPT
#define LS_CRYPT

#undef LS_SM4
#define LS_SM4

#undef HAVE_AES_ECB
#define HAVE_AES_ECB

#undef  NO_OLD_SHA_NAMES
#define NO_OLD_SHA_NAMES
#undef  NO_OLD_WC_NAMES
#define NO_OLD_WC_NAMES

#undef  NO_WOLFSSL_CLIENT
#define NO_WOLFSSL_CLIENT

#undef  WOLFSSL_NO_CLIENT_AUTH
#define WOLFSSL_NO_CLIENT_AUTH

#undef  WOLFSSL_NOSHA512_256
#define WOLFSSL_NOSHA512_256

#undef  WOLFSSL_NOSHA512_224
#define WOLFSSL_NOSHA512_224


#define NO_HMAC
#define WOLFSSL_AEAD_ONLY
#define NO_RSA
#define NO_DH

#undef  WOLFSSL_SM3
#define WOLFSSL_SM3

#undef  WOLFSSL_SM4
#define WOLFSSL_SM4

#undef  WOLFSSL_SM4_ECB
#define WOLFSSL_SM4_ECB

#undef  WOLFSSL_SM4_CTR
#define WOLFSSL_SM4_CTR

#undef  WOLFSSL_ZEPHYR
#define WOLFSSL_ZEPHYR

#undef WOLFSSL_USER_MUTEX
#define WOLFSSL_USER_MUTEX

#undef  TFM_TIMING_RESISTANT
#define TFM_TIMING_RESISTANT

#undef  ECC_TIMING_RESISTANT
#define ECC_TIMING_RESISTANT

#undef  WC_RSA_BLINDING
#define WC_RSA_BLINDING

#undef  HAVE_AESGCM
// #define HAVE_AESGCM

#undef  WOLFSSL_SHA512
#define WOLFSSL_SHA512

#undef  WOLFSSL_SHA384
#define WOLFSSL_SHA384

#undef  NO_DSA
#define NO_DSA

#undef  HAVE_ECC
#define HAVE_ECC

#undef  TFM_ECC256
#define TFM_ECC256

#undef  WOLFSSL_BASE64_ENCODE
#define WOLFSSL_BASE64_ENCODE

#undef  WOLFSSL_BASE16
#define WOLFSSL_BASE16

#undef  NO_RC4
#define NO_RC4

#undef  WOLFSSL_SHA224
#define WOLFSSL_SHA224

#undef  WOLFSSL_SHA3
#define WOLFSSL_SHA3

#undef  HAVE_POLY1305
#define HAVE_POLY1305

#undef  HAVE_ONE_TIME_AUTH
#define HAVE_ONE_TIME_AUTH

#undef  HAVE_CHACHA
#define HAVE_CHACHA

#undef  HAVE_HASHDRBG
#define HAVE_HASHDRBG

#undef  HAVE_TLS_EXTENSIONS
#define HAVE_TLS_EXTENSIONS

#undef  HAVE_SUPPORTED_CURVES
#define HAVE_SUPPORTED_CURVES

#undef  HAVE_EXTENDED_MASTER
#define HAVE_EXTENDED_MASTER

#undef  NO_PSK
#define NO_PSK

#undef  NO_MD4
#define NO_MD4

#undef  NO_PWDBASED
#define NO_PWDBASED

#undef  USE_FAST_MATH
#define USE_FAST_MATH

#undef  WOLFSSL_NO_ASM
#define WOLFSSL_NO_ASM

#undef  WOLFSSL_X86_BUILD
#define WOLFSSL_X86_BUILD

#undef  WC_NO_ASYNC_THREADING
#define WC_NO_ASYNC_THREADING

#undef  NO_DES3
#define NO_DES3

#undef  WOLFSSL_TLS13
#define WOLFSSL_TLS13

#undef  HAVE_HKDF
#define HAVE_HKDF

#undef  WC_RSA_PSS
#define WC_RSA_PSS

#undef  HAVE_FFDHE_2048
#define HAVE_FFDHE_2048

#undef NO_FILESYSTEM
#define NO_FILESYSTEM

#undef  WOLFSSL_STATIC_MEMORY
#define WOLFSSL_STATIC_MEMORY

#undef  WOLFSSL_SMALL_STACK
#define WOLFSSL_SMALL_STACK

#undef WOLFSSL_STATIC_MEMORY_TEST_SZ
#define WOLFSSL_STATIC_MEMORY_TEST_SZ 40960

#undef WOLFSSL_AES_SMALL_TABLES
#define WOLFSSL_AES_SMALL_TABLES

#undef WOLFSSL_TLS13

#undef USE_CERT_BUFFERS_1024
#define USE_CERT_BUFFERS_1024

#undef USE_CERT_BUFFERS_2048

#define WOLFMEM_DEF_BUCKETS 20

#undef WOLFSSL_VALIDATE_ECC_IMPORT
#define  WOLFSSL_VALIDATE_ECC_IMPORT

#undef  WOLFSSL_SM2
#define WOLFSSL_SM2

#define DEBUG_WOLFSSL
#ifdef __cplusplus
}
#endif

#endif /* WOLFSSL_USER_SETTINGS_H */
