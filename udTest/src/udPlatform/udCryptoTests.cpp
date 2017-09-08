#include "gtest/gtest.h"
#include "udCrypto.h"
#include "udPlatformUtil.h"

TEST(CryptoTests, AES_CBC_MonteCarlo)
{
  // Do the first only monte carlo tests for CBC mode (400 tests in official monte carlo)
  static const unsigned char aes_test_cbc_dec[2][16] =
  {
    { 0xFA, 0xCA, 0x37, 0xE0, 0xB0, 0xC8, 0x53, 0x73, 0xDF, 0x70, 0x6E, 0x73, 0xF7, 0xC9, 0xAF, 0x86 },
    { 0x48, 0x04, 0xE1, 0x81, 0x8F, 0xE6, 0x29, 0x75, 0x19, 0xA3, 0xE8, 0x8C, 0x57, 0x31, 0x04, 0x13 }
  };

  static const unsigned char aes_test_cbc_enc[2][16] =
  {
    { 0x8A, 0x05, 0xFC, 0x5E, 0x09, 0x5A, 0xF4, 0x84, 0x8A, 0x08, 0xD3, 0x28, 0xD3, 0x68, 0x8E, 0x3D },
    { 0xFE, 0x3C, 0x53, 0x65, 0x3E, 0x2F, 0x45, 0xB5, 0x6F, 0xCD, 0x88, 0xB2, 0xCC, 0x89, 0x8F, 0xF0 }
  };

  unsigned char key[32];
  unsigned char buf[64];
  unsigned char iv[16];
  unsigned char prv[16];
  udCryptoCipherContext *pCtx = nullptr;

  for (int i = 0; i < 4; i++)
  {
    int test256 = i >> 1;
    int testEncrypt = i & 1;

    memset(key, 0, sizeof(key));
    memset(iv, 0, sizeof(iv));
    memset(prv, 0, sizeof(prv));
    memset(buf, 0, sizeof(buf));

    udResult result = udCrypto_CreateCipher(&pCtx, test256 ? udCC_AES256 : udCC_AES128, udCPM_None, key, udCCM_CBC);
    EXPECT_EQ(udR_Success, result);

    if (!testEncrypt)
    {
      for (int j = 0; j < 10000; j++)
        udCrypto_Decrypt(pCtx, iv, sizeof(iv), buf, 16, buf, sizeof(buf), nullptr, iv); // Note: specifically decrypting exactly 16 bytes, not sizeof(buf)

      EXPECT_EQ(0, memcmp(buf, aes_test_cbc_dec[test256], 16));
    }
    else
    {
      for (int j = 0; j < 10000; j++)
      {
        udCrypto_Encrypt(pCtx, iv, sizeof(iv), buf, 16, buf, sizeof(buf), nullptr, iv); // Note: specifically encrypting exactly 16 bytes, not sizeof(buf)
        unsigned char tmp[16];
        memcpy(tmp, prv, 16);
        memcpy(prv, buf, 16);
        memcpy(buf, tmp, 16);
      }

      EXPECT_EQ(0, memcmp(prv, aes_test_cbc_enc[test256], 16));
    }
    EXPECT_EQ(udR_Success, udCrypto_DestroyCipher(&pCtx));
  }
}

TEST(CryptoTests, AES_CTR_MonteCarlo)
{
  // Do the first only monte carlo tests for CTR mode (400 tests in official monte carlo)
  static const unsigned char aes_test_ctr_key[3][16] =
  {
    { 0xAE, 0x68, 0x52, 0xF8, 0x12, 0x10, 0x67, 0xCC, 0x4B, 0xF7, 0xA5, 0x76, 0x55, 0x77, 0xF3, 0x9E },
    { 0x7E, 0x24, 0x06, 0x78, 0x17, 0xFA, 0xE0, 0xD7, 0x43, 0xD6, 0xCE, 0x1F, 0x32, 0x53, 0x91, 0x63 },
    { 0x76, 0x91, 0xBE, 0x03, 0x5E, 0x50, 0x20, 0xA8, 0xAC, 0x6E, 0x61, 0x85, 0x29, 0xF9, 0xA0, 0xDC }
  };

  static const unsigned char aes_test_ctr_nonce_counter[3][16] =
  {
    { 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 },
    { 0x00, 0x6C, 0xB6, 0xDB, 0xC0, 0x54, 0x3B, 0x59, 0xDA, 0x48, 0xD9, 0x0B, 0x00, 0x00, 0x00, 0x01 },
    { 0x00, 0xE0, 0x01, 0x7B, 0x27, 0x77, 0x7F, 0x3F, 0x4A, 0x17, 0x86, 0xF0, 0x00, 0x00, 0x00, 0x01 }
  };

  static const unsigned char aes_test_ctr_pt[3][48] =
  {
    { 0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62, 0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67 },

    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F },

    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
      0x20, 0x21, 0x22, 0x23 }
  };

  static const unsigned char aes_test_ctr_ct[3][48] =
  {
    { 0xE4, 0x09, 0x5D, 0x4F, 0xB7, 0xA7, 0xB3, 0x79, 0x2D, 0x61, 0x75, 0xA3, 0x26, 0x13, 0x11, 0xB8 },

    { 0x51, 0x04, 0xA1, 0x06, 0x16, 0x8A, 0x72, 0xD9, 0x79, 0x0D, 0x41, 0xEE, 0x8E, 0xDA, 0xD3, 0x88,
      0xEB, 0x2E, 0x1E, 0xFC, 0x46, 0xDA, 0x57, 0xC8, 0xFC, 0xE6, 0x30, 0xDF, 0x91, 0x41, 0xBE, 0x28 },

    { 0xC1, 0xCF, 0x48, 0xA8, 0x9F, 0x2F, 0xFD, 0xD9, 0xCF, 0x46, 0x52, 0xE9, 0xEF, 0xDB, 0x72, 0xD7,
      0x45, 0x40, 0xA4, 0x2B, 0xDE, 0x6D, 0x78, 0x36, 0xD5, 0x9A, 0x5C, 0xEA, 0xAE, 0xF3, 0x10, 0x53,
      0x25, 0xB2, 0x07, 0x2F }
  };

  static const int aes_test_ctr_len[3] = { 16, 32, 36 };

  unsigned char key[32];
  unsigned char buf[64];
  unsigned char nonce_counter[16];
  udCryptoCipherContext *pCtx = nullptr;

  for (int i = 0; i < 4; i++)
  {
    int testNumber = i >> 1;
    int testEncrypt = i & 1;

    memset(key, 0, sizeof(key));
    memset(buf, 0, sizeof(buf));
    memcpy(nonce_counter, aes_test_ctr_nonce_counter[testNumber], 16);
    memcpy(key, aes_test_ctr_key[testNumber], 16);

    udResult result = udCrypto_CreateCipher(&pCtx, udCC_AES128, udCPM_None, key, udCCM_CTR); // We only test 128-bit for CTR mode
    EXPECT_EQ(udR_Success, result);

    if (!testEncrypt)
    {
      int len = aes_test_ctr_len[testNumber];
      memcpy(buf, aes_test_ctr_ct[testNumber], len);
      udCrypto_Decrypt(pCtx, aes_test_ctr_nonce_counter[testNumber], 16, buf, len, buf, len);
      EXPECT_EQ(0, memcmp(buf, aes_test_ctr_pt[testNumber], len));
    }
    else
    {
      int len = aes_test_ctr_len[testNumber];
      memcpy(buf, aes_test_ctr_pt[testNumber], len);
      udCrypto_Decrypt(pCtx, aes_test_ctr_nonce_counter[testNumber], 16, buf, len, buf, len);
      EXPECT_EQ(0, memcmp(buf, aes_test_ctr_ct[testNumber], len));
    }
    EXPECT_EQ(udR_Success, udCrypto_DestroyCipher(&pCtx));
  }
}

TEST(CryptoTests, CipherErrorCodes)
{
  udResult result;
  unsigned char key[32];
  udCryptoCipherContext *pCtx = nullptr;
  unsigned char buf[64];
  unsigned char iv[16];

  memset(iv, 0, sizeof(iv));
  memset(key, 0, sizeof(key));
  memset(buf, 0, sizeof(buf));
  result = udCrypto_CreateCipher(nullptr, udCC_AES128, udCPM_None, key, udCCM_CTR);
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_CreateCipher(&pCtx, udCC_AES128, udCPM_None, nullptr, udCCM_CTR);
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_CreateCipher(&pCtx, udCC_AES128, udCPM_None, key, udCCM_CTR);
  EXPECT_EQ(udR_Success, result);

  result = udCrypto_Encrypt(nullptr, iv, 16, buf, sizeof(buf), buf, sizeof(buf)); // context
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Encrypt(pCtx, nullptr, 16, buf, sizeof(buf), buf, sizeof(buf)); // iv
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Encrypt(pCtx, iv, 15, buf, sizeof(buf), buf, sizeof(buf)); // iv length
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Encrypt(pCtx, iv, 16, nullptr, sizeof(buf), buf, sizeof(buf)); // input null
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Encrypt(pCtx, iv, 16, buf, 1, buf, sizeof(buf)); // input alignment
  EXPECT_EQ(udR_AlignmentRequirement, result);
  result = udCrypto_Encrypt(pCtx, iv, 16, buf, sizeof(buf), nullptr, sizeof(buf)); // output null
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Encrypt(pCtx, iv, 16, buf, sizeof(buf), buf, sizeof(buf) - 1); // output size
  EXPECT_EQ(udR_BufferTooSmall, result);

  result = udCrypto_Decrypt(nullptr, iv, 16, buf, sizeof(buf), buf, sizeof(buf)); // context
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Decrypt(pCtx, nullptr, 16, buf, sizeof(buf), buf, sizeof(buf)); // iv
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Decrypt(pCtx, iv, 15, buf, sizeof(buf), buf, sizeof(buf)); // iv length
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Decrypt(pCtx, iv, 16, nullptr, sizeof(buf), buf, sizeof(buf)); // input null
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Decrypt(pCtx, iv, 16, buf, 1, buf, sizeof(buf)); // input alignment
  EXPECT_EQ(udR_AlignmentRequirement, result);
  result = udCrypto_Decrypt(pCtx, iv, 16, buf, sizeof(buf), nullptr, sizeof(buf)); // output null
  EXPECT_EQ(udR_InvalidParameter_, result);
  result = udCrypto_Decrypt(pCtx, iv, 16, buf, sizeof(buf), buf, sizeof(buf) - 1); // output size
  EXPECT_EQ(udR_BufferTooSmall, result);

  result = udCrypto_DestroyCipher(&pCtx);
  EXPECT_EQ(udR_Success, result);
}


TEST(CryptoTests, SHA)
{
  /*
  * FIPS-180-1 test vectors. See https://www.di-mgt.com.au/sha_testvectors.html
  */
  static const char* s_testMessages[3] =
  {
    "abc",
    "",
    "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
  };

  static udCryptoHashes s_testHashes[2] = { udCH_SHA1, udCH_SHA256 };
  static const unsigned char s_testHashSizes[2] { udCHL_SHA1Length, udCHL_SHA256Length };
  static const unsigned char s_testHashResults[2][3][udCHL_MaxHashLength] =
  {
    { // SHA-1
      { 0xA9, 0x99, 0x3E, 0x36, 0x47, 0x06, 0x81, 0x6A, 0xBA, 0x3E, 0x25, 0x71, 0x78, 0x50, 0xC2, 0x6C, 0x9C, 0xD0, 0xD8, 0x9D },
      { 0xDA, 0x39, 0xA3, 0xEE, 0x5E, 0x6B, 0x4B, 0x0D, 0x32, 0x55, 0xBF, 0xEF, 0x95, 0x60, 0x18, 0x90, 0xAF, 0xD8, 0x07, 0x09 },
      { 0x84, 0x98, 0x3E, 0x44, 0x1C, 0x3B, 0xD2, 0x6E, 0xBA, 0xAE, 0x4A, 0xA1, 0xF9, 0x51, 0x29, 0xE5, 0xE5, 0x46, 0x70, 0xF1 },
    },
    { // SHA-256
      { 0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA, 0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23, 0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C, 0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD },
      { 0xE3, 0xB0, 0xC4, 0x42, 0x98, 0xFC, 0x1C, 0x14, 0x9A, 0xFB, 0xF4, 0xC8, 0x99, 0x6F, 0xB9, 0x24, 0x27, 0xAE, 0x41, 0xE4, 0x64, 0x9B, 0x93, 0x4C, 0xA4, 0x95, 0x99, 0x1B, 0x78, 0x52, 0xB8, 0x55 },
      { 0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8, 0xE5, 0xC0, 0x26, 0x93, 0x0C, 0x3E, 0x60, 0x39, 0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67, 0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1 }
    },
  };

  for (int shaType = 0; shaType < UDARRAYSIZE(s_testHashes); ++shaType)
  {
    for (int testNumber = 0; testNumber < UDARRAYSIZE(s_testMessages); ++testNumber)
    {
      unsigned char resultHash[udCHL_MaxHashLength];
      size_t actualHashSize;
      size_t inputLength = udStrlen(s_testMessages[testNumber]);

      memset(resultHash, 0, sizeof(resultHash));
      udResult result = udCrypto_Hash(s_testHashes[shaType], s_testMessages[testNumber], inputLength, resultHash, sizeof(resultHash), &actualHashSize);
      EXPECT_EQ(udR_Success, result);
      EXPECT_EQ(s_testHashSizes[shaType], actualHashSize);
      EXPECT_EQ(0, memcmp(resultHash, s_testHashResults[shaType][testNumber], actualHashSize));

      // Do an additional test of digesting the string in two separate parts
      size_t length1 = inputLength / 2;
      size_t length2 = inputLength - length1;
      memset(resultHash, 0, sizeof(resultHash));
      result = udCrypto_Hash(s_testHashes[shaType], s_testMessages[testNumber], length1, resultHash, sizeof(resultHash), &actualHashSize, s_testMessages[testNumber] + length1, length2);
      EXPECT_EQ(udR_Success, result);
      EXPECT_EQ(s_testHashSizes[shaType], actualHashSize);
      EXPECT_EQ(0, memcmp(resultHash, s_testHashResults[shaType][testNumber], actualHashSize));
    }
  }

}

TEST(CryptoTests, Self)
{
  EXPECT_EQ(udR_Success, udCrypto_TestCipher(udCC_AES256));
  EXPECT_EQ(udR_Success, udCrypto_TestHash(udCH_SHA1));
  EXPECT_EQ(udR_Success, udCrypto_TestHash(udCH_SHA256));
}
