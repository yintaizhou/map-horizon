#ifndef RSA_CRYPTO_H
#define RSA_CRYPTO_H

#include <string>
#include <iostream>

class RsaCrypt {
public:
    // 生成RSA密钥对
    static void GenerateRSAKey(std::string& out_pub_key, std::string& out_pri_key);

    // 使用公钥加密
    static std::string rsa_pub_encrypt(const std::string& clear_text, const std::string& pub_key);

    // 使用私钥解密
    static std::string rsa_pri_decrypt(const std::string& cipher_text, const std::string& pri_key);
    static std::string base64_encode(const std::string& src, bool url = false);
    static std::string base64_decode(const std::string& src, bool url = false);
    static std::string url_encode(const std::string& src, bool encode_slash = true);
    static std::string url_decode(const std::string& src);

    static int hexchar2int(char c) {
        if (c >= '0' && c <= '9') {
            return c - '0';
        } else if (c >= 'A' && c <= 'Z') {
            return c - 'A' + 10;
        } else if (c >= 'a' && c <= 'z') {
            return c - 'a' + 10;
        }
        return -1;
    }

private:
    static inline unsigned char base64_decode_char(const unsigned char);
    const static char* _s_empty_chars;
    const static char* _s_base64_chars;
    const static char* _s_base64_url_chars;
};

#endif  // RSA_CRYPTO_H
