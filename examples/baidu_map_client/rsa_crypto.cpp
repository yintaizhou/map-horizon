#include "rsa_crypto.h"
#include <openssl/pem.h>
#include <openssl/rsa.h>
#include <openssl/err.h>
#include <openssl/bio.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <limits.h>
const char* RsaCrypt::_s_empty_chars = " \n\t\r\v\f";
const char* RsaCrypt::_s_base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
// https://datatracker.ietf.org/doc/html/rfc4648#page-7
const char* RsaCrypt::_s_base64_url_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_=";
// RSA密钥长度
#define KEY_LENGTH 2048  // 密钥长度

void RsaCrypt ::GenerateRSAKey(std::string& out_pub_key, std::string& out_pri_key) {
    size_t pri_len = 0;       // 私钥长度
    size_t pub_len = 0;       // 公钥长度
    char* pri_key = nullptr;  // 私钥
    char* pub_key = nullptr;  // 公钥

    // 生成密钥对
    RSA* keypair = RSA_generate_key(KEY_LENGTH, RSA_3, NULL, NULL);

    BIO* pri = BIO_new(BIO_s_mem());
    BIO* pub = BIO_new(BIO_s_mem());

    // 生成私钥
    PEM_write_bio_RSAPrivateKey(pri, keypair, NULL, NULL, 0, NULL, NULL);
    // PEM_write_bio_RSAPublicKey(pub, keypair);
    PEM_write_bio_RSA_PUBKEY(pub, keypair);

    // 获取长度
    pri_len = BIO_pending(pri);
    pub_len = BIO_pending(pub);

    // 密钥对读取到字符串
    pri_key = (char*)malloc(pri_len + 1);
    pub_key = (char*)malloc(pub_len + 1);

    BIO_read(pri, pri_key, pri_len);
    BIO_read(pub, pub_key, pub_len);

    pri_key[pri_len] = '\0';
    pub_key[pub_len] = '\0';

    out_pub_key = pub_key;
    out_pri_key = pri_key;

    // 释放内存
    RSA_free(keypair);
    BIO_free_all(pub);
    BIO_free_all(pri);

    free(pri_key);
    free(pub_key);
}

std::string RsaCrypt::rsa_pub_encrypt(const std::string& clear_text, const std::string& pub_key) {
    std::cout << "clear_text: " << clear_text << std::endl;
    std::cout << "pub_key: " << pub_key << std::endl;
    std::string encrypt_text;
    // 检查并修复公钥格式
    std::string formatted_pub_key = pub_key;
    if (pub_key.find("-----BEGIN PUBLIC KEY-----") == std::string::npos) {
        // 如果没有PEM头，添加PEM格式
        formatted_pub_key = "-----BEGIN PUBLIC KEY-----\n" + pub_key + "\n-----END PUBLIC KEY-----";
    }

    BIO* keybio = BIO_new_mem_buf((unsigned char*)formatted_pub_key.c_str(), -1);
    RSA* rsa = RSA_new();
    // 这种读取公钥的方式 读取的是 裸的公钥。不包含任何额外的其他信息
    // rsa = PEM_read_bio_RSAPublicKey(keybio, &rsa, NULL, NULL);
    // 这种读取公钥的方式 读取的是 X.509结构编码的rsa公钥。更通用的标准
    rsa = PEM_read_bio_RSA_PUBKEY(keybio, &rsa, NULL, NULL);

    // 添加空指针检查
    if (rsa == nullptr) {
        unsigned long err = ERR_get_error();  // 获取错误号
        char err_msg[1024] = {0};
        ERR_error_string(err, err_msg);  // 格式：error:errId:库:函数:原因
        printf("err msg: err:%ld, msg:%s\n", err, err_msg);
        BIO_free_all(keybio);
        return std::string();
    }

    // 获取RSA单次可以处理的数据块的最大长度
    int key_len = RSA_size(rsa);
    int block_len = key_len - 11;  // 因为填充方式为RSA_PKCS1_PADDING, 所以要在key_len基础上减去11

    // 申请内存：存贮加密后的密文数据
    char* sub_text = new char[key_len + 1];
    memset(sub_text, 0, key_len + 1);
    int ret = 0;
    size_t pos = 0;
    std::string sub_str;

    // 对数据进行分段加密（返回值是加密后数据的长度）
    while (pos < clear_text.length()) {
        sub_str = clear_text.substr(pos, block_len);
        memset(sub_text, 0, key_len + 1);
        ret = RSA_public_encrypt(
                sub_str.length(),
                (const unsigned char*)sub_str.c_str(),
                (unsigned char*)sub_text,
                rsa,
                RSA_PKCS1_PADDING);

        if (ret >= 0) {
            encrypt_text.append(std::string(sub_text, ret));
        }

        pos += block_len;
    }

    // 释放内存
    BIO_free_all(keybio);
    RSA_free(rsa);
    delete[] sub_text;

    return encrypt_text;
}

std::string RsaCrypt::rsa_pri_decrypt(const std::string& cipher_text, const std::string& pri_key) {
    std::string decrypt_text;
    RSA* rsa = RSA_new();
    BIO* keybio = nullptr;

    // 检查并修复私钥格式
    std::string formatted_pri_key = pri_key;
    if (pri_key.find("-----BEGIN RSA PRIVATE KEY-----") == std::string::npos) {
        // 如果没有PEM头，添加PEM格式
        formatted_pri_key =
                "-----BEGIN RSA PRIVATE KEY-----\n" + pri_key + "\n-----END RSA PRIVATE KEY-----";
    }

    keybio = BIO_new_mem_buf((unsigned char*)formatted_pri_key.c_str(), -1);

    rsa = PEM_read_bio_RSAPrivateKey(keybio, &rsa, NULL, NULL);

    if (rsa == nullptr) {
        unsigned long err = ERR_get_error();  // 获取错误号
        char err_msg[1024] = {0};
        ERR_error_string(err, err_msg);  // 格式：error:errId:库:函数:原因
        printf("err msg: err:%ld, msg:%s\n", err, err_msg);
        BIO_free_all(keybio);
        return std::string();
    }

    // 获取RSA单次处理的最大长度
    int key_len = RSA_size(rsa);
    char* sub_text = new char[key_len + 1];
    memset(sub_text, 0, key_len + 1);
    int ret = 0;
    std::string sub_str;
    size_t pos = 0;

    // 对密文进行分段解密
    while (pos < cipher_text.length()) {
        sub_str = cipher_text.substr(pos, key_len);
        memset(sub_text, 0, key_len + 1);
        ret = RSA_private_decrypt(
                sub_str.length(),
                (const unsigned char*)sub_str.c_str(),
                (unsigned char*)sub_text,
                rsa,
                RSA_PKCS1_PADDING);

        if (ret >= 0) {
            decrypt_text.append(std::string(sub_text, ret));
            pos += key_len;
        }
    }

    // 释放内存
    delete[] sub_text;
    BIO_free_all(keybio);
    RSA_free(rsa);

    return decrypt_text;
}

std::string RsaCrypt::base64_encode(const std::string& src, bool url) {
    const char* chars = _s_base64_chars;
    if (url) {
        chars = _s_base64_url_chars;
    }
    std::stringstream ss;
    ss.clear();

    size_t length = src.size();
    if (src.empty()) {
        return src;
    }

    unsigned char parts[4];
    for (size_t i = 0; i < length; i += 3) {
        parts[0] = (src[i] & 0xfc) >> 2;
        parts[1] = ((src[i] & 0x03) << 4) | (((length > (i + 1) ? src[i + 1] : 0x00) & 0xf0) >> 4);
        parts[2] = length > (i + 1) ? (((src[i + 1] & 0x0f) << 2) |
                                       (((length > (i + 2) ? src[i + 2] : 0x00) & 0xc0) >> 6))
                                    : 0x40;
        parts[3] = length > (i + 2) ? (src[i + 2] & 0x3f) : 0x40;

        for (int j = 0; j < 4; ++j) {
            ss << chars[parts[j]];
        }
    }
    return ss.str();
}

unsigned char RsaCrypt::base64_decode_char(const unsigned char c) {
    unsigned char result = 0;
    if (isupper(c)) {
        result = static_cast<unsigned char>(c - 'A');
    } else if (islower(c)) {
        result = static_cast<unsigned char>(c - 'a' + 26);
    } else if (isdigit(c)) {
        result = static_cast<unsigned char>(c - '0' + 52);
    } else if ('+' == c || '-' == c) {
        result = 62;
    } else if ('/' == c || '_' == c) {
        result = 63;
    } else if ('=' == c) {
        result = 64;
    } else {
        result = UCHAR_MAX;
    }
    return result;
}

std::string RsaCrypt::base64_decode(const std::string& src, bool url) {
    std::stringstream ss;
    ss.clear();
    size_t length = src.size();
    if (src.empty()) {
        return src;
    }

    unsigned char parts[4];
    for (size_t i = 0; i < length; i += 4) {
        parts[0] = base64_decode_char(src[i]);
        parts[1] = length > (i + 1) ? base64_decode_char(src[i + 1]) : 64;
        parts[2] = length > (i + 2) ? base64_decode_char(src[i + 2]) : 64;
        parts[3] = length > (i + 3) ? base64_decode_char(src[i + 3]) : 64;

        ss << static_cast<unsigned char>(((parts[0] << 2) & 0xfc) | ((parts[1] >> 4) & 0x03));
        if (64 == parts[2]) {
            break;
        }
        ss << static_cast<unsigned char>(((parts[1] << 4) & 0xf0) | ((parts[2] >> 2) & 0x0f));
        if (64 == parts[3]) {
            break;
        }
        ss << static_cast<unsigned char>(((parts[2] << 6) & 0xc0) | (parts[3] & 0x3f));
    }
    return ss.str();
}

std::string RsaCrypt::url_encode(const std::string& src, bool encode_slash) {
    std::ostringstream ss;
    for (size_t i = 0; i < src.size(); ++i) {
        char c = src[i];
        if ((c >= -1 && c <= 255 && isalnum(c)) || c == '_' || c == '-' || c == '~' || c == '.' ||
            (c == '/' && !encode_slash)) {
            ss << c;
        } else {
            ss << '%';
            int tmp = (c >> 4) & 0xf;
            if (tmp >= 10) {
                tmp = tmp - 10 + 'A';
            } else {
                tmp = tmp + '0';
            }
            ss << (char)tmp;
            tmp = c & 0xf;
            if (tmp >= 10) {
                tmp = tmp - 10 + 'A';
            } else {
                tmp = tmp + '0';
            }
            ss << (char)tmp;
        }
    }
    return ss.str();
}

std::string RsaCrypt::url_decode(const std::string& src) {
    std::ostringstream ss;
    for (size_t i = 0; i < src.size();) {
        if (src[i] != '%') {
            ss << src[i++];
        } else {
            ss << (char)((hexchar2int(src[i + 1]) << 4) | hexchar2int(src[i + 2]));
            i += 3;
        }
    }
    return ss.str();
}
