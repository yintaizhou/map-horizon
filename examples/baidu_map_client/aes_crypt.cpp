#include <random>
#include <vector>
#include <iostream>
#include <openssl/evp.h>
#include <openssl/aes.h>
#include <openssl/rand.h>
#include <openssl/err.h>

#include "aes_crypt.h"

std::string AesCrypt::generate_random_string(const size_t length) {
    std::string chars =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<> distribution(0, chars.size() - 1);

    std::string random_string;
    for (size_t i = 0; i < length; ++i) {
        random_string += chars[distribution(generator)];
    }

    return random_string;
}

std::string AesCrypt::aes_encrypt(const std::string& plaintext, const std::string& key, const std::string& iv) {
    std::vector<unsigned char> ciphertext(plaintext.length() + AES_BLOCK_SIZE);
    int len = 0;
    int ciphertext_len = 0;

    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        std::cout << "EVP_CIPHER_CTX_new failed" << std::endl;
        return "";
    }

    if (1 != EVP_EncryptInit_ex(ctx, EVP_aes_128_cbc(), 
                                nullptr, reinterpret_cast<const unsigned char*>(key.data()), 
                                reinterpret_cast<const unsigned char*>(iv.data()))) {
        EVP_CIPHER_CTX_free(ctx);
        std::cout << "EVP_EncryptInit_ex failed" << std::endl;
        return "";
    }

    if (1 != EVP_EncryptUpdate(ctx, ciphertext.data(), 
                                &len, reinterpret_cast<const unsigned char*>(plaintext.data()), 
                                plaintext.length())) {
        EVP_CIPHER_CTX_free(ctx);
        std::cout << "EVP_EncryptUpdate failed" << std::endl;
        return "";
    }
    ciphertext_len = len;

    if (1 != EVP_EncryptFinal_ex(ctx, ciphertext.data() + len, &len)) {
        EVP_CIPHER_CTX_free(ctx);
        std::cout << "EVP_EncryptFinal_ex failed" << std::endl;
        return "";
    }
    ciphertext_len += len;

    EVP_CIPHER_CTX_free(ctx);

    return std::string(ciphertext.begin(), ciphertext.begin() + ciphertext_len);

}

std::string AesCrypt::aes_decrypt(const std::string& ciphertext, const std::string& key, const std::string& iv) {
    std::vector<unsigned char> plaintext(ciphertext.length());
    int len = 0;
    int plaintext_len = 0;

    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        std::cout << "EVP_CIPHER_CTX_new failed" << std::endl;
        return "";
    }

    if (1 != EVP_DecryptInit_ex(ctx, EVP_aes_128_cbc(), 
                                nullptr, reinterpret_cast<const unsigned char*>(key.data()), 
                                reinterpret_cast<const unsigned char*>(iv.data()))) {
        EVP_CIPHER_CTX_free(ctx);
        std::cout << "EVP_DecryptInit_ex failed" << std::endl;
        return "";
    }

    if (1 != EVP_DecryptUpdate(ctx, plaintext.data(), 
                            &len, reinterpret_cast<const unsigned char*>(ciphertext.data()), 
                            ciphertext.length())) {
        EVP_CIPHER_CTX_free(ctx);
        std::cout << "EVP_DecryptUpdate failed" << std::endl;
        return "";
    }
    plaintext_len = len;

    if (1 != EVP_DecryptFinal_ex(ctx, plaintext.data() + len, &len)) {
        EVP_CIPHER_CTX_free(ctx);
        std::cout << "EVP_DecryptFinal_ex failed" << std::endl;
        return "";
    }
    plaintext_len += len;

    EVP_CIPHER_CTX_free(ctx);

    return std::string(plaintext.begin(), plaintext.begin() + plaintext_len);

}