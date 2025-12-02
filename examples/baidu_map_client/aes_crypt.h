#pragma once
#include <iostream>
#include <string>
class AesCrypt {
public:
    static std::string generate_random_string(const size_t length = 16);

    static std::string aes_encrypt(const std::string& clear_text, const std::string& key, const std::string& iv);

    static std::string aes_decrypt(const std::string& cipher_text, const std::string& key, const std::string& iv);
};
