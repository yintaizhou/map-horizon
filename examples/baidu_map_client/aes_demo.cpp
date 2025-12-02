#include "aes_crypt.h"
#include <iostream>
#include <string>
#include <iomanip>
#include <vector> // Added for different data length tests

int main() {
    std::cout << "=== AES加密解密Demo ===" << std::endl;

    // 1. 生成随机密钥和IV
    std::cout << "\n1. 生成随机密钥和IV..." << std::endl;
    std::string key = AesCrypt::generate_random_string(16);  // AES-128需要16字节密钥
    std::string iv = AesCrypt::generate_random_string(16);   // AES-CBC需要16字节IV
    
    std::cout << "密钥长度: " << key.length() << " 字节" << std::endl;
    std::cout << "IV长度: " << iv.length() << " 字节" << std::endl;
    
    // 显示密钥和IV的十六进制表示
    std::cout << "密钥(Hex): ";
    for (char c : key) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(static_cast<unsigned char>(c)) << " ";
    }
    std::cout << std::dec << std::endl;
    
    std::cout << "IV(Hex): ";
    for (char c : iv) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(static_cast<unsigned char>(c)) << " ";
    }
    std::cout << std::dec << std::endl;

    // 2. 测试数据
    std::string plainText = "Hello, AES加密解密测试! 123456";
    std::cout << "\n2. 原始数据: " << plainText << std::endl;
    std::cout << "数据长度: " << plainText.length() << " 字节" << std::endl;

    // 3. 使用AES加密
    std::cout << "\n3. 使用AES-128-CBC加密..." << std::endl;
    std::string encryptedData = AesCrypt::aes_encrypt(plainText, key, iv);
    if (encryptedData.empty()) {
        std::cout << "❌ AES加密失败！" << std::endl;
        return -1;
    }
    std::cout << "加密后长度: " << encryptedData.length() << " 字节" << std::endl;
    
    // 显示加密数据的十六进制表示
    std::cout << "加密数据(Hex): ";
    for (size_t i = 0; i < std::min(encryptedData.length(), size_t(32)); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(static_cast<unsigned char>(encryptedData[i])) << " ";
    }
    if (encryptedData.length() > 32) {
        std::cout << "...";
    }
    std::cout << std::dec << std::endl;

    // 4. 使用AES解密
    std::cout << "\n4. 使用AES-128-CBC解密..." << std::endl;
    std::string decryptedData = AesCrypt::aes_decrypt(encryptedData, key, iv);
    if (decryptedData.empty()) {
        std::cout << "❌ AES解密失败！" << std::endl;
        return -1;
    }
    std::cout << "解密后数据: " << decryptedData << std::endl;

    // 5. 验证结果
    std::cout << "\n5. 验证结果..." << std::endl;
    if (plainText == decryptedData) {
        std::cout << "✅ 加密解密成功！原始数据和解密后数据完全一致。" << std::endl;
    } else {
        std::cout << "❌ 加密解密失败！原始数据和解密后数据不一致。" << std::endl;
        std::cout << "原始数据: [" << plainText << "]" << std::endl;
        std::cout << "解密数据: [" << decryptedData << "]" << std::endl;
    }

    // 6. 性能测试
    std::cout << "\n6. 性能测试..." << std::endl;
    std::string testData = "这是一个用于性能测试的数据，包含中文字符和English characters!";
    std::cout << "测试数据长度: " << testData.length() << " 字节" << std::endl;
    
    // 多次加密解密测试
    const int testCount = 1000;
    std::cout << "执行 " << testCount << " 次加密解密测试..." << std::endl;
    
    for (int i = 0; i < testCount; ++i) {
        std::string encrypted = AesCrypt::aes_encrypt(testData, key, iv);
        std::string decrypted = AesCrypt::aes_decrypt(encrypted, key, iv);
        
        if (testData != decrypted) {
            std::cout << "❌ 第 " << (i+1) << " 次测试失败！" << std::endl;
            return -1;
        }
    }
    std::cout << "✅ " << testCount << " 次测试全部通过！" << std::endl;

    // 7. 不同数据长度测试
    std::cout << "\n7. 不同数据长度测试..." << std::endl;
    std::vector<std::string> testStrings;
    testStrings.push_back("");                    // 空字符串
    testStrings.push_back("A");                   // 1字节
    testStrings.push_back("Hello");               // 5字节
    testStrings.push_back("Hello World!");        // 12字节
    testStrings.push_back("这是一个测试字符串");    // 中文字符
    testStrings.push_back("Mixed 中英文 Mixed");   // 混合字符
    testStrings.push_back(std::string(100, 'A')); // 100字节
    testStrings.push_back(std::string(1000, 'B')); // 1000字节
    
    for (size_t i = 0; i < testStrings.size(); ++i) {
        std::string testStr = testStrings[i];
        std::string encrypted = AesCrypt::aes_encrypt(testStr, key, iv);
        std::string decrypted = AesCrypt::aes_decrypt(encrypted, key, iv);
        
        if (testStr == decrypted) {
            std::cout << "✅ 测试 " << (i+1) << " 通过 (长度: " << testStr.length() << " 字节)" << std::endl;
        } else {
            std::cout << "❌ 测试 " << (i+1) << " 失败 (长度: " << testStr.length() << " 字节)" << std::endl;
            return -1;
        }
    }

    std::cout << "\n=== AES Demo完成 ===" << std::endl;
    return 0;
}
