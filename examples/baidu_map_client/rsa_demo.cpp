#include "rsa_crypto.h"
#include <iostream>
#include <string>

int main() {
    std::cout << "=== RSA加密解密Demo ===" << std::endl;

    // 1. 生成RSA密钥对
    std::cout << "\n1. 生成RSA密钥对..." << std::endl;
    std::string publicKey, privateKey;
    RsaCrypt::GenerateRSAKey(publicKey, privateKey);

    // publicKey =
    //         "-----BEGIN PUBLIC KEY-----\n"
    //         "MIIBIDANBgkqhkiG9w0BAQEFAAOCAQ0AMIIBCAKCAQEArXbXuS/AUPE1oIKkWdew\n"
    //         "//G0HD6tub3FQrBF/GhU7MlsH8J2PwOzSBi31S1Yox0Pg2i9/LjVman+QXDTnsTb\n"
    //         "zMjmoe3zgh8j6Cquo/JqVODfCUWanB8bh96zH/L0fJZDdfKZppzqjyr4LjHGl+NB\n"
    //         "TD/O/MAuFhgNYppDJIi9FtP2nu0W96vPY0aqcaZ/KJW78S9/CaxyFLqwXc2g4MaM\n"
    //         "ieY3ofOBAJayrt8BVzpOCEpCVhSOAKwW58rEdxMTmrjmhZIyDs0daG842if8Ksck\n"
    //         "pAOZlmRpyAve8ci3vdIqaPg3ww1+FS16pCABKlrV4c/Zz12DxYrP7ryLiPxi23uR\n"
    //         "4QIBAw==\n"
    //         "-----END PUBLIC KEY-----";
    // privateKey =
    //         "-----BEGIN RSA PRIVATE KEY-----\n"
    //         "MIIEowIBAAKCAQEArXbXuS/AUPE1oIKkWdew//G0HD6tub3FQrBF/GhU7MlsH8J2\n"
    //         "PwOzSBi31S1Yox0Pg2i9/LjVman+QXDTnsTbzMjmoe3zgh8j6Cquo/JqVODfCUWa\n"
    //         "nB8bh96zH/L0fJZDdfKZppzqjyr4LjHGl+NBTD/O/MAuFhgNYppDJIi9FtP2nu0W\n"
    //         "96vPY0aqcaZ/KJW78S9/CaxyFLqwXc2g4MaMieY3ofOBAJayrt8BVzpOCEpCVhSO\n"
    //         "AKwW58rEdxMTmrjmhZIyDs0daG842if8KsckpAOZlmRpyAve8ci3vdIqaPg3ww1+\n"
    //         "FS16pCABKlrV4c/Zz12DxYrP7ryLiPxi23uR4QIBAwKCAQBzpI/QyoA19iPAVxg7\n"
    //         "5SCqoSK9fx5709jXIC6oRY3zMPK/1vl/V8zauyU4yOXCE1+s8H6oezkRG/7WSze/\n"
    //         "Lefd20Rr8/esFMKaxx8X9vGN6z9bg7xoFL0FPyIVTKL9uYJOobvEaJxfcfrJdoRl\n"
    //         "QiuIKon91XQOurOXEYIYWyi54YsuMfAvUarfau8eny/Pqzkm454NO13dYjwu2Dux\n"
    //         "TQxEbEAGoNv+DagsQzc7Z2lIeH81aV7hZ8ZAuYoa+RrUTnROTHsS+2BcBicnisAZ\n"
    //         "jbKX1JxlDqr0JaSXO2n+gjaAvgDJN3iKqwa5/xntojTY/RF8I+aYsQS0rExje3rO\n"
    //         "tNoDAoGBANwCn75HZKq4Awda2B2MS0FT0SvS7pTWywJs9Lr408nh9vxPQKMH0/cM\n"
    //         "DuXQyaK1ZkvjFzmnk3Z5W1a8c9++suffbgFhEdSCWC2iRckCxeeMEZJrkl5QhXSx\n"
    //         "9luZCIM7Tvro+TxWO946POV9fVRkt6oYpiFtO1vOCW7k5zsH85snAoGBAMnXAnBp\n"
    //         "TKRcIzxo35nkyaTjCOYi5NBx3FOqlIxd/zBBTNtIwZP7rj9ga0itVY1mLzei30YG\n"
    //         "/PcNWR6SKYudkFAwsXgRQjhcDdB8khMBrLE0MxyTPAsJTfNc+T0nBlVsjDusQQzu\n"
    //         "8RJS5HTIajI3nJV8gYhzRe0RsKoO2++deK+3AoGBAJKsan7aQxx6rK+R5WkIMiuN\n"
    //         "Nh03SbiPMgGd+Hyl4oaWpKg01cIFN/oICe6LMRcjmYfsuiZvt6RQ548oTT/Ud0U/\n"
    //         "nquWC+MBkB5sLoYB2UUIC7byYZQ1rk3L+ZJmBazSNKdF+32O0pQm00Oo/jhDJRwQ\n"
    //         "bsDzfOfesPSYmidaomdvAoGBAIaPVvWbiG2SwihF6maYhm3ssJlsmIr2kuJxuF2T\n"
    //         "/3WA3eeF1mKnyX+VnNseOQjuyiUXP4QEqKSzkL8MG7JpCuAgdlALgXroCTWoYWIB\n"
    //         "Hct4IhMM0rIGM/eTUNNvWY5IXX0dgLNJ9gw3QviFnCF6aGOoVlr3g/NhIHFfPUpo\n"
    //         "+x/PAoGBAL4zjo8vld1oWh9qP6t9qi0GDaH0NtwCs3NgA07vomcIQYnRieKNkPP0\n"
    //         "w6Uj8yeshGrTLmDC81yrk8uOQtbzIb6YIaYke4XEwfX2Pqxv0dETl4UH8NqqyPN/\n"
    //         "a1uAxevQKkSiQ7mHNoxoBK2g7A73AioL6FIz77/uJZH1MDkx4+Q8\n"
    //         "-----END RSA PRIVATE KEY-----";
    std::cout << "公钥信息: " << std::endl << publicKey << std::endl;
    std::cout << "公钥长度: " << publicKey.length() << " 字符" << std::endl;

    std::cout << "私钥信息: " << std::endl << privateKey << std::endl;
    std::cout << "私钥长度: " << privateKey.length() << " 字符" << std::endl;

    // 2. 测试数据 //服务器会用公钥把这些数据加密
    std::string plainText = R"({"ack":1,"aes_iv":"cdJsk1n8skUqqmMI","aes_key":"gjYro3BWe2mWF5Am","message":"ok","status":0})";
    std::cout << "\n2. 原始数据: " << plainText << std::endl;
    std::cout << "数据长度: " << plainText.length() << " 字节" << std::endl;

    // 3. 使用公钥加密
    std::cout << "\n3. 使用公钥加密..." << std::endl;
    std::string encryptedData = RsaCrypt::rsa_pub_encrypt(plainText, publicKey);
    std::cout << "加密后长度: " << encryptedData.length() << " 字节" << std::endl;

    // 4. Base64编码加密数据（用于传输）
    std::cout << "\n4. Base64编码加密数据..." << std::endl;
    std::string base64Encrypted = RsaCrypt::base64_encode(encryptedData);
    std::cout << "Base64编码后: " << base64Encrypted << std::endl;

    // std::string fuwuqixiafadeshuju = "HcEudxzPrpjSwceJeRULO0aIJvf43BUvdZe2DRisQQgeejNRAV1R0jc/Ry/ReXSDF/GUMbXIEZWRbpVhSRVrf4um852z89DF15xc7Vsr5HP46ChMc8f7gMqxgYMKkiYrsY+jyEAieVxeKqdR+qhSoYp1RlX+AC9JBa54qMJ1h4FF7svk9Vs5Uq42VZyCKqz3Ej9YJaws2Hrzlpdite/JrGC+5ImGYzraPYA2igmukd9pglQvJqweCALR/O8cuWIb6vjZ3Cs6RduVdO7fYiZLDjf9n6Sr528bgbj+/tlvoAja1X1RjUr+AWBqRgVKl1EB0AceempaO5qHV5pCcUxjfQ==";
    // 5. Base64解码
    std::cout << "\n5. Base64解码..." << std::endl;
    std::string base64Decoded = RsaCrypt::base64_decode(base64Encrypted);
    std::cout << "Base64解码后长度: " << base64Decoded.length() << " 字节" << std::endl;
    std::cout << "Base64解码后: " << base64Decoded << std::endl;

    // 6. 使用私钥解密
    std::cout << "\n6. 使用私钥解密..." << std::endl;
    std::string decryptedData = RsaCrypt::rsa_pri_decrypt(base64Decoded, privateKey);
    std::cout << "解密后数据: " << decryptedData << std::endl;

    // 7. 验证结果
    std::cout << "\n7. 验证结果..." << std::endl;
    if (plainText == decryptedData) {
        std::cout << "✅ 加密解密成功！原始数据和解密后数据完全一致。" << std::endl;
    } else {
        std::cout << "❌ 加密解密失败！原始数据和解密后数据不一致。" << std::endl;
    }
    std::cout << "\n=== Demo完成 ===" << std::endl;

    return 0;
}