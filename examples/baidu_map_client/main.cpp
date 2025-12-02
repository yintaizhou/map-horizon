#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <iomanip>
#include <nlohmann/json.hpp>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include "rsa_crypto.h"
#include "aes_crypt.h"

typedef websocketpp::client<websocketpp::config::asio_client> client;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

using json = nlohmann::json;

typedef websocketpp::config::asio_client::message_type::ptr message_ptr;
typedef client::connection_ptr connection_ptr;

const bool JSON_CASE = false;

const std::string AK = "您的AK";
const std::string URI = "ws://api.map.baidu.com/websocket";

// 传入客户端设备对唯一标识
std::string ENTITY_ID = "yintaizhou";

// 执行的case的文件名
// case中为要上报的json数据
std::string CASE = "shanghai_gaosu_cruise_case.jsonl";

// 全局变量：默认不启用加密（1=不加密，2=加密）
int ENCRYPTION_MODE = 1;

// 全局变量：默认不显示解密数据（1=不显示，2=显示）
int SHOW_DECRYPTED = 1;

// @brief 构造一个鉴权用的json字符串
// @param rsa_public_key RSA公钥，如果为空则不使用加密（enc=1），否则使用RSA加密（enc=2）
void get_verify_json(int64_t msgid, std::string& message, const std::string& rsa_public_key = "") {
    // 构造鉴权请求的json字符串
    json j;
    j["id"] = msgid;
    j["qt"] = "auth";
    j["ver"] = "1.0";
    j["prt"] = 1;

    // 根据是否有RSA公钥来决定enc字段
    // enc=1: 不加密
    // enc=2: 使用RSA加密
    int enc_value = 1;
    if (!rsa_public_key.empty()) {
        enc_value = 2;
    }
    j["enc"] = enc_value;
    j["ts"] = time(nullptr);

    json data;
    // 从百度地图开放平台申请的ak，ak为服务端ak，需要开通巡航服务权限
    data["ak"] = AK;
    // entity_id是每个终端对唯一标识，服务端会根据此id建立服务端对ssesion
    // 如果两个终端的entity_id重复，后上线对终端会将之前的踢下线
    data["entity_id"] = ENTITY_ID;

    // 如果提供了RSA公钥，需要对公钥进行urlencode后上传
    if (!rsa_public_key.empty()) {
        data["rsa_puc"] = RsaCrypt::url_encode(rsa_public_key);
    }

    j["data"] = data;

    message = j.dump();
    return;
}

// 异步日志器：避免std::cout阻塞WebSocket IO线程，并减少IO线程内的大payload拷贝
class AsyncLogger {
public:
    AsyncLogger() : _stop(false) {
        _log_thread = std::thread(&AsyncLogger::_log_worker, this);
    }

    ~AsyncLogger() {
        _stop = true;
        _cv.notify_all();
        if (_log_thread.joinable()) {
            _log_thread.join();
        }
    }

    void log_plain(const std::string& message) {
        _enqueue({ "", message, nullptr, false });
    }

    void log_send(const std::string& prefix, const std::string& payload) {
        _enqueue({ prefix, payload, nullptr, true });
    }

    void log_receive(const std::string& prefix, message_ptr msg) {
        _enqueue({ prefix, "", msg, true });
    }

private:
    struct Task {
        std::string prefix;
        std::string text;
        message_ptr ws_msg;
        bool append_newline;
    };

    void _enqueue(Task&& task) {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.push(std::move(task));
        _cv.notify_one();
    }

    void _log_worker() {
        while (!_stop || !_queue.empty()) {
            std::unique_lock<std::mutex> lock(_mutex);
            _cv.wait(lock, [this] { return _stop || !_queue.empty(); });

            while (!_queue.empty()) {
                Task task = std::move(_queue.front());
                _queue.pop();
                lock.unlock();

                std::cout << task.prefix;
                if (task.ws_msg) {
                    std::cout << task.ws_msg->get_payload();
                } else {
                    std::cout << task.text;
                }
                if (task.append_newline) {
                    std::cout << '\n';
                }
                std::cout << std::flush;

                lock.lock();
            }
        }
    }

    std::mutex _mutex;
    std::condition_variable _cv;
    std::queue<Task> _queue;
    std::atomic<bool> _stop;
    std::thread _log_thread;
};

// 全局异步日志器实例
AsyncLogger g_logger;

std::string make_log_prefix(const char* action) {
    auto now = std::chrono::system_clock::now();
    time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << ' ' << action << ' ';
    return oss.str();
}

// 实现一个处理websocket连接的client对象
class WebsocketClient {
public:
    WebsocketClient() {
        _init_flag = false;

        // 只有在加密模式时才生成RSA密钥对
        const int encryption_mode = 2;
        if (ENCRYPTION_MODE == encryption_mode) {
            RsaCrypt::GenerateRSAKey(_rsa_public_key, _rsa_private_key);
        }

        // WebsocketClient初始化
        m_endpoint.set_access_channels(websocketpp::log::alevel::none);
        m_endpoint.set_error_channels(websocketpp::log::elevel::info);

        m_endpoint.init_asio();

        // 注册事件处理的回调
        m_endpoint.set_message_handler(bind(&WebsocketClient::on_message,this,::_1,::_2));
        m_endpoint.set_open_handler(bind(&WebsocketClient::on_open,this,::_1));
        m_endpoint.set_close_handler(bind(&WebsocketClient::on_close,this,::_1));
        m_endpoint.set_fail_handler(bind(&WebsocketClient::on_fail,this,::_1));

    }

    ~WebsocketClient() {}
    
    // @brief 启动websocket客户端
    // @param uri 建立连接的服务端地址
    void start() {
        // 建立连接
        websocketpp::lib::error_code ec;
        client::connection_ptr con = m_endpoint.get_connection(URI, ec);

        // 获取连接，此处不会真正连接服务端，只是获取一个连接实例
        if (ec) {
            std::ostringstream oss;
            oss << "websocket get connection failed: " << ec.message() << "\n";
            g_logger.log_plain(oss.str());
            return;
        }

        con = m_endpoint.connect(con);

        m_hdl = con->get_handle();

        // 在run之前无法判断连接是否成功
        // 如果连接失败，run会退出，否则run会一直阻塞
        // 启动客户端
        m_endpoint.run();

        // run失败了，重置下endpoint用于重连
        m_endpoint.reset();
    }

    // @brief 发送消息到服务端，主要用于上报位置
    // @param message 要发送的json字符串
    void send(std::string message) {
        // 检查连接是否鉴权通过
        if (!_init_flag.load()) {
            g_logger.log_plain("connection not established, ignore this message\n");
            return;
        }

        // 检查连接是否是open状态
        client::connection_ptr con = m_endpoint.get_con_from_hdl(m_hdl);

        if (con->get_state() != websocketpp::session::state::open) {
            std::ostringstream oss;
            oss << "connection is " << con->get_state() << ", ignore this message\n";
            g_logger.log_plain(oss.str());
            return;
        }

        // 如果已经获取了AES密钥，需要对消息进行加密
        std::string final_message = message;
        bool is_encrypted = false;
        if (!_aes_key.empty() && !_aes_iv.empty()) {
            // 解析原始JSON消息
            json j = json::parse(message, /*cb=*/nullptr, /*allow_exceptions=*/false);
            if (j.is_discarded()) {
                g_logger.log_plain("parse message for encryption failed\n");
                return;
            }
            
            // 提取data部分进行AES加密
            if (j.contains("data")) {
                std::string data_str = j["data"].dump();
                
                // Step 1: AES加密
                std::string aes_encrypted = AesCrypt::aes_encrypt(data_str, _aes_key, _aes_iv);
                if (aes_encrypted.empty()) {
                    g_logger.log_plain("AES encrypt failed\n");
                    return;
                }
                
                // Step 2: Base64编码
                std::string base64_encoded = RsaCrypt::base64_encode(aes_encrypted);
                
                // Step 3: 替换data字段为加密后的字符串，并设置enc=3
                j["data"] = base64_encoded;
                j["enc"] = 3;
                
                final_message = j.dump();
                is_encrypted = true;
            }
        }

        ::websocketpp::lib::error_code ec;

        m_endpoint.send(m_hdl, final_message, websocketpp::frame::opcode::text, ec);
        // 打印发送的加密消息
        g_logger.log_send(make_log_prefix("send"), final_message);
        // 如果消息被加密了，且启用了显示解密数据，额外打印解密后的原始消息（仅用于本地日志）
        // 注意：只有当 SHOW_DECRYPTED == 2 时才打印
        if (is_encrypted) {
            if (SHOW_DECRYPTED == 2) {
                std::ostringstream oss;
                oss << make_log_prefix("send_decrypted") << message << "\n";
                g_logger.log_plain(oss.str());
            }
        }
    }


    // @brief webscoket回调函数，消息或接收失败回调
    void on_fail(websocketpp::connection_hdl hdl) {
        client::connection_ptr con = m_endpoint.get_con_from_hdl(hdl);

        std::ostringstream oss;
        oss << "Fail handler\n"
            << con->get_state() << "\n"
            << con->get_local_close_code() << "\n"
            << con->get_local_close_reason() << "\n"
            << con->get_remote_close_code() << "\n"
            << con->get_remote_close_reason() << "\n"
            << con->get_ec() << " - " << con->get_ec().message() << "\n";
        g_logger.log_plain(oss.str());
    }

    // @brief webscoket回调函数，建立websocket连接回调
    void on_open(websocketpp::connection_hdl hdl) {
        // 连接建立后，需要立即向服务端发送鉴权请求
        // 当鉴权通过后，服务端才会接收当前终端读数据，并开始巡航
        // 如果服务端10s内没有接收到鉴权请求或者鉴权失败，将主动关闭连接
        
        std::string verify_message;
        // 根据加密模式决定是否传入RSA公钥
        const int encryption_mode = 2;
        if (ENCRYPTION_MODE == encryption_mode) {
            // 传入RSA公钥，启用加密通信（enc=2）
            get_verify_json(++m_msgid, verify_message, _rsa_public_key);
        } else {
            // 不使用加密（enc=1）
            get_verify_json(++m_msgid, verify_message);
        }
        
        // 向服务端发送鉴权请求
        m_endpoint.send(hdl, verify_message, websocketpp::frame::opcode::text);

        g_logger.log_send(make_log_prefix("send"), verify_message);
    }

    // @brief webscoket回调函数，接收到消息回调
    void on_message(websocketpp::connection_hdl hdl, message_ptr msg) {
        std::string payload = msg->get_payload();
        
        // 打印接收到的原始消息（异步，不阻塞IO线程）
        g_logger.log_receive(make_log_prefix("receive"), msg);

        // 如果启用了显示解密数据，尝试解密并打印
        if (SHOW_DECRYPTED == 2) {
            std::string decrypted_payload = _try_decrypt_message(payload);
            if (!decrypted_payload.empty() && decrypted_payload != payload) {
                std::ostringstream oss;
                oss << make_log_prefix("decrypted") << decrypted_payload << "\n";
                g_logger.log_plain(oss.str());
            }
        }

        // 解析返回的json
        _process_message(payload);
    }

    // @brief webscoket回调函数，连接关闭回调
    void on_close(websocketpp::connection_hdl hdl) {
        _init_flag = false;
        m_endpoint.close(hdl, websocketpp::close::status::abnormal_close, "connection close");
        g_logger.log_plain("close connection\n");
    }

    // @brief 检查连接是否建立
    bool is_open() {
        if (!_init_flag.load()) {
            g_logger.log_plain("connection not established, ignore this message\n");
            return false;
        }

        // 检查连接是否是open状态
        client::connection_ptr con = m_endpoint.get_con_from_hdl(m_hdl);

        if (con->get_state() != websocketpp::session::state::open) {
            std::ostringstream oss;
            oss << "connection is " << con->get_state() << ", not ready\n";
            g_logger.log_plain(oss.str());
            return false;
        }
        
        return true;
    }

private:
    // @brief 尝试解密收到的消息
    // @param message 接收到的json字符串
    // @return 解密后的完整json（如果成功），否则返回空字符串
    std::string _try_decrypt_message(const std::string& message) {
        // 解析JSON
        json j = json::parse(message, /*cb=*/nullptr, /*allow_exceptions=*/false);
        if (j.is_discarded()) {
            return "";
        }

        // 检查是否是加密消息（enc=3）
        int enc_value = 1;
        if (j.contains("enc")) {
            enc_value = j["enc"];
        }

        if (enc_value == 3 && j.contains("data") && j["data"].is_string() && 
            !_aes_key.empty() && !_aes_iv.empty()) {
            // 这是一个加密消息，尝试解密
            std::string encrypted_data = j["data"];
            
            // Step 1: Base64解码
            std::string base64_decoded = RsaCrypt::base64_decode(encrypted_data);
            
            // Step 2: AES解密
            std::string decrypted_data = AesCrypt::aes_decrypt(base64_decoded, _aes_key, _aes_iv);
            
            if (!decrypted_data.empty()) {
                // 构造解密后的完整JSON（替换data字段）
                j["data"] = json::parse(decrypted_data, /*cb=*/nullptr, /*allow_exceptions=*/false);
                return j.dump(); // 单行输出，便于阅读
            }
        }

        return "";
    }

    // @brief 解析返回的数据
    // @param message 接收到json字符串
    void _process_message(const std::string& message) {
        // 鉴权通过后就不再耗费时间解析后续的业务消息，避免阻塞 IO 线程
        if (_init_flag.load()) {
            return;
        }

        json j = json::parse(message, /*cb=*/nullptr, /*allow_exceptions=*/false);
        if (j.is_discarded()) {
            g_logger.log_plain("parse auth response failed\n");
            return;
        }

        if (!j.contains("qt")) {
            g_logger.log_plain("auth response missing qt field\n");
            return;
        }

        if (j["qt"] == "auth") {
            // 检查是否使用了加密（enc=2）
            int enc_value = 1;
            if (j.contains("enc")) {
                enc_value = j["enc"];
            }
            
            if (enc_value == 2 && j.contains("data") && j["data"].is_string()) {
                // 加密模式：data是用RSA公钥加密后再base64编码的字符串
                // 需要先base64解码，再用RSA私钥解密
                std::string encrypted_data = j["data"];
                
                // Step 1: Base64解码
                std::string base64_decoded = RsaCrypt::base64_decode(encrypted_data);
                
                // Step 2: RSA私钥解密
                std::string decrypted_data = RsaCrypt::rsa_pri_decrypt(base64_decoded, _rsa_private_key);
                
                if (decrypted_data.empty()) {
                    g_logger.log_plain("RSA decrypt failed\n");
                    return;
                }
                
                // Step 3: 解析解密后的JSON
                json decrypted_json = json::parse(decrypted_data, /*cb=*/nullptr, /*allow_exceptions=*/false);
                if (decrypted_json.is_discarded()) {
                    g_logger.log_plain("parse decrypted auth data failed\n");
                    return;
                }

                // 如果启用了显示解密数据，打印解密后的完整鉴权响应
                if (SHOW_DECRYPTED == 2) {
                    j["data"] = decrypted_json;
                    std::ostringstream oss_decrypted;
                    oss_decrypted << make_log_prefix("decrypted") << j.dump() << "\n";
                    g_logger.log_plain(oss_decrypted.str());
                }

                // Step 4: 检查鉴权状态并提取AES密钥和IV
                if (decrypted_json.contains("status") && decrypted_json["status"] == 0) {
                    if (decrypted_json.contains("aes_key") && decrypted_json.contains("aes_iv")) {
                        _aes_key = decrypted_json["aes_key"];
                        _aes_iv = decrypted_json["aes_iv"];
                        _init_flag = true;
                    } else {
                        g_logger.log_plain("auth response missing aes_key or aes_iv\n");
                    }
                } else {
                    // 鉴权失败，打印状态信息
                    std::ostringstream oss;
                    oss << "Auth failed. Status: ";
                    if (decrypted_json.contains("status")) {
                        oss << decrypted_json["status"].get<int>();
                    } else {
                        oss << "unknown";
                    }
                    oss << ", Message: ";
                    if (decrypted_json.contains("msg")) {
                        oss << decrypted_json["msg"].get<std::string>();
                    } else {
                        oss << "unknown";
                    }
                    oss << "\n";
                    g_logger.log_plain(oss.str());
                }
            } else if (j.contains("data") && j["data"].contains("status") && j["data"]["status"] == 0) {
                // 非加密模式（enc=1）：data是普通的JSON对象
                _init_flag = true;
            }
        }
    }

private:
    // websocket endpoint
    client m_endpoint;

    // websocket handler
    websocketpp::connection_hdl m_hdl;

    // 发送消息时需要唯一对id，该变量用于消息id的自增
    std::atomic<long> m_msgid = {0};

    // 建立连接并鉴权通过后才能向服务端上报位置数据，
    // 该变量用于标识是否完成这些动作
    std::atomic<bool> _init_flag;
    
    // RSA密钥对，用于鉴权阶段
    std::string _rsa_public_key;   // RSA公钥
    std::string _rsa_private_key;  // RSA私钥
    
    // AES密钥和IV，从服务端鉴权响应中获取，用于后续通信加密
    std::string _aes_key;  // AES对称密钥
    std::string _aes_iv;   // AES初始化向量
};

// @brief 启动一个线程回放提前记录在case文件中的json数据
void replay_worker(std::weak_ptr<WebsocketClient> client) {
    g_logger.log_plain("start replay_worker\n");

    while (!client.lock()->is_open()) {
        // 等待建立连接后再发送
        g_logger.log_plain("connection not establish, waiting 1s...\n");
        sleep(1);
    }
        
    // 上报到服务端的消息已经保存到case文件中
    // 逐行读取并回放
    std::ifstream in_loc_stream(CASE);

    int64_t last_timestamp = 0;
    std::string line;

    while (getline(in_loc_stream, line)) {
        // 向服务端上报路线数据
        if (!client.expired()) {
            client.lock()->send(line);

        } else {
            g_logger.log_plain("endpoint expired\n");
        }
    
        // 建议按1s间隔上报数据
        usleep(1000 * 1000);

    }
}

// @brief 启动一个线程保持websocket client到服务端的连接
void client_worker(std::shared_ptr<WebsocketClient> endpoint_ptr) {
    while (true) {
        try {
            // 启动WebsocketClient
            // 如果启动成功将会阻塞住该线程
            endpoint_ptr->start();

        } catch (websocketpp::exception const & e) {
            std::ostringstream oss;
            oss << e.what() << "\n";
            g_logger.log_plain(oss.str());
        } catch (std::exception const & e) {
            std::ostringstream oss;
            oss << e.what() << "\n";
            g_logger.log_plain(oss.str());
        } catch (...) {
            g_logger.log_plain("other exception\n");
        }

        g_logger.log_plain("retry connect to baidu map service, wait 5 sec\n");
        sleep(5);
    }
}

int main(int argc, char* argv[]) {
    // 参数解析：./baidu_map_client <entity_id> <data_file> [-e 1|2]
    // -e 1: 明文模式（默认）
    // -e 2: 加密模式
    
    const int encryption_mode = 2;
    const int plain_mode = 1;
    
    if (argc < 3) {
        std::cout << "用法: " << argv[0] << " <entity_id> <data_file> [-e 1|2]" << std::endl;
        std::cout << "  -e 1: 明文模式（默认）" << std::endl;
        std::cout << "  -e 2: 加密模式（RSA + AES）" << std::endl;
        return 1;
    }
    
    std::string entity_id = std::string(argv[1]);
    std::string case_file = std::string(argv[2]);

    if (entity_id != "") {
        ENTITY_ID = entity_id;
    }

    if (case_file != "") {
        CASE = case_file;
    }

    // 默认使用明文模式
    ENCRYPTION_MODE = plain_mode;
    // 默认不显示解密数据
    SHOW_DECRYPTED = 1;
    // 解析命令行参数
    // 格式：./baidu_map_client entity_id data_file [-e encryption_mode [show_decrypted]]
    // -e 1: 明文模式
    // -e 2: 加密模式
    // -e 2 1: 加密模式，不显示解密数据（默认）
    // -e 2 2: 加密模式，显示解密数据
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-e" && i + 1 < argc) {
            int mode = std::atoi(argv[i + 1]);
            if (mode == encryption_mode) {
                ENCRYPTION_MODE = encryption_mode;
            } else {
                ENCRYPTION_MODE = plain_mode;
            }
            i++; // 跳过下一个参数
            
            // 检查是否有第二个参数（显示解密数据的选项）
            if (i + 1 < argc) {
                int show_mode = std::atoi(argv[i + 1]);
                if (show_mode == 2) {
                    SHOW_DECRYPTED = 2;  // 显示解密数据
                } else {
                    SHOW_DECRYPTED = 1;  // 不显示解密数据
                }
                i++;  // 跳过这个参数（无论是1还是2）
            }
        }
    }
    
    // 打印配置信息
    std::cout << "========================================" << std::endl;
    std::cout << "百度地图WebSocket客户端" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Entity ID: " << ENTITY_ID << std::endl;
    std::cout << "数据文件: " << CASE << std::endl;
    std::cout << "加密模式: " << (ENCRYPTION_MODE == encryption_mode ? "启用" : "禁用") << std::endl;
    if (ENCRYPTION_MODE == encryption_mode) {
        std::cout << "显示解密: " << (SHOW_DECRYPTED == 2 ? "启用" : "禁用") << std::endl;
    }
    std::cout << "========================================" << std::endl;

    // 创建一个WebsocketClient对象 
    std::shared_ptr<WebsocketClient> endpoint_ptr = std::make_shared<WebsocketClient>();
    std::weak_ptr<WebsocketClient> w_endpoint_ptr(endpoint_ptr);

    // 启动获取GPS数据的线程
    std::thread replay_thread(replay_worker, w_endpoint_ptr);

    // 启动websocket client线程
    std::thread client_thread(client_worker, endpoint_ptr);
    
    // join线程
    replay_thread.join();
    client_thread.join();

    return 0;
}
