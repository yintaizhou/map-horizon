#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

using json = nlohmann::json;

typedef websocketpp::config::asio_client::message_type::ptr message_ptr;
typedef client::connection_ptr connection_ptr;

const std::string AK = "您的ak";
const std::string URI = "wss://apitest.map.baidu.com/websocket";

// 传入客户端设备对唯一标识
std::string ENTITY_ID = "yintaizhou";

// 执行的case名称，例如当case名称为【shanghai_gaosu】时
// 对应的轨迹点文件【shanghai_gaosu_case】
// 对应的导航路线文件【shanghai_gaosu_route】
std::string CASE = "shanghai_gaosu";

// 配置模式，当传入路线时，探路会沿着导航路线进行
int ROUTE_TYPE = 2; // 1 巡航，2 导航

// 使用websocketpp实现一个Client
class WebsocketClient {
public:
    WebsocketClient() {
        _init_flag = false;

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
            std::cout << "websocket get connection failed: " << ec.message() << std::endl;
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
            std::cout << "connection not established, ignore this message" << std::endl;
            return;
        }

        // 检查连接是否是open状态
        client::connection_ptr con = m_endpoint.get_con_from_hdl(m_hdl);

        if (con->get_state() != websocketpp::session::state::open) {
            std::cout << "connection is " << con->get_state() << ", ignore this message" << std::endl;
            return;
        }

        ::websocketpp::lib::error_code ec;

        m_endpoint.send(m_hdl, message, websocketpp::frame::opcode::text, ec);
    }


    // @brief webscoket回调函数，消息或接收失败回调
    void on_fail(websocketpp::connection_hdl hdl) {
        client::connection_ptr con = m_endpoint.get_con_from_hdl(hdl);
        
        std::cout << "Fail handler" << std::endl;
        std::cout << con->get_state() << std::endl;
        std::cout << con->get_local_close_code() << std::endl;
        std::cout << con->get_local_close_reason() << std::endl;
        std::cout << con->get_remote_close_code() << std::endl;
        std::cout << con->get_remote_close_reason() << std::endl;
        std::cout << con->get_ec() << " - " << con->get_ec().message() << std::endl;
    }

    // @brief webscoket回调函数，建立websocket连接回调
    void on_open(websocketpp::connection_hdl hdl) {
        // 连接建立后，需要立即向服务端发送鉴权请求
        // 当鉴权通过后，服务端才会接收当前终端读数据，并开始巡航
        // 如果服务端10s内没有接收到鉴权请求或者鉴权失败，将主动关闭连接
        
        // 构造鉴权请求的json字符串
        json j;  
        j["id"] = m_msgid++;  
        j["qt"] = "auth";  
        j["ver"] = "1.0";  
        j["prt"] = 1;  
        j["enc"] = 1;  
        j["ts"] = time(nullptr);

        json data;
        // 从百度地图开放平台申请的ak，ak为服务端ak，需要开通巡航服务权限
        data["ak"] = AK;
        // entity_id是每个终端对唯一标识，服务端会根据此id建立服务端对ssesion
        // 如果两个终端的entity_id重复，后上线对终端会将之前的踢下线
        data["entity_id"] = ENTITY_ID;  

        j["data"] = data;  

        std::string message = j.dump();

        auto now = std::chrono::system_clock::now();
        time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::cout << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << " send " << message << std::endl;

        // 向服务端发送鉴权请求
        m_endpoint.send(hdl, message, websocketpp::frame::opcode::text);
    }

    // @brief webscoket回调函数，接收到消息回调
    void on_message(websocketpp::connection_hdl hdl, message_ptr msg) {
        // 打印接收到的消息
        auto now = std::chrono::system_clock::now();
        time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::cout << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << " receive " << msg->get_payload() << std::endl;

        // 解析返回的json
        _process_message(msg->get_payload());
    }

    // @brief webscoket回调函数，连接关闭回调
    void on_close(websocketpp::connection_hdl hdl) {
        _init_flag = false;
        m_endpoint.close(hdl, websocketpp::close::status::abnormal_close, "connection close");
        std::cout << "close connection" << std::endl;
    }

    // @brief 设置路线规划服务返回到导航路线


    // @brief 检查连接是否建立
    bool is_open() {
        if (!_init_flag.load()) {
            std::cout << "connection not established, ignore this message" << std::endl;
            return false;
        }

        // 检查连接是否是open状态
        client::connection_ptr con = m_endpoint.get_con_from_hdl(m_hdl);

        if (con->get_state() != websocketpp::session::state::open) {
            std::cout << "connection is " << con->get_state() << ", not ready" << std::endl;
            return false;
        }
        
        return true;
    }

private:
    // @brief 解析返回的数据
    // @param message 接收到json字符串
    void _process_message(const std::string& message) {
        json j = json::parse(message);

        // 返回的是鉴权结果，判断鉴权结果是否成功
        if (j["qt"] == "auth" && j["data"]["status"] == 0) {
            // 鉴权成功后将初始化标识位置为true，此时可以进行位置数据的上报了
            _init_flag = true;
            return;
        }

        // 返回的是巡航消息
        if (j["qt"] == "horizon") {
            // ...
        
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
};

void set_route(std::weak_ptr<WebsocketClient>& client, int64_t& msgid) {
    // 如果有路线，先设置路线，要注意模拟测试时轨迹点一定要在路线上，否则会导致无数据下发
    if (ROUTE_TYPE == 2) {
        std::ifstream infile;
        infile.open(CASE + "_route");

        if (!infile) {
            std::cout << "file route.json read failed" << std::endl;
            return;
        }

        std::string route_message((std::istreambuf_iterator<char>(infile)), std::istreambuf_iterator<char>());
        json data = json::parse(route_message);

        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
        json j;  
        j["id"] = msgid++;  
        j["qt"] = "horizon";  
        j["ver"] = "1.0";  
        j["prt"] = 1;  
        j["enc"] = 1;  
        j["ts"] = milliseconds.count();

        data["route"]["type"] = 2;
        json field;
        // field.push_back("link");
        field.push_back("slope");

        data["route"]["route_rsp_field"] = field;
        j["data"] = data;

        // 向服务端上报路线
        if (!client.expired()) {
            std::string message = j.dump();
            client.lock()->send(message);

            auto now = std::chrono::system_clock::now();
            time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::cout << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << " send " << message << std::endl;
        } else {
            std::cout << "endpoint expired" << std::endl;
        }
    } else if (ROUTE_TYPE == 1) {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
        json j;  
        j["id"] = msgid++;  
        j["qt"] = "horizon";  
        j["ver"] = "1.0";  
        j["prt"] = 1;  
        j["enc"] = 1;  
        j["ts"] = milliseconds.count();

        json route_info;
        route_info["type"] = 1;

        json data;
        data["route"] = route_info;  

        j["data"] = data;

        // 向服务端上报
        if (!client.expired()) {
            std::string message = j.dump();
            client.lock()->send(message);

            auto now = std::chrono::system_clock::now();
            time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::cout << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << " send " << message << std::endl;
        } else {
            std::cout << "endpoint expired" << std::endl;
        }
    }
    
    return;
}

void gps_worker(std::weak_ptr<WebsocketClient> client) {
    std::cout << "start gps_worker" << std::endl;

    // 假设gps坐标已经录制好，并存放在case.csv文件中
    // 从case.csv文件读取坐标并进行回放
    std::ifstream in_loc_stream(CASE + "_case");

    int64_t msgid = 0;
    std::string line;

    while (!client.lock()->is_open()) {
        std::cout << "connection not establish, waiting 1s..." << std::endl;
        sleep(1);
    }
    
    // 设置导航态、巡航态
    set_route(client, msgid);
    
    while (getline(in_loc_stream, line)) {
        // 按","对字符串进行分割
        std::stringstream ss(line);
        std::vector<std::string> loc_infos;
        std::string info;
        while (std::getline(ss, info, ',')) {
            loc_infos.push_back(info);
        }

        if (loc_infos.size() < 4) {
            continue;
        }

        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

        // 生成上报位置的json字符串
        json j;  
        j["id"] = msgid++;  
        j["qt"] = "horizon";  
        j["ver"] = "1.0";  
        j["prt"] = 1;  
        j["enc"] = 1;  
        j["ts"] = milliseconds.count();

        json loc;
        loc["x"] = std::atof(loc_infos[0].c_str());
        loc["y"] = std::atof(loc_infos[1].c_str());
        loc["speed"] = std::atof(loc_infos[2].c_str());
        loc["dir"] = std::atof(loc_infos[3].c_str());
        loc["coordtype"] = 1;
        loc["ts"] = milliseconds.count();

        json data;
        data["loc"] = loc;  

        j["data"] = data;

        std::string message = j.dump();

        // 向服务端上报位置
        if (!client.expired()) {
            client.lock()->send(message);

            auto now = std::chrono::system_clock::now();
            time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::cout << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << " send " << message << std::endl;
        } else {
            std::cout << "endpoint expired" << std::endl;
        }

        usleep(950000);
    }
}

void client_worker(std::shared_ptr<WebsocketClient> endpoint_ptr) {
    while (true) {
        try {
            // 启动WebsocketClient
            endpoint_ptr->start();

        } catch (websocketpp::exception const & e) {
            std::cout << e.what() << std::endl;
        } catch (std::exception const & e) {
            std::cout << e.what() << std::endl;
        } catch (...) {
            std::cout << "other exception" << std::endl;
        }

        std::cout << "retry connect to baidu service, wait 5 sec" << std::endl;
        sleep(5);
    }
}

int main(int argc, char* argv[]) {
    ENTITY_ID = std::string(argv[1]);
    CASE = std::string(argv[2]);

    // 创建一个WebsocketClient对象 
    std::shared_ptr<WebsocketClient> endpoint_ptr = std::make_shared<WebsocketClient>();
    std::weak_ptr<WebsocketClient> w_endpoint_ptr(endpoint_ptr);

    // 启动获取GPS数据的线程
    std::thread gps_thread(gps_worker, w_endpoint_ptr);

    // 启动websocket client线程
    std::thread client_thread(client_worker, endpoint_ptr);
    
    // join线程
    gps_thread.join();
    client_thread.join();

    return 0;
}
