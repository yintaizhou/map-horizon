# 百度地图 WebSocket 客户端
## 编译说明
1. **依赖库**：gcc/g++ 版本 12.0 及以上，boost 库（需自行下载放入 `boost_1_83_0` 目录）
2. 修改 `CMakeLists.txt` 中的 gcc/g++ 路径为本机路径
3. 在 `main.cpp` 中设置 AK
4. 执行编译：
   cmake ./
   make
5. 编译成功后生成以下可执行文件：
   - `baidu_map_client` - 主程序
   - `aes_demo` - AES 加密测试
   - `rsa_demo` - RSA 加密测试

## 使用方法
执行 ./baidu_map_client entity_id 数据文件 -e 加密模式 显示解密
例如 
# 明文模式
./baidu_map_client entity0 shanghai_gaosu_cruise_case.jsonl
./baidu_map_client entity0 shanghai_gaosu_cruise_case.jsonl -e 1
# 加密模式（不显示解密数据）
./baidu_map_client entity0 shanghai_gaosu_cruise_case.jsonl -e 2
./baidu_map_client entity0 shanghai_gaosu_cruise_case.jsonl -e 2 1
# 加密模式（显示解密数据）
./baidu_map_client entity0 shanghai_gaosu_cruise_case.jsonl -e 2 2

### 参数说明
- `-e 1`: 明文模式（默认）
- `-e 2`: 加密模式（RSA + AES）
- `-e 2 1`: 加密模式，不显示解密数据（默认）
- `-e 2 2`: 加密模式，显示解密数据（调试用）

## 加密模式说明

### 加密流程
1. **鉴权阶段（enc=2）**：客户端生成 RSA 密钥对，公钥 urlencode 后发送；服务端生成 AES 密钥并用客户端公钥加密返回
2. **数据传输（enc=3）**：使用 AES 对称加密进行数据传输

### enc 字段
- `enc=1`: 明文传输
- `enc=2`: 鉴权阶段，RSA 加密
- `enc=3`: 数据传输，AES 加密

### 日志输出

**加密模式，不显示解密（-e 2 或 -e 2 1）：**
```
send {"data":"加密数据...","enc":3,...}
receive {"data":"加密数据...","enc":3,...}
```

**加密模式，显示解密（-e 2 2）：**
```
send {"data":"加密数据...","enc":3,...}
send_decrypted {"data":{"loc":{...}},"enc":1,...}
receive {"data":"加密数据...","enc":3,...}
decrypted {"data":{"ack":[...]},"enc":3,...}
```

## 注意事项
1. entity_id 需唯一，重复会导致服务出现问题导致数据混乱出错！
2. 数据文件为明文格式，程序自动加密

