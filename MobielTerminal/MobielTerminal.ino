// arduino IDE 上的型号选择 AiM2M CORE ESP32C3
#include <FastLED.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <wifi_client_secure.h>
#include <HardwareSerial.h>

// 屏幕串口配置
#define TJC Serial1
#define TX_Pin 0
#define RX_Pin 1
#define FRAME_LENGTH 6
#define DATA_PIN     8  // 灯带数据引脚 (GPIO8)
#define NUM_LEDS     5  // LED 数量
#define BRIGHTNESS  100 // 初始亮度 (0-255)
CRGB leds[NUM_LEDS];    // LED 状态数组

// WiFi 配置
const char* ssid = "DESKTOP-4R2U1JS 3676";
const char* password = "15)83w2e";

// MQTT 配置
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 8883; // SSL 端口
const char* mqttUser = "";
const char* mqttPassword = "";
const char* mqttTopic = "home/catTracker";
const char* commandTopic = "home/catCommand"; // 命令主题

WiFiClientSecure espClient;
PubSubClient client(espClient);

// 猫咪位置数据结构
struct CatPosition {
    String room;
    float distance;
    int angle;
    bool detected;
    unsigned long timestamp;
};

// 存储猫咪位置
CatPosition catPositions[5]; // 最多 5 个位置
int positionCount = 0;

// 根证书
const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGRElnSWRlcnQxGzAZBgNVBAMTEkRp\n" \
"Z2lDZXJ0IEdsb2JhbCBSb290IEcyMB4XDTEzMDgwMTEyMDAwMFoXDTM4MDExNTEy\n" \
"MDAwMFowOTELMAkGA1UEBhMCVVMxDzANBgNVBAoTbkRpZ2lDZXJ0IEluYzEbMBkG\n" \
"A1UEAxMSRGlnaUNlcnQgR2xvYmFsIFJvb3QgRzIwggEiMA0GCSqGSIb3DQEBAQUA\n" \
"A4IBDwAwggEKAoIBAQC7SnSeiBCo0gDdkJzHvcNaQEBBAQDDCaQoCcGgeEBALJ4g\n" \
"ca9HgFB0fW7Y14h29JioL91gHYP10hAEvrAitht0gQ3pOsqTQNroBvo3bSMgFzZM\n" \
"9O6II8c+6zfRtnR4S1wi3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNmcmzU5L/qw\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRwa6\n" \
"V0ujw5H55Nz/0egwLXOtdHA114gk957EllW67c4x8JJKLHd+rcdqsq08p8kDi1ll\n" \
"93FcXmn/6pUCyziKr1A4b9v7LWi bXcceV0F34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n" \
"AYYwHQYDVR0OBBYEFEuM1ljQucFmcz7IQTgoImA0GCSqGSIb3DQEBCwUAA4IBAQCy\n" \
"A4IBAQCYBjdaQZChGsV2USggNiM0ruYo6r4lk5IpDB/G/wkjUu0yKGX9rbxenDI\n" \
"U5PHCCjjmCXPI6T53iHTfIUJrU6adTrCC2qjEHzeRXhlbiI03jt/msv0tadQi1W\n" \
"N+gDS63pYaACbVXy8MNy7Vu33PquXHeE6V/Uq2v8vito96LXfXvKWjybK8U90uv\n" \
"o/uFQJvtMVT0tPhRh8jrdkPSHCa2XV4cdFyQzR1bl dZwgJcJmApzyMZfo6IQ6XU\n" \
"5mSi+yMRQ+hDKXJioaldXgjUkK642M4UutBV8ob2xJNDd2ZhwLnoQdeXeGADBkpy\n" \
"rqXRfboQnoZsG4q5WTP4685QvvG5\n" \
"-----END CERTIFICATE-----\n";

void setup() {
    Serial.begin(115200);

    // 设置根证书
    espClient.setCACert(root_ca);

    // 1. 先设置 WiFi 模式
    WiFi.mode(WIFI_STA);

    // 2. 在连接 WiFi 前设置频道
    esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);

    // 3. 打印当前频道
    uint8_t primary;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    Serial.printf("初始频道：%d\n", primary);

    // 连接 WiFi
    connectToWiFi();

    // 打印实际频道
    esp_wifi_get_channel(&primary, &second);
    Serial.printf("网关实际频道：%d\n", primary);

    // 初始化 MQTT
    client.setServer(mqttServer, mqttPort);
    client.setCallback(mqttCallback);

    // 初始化 ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 初始化失败");
        return;
    }

    // 注册接收回调
    esp_now_register_recv_cb(onDataReceived);

    Serial.println("家庭网关启动");

    // 打印网关 MAC 地址
    Serial.print("网关 MAC 地址：");
    Serial.println(WiFi.macAddress());
}

void loop() {
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();

    // 每 2 秒发送一次数据到云端
    static unsigned long lastSendTime = 0; // 添加变量声明
    if (millis() - lastSendTime > 2000) {
        sendDataToCloud();
        lastSendTime = millis(); // 更新最后发送时间
    }

    delay(10);
}

void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("连接 WiFi");

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        // 强制设置 WiFi 频道为 11
        esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);

        // 打印实际频道
        uint8_t primary;
        wifi_second_chan_t second;
        esp_wifi_get_channel(&primary, &second);
        Serial.printf("\n连接成功！IP: %s, 频道: %d\n", WiFi.localIP().toString().c_str(), primary);
    } else {
        Serial.println("\nWiFi 连接失败");
    }
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("连接 MQTT 服务器...");

        if (client.connect("CatGateway", mqttUser, mqttPassword)) {
            client.subscribe(commandTopic);
            Serial.println("连接成功");
        } else {
            Serial.print("失败, rc=");
            Serial.print(client.state());
            Serial.println(" 5 秒后重试");
            delay(5000);
        }
    }
}

void sendDataToCloud() {
    // 创建 JSON 文档
    DynamicJsonDocument doc(1024);
    JsonArray nodes = doc.createNestedArray("nodes");

    Serial.print("可用的节点数据：");
    Serial.println(nodeCount);

    // 添加所有节点数据
    for (int i = 0; i < nodeCount; i++) {
        // 移除时间限制，发送所有数据
        Serial.print("发送节点 ");
        Serial.print(i);
        Serial.print(": 房间=");
        Serial.print(nodeData[i].room);
        Serial.print(", 距离=");
        Serial.print(nodeData[i].distance);
        Serial.print(", 角度=");
        Serial.println(nodeData[i].angle);

        JsonObject node = nodes.createNestedObject();
        node["id"] = nodeData[i].nodeId;
        node["room"] = nodeData[i].room;
        node["distance"] = nodeData[i].distance;
        node["angle"] = nodeData[i].angle;
        node["detected"] = nodeData[i].detected;
        node["timestamp"] = nodeData[i].timestamp;
    }

    // 序列化 JSON
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    Serial.print("发送到云端：");
    Serial.println(jsonBuffer);

    // 发布到 MQTT
    client.publish(mqttTopic, jsonBuffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("收到 MQTT 消息：");
    Serial.println(topic);

    if (strcmp(topic, mqttTopic) == 0) {
        // 处理位置信息 (原有逻辑)
        Serial.print("MQTT 消息长度：");
        Serial.println(length);

        // 解析 JSON
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, payload, length);

        if (error) {
            Serial.print("JSON 解析失败：");
            Serial.println(error.c_str());
            return;
        }

        // 重置位置计数
        nodeCount = 0;

        // 提取节点数据
        if (doc.containsKey("nodes")) {
            JsonArray nodes = doc["nodes"];
            Serial.print("找到节点数量：");
            Serial.println(nodes.size());

            for (JsonObject node : nodes) {
                if (nodeCount >= 5) break;

                Serial.print("处理节点：");
                Serial.println(nodeCount);

                nodeData[nodeCount].room = node["room"].as<String>();
                nodeData[nodeCount].distance = node["distance"];
                nodeData[nodeCount].angle = node["angle"];
                nodeData[nodeCount].detected = node["detected"];
                nodeData[nodeCount].timestamp = node["timestamp"];

                Serial.print("房间: ");
                Serial.print(nodeData[nodeCount].room);
                Serial.print(", 距离: ");
                Serial.print(nodeData[nodeCount].distance);
                Serial.print(", 角度: ");
                Serial.print(nodeData[nodeCount].angle);
                Serial.print(", 检测: ");
                Serial.println(nodeData[nodeCount].detected);

                nodeCount++;
            }
        } else {
            Serial.println("JSON 中没有 'nodes' 字段");
        }

        Serial.print("最终位置计数：");
        Serial.println(nodeCount);
    }
    // 添加对命令主题的处理
    else if (strcmp(topic, commandTopic) == 0) {
        Serial.print("收到播放命令：");
        Serial.println((char*)payload);

        // 解析 JSON 命令
        DynamicJsonDocument doc(128);
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error) {
            Serial.print("JSON 解析失败：");
            Serial.println(error.c_str());
            return;
        }

        int targetNodeId = doc["node"];
        const char* soundFile = doc["sound"];

        Serial.print("目标节点：");
        Serial.print(targetNodeId);
        Serial.print(", 声音文件：");
        Serial.println(soundFile);

        // 创建播放命令
        PlayCommand command;
        command.targetNodeId = targetNodeId;
        strncpy(command.soundFile, soundFile, sizeof(command.soundFile));

        // 根据节点 ID 选择目标 MAC 地址
        uint8_t* targetMac = nullptr;
        if (targetNodeId == 1) {
            targetMac = node1Mac;
        } else if (targetNodeId == 2) {
            targetMac = node2Mac;
        } else if (targetNodeId == 3) {
            targetMac = node3Mac;
        }

        if (targetMac != nullptr) {
            esp_err_t result = esp_now_send(targetMac, (uint8_t *) &command, sizeof(command));
            if (result == ESP_OK) {
                Serial.println("ESP-NOW 命令发送成功");
            } else {
                Serial.print("ESP-NOW 发送失败，错误代码：");
                Serial.println(result);
            }
        } else {
            Serial.println("未知节点 ID，无法发送");
        }
    }
}

// ESP-NOW 数据接收回调
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *mac = info->src_addr;

    Serial.print("收到数据来自：");
    for(int i=0; i<6; i++) {
        Serial.printf("%02X", mac[i]);
        if(i<5) Serial.print(":");
    }
    Serial.println();

    Serial.print("收到 ESP-NOW 数据，长度：");
    Serial.println(len);

    // 修改：检查长度是否为 40 (节点实际发送的长度)
    if (len == 40) {
        SensorData receivedData;
        // 手动解析 40 字节的数据
        receivedData.nodeId = *((int*)data);
        receivedData.room = String((const char*)(data + 4)); // 4 字节后是字符串
        receivedData.distance = *((float*)(data + 24));    // 假设 room 占用 20 字节
        receivedData.angle = *((int*)(data + 28));
        receivedData.detected = *((bool*)(data + 32));
        receivedData.timestamp = *((uint32_t*)(data + 36));

        // 更新或添加节点数据
        bool found = false;
        for (int i = 0; i < nodeCount; i++) {
            if (nodeData[i].nodeId == receivedData.nodeId) {
                nodeData[i] = receivedData;
                found = true;
                break;
            }
        }

        if (!found && nodeCount < 5) {
            nodeData[nodeCount] = receivedData;
            nodeCount++;
        }

        Serial.print("收到节点数据：");
        Serial.print(receivedData.nodeId);
        Serial.print(", 房间：");
        Serial.print(receivedData.room);
        Serial.print(", 距离：");
        Serial.print(receivedData.distance);
        Serial.print("m, 角度：");
        Serial.print(receivedData.angle);
        Serial.println("°");

        // 立即打印时间戳
        Serial.print("时间戳：");
        Serial.println(receivedData.timestamp);
        Serial.print("当前时间：");
        Serial.println(millis());
    } else {
        Serial.print("数据长度不匹配，期望：40");
        Serial.print(", 实际：");
        Serial.println(len);
    }
}