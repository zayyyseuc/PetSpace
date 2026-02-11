#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SD.h>
#include <SPI.h>
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"
#include "AudioFileSourceSD.h"

// 硬件定义 - 修改了SD_CS引脚
#define RADAR_RX 1
#define RADAR_TX 0
#define I2S_DOUT 2
#define I2S_BCLK 3
#define I2S_LRC 4
#define PIR_PIN 5      // PIR传感器使用GPIO5
#define SD_CS 10       // SD卡CS引脚改为GPIO10
#define SD_MOSI 6
#define SD_MISO 7
#define SD_SCK 8

// 节点配置
const int NODE_ID = 1;
const String ROOM_NAME = "客厅";
uint8_t gatewayMac[] = {0x50, 0x78, 0x7D, 0xF2, 0x32, 0xD8};

// 音频对象
AudioGeneratorWAV *audio;
AudioFileSourceSD *file;
AudioOutputI2S *out;

HardwareSerial radarSerial(1);

// 存储解析后的数据
String status = "OFF";   // 存储"ON"或"OFF"，默认为OFF
int rangeValue = 0;      // 存储距离值，默认为0
bool pirDetected = false; // 存储HC-SR501检测状态

// 数据结构
struct SensorData {
    int nodeId;
    char room[20];
    float distance;
    int angle;
    bool detected;
    uint32_t timestamp;
};

struct PlayCommand {
    int targetNodeId;
    char soundFile[20];
};

void setup() {
    Serial.begin(115200);
    radarSerial.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);
    pinMode(PIR_PIN, INPUT);
    Serial.println("1");

    // 初始化SD卡 - 使用新的SD_CS引脚
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
        Serial.println("SD卡初始化失败");
    } else {
        Serial.println("SD卡初始化成功");
    }

    // 初始化音频
    out = new AudioOutputI2S(0, AudioOutputI2S::EXTERNAL_I2S);
    out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    out->SetOutputModeMono(true);
    out->SetGain(0.8);
    Serial.println("2");

    // 初始化ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW初始化失败");
        return;
    }

    esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
    // 在设置频道后添加
    uint8_t primary;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    Serial.printf("实际频道：%d\n", primary);
    Serial.println("3");

    // 添加发送状态回调 - 放在这里
    esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t status) {
        Serial.printf("发送状态：%s\n", status == ESP_NOW_SEND_SUCCESS ? "成功" : "失败");
        
        // 可选：打印目标MAC地址
        Serial.print("目标MAC: ");
        for(int i=0; i<6; i++) {
            Serial.printf("%02X", mac[i]);
            if(i<5) Serial.print(":");
        }
        Serial.println();
    });

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, gatewayMac, 6);
    peerInfo.channel = 11;
    peerInfo.encrypt = false;

    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus != ESP_OK) {
        Serial.printf("添加网关失败，错误代码：%d\n", addStatus);
        return;
    } else {
        Serial.println("添加网关成功");
    }
    Serial.println("4");

    esp_now_register_recv_cb(onDataReceived);
    Serial.println("传感器节点启动");
}

void loop() {
    // 打印WiFi状态
    Serial.printf("WiFi状态：%d\n", WiFi.status());

    // 打印网关MAC
    Serial.print("目标网关MAC: ");
    for(int i=0; i<6; i++) {
        Serial.printf("%02X", gatewayMac[i]);
        if(i<5) Serial.print(":");
    }
    Serial.println();

    SensorData data;
    data.nodeId = NODE_ID;
    strcpy(data.room, ROOM_NAME.c_str());
    data.timestamp = millis();

    readSensors(data);

    // 打印要发送的数据
    Serial.print("发送数据：");
    Serial.print("节点ID="); Serial.print(data.nodeId);
    Serial.print(", 房间="); Serial.print(data.room);
    Serial.print(", 距离="); Serial.print(data.distance);
    Serial.print(", 角度="); Serial.print(data.angle);
    Serial.print(", 检测="); Serial.println(data.detected);

    esp_err_t result = esp_now_send(gatewayMac, (uint8_t *) &data, sizeof(data));
    if (result == ESP_OK) {
        Serial.println("数据发送成功");
    } else {
        Serial.print("数据发送失败，错误代码：");
        Serial.println(result);
    }

    // 音频处理
    if (audio && audio->isRunning()) {
        if (!audio->loop()) {
            audio->stop();
            delete audio;
            delete file;
            audio = nullptr;
            file = nullptr;
        }
    }

    delay(2000); // 使用延迟代替睡眠
    // esp_sleep_enable_timer_wakeup(500000);
    // esp_light_sleep_start();
}

void readSensors(SensorData &data) {
    static String receivedLine = ""; // 存储一行数据

    // 1. 读取HC-SR501状态
    pirDetected = digitalRead(PIR_PIN) == HIGH;

    // 2. 检查串口1是否有数据到达
    while (radarSerial.available()) {
        char receivedChar = radarSerial.read();

        // 将接收到的数据原样发送到串口监视器
        Serial.write(receivedChar);

        // 构建完整的一行
        if (receivedChar == '\n') {
            // 解析接收到的行
            parseLine(receivedLine);

            // 只有当两个模块都检测到有人时，才输出ON和实际距离
            if (status == "ON" || pirDetected) {
                data.distance = rangeValue;
                data.detected = 1;
            }
            // 否则输出OFF和0cm
            else {
                data.detected = 0;
            }

            receivedLine = ""; // 清空行缓存
        } else if (receivedChar != '\r') {
            // 忽略回车符，只添加有效字符
            receivedLine += receivedChar;
        }
    }

    // 3. 定期检查并输出结果（即使没有新数据）
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 1000) { // 每秒检查一次
        lastCheck = millis();
    }
}

// 解析一行数据的函数
void parseLine(String line) {
    line.trim(); // 去除首尾空白字符

    if (line.equalsIgnoreCase("ON")) {
        status = "ON";
    }
    else if (line.equalsIgnoreCase("OFF")) {
        status = "OFF";
    }
    else if (line.startsWith("Range ")) {
        // 提取距离数值部分
        String rangeStr = line.substring(6); // 跳过"Range "
        rangeValue = rangeStr.toInt();       // 转换为整数
    }
}

// ESP-NOW数据接收回调（新签名）
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    const uint8_t *mac = info->src_addr; // 获取源MAC地址

    if (len == sizeof(PlayCommand)) {
        PlayCommand command;
        memcpy(&command, incomingData, len);

        if (command.targetNodeId == NODE_ID) {
            Serial.print("收到播放命令：");
            Serial.println(command.soundFile);
            playSound(command.soundFile);
        }
    }
}

void playSound(const char* filename) {
    if (audio && audio->isRunning()) return; // 防止重复播放

    file = new AudioFileSourceSD(filename);
    audio = new AudioGeneratorWAV();
    if (audio->begin(file, out)) {
        Serial.print("播放：");
        Serial.println(filename);
    } else {
        Serial.print("播放失败：");
        Serial.println(filename);
        delete audio;
        delete file;
        audio = nullptr;
        file = nullptr;
    }
}