#include <DHT.h>
#include <SPI.h>
#include <WiFi.h>
#include <DHT_U.h>
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <PubSubClient.h>

/** WIFI反馈引脚 */
#define WIFI_STATE_PIN 22

/** 振动器引脚 */
#define SHOCK_PIN 1

/** 温湿度采集引脚 */
#define DHTPIN 15

/** 温湿度采集器型号 */
#define DHTTYPE DHT11

/** 最高温度阈值 */
#define MAX_TEMP 31

/** 蜂鸣器引脚 */
#define BUZZER_PIN 23

/** 文本大小 */
#define TEXT_SIZE 1

/** RGB R */
#define R_PIN 19

/** RGB G */
#define G_PIN 21

/** RGB B */
#define B_PIN 3

/** 红色值 */
int R_NUM = 255;

/** 绿色值 */
int G_NUM = 255;

/** 蓝色值 */
int B_NUM = 255;

/**
 * 蜂鸣器是否开启
 */
bool buzzerState = false;

/**
 * 振动器是否开启
 */
bool shockState = false;

/**
 * 蜂鸣器开关函数
 */
void onOrOffBuzzer();

/**
 * 温湿度采集实例
 */
sensors_event_t DHT_event;

/**
 * 老的 温度
 */
float oldTemperature;

/**
 * 湿热度
 */
float THI;

/**
 * 露珠点
 */
float DPT;

/**
 * 老的 湿度
 */
float oldHumidity;

/**
 * 初始化温度采集器
 */
DHT_Unified dht(DHTPIN, DHTTYPE);

/**
 *  数据采集
 *  切换灯光状态
 *  切换蜂鸣器状态
 *  切换振动器
 */
void dataCollection();

/**
 *  屏幕实例
 */
TFT_eSPI tft = TFT_eSPI();

/**
 *  更新屏幕信息
 */
void updateText();

/**
 *  初始化引脚
 */
void initPinMode();

/**
 *  根据环境温度变化灯光颜色
 */
void changeLight();

/**
 *  根据环境温度决定是否触发振动器
 */
void changeShock();

/**
 * WIFI 连接
 */
WiFiClient espClient;

/**
 * wifi 重连
 */
void reconnectWifi();

/**
 * wifi 连接
 */
void connectingWifi();

// MQTT Broker 服务端连接
const int mqtt_port = 1883;
const char *mqtt_topic = "attributes";
const char *mqtt_password = "xaOLytUEQI";
const char *mqtt_username = "5mk5srrjyi245nf4";
const char *mqtt_broker = "sh-3-mqtt.iot-api.com";

/**
 * 客户端事件
 */
PubSubClient client(espClient);

/**
 * mqtt 连接
 */
void connectingMqtt();

/**
 * mqtt 数据上送
 */
void publishMqttMesage();

void setup()
{
  Serial.begin(9600);
  connectingWifi();
  connectingMqtt();
  initPinMode();
  tft.init();
  dht.begin();
}

void loop()
{
  reconnectWifi();
  client.loop();
  dataCollection();
  delay(5000);
}

void initPinMode()
{
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  pinMode(SHOCK_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(WIFI_STATE_PIN, OUTPUT);
}

void dataCollection()
{
  dht.temperature().getEvent(&DHT_event);
  float temperature = DHT_event.temperature;
  dht.humidity().getEvent(&DHT_event);
  float humidity = DHT_event.relative_humidity;
  bool dataHasChange = temperature != oldTemperature || humidity != oldHumidity;
  if (dataHasChange)
  {
    oldHumidity = humidity;
    oldTemperature = temperature;
    onOrOffBuzzer();
    changeLight();
    changeShock();
    updateText();
    publishMqttMesage();
  }
}

void updateText()
{

  tft.setTextSize(TEXT_SIZE);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.drawString("<  SMART_VIVEW  >", 0, 0, 2);

  tft.drawString("tem:", 0, 30);
  tft.drawString(String(oldTemperature).c_str(), 30, 30);
  tft.drawString("℃", 70, 30);

  tft.drawString("hum:", 0, 45);
  tft.drawString(String(oldHumidity).c_str(), 30, 45);
  tft.drawString("%", 70, 45);

  THI = ((1.8 * oldTemperature + 32) - (0.55 - 0.55 * oldHumidity * 0.01) * (1.8 * oldTemperature - 26));
  tft.drawString("THI:", 0, 60);
  tft.drawString(String(THI).c_str(), 30, 60);
  tft.drawString("%", 70, 60);

  DPT = +(oldTemperature - ((100 - oldHumidity) / 5));
  tft.drawString("DPT:", 0, 75);
  tft.drawString(String(DPT).c_str(), 30, 75);
  tft.drawString("%", 70, 75);

  tft.drawString("maxTemp:", 0, 90);
  tft.drawString(String(MAX_TEMP).c_str(), 60, 90);

  String buzzerText = buzzerState ? "ON" : "OFF";
  tft.drawString("Buzzer:", 0, 105);
  tft.drawString(buzzerText, 60, 105);

  String shockText = shockState ? "ON" : "OFF";
  tft.drawString("Shock:", 0, 120);
  tft.drawString(shockText, 60, 120);

  String wifiStateText = WiFi.status() == WL_CONNECTED ? "online" : "offline";
  tft.drawString("WifiState:", 0, 135);
  tft.drawString(wifiStateText, 70, 135);

  String mqttStateText = client.connected() ? "online" : "offline";
  tft.drawString("mqttState:", 0, 150);
  tft.drawString(mqttStateText, 70, 150);
}

void onOrOffBuzzer() 
{
  buzzerState = oldTemperature >= MAX_TEMP;
  buzzerState ? digitalWrite(BUZZER_PIN, HIGH) : digitalWrite(BUZZER_PIN, LOW);
}

void changeLight()
{

  if (oldTemperature <= 28)
  {
    R_NUM = 255;
    G_NUM = 255;
    B_NUM = 255;
  }
  else if (oldTemperature <= 30)
  {
    R_NUM = 0;
    G_NUM = 0;
    B_NUM = 255;
  }
  else
  {
    R_NUM = 255;
    G_NUM = 0;
    B_NUM = 0;
  }

  analogWrite(R_PIN, R_NUM);
  analogWrite(G_PIN, G_NUM);
  analogWrite(B_PIN, B_NUM);
}

void changeShock()
{
  shockState = oldTemperature >= MAX_TEMP;
  shockState ? digitalWrite(SHOCK_PIN, HIGH) : digitalWrite(SHOCK_PIN, LOW);
}

void connectingWifi()
{
  const char *wifiName = "360WiFi-5DEFC7";
  const char *wifiPassword = "KD13949977409";
  WiFi.disconnect(true);
  WiFi.begin(wifiName, wifiPassword);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    digitalWrite(WIFI_STATE_PIN, HIGH);
  }
  Serial.println("wifi 连接成功");
}

void reconnectWifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    connectingWifi();
  }
}

void connectingMqtt()
{
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected())
  {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("连接成功");
      client.subscribe(mqtt_topic);
    }
    else
    {
      Serial.print("连接失败");
      Serial.print(client.state()); // 返回连接状态
      delay(2000);
    }
  }
}

void publishMqttMesage()
{
  if (!client.connected())
  {
    Serial.println("数据重连");
    connectingMqtt();
  }
  char msg[200];
  snprintf(
      msg, sizeof(msg),
      "{ \"temperature\":%.1f, \"humidity\":%.1f, \"buzzer\":%.1f }",
      oldTemperature,
      oldHumidity,
      buzzerState);
  client.publish(mqtt_topic, msg);
}