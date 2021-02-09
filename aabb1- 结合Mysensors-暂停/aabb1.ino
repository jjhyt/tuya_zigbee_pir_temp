
#define MY_RADIO_RF24
#define MY_RF24_DATARATE RF24_250KBPS
#define MY_BAUD_RATE 9600
char SKETCH_NAME[] = "PirTemp";
char SKETCH_VERSION[] = "1.0";

#define SLEEP_TIME 1200000   // 无触发上报温湿度时间20分钟
#define BATT_REPORT_CYCLE 240   
float lastVcc = 3.3 ;           //电池电压
float lastTemperature = -1;      //温度
float lastHumidity = -1;         //湿度
uint16_t lastlux = 0;            //光线
int batteryPcnt ;
char serialMsg;
/*Sensor & Battery Pin */
#define ANNIU_PIN 2 // 配网按钮，d2（拉高）
#define PIR_PIN 3 // PIR人体触发，d3（拉高）
#define LED_PIN 5  // 指示灯（低电平亮），D5（拉高）
#define TY_PIN 4  // 涂鸦模组唤醒脚（低电平1-10S唤醒），D4（拉高）
#define BATTERY_SENSE_PIN  A1  // 电量检测A1
#define LIGHT_SENSOR_ANALOG_PIN A2  // 光敏电阻检测A2
#define VMIN 1.8
#define VMAX 3.3
#define CHILD_ID_VBAT 254 //电池电量上报ID
#define CHILD_ID 4   // 人体感应上报ID
#define CHILD_ID_LIGHT 13  //光线强度上报ID
#define TEMP_CHILD_ID 10        //温度上报ID
#define HUM_CHILD_ID 11         //湿度上报ID

//涂鸦zegbee模组SDK移植测试
#include "mcu_api.h"
#include "protocol.h"
#include "zigbee.h"

#include <MySensors.h>
#include <Wire.h>
#include "SHTSensor.h"
SHTSensor sht;
// MYSENSORS COMMUNICATION VARIABLES
MyMessage msg(CHILD_ID, V_TRIPPED);
MyMessage temperatureMsg(TEMP_CHILD_ID, V_TEMP);
MyMessage humidityMsg(HUM_CHILD_ID, V_HUM);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgVbat(CHILD_ID_VBAT, V_VOLTAGE); 

void setup() {
  // put your setup code here, to run once:
  pinMode(ANNIU_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,HIGH);
  pinMode(TY_PIN, OUTPUT);
  digitalWrite(TY_PIN,HIGH);
  Wire.begin();
  Serial.begin(9600);
  delay(500); 

  if (sht.init()) {
      //Serial.print("init(): success\n");
  } else {
      //Serial.print("init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
  while(Serial.read()>= 0){};   //清除串口数据
  zigbee_protocol_init();       //初始化Zigbee模块,该函数在mcu_api.cpp中,注意写好后删除函数中的#error行
  
}

void presentation()  {
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  present(CHILD_ID, S_MOTION);
  present(TEMP_CHILD_ID, S_TEMP);
  present(HUM_CHILD_ID, S_HUM);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  wait(50);
  present(CHILD_ID_VBAT, S_MULTIMETER, "VCC"); //Battery

}

void loop() {
  // put your main code here, to run repeatedly:
  bool tripped = digitalRead(PIR_PIN) == HIGH;  //人体感应状态
  bool reseted = digitalRead(ANNIU_PIN) == HIGH; //按钮状态
  int16_t lightLevel = analogRead(LIGHT_SENSOR_ANALOG_PIN);  //光敏的值
  send(msg.set(tripped?"1":"0"));  // 向网关发送触发状态
  send(msgLight.set(lightLevel)); // 向网关发送光线状态
  batterylevel();                 // 向网关发送电池状态
  if (sht.readSample()) {
      lastHumidity = sht.getHumidity();
      lastTemperature = sht.getTemperature();
      send(temperatureMsg.set(lastTemperature, 1));  // 向网关发送温度
      send(humidityMsg.set(lastHumidity, 1));        // 向网关发送湿度
  }
  while (Serial.available())
  {
    char c = Serial.read();
    uart_receive_input(c);
  }
  zigbee_uart_service();
  
  sleep(digitalPinToInterrupt(PIR_PIN), CHANGE,digitalPinToInterrupt(ANNIU_PIN), FALLING, SLEEP_TIME);  //睡眠
}


void batterylevel() {
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  float batteryV = sensorValue * 0.003363075;
  float v = (float) (batteryV - lastVcc);
  if ((batteryV > VMIN ) && ( batteryV < VMAX)) {
    batteryPcnt = int(((batteryV - VMIN) / (VMAX - VMIN)) * 100.);
  }
  else {
    if (batteryV < VMIN ) {
      batteryPcnt = 0 ;
    }
    else {
      batteryPcnt = 100;
    }
  }
  if ((lastVcc != batteryV) && (abs(v) >= 0.1)) {
    sendBatteryLevel(batteryPcnt);
    send(msgVbat.set(batteryV, 2));
    lastVcc = batteryV;
  }
}
