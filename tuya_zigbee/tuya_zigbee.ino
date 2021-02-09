#include "mcu_api.h"
#include "protocol.h"

#include "zigbee.h"

#define ANNIU_PIN 2 // 配网按钮，d2（拉高）
#define PIR_PIN 3 // PIR人体触发，d3（拉高）
#define LED_PIN 5  // 指示灯（低电平亮），D5（拉高）
#define TY_PIN 4  // 涂鸦模组唤醒脚（低电平1-10S唤醒），D4（拉高）
#define BATTERY_SENSE_PIN  A1  // 电量检测A1
#define LIGHT_SENSOR_ANALOG_PIN A2  // 光敏电阻检测A2
int time_cnt = 0, cnt = 0, init_flag = 0;
float lastVcc = 3.3 ;           //电池电压
float lastTemperature = 28.02;      //温度
float lastHumidity = 68.03;         //湿度
uint16_t lastlux = 200;            //光线
bool lasttripped = HIGH;        //人体


void setup() {

  pinMode(ANNIU_PIN, INPUT_PULLUP); // 配网按钮
  pinMode(PIR_PIN, INPUT_PULLUP);   // PIR人体触发
  pinMode(LED_PIN, OUTPUT);         // 指示灯（低电平亮）
  digitalWrite(LED_PIN, HIGH);
  pinMode(TY_PIN, OUTPUT);          // 涂鸦模组唤醒脚
  digitalWrite(TY_PIN, HIGH);
  Serial.begin(9600);              // 串口初始化
  //Serial.println("serial init successful!");

  zigbee_protocol_init();
}

void loop() {
  zigbee_uart_service();
  myserialEvent();      // 串口接收处理
  key_scan();           // 重置配网按键检测
  reportTemp(lastTemperature);  //上报温度值
  delay(5000);
}

void myserialEvent() {
  if (Serial.available()) {
    unsigned char ch = (unsigned char)Serial.read();
    uart_receive_input(ch);
    //Serial.print(ch);
  }
}

void key_scan(void)
{
  bool reseted = digitalRead(ANNIU_PIN) == HIGH; //按钮状态
  if (reseted == 0) {
    //Serial.print("123\r\n");
    digitalWrite(TY_PIN, LOW);   //先唤醒模组
    delay(2);
    digitalWrite(TY_PIN, HIGH);   //拉高唤醒脚
    delay(100);
    mcu_network_start();
    delay(100);
    
  }
}

void reportTemp(float utemp){
  digitalWrite(TY_PIN, LOW);   //先唤醒模组
  delay(1);
  digitalWrite(TY_PIN, HIGH);   //拉高唤醒脚
  mcu_dp_value_update(DPID_TEMP_CURRENT,long(utemp));         //VALUE型数据上报;

}
