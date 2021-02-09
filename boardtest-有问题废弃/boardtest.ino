//#define SLEEP_TIME 1200000   // 无触发上报温湿度时间20分钟
float lastVcc = 3.3 ;           //电池电压
float lastTemperature = -1;      //温度
float lastHumidity = -1;         //湿度
uint16_t lastlux = 0;            //光线
//bool lastReseted = false;         //配置按钮
bool lastTripped;          //人体感应
int batteryPcnt ;
char serialMsg;
int shu = 0;                     //看门狗延时计数
bool wakebypir;             //是否PIR中断唤醒
bool wakebyanniu;           //是否按钮中断唤醒
bool wakebydog;             //是否看门狗唤醒
int wakeby = 1;                     //1=看门狗,2=PIR中断,3=按钮中断
int pirTimeSet = 4;              //PIR保持时间设置,默认4*8=32秒
int pirTimeKeep = 0;              //PIR保持时间变量
int zigbeeStated = 1;             //zigbee网络状态 1=未入网,2=入网中,3=已入网,4=出错

/*Sensor & Battery Pin */
#define ANNIU_PIN 2 // 配网按钮，d2（拉高）
#define PIR_PIN 3 // PIR人体触发，d3（拉高）
#define LED_PIN 5  // 指示灯（低电平亮），D5（拉高）
#define TY_PIN 4  // 涂鸦模组唤醒脚（低电平1-10S唤醒），D4（拉高）
#define BATTERY_SENSE_PIN  A1  // 电量检测A1
#define LIGHT_SENSOR_ANALOG_PIN A2  // 光敏电阻检测A2
#define VMIN 1.8
#define VMAX 3.3

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <SHT3x.h>
SHT3x sht;

ISR(WDT_vect) {
  //看门狗唤醒计时函数
  shu++;
  pirTimeKeep++;
  wakebydog = true;
  wakeby = 1;
  
}

void wdt_setup(int ii)
{
  // ii为看门狗超时时间，支持以下数值：
  // 0=16毫秒, 1=32毫秒,2=64毫秒,3=128毫秒,4=250毫秒,5=500毫秒
  // 6=1秒 ,7=2秒, 8=4秒, 9=8秒
  byte bb;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  //开始设置看门狗中断
  MCUSR &= ~(1 << WDRF); //清除复位标志
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  //设置新的看门狗超时时间
  WDTCSR = bb;
  //设置为定时中断而不是复位
  WDTCSR |= _BV(WDIE);
  //别忘了设置【看门狗唤醒计时函数】
}

void setup() {
  pinMode(ANNIU_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(TY_PIN, OUTPUT);
  digitalWrite(TY_PIN, HIGH);
  //Wire.begin();
  Serial.begin(9600);
  //delay(200);

  sht.Begin();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);              //设置睡眠模式
  int intNum = digitalPinToInterrupt(PIR_PIN);       //普通IO编号转换成中断号
  int intNuma = digitalPinToInterrupt(ANNIU_PIN);
  sleep_enable();                                    //开启休眠功能
  attachInterrupt(intNum, pirWakeup, FALLING);    //PIR在下降沿开启中断,执行pirWakeup
  attachInterrupt(intNuma, anniuWakeup, FALLING);  //按钮在下降沿开启中断,执行anniuWakeup
  //ACSR |=_BV(ACD);//关掉ACD，据说很省电。不知道唤醒以后要不要重新开，怎么开？
  //ADCSRA=0;//关掉ADC，据说很省电。不知道唤醒以后要不要重新开，怎么开？
  //按照官方解释，sleep_enable()最好写在中断(attachInterrupt())前，防止中断在开始休眠前就提前释放而造成休眠后无法唤醒。
  //开始设置看门狗中断，用来唤醒。
  wdt_setup(9);
}

void loop() {
  if (wakeby == 2) {
    lastTripped = true;
    wakebypir = false;
    Serial.println("有人");
    delay(50);
    int16_t lightLevel = analogRead(LIGHT_SENSOR_ANALOG_PIN);  //光敏的值
    if (lightLevel != lastlux) {
      lastlux = lightLevel;
      Serial.println(lightLevel);
    }
    delay(50);
    batterylevel();    //上报电池

  }else if (wakeby == 3) {
    Serial.println("重新配网");
    //wakebyanniu = false;
    

  }else if (wakeby == 1) {
    wakebydog = false;
    if (shu >= 75) {

      sht.UpdateData();
      lastHumidity = sht.GetRelHumidity();                 //取得湿度
      lastTemperature = sht.GetTemperature(SHT3x::Cel);    //取得温度
      Serial.print("温度: ");
      Serial.print(lastTemperature);
      Serial.write("\xC2\xB0"); //The Degree symbol
      Serial.print("C\n");
      Serial.print("湿度: ");
      Serial.print(lastHumidity);
      Serial.print("%\n");
      shu = 0;
      delay(50);
      batterylevel();    //上报电池

    } 
    
  }

  if (pirTimeKeep >= pirTimeSet) {
    lastTripped = false;
    pirTimeKeep = 0;
    Serial.println("有人状态结束");
    batterylevel();    //上报电池
    
  }
  if (zigbeeStated == 1) {       //重新配网后,未入网状态LED慢闪
    wakebyanniu = false;
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    
  }
    Serial.println(wakeby);
    Serial.println(pirTimeKeep);
    Serial.println(shu);
    Serial.println(lastTripped);
    delay(50);
    sleep_cpu();//进入休眠状态，从此处开始进入休眠。
  


}

//电池检测
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
    //sendBatteryLevel(batteryPcnt);
    //send(msgVbat.set(batteryV, 2));
    Serial.print("电池电量: ");
    Serial.print(batteryPcnt);
    Serial.print("%\n");
    Serial.print("电池电压: ");
    Serial.print(batteryV);
    Serial.print("V\n");
    lastVcc = batteryV;
  }
}

void pirWakeup(void)
{
  wakeby = 2;
  //wakebypir = true;          //人体感应唤醒
  pirTimeKeep = 0;     //PIR唤醒后,PIR保持时间清零
  shu++;
  Serial.println("Pir");
}

void anniuWakeup(void)
{
  wakeby = 3;
  wakebyanniu = true;          //重置配网按钮唤醒
  zigbeeStated = 1;          //发起重新配网,MCU不进入睡眠直到配网结束
  Serial.println("Anniu");
}
