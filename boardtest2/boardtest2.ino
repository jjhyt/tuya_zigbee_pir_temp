
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SHT3x.h>
SHT3x sht;


#define ANNIU_PIN 2   // 配网按钮，d2（拉高）
#define PIR_PIN 3     // PIR人体触发，d3（拉高）
#define LED_PIN 5  // 指示灯（低电平亮），D5（拉高）
#define TY_PIN 4  // 涂鸦模组唤醒脚（低电平1-10S唤醒），D4（拉高）
#define BATTERY_SENSE_PIN  A1  // 电量检测A1
#define LIGHT_SENSOR_ANALOG_PIN A2  // 光敏电阻检测A2
#define VMIN 1.8
#define VMAX 3.3

int SLEEP_TIME = 10;      // 上报温湿度间隔时间,看门狗周期为8秒,故上报时间为设置数*8秒
int shu = 0;          //看门狗唤醒记数
int pirshu = 0;       //PIR唤醒时看门狗记数
int wakeby;           //记录哪个中断唤醒 1=PIR,2=按钮,3=看门狗
float lastVcc = 3.3 ;           //电池电压
float lastTemperature = -1;      //温度
float lastHumidity = -1;         //湿度
uint16_t lastlux = 0;            //光线
//bool lastReseted = false;         //配置按钮
bool lastTripped;          //人体感应
int batteryPcnt ;
int pirTimeSet = 4;              //PIR保持时间设置,默认4*8=32秒
int zigbeeStated;               //zigbee网络状态 1=未入网,2=入网中,3=已入网,4=出错

ISR(WDT_vect) {
  //看门狗唤醒执行函数
  shu++;
  wakeby = 3;
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
  //别忘了设置【看门狗唤醒执行函数】
}

void setup() {
  pinMode(ANNIU_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(TY_PIN, OUTPUT);
  digitalWrite(TY_PIN, HIGH);
  Serial.begin(9600);
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);              //设置睡眠模式
  int intNum = digitalPinToInterrupt(PIR_PIN);       //普通IO编号转换成中断号
  int intNuma = digitalPinToInterrupt(ANNIU_PIN);
  sleep_enable();                                    //开启休眠功能
  attachInterrupt(intNum, sleepHandler, FALLING);    //开启中断
  attachInterrupt(intNuma, sleepHandlera, FALLING);
  //ACSR |=_BV(ACD);//关掉ACD，据说很省电。不知道唤醒以后要不要重新开，怎么开？
  //ADCSRA=0;//关掉ADC，据说很省电。不知道唤醒以后要不要重新开，怎么开？
  //按照官方解释，sleep_enable()最好写在中断(attachInterrupt())前，防止中断在开始休眠前就提前释放而造成休眠后无法唤醒。
  //开始设置看门狗中断，用来唤醒。
  wdt_setup(9);
  sht.Begin();
}

void loop() {
  //zigbeeStated = 1;           //测试数字,记得删除
  if (wakeby == 2) {
    int i;
    for (i = 0; i < 10; i++) {    //确保每个闪灯环节都是大概1秒,超过10秒退出入网循环
      Serial.print("循环次数:");
      Serial.println(i);
     
      switch (zigbeeStated) {
        case 1: {      //未入网快闪灯
          delay(250);
          digitalWrite(LED_PIN, LOW);
          delay(250);
          digitalWrite(LED_PIN, HIGH);
          delay(250);
          digitalWrite(LED_PIN, LOW);
          delay(250);
          digitalWrite(LED_PIN, HIGH);
          
            break;
          }
        case 2: {       //入网中慢闪灯
          delay(500);
          digitalWrite(LED_PIN, LOW);
          delay(500);
          digitalWrite(LED_PIN, HIGH);

            break;
          }
        case 4: {       //出错灯长亮
          digitalWrite(LED_PIN, LOW);
          delay(1000);
            
            break;
          }
        default: {       //默认已入网,关灯,跳出循环
          digitalWrite(LED_PIN, HIGH);
 
            break;
          }
      }
      //zigbeeStated++;    //测试数字,记得删除
    }
    Serial.println("按钮唤醒执行完毕");
    delay(50);
    sleep_cpu();
  } else {
    if (shu >= SLEEP_TIME) {
      Serial.println("记时10min");
      reportTemp();
      shu = 0;
      delay(50);
      sleep_cpu();
    } else {
      if ( shu >= pirshu + pirTimeSet && shu + pirTimeSet < SLEEP_TIME) {
        reportPirNone();
      }

      Serial.print("狗循环:");
      Serial.println(shu);
      Serial.print("唤醒人:");
      Serial.println(wakeby);
      delay(10);
      Serial.print("起始值:");
      Serial.println(pirshu);
      delay(50);
      sleep_cpu();//进入休眠状态，从此处开始进入休眠。
      //这里不需要喂狗。目的就是等狗超时后执行唤醒函数。
    }
  }
}

void sleepHandler(void)
{
  reportPir();
  wakeby = 1;
  if (shu + pirTimeSet >= SLEEP_TIME) {    //存下看门狗当前记数,如果加上设置的保持时间已经大于看门狗记数总数则户数为0
    pirshu =  shu - SLEEP_TIME;
  }
  else {
    pirshu = shu;
  }
  //delay(100);
}
void sleepHandlera(void)
{
  Serial.println("按钮");
  wakeby = 2;
  //delay(100);
}

//温湿度上报
void reportTemp(void)
{
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
  delay(50);
  batterylevel();    //上报电池

}

//PIR上报
void reportPir(void)
{

  int16_t lightLevel = analogRead(LIGHT_SENSOR_ANALOG_PIN);  //光敏的值
  if (lightLevel != lastlux) {
    lastlux = lightLevel;
    Serial.print("光线: ");
    Serial.println(lightLevel);
  }
  Serial.println("有人");
  delay(50);
}

//PIR_None上报
void reportPirNone(void)
{
  Serial.println("没人了");
  pirshu = SLEEP_TIME;
}

//电池检测
void batterylevel(void)
{
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
