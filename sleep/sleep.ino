
#include <avr/sleep.h>
#include <avr/wdt.h>

#define ANNIU_PIN 2   // 配网按钮，d2（拉高）
#define PIR_PIN 3     // PIR人体触发，d3（拉高）
int shu=0;            //看门狗唤醒记数
int wakeby;           //记录哪个中断唤醒 1=PIR,2=按钮,3=看门狗

ISR(WDT_vect){
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
  MCUSR &= ~(1<<WDRF);  //清除复位标志
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  //设置新的看门狗超时时间
  WDTCSR = bb;
  //设置为定时中断而不是复位
  WDTCSR |= _BV(WDIE); 
  //别忘了设置【看门狗唤醒执行函数】
}

void setup() {
  pinMode(ANNIU_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);              //设置睡眠模式
  int intNum = digitalPinToInterrupt(PIR_PIN);       //普通IO编号转换成中断号
  int intNuma = digitalPinToInterrupt(ANNIU_PIN);
  sleep_enable();                                    //开启休眠功能
  attachInterrupt(intNum,sleepHandler,FALLING);      //开启中断
  attachInterrupt(intNuma,sleepHandlera,FALLING);
  //ACSR |=_BV(ACD);//关掉ACD，据说很省电。不知道唤醒以后要不要重新开，怎么开？
  //ADCSRA=0;//关掉ADC，据说很省电。不知道唤醒以后要不要重新开，怎么开？
  //按照官方解释，sleep_enable()最好写在中断(attachInterrupt())前，防止中断在开始休眠前就提前释放而造成休眠后无法唤醒。
  //开始设置看门狗中断，用来唤醒。   
  wdt_setup(9);
}

void loop() {
  if (shu>=7){
    Serial.println("记时10min");     
    shu=0;
    delay(50);
    sleep_cpu();
  }else{
    
    Serial.print("狗循环:");
    Serial.println(shu);
    Serial.print("唤醒人:");
    Serial.println(wakeby);
    delay(50);
    sleep_cpu();//进入休眠状态，从此处开始进入休眠。
    //这里不需要喂狗。目的就是等狗超时后执行唤醒函数。
  }
}

void sleepHandler(void)
{
  Serial.println("有人");
  wakeby = 1;
  //delay(100);
}
void sleepHandlera(void)
{
  Serial.println("按钮");
  wakeby = 2;
  //delay(100);
}
