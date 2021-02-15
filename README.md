本工程硬件开源地址:https://oshwhub.com/520world/tu-ya-di-gong-hao-ren-ti-wen-shi-du
最终使用程序为 tuya-zigbee-pir-sht30-2 里面的源码!

学习笔记:

=============================移植开始!!!!!!!!!

mcu_api.cpp

1.uart_receive_input  直接注释error

2.zigbee_protocol_init 注释error

3.zigbee_uart_service  直接注释error

mcu_api.h  无

protocol.cpp

1.加如下引用:

-----------
#include "Arduino.h"

#include "zigbee.h"

------------

2.uart_transmit_output  写入

--------------
Serial.write(value);
并注释掉error
--------------------

3.all_data_update 这里整段注释掉,反正这个函数也不调用

4.dp_download_pir_sensitivity_handle  这里要写取得服务器设置的PIR灵敏度后的MCU操作
这里我想把灵敏度设置改为温湿度上报时长设置:0=10分钟,1=20分钟,2=30分钟,默认30分钟

--------------SLEEP_TIME = 10;      

// 上报温湿度间隔时间,看门狗周期为8秒,故上报时间为设置数*8秒 0=75=10Min 1=150=20Min 2=225=30Min
代码先留空待填?????????????????????

-------------
5.dp_download_pir_time_handle  

这里要写取得服务器设置的PIR保持时间后的MCU操作:设置:0=30秒,1=60秒,2=120秒,默认30秒

--------------pirTimeSet    //PIR保持时间设置,默认0=4*8=32秒 1=8*8 2=15*8
代码先留空待填?????????????????????
-------------

6.zigbee_work_state_event 这里是判断zigbee的在网状态处理函数,程序里是要用到的!

--------------zigbeeStated       //zigbee网络状态 1=未入网,2=入网中,3=已入网,4=出错
代码先留空待填??????????两个CPP间变量不知道怎么传递,只能读写内存了!
-------------

7.mcu_write_rtctime 时间的处理函数,注释掉error

8.zigbee_test_result   test模式中相关的操作,注释掉error

9.reveived_mcu_ota_data_handle   注释掉error

10.ota_fw_data_handle             注释掉error,不注释也不要紧,我用不到.

protocol.h

没东西改,这里面申明了你的DP点常数,写程序时可以参考

system.cpp

1.#include "zigbee.h" 
后加:(否则编译会出错,不知道什么原因说这个函数没有申明?涂鸦的程序写的有问题?)
SYSTEM_EXTERN unsigned char current_mcu_pid[8];	///< current mcu pid

2.init_nwk_paremeter  注释掉error,这个应该是强制定义zigbee信道的,不注释掉也行,用不到

3.mcu_set_zigbee_nwk_parameter  注释掉error,不注释掉也行,用不到

system.h

注释掉出错的  SYSTEM_EXTERN _calendar_obj _time; 或打开protocol.h中的#define    SUPPORT_MCU_RTC_CHECK
我用的后一种方法解决的!涂鸦这里写的有问题?
注释掉出错的最后倒数第3行多余的  },否则会出错!涂鸦的错?

zigbee.h  

注释掉error,
#include "Arduino.h"

============================重要!!

去掉 protocol.cpp 和 system.cpp DOWNLOAD_CMD_S 前的 const,否则编译就是不过
参考 https://developer.tuya.com/cn/docs/iot/device-development/tuya-development-board-kit/tuya-sandwich-evaluation-kits/development-guide/arduino-implements-simple-demo?id=K96yyxgetltzj
涂鸦改强制转换函数!system.cpp

mcu_api.cpp 里的函数

unsigned long my_strlen  这个有问题,提示返回了NULL不是long类型,我改成 return 0;
这个不改也可以编译过,不过有出错提示!

==================================================================
到此:搞定SDK移植!!!!!!!!!!!!!!!!!!!!!!!!!

友情提示:protocol.cpp中
dp_download_handle  里面写收到服务下发的处理case
ret = dp_download_dout_1_handle(value,length);在dp_download_dout_1_handle写处理方法


============================\
zigbee几个状态参考在system.h里
#define ZIGBEE_NOT_JION					        0x00	//zigbee module not jion network
#define ZIGBEE_JOIN_GATEWAY					0x01	//zigbee module had jioned network 
#define ZIGBEE_JOIN_ERROR					0x02	//zigbee module network state error
#define ZIGBEE_JOINING					    	0x03	//zigbee module jioning 

#define REPORT_DATA_SUCESS	                        0x10	//translate sucess
#define REPORT_DATA_FAILURE	                        0x20	//translate failure 
#define REPORT_DATA_TIMEOUT	                        0x40	//translate timeout 
#define REPORT_DATA_BUSY                                0x80	//translate busy

#define RESET_ZIGBEE_OK		0x00	//rest zigbee success
#define RESET_ZIGBEE_ERROR  	0x01	//reset zigbee error
===================
硬件电路中使用的引脚:
A1  电池测量
A2  光敏
D3  人体
D2  复位按钮
D5  指示灯
D4  唤醒模组


参考资料:
---------
Serial.print(78, BIN) 得到 "1001110"
Serial.print(78, OCT) 得到 "116"
Serial.print(78, DEC) 得到 "78"
Serial.print(78, HEX) 得到 "4E"
Serial.print(1.23456, 0) 得到 "1"
Serial.print(1.23456, 2) 得到 "1.23"
Serial.print(1.23456, 4) 得到 "1.2346"
Serial.print('N') 得到 "N"
Serial.print("Hello world.") 得到 "Hello world."
----------

void setup() {
  Serial.begin(9600);
  while(Serial.read()>= 0){}//clear serialbuffer
}
 
void loop() {
   if (Serial.available() > 0) {
    delay(100); // 等待数据传完
    int numdata = Serial.available();
    Serial.print("Serial.available = :");
    Serial.println(numdata);
  }
  while(Serial.read()>=0){} //清空串口缓存

}
---------------------读取函数,每次读一个字节并把读过的删除

char comchar;
 
void setup() {
  Serial.begin(9600);
  while(Serial.read()>= 0){}/ /clear serialbuffer
}
 
void loop() {
  // read data from serial port
 
  while(Serial.available()>0){
    comchar = Serial.read();//读串口第一个字节
    Serial.print("Serial.read: ");
    Serial.println(comchar);
    delay(100); 
    }
}
-------------------
Serial.readBytes(buffer,length);
说明
从串口读取指定长度length的字符到缓存数组buffer。

语法
Serial.readBytes(buffer,length);

参数
buffer: 缓存变量

length:设定的读取长度

返回
返回存入缓存的字符数，0表示没有有效数据。

char buffer[18];
int numdata=0;
 
void setup() {
  Serial.begin(9600);
  while(Serial.read()>= 0){}//clear serial port
}
 
void loop() {
  // read data from serial port
  if(Serial.available()>0){
      delay(100);
      numdata = Serial.readBytes(buffer,3);
      Serial.print("Serial.readBytes:");
      Serial.println(buffer);    
    }
    // clear serial buffer
  while(Serial.read() >= 0){}
  for(int i=0; i<18; i++){
        buffer[i]='\0';

    }
}
-----------------------
Serial.readString()；
说明
从串口缓存区读取全部数据到一个字符串型变量。

语法
Serial.readString();

参数
None

返回
返回从串口缓存区中读取的一个字符串。

示例

String comdata = "";
 
void setup() {
  Serial.begin(9600);
  while(Serial.read()>= 0){} //clear serialbuffer
}
 
void loop() {
  // read data from serial port
  if(Serial.available()>0){
      delay(100);
      comdata = Serial.readString();
      Serial.print("Serial.readString:");
      Serial.println(comdata);
    }
    comdata = "";
}
----------------------
Serial.write();    
说明
串口输出数据函数。写二进制数据到串口。

语法 
Serial.write(val) 
Serial.write(str) 
Serial.write(buf, len)

参数 
val: 字节 
str: 一串字节 
buf: 字节数组 
len: buf的长度

返回 
字节长度

示例

void setup(){ 
Serial.begin(9600); 
}
void loop(){ 
  Serial.write(45); // send a byte with thevalue 45 
  int bytesSent = Serial.write(“hello”); //sendthe string “hello” and return the length of the string. 
}
----------------------------
中断函数
const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
  digitalWrite(ledPin, state);
}

void blink() {
  state = !state;
}
---------
const byte LED = 13;
const byte BUTTON = 2;

// Interrupt Service Routine (ISR)
void switchPressed ()
{
  if (digitalRead (BUTTON) == HIGH)
    digitalWrite (LED, HIGH);
  else
    digitalWrite (LED, LOW);
}  // end of switchPressed

void setup ()
{
  pinMode (LED, OUTPUT);  // so we can update the LED
  digitalWrite (BUTTON, HIGH);  // internal pull-up resistor
  attachInterrupt (digitalPinToInterrupt (BUTTON), switchPressed, CHANGE);  // attach interrupt handler
}  // end of setup

void loop ()
{
  // loop doing nothing 
} 
------------


Mysensors读写EEPROM  

#include "EEPROM.h"
arduino从1.6.2开始支持将float、int、字符串写入、读出EEPROM。
教程见:
https://www.geek-workshop.com/forum.php?mod=viewthread&tid=14659&ordertype=1
#include <EEPROM.h>

struct MyObject{
  float field1;
  byte field2;
  char name[10];
};

void setup(){

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  float f = 123.456f;  //Variable to store in EEPROM.
  int eeAddress = 0;   //Location we want the data to be put.
  
  
  //One simple call, with the address first and the object second.
  EEPROM.put( eeAddress, f );
  
  Serial.println("Written float data type!");
  
  /** Put is designed for use with custom structures also. **/
  
  //Data to store.
  MyObject customVar = {
    3.14f,
    65,
    "Working!"
  };

  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  
  EEPROM.put( eeAddress, customVar );
  Serial.print( "Written custom data type! \n\nView the example sketch eeprom_get to see how you can retrieve the values!" );
}

首次github push:
1.新建远程仓库
首先在你的git上新建一个仓库，记下远程仓库的地址


2.git init
使用 git init 命令 ，初始化一个git 本地仓库此时会在本地创建一个 .git 的文件夹，一般这个文件夹是隐藏的

3.git remote add origin https://*****************************
使用git remote add origin https......(刚刚新建远程仓库的地址) ，添加远程仓库到本地

4.git add .
使用git add .  （. 表示所有的）或者 git add + 文件名 ，将文件保存到缓存区

5.git status
使用git status命令查看你需要上传文件的状态

6.git config user.name '****'
 git config user.email '****'

配置你git的使用账户和邮箱

7.git commit -m 'First'
使用git commit -m “新添加的文件内容描述” ，添加文件描述

8.git push -u origin --all
推送到远程仓库
