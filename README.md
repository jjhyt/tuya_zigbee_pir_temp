������Ӳ����Դ��ַ:https://oshwhub.com/520world/tu-ya-di-gong-hao-ren-ti-wen-shi-du
����ʹ�ó���Ϊ tuya-zigbee-pir-sht30-2 �����Դ��!

ѧϰ�ʼ�:

=============================��ֲ��ʼ!!!!!!!!!

mcu_api.cpp

1.uart_receive_input  ֱ��ע��error

2.zigbee_protocol_init ע��error

3.zigbee_uart_service  ֱ��ע��error

mcu_api.h  ��

protocol.cpp

1.����������:

-----------
#include "Arduino.h"

#include "zigbee.h"

------------

2.uart_transmit_output  д��

--------------
Serial.write(value);
��ע�͵�error
--------------------

3.all_data_update ��������ע�͵�,�����������Ҳ������

4.dp_download_pir_sensitivity_handle  ����Ҫдȡ�÷��������õ�PIR�����Ⱥ��MCU����
������������������ø�Ϊ��ʪ���ϱ�ʱ������:0=10����,1=20����,2=30����,Ĭ��30����

--------------SLEEP_TIME = 10;      

// �ϱ���ʪ�ȼ��ʱ��,���Ź�����Ϊ8��,���ϱ�ʱ��Ϊ������*8�� 0=75=10Min 1=150=20Min 2=225=30Min
���������մ���?????????????????????

-------------
5.dp_download_pir_time_handle  

����Ҫдȡ�÷��������õ�PIR����ʱ����MCU����:����:0=30��,1=60��,2=120��,Ĭ��30��

--------------pirTimeSet    //PIR����ʱ������,Ĭ��0=4*8=32�� 1=8*8 2=15*8
���������մ���?????????????????????
-------------

6.zigbee_work_state_event �������ж�zigbee������״̬������,��������Ҫ�õ���!

--------------zigbeeStated       //zigbee����״̬ 1=δ����,2=������,3=������,4=����
���������մ���??????????����CPP�������֪����ô����,ֻ�ܶ�д�ڴ���!
-------------

7.mcu_write_rtctime ʱ��Ĵ�����,ע�͵�error

8.zigbee_test_result   testģʽ����صĲ���,ע�͵�error

9.reveived_mcu_ota_data_handle   ע�͵�error

10.ota_fw_data_handle             ע�͵�error,��ע��Ҳ��Ҫ��,���ò���.

protocol.h

û������,���������������DP�㳣��,д����ʱ���Բο�

system.cpp

1.#include "zigbee.h" 
���:(�����������,��֪��ʲôԭ��˵�������û������?Ϳѻ�ĳ���д��������?)
SYSTEM_EXTERN unsigned char current_mcu_pid[8];	///< current mcu pid

2.init_nwk_paremeter  ע�͵�error,���Ӧ����ǿ�ƶ���zigbee�ŵ���,��ע�͵�Ҳ��,�ò���

3.mcu_set_zigbee_nwk_parameter  ע�͵�error,��ע�͵�Ҳ��,�ò���

system.h

ע�͵������  SYSTEM_EXTERN _calendar_obj _time; ���protocol.h�е�#define    SUPPORT_MCU_RTC_CHECK
���õĺ�һ�ַ��������!Ϳѻ����д��������?
ע�͵�������������3�ж����  },��������!Ϳѻ�Ĵ�?

zigbee.h  

ע�͵�error,
#include "Arduino.h"

============================��Ҫ!!

ȥ�� protocol.cpp �� system.cpp DOWNLOAD_CMD_S ǰ�� const,���������ǲ���
�ο� https://developer.tuya.com/cn/docs/iot/device-development/tuya-development-board-kit/tuya-sandwich-evaluation-kits/development-guide/arduino-implements-simple-demo?id=K96yyxgetltzj
Ϳѻ��ǿ��ת������!system.cpp

mcu_api.cpp ��ĺ���

unsigned long my_strlen  ���������,��ʾ������NULL����long����,�Ҹĳ� return 0;
�������Ҳ���Ա����,�����г�����ʾ!

==================================================================
����:�㶨SDK��ֲ!!!!!!!!!!!!!!!!!!!!!!!!!

������ʾ:protocol.cpp��
dp_download_handle  ����д�յ������·��Ĵ���case
ret = dp_download_dout_1_handle(value,length);��dp_download_dout_1_handleд������


============================\
zigbee����״̬�ο���system.h��
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
Ӳ����·��ʹ�õ�����:
A1  ��ز���
A2  ����
D3  ����
D2  ��λ��ť
D5  ָʾ��
D4  ����ģ��


�ο�����:
---------
Serial.print(78, BIN) �õ� "1001110"
Serial.print(78, OCT) �õ� "116"
Serial.print(78, DEC) �õ� "78"
Serial.print(78, HEX) �õ� "4E"
Serial.print(1.23456, 0) �õ� "1"
Serial.print(1.23456, 2) �õ� "1.23"
Serial.print(1.23456, 4) �õ� "1.2346"
Serial.print('N') �õ� "N"
Serial.print("Hello world.") �õ� "Hello world."
----------

void setup() {
  Serial.begin(9600);
  while(Serial.read()>= 0){}//clear serialbuffer
}
 
void loop() {
   if (Serial.available() > 0) {
    delay(100); // �ȴ����ݴ���
    int numdata = Serial.available();
    Serial.print("Serial.available = :");
    Serial.println(numdata);
  }
  while(Serial.read()>=0){} //��մ��ڻ���

}
---------------------��ȡ����,ÿ�ζ�һ���ֽڲ��Ѷ�����ɾ��

char comchar;
 
void setup() {
  Serial.begin(9600);
  while(Serial.read()>= 0){}/ /clear serialbuffer
}
 
void loop() {
  // read data from serial port
 
  while(Serial.available()>0){
    comchar = Serial.read();//�����ڵ�һ���ֽ�
    Serial.print("Serial.read: ");
    Serial.println(comchar);
    delay(100); 
    }
}
-------------------
Serial.readBytes(buffer,length);
˵��
�Ӵ��ڶ�ȡָ������length���ַ�����������buffer��

�﷨
Serial.readBytes(buffer,length);

����
buffer: �������

length:�趨�Ķ�ȡ����

����
���ش��뻺����ַ�����0��ʾû����Ч���ݡ�

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
Serial.readString()��
˵��
�Ӵ��ڻ�������ȡȫ�����ݵ�һ���ַ����ͱ�����

�﷨
Serial.readString();

����
None

����
���شӴ��ڻ������ж�ȡ��һ���ַ�����

ʾ��

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
˵��
����������ݺ�����д���������ݵ����ڡ�

�﷨ 
Serial.write(val) 
Serial.write(str) 
Serial.write(buf, len)

���� 
val: �ֽ� 
str: һ���ֽ� 
buf: �ֽ����� 
len: buf�ĳ���

���� 
�ֽڳ���

ʾ��

void setup(){ 
Serial.begin(9600); 
}
void loop(){ 
  Serial.write(45); // send a byte with thevalue 45 
  int bytesSent = Serial.write(��hello��); //sendthe string ��hello�� and return the length of the string. 
}
----------------------------
�жϺ���
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


Mysensors��дEEPROM  

#include "EEPROM.h"
arduino��1.6.2��ʼ֧�ֽ�float��int���ַ���д�롢����EEPROM��
�̳̼�:
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

�״�github push:
1.�½�Զ�ֿ̲�
���������git���½�һ���ֿ⣬����Զ�ֿ̲�ĵ�ַ


2.git init
ʹ�� git init ���� ����ʼ��һ��git ���زֿ��ʱ���ڱ��ش���һ�� .git ���ļ��У�һ������ļ��������ص�

3.git remote add origin https://*****************************
ʹ��git remote add origin https......(�ո��½�Զ�ֿ̲�ĵ�ַ) �����Զ�ֿ̲⵽����

4.git add .
ʹ��git add .  ��. ��ʾ���еģ����� git add + �ļ��� �����ļ����浽������

5.git status
ʹ��git status����鿴����Ҫ�ϴ��ļ���״̬

6.git config user.name '****'
 git config user.email '****'

������git��ʹ���˻�������

7.git commit -m 'First'
ʹ��git commit -m ������ӵ��ļ����������� ������ļ�����

8.git push -u origin --all
���͵�Զ�ֿ̲�
