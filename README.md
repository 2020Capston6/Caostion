## 캡스톤 디자인 프로젝트 

  공식 프로젝트 명 : 비전센서, 거리센서, 터치센서를 이용한 시각장애인 신호등 자동 감지 스마트 흰지팡이    
  팀명 :  알아서 잘하자    
  제품명 : 내가봐

  
## What's Problem? 
   
  흰지팡이는 시각장애인이 길을 찾고 활동하는데 사용하는 가장 보편된 도구이며 전세계적으로 시각장애인의 자립과 성취를 나타내는 공인된 상징입니다. 이런 흰지팡이의 개념은 1차 세계대전 당시 채택되었으며 지금까지 그때의 원형을 그대로 사용하고 있습니다. 현재 보편화된 흰지팡이는 단순히 흰색 막대기로써 시각장애인들이 직접 땅을 치며 보행 중 장애물이나 유도블럭(점자블록)을 파악하는등 단순한 정보의 파악만을 도와줍니다. 하지만 이마저도 시각장애인의 보행숙련도에 의존하기 때문에 시각장애인들은 따로 보행할수 있는 능력을 길러야 한다고 합니다. 저희는 이러한 문제들을 개선하고자 흰지팡이에 Iot 기술을 접목시켜 다양한 방법으로 보행을 보조하여 사회적 약자인 시각장애인들이 세상을 보다 편하고, 안전하게 보행했으면 하는 바람을 담아 이 프로젝트를 진행 하게 되었습니다..
  
## About Our Goal
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/about1.PNG" width="600" height="300"></p>
  보통 시각장애인들은 흰지팡이를 이용하여 위와 같은 유도블럭의 도움을 받아 보행을 합니다. 길쭉한 유도블럭은 그쪽 방향으로 서행하라는 의미이며, 촘촘한 점 유도블럭은 멈추라는 의미입니다. 횡단보도 등이 나오면 이러한 멈추라는 의미의 유도블럭이 나옵니다. 하지만 흰지팡이와 유도블럭만으로는 보행에 한계가 있습니다. 저희는 다음과 같은 문제점들에 대한 해결책을 제시하려고 합니다.
  
  ### Problem1
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/about2.PNG" width="500" height="250"></p>
  보행자 신호등 옆에는 다음과 같이 시각장애인들을 위한 버튼이 있습니다. 이는 신호등의 색을 청각 정보를 전달해줍니다. 그러나 시각장애인들은 이마저도 본인들이 찾아야 하기 때문에 어려움이 있습니다. 이 문제를 비전센서를 통해 해소하고자 합니다.
  
  ### Problem2
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/about3.PNG" width="500" height="250"></p>
  또한 다음과 같이 흰지팡이를 이용하더라도 모든 장애물들을 감지하기에는 한계가 있습니다. 이 문제를 초음파 거리센서를 통해 해소하고자 합니다.
  
  ### Problem3
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/about4.PNG" width="250" height="300"></p>
  또한 시각장애인들은 도움이 필요한 경우, 주변에 도움을 줄 수 있는 사람이 있는지, 어디에 있는지를 파악하기가 힘듭니다. 따라서 위와 같이 도움이 필요한 경우를 표출하기도 하지만, 보행자들은 이 의미를 파악하기 힘듭니다. 또한 정말 위급하며 주변에 도와줄 사람이 없는 경우에도 큰 문제가 됩니다. 이 문제를 터치센서와 오디오 장치, GPS 모듈을 통해 해소하고자 합니다.
    
## GOAL
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/ggoal.PNG" width="1013" height="440"></p>    
  저희의 휠지팡이는 기존의 시각장애인이 횡단보도 이용시 불편함을 해소하여 편히 이용할 수 있도록 합니다.
  
## GOAL1
기술 데모
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/goal1.PNG" width="400" height="250"></p>
DEMODAY (최종 구현)
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/ggoal1.PNG" width="600" height="350"></p>
  비전센서를 이용하여 보행자가 신호등에 도착했을 경우, 파란불이 들어왔을 때 손잡이에 달린 진동 센서를 통해 정보를 전달해줍니다.

## GOAL2

기술 데모
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/goal2.PNG" width="400" height="250"></p>
DEMODAY (최종 구현)
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/ggoal2.PNG" width="600" height="350"></p>
  초음파 거리센서를 이용하여 전방에 접근하는 보행자나 부딪칠만한 장애물이 있으면 검출하여 버저의 소리를 통해 위험 경보를 전달해줍니다.

## GOAL3

기술 데모
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/goal3.PNG" width="400" height="250"></p>
DEMODAY (최종 구현)
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/ggoal3.PNG" width="600" height="400"></p>
  터치센서를 이용하여 시각장애인이 위험에 처한 경우 도움을 요청하는 소리를 출력합니다. 또한 위치정보를 서버에 전송하여 위급한 상황인 경우에 즉각적인 도움도 줄 수 있도록 합니다.  위치정보는 DB에 저장하여 추후에 위험이 많이 일어난 장소에 대한 보수 및 시설물 설치가 가능하도록 돕습니다.

## Detail
  
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/detail.PNG" width="1000" height="600"></p>
  
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/detail2.PNG" width="800" height="350"></p>
  
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/modules.PNG" width="600" height="350"></p>
 
  
## Expected Achievement
  
  비전센서를 이용해 hue based color filtering 알고리즘을 통하여 신호등의 색을 검출합니다. 파란불이여서 횡단보도 서행이 가능한 경우 손잡이의 진동센서를 이용해 사용자에게 정보를 전달합니다. 파란불을 검출하고 진동정보를 검출하여 알려주기 때문에 횡단보도 서행이 더욱 안전해지고 편리해집니다.

초음파 거리 센서를 이용해 근접한 물체가 있는 경우 경고를 위해 버저를 울립니다. 지팡이로 제대로 파악하지 못하는 장애물이나, 앞을 보고 걷지 않는 보행자 등 발생할 수 있는 여러가지 충돌 상황을 방지합니다. 다소 낮은곳에 있는 장애물도 파악하기 위해 초음파 거리센서는 지팡이의 하단에 부착합니다. 150m 이내, 100m 이내, 50m 이내에 장애물이 있는 경우 버저에서 출력되는 소리를 다르게 합니다. 가까운 장애물일수록 버저가 더 빠르게 소리를 냅니다. 소리에 따라서 지팡이 사용자도 장애물의 정보를 파악할 수 있지만, 전방을 주시하지 않으며 다가오던 보행자도 소리를 듣고 상황을 파악 할 수 있습니다.

터치센서를 터치한 경우, 오디오 소리를 출력하여 주변에 도움을 요청하며, GPS 모듈을 통해 위치정보를 서버 DB에 저장하여 위험에 처한 장소를 알 수 있도록 합니다. 한번의 도움 요청이 아닌 지속적인 도움 요청은 급박한 상황이라고 판단할 수 있습니다. 이를 통해 다급하게 도움이 필요한 상황에 있어서는 즉각적인 도움을 줄 수 있으며, 위험 다발 지역을 저장하여 해당되는 위치에 시각장애인 보조 장치들을 마련하거나 유지보수하는 등으로 데이터를 이용할 수 있습니다. 따라서 데이터의 축적은 시각장애인들의 안전에 큰 도움이 됩니다.
  
## 구현 내용
  
  Expected Achievement 에서 계획했던 것들을 모두 구현하였습니다. 서버에 저장된 정보 (도움이 필요한 경우 터치센서를 눌렀을 경우 전송된 GPS 정보)
  <img src="http://cscp2.sogang.ac.kr/CSE4186/CSE4186/UserData/Server.png">
  
## Our Product
### 앞
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/Front.jpg" width="430" height="600"></p>
  
### 뒤
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/Back.jpg"  width="430" height="600"></p>
  
## 기술 데모 시연
  ### 비전센서
  https://www.youtube.com/watch?time_continue=10&v=WdmUXlJQAFE&feature=emb_logo
  
  https://www.youtube.com/watch?time_continue=4&v=fvFuvFG0NY0&feature=emb_logo
  
  ### 거리센서
  https://www.youtube.com/watch?v=KZO6JP7cUYI&feature=emb_logo
  
  ### 터치센서
  https://www.youtube.com/watch?v=t7IN3Yq9SIw&feature=emb_logo
 

## DEMODAY 시연 (최종 시연)
  ### 비전센서
  신호등 색 판별
https://www.youtube.com/watch?v=B_iiDCr6e9M
  
  ### 거리센서
  일반적인 지팡이인 경우 판단하지 못할 수 있는 장애물
https://www.youtube.com/watch?v=J1JPnyd9oag

장애물 감지
https://www.youtube.com/watch?v=P7-h86daFy4

보행자 감지
https://www.youtube.com/watch?v=lORg5qbO4WI
  
  ### 터치센서
  터치 센서를 이용한 도움 요청
https://www.youtube.com/watch?v=06cM8VkekFg

위치정보 서버를 통한 DB 저장
https://www.youtube.com/watch?v=jRz4xJNekdY

## 코드

### 비전센서 (arduino uno)
```cpp
#include <SPI.h>
#include <Pixy2.h>
#include "SoftReset.h"

Pixy2 pixy;

const int VibPin =  2; 

void setup(){
  pixy.init();
  Serial.begin(9600);
  Serial.print("Starting…\n");
  pinMode (VibPin, OUTPUT);
  digitalWrite(VibPin, LOW);
}

void loop(){
  static int i=1;
  int j;
  uint16_t blocks;
  Serial.println(i);
  if(i%50==0){
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks){
    Serial.print("Detected ");
    int val=pixy.ccc.blocks[j].m_signature;
    Serial.println(pixy.ccc.blocks[0].m_signature);
    if(pixy.ccc.blocks[0].m_signature == 2){
      digitalWrite(VibPin, HIGH);
      delay(4000);
      pixy.init();
      soft_restart();
    }
  }
  i=0;
  }
  i++;
}
```

### 초음파 거리센서, 터치센서 (esp32)
```cpp
#include <TinyGPS++.h> // Library über http://arduiniana.org/libraries/tinygpsplus/ downloaden und installieren
#include <HardwareSerial.h> // sollte bereits mit Arduino IDE installiert sein
#include "EEPROM.h" // sollte bereits mit Arduino IDE installiert sein
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#define EEPROM_SIZE 128
#define TASK_SERIAL_RATE 1000 // ms
#define C 262 // 도 
#define D 294 // 레 
#define E 330 // 미 
//#define F 349 // 파 
#define G 392 // 솔 
#define A 440 // 라 
#define B 494 // 시
#define SOUND_PWM_CHANNEL   0
#define SOUND_RESOLUTION    8 // 8 bit resolution
#define SOUND_ON            (1<<(SOUND_RESOLUTION-1)) // 50% duty cycle
#define SOUND_OFF           0
#define d_dis 30

uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;
clock_t start,endt;
char _get[200];
int notes[25] = { G, G, A, A, G, G, E, G, G, E, E, D, G, G, A, A, G, G, E, G, E, D, E, C };

WiFiMulti WiFiMulti;
HTTPClient http;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};
//////////////
/// pin 번호 //
//////////////

const int trigPin = 32;  // 초음파 Trig
const int buz = 33;      // 초음파 소리
const int touch=34;      // 터치센서
const int echoPin = 35;  // 초음파 echo
const int PlayE = 25;     // 스피커

/////////////
/////////////
/////////////
int check;
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}
void tone(int pin, int frequency, int duration)
{
  ledcSetup(SOUND_PWM_CHANNEL, frequency, SOUND_RESOLUTION);  // Set up PWM channel
  ledcAttachPin(pin, SOUND_PWM_CHANNEL);                      // Attach channel to pin
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
  delay(duration);
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
}
void GPS_CALL(){
  static int p0 = 0;
  Serial.println("GPS START......");
  // GPS Koordinaten von Modul lesen
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();

  // Aktuelle Position in nichtflüchtigen ESP32-Speicher schreiben
  long writeValue;
  writeValue = gpsState.originLat * 1000000;
  EEPROM_writeAnything(0, writeValue);
  writeValue = gpsState.originLon * 1000000;
  EEPROM_writeAnything(4, writeValue);
  writeValue = gpsState.originAlt * 1000000;
  EEPROM_writeAnything(8, writeValue);
  EEPROM.commit(); // erst mit commit() werden die Daten geschrieben

  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;

  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  if (gps.satellites.value() > 4) {
    gpsState.dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gpsState.originLat, gpsState.originLon);

    if (gpsState.dist > gpsState.distMax && abs(gpsState.prevDist - gpsState.dist) < 50) {
      gpsState.distMax = gpsState.dist;
    }
    gpsState.prevDist = gpsState.dist;

    if (gps.altitude.meters() > gpsState.altMax) {
      gpsState.altMax = gps.altitude.meters();
    }

    if (gps.speed.kmph() > gpsState.spdMax) {
      gpsState.spdMax = gps.speed.kmph();
    }

    if (gps.altitude.meters() < gpsState.altMin) {
      gpsState.altMin = gps.altitude.meters();
    }
  }


  if (nextSerialTaskTs < millis()) {
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    check=gps.location.lat();
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());
    Serial.print("Sats=");  Serial.println(gps.satellites.value());
    Serial.print("DST: ");
    Serial.println(gpsState.dist, 1);
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    sprintf(_get, "http://3.83.212.42:8000/arduino/?LAT=%f&LONG=%f&ALT=%f&ID=%d",gps.location.lat(),gps.location.lng(),gps.altitude.meters(), 20151556);
  }
  Serial.println("end");
}

void setup() {
  // put your setup code here, to run once:`
  Serial.begin(115200);
  
  WiFiMulti.addAP("Bohhh", "hongsu3637");
  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi... ");

  while(WiFiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);

  //GPS
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  while (!EEPROM.begin(EEPROM_SIZE)) {
    true;
  }
  long readValue;
  EEPROM_readAnything(0, readValue);
  gpsState.originLat = (double)readValue / 1000000;

  EEPROM_readAnything(4, readValue);
  gpsState.originLon = (double)readValue / 1000000;

  EEPROM_readAnything(8, readValue);
  gpsState.originAlt = (double)readValue / 1000000;



  ///Pin Input Ouput///
  //touch
  pinMode(touch, INPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(buz, OUTPUT);
  pinMode(PlayE,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //초음파 거리 센서
  digitalWrite(trigPin, LOW);
  digitalWrite(echoPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH);
  // 초음파의 속도는 초당 340미터를 이동하거나, 29마이크로초 당 1센치를 이동합니다.
  // 따라서, 초음파의 이동 거리 = duration(왕복에 걸린시간) / 29 / 2 입니다.
  float distance = duration / 29.0 / 2.0;
  Serial.println(distance);
  delayMicroseconds(2);
  if(0<distance && distance <d_dis+50){
    for(int i=0;i<5;i++){
      tone(buz,notes[0],2000/30);
    }
  }
  else if(d_dis+50<=distance && distance < d_dis+100){
    for(int i=0;i<5;i++){
      tone(buz,notes[0],2000/10);
    }
  }
  else if(d_dis+100< distance && distance < d_dis+150){
    for(int i=0;i<5;i++){
      tone(buz,notes[0],2000/3);
    }
  }
  
  //응급시 GPS 송신
  static int i=0;
  
  int touchValue = digitalRead(touch);
  //Serial.println(touchValue);
  if (touchValue == LOW){      //  터치됨
    if(i==3){
      check=0;
      digitalWrite(PlayE,HIGH);
      delay(500);
      digitalWrite(PlayE,LOW);
      for(int j=0;j<2000;j++){
      GPS_CALL();
      Serial.println(check);
      if(check!=0){
        http.begin(_get);
        int httpResponseCode = http.GET();
        http.end();
        break;
        }
      }
      i=0;
    }
    else i++;
    Serial.println("TOUTCHED");
  }
  else{
    Serial.println("NOT TOUCHED");
    i=0;
  }
  delay(1000);
}
```
 
## Teams
  
  팀 구성원 :  송민국  장승민  변홍수    
  <img src="https://github.com/2020Capston6/Capston/blob/master/img/1.jpg" width="150" height="190">
  <img src="https://github.com/2020Capston6/Capston/blob/master/img/2.jpeg" width="150" height="190">
  <img src="https://github.com/2020Capston6/Capston/blob/master/img/3.jpeg" width="150" height="190">    
  ### 역할

    * 송민국 (팀장)
    발표담당
    발표 PPT 제작
    MileStones 관리
    실험 계획 및 구체화 
    지팡이 물리적 메커니즘 담당
    비전센서 머신러닝 객체검출 담당 1
    지팡이 소프트웨어 연동 담당
    각 단계별 테스트 담당

    * 변홍수
    데모 시나리오 준비 담당
    esp 보드와 와이파이 연결 및 데이터 전송
    초음파 센서를 이용한 물체 감지 담당
    위험상황 경보 시 알림 작동 담당
    머신러닝에 필요한 데이터 제작
    데이터 베이스 담당

    * 장승민
    자료 조사
    우분투 서버 관리
    비전센서 머신러닝 객체검출 담당 2
    카메라 모듈 ESP보드 연동
    GPS 모듈 ESP보드 연동
    GPS 위험지역 정보 축적 담당

## Environment

 github, slack , visual studio code    
 github 중심의 pull request 방식으로 agile 개발한다
 주 계발 방법으로는 XP(eXtreme Programming)을 이용한다.
  ### 주 계발 방법으로는 XP(eXtreme Programming)을 이용한다.
+ 짧고 반복적인 개발 주기, 단순한 설계를 지향한다.
+ 릴리즈의 기간을 짧게 반복하면서 요구사항을 충분히 반영한다.
+ 핵심가치 : 의사소통 , 단순성 ,용기 ,존중 ,피드백
  
 
## MileStones 
  |주차|월|what did we do|
  |---|---|---|
  |1주차|4월 3주차|프로젝트 주제 선정 및 멘토님과 미팅|
  |2주차|4월 4주차|프로젝트 주제 수정 및 필요 센서, 부품 계획 및 주문|
  |3주차|5월 1주차|비전센서를 이용한 신호등 색 검출 구현|
  |4주차|5월 2주차|터치센서로 도움 요청, 초음파 거리 센서로 장애물 경고 구현|
  |5주차|5월 3주차|기술데모 발표 및 조별 설문|
  |6주차|5월 4주차|GPS 모듈을 통해 도움 요청시 위치정보 전송 구현|
  |7주차|6월 1주차|서버와 DB 구축 및 위치정보 저장|
  |8주차|6월 2주차|비전센서에 진동센서 연동, 터치센서에 오디오 모듈과 GPS 모듈 연동, 초음파 거리 센서에 버저 연동|
  |9주차|6월 3주차|센서들 흰지팡이에 최종적으로 연결 및 테스트 / 문서 및 발표 자료 작성, 마무리|
  |10주차|6월 4주차|Demo Day
  
## Demo Day Preparation : What will we demonstrate?
  ### 비전 센서, 진동 모터 모듈
  + 실제 신호등에서 파란불을 검출할 수 있는지를 확인한다.
  + 검출된 정보를 진동을 통해 손으로 전달한다. 파란불이 등장하면 진동을 통해서 사용자에게 전달한다.
  + 진동을 실제로 보이기 위해 진동 센서에 물을 담은 용기를 닿게 하여 물의 진동을 찍는다.
  ### 초음파 거리 감지 센서, 버저
  + 벽 밎 단순 지팡이로는 감지에 실패할 수 있는 장애물에서 시연을 한다.
  + 초음파 센서로 장애물들과 다가오는 보행자를 검출하여 일정거리 이내에 다가오면 버저에서 소리를 출력하여 경고를 전달한다.
  + 전방을 주시하지 않는 보행자와 마주했을 때 소리를 통해 지팡이 사용자만이 아니라 보행자도 상황을 알아차리는 시연을 보여준다.
  
  ### 터치 센서, 음성 녹음 및 플레이 모듈, GPS 모듈
  + 도움이 필요한 경우 터치 센서를 누르는 상황을 연출한다. 
  + 장애물에 의해 넘어져서 터치센서로 도움을 요청하며 소리가 들리는 것을 찍는다.
  + 터치 센서를 누르는 경우, 녹음된 소리를 플레이 모듈에서 출력한다.
  + 소리는 잘 들리지 않을 수 있으니, 상황의 소리를 녹음기로 녹음해서 동영상에 합치도록 한다.
  + 터치 센서를 누르는 경우, GPS 모듈을 통해 위치정보를 서버에 전송한다.
  + 실제로 서버를 통해 DB에 실시간으로 저장되는 정보를 찍어서 GPS 정보가 실시간으로 자동 저장되는 것을 보여준다.
  + 서버에 전송된 자료가 정확한 자료인지, 위도 경도로 지도에 검색해서 현재 위치와 비교하는 모습을 보여준다.
  
## 미래지향성 : What Can We More?    

###  1.성능이 더 좋은 비전 센서나 카메라를 사용하면 안정성을 극대화할 수 있습니다. 
  
   현재 아두이노에서 사용할 수 있는 비전센서들의 성능에는 한계가 있습니다. 실제로 실험해본 결과 거리가 먼 보행자 신호등의 색을 판별하기 매우 어려우며 화질이 좋지 않습니다. 저희 조는 현재 상황에서 이를 최대한 해결하기 위해 스마트폰 카메라 5배줌 확대경을 비전센서에 결합하여 횡단보도 반대편 등의 다소 먼 거리도 검출을 성공했습니다. 하지만 실제로 제품이 실생활에 도입되기 위해서는 더 정확한 정보를 도출하기 위해 성능이 더 좋은 카메라가 반드시 필요합니다. 성능이 더 좋은 카메라를 사용하면 비단 신호등뿐만이 아니라 보행자나 자동차 등 여러 가지 객체가 추가적으로 검출이 가능하며 전방만이 아닌 좌우 측면까지도 객체가 검출 가능할 것입니다.. 또한 도보 위의 위험요소는 시각장애인들의 생명에 영향을 미칠 수 있을 만큼 매우 위험합니다. 특히 도로가 나오기 직전의 위험요소는 시각장애인들이 원치 않게 도로로 넘어져서 더욱 큰 사고로 이어질 수 있습니다. 카메라의 성능이 더 좋다면 갑자기 꺼진 도로나 구덩이, 파손된 보도블럭 등 도보의 위험요소를 판단하여 경고하고 그 정보를 시설관리공단, 장애인협회 등에 자동 전송하여 조치가 취해질 수 있게끔 하여 안전성을 더욱 극대화할 수 있습니다.    

###  2.향상된 성능의 비전센서와 거리센서  

  비전센서의 성능이 향상되면, 비단 전방에 근접하는 물체에 대한 경고가 아니라, 실제로 어떠한 물체가 접근하는지를 파악하여 실제 위험요소를 정확히 알려줄 수 있습니다.
    
###  3.흰지팡이를 놓치게 되는 위급 상황 알림    
 
  시각장애인들에게 있어서 흰지팡이는 눈과도 같은 매우 중요한 도구입니다. 하지만 상황에 따라서 문제가 발생한 경우 이를 놓치는 사고가 발생할 수 있습니다. 시각장애인들은 지팡이 없이는 상황 판단이 힘들며, 실제로 손에서 벗어난 지팡이가 가까이 있더라도 찾는데 매우 큰 어려움을 겪을 수 있습니다. 따라서 주인과 일정 거리 이상 떨어진 경우이거나 손잡이가 손에서 벗어난 경우 소리가 나게 만들어 그 지팡이의 위치를 파악하게 해줄 수 있습니다.    

###  4.간편한 여러 기능들의 추가  
 
  저희의 제품은 시각장애인들을 돕기 위해 생소한 새로운 물건을 만든게 아니라 기존에 사용하던 흰지팡이에 여러 가지 기능을 집약적으로 추가하여 편의성와 간편함을 극대화했다는 장점이 있습니다. 따라서 이에 더욱 다양한 기능을 추가하여 지팡이 하나만으로도 편리한 외출을 할 수 있도록 도울 수 있습니다. 장애인 복지카드와 연동하여 대중교통을 이용할 때나 물건을 구매할 경우 간편하게 지팡이로 가능하게 할 수도 있습니다. 또한 현관문을 여는 경우 지팡이에 연결된 블루투스를 통해 자동으로 문을 열어주는 등 이외에도 다양한 기능을 추가할 수 있을 것으로 예측합니다.    
