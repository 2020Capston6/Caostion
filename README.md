## 캡스톤 디자인 프로젝트 

  공식 프로젝트 명 : 비전세서, 거리센서, 터치센서를 이용한 시각장애인 신호등 자동 감지 스마트 흰지팡이    
  팀명 :  알아서 잘하자    
  제품명 : 같이봐

  
## What's Problem? 
   
  흰지팡이는 시각장애인이 길을 찾고 활동하는데 사용하는 가장 보편된 도구이며 전세계적으로 시각장애인의 자립과 성취를 나타내는 공인된 상징입니다. 이런 흰지팡이의 개념은 1차 세계대전 당시 채택되었으며 지금까지 그때의 원형을 그대로 사용하고 있습니다.        
  현재 보편화된 흰지팡이는 단순히 흰색 막대기로써 시각장애인들이 직접 땅을 치며 보행 중 장애물이나 유도블럭(점자블록)을 파악하는등 단순한 정보의 파악만을 도와줍니다.
  하지만 이마저도 시각장애인의 보행숙련도에 의존하기 때문에 시각장애인들은 따로 보행할수 있는 능력을 길러야 한다고 합니다.
  저희는 이러한 문제들을 개선하고 흰지팡이에 Iot 기술을 접목시켜 다양한 방법으로 보행을 보조하여 사회적 약자인 시각장애인들이 세상을 보다 편하고, 안전하게 보행했으면 하는 바람을 담아 이 프로젝트를 진행 하게 되었습니다. 
  
## About Out Goal
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
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/goal.PNG" width="1013" height="440"></p>    
  저희의 휠지팡이는 기존의 시각장애인이 횡단보도 이용시 불편함을 해소하여 편히 이용할 수 있도록 합니다.
  
## GOAL1
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/goal1.PNG" width="400" height="250"></p>
  비전센서를 이용하여 보행자가 신호등에 도착함을 인지했을 경우, 빨간불인지 파란불인지 판별하여 청각적인 정보로 전달해줍니다.

## GOAL2
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/goal2.PNG" width="350" height="270"></p>
  초음파 거리센서를 이용하여 전방에 접근하는 보행자나 부딪칠만한 장애물이 있으면 검출하여 진동을 통해 위험 경보를 전달해줍니다.

## GOAL3
<p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/goal3.PNG" width="400" height="250"></p>
  터치센서를 이용하여 시각장애인이 위험해 처한 경우, 도움을 요청할 수 있도록 "도와주세요" 등의 소리를 출력하며 위치정보를 전송하여 위급한 상황인 경우에 즉각적인 도움도 줄 수 있도록 합니다.

## Detail
  
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/detail.PNG" width="1000" height="600"></p>
  
  <p align="center"><img src="https://github.com/2020Capston6/Capston/blob/master/img/detail2.PNG" width="800" height="350"></p>
 
  
## Expected Achievement
  
  비전센서를 이용해 CNN 객체검출 알고리즘을 통하여 신호등 및 접근 물체를 검출합니다.
  비전센서를 이용해 hue based color filtering 알고리즘을 통하여 신호등의 색을 검출합니다.
  해당 정보들을 본인에게 음성을 통해 제공합니다.
  
  초음파 거리 센서에 근접한 물체가 감지되면 위험을 진동을 통해 시각장애인에게 전달해줍니다.

  터치센서를 터치한 경우, 도와달라는 내용의 음성을 출력하며 GPS 모듈을 통해 위치정보를 저장하여 위험에 처한 장소를 알 수 있도록 합니다.
  이는 다급하게 도움이 필요한 상황에 있어서는 즉각적인 도움을 줄 수 있으며, 위험 다발 지역을 저장하여 해당되는 위치에 시각장애인 보조 장치들을 마련하거나 유지보수하는 등으로 데이터를 이용할 수 있습니다.
  따라서 데이터의 축적은 시각장애인들의 안전에 큰 도움이 됩니다.
  
## 구현 내용
  
  분석한 객체검출의 결과를 본인에게 음성 메세지를 통해서 알려줍니다.
  분석한 내용 및 위험들은 웹서버에 저장합니다.
  해당되는 정보는 판단 후에 디바이스에서 오디오 시스템으로 출력합니다.
  
## Our Product
  <img src="https://github.com/2020Capston6/Capston/blob/master/img/product.PNG" width="350" height="250"><img src="https://github.com/2020Capston6/Capston/blob/master/img/product1.PNG" width="200" height="250"><img src="https://github.com/2020Capston6/Capston/blob/master/img/product2.PNG" width="250" height="200">
  
## 시연
  ### 비전센서
  https://www.youtube.com/watch?time_continue=10&v=WdmUXlJQAFE&feature=emb_logo
  https://www.youtube.com/watch?time_continue=4&v=fvFuvFG0NY0&feature=emb_logo
  
  ### 거리센서
  https://www.youtube.com/watch?v=KZO6JP7cUYI&feature=emb_logo
  
  ### 터치센서
  https://www.youtube.com/watch?v=t7IN3Yq9SIw&feature=emb_logo
  

## 코드

### 비전센서 (arduino uno)
```cpp
#include <SPI.h>
#include <Pixy2.h>

Pixy2 pixy;

int red = 6;
int blu = 7;

void setup(){
  pixy.init();
  Serial.begin(115200);
  Serial.print("Starting...\n");
  pinMode (red, OUTPUT);
  pinMode (blu, OUTPUT);
}

void loop(){
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  digitalWrite (red, LOW);
  digitalWrite (blu, LOW);
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks){
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);

    if (i%50==0){
      int val=pixy.ccc.blocks[j].m_signature;
      Serial.println(val);
    
      if(pixy.ccc.blocks[0].m_signature == 1){
        digitalWrite (red, HIGH);
        delay (100);
      }
    
      if(pixy.ccc.blocks[0].m_signature == 2){
        digitalWrite (blu, HIGH);
        delay (100);
      }
    }
  }
}
```

### 초음파 거리센서, 터치센서 (esp32)
```cpp
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

const int trigPin = 32;
const int echoPin = 35;
const int dis_gnd = 33;

const int led = 26; 
const int led_gnd = 25; 

long duration;
int distance;

void tone(int pin, int frequency, int duration)
{
  ledcSetup(SOUND_PWM_CHANNEL, frequency, SOUND_RESOLUTION);  // Set up PWM channel
  ledcAttachPin(pin, SOUND_PWM_CHANNEL);                      // Attach channel to pin
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
  delay(duration);
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
}

int touch = 18;  // 터치센서 핀 설정
int buz = 19;       // LED 핀 설정
int tempo = 200;  
int notes[25] = { G, G, A, A, G, G, E, G, G, E, E, D, G, G, A, A, G, G, E, G, E, D, E, C };
void setup() {
    pinMode(led_gnd, OUTPUT);   
    pinMode(led, OUTPUT);   
    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT); 
    pinMode(dis_gnd, OUTPUT);
  Serial.begin(115200);
  pinMode(buz, OUTPUT);  
  pinMode(touch, INPUT);
}
 
void loop() {
  //터치값 읽음
      digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    digitalWrite(led, 0);
    digitalWrite(led_gnd, 0);
    digitalWrite(dis_gnd, 0);
    
    distance= duration*0.034/2;
    if(distance < 100){
      digitalWrite(led, 1);
    }
   
    Serial.print("Distance: ");
    Serial.println(distance);

  int touchValue = digitalRead(touch);
  static int i=0;
  if (touchValue == LOW){      // 터치됨
       tone(buz,notes[i%25],tempo);
       i++;
       //delay(300);
       Serial.println("TOUCHED");
   }
   else {                      //터치 안됨
    Serial.println("NOT TOUCHED");
    i=0;
  }
}
```
 
## Teams
  
  팀 구성원 :  송민국  장승민  변홍수    
  <img src="https://github.com/2020Capston6/Capston/blob/master/img/1.jpg" width="150" height="190">
  <img src="https://github.com/2020Capston6/Capston/blob/master/img/2.jpeg" width="150" height="190">
  <img src="https://github.com/2020Capston6/Capston/blob/master/img/3.jpeg" width="150" height="190">    
  ### 역할

    * 송민국 (팀장)
    발표 PPT 제작
    MileStones 관리
    실험 계획 및 구체화 
    지팡이 물리적 메커니즘 담당
    비전센서 머신러닝 객체검출 담당
    지팡이 소프트웨어 연동 담당
    각 단계별 테스트 담당

    * 변홍수
    esp 보드와 와이파이 연결 및 데이터 전송
    비전센서 머신러닝 객체검출 담당
    머신러닝에 필요한 데이터 제작
    데이터 베이스 담당
    우분투 서버 관리
    발표담당

    * 장승민
    자료 조사
    데모 시나리오 준비 담당
    초음파 센서를 이용한 물체 감지 담당
    위험상황 경보 시 알림 작동 담당
    카메라 모듈 ESP보드 연동
    GPS 모듈 ESP보드 연동
    GPS 위험지역 정보 축적 담당

## Environment

  github, slack , visual studio code    
  github 중심의 pull request 방식으로 agile 개발한다
  ### 주 계발 방법으로는 XP(eXtreme Programming)을 이용한다.
+ 짧고 반복적인 개발 주기, 단순한 설계를 지향한다.
+ 릴리즈의 기간을 짧게 반복하면서 요구사항을 충분히 반영한다.
+ 핵심가치 : 의사소통 , 단순성 ,용기 ,존중 ,피드백
  
 
## MileStones 
  * 4월 : 센서 연동 확인 , 데모 디자인 설계         
  * 5월 : 객체 인식 알고리즘 설계       
  * 6월 : UI설계        
  
