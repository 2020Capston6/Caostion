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
