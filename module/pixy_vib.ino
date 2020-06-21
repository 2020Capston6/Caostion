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
