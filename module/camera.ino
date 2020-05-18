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
