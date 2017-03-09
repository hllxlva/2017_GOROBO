//Rotary Encoder

#include <Wire.h>
unsigned long time;//時間を格納する君
unsigned long t0 = 0;//ひとつ前の時間を格納する君
int dt = 0;
int ave_time = 20000;//何µsごとのpulseを取るか
int dRotAPin = 2;//ロータリーエンコーダA相
int dRotBPin = 3;//ロータリーエンコーダB相
int N;//パルスの番号格納
int sign = 1;//回転方向
int PN;//Pulse Number//
volatile int m_nValueB = 0;//B相の値
volatile int m_nValueA = 0;//A相の値
void va();//velocityA//
void vb();//velocityB//
int number();//0,1,2,3//
float d = 40.5;//オムニホイールの直径 
float V;//最終エンコーダの速度出力
int i;
unsigned int value;
byte val[7];
void requestEvent();

void setup() {
  //INPUT Mode
  Serial.begin(250000);
  pinMode(dRotAPin, INPUT);
  pinMode(dRotBPin, INPUT);

  //I2C
  Wire.begin(8);
  Wire.onRequest(requestEvent);

  //Pull Up
  digitalWrite(dRotAPin, HIGH);
  digitalWrite(dRotBPin, HIGH);
  attachInterrupt(0, va, CHANGE);
  attachInterrupt(1, vb, CHANGE);
}

void va(){
  m_nValueA = digitalRead(dRotAPin);
  number();
  i++;
}

void vb(){
  m_nValueB = digitalRead(dRotBPin);
  number();
}

int number(){ 
  //数字割り当て
  N = m_nValueA + 2 * m_nValueB;
  switch(N){
    case 2:
      N = 3;
      break;
    case 3:
      N = 2;
      break;
  }
  switch(N){
    case 0:
      if(PN == 3)sign = 1;
      else if(PN == 1)sign = -1;
      else sign = 0;
      break;
    case 1:
      if(PN == 0)sign = 1;
      else if(PN == 2)sign = -1;
      else sign = 0;
      break;
    case 2:
      if(PN == 1)sign = 1;
      else if(PN == 3)sign = -1;
      else sign = 0;
      break;
    case 3:
      if(PN == 2)sign = 1;
      else if(PN == 0)sign = -1;
      else sign = 0;
      break;
  }
  PN = N;//ナンバー割り当て
}

void loop() {
  time = micros();
  dt = time - t0;
  if(dt >= ave_time){
    //速度の単位[mm/ms]
    V = d*PI*float(i)/0.720/ave_time;
    i = 0;
    t0 = time;
  }
  //対Arduino
  value = 100000*V; //100000倍 = 速度の単位[100000×mm/ms]
  val[0] = highByte(value);
  val[1] = lowByte(value);
  val[2] = (sign+1);
  //Serial.println(value);
}

void requestEvent(){
  Wire.write(val, 7);
}
