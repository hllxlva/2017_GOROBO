#include <Wire.h>
float data[4] = {0,0,0,0};//エンコーダの値
unsigned long time;
unsigned long t0;
float dt;
float dis[4] = {0,0,0,0};
void setup() {
  Wire.begin();
  Serial.begin(250000);
}

void I2Crequest(int wireRequest, int data_num){
  byte val[7];
  Wire.requestFrom(wireRequest, 7);
  while(Wire.available()){
    for(int i = 0; i < 7; i++){
      val[i] = Wire.read();
    }
    unsigned int x = 256*val[0]+val[1];
    int Sign = val[2] - 1;
    data[data_num] = Sign* double(x/100.00);//[mm/s]
    Serial.print("data[");
    Serial.print(data_num);
    Serial.print("]: ");
    Serial.print(data[data_num]);
  }
}

void loop() {
  time = micros();
  dt = float(time - t0)/1000000.00;
  t0 = time;
  I2Crequest(8, 0);
  Serial.print(" | ");
  I2Crequest(9, 1);
  Serial.print(" | ");
  dis[0] += data[0]*dt;
  Serial.println(dis[0]);
}
