#include <Wire.h>
#include <math.h>
int LimitSwitch = 4;
boolean mobile = false;
const double pi = 3.14;
double data[4] = {0,0,0,0};//エンコーダの値
unsigned long time;
unsigned long t0;
double dt;
int prog_num = 0;
//Approximate-------------------------------
int n = 1;//初期設定フラグ
//route_trace-------------------------------
#include "tyokusen.h"//ROUTE_POINT_NUM_a, signed short route_a[][]
#include "back.h"//ROUTE_POINT_NUM_b, signed short route_b[][]
int ROUTE_POINT_NUM;
signed short route[100][3];
boolean route_read = true;
boolean flag, PRE_R = true;
double pre_pos;
//elevating---------------------------------
boolean limit_flag, elevat = false;
int PhotoPin = 10;
volatile int PhotoValue = 0;
volatile int pre_PhotoValue = 0;
int Height;
int Initial_Height = 115;
int V_out[5] = {0,0,0,0,0};
int V_out_e_max=150;
int V_out_e_min=30;
int dh = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(9,OUTPUT);
  pinMode(LimitSwitch,INPUT) ;
  pinMode(PhotoPin,INPUT) ;
  Height = Initial_Height;
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
  }
}

void conversion_rate(double *Data, double Vd[4], double t){
  for(int i = 0; i < 4; i++){
    *(Data+i) = Vd[i]*t;//mm/s→mm/period
  }
}

void Approx(double *now_pos_aveX, double *now_pos_aveY, double *now_pos_aveA, double Vd[4]){//センサーの位置，エンコーダーの値から自己位置を推定する．
  int C[4][2] = {//センサーの位置
    { 314.75, 0},
    { 0, 185.93},
    {-314.75, 0},
    { 0,-185.93}
  };
  int initial_value[] = {0,0,0};//初期位置(最初のロボットの中心の位置を{mm, mm, deg})
  static double now_p[3][8];//今の位置 0:X, 1:Y, 2:Ang
  double now_v[3][4];//今の速度 0:X, 1:Y, 2:Ang
  double Cr[4] = {314.75, 185.93, 314.75, 185.93};//センサーまでの距離
  double now_pos_ave_c[4]; 
  
  if(n == 1){//初期設定
    /*float Cp[4][2];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 2; j++) {
        Cp[i][j] = float(C[i][j])/100;//値が大きすぎてintやと足りない…
      }
    }
    for (int i = 0; i < 4; i++) {
      Cr[i] = 100*sqrt(Cp[i][0]*Cp[i][0]+Cp[i][1]*Cp[i][1]);//極座標系に
    }*/
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 8; j++){
        now_p[i][j] = initial_value[i];//初期値
      }
    }
    n = 0;
  }
  for(int i = 0; i < 2; i++){//ω４つのセンサーから２通りωが出せる．
    now_v[2][i] = (Vd[i]+Vd[i+2])/(Cr[i]+Cr[i+2]);
  }
  for (int i = 0; i < 2; i++){
    for (int j = 0; j < 2; j++){//速度の各成分は４通りの出し方がある
      now_v[0][2*i+j] = (-2*i+1)*(Cr[2*i+1]*now_v[2][j]-Vd[2*i+1]);
      now_v[1][2*i+j] = (-2*i+1)*(Vd[2*i]-Cr[2*i]*now_v[2][j]);
    }
  }
  for (int i = 0; i < 2; i++){//θを求める
    now_p[2][i] = now_p[2][i] + now_v[2][i];
  }
  now_pos_ave_c[2] = (now_p[2][0]+now_p[2][1])/2;
  now_pos_ave_c[2] = now_pos_ave_c[2]*180/PI;
  for (int i = 0; i < 4; i++){//位置の各成分はここの４通りと
    now_p[0][i] = now_p[0][i]+(now_v[0][i]*cos(now_p[2][0])-now_v[1][i]*sin(now_p[2][0]));
    now_p[1][i] = now_p[1][i]+(now_v[0][i]*sin(now_p[2][0])+now_v[1][i]*cos(now_p[2][0]));
  }
  for (int i = 0; i < 4; i++){//ここの４通りで求めることが出来る．
    now_p[0][i+4] = now_p[0][i+4]+(now_v[0][i]*cos(now_p[2][1])-now_v[1][i]*sin(now_p[2][1]));
    now_p[1][i+4] = now_p[1][i+4]+(now_v[0][i]*sin(now_p[2][1])+now_v[1][i]*cos(now_p[2][1]));
  }
  for (int i = 0; i < 2; i++){
    now_pos_ave_c[i] = 0;
    for (int j = 0; j < 8; j++){
      now_pos_ave_c[i] = (j*now_pos_ave_c[i]+now_p[i][j])/(j+1);//値の重心を求める
    }
  }
  *now_pos_aveX = now_pos_ave_c[0];
  *now_pos_aveY = now_pos_ave_c[1];
  *now_pos_aveA = now_pos_ave_c[2];
}

void velocity(double pre_p, double *vv, double now_posX, double now_posY, double now_posA, int *min_num){
  double min_m_dist;
  double pre_min_m_dist = 1000;
  int min_m_dist_num;
  double v[4];//演算した速度{x, y, θ}
  double v_t[2];//接線方向の速度
  double p_v[2];//比例制御
  double r;//
  double pre_r;
  double e, eq;//, pre_e, pre_eq;//編差
  double v_max[] = {10,20};//最高速度{並進, 回転}(並進と回転の出力割合)
  double Kt = 20.0/*20*/, Kp = 2.0/*5*/;//, Kd = 2;//2(PID)
  double Cp = 0.1, Cd = 0.5;//(PID)

  //double V_out_max=255;//(アナログ出力最大)
  
  for(int i = 0; i < ROUTE_POINT_NUM; i++){//マンハッタン距離最小値
    min_m_dist = sq(double(now_posX-route[i][0]))+sq(double(now_posY-route[i][1]));
    if(flag){
      pre_min_m_dist = min_m_dist;
      flag = false;
    }
    if(pre_min_m_dist >= min_m_dist){
      pre_min_m_dist = min_m_dist;
      min_m_dist_num = i;
    }
  }
  flag = true;
  if(min_m_dist_num == ROUTE_POINT_NUM - 1){//最後まで行ったら速度0に
    mobile = true;
    //接線方向の速度
    v_t[0] = route[min_m_dist_num][0] - now_posX;
    v_t[1] = route[min_m_dist_num][1] - now_posY;
    r = sqrt(sq(v_t[0])+sq(v_t[1]));
    if(PRE_R){
      pre_r = r;
      if(pre_r < 1)pre_r = 1;
      PRE_R = false;
    }
    for(int i = 0; i < 2; i++){
      v_t[i] = v_t[i]/pre_r;
      p_v[i] = 0;
    }
    //法線方向の偏差
    e = 0;
  }else{
    PRE_R = true;
    //接線方向の速度
    for(int i = 0; i < 2; i++){
      v_t[i] = route[min_m_dist_num + 1][i] - route[min_m_dist_num][i];
    }
    r = sqrt(sq(v_t[0])+sq(v_t[1]));//大きさを計算
    for(int i = 0; i < 2; i++){
      v_t[i] = v_t[i]/r;//方向のみ
    }
    e = sqrt(sq((route[min_m_dist_num][1] - now_posY)*(v_t[0])+(now_posX - route[min_m_dist_num][0])*(v_t[1]))/(sq(v_t[0])+sq(v_t[1])));
    //線のどちら側にあるかを調べる
    if(v_t[0]*(route[min_m_dist_num][1] - now_posY)+v_t[1]*(now_posX - route[min_m_dist_num][0]) > 0){
      e = -e;
    }
    //法線方向の比例制御
    p_v[0] = e * +v_t[1]/r;
    p_v[1] = e * -v_t[0]/r;
  }
  //制御の係数を代入
  for(int i = 0; i < 2; i++){
    v[i] = v_t[i]*Kt + p_v[i]*Kp;
  }
  double R = sq(v[0]) + sq(v[1]);
  if(R>sq(v_max[0])){//最高速度
    for(int i = 0; i < 2; i++){
      v[i] = v_max[0]*v[i]/sqrt(R);
    }
  }
  //------------角度操作---------------------
  eq = route[min_m_dist_num][2] - now_posA;
  v[2] = eq * Cp + (now_posA - pre_p) * Cd;//v[2] = eq * Cp - (pre_p - now_posA) * Cd;
  if(v[2] > v_max[1])v[2] = v_max[1];
  if(v[2] < -v_max[1])v[2] = -v_max[1];

  for(int i = 0; i < 3; i++){
    *(vv+i) = v[i];
  }
  *min_num = min_m_dist_num;
}

//Auto
void route_set(int route_num){
  static int pre_route_num;
  if(pre_route_num != route_num){
    switch(route_num){
      case 1:
        ROUTE_POINT_NUM = ROUTE_POINT_NUM_a;
        for(int i = 0; i < ROUTE_POINT_NUM; i++){
          for(int j = 0; j < 3; j++){
            route[i][j] = route_a[i][j];
          }
        }
        break;
      case 2:
        ROUTE_POINT_NUM = ROUTE_POINT_NUM_b;
        for(int i = 0; i < ROUTE_POINT_NUM; i++){
          for(int j = 0; j < 3; j++){
            route[i][j] = route_b[i][j];
          }
        }
        break;
    }
  pre_route_num = route_num;
  }
}

void slit_count(int sign){//1か-1を引数に
  PhotoValue = digitalRead(PhotoPin);
  if(pre_PhotoValue != PhotoValue){
    dh += sign*3;//スリットが3mm刻みである故
    pre_PhotoValue = PhotoValue;
  }
}

void elevating(int value){
  int sign = value/abs(value);
  slit_count(sign);
  V_out[4] = sign*V_out_e_max*sin(PI/float(value)*float(dh));
  if(abs(V_out[4]) < abs(V_out_e_min)) V_out[4] = sign*V_out_e_min;
  if(abs(dh) >= abs(value)){
    elevat = true;
    V_out[4] = 0;
  }
}

void Send(int id, int mdd_output_num, int pwm){
  Serial.write('H');
  Serial.write(id);
  Serial.write(mdd_output_num);
  Serial.write(highByte(pwm));
  Serial.write(lowByte(pwm));
}

void loop() {
  digitalWrite(9, HIGH);//LTC485, 電圧をかける
  double now_p_aveX;//今の位置の平均 0:X, 1:Y, 2:Ang[rad]
  double now_p_aveY;
  double now_p_aveA;
  double V[3];//ロボットの原点の速度 0:X, 1:Y, 2:Ang[rad]
  int now_num;//routeの今の位置(一番近い点)
  
  
    
  time = micros();
  dt = float(time - t0)/1000000.00;
  t0 = time;
  //-------------------------------
  
  switch(prog_num){
    case 0:
      elevating(135);
      route_set(1);
      break;
    case 1:
      route_set(2);
      //elevat = true;
      break;
    /*case 2:
      route_set(2);
      mobile = true;
      break;*/
  }
  //-------------------------------
  boolean limit = digitalRead(LimitSwitch);
  /*if(limit == 0){
    V_out[4] = 0;
    if(limit_flag)elevat = true;
    limit_flag = false;
  }else{
    limit_flag = true;
  }*/
  if(elevat == 1 && mobile == 1){
    prog_num ++;
    dh = 0;
    elevat = false;
    mobile = false;
    
    Serial.println("-----------------------");
  }
  
  I2Crequest(6, 2);//I2Crequest(リクエストするArduinoの番号, dataの何番に返すか);
  I2Crequest(7, 1);
  I2Crequest(8, 0);
  I2Crequest(9, 3);
  conversion_rate(data,data,dt);
  Approx(&now_p_aveX, &now_p_aveY, &now_p_aveA, data);
  //ここからは経路によって変わる
  velocity(pre_pos, V, now_p_aveX, now_p_aveY, now_p_aveA, &now_num);//v[0], v[1], v[2]を出す
  pre_pos = now_p_aveA;
  
  //それぞれのメカナムの出力を返す
  const double Meca_a_x = 243.75;
  const double Meca_a_y = -263.50;
  const double Meca_a_t = 45.00;
  const double Meca_b_x = -243.75;
  const double Meca_b_y = -263.50;
  const double Meca_b_t = 135.00;
  const double Meca_c_x = -243.75;
  const double Meca_c_y = 263.50;
  const double Meca_c_t = 225.00;
  const double Meca_d_x = 243.75;
  const double Meca_d_y = 263.50;
  const double Meca_d_t = 315.00;

  double V_out_float[4] = {0,0,0,0};
  
  V_out_float[0] = (-V[2]*pi/180.00*Meca_a_y+(V[0]*cos(float(now_p_aveA*pi/180.00))-V[1]*sin(float(now_p_aveA*pi/180.00))))*sin(float((Meca_a_t)*pi/180.00))+(V[2]*pi/180.00*Meca_a_x+(V[0]*sin(float(now_p_aveA*pi/180.00))+V[1]*cos(float(now_p_aveA*pi/180.00))))*cos(float((Meca_a_t)*pi/180.00));
  V_out_float[1] = (-V[2]*pi/180.00*Meca_b_y+(V[0]*cos(float(now_p_aveA*pi/180.00))-V[1]*sin(float(now_p_aveA*pi/180.00))))*sin(float((Meca_b_t)*pi/180.00))+(V[2]*pi/180.00*Meca_b_x+(V[0]*sin(float(now_p_aveA*pi/180.00))+V[1]*cos(float(now_p_aveA*pi/180.00))))*cos(float((Meca_b_t)*pi/180.00));
  V_out_float[2] = (-V[2]*pi/180.00*Meca_c_y+(V[0]*cos(float(now_p_aveA*pi/180.00))-V[1]*sin(float(now_p_aveA*pi/180.00))))*sin(float((Meca_c_t)*pi/180.00))+(V[2]*pi/180.00*Meca_c_x+(V[0]*sin(float(now_p_aveA*pi/180.00))+V[1]*cos(float(now_p_aveA*pi/180.00))))*cos(float((Meca_c_t)*pi/180.00));
  V_out_float[3] = (-V[2]*pi/180.00*Meca_d_y+(V[0]*cos(float(now_p_aveA*pi/180.00))-V[1]*sin(float(now_p_aveA*pi/180.00))))*sin(float((Meca_d_t)*pi/180.00))+(V[2]*pi/180.00*Meca_d_x+(V[0]*sin(float(now_p_aveA*pi/180.00))+V[1]*cos(float(now_p_aveA*pi/180.00))))*cos(float((Meca_d_t)*pi/180.00));

  
  double slow_stop = 1.0;//slow_stop以外のところでの倍率
  double slow_start = 1.0;//slow_start以外のところでの倍率
  double slow = 30;//スローで何％まで落とすか(slow関数が効いて進まないとき大きくする)
  double count = 20;
  
  //スローストップ
  int slow_stop_count = ROUTE_POINT_NUM-1-now_num;
  if(slow_stop_count <= count){//最後から何個前の点から減速するか
    slow_stop = slow_stop_count/20.0*(1-slow/100) + slow/100;
    if(slow_stop > 1.0) slow_stop = 1.0;
  }else{
    slow_stop = 1.0;
  }

  //スロースタート
  int slow_start_count = now_num;
  if(slow_start_count <= count){//最初から何個後の点まで減速するか
    slow_start = slow_start_count/20.0*(1-slow/100) + slow/100;
    if(slow_start > 1.0) slow_start = 1.0;
  }else{
    slow_start = 1.0;
  }
  
  //最高速度設定
  boolean flag_v = true;
  float unsign_v, pre_unsign_v;
  int max_v_num = 0;
  for(int i = 0; i < 4; i++){//最高速度
    unsign_v = abs(V_out_float[i]);
    if(flag_v){
      pre_unsign_v = unsign_v;
      flag_v = false;
    }
    if(pre_unsign_v <= unsign_v){
      pre_unsign_v = unsign_v;
      max_v_num = i;
    }
  }
  
  double V_out_max=150;//(アナログ出力最大)
  //最高速度を最高出力に合わせる
  float V_max = abs(V_out_float[max_v_num]);
  for (int i = 0; i < 4; i++){
    V_out_float[i] = V_out_float[i] / V_max * V_out_max * slow_stop * slow_start;
  }

  //PIDせんでいいんかな…
  /*ここはハズさない
  float PID_V_out_float[4];
  float m[4]
  float Vp = 0.3, Vd = 1, a = 1, Ve;
  for (int i = 0; i < 4; i++){
    Ve = V_out_float[i] - V_PID_out_float[i];
    m[i] = Vp*Ve+(Vd/(dt+Vd/a)*(pre_PID_V_out_float[i] - V_PID_out_float[i]))
    pre_PID_V_out_float[i] = V_PID_out_float[i]
    V_PID_out_float[i] += m[i]
  }
  */

  for (int i = 0; i < 4; i++){
    if(V_out_float[i] > 0){
      V_out[i] = int(V_out_float[i]+0.5);
    }else{
      V_out[i] = int(abs(V_out_float[i])+0.5);
      V_out[i] = -V_out[i];
    }
  }

  int direct[5] = {-1,-1,1,1,-1};
  for (int i = 0; i < 5; i++){
    V_out[i] = direct[i]*V_out[i];
  }
  
    Serial.print(now_p_aveX);
    Serial.print(F("|"));
    Serial.print(now_p_aveY);
    Serial.print(F("|"));
    Serial.print(now_p_aveA);
    Serial.print(F("|"));
  for (int i = 0; i < 5; i++){
    Serial.print(V_out[i]);
    Serial.print(F("|"));
  }
  Serial.println(prog_num);  
  Send(15, 2, V_out[0]); //右後
  Send(16, 2, V_out[1]); //左後
  Send(15, 3, V_out[2]); //左前
  Send(16, 3, V_out[3]); //右前
  Send(25, 2, V_out[4]); //上下
}
