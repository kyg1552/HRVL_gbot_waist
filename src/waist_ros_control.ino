/*
   프로그램: Linear Actuator Control with Jrk Motor Controller and ros
   작성자: Young-gi Kim
   최근작성일: 2019.05.15
   설명:  .3개의 Linear Actuator를 제어(각도(roll,pitch),길이) ros이용, stewartPlatform, 3-DOF 3-RPS Parallel Manipulator
          .각 Actuator의 현재 위치 값(Feedback 신호)를 받는다. timer0사용 100ms주기
          .타이머를 이용해 100ms 주기로 초음파 센서 8개 값 입력받는다.
   사용보드: Arduino Mega
   reference:
            - ROS control
           http://docs.ros.org/api/geometry_msgs/html/index-msg.html - ros msg geometry_msgs manual
           https://github.com/dreamster/rosserial-example - rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
           - 3DOF Stewart Platform
           https://github.com/adamweld/microgoats_stewie
           https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2017/psl58_aw698_eb645/psl58_aw698_eb645/index.html#
           https://www.pololu.com/product/2327 - Linear Actuator Manual
           https://www.pololu.com/docs/0J38 - jrk motor controller Manual
           https://github.com/pololu/jrk-g2-arduino  - JrkG2 Manual
           http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html - 리니어엑추에이터 각도,길이
           - Ultrasound Sensor 
           https://fleshandmachines.wordpress.com/2011/09/16/arduino-double-sonar-with-ros/ - 초음파센서 2개 이상 사용 시
           http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html  - 초음파 센서 값 ros type
*/
/////// 라이브러리 ////////////////////////
#include "Arduino.h"  
#include <JrkG2.h>        //jrk control 헤더
#include <SoftwareSerial.h>  //소프트웨어 시리얼포트 사용 헤더
#include <ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>
/////// 라이브러리 ////////////////////////

///// 매크로 선언 /////
#define LANumber 11  //Actuator Device Number, 3개 모두 11로 설정
#define DIST_S 150*58.2 // 초음파센서 TimeOut 설정(최대거리) 
#define BAUDRATE1 9600
#define BAUDRATE2 115200
const float pi = 3.141592653; // PI
///// 매크로 선언 /////

///// jrk, Linear Actuator setup /////
JrkG2Serial LA1(Serial1, LANumber); // LinearActuator1
JrkG2Serial LA2(Serial2, LANumber); // LinearActuator2
JrkG2Serial LA3(Serial3, LANumber); // LinearActuator3
///// jrk, Linear Actuator setup end/////

//// 함수 선언 ////
long trig_echo(long TRIG, long ECHO); // 거리 측정
void ultra_sensor(void);  // 측정된 거리 ros publish
void ultra_setup(void);   // 초음파 echo, trig 핀 셋업
void ultra_pub_setup(void); // 초음파 센서 publish 셋업

float getRadian(float theta);
void create_l_vectors(void); // 엑추에이터 길이 계산
void BRP(float theta, float phi); 
void set_value(void);  // B, P, L 값 세팅
void mul_matrix(float A[3][3], float B[3][3], float C[3][3]);
void add_matrix(float A[3][3], float B[3][3], float C[3][3]);
void sub_matrix(float A[3][3], float B[3][3], float C[3][3]);

//void GetFeedback();
void LA_setup(void); // Linear Actuator 초기 셋업
void LA_Cb(const geometry_msgs::Transform& LA_val); // 엑추에이터 제어 subscriber call back 함수
void LA_FB_pub(void); // 엑추에이터 현재 길이 feedback publish
//// 함수 선언 /////

///// 변수 선언 /////
float p_rad = 16.0;   // platform에서 각 엑추에이터까지 반지름(cm)
float b_rad = 16.0;   // base에서 각 엑추에이터까지 반지름(cm)

float LA1_length, LA2_length, LA3_length; //입력받은 각도에 따른 각 엑추에이터 길이 계산 값(0~4096)
float feedback1_length, feedback2_length, feedback3_length; // 현재 엑추에이터 길이 publish할 피드백 값(0~30cm)
//Feedback value, 0~4095
short feedbackPosition[4]; // 현재 피드백 값 받는 변수

float z_set = 30.0; // 2개의 회전 DOF로 움직임을 구속하기 위한 상수 값, 길이 변화 값으로 지정
                    // 엑추에이터 길이가 0~30이므로 30으로 지정함.(cm)
float z0 = 15.0;   // 사물이 충돌하기 전에 시스템의 최소 변환 z 값, platform에서부터 base까지의 거리(cm)
// ex) z0이 10이면, 10cm가 올라가는 것이 아니라,총 길이 30cm에서부터 10cm 내려오는 것을 말함.
float temp_z;

float theta = 0.0;  // roll(x-axis)
float phi = 0.0;  // pitch(y-axis)

float B[3][3]; // base, platform: 각 축 x, y, z
float P[3][3]; // base, platform: 각 축 x, y, z
float T[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {z_set, z_set, z_set} }; //T: z_set(base,platform 2DOF 고정 길이)(cm)
float L[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
float bRp[3][3];// base to platform rate

float sin30 = 0.5;
float cos30 = 0.866;

//높이 최대,최소 / 기울일 각도 최대, 최소 값 설정
int minz0 = 10;
int maxz0 = 30;
int mintheta = -20;
int maxtheta =  20;
int minphi   = -20;
int maxphi   =  20;

// Ultra Sensor Trig, Echo Pin
const unsigned char trig[8] = {30, 32, 34, 36, 38, 40, 42, 44};
const unsigned char echo[8] = {31, 33, 35, 37, 39, 41, 43, 45};

//using timer0, millis(), micros()
unsigned long pre_time = 0;
unsigned long cur_time = 0;
const long utime = 2500; // 2500us
const long mtime = 100; // 100ms

///// 변수 선언부 /////

//// ros 선언 부////
ros::NodeHandle nh;

geometry_msgs::Vector3 Feedback_msg;
sensor_msgs::Range range_msg;

ros::Publisher pub_range1("/ultrasound1", &range_msg);//초음파 값 상부로 전송
ros::Publisher pub_range2("/ultrasound2", &range_msg);
ros::Publisher pub_range3("/ultrasound3", &range_msg);
ros::Publisher pub_range4("/ultrasound4", &range_msg);
ros::Publisher pub_range5("/ultrasound5", &range_msg);
ros::Publisher pub_range6("/ultrasound6", &range_msg);
ros::Publisher pub_range7("/ultrasound7", &range_msg);
ros::Publisher pub_range8("/ultrasound8", &range_msg);
char frameid[] = "/ultrasound";

ros::Publisher pub("/LA_Feedback", &Feedback_msg);

ros::Subscriber<geometry_msgs::Transform> sub("/LA_control", LA_Cb); //제어값받음

///////// ros 선언 부 끝 //////////

////***** Setup************** //////////////
void setup()
{
  ultra_setup();
  LA_setup();   //리니어 엑추에이터 초기값 설정
  sp_control();
  nh.initNode();
  ultra_pub_setup();
  nh.subscribe(sub);
  nh.advertise(pub);
}
//////setup /////

///////*****  Loop  ********* /////
void loop()
{
  nh.spinOnce();

  cur_time = millis();

  if (cur_time - pre_time >= mtime) //mtime = 100, 100ms주기
  {
  pre_time = cur_time;
  ultra_sensor(); // 초음파 신호 값을 받아 토픽을 보낸다.
  LA_FB_pub();  // Actuator 피드백 신호 값 publish
  }
}
///// 함수 정의 부  시작 /////
//////////초음파 모듈/////////////////////
void ultra_setup(void)
{
  for (int i = 0; i <= 7; i++) // 초음파 관련 핀 셋업.
  {
  pinMode(trig[i], OUTPUT);
  pinMode(echo[i], INPUT);
  }
}
void ultra_pub_setup(void)
{
   nh.advertise(pub_range1);
   nh.advertise(pub_range2);
   nh.advertise(pub_range3);
   nh.advertise(pub_range4);
   nh.advertise(pub_range5);
   nh.advertise(pub_range6);
   nh.advertise(pub_range7);
   nh.advertise(pub_range8);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;  //minimum range value [m]
  //range_msg.max_range = 2.0;  //minimum range value [m]
  range_msg.max_range = DIST_S/58.2*100; // maximum range value [m]
}
// UltraSensor Function Def.
long trig_echo(long TRIG, long ECHO)
{ // 초음파 trig 신호 보내고, echo로 받음
  long dist;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  dist = pulseIn(ECHO, HIGH, DIST_S)/58.2; //거리 측정 식
  return (dist);
}
void ultra_sensor(void)
{ // 8개 초음파 센서 값 
  range_msg.range = trig_echo(trig[0], echo[0]);
  range_msg.header.stamp = nh.now();
  pub_range1.publish(&range_msg);

  range_msg.range = trig_echo(trig[1], echo[1]);
  range_msg.header.stamp = nh.now();
  pub_range2.publish(&range_msg);

  range_msg.range = trig_echo(trig[2], echo[2]);
  range_msg.header.stamp = nh.now();
  pub_range3.publish(&range_msg);

  range_msg.range = trig_echo(trig[3], echo[3]);
  range_msg.header.stamp = nh.now();
  pub_range4.publish(&range_msg);

  range_msg.range = trig_echo(trig[4], echo[4]);
  range_msg.header.stamp = nh.now();
  pub_range5.publish(&range_msg);

  range_msg.range = trig_echo(trig[5], echo[5]);
  range_msg.header.stamp = nh.now();
  pub_range6.publish(&range_msg);

  range_msg.range = trig_echo(trig[6], echo[6]);
  range_msg.header.stamp = nh.now();
  pub_range7.publish(&range_msg);

  range_msg.range = trig_echo(trig[7], echo[7]);
  range_msg.header.stamp = nh.now();
  pub_range8.publish(&range_msg);
}
///////초음파 모듈 끝 /////////////////////

///// Actuator 관련 모듈/////////////////////////////////////
//GetFeedback Funtion Def.
void LA_setup(void)
{
  Serial1.begin(BAUDRATE2); //LinearActuator1 baud rate, Mega H/W UART1 사용
  Serial2.begin(BAUDRATE2); //LinearActuator2 baud rate, Mega H/W UART2 사용
  Serial3.begin(BAUDRATE2); //LinearActuator3 baud rate, Mega H/W UART3 사용
  Serial1.flush();
  Serial2.flush();
  Serial3.flush();
}

////  3DOF Stewart Platform 관련 함수들.
float getRadian(float theta)
{ // 각도를 라디안 값으로 변환, tan()의 매개변수로 전달하기 위함
  return theta * (pi / (float)180.0);  // dtr = pi / (float)180.0
}
void set_value(void) // 3dof stewart platform 각 변수 초기화
{
  B[0][0] = p_rad;          // body origin to motor 1 shaft center X position
  B[1][0] = 0.0;            // body origin to motor 1 shaft center Y position
  B[2][0] = z0;             // body origin to motor 1 shaft center Z position
  B[0][1] = -b_rad * sin30; // same for motor 2
  B[1][1] = b_rad * cos30;
  B[2][1] = z0;
  B[0][2] = -b_rad * sin30; // same for motor 3
  B[1][2] = -b_rad * cos30;
  B[2][2] = z0;

  P[0][0] = p_rad;          // platform origin to effector pivot vector X position
  P[1][0] = 0.0;            // platform origin to effector pivot vector Y position
  P[2][0] = 0.0;            // platform origin to effector pivot vector Z position
  P[0][1] = -p_rad * sin30; // same for second effector pivot point
  P[1][1] = p_rad * cos30;
  P[2][1] = 0.0;
  P[0][2] = -p_rad * sin30; // same for third effector pivot point
  P[1][2] = -p_rad * cos30;
  P[2][2] = 0.0;

  L[0][0] = 0.0;
  L[0][1] = 0.0;
  L[0][2] = 0.0;
  L[1][0] = 0.0;
  L[1][1] = 0.0;
  L[1][2] = 0.0;
  L[2][0] = 0.0;
  L[2][1] = 0.0;
  L[2][2] = 0.0;
}
void BRP(void) {   // bRP matrix 부분.
  bRp[0][0] = cos(getRadian(theta));
  bRp[0][1] = sin(getRadian(theta)) * sin(getRadian(phi));
  bRp[0][2] = cos(getRadian(phi)) * sin(getRadian(theta));
  bRp[1][0] = 0;
  bRp[1][1] = cos(getRadian(phi));
  bRp[1][2] = -sin(getRadian(phi));
  bRp[2][0] = -sin(getRadian(theta));
  bRp[2][1] = cos(getRadian(theta)) * sin(getRadian(phi));
  bRp[2][2] = cos(getRadian(phi)) * cos(getRadian(theta));
}
void create_L_vectors(void) {
  mul_matrix(bRp, P, L);
  add_matrix(T, L, L);
  sub_matrix(B, L, L);
}
void sp_control(void) // y, x 축 각도에 따라서 길이 계산
{
  set_value();
  BRP();
  create_L_vectors();

  LA1_length = round(sqrt((L[0][0] * L[0][0]) + (L[1][0] * L[1][0]) + (L[2][0] * L[2][0]))); //얼마만큼움직여야하는지
  LA2_length = round(sqrt((L[0][1] * L[0][1]) + (L[1][1] * L[1][1]) + (L[2][1] * L[2][1]))); //계산하는부분
  LA3_length = round(sqrt((L[0][2] * L[0][2]) + (L[1][2] * L[1][2]) + (L[2][2] * L[2][2])));

  LA1.setTarget(map(LA1_length, 0, 30, 0, 4095)); delay(2);
  LA2.setTarget(map(LA2_length, 0, 30, 0, 4095)); delay(2);
  LA3.setTarget(map(LA3_length, 0, 30, 0, 4095)); delay(2);

}
void LA_FB_pub(void) // 피드백 신호 퍼블리쉬
{ 
  // 3개의 엑추에이터의 피드백 신호 받음.
  feedbackPosition[1] = LA1.getScaledFeedback(); // 0~4095
  feedbackPosition[2] = LA2.getScaledFeedback(); // 0~4095
  feedbackPosition[3] = LA3.getScaledFeedback(); // 0~4095

  feedback1_length = map(feedbackPosition[1], 0, 4095, 0, 30); // 움직이고 있는 엑추에이터 길이.
  feedback2_length = map(feedbackPosition[2], 0, 4095, 0, 30);
  feedback3_length = map(feedbackPosition[3], 0, 4095, 0, 30);

  Feedback_msg.x = feedback1_length; // x길이
  Feedback_msg.y = feedback2_length; // y길이
  Feedback_msg.z = feedback3_length; // z길이

  pub.publish(&Feedback_msg);
}
void LA_Cb(const geometry_msgs::Transform& LA_val) // 명령 값 받아서 엑추에이터 제어하는 콜백함수
{
  temp_z = LA_val.translation.z;
  if ( temp_z <= 10) temp_z = 10;
  z0 = (z_set - temp_z);
  theta = -constrain(LA_val.rotation.x, mintheta, maxtheta); // 입력 양수: 앞, 음수: 뒤
  phi = -constrain(LA_val.rotation.y, minphi, maxphi); //입력 양수: 오른쪽, 음수: 왼쪽

  sp_control();
}

void mul_matrix(float A[3][3], float B[3][3], float C[3][3])
{
  int c, r, n;
  for (c = 0; c < 3; c++)
  {
  for (r = 0; r < 3; r++)
  {
    for (n = 0; n < 3; n++)
      C[r][c] += A[r][n] * B[n][c];
  }
  }
}
void add_matrix(float A[3][3], float B[3][3], float C[3][3])
{
  int i, j;
  for (i = 0; i < 3; ++i)
  {
  for (j = 0; j < 3; ++j)
  {
    C[i][j] = A[i][j] + B[i][j];
  }
  }
}
void sub_matrix(float A[3][3], float B[3][3], float C[3][3])
{
  int i, j;
  for (i = 0; i < 3; ++i)
  {
  for (j = 0; j < 3; ++j)
  {
    C[i][j] = A[i][j] - B[i][j];
  }
  }
}
///// 함수 정의 부  끝 /////
