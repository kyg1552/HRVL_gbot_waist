/*
   프로그램: HRVL gbot의 초음파 센서 사용
   작성자: Young-gi Kim
   최근작성일: 2019.08.08
   사용보드: Arduino Mega
   설명:
        .타이머를 이용해 100ms 주기로 초음파 센서 8개 값 입력받는다.

   reference:
  - Ultrasound Sensor
  https://fleshandmachines.wordpress.com/2011/09/16/arduino-double-sonar-with-ros/ - 초음파센서 2개 이상 사용 시
  http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html  - 초음파 센서 값 ros type
*/

/////// 라이브러리 ////////////////////////
#include <sensor_msgs/Range.h>
/////// 라이브러리 ////////////////////////

///// 매크로 선언 /////
#define DIST_S 150*58.2 // 초음파센서 TimeOut 설정(최대거리) 150cm 일때 최대주기 72~73ms, 200cm일 때 최대주기 93~94ms
#define BAUDRATE1 9600
#define BAUDRATE2 115200
///// 매크로 선언 /////


//// 함수 선언 ////
long trig_echo(long TRIG, long ECHO); // 거리 측정
void ultra_sensor(void);  // 측정된 거리 ros publish
void ultra_setup(void);   // 초음파 echo, trig 핀 셋업
void ultra_pub_setup(void); // 초음파 센서 publish 셋업
//// 함수 선언 ////

///// 변수 선언 /////
// Ultra Sensor Trig, Echo Pin
const unsigned char trig[8] = {30, 32, 34, 36, 38, 40, 42, 44};
const unsigned char echo[8] = {31, 33, 35, 37, 39, 41, 43, 45}; //회로연결 (22,30),(23,31),(24,32),(25,33),(26,34),(27,35),(29,36),(40,38)

//using timer0, millis(), micros()
unsigned long pre_time = 0;
unsigned long cur_time = 0;
const long utime = 2500; // 2500us
const long mtime = 100; // 100ms
///// 변수 선언부 /////

//// ros 선언 부////
ros::NodeHandle nh;

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
///////// ros 선언 부 끝 //////////

* /

//////setup /////
void setup() {
  // put your setup code here, to run once:
  ultra_setup();
  nh.initNode();
  ultra_pub_setup();
}

///////*****  Loop  ********* /
void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  cur_time = millis();

  if (cur_time - pre_time >= mtime) //mtime = 100, 100ms주기
  {
    pre_time = cur_time;
    ultra_sensor(); // 초음파 신호 받고, 토픽 보냄.
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
  range_msg.max_range = DIST_S / 58.2 * 100; // maximum range value [m]
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
  dist = pulseIn(ECHO, HIGH, DIST_S) / 58.2; //거리 측정 식
  return (dist);
}
void ultra_sensor(void)
{ // 8개 초음파 센서 값 읽어들임
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
