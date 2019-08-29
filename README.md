# HRVL_gbot_waist


## Waist: 3-RPS 3DOF Parallel Manipulator
### Image
![waist 1](https://user-images.githubusercontent.com/37207332/61573044-e0a8cb00-aae2-11e9-916d-d7835c57ecef.jpg)

***
## Architecture
![4](https://user-images.githubusercontent.com/37207332/61940219-eb29ff80-afcf-11e9-87cc-739465d5ddbd.JPG)

***
## Linear Actuator(Pololu)
* Glideforce LACT12P-12V-05 Light-Duty Linear Actuator with Feedback: 15kgf, 12" Stroke (11.8" Usable), 1.7"/s, 12V
    * https://www.pololu.com/product/2327


## Linear Actuator Controller(Pololu)
* Jrk 21v3 USB Motor Controller with Feedback (Connectors Soldered)
    * https://www.pololu.com/product/1394
* Pololu Jrk USB Motor Controller User’s Guide
    * https://www.pololu.com/docs/0J38
* Motor Controller Utility SoftWare(Windows용)
    * https://www.pololu.com/docs/0J38/3.a
    * Utility 설치 후 아래 링크를 참조하여 사용하는 Actuator에 맞는 Utility setup txt파일을 다운로드 받아 
      Utility 프로그램을 Setting해준다.
        * https://www.pololu.com/product/2327
        * https://github.com/kyg1552/HRVL_gbot_waist/blob/master/Reference/LinearActuator_jrk_setup.pdf
         ![1](https://user-images.githubusercontent.com/37207332/61711681-939d5100-ad8f-11e9-8887-19512c0c3bb8.JPG)
***
## 사용보드
* Arduino Mega2560

## Test 1Mega
* ROS Package install
      1. gbot_waist directory를 catkin_ws/src에 다운받는다.
      2. $cd ~/catkin_ws && catkin_make
      
* Arduino IDE Setup
    
      $sudo apt-get install ros-kinetic-rosserial-arduino
      $sudo apt-get install ros-kinetic-rosserial
    
* Installing from Source onto the ROS workstation

      $cd ~/catkin_ws/src
      $git clone https://github.com/ros-drivers/rosserial.git
      $cd ~/catkin_ws && catkin_make
    
* Install ros_lib into the Arduino Environment
    
      $cd ~/Arduino/libraries
      $rm -rf ros_lib
      $rosrun rosserial_arduino make_libraries.py .
* http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

* Start waist ROS Master
      
      $roscore
      $rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
      
* 허리 제어
      
      $rostopic pub -r 15 /waist_control geometry_msgs/Transform ‘[translation: [translation(x), 
      translation(y), translation(z)], rotation: [rotation(x), rotation(y), rotation(z), w]’
      
      Example
      
      $roscore
      $rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
      $rostopic pub -r 15 /waist_control geometry_msgs/Transform '{translation: [0, 0, 15], rotation: [0, 0, 0, 0]}' 

* 허리부(Waist) Feedback Node Run
      
      $rosrun gbot_waist waist_Feedback #허리의 각 Actuator들의 길이 Feedback 토픽 Publish 
      
* 초음파 Distance Feedback Node Run 
 
      $rosrun gbot_waist ultrasonic #초음파에서 측정된 거리 값 퍼블리셔 토픽 Publish
