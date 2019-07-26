# HRVL_gbot_waist


## Waist Manipulator Image
![waist 1](https://user-images.githubusercontent.com/37207332/61573044-e0a8cb00-aae2-11e9-916d-d7835c57ecef.jpg)

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

***
## 허리제어 Command
    $roscore
    $rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
    $rostopic pub -1 /LA_control geometry_msgs/Transform ‘[translation: [translation(x), 
     translation(y), translation(z)], rotation: [rotation(x), rotation(y), rotation(z), w]’
### Example
    $roscore
    $rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
    $rostopic pub -1 /LA_control geometry_msgs/Transform ‘[translation: [0, 0, 10], rotation: [10, 0, 0, 0]’ 
***
## Feedback Command
    $roscore
    $rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
    $rosrun waist LA_Feedback #허리의 각 Actuator들의 길이 Feedback 토픽 Publish 
    
    $rosrun waist_Feedback ultrasound  #초음파에서 측정된 거리 값 퍼블리셔 토픽 Publish
