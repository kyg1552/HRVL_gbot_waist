# HRVL_gbot_waist


![waist 1](https://user-images.githubusercontent.com/37207332/61573044-e0a8cb00-aae2-11e9-916d-d7835c57ecef.jpg)


## Board
* Arduino Mega2560

## Transfrom Command
    $roscore
    $rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
    $rostopic pub -1 /LA_control geometry_msgs/Transform ‘[translation: [translation(x), 
     translation(y), translation(z)], rotation: [rotation(x), rotation(y), rotation(z), w]’
     
## Feedback Command
    $roscore
    $rosrun waist_Feedback LA_Feedback
    $rosrun waist_Feedback ultrasound
