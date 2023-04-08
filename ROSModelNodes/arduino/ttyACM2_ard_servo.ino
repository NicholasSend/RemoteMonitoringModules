#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>

#define SERVO_PIN 9
ros::NodeHandle  nh;

Servo servo;

char direct[1];
char lastDirect[1];
char pre_int[3];
int servo_pos = 0;
int turnVelocity = 1;     // initial turn vel
int maxTurnVelocity = 20; // maximum turn vel

void servo_cb( const std_msgs::String& cmd_msg){
  strcpy(direct, cmd_msg.data);
  if (direct[0] == 'N'){
    //servo_pos = servo_pos;
    turnVelocity = 1;
  }
  else if(direct[0] == lastDirect[0]){
    if(turnVelocity <= maxTurnVelocity){
      turnVelocity = turnVelocity + 1;
    }
    else{
      turnVelocity = maxTurnVelocity;
    }
  }
  else {
    turnVelocity = 1;
  }
  if ((direct[0] == 'L')&&(servo_pos < 180)){
    servo_pos = servo_pos + turnVelocity;
  }
  else if ((direct[0] == 'R')&&(servo_pos > (-180))){
    servo_pos = servo_pos - turnVelocity;
  }
  else {
    //servo_pos = servo_pos;
  }

  lastDirect = direct;
  
  servo.write(servo_pos); //set servo angle from -180 to 180  
}

ros::Subscriber<std_msgs::String> sub("servo_in", servo_cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  nh.getHardware()->setBaud(115200);
  
  servo.attach(SERVO_PIN); //attach to pin 9
  servo_pos = 0;

  
}

void loop(){

  nh.spinOnce();
  delay(1);
}
