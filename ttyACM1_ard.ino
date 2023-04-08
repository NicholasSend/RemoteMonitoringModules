 // Arduino file

//includes
#include <ros.h>
#include <stdio.h>
#include <std_msgs/String.h>


//Defines
#define trigPin 10
#define echoPin 11
#define irSensorPin A1
#define L_EN 3
#define R_EN 4
#define L_PWM 6
#define R_PWM 5
#define LED 13

//distance threshold in cm for Ultrasonic sensor and IR sensor
#define distanceThresholdUltraSonic 20
#define distanceThresholdIR 50

ros::NodeHandle nh;

char msg_tmp[4];
char pre_int[3];
char flag;
int PWM_IN;
/*
// Collision Avoidance Variables
long duration;
int distance;
int irSensorValue;

*/
void messageCb( const std_msgs::String& action){
  
  // String Manipulation to Integer Values
  strcpy(msg_tmp, action.data);

  pre_int[0] = msg_tmp[1];
  pre_int[1] = msg_tmp[2];
  pre_int[2] = msg_tmp[3];
  
  sscanf(pre_int, "%d", &PWM_IN);

  flag = '_';

  // Collision Avoidance
  /*
  irSensorValue = analogRead(irSensorPin);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58;
  // If too close:
  if ((distance < distanceThresholdUltraSonic) || (irSensorValue < distanceThresholdIR)) {
    if (flag == '_'){ // if flag is down
      // Raise flag
      flag = msg_tmp[0];
      digitalWrite(LED, LOW);
      // Raised flag blocks any flagged input from executing and moving further
    }
  } 
  
  else {
    // Clear flag
    flag = '_';
    digitalWrite(LED, HIGH);
  }
  */
  
  
  if (msg_tmp[0] == 'N'){ // Neutral -> no movement, safety state
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
//   digitalWrite(LED, HIGH); 
  }
  
  else {
    
    if ((msg_tmp[0] == 'B') && (flag != 'B')){ // Forward
      analogWrite(R_PWM, PWM_IN/4);
      
      analogWrite(L_PWM, 0);
    }

    else if ((msg_tmp[0] == 'F') && (flag != 'F')){  // Back
      analogWrite(L_PWM, PWM_IN/4);
      analogWrite(R_PWM, 0);
    }
    
    else if ((msg_tmp[0] == 'R') && (flag != 'R')){  // Right
      analogWrite(L_PWM, PWM_IN/1.5);
      analogWrite(R_PWM, 0);
    }
    
    else if ((msg_tmp[0] == 'L') && (flag != 'L')){  // Left
      analogWrite(R_PWM, PWM_IN/1.5);
      analogWrite(L_PWM, 0);
    } 

    else{ // Uknown or flagged input -> No movement, safety state
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, 0);
    }
  }
}

ros::Subscriber<std_msgs::String> sub("ard_in", &messageCb );

//std_msgs::String str_msg;
//ros::Publisher pub("ard_out", &str_msg);

//char hello[13] = "hello world!";

void setup()
{
/*
  // Collision Avoidance Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irSensorPin, INPUT);
  */
  pinMode(LED, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  //Serial.begin(9600);
 
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(pub);
  nh.getHardware()->setBaud(115200);
}

void loop()
{
  /*
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  */
//  duration = pulseIn(echoPin, HIGH);
//  distance = duration / 58;

  // Ask about serial prints - do we need these piped to the pi?
//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println(" cm");

//  irSensorValue = analogRead(irSensorPin);
//  Serial.print("IR Sensor Value: ");
//  Serial.println(irSensorValue);


 // str_msg.data = hello;
//  pub.publish( &str_msg );
  nh.spinOnce();
  delay(1);
}
