// Arduino file

//includes
#include <ros.h>
#include <stdio.h>
#include <std_msgs/String.h>


//Defines
#define L_EN 3
#define R_EN 4
#define L_PWM 6
#define R_PWM 5
#define LED 13

ros::NodeHandle nh;

char msg_tmp[4];
char pre_int[3];
int PWM_IN;

void messageCb( const std_msgs::String& action){
  
  // PWM if else flow
  strcpy(msg_tmp, action.data);

  pre_int[0] = msg_tmp[1];
  pre_int[1] = msg_tmp[2];
  pre_int[2] = msg_tmp[3];
  
  
  sscanf(pre_int, "%d", &PWM_IN);
//  PWM_IN = atoi((msg_tmp[1] + msg_tmp[2] + msg_tmp[3]));
  
  if (msg_tmp[0] == 'N'){ // Neutral -> no movement
    digitalWrite(R_PWM, 0);
    digitalWrite(L_PWM, 0);
//   digitalWrite(LED, HIGH); 
  }
  
  else {
    
  
    if (msg_tmp[0] == 'F'){ // Forward
      
      digitalWrite(R_PWM, PWM_IN);
      digitalWrite(L_PWM, 0);
    }

    else if (msg_tmp[0] == 'B'){  // Back
      digitalWrite(R_PWM, 0);
      digitalWrite(L_PWM, PWM_IN);
    }

    
    else if (msg_tmp[0] == 'R'){  // Right
      digitalWrite(R_PWM, 0);
      digitalWrite(L_PWM, PWM_IN);
    }
    
    else if (msg_tmp[0] == 'L'){  // Left
      digitalWrite(R_PWM, PWM_IN);
      digitalWrite(L_PWM, 0);
    } 
    
     
//  digitalWrite(LED, HIGH);   // blink the led
  }
}

ros::Subscriber<std_msgs::String> sub("ard_in", &messageCb );

std_msgs::String str_msg;
ros::Publisher pub("ard_out", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
 
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{
  str_msg.data = hello;
//  pub.publish( &str_msg );
  nh.spinOnce();
  delay(1);
}
