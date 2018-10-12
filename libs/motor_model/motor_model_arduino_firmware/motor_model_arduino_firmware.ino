/*
  Arduino ROS node for teleoperation of the Slash Platform
  MIT License
  Based on 
  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  JetsonHacks (2016)
  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
*/

/////////////////////////////////////////////////////////////////
#include "Arduino.h"
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

//////////////////////////////////////////////////////////////////

// ROS
ros::NodeHandle  nodeHandle;
//std_msgs::Int32 str_msg;
geometry_msgs::Twist  twistPub;
ros::Publisher chatter("arduino_debug_feedback", &twistPub);

//////////////////////////////////////////////////////////////////

// Serial Communication
const int baud_rate = 115200;

// Pins for outputs
const int led = 13;  // LED 
const int drive = 10;  // DRIVE

// Set the PWM min-max of the experimentation (Only used to find J)
const int pwm_min = 0  ;
const int pwm_max = 255  ;

// H-Bridge gain
const float H_Gain = 12/5;


///////////////////////////////////////////////////////////////////

// Main Loop
void cmdCallback ( const geometry_msgs::Twist&  twistMsg )
{
  
  // PWM sent to the Drive 
  int arduino_pwm = twistMsg.linear.x; //volt
  int drive_pwm = arduino_pwm*H_Gain;
  
  // Write the PWM  
  analogWrite(drive,arduino_pwm) ;
  
  // LED
  digitalWrite(led, HIGH-digitalRead(led));  //toggle led  
  
  // Debug feedback
  twistPub.linear.x = arduino_pwm;
  twistPub.linear.y = drive_pwm;
  chatter.publish( &twistPub );
  
}

// ROS suscriber
ros::Subscriber<geometry_msgs::Twist> cmdSubscriber("/motor_volt_in", &cmdCallback) ;


///////////////////////////////////////////////////////////////////
void setup(){
  
  // Init LED output
  pinMode(led, OUTPUT);
  
  // Init PWM output Pins 
  analogWrite(drive, pwm_min); 
  
  // Init Communication
  Serial.begin(baud_rate);
  
  // Init ROS
  nodeHandle.initNode();
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(cmdSubscriber) ;
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  
  //
  delay(1000) ;

}

////////////////////////////////////////////////////////////////////
void loop(){
  nodeHandle.spinOnce();
  delay(1);
}
