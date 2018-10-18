//=========================HEADER=============================================================
/*
 * Teleoperation for a dual propulsion driven by a dual Cytron drive on a RC car.
 * 
 * ROS NODE WRITTEN BY: William Therrien, SherbyRobotics
   DATE: Octobre 12, 2018
   *****************************************************************************************
   Inspired by:
   Arduino ROS node for teleoperation of the Slash Platform
   MIT License
   Based on 
   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
   Arduino ROS node for JetsonCar project
   The Arduino controls a TRAXXAS Rally Car
   JetsonHacks (2016)
   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
   ******************************************************************************************

*/

/////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include <SPI.h> 
#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

//////////////////////////////////////////////////////////////////

// Servo objects for PWM control
Servo steeringServo;

// ROS
ros::NodeHandle  nodeHandle;
//std_msgs::Int32 str_msg;
geometry_msgs::Twist  cmdPub;
ros::Publisher chatter_cmd("arduino_debug_feedback", &cmdPub);

//////////////////////////////////////////////////////////////////

// Serial Communication
const int baud_rate = 9600;

// Pins for outputs
const int ser = 9;   // Servo 
const int motA = 5;  // Motor A
const int motB = 6;  // Motor B
const int dirA = 4;  // Direction motor A
const int dirB = 3;  // Direction motor B

// Hardware min-zero-max range for the steering servo and the Cytron drive
// arduino pwm-servo lib unit = 0-180 deg angle range corresponding to 544-2400 microseconds pulse-witdh
const int pwm_min_ser = 30  ;
const int pwm_zer_ser = 90  ;
const int pwm_max_ser = 150 ;

const int pwm_min_cmd = 0 ;
const int pwm_max_cmd = 255 ;

// Conversion
const double  rad2pwm = 60;

///////////////////////////////////////////////////////////////////

// Convertion function : Command --> PWM
double cmd2pwm (double cmd, double slope, int pmw_min, int pwm_zer, int pwm_max) {
  // Scale and offset
  double pwm_d = cmd * slope + (double) pwm_zer;
  // Rounding and conversion
  int pwm = (int) ( pwm_d + 0.5 );
  // Saturations
  if (pwm < pmw_min) { 
    pwm = pmw_min;
  }
  if (pwm > pwm_max) {
    pwm = pwm_max;
  }
  return pwm;
}

///////////////////////////////////////////////////////////////////


float floatmap(double cmd, int from_min, int from_max, int to_min, int to_max) {
  float pwm_f = cmd*to_max/from_max;

  return pwm_f;
}

///////////////////////////////////////////////////////////////////

// Convertion function: Command --> PWM Motors
double cmd2cyt(double cmd, int pwm_min, int pwm_max) {
  float pwm_f;
  int pwm;
  
  if (cmd >= 0) {
    digitalWrite(dirA, HIGH);
    digitalWrite(dirB, HIGH);
    float pwm_f = floatmap(cmd,0,1,0,255);
    int pwm = (int) pwm_f;
    return pwm;
  }
  
  if (cmd < 0) {
    digitalWrite(dirA, LOW);
    digitalWrite(dirB, LOW);
    float pwm_f = floatmap(cmd,0,-1,0,255);
    int pwm = (int) pwm_f;
    return pwm; 
  }
   
}


///////////////////////////////////////////////////////////////////

// Main Loop
void cmdCallback ( const geometry_msgs::Twist&  twistMsg )
{
  // Steering
  double ser_cmd   = twistMsg.angular.z; //rad
  int ser_pwm      = cmd2pwm( ser_cmd, rad2pwm, pwm_min_ser, pwm_zer_ser, pwm_max_ser) ;

  // Cytron Drive 
  double cytron_cmd = twistMsg.linear.x; //volt
  int cytron_pwm   = cmd2cyt(cytron_cmd, pwm_min_cmd, pwm_max_cmd);

  // Write PWM for the steering servo and both motors
  steeringServo.write(ser_pwm) ;
  analogWrite(motA,cytron_pwm);
  analogWrite(motB,cytron_pwm);

  // Debug feedback
  cmdPub.linear.x = cytron_pwm;
  cmdPub.angular.z = ser_pwm;
  chatter_cmd.publish( &cmdPub );

}

// ROS suscriber
ros::Subscriber<geometry_msgs::Twist> cmdSubscriber("/cmd_vel", &cmdCallback) ;

///////////////////////////////////////////////////////////////////

void setup(){
   
  // Init PWM output Pins
  pinMode(motA, OUTPUT);
  pinMode(motB, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  
  // Init PWM output Pins
  steeringServo.attach(ser); 
  
  // Init Communication
  Serial.begin(baud_rate);
  
  // Init ROS
  nodeHandle.initNode();
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(cmdSubscriber) ;
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter_cmd);
  
  // Initialize Steering and Drive cmd to neutral
  digitalWrite(dirA, HIGH);
  digitalWrite(dirB, HIGH);
  // Init Cytron Drive
  int cytron_pwm = 0;
  analogWrite(motA,cytron_pwm);
  analogWrite(motB,cytron_pwm);
  steeringServo.write(pwm_zer_ser);
  
  //
  delay(3000) ;

}

////////////////////////////////////////////////////////////////////

void loop(){
  nodeHandle.spinOnce();
  delay(1);
}
