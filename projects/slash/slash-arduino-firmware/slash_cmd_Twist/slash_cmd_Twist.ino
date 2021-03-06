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
Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo

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
const int ser = 9;   // Servo 
const int esc = 10;  // ESC

// Hardware min-zero-max range for the steering servo and the TRAXXAS ESC
// arduino pwm-servo lib unit = 0-180 deg angle range corresponding to 544-2400 microseconds pulse-witdh
const int pwm_min_ser = 30  ;
const int pwm_zer_ser = 90  ;
const int pwm_max_ser = 150 ;
const int pwm_min_esc = 0   ;
const int pwm_zer_esc = 90  ;
const int pwm_max_esc = 180 ;

// Conversion
const double  rad2pwm = 60;
const double volt2pwm = 30;

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

// Main Loop
void cmdCallback ( const geometry_msgs::Twist&  twistMsg )
{
  // Steering
  double ser_cmd   = twistMsg.angular.z; //rad
  int ser_pwm      = cmd2pwm( ser_cmd, rad2pwm, pwm_min_ser, pwm_zer_ser, pwm_max_ser) ;

  steeringServo.write(ser_pwm) ;
  
  // ESC 
  double esc_cmd = twistMsg.linear.x; //volt
  int esc_pwm    = cmd2pwm( esc_cmd, volt2pwm, pwm_min_esc, pwm_zer_esc, pwm_max_esc) ;
    
  electronicSpeedController.write(esc_pwm) ;
  
  // LED
  digitalWrite(led, HIGH-digitalRead(led));  //toggle led  
  
  // Debug feedback
  twistPub.linear.x = esc_pwm;
  twistPub.angular.z = ser_pwm;
  chatter.publish( &twistPub );
  
}

// ROS suscriber
ros::Subscriber<geometry_msgs::Twist> cmdSubscriber("/cmd_vel", &cmdCallback) ;


///////////////////////////////////////////////////////////////////
void setup(){
  
  // Init LED output
  pinMode(led, OUTPUT);
  
  // Init PWM output Pins
  steeringServo.attach(ser); 
  electronicSpeedController.attach(esc); 
  
  // Init Communication
  Serial.begin(baud_rate);
  
  // Init ROS
  nodeHandle.initNode();
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(cmdSubscriber) ;
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  
  
  // Initialize Steering and ESC cmd to neutral
  steeringServo.write(pwm_zer_ser) ;
  electronicSpeedController.write(pwm_zer_esc) ;
  
  //
  delay(3000) ;

}

////////////////////////////////////////////////////////////////////
void loop(){
  nodeHandle.spinOnce();
  delay(1);
}
