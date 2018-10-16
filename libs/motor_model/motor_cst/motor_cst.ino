//=========================HEADER=============================================================
/*
 * Feedback control loop for a dual propulsion RC car.
 * 
 * ROS NODE WRITTEN BY: William Therrien, SherbyRobotics
   DATE: Octobre 12, 2018
   *****************************************************************************************
   Inspired by:
   AUTHOR: Jason Traud
   DATE: June 22, 2013
   License: CCAv3.0 Attribution-ShareAlike (http://creativecommons.org/licenses/by-sa/3.0/)
   AND:
   Arduino ROS node for teleoperation of the Slash Platform
   MIT License
   Based on 
   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
   Arduino ROS node for JetsonCar project
   The Arduino controls a TRAXXAS Rally Car
   JetsonHacks (2016)
   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
   ******************************************************************************************
   
   This is a simple ROS node program to read encoder counts
   collected by the LS7366 breakout board. The counts are
   then displayed in a ROS message
   
   Hardware: Arduino Uno R3
   Powered 
   
   LS7366 Breakout    -------------   Arduino
   -----------------                    -------
            MOSI   -------------------   SDO (D11)
            MISO   -------------------   SDI (D12)
            SCK    -------------------   SCK (D13)
            SS1    -------------------   SS1 (D7)
            SS2    -------------------   SS2 (D8)
            GND    -------------------   GND
            VDD    -------------------   VCC (5.0V)

//============================================================================================
*//*

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
//Publisher attribution
geometry_msgs::Twist  encdPub;
ros::Publisher chatter_encd("encoders_counts", &encdPub);
geometry_msgs::Twist  cmdPub;
ros::Publisher chatter_cmd("arduino_debug_feedback", &cmdPub);

//////////////////////////////////////////////////////////////////

// Serial Communication
const int baud_rate = 9600;

// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;

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

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc1,HIGH);
  digitalWrite(slaveSelectEnc2,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc2,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc2,HIGH);       // Terminate SPI conversation 
}

///////////////////////////////////////////////////////////////////

long readEncoder(int encoder) {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  }
  
  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

///////////////////////////////////////////////////////////////////

void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation   
  
  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
}

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
  // int test_pwm = 180;
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

  // Init and Clear Encoders
  initEncoders();    
  clearEncoderCount(); 
  
  // Init ROS
  nodeHandle.initNode();
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(cmdSubscriber) ;
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter_cmd);
  nodeHandle.advertise(chatter_encd);
  
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

  // Retrieve current encoder counters
  encoder1count = -readEncoder(1); 
  encoder2count = readEncoder(2);
    
  // Publish counts
  encdPub.linear.x = encoder1count;
  encdPub.linear.y = encoder2count;
  chatter_encd.publish( &encdPub );
  
  delay(1);
}
