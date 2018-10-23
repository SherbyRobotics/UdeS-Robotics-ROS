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
const int motA = 6;  // Motor R
const int motB = 5;  // Motor L
const int dirA = 4;  // Direction motor R
const int dirB = 3;  // Direction motor L

// Hardware min-zero-max range for the steering servo and the Cytron drive
// arduino pwm-servo lib unit = 0-180 deg angle range corresponding to 544-2400 microseconds pulse-witdh
const int pwm_min_ser = 30  ;
const int pwm_zer_ser = 90  ;
const int pwm_max_ser = 150 ;

const int pwm_min_cmd = 0 ;
const int pwm_max_cmd = 255 ;

// From_max param init. for the different control loops
const float batteryV = 8.4;   //Battery voltage in Volts (for CC0)
const int   vel_max  = 15;    //Max velocity in m/s      (for CC1)

// Conversion
const double  rad2pwm = 360/(2*3.1416);

///////////////////////////////////////////////////////////////////
//                        ENCODERS                               //
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

// Convertion function : Command Servo --> PWM
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

// Direction function (Low-->Backward or High-->Forward)
int lowHigh(double cmd) {
  int forward;
  
  if (cmd >= 0) {
    digitalWrite(dirA, HIGH);
    digitalWrite(dirB, HIGH);
    int forward = 1;
    return forward;
  }
  
  if (cmd < 0) {
    digitalWrite(dirA, LOW);
    digitalWrite(dirB, LOW);
    int forward = 0;
    return forward; 
  }
   
}///////////////////////////////////////////////////////////////////

// Convertion function: Command --> PWM Motors
double cmd2cyt(double cmd, float from_min, float from_max, int to_min, int to_max) {
  float pwm_f;
  int pwm;
  
  if (cmd >= 0) {
    float pwm_f = cmd*to_max/from_max;
    int pwm = (int) pwm_f;
    return pwm;
  }
  
  if (cmd < 0) {
    float pwm_f = -cmd*to_max/from_max;
    int pwm = (int) pwm_f;
    return pwm; 
  }
   
}


///////////////////////////////////////////////////////////////////

long pubWrite(int cytron_pwmA, int cytron_pwmB, int ser_pwm, int forwardA, int forwardB, int CtrlChoice){
  
  // Write PWM for the steering servo and both motors
  steeringServo.write(ser_pwm) ;
  analogWrite(motA,cytron_pwmA);
  analogWrite(motB,cytron_pwmB);

  // Debug feedback
  cmdPub.linear.x = cytron_pwmA;     //PWM Motor A
  cmdPub.linear.y = cytron_pwmB;     //PWM Motor B
  cmdPub.linear.z = CtrlChoice;      //CtrlChoice (given by the propulsion algo)
  cmdPub.angular.x = forwardA;       //Direction A
  cmdPub.angular.y = forwardB;       //Direction B
  cmdPub.angular.z = ser_pwm;        //PWM Servo
  chatter_cmd.publish( &cmdPub );
  
}
///////////////////////////////////////////////////////////////////

// Main Loop
void cmdCallback ( const geometry_msgs::Twist&  twistMsg )
{
  // Declaration
  int cytron_pwmA;
  int cytron_pwmB;
  
  // CtrlChoice definition
  int CtrlChoice   = twistMsg.linear.z;
  
  // Steering
  double ser_cmd   = twistMsg.angular.z; //Read servo cmd in rad
  int ser_pwm      = cmd2pwm( ser_cmd, rad2pwm, pwm_min_ser, pwm_zer_ser, pwm_max_ser) ;

  // Cytron Drive 
  double cytron_cmdA = twistMsg.linear.x; //Read motor A cmd 
  double cytron_cmdB = twistMsg.linear.y; //Read motor B cmd

  float from_min     = 0;

  // Call the right control loop
  // Openloop control in volts (CC0)
  if (CtrlChoice == 0){
    float from_max    = batteryV;
    int forwardA      = lowHigh(cytron_cmdA);
    int forwardB      = lowHigh(cytron_cmdB);
    int cytron_pwmA   = cmd2cyt(cytron_cmdA, from_min, from_max, pwm_min_cmd, pwm_max_cmd);
    int cytron_pwmB   = cmd2cyt(cytron_cmdB, from_min, from_max, pwm_min_cmd, pwm_max_cmd);
    pubWrite(cytron_pwmA,cytron_pwmB,ser_pwm,forwardA,forwardB,CtrlChoice);
  }
  // Openloop control in m/s (CC1)
  if (CtrlChoice == 1){
    float from_max    = vel_max;
    int forwardA      = lowHigh(cytron_cmdA);
    int forwardB      = lowHigh(cytron_cmdB);
    int cytron_pwmA   = cmd2cyt(cytron_cmdA, from_min, from_max, pwm_min_cmd, pwm_max_cmd);
    int cytron_pwmB   = cmd2cyt(cytron_cmdB, from_min, from_max, pwm_min_cmd, pwm_max_cmd);
    pubWrite(cytron_pwmA,cytron_pwmB,ser_pwm,forwardA,forwardB,CtrlChoice);
  }
  // Closedloop control for position in m (CC3)
  if (CtrlChoice == 3){
    float targA       = twistMsg.angular.x; //Read the targeted position of motor A
    float targB       = twistMsg.angular.y; //Read the targeted position of motor B
    int forwardA      = lowHigh(cytron_cmdA);
    int forwardB      = lowHigh(cytron_cmdB);
    int cytron_pwmA   = cmd2cyt(cytron_cmdA, from_min, targA, pwm_min_cmd, pwm_max_cmd/2); //pwm_max_cmd is divided by 2 to ensure the wheels won't slip at the beginning!**Might need to limit the acceleration**
    int cytron_pwmB   = cmd2cyt(cytron_cmdB, from_min, targB, pwm_min_cmd, pwm_max_cmd/2);
    pubWrite(cytron_pwmA,cytron_pwmB,ser_pwm,forwardA,forwardB,CtrlChoice);
  }
  // Closedloop control for velocity in m/s (CC4)
  if (CtrlChoice == 4){
    float targA       = twistMsg.angular.x; //Read the targeted velocity for motor A
    float targB       = twistMsg.angular.y; //Read the targeted velocity for motor B
    int forwardA      = lowHigh(cytron_cmdA);
    int forwardB      = lowHigh(cytron_cmdB);
    int cytron_pwmA   = cmd2cyt(cytron_cmdA, from_min, targA, pwm_min_cmd, pwm_max_cmd); //**Might need to limit the acceleration**
    int cytron_pwmB   = cmd2cyt(cytron_cmdB, from_min, targB, pwm_min_cmd, pwm_max_cmd);
    pubWrite(cytron_pwmA,cytron_pwmB,ser_pwm,forwardA,forwardB,CtrlChoice);
  }
}

// ROS suscriber
ros::Subscriber<geometry_msgs::Twist> cmdSubscriber("/cmd_prop", &cmdCallback) ;

///////////////////////////////////////////////////////////////////

void setup(){
   
  // Init PWM output Pins
  pinMode(motA, OUTPUT);
  pinMode(motB, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
   // Initialize Steering and Drive cmd to neutral
  digitalWrite(dirA, HIGH);
  digitalWrite(dirB, HIGH);
  // Init Cytron Drive
  int cytron_pwm = 0;
  analogWrite(motA,cytron_pwm);
  analogWrite(motB,cytron_pwm);
  steeringServo.write(pwm_zer_ser);
  
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
