//=========================HEADER=============================================================
/*
 * Dual LS7366 Quadrature ROS Node Counter
 * 
 * ROS NODE WRITTEN BY: William Therrien, SherbyRobotics
   DATE: Octobre 12, 2018
   
   Inspired by:
   AUTHOR: Jason Traud
   DATE: June 22, 2013
   License: CCAv3.0 Attribution-ShareAlike (http://creativecommons.org/licenses/by-sa/3.0/)

   
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
*/

// Inclde the standard Arduino SPI and ROS Libraries, please ensure the SPI pins are
// connected properly for your Arduino version and the you downloaded the ros_lib
#include <SPI.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// ROS
ros::NodeHandle  nodeHandle;
//Publisher attribution
geometry_msgs::Twist  twistPub;
ros::Publisher chatter("encoders_counts", &twistPub);

// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;

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

void setup(){
  
  // Init Communication
  Serial.begin(9600);

  initEncoders();    
  clearEncoderCount(); 
  
  // Init ROS
  nodeHandle.initNode();
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  
  //
  delay(1000) ;

}

////////////////////////////////////////////////////////////////////
void loop(){
  nodeHandle.spinOnce();
  // Retrieve current encoder counters
  encoder1count = -readEncoder(1); 
  encoder2count = readEncoder(2);
    
  // Publish counts
  twistPub.linear.x = encoder1count;
  twistPub.linear.y = encoder2count;
  chatter.publish( &twistPub );
  delay(1);
}
