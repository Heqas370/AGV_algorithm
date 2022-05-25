/****************************************************************************************************************************
   AGV_controller.ino
   For Project Base Learning "Automatic charging of AGV platform using 
renewable energy resources
   Written by Adam Herman

****************************************************************************************************************************/

#define ROSSERIAL_ARDUINO_TCP
#define TIMER0_INTERVAL_MS        1000
#define TIMER_INTERVAL_0_1S       100L

/********************************************************
                     INCLUDE LIBRARIES
********************************************************/

#include <ros.h>
#include <SPI.h>
#include <Ethernet.h>
#include<VL53L1X.h>
#include <EthernetUdp.h> //UDP Ethernet Library
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <Arduino_PortentaBreakout.h>
#include <Wire.h> // Library for using I2C bus
#include "Portenta_H7_TimerInterrupt.h"
#include "Portenta_H7_ISR_Timer.h"

/*****************************************************
                         PINS
*****************************************************/

breakoutPin IN1 = PWM2; breakoutPin IN2 = PWM3; breakoutPin IN3 = PWM4; 
breakoutPin IN4 = PWM5;
breakoutPin enLA = GPIO_3; breakoutPin enLB = GPIO_4; breakoutPin enRA = 
GPIO_5; breakoutPin enRB = GPIO_6;
breakoutPin enA = PWM0; breakoutPin enB = PWM1;
breakoutPin xshut0 = GPIO_0; breakoutPin xshut1 = GPIO_1; breakoutPin 
xshut2 = GPIO_2; breakoutPin xshut3 = PWM7;

/*********************************************************************************************
                                VARIABLES FOR FUNCTIONS
*********************************************************************************************/

VL53L1X pololu0; VL53L1X pololu1; VL53L1X pololu2; VL53L1X pololu3;

//ROS parameters for encoders
std_msgs::UInt16 encoder_Left;
std_msgs::UInt16 encoder_Right;
int encoder_left = 0;
int encoder_right = 0;

//Parameters for controling motors
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;
double w_r=0, w_l=0;
double wheel_rad = 0.0325, wheel_sep = 0.295;

//Timers for interrupt transfer data to ROS
Portenta_H7_Timer ITimer0(TIM12);
Portenta_H7_ISR_Timer ISR_Timer;
/**********************************************************************************************
                                           CLASSES
**********************************************************************************************/

class TFmini // Class allows to operate TFMini plus sensor
{
   struct TFmini_data
     {
       uint8_t distanceL; // Byte2
       uint8_t distanceH; // Byte3
       int distance; // sum of Byte2 and Byte3

       uint8_t strengthL; // Byte4
       uint8_t strengthH; // Byte5
       int strength; // sum of Byte4 and Byte5

       uint8_t tempL; // Byte6
       uint8_t tempH; // Byte7
       int temp; // sum of Byte6 and Byte7
     };

   private:
     HardwareSerial *serial;
     struct TFmini_data data;
     uint8_t frame[9] = {0}; // Data frame for TF mini plus

   public:
   TFmini(HardwareSerial *serial_port)
   {
     serial = serial_port;
   }
   void init() // set the baudrate of UART
     {
       serial->begin(115200); // must be that baud rate, because TFMini 
don't operate others
     }

   void read()// reading a frame from TF mini plus
   {
     uint8_t checksum = 0; // Byte8
     uint8_t read_byte = 0;

     while (true)
     {
       checksum = 0;

       do
       {
         if (serial->available())
         read_byte = serial->read();
       }
       while (read_byte != 0x59); // checking if first byte = 0x59
       frame[0] = read_byte;
       checksum += read_byte;

       while (!serial->available()) {}
       read_byte = serial->read();
       if (read_byte == 0x59) // checking if second bytr = 0x59
       {
         frame[1] = read_byte;
         checksum += read_byte;
         break;
       }
     }

     for (uint8_t i = 2; i < 8; ++i) // reading rest of the frame
     {
       while (!serial->available()) {}
       read_byte = serial->read();
       frame[i] = read_byte;
       checksum += read_byte;
     }

     while (!serial->available()) {}
     read_byte = serial->read();
     if (checksum == read_byte) // checksum has to be eqaul to read_byte
     {
       data.distanceL = frame[2];
       data.distanceH = frame[3];
       data.distance  = (frame[3] << 8) | frame[2];

       data.strengthL = frame[4];
       data.strengthH = frame[5];
       data.strength  = (frame[5] << 8) | frame[4];

       data.tempL = frame[6];
       data.tempH = frame[7];
       data.temp = (frame[7] << 8) | frame[6];
     }
   }

   struct TFmini_data get_data()
   {
     return data;
   }

   int get_distance_cm() // read the distance from TF mini plus
   {
     return data.distance;
   }
};

/***********************************************************************************************
                                            FUNCTIONS
************************************************************************************************/

//Functions responsible for controling Motors and Encoders
void Motors_init()
{
      Breakout.pinMode(IN1, OUTPUT);
      Breakout.pinMode(IN2, OUTPUT);
      Breakout.pinMode(enA, OUTPUT);
      Breakout.pinMode(enLA, INPUT);
      Breakout.pinMode(enLB, INPUT);

      Breakout.pinMode(enRA, INPUT);
      Breakout.pinMode(enRB, INPUT);
      Breakout.pinMode(IN4, OUTPUT);
      Breakout.pinMode(IN3, OUTPUT);
      Breakout.pinMode(enB, OUTPUT);

      Breakout.digitalWrite(IN1, LOW);
      Breakout.digitalWrite(IN3, LOW);
      Breakout.digitalWrite(IN2, LOW);
      Breakout.digitalWrite(IN4, LOW);
}

void MotorL(int Pulse_Width1)
{
   if (Pulse_Width1 > 0)
   {
     Breakout.analogWrite(enA, Pulse_Width1);
     Breakout.digitalWrite(IN1, HIGH);
     Breakout.digitalWrite(IN3, LOW);
   }
   if (Pulse_Width1 < 0)
   {
      Pulse_Width1=abs(Pulse_Width1);
      Breakout.analogWrite(enA, Pulse_Width1);
      Breakout.digitalWrite(IN1, LOW);
      Breakout.digitalWrite(IN3, HIGH);
   }
   if (Pulse_Width1 == 0)
   {
      Breakout.analogWrite(enA, Pulse_Width1);
      Breakout.digitalWrite(IN1, LOW);
      Breakout.digitalWrite(IN3, LOW);
   }
}

void MotorR(int Pulse_Width2)
{
   if (Pulse_Width2 > 0)
   {
      Breakout.analogWrite(enB, Pulse_Width2);
      Breakout,digitalWrite(IN2, LOW);
      Breakout.digitalWrite(IN4, HIGH);
   }
   if (Pulse_Width2 < 0)
   {
     Pulse_Width2=abs(Pulse_Width2);
     Breakout.analogWrite(enB, Pulse_Width2);
     Breakout.digitalWrite(IN2, HIGH);
     Breakout.digitalWrite(IN4, LOW);
   }
   if (Pulse_Width2 == 0)
   {
     Breakout.analogWrite(enB, Pulse_Width2);
     Breakout.digitalWrite(IN2, LOW);
     Breakout.digitalWrite(IN4, LOW);
   }
}

void EncoderLeft()
{
   if(digitalRead(enLB) == HIGH)
   {
     encoder_left++;
   }
   else
   {
     encoder_left--;
   }
}

void EncoderRight()
{
   if(digitalRead(enRB) == HIGH)
   {
     encoder_right++;
   }
   else
   {
     encoder_right--;
   }
}

void Pub_Encoder()
{
   encoder_Left.data = encoder_left;
   encoder_Right.data = encoder_right;
   encoder_left = 0;
   encoder_right = 0;
}

//Functions for controlling ROS

void messageCb( const geometry_msgs::Twist& msg)
{
   speed_ang = msg.angular.z;
   speed_lin = msg.linear.x;
   w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
   w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

void TimerHandler()
{
   ISR_Timer.run();
}

//Functions for controlling Pololu VL53L1X
void Pololu_init()
{
   Breakout.pinMode(xshut0,OUTPUT); //pololu0 XSHUT
   Breakout.pinMode(xshut1,OUTPUT); //pololu1 XSHUT
   Breakout.pinMode(xshut2,OUTPUT); //pololu2 XSHUT
   Breakout.pinMode(xshut3,OUTPUT); //pololu3 XSHUT

   //GPIO pinouts must be on LOW value, otherwise we will damaged the 
VL53L1X
   Breakout.digitalWrite(xshut0, LOW);
   Breakout.digitalWrite(xshut1, LOW);
   Breakout.digitalWrite(xshut2, LOW);
   Breakout.digitalWrite(xshut3, LOW);

   /* pololu0 */
   // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
   pinMode(xshut0, INPUT);
   delay(150);
   pololu0.init();
   delay(100);
   pololu0.setAddress((uint8_t)0x2a); //each pololu has to have its 
unique address for proper working I2C

   /* pololu1 */
   // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
   pinMode(xshut1, INPUT);
   delay(150);
   pololu1.init();
   delay(100);
   pololu1.setAddress((uint8_t)0x2b); //each pololu has to have its 
unique address for proper working I2C

   /* pololu2 */
   // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
   pinMode(xshut2, INPUT);
   delay(150);
   pololu2.init();
   delay(100);
   pololu2.setAddress((uint8_t)0x2c); //each pololu has to have its 
unique address for proper working I2C

   /* pololu3 */
   // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
   pinMode(xshut3, INPUT);
   delay(150);
   pololu3.init();
   delay(100);
   pololu3.setAddress((uint8_t)0x2d); //each pololu has to have its 
unique address for proper working I2C

   /* pololu0*/
   pololu0.setDistanceMode(VL53L1X::Long);
   pololu0.setMeasurementTimingBudget(33000);
   pololu0.startContinuous(33);

   /* pololu1 */
   pololu1.setDistanceMode(VL53L1X::Long);
   pololu1.setMeasurementTimingBudget(33000);
   pololu1.startContinuous(33);

   /* pololu2 */
   pololu2.setDistanceMode(VL53L1X::Long);
   pololu2.setMeasurementTimingBudget(33000);
   pololu2.startContinuous(33);

   /* pololu3 */
   pololu3.setDistanceMode(VL53L1X::Long);
   pololu3.setMeasurementTimingBudget(33000);
   pololu3.startContinuous(33);
}

/******************************************************
                    VARIABLES
******************************************************/

//Declare TCP/IP parameters
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(172,17,10,99);
IPAddress server(172,17,10,3);
const uint16_t serverPort = 11411;
EthernetUDP Udp; // Create a UDP Object

//ROS publishers
ros::Publisher Encoder_Left("Encoder_Left", &encoder_Left);
ros::Publisher Encoder_Right("Encoder_Right", &encoder_Right);

//ROS subscribers
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

//ROS Node Handle
ros::NodeHandle nh;

char packetBuffer[10]; // Dimensian a char array to hold our data packet
char pololu[100]; // Buffor to save a package from sensors
char tfmini[100];
String datReq; // String for our data
int packetSize; // Size of the packet
int dist0 = 0, dist1 = 0, dist2 = 0, dist3 = 0;

// Initialize the UART's for TF mini plus
TFmini tfmini0(&Breakout.UART0);
TFmini tfmini1(&Breakout.UART1);
TFmini tfmini2(&Breakout.UART2);
TFmini tfmini3(&Breakout.UART3);

void setup()
{
   Ethernet.begin(mac, ip);
   nh.getHardware()->setConnection(server, serverPort);

   attachInterrupt(digitalPinToInterrupt(enLA), EncoderLeft, RISING);
   attachInterrupt(digitalPinToInterrupt(enRA), EncoderRight, RISING);
   ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, 
TimerHandler);
   ISR_Timer.setInterval(TIMER_INTERVAL_0_1S,  Pub_Encoder);

   nh.initNode();
   nh.subscribe(sub);
   nh.advertise(Encoder_Left);
   nh.advertise(Encoder_Right);

   Pololu_init();
   Motors_init();

   tfmini0.init();
   tfmini1.init();
   tfmini2.init();
   tfmini3.init();
}

void loop()
{
   int i;
   packetSize =Udp.parsePacket(); //Reads the packet size
   if(packetSize>0) //if packetSize is >0, that means someone has sent a 
request
   {
      Udp.read(packetBuffer, 10); //Read the data request
      String datReq(packetBuffer); //Convert char array packetBuffer 
into a string called datReq

      if (datReq == "Pololu") // Read data from VL53L1X
      {
         sprintf(pololu + strlen(pololu) , ",%d", pololu0.read());
         sprintf(pololu + strlen(pololu) , ",%d", pololu1.read());
         sprintf(pololu + strlen(pololu) , ",%d", pololu2.read());
         sprintf(pololu + strlen(pololu) , ",%d", pololu3.read());

         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Initialize 
packet send
         Udp.print(pololu); //Send the temperature data
         Udp.endPacket(); //End the packet
      }
      if (datReq == "TFMini") // Read data from TFMini plus
      {
         //initialize the read function for TFMini
         tfmini0.read();
         tfmini1.read();
         tfmini2.read();
         tfmini3.read();
         //Get the length distance
         dist0 = tfmini0.get_distance_cm();
         dist1 = tfmini1.get_distance_cm();
         dist2 = tfmini2.get_distance_cm();
         dist3 = tfmini3.get_distance_cm();

         for(i = 0; i< 2 ; i++)
         {
            sprintf(tfmini + strlen(tfmini) ,",%d", dist0);
            sprintf(tfmini + strlen(tfmini) ,",%d", dist1);
            sprintf(tfmini + strlen(tfmini) ,",%d", dist2);
            sprintf(tfmini + strlen(tfmini) ,",%d", dist3);
         }

         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Initialize 
packet send
         Udp.print(tfmini); //Send the temperature data
         Udp.endPacket(); //End the packet
      }
   }

   memset(packetBuffer, 0, 10); //clear out the packetBuffer array
   if(nh.connected())
   {
     MotorL(w_l*10);
     MotorR(w_r*10);

     Encoder_Left.publish(&encoder_Left);
     Encoder_Right.publish(&encoder_Right);
   }

   nh.spinOnce();
   delay(1);
}
