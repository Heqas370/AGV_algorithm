#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <SPI.h>
#include <Ethernet.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <Arduino_PortentaBreakout.h>


breakoutPin ECA1 = PWM2;
breakoutPin ECB1 = PWM3;
breakoutPin IN1 = GPIO_0;
breakoutPin IN2 = GPIO_1;
breakoutPin ENA = PWM0;

breakoutPin ECA2 = PWM4;
breakoutPin ECB2 = PWM5;
breakoutPin IN3 = GPIO_2;
breakoutPin IN4 = GPIO_3;
breakoutPin ENB = PWM1;

std_msgs::UInt16 encoder_Left;
std_msgs::UInt16 encoder_Right;

int encoder_left = 0;
int encoder_right = 0;

double w_r=0, w_l=0;
double wheel_rad = 0.0325, wheel_sep = 0.295;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(172,17,10,99);

// Set the rosserial socket server IP address
IPAddress server(172,17,10,3);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Publisher Encoder_Left("Encoder_Left", &encoder_Left);

ros::Publisher Encoder_Right("Encoder_Right", &encoder_Right);


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

void setup() {
  // put your setup code here, to run once:
 Ethernet.begin(mac, ip);


  Motors_init();
    attachInterrupt(digitalPinToInterrupt(ECA1), EncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ECA2), EncoderRight, RISING);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(Encoder_Left);
  nh.advertise(Encoder_Right);



}

void loop() 
{

if(nh.connected())

{
encoder_Left.data = encoder_left;
encoder_Right.data = encoder_right;


   Encoder_Left.publish( &encoder_Left);
   Encoder_Right.publish( &encoder_Right);
 
 MotorL(w_l*10);

 MotorR(w_r*10);

}

 nh.spinOnce();

  delay(1);
}

void Motors_init(){

     Breakout.pinMode(IN1, OUTPUT);
     Breakout.pinMode(IN2, OUTPUT);
     Breakout.pinMode(ENA, OUTPUT);
     Breakout.pinMode(ECA1, INPUT);
     Breakout.pinMode(ECB1, INPUT);

     Breakout.pinMode(ECA2, INPUT);
     Breakout.pinMode(ECB2, INPUT);
     Breakout.pinMode(IN4, OUTPUT);
     Breakout.pinMode(IN3, OUTPUT);
     Breakout.pinMode(ENB, OUTPUT);


     Breakout.digitalWrite(IN1, LOW);
     Breakout.digitalWrite(IN3, LOW);
     Breakout.digitalWrite(IN2, LOW);
     Breakout.digitalWrite(IN4, LOW);

}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){

     analogWrite(ENA, Pulse_Width1);

     digitalWrite(IN1, HIGH);

     digitalWrite(IN3, LOW);

 }

 if (Pulse_Width1 < 0){

     Pulse_Width1=abs(Pulse_Width1);

     analogWrite(ENA, Pulse_Width1);

     digitalWrite(IN1, LOW);

     digitalWrite(IN3, HIGH);

 }

 if (Pulse_Width1 == 0){

     analogWrite(ENA, Pulse_Width1);

     digitalWrite(IN1, LOW);

     digitalWrite(IN3, LOW);

 }

}


void MotorR(int Pulse_Width2){


 if (Pulse_Width2 > 0){

     analogWrite(ENB, Pulse_Width2);

     digitalWrite(IN2, LOW);

     digitalWrite(IN4, HIGH);

 }

 if (Pulse_Width2 < 0){

     Pulse_Width2=abs(Pulse_Width2);

     analogWrite(ENB, Pulse_Width2);

     digitalWrite(IN2, HIGH);

     digitalWrite(IN4, LOW);

 }

 if (Pulse_Width2 == 0){

     analogWrite(ENB, Pulse_Width2);

     digitalWrite(IN2, LOW);

     digitalWrite(IN4, LOW);

 }

}

void EncoderLeft()
{
if(digitalRead(ECB1) == HIGH)
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
if(digitalRead(ECB1) == HIGH)
{
  encoder_right++;
  }
else
{
  encoder_right--;
  }
  }
