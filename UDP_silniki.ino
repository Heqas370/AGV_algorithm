#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <Ethernet.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <Arduino_PortentaBreakout.h>
#include <sensor_msgs/Imu.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <EthernetUdp.h>
#include "MPU9250.h"


// Creating objects for Pololu sesnors
VL53L1X pololu0;
VL53L1X pololu1;
VL53L1X pololu2;
VL53L1X pololu3;
VL53L1X pololu4;
VL53L1X pololu5;
VL53L1X pololu6;
VL53L1X pololu7;

geometry_msgs::Vector3 accVec;
geometry_msgs::Vector3 velVec;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float Axyz[3];
float Gxyz[3];


MPU9250 accelgyro;

breakoutPin ECA1 = GPIO_3;
breakoutPin ECB1 = GPIO_4;
breakoutPin IN1 = PWM2;
breakoutPin IN2 = PWM3;
breakoutPin ENA = PWM0;

breakoutPin ECA2 = GPIO_5;
breakoutPin ECB2 = GPIO_6;
breakoutPin IN3 = PWM4;
breakoutPin IN4 = PWM5;
breakoutPin ENB = PWM1;

breakoutPin xshut0 = PWM6; breakoutPin xshut1 = GPIO_1; breakoutPin xshut2 = GPIO_2; breakoutPin xshut3 = GPIO_0;

breakoutPin xshut4 = ANALOG_A4; breakoutPin xshut5 = PWM7; breakoutPin xshut6 = PWM8; breakoutPin xshut7 = PWM9;


std_msgs::UInt16 encoder_Left;
std_msgs::UInt16 encoder_Right;

int encoder_left = 0;
int encoder_right = 0;

String datReq; // String for our data
char packetBuffer[10]; // Dimensian a char array to hold our data packet
char pololu[200]; // Buffor to save a package from sensors
int packetSize;
char encoder[50];
EthernetUDP Udp;


double w_r = 0, w_l = 0;
double wheel_rad = 0.0325, wheel_sep = 0.295;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(172,17,10,173);
IPAddress server(172,17,10,3);

const uint16_t serverPort = 11411;

ros::NodeHandle nh;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang = 0, speed_lin = 0;

void messageCb( const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}

sensor_msgs::Imu imu_msg;
ros::Publisher accPub("Acceleration", &accVec);
ros::Publisher velPub("Velocity", &velVec);
ros::Publisher Encoder_Left("Encoder_Left", &encoder_Left);
ros::Publisher Encoder_Right("Encoder_Right", &encoder_Right);


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Ethernet.begin(mac, ip);
  Udp.begin(serverPort);

  accelgyro.initialize();


  Breakout.pinMode(xshut0, OUTPUT); //pololu0 XSHUT
  Breakout.pinMode(xshut1, OUTPUT); //pololu1 XSHUT
  Breakout.pinMode(xshut2, OUTPUT); //pololu2 XSHUT
  Breakout.pinMode(xshut3, OUTPUT); //pololu3 XSHUT
  Breakout.pinMode(xshut4, OUTPUT); //pololu0 XSHUT
  Breakout.pinMode(xshut5, OUTPUT); //pololu1 XSHUT
  Breakout.pinMode(xshut6, OUTPUT); //pololu2 XSHUT
  Breakout.pinMode(xshut7, OUTPUT); //pololu3 XSHUT


  //GPIO pinouts must be on LOW value, otherwise we will damaged the VL53L1X
  Breakout.digitalWrite(xshut0, LOW);
  Breakout.digitalWrite(xshut1, LOW);
  Breakout.digitalWrite(xshut2, LOW);
  Breakout.digitalWrite(xshut3, LOW);
  Breakout.digitalWrite(xshut4, LOW);
  Breakout.digitalWrite(xshut5, LOW);
  Breakout.digitalWrite(xshut6, LOW);
  Breakout.digitalWrite(xshut7, LOW);


  /* pololu0 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut0, INPUT);
  pololu0.init();
  pololu0.setAddress((uint8_t)0x2a); //each pololu has to have its unique address for proper working I2C

  /* pololu1 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut1, INPUT);
  pololu1.init();
  pololu1.setAddress((uint8_t)0x2b); //each pololu has to have its unique address for proper working I2C

  /* pololu2 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut2, INPUT);
  pololu2.init();
  pololu2.setAddress((uint8_t)0x2c); //each pololu has to have its unique address for proper working I2C

  /* pololu3 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut3, INPUT);
  pololu3.init();
  pololu3.setAddress((uint8_t)0x2d); //each pololu has to have its unique address for proper working I2C


  /* pololu4 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut4, INPUT);
  pololu4.init();
  pololu4.setAddress((uint8_t)0x1a); //each pololu has to have its unique address for proper working I2C

  /* pololu5 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut5, INPUT);
  pololu5.init();
  pololu5.setAddress((uint8_t)0x1b); //each pololu has to have its unique address for proper working I2C

  /* pololu6 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut6, INPUT);
  pololu6.init();
  pololu6.setAddress((uint8_t)0x1c); //each pololu has to have its unique address for proper working I2C

  /* pololu7 */
  // allow the sensor board pololu0 to pull its XSHUT pin up to 2.8V
  pinMode(xshut7, INPUT);
  pololu7.init();
  pololu7.setAddress((uint8_t)0x1d); //each pololu has to have its unique address for proper working I2C

  pololu0.setDistanceMode(VL53L1X::Long);
  pololu0.setMeasurementTimingBudget(33000);
  pololu0.startContinuous(33);

  pololu1.setDistanceMode(VL53L1X::Long);
  pololu1.setMeasurementTimingBudget(33000);
  pololu1.startContinuous(33);


  pololu2.setDistanceMode(VL53L1X::Long);
  pololu2.setMeasurementTimingBudget(33000);
  pololu2.startContinuous(33);


  pololu3.setDistanceMode(VL53L1X::Long);
  pololu3.setMeasurementTimingBudget(33000);
  pololu3.startContinuous(33);


  pololu4.setDistanceMode(VL53L1X::Long);
  pololu4.setMeasurementTimingBudget(33000);
  pololu4.startContinuous(33);


  pololu5.setDistanceMode(VL53L1X::Long);
  pololu5.setMeasurementTimingBudget(33000);
  pololu5.startContinuous(33);

  pololu6.setDistanceMode(VL53L1X::Long);
  pololu6.setMeasurementTimingBudget(33000);
  pololu6.startContinuous(33);

  pololu7.setDistanceMode(VL53L1X::Long);
  pololu7.setMeasurementTimingBudget(33000);
  pololu7.startContinuous(33);


  attachInterrupt(digitalPinToInterrupt(ECA1), EncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ECA2), EncoderRight, RISING);

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  Motors_init();
  nh.subscribe(sub);
  nh.advertise(accPub);
  nh.advertise(velPub);

}

void loop()
{
  getAccel_Data();
  getGyro_Data();

  packetSize = Udp.parsePacket(); //Reads the packet size
  if (packetSize > 0) //if packetSize is >0, that means someone has sent a request
  {

    Udp.read(packetBuffer, 10); //Read the data request
    String datReq(packetBuffer); //Convert char array packetBuffer into a string called datReq

    if (datReq == "Pololu") // Read data from VL53L1X
    {
      sprintf(pololu , "%d", pololu0.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu1.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu2.read());
      sprintf(pololu + strlen(pololu) , ",%d", 0);
      sprintf(pololu + strlen(pololu) , ",%d", pololu4.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu5.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu6.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu7.read());

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Initialize packet send
      Udp.print(pololu); //Send the temperature data
      Udp.endPacket(); //End the packet
    }



  if (datReq == "Encoder") // Read data from VL53L1X
  {
    sprintf(encoder, "%d", encoder_left);
    sprintf(encoder + strlen(encoder), ",%d", encoder_right);

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Initialize packet send
    Udp.print(encoder); //Send the temperature data
    Udp.endPacket(); //End the packet
  }
}
  memset(packetBuffer, 0, 10);  
  memset(pololu, 0, sizeof(pololu));


  MotorL(w_l * 10);

  MotorR(w_r * 10);

  accVec.x = Axyz[0];
  accVec.y = Axyz[1];
  accVec.z = Axyz[2];

  velVec.x = Gxyz[0];
  velVec.y = Gxyz[1];
  velVec.z = Gxyz[2];

  accPub.publish( &accVec );
  velPub.publish( &velVec);

  nh.spinOnce();
  
}
void Motors_init() {

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

void MotorL(int Pulse_Width1) {
  if (Pulse_Width1 > 0) {

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, HIGH);

    digitalWrite(IN3, LOW);

  }

  if (Pulse_Width1 < 0) {

    Pulse_Width1 = abs(Pulse_Width1);

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, LOW);

    digitalWrite(IN3, HIGH);

  }

  if (Pulse_Width1 == 0) {

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, LOW);

    digitalWrite(IN3, LOW);

  }

}


void MotorR(int Pulse_Width2) {


  if (Pulse_Width2 > 0) {

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN2, LOW);

    digitalWrite(IN4, HIGH);

  }

  if (Pulse_Width2 < 0) {

    Pulse_Width2 = abs(Pulse_Width2);

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN2, HIGH);

    digitalWrite(IN4, LOW);

  }

  if (Pulse_Width2 == 0) {

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN2, LOW);

    digitalWrite(IN4, LOW);

  }

}

void EncoderLeft()
{
  if (digitalRead(ECB1) == HIGH)
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
  if (digitalRead(ECB2) == HIGH)
  {
    encoder_right++;
  }
  else
  {
    encoder_right--;
  }
}
void getAccel_Data() {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
}

void getGyro_Data() {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}
