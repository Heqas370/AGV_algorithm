//Must be define if we we want to you rosserial socket.launch
#define ROSSERIAL_ARDUINO_TCP

#include <SPI.h>
#include <ros.h>
#include <Ethernet.h>
#include <geometry_msgs/Twist.h> //Library for controlling motors via teleop
#include <Arduino_PortentaBreakout.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h> //Library for changing rostopic controlling motors
#include <std_msgs/Float64.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <EthernetUdp.h>
#include "MPU9250.h" // IMU library

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

// Creating objects for Pololu sesnors
VL53L1X pololu0;
VL53L1X pololu1;
VL53L1X pololu2;
VL53L1X pololu3;
VL53L1X pololu4;
VL53L1X pololu5;
VL53L1X pololu6;
VL53L1X pololu7;


MPU9250 accelgyro;
EthernetUDP Udp;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
uint16_t period = 25;
uint32_t last_time = 0;
const uint16_t serverPort = 11411;
const uint16_t Port = 5000;

float Axyz[3];
float Gxyz[3];
float previousMillis = 0 ;
float currentMillis = 0;
float rpm_right = 0;
float rpm_left = 0;
float w_r = 0, w_l = 0;
double wheel_rad = 0.0325, wheel_sep = 0.295;
double speed_ang = 0, speed_lin = 0;

int packetSize;
int interval = 50;
int encoder_left = 0;
int encoder_right = 0;
int lowSpeed = 200;
int highSpeed = 50;

String datReq; // String for our data
char packetBuffer[10]; // Dimensian a char array to hold our data packet
char pololu[200]; // Buffor to save a package from sensors
char encoder[50];


byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(172, 17, 10, 173);
IPAddress server(172, 17, 10, 3);

ros::NodeHandle nh;

// Enum for mode switching
enum control_mode
{
  CMD_VEL,
  INDPENDENT_CTRL
};

control_mode ctrl_mode = CMD_VEL;

void control2motors( const geometry_msgs::Twist& msg)
{
  if (ctrl_mode == CMD_VEL)
  {
    speed_ang = msg.angular.z;
    speed_lin = msg.linear.x;
    w_r = calculateSpeed(speed_ang, speed_lin, "right");
    w_l = calculateSpeed(speed_ang, speed_lin, "left");
  }
}

void controlRight(const std_msgs::Float64& pwm) 
{
  if (ctrl_mode == INDPENDENT_CTRL) 
  {

    w_r = pwm.data;
  }
}

void controlLeft(const std_msgs::Float64& pwm) 
{
  if (ctrl_mode == INDPENDENT_CTRL) 
  {
    w_l = pwm.data;
  }
}

float calculateSpeed(float speed_ang, float speed_lin, String side) 
{
  float output_speed = speed_lin / wheel_rad;
  if (side == "right") return output_speed + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  if (side == "left") return output_speed - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}

void change_topic(const std_msgs::String& cmd_msg)
{
  if (cmd_msg.data[0] == 'q')
  {
    ctrl_mode = CMD_VEL;
  }
  if (cmd_msg.data[0] == 'a')
  {
    ctrl_mode = INDPENDENT_CTRL;
  }
}

sensor_msgs::Imu data;
ros::Publisher imu_data("IMU", &data);

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &control2motors );
ros::Subscriber<std_msgs::String> sub1("change_topic", &change_topic);

ros::Subscriber<std_msgs::Float64> sub3("control_right", &controlRight);
ros::Subscriber<std_msgs::Float64> sub4("control_left", &controlLeft);

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Ethernet.begin(mac, ip);
  Udp.begin(Port);

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
  nh.subscribe(sub1);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.advertise(imu_data);

}

void loop()
{

  int i, j;
  getAccel_Data();
  getGyro_Data();

  if (ctrl_mode == CMD_VEL) {
    MotorL(w_l * 10);
    MotorR(w_r * 10);
  }
  else if (ctrl_mode == INDPENDENT_CTRL) {
    MotorL(w_l);
    MotorR(w_r);
  }

  currentMillis = millis();

  convert_RPM();



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
      sprintf(pololu + strlen(pololu) , ",%d", pololu3.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu4.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu5.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu6.read());
      sprintf(pololu + strlen(pololu) , ",%d", pololu7.read());

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Initialize packet send
      Udp.print(pololu); //Send the data
      Udp.endPacket(); //End the packet
    }

    if (datReq == "Encoder") // Read data from encoders
    {

      sprintf(encoder, "%f", rpm_left);
      sprintf(encoder + strlen(encoder), ",%f", rpm_right);

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Initialize packet send
      Udp.print(encoder); //Send the data
      Udp.endPacket(); //End the packet
    }


  }

  memset(packetBuffer, 0, 10);

  memset (encoder, 0 , sizeof(encoder));
  memset(pololu, 0, sizeof(pololu));


  data.linear_acceleration.x = Axyz[0];
  data.linear_acceleration.y = Axyz[1];
  data.linear_acceleration.z = Axyz[2];


  data.angular_velocity.x = Gxyz[0];
  data.angular_velocity.y = Gxyz[1];
  data.angular_velocity.z = Gxyz[2];



  data.orientation.x = 0;
  data.orientation.y = 0;
  data.orientation.z = 0;

  imu_data.publish(&data);

  delay(20);

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

    digitalWrite(IN2, LOW);

  }

  if (Pulse_Width1 < 0) {

    Pulse_Width1 = abs(Pulse_Width1);

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, HIGH);

  }

  if (Pulse_Width1 == 0) {

    analogWrite(ENA, Pulse_Width1);

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, LOW);

  }

}


void MotorR(int Pulse_Width2) {


  if (Pulse_Width2 > 0) {

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN3, LOW);

    digitalWrite(IN4, HIGH);

  }

  if (Pulse_Width2 < 0) {

    Pulse_Width2 = abs(Pulse_Width2);

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN3, HIGH);

    digitalWrite(IN4, LOW);

  }

  if (Pulse_Width2 == 0) {

    analogWrite(ENB, Pulse_Width2);

    digitalWrite(IN3, LOW);

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

void convert_RPM()
{
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    rpm_right = (float) (encoder_right * 20 * 6.28) / 6533;
    rpm_left = (float)(encoder_left * 20 * 6.28) / 6533;

    Serial.println(rpm_right);
    Serial.println(rpm_left);

    encoder_right = 0;
    encoder_left = 0;
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
