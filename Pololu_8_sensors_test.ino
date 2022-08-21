#include <Wire.h>
#include <VL53L1X.h>
#include <Arduino_PortentaBreakout.h>

VL53L1X pololu0;
VL53L1X pololu1;
VL53L1X pololu2;
VL53L1X pololu3;
VL53L1X pololu4;
VL53L1X pololu5;
VL53L1X pololu6;
VL53L1X pololu7;


breakoutPin xshut0 = PWM6; breakoutPin xshut1 = GPIO_1; breakoutPin xshut2 = GPIO_2; breakoutPin xshut3 = GPIO_0;

breakoutPin xshut4 = ANALOG_A4; breakoutPin xshut5 = PWM7; breakoutPin xshut6 = PWM8; breakoutPin xshut7 = PWM9;


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  delay(5000);

  Breakout.pinMode(xshut0, OUTPUT); //pololu0 XSHUT
  Breakout.pinMode(xshut1, OUTPUT); //pololu1 XSHUT
  Breakout.pinMode(xshut2, OUTPUT); //pololu2 XSHUT
  Breakout.pinMode(xshut3, OUTPUT); //pololu3 XSHUT
  Breakout.pinMode(xshut4, OUTPUT); //pololu4 XSHUT
  Breakout.pinMode(xshut5, OUTPUT); //pololu5 XSHUT
  Breakout.pinMode(xshut6, OUTPUT); //pololu6 XSHUT
  Breakout.pinMode(xshut7, OUTPUT); //pololu7 XSHUT


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
  pinMode(xshut0, INPUT);
  pololu0.init();
  pololu0.setAddress((uint8_t)0x2a); //each pololu has to have its unique address for proper working I2C

  /* pololu1 */
  pinMode(xshut1, INPUT);
  pololu1.init();
  pololu1.setAddress((uint8_t)0x2b); //each pololu has to have its unique address for proper working I2C

  /* pololu2 */
  pinMode(xshut2, INPUT);
  pololu2.init();
  pololu2.setAddress((uint8_t)0x2c); //each pololu has to have its unique address for proper working I2C

  /* pololu3 */
  pinMode(xshut3, INPUT);
  pololu3.init();
  pololu3.setAddress((uint8_t)0x2d); //each pololu has to have its unique address for proper working I2C


  /* pololu4 */
  pinMode(xshut4, INPUT);
  pololu4.init();
  pololu4.setAddress((uint8_t)0x2e); //each pololu has to have its unique address for proper working I2C
  /* pololu5 */
  pinMode(xshut5, INPUT);
  pololu5.init();
  pololu5.setAddress((uint8_t)0x2f); //each pololu has to have its unique address for proper working I2C

  /* pololu6 */
  pinMode(xshut6, INPUT);
  pololu6.init();
  pololu6.setAddress((uint8_t)0x20); //each pololu has to have its unique address for proper working I2C

  /* pololu7 */
  pinMode(xshut7, INPUT);
  pololu7.init();
  pololu7.setAddress((uint8_t)0x21); //each pololu has to have its unique address for proper working I2C


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




}

void loop()
{
  Serial.print("Pololu0:");
  Serial.print(pololu0.read());
  Serial.print("   ");
  Serial.print("Pololu1:");
  Serial.print(pololu1.read());
  Serial.print("   ");
  Serial.print("Pololu2:");
  Serial.print(pololu2.read());
  Serial.print("   ");
  Serial.print("Pololu3:");
  Serial.print(pololu3.read());
  Serial.print("   ");
  Serial.print("Pololu4:");
  Serial.print(pololu4.read());
  Serial.print("   ");
  Serial.print("Pololu5:");
  Serial.print(pololu5.read());
  Serial.print("   ");
  Serial.print("Pololu6:");
  Serial.print(pololu6.read());
  Serial.print("   ");
  Serial.print("Pololu7:");
  Serial.print(pololu7.read());
  Serial.print("   ");
  Serial.println();
