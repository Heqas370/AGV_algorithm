#include <Arduino_PortentaBreakout.h> // Library to use Portenta Breakout
#include <Wire.h>
#include <VL53L1X.h> // Library for Pololu VL53L1X sensor
#include <Ethernet.h> //Ethernet Library
#include <EthernetUdp.h> //UDP Ethernet Library

VL53L1X pololu0;
VL53L1X pololu1;
VL53L1X pololu2;
VL53L1X sensor3;

IPAddress ip(169,254,72,130); // Assign the IP Adress
byte mac[] ={ 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; // Assign mac address
unsigned int localPort = 5000; // Assign a port to talk over
char packetBuffer[10]; // Dimensian a char array to hold our data packet
char buffor[100]; // Buffor to save a package from sensors
String datReq; // String for our data
int packetSize; // Size of the packet
int i,j = 0;
EthernetUDP Udp; // Create a UDP Object


class TFmini // Class allows to operate TF mini plus sensor
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
  void init() //set the baudrate of UART
    {
      serial->begin(115200);
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

// initialize the UART's for TF mini plus
TFmini tfmini0(&Breakout.UART0);
TFmini tfmini1(&Breakout.UART1);
TFmini tfmini2(&Breakout.UART2);
TFmini tfmini3(&Breakout.UART3);

void setup() 
{
Serial.begin(9600); // initialize the Serial Port Monitor

Ethernet.begin( mac, ip); //Inialize the Ethernet
Udp.begin(localPort); //Initialize Udp


}

void loop() 
{
   packetSize =Udp.parsePacket(); //Reads the packet size

   if(packetSize>0) //if packetSize is >0, that means someone has sent a request
   {
      Udp.read(packetBuffer, 10); //Read the data request
      String datReq(packetBuffer); //Convert char array packetBuffer into a string called datReq

      if (datReq =="TFmini")//Do the following if Temperature is requested 
      { 
         tfmini0.read();
         sprintf(buffor,"%d: ",j);
         j++;
         for(i = 0; i< 5; i++)
         {
            sprintf(buffor + strlen(buffor) ,",%d",tfmini0.get_distance_cm());
         }
         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Initialize packet send
         Udp.print(buffor); //Send the temperature data
         Udp.endPacket(); //End the packet
      }
   }
  memset(packetBuffer, 0, 10); //clear out the packetBuffer array
}
