
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     2  // The same on all nodes that talk to each other
#define NODEID        2    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

//Match frequency to the hardware version of the radio on your Feather
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   57600

//#define RFM69_CS      10
//#define RFM69_IRQ     2
//#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
//#define RFM69_RST     9
//#define LED           3  // onboard blinky

/* for Feather 32u4 */
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
#define RFM69_RST     4
#define LED           13  // onboard blinky

//********************************************************************************************************************************************************************************************************************************************************************
//Accelerometer setup

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
//values to potentially be read
//AccelerationValues
float xThen = 0, yThen = 0,zThen = 0,xNow,yNow,zNow,dX,dY,dZ;
//Angles
float phi,thet;
int ARRAYSIZE = 5;
int chanSel;
int finger1=0;
int finger2=0;
int finger3=0;
int finger4=0;

int16_t packetnum = 0;  // packet counter, we increment per transmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);

  gyro.getSensor(&sensor);
  
  mag.getSensor(&sensor);

  delay(5);
}

void setup() { // wait until serial console is open, remove if not tethered to computer
  Serial.begin(SERIAL_BAUD);
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  
  delay(500);
  
  // Initialize radio
  if(radio.initialize(FREQUENCY, NODEID, NETWORKID))
  Serial.println("we got true");
  
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

  radio.encrypt(ENCRYPTKEY);

  pinMode(LED, OUTPUT);

  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void getAccel(sensors_event_t event)
{
  //get and store accel vals in global variables
  accel.getEvent(&event);
  xNow = event.acceleration.x;
  yNow = event.acceleration.y;
  zNow = event.acceleration.z;
  dX = xThen-xNow;
  dY = yThen-yNow; 
  dZ = zThen-zNow;
 // vect = sqrt(pow(xNow,2)+pow(yNow,2)+pow(zNow,2)); //range 9.5-10.8 
  xThen = xNow;
  yThen = yNow;
  zThen = zNow;
}
void getAngles()//gets values for angles
{
  phi =  atan(-xNow/zNow); //rotating fingers up/down
  thet = atan(-yNow/zNow); //twisting wrist left/right
}
void checkValues()
{

  if (zNow > -11 && zNow <-9)
  {
    phi = 0;
    thet = 0;
  }
  if (fabs(phi) >.1)
  {
    dX = xNow - 9.8*sin(phi); //removes constant acceleration due to gravity in each of the components 
    dZ = zNow + 9.8*cos(phi);// have to aproximate acceleraition due to gravity
  }
  if (fabs(thet) > .1)
  {
    dY = yNow - 9.8*sin(thet);
    dZ = zNow + 9.8*cos(thet);
  } 
}

void prepareData(uint8_t buff[])
{
  if(dY<15&&dY>-15)
    buff[0] = (uint8_t)((dY+15)*8); // produces int ranges 0 to 240 with accel range of -15 to 15
  else if (dY<-15)
    buff[0] = (uint8_t)(0);
  else if (dY>15)
    buff[0] = (uint8_t)(31*8);
  if(dZ<15&&dZ>-15)
    buff[1] = (uint8_t)((dZ+15)*8); // produces int ranges 0 to 240 with accel range of -15 to 15
  else if (dZ<-15)
    buff[1] = (uint8_t)(0);
  else if (dZ>15)
    buff[1] = (uint8_t)(30*8);
  float phiMod = (phi+1.57)*80;
  buff[2] = (uint8_t)phiMod;
  float thetMod = (thet+1.57)*80;
  buff[3] = (uint8_t)thetMod;
  buff[4] = (uint8_t)chanSel;
}

void getFingers()
{
  finger1 = analogRead(A1);
//  Serial.println(finger1);
  finger2 = analogRead(A2);
//  Serial.println(finger2);
  finger3 = analogRead(A3); 
  Serial.println(finger3);
  finger4 = analogRead(A4); 
  Serial.println(finger4);
  
  if (finger1>650 && finger2>700 && finger4>570)
    chanSel = 4;
  else if (finger1>650 && finger2>700)
    chanSel = 5;
  else if (finger2>700 && finger4>570)
    chanSel = 6;
  else if (finger1>650 && finger4 > 570)
    chanSel = 7;
  else if (finger1>600)
    chanSel = 1;
  else if (finger2>700)
    chanSel = 2;
  else if (finger4>570)
    chanSel = 3;
  else
    chanSel = 0;
    
   
}
void loop() {
  /* Get a new sensor event */
  sensors_event_t event;
   chanSel=1;
   getAccel(event);
  /* Display the results (acceleration is measured in m/s^2) */
   getAngles();
   getFingers();
   checkValues();
   
//  Serial.print("dY: ");Serial.print(dY); Serial.print(" dZ: ");Serial.print(dY);Serial.print(" phi: "); Serial.print(phi); Serial.print(" theta: ");Serial.print(thet);Serial.print(" Chan: ");Serial.println(chanSel);
   
  /* Display the results (acceleration is measured in m/s^2) */

  /* Display the results (gyrocope values in rad/s)
  gyro.getEvent(&event); */

  delay(50);// Wait  between transmits, could also 'sleep' here!
  //Blink(LED, 50, 1);
  uint8_t radiopacket[ARRAYSIZE];//delta and current values for each reading
  prepareData(radiopacket);
// Serial.print("Sending "); 
/* */ for(int i=0; i < ARRAYSIZE; i++)
  {
  Serial.print(radiopacket[i]);
  Serial.print(" ");
  }
 Serial.println();

  if (radio.sendWithRetry(RECEIVER, radiopacket, sizeof(radiopacket), 1, 100)) { //target node Id, message as string or byte array, message length, #retries, time for retry)
   Serial.println("OK");
    //Blink(LED, 50, 1); //blink LED 3 times, 50ms between blinks
  }

  radio.receiveDone(); //put radio in RX mode
 // Serial.println(F(""));
  
  Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
}




  
void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i = 0; i < loops; i++)
  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }  
}
