#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Math.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

  /* Get a new sensor event */
sensors_event_t event;
  
float xThen = 0;  
float yThen = 0; 
float zThen = 0;
float prevHgt=0;
float xNow; //need to make floats into uint8_t
float yNow; //need to make floats into uint8_t
float zNow; //need to make floats into uint8_t
float dX; //need to make floats into uint8_t
float dY; //need to make floats into uint8_t
float dZ; //need to make floats into uint8_t
float thet;
float phi;
float vect;
float aclHgt;
int MSRMNTINT = 10; // interval that measurements are taken at in ms


void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}

void setup(void)
{
  Serial.begin(57600);
  Serial.println(F("Adafruit 9DOF Tester")); Serial.println("");
  
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

void getAccel()
{
  //get and store accel vals in global variables
  accel.getEvent(&event);
  xNow = event.acceleration.x;
  yNow = event.acceleration.y;
  zNow = event.acceleration.z;
  dX = xThen-xNow;
  dY = yThen-yNow; 
  dZ = zThen-zNow;
  vect = sqrt(pow(xNow,2)+pow(yNow,2)+pow(zNow,2)); //range 9.5-10.8 
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
/*  if(fabs(phi) > .2 && fabs(thet) < .5) //linits twist to one axis at a time
    thet = 0;
  else if (fabs(phi)< .5 && fabs(thet) > .2)
    phi = 0;
  else if (fabs(thet) > .5)
    phi = 0;
  else if (fabs(phi) > .5)
    thet = 0; */
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

void height()//calculates up/down acceleration based on the angle fo the accelerometer
{
  if(phi != 0 && fabs(dZ) >.5 && fabs(dX)>1) 
    aclHgt = dX/sin(phi) + dZ/cos(phi);
  else if (thet != 0 && fabs(dZ) >.5 && fabs(dY)>1)
    aclHgt = dY/sin(thet) + dZ/cos(thet);
  else if (fabs(dZ) >.5 )
    aclHgt = dZ;
  else
    aclHgt = 0;
    
  float timeSqrd = pow(MSRMNTINT,2)/pow(1000,2);
 // Serial.print("okay: ");
// Serial.println(timeSqrd);
  prevHgt += 1000*aclHgt*timeSqrd; //hgt in centimeters hopefully
}

void loop(void)
{

   getAccel();
  /* Display the results (acceleration is measured in m/s^2) */
   getAngles();
   checkValues();
 //  height();

   
  Serial.print(F("DELTA ACCEL "));
  Serial.print("X:"); Serial.print(dX,4); Serial.print("\t");
  Serial.print("Y:"); Serial.print(dY,4); Serial.print("\t");
  Serial.print("Z:"); Serial.print(dZ,4); Serial.print("\t");
  Serial.print(F("ANGLES "));
  Serial.print("Phi:"); Serial.print(phi,4); Serial.print("\t");
  Serial.print("Theta:"); Serial.print(thet,4); Serial.print("\t");
//  Serial.print(F("ACCEL "));
//  Serial.print("X:"); Serial.print(event.acceleration.x,4); Serial.print("\t");
//  Serial.print("Y:"); Serial.print(event.acceleration.y,4); Serial.print("\t");
//  Serial.print("Z:"); Serial.print(event.acceleration.z,4); Serial.print("\t");
//  Serial.print("Tot:"); Serial.print(vect,4); Serial.print("\t");
//  Serial.print("HGT: "); Serial.print(height()); Serial.print("  ");  
//  Serial.print("Prev HGT:"); Serial.print(prevHgt,4); Serial.print("\t"); 
//  Serial.print("ACEL HGT:"); Serial.print(aclHgt,4); Serial.print("\t"); 
//Serial.println();

  
  /* Display the results (magnetic vector values are in micro-Tesla (uT))*/
  mag.getEvent(&event);
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x,4); Serial.print("\t");
  Serial.print("Y: "); Serial.print(event.magnetic.y,4); Serial.print("\t");
  Serial.print("Z: "); Serial.print(event.magnetic.z,4); Serial.print("\t");Serial.println(); 



  /* Display the results   
  gyro.getEvent(&event);
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s "); */

 // Serial.println(F(""));
  delay(MSRMNTINT);
  
}

