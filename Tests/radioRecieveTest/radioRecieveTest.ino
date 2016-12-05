/* RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
// Get libraries at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses></http:>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************/

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     2  //the same on all nodes that talk to each other
#define NODEID        1  

//Match frequency to the hardware version of the radio on your Feather
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY      RF69_915MHZ
#define ENCRYPTKEY     "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW    true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   57600
/*For Arduino Uno*/
#define RFM69_CS      10 //try using direct ICSP port on UNO 
#define RFM69_IRQ     2 // need female to male jumpers
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9
#define LED           3 

/* for Feather 32u4 */
//#define RFM69_CS      8
//#define RFM69_IRQ     7
//#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
//#define RFM69_RST     4
//#define LED           13  // onboard blinky



int16_t packetnum = 0;  // packet counter, we increment per xmission
int lastNote;

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

void setup() {
  
  Serial.begin(SERIAL_BAUD);

//  Serial.println("RFM69HCW Receiver");
    
  pinMode(LED, OUTPUT);
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  Blink(LED, 50, 3); 
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  
  // Initialize radio
  if(radio.initialize(FREQUENCY,NODEID,NETWORKID))
//  Serial.println("Okay");
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);

//  Serial.print("\nListening at ");
//  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
//  Serial.println(" MHz");
}

void readData(uint8_t data[], float floatsDat[])
{
  floatsDat[0] = (data[0]-50)/3;
  floatsDat[1] = (data[1]-50)/3;  
  floatsDat[2] = (float)data[2];
  floatsDat[2] = floatsDat[2]*25/80;
  floatsDat[3] = (data[3]/80)-1.57;
}

//  plays a MIDI note.  Doesn't check to see that
//  cmd is greater than 127, or that data values are  less than 127:
void noteOn(int channel, int pitch, int velocity) {
  Serial.write(0x90+channel);
  Serial.write(pitch);
  Serial.write(velocity);
}
void noteOff(int channel, int pitch) {
  Serial.write(0x90+channel);
  Serial.write(pitch);
}

void sendMIDI(float floatsDat[])
{
  int note = (int)(floatsDat[2]+2);
  noteOn(0,note,60);
  if(note!=lastNote)
  {
    noteOff(0,lastNote);
    noteOn(0,note,60);
  }
  lastNote= note;
}


void loop() {
  //check if something was received (could be an interrupt from the radio)
  if (radio.receiveDone())
  {
    //print message received to serial
   // Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
    uint8_t data[4];
   for(int i = 0; i <4; i++)
   {
     uint8_t x;
     x = radio.DATA[i];
     data[i] = x;
   }
   /*for(int i = 0 ; i <4; i++)
   {
     Serial.print(data[i]);
     Serial.print(" ");
   }*/
   float floatsDat[4];
  readData(data,floatsDat);
  sendMIDI(floatsDat);
   //Serial.println();
    //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

     //check if sender wanted an ACK
      if (radio.ACKRequested())
      {
        radio.sendACK();
       // Serial.println(" - ACK sent");
      }
      //Blink(LED, 50, 1); //blink LED 3 times, 40ms between blinks
    }  
  

  radio.receiveDone(); //put radio in RX mode
  Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
//Blink(LED, 200, 1); 
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
