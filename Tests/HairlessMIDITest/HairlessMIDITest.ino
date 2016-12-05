/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */
 const int ONBOARDLED = 13;

 
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(ONBOARDLED, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // play notes from F#-0 (0x1E) to F#-5 (0x5A):
  for (int note = 0x2A; note < 0x5A; note ++) {
    //Note on channel 1 (0x90), some note value (note), middle velocity (0x45):
    noteOn(1, note, 0x45);
    noteOn(2, note-9, 0x45);
    digitalWrite(ONBOARDLED,HIGH);
    delay(1000);
    //Note on channel 1 (0x90), some note value (note), silent velocity (0x00):
    noteOn(1, note, 0x00);    
    noteOn(2, note-9, 0x00);
    digitalWrite(ONBOARDLED,LOW);
    delay(10);
  }
}

//  plays a MIDI note.  Doesn't check to see that
//  cmd is greater than 127, or that data values are  less than 127:
void noteOn(int channel, int pitch, int velocity) {
  Serial.write(0x90+channel);
  Serial.write(pitch);
  Serial.write(velocity);
}
